"""Local costmap rules on hand-made observations. No simulator needed."""
import json,unittest
from pathlib import Path
import numpy as np
from navigation.lane_map import inside
from perception.local_costmap import LocalCostmap,load_config,inside_polygon,UNKNOWN,FREE,OCCUPIED,INSCRIBED,LETHAL,NO_INFORMATION

ROOT=Path(__file__).resolve().parents[1]
MAP={'footprint':{'body_radius_m':.35,'step_sway_m':.08,'pose_error_m':.04,'margin_m':.1},
     'allowed_lanes':[{'polygon':[[-10,-10],[10,-10],[10,10],[-10,10]]}],'forbidden_zones':[{'polygon':[[1.5,-3],[3,-3],[3,-1],[1.5,-1]]}]}


def observation(floor=(),obstacles=(),classes=None):
    """Points on a 2 cm lattice inside the given world rectangles [(x0, y0, x1, y1)]."""
    points=[]; is_obstacle=[]; class_id=[]
    for group,flag in ((floor,False),(obstacles,True)):
        for index,(x0,y0,x1,y1) in enumerate(group):
            xs,ys=np.meshgrid(np.arange(x0+.01,x1,.02),np.arange(y0+.01,y1,.02))
            points.append(np.stack([xs.ravel(),ys.ravel(),np.full(xs.size,.5 if flag else 0.)],axis=1))
            is_obstacle.append(np.full(xs.size,flag)); class_id.append(np.full(xs.size,(classes or {}).get(index,-1) if flag else -1))
    world=np.concatenate(points) if points else np.zeros((0,3)); mask=np.concatenate(is_obstacle) if points else np.zeros(0,dtype=bool)
    ids=np.concatenate(class_id).astype(np.int16) if points else np.zeros(0,dtype=np.int16)
    return {'world':world,'obstacle':mask,'floor':~mask,'class_id':ids,'confidence':np.where(ids>=0,.9,0.).astype(np.float32)}


class LocalCostmapTests(unittest.TestCase):
    def setUp(self): self.map=LocalCostmap(load_config(),MAP)
    def state(self,snap,x,y): ix,iy=self.map.cell(snap,x,y); return snap['state'][ix,iy]
    def cost(self,snap,x,y): ix,iy=self.map.cell(snap,x,y); return snap['cost'][ix,iy]

    def test_size_and_default_unknown(self):
        snap=self.map.update(observation(),(0.,0.),0.)
        self.assertEqual(snap['state'].shape,(60,60)); self.assertEqual(snap['resolution_m'],.1)
        self.assertEqual(self.state(snap,-2.,0.),UNKNOWN); self.assertEqual(self.cost(snap,-2.,0.),NO_INFORMATION)
        self.assertFalse(snap['traversable'][self.map.cell(snap,-2.,0.)])   # unseen is never traversable
        self.assertEqual(self.state(snap,0.05,0.05),FREE)                     # the robot's own footprint

    def test_free_only_where_floor_was_seen(self):
        snap=self.map.update(observation(floor=[(1.,-1.,2.5,1.)]),(0.,0.),0.)
        self.assertEqual(self.state(snap,2.,0.),FREE); self.assertTrue(snap['traversable'][self.map.cell(snap,2.,0.)])
        self.assertEqual(self.state(snap,.7,0.),UNKNOWN)    # blind zone between the footprint and the first seen floor
        self.assertEqual(self.state(snap,2.8,0.),UNKNOWN)   # beyond what was seen

    def test_obstacle_inflation_radii(self):
        snap=self.map.update(observation(floor=[(.5,-2.,2.9,2.)],obstacles=[(2.,-.1,2.2,.1)]),(0.,0.),0.)
        self.assertEqual(self.state(snap,2.1,0.),OCCUPIED); self.assertEqual(self.cost(snap,2.1,0.),LETHAL)
        self.assertEqual(self.cost(snap,1.75,0.),INSCRIBED)                 # 0.25 m from the obstacle: robot centre would touch it
        self.assertTrue(0<self.cost(snap,1.55,0.)<INSCRIBED)                # 0.45 m: inside the 0.57 m inflation ring
        self.assertEqual(self.cost(snap,1.25,0.),0)                         # 0.75 m: clear
        self.assertFalse(snap['traversable'][self.map.cell(snap,1.55,0.)]); self.assertTrue(snap['traversable'][self.map.cell(snap,1.25,0.)])

    def test_person_gets_extra_clearance_and_label(self):
        plain=self.map.update(observation(floor=[(.5,-2.,2.9,2.)],obstacles=[(2.,-.1,2.2,.1)]),(0.,0.),0.)
        other=LocalCostmap(load_config(),MAP)
        human=other.update(observation(floor=[(.5,-2.,2.9,2.)],obstacles=[(2.,-.1,2.2,.1)],classes={0:0}),(0.,0.),0.)
        self.assertEqual(human['class_id'][other.cell(human,2.1,0.)],0)
        self.assertEqual(self.cost(plain,1.25,0.),0); ix,iy=other.cell(human,1.25,0.)
        self.assertGreater(human['cost'][ix,iy],0)   # 0.75 m from a person is still inside 0.57 + 0.30 m

    def test_factory_forbidden_area_is_never_free(self):
        # The camera "sees floor" inside a machine island and outside the building: both stay lethal.
        snap=self.map.update(observation(floor=[(1.,-2.9,2.9,1.)]),(0.,0.),0.)
        self.assertEqual(self.cost(snap,2.,-2.),LETHAL); self.assertFalse(snap['traversable'][self.map.cell(snap,2.,-2.)])
        self.assertEqual(self.cost(snap,1.3,-2.),INSCRIBED)   # 0.2 m outside the island edge
        far=LocalCostmap(load_config(),MAP).update(observation(floor=[(8.,-1.,12.,1.)]),(9.,0.),0.)
        self.assertTrue(far['static_lethal'][self.map.cell(far,10.5,0.)])
        self.assertFalse(far['traversable'][LocalCostmap.cell(self.map,far,10.5,0.)])

    def test_old_data_becomes_unknown_not_free(self):
        self.map.update(observation(floor=[(.5,-2.,2.9,2.)],obstacles=[(2.,-.1,2.2,.1)]),(0.,0.),0.)
        later=self.map.update(observation(),(0.,0.),9.)      # looking elsewhere, 9 s later: still remembered
        self.assertEqual(self.state(later,2.1,0.),OCCUPIED); self.assertEqual(self.state(later,1.,1.),FREE)
        stale=self.map.update(observation(),(0.,0.),10.5)    # past the 10 s memory
        self.assertEqual(self.state(stale,2.1,0.),UNKNOWN); self.assertEqual(self.state(stale,1.,1.),UNKNOWN)
        self.assertFalse(stale['traversable'][self.map.cell(stale,1.,1.)])

    def test_person_forgotten_sooner_than_static(self):
        self.map.update(observation(obstacles=[(2.,-.1,2.2,.1),(2.,1.,2.2,1.2)],classes={0:0}),(0.,0.),0.)
        later=self.map.update(observation(),(0.,0.),4.)
        self.assertEqual(self.state(later,2.1,0.),UNKNOWN); self.assertEqual(self.state(later,2.1,1.1),OCCUPIED)

    def test_seen_again_as_floor_clears_obstacle(self):
        self.map.update(observation(obstacles=[(2.,-.1,2.2,.1)]),(0.,0.),0.)
        cleared=self.map.update(observation(floor=[(1.5,-1.,2.9,1.)]),(0.,0.),.2)
        self.assertEqual(self.state(cleared,2.1,0.),FREE)

    def test_window_rolls_with_robot_and_keeps_world_cells(self):
        first=self.map.update(observation(obstacles=[(2.,-.1,2.2,.1)]),(0.,0.),0.)
        moved=self.map.update(observation(),(1.03,.42),1.)
        self.assertEqual(self.state(moved,2.1,0.),OCCUPIED)                 # same world place, different array index
        self.assertNotEqual(self.map.cell(first,2.1,0.),self.map.cell(moved,2.1,0.))
        self.assertAlmostEqual(moved['origin_world_m'][0]+3.,1.,delta=.1)   # window stays centred on the robot
        self.assertIsNone(self.map.cell(moved,-2.5,0.))                     # cells that left the window are dropped

    def test_polygon_rule_matches_navigation_map(self):
        config=json.loads((ROOT/'configs/navigation_map_v2.json').read_text())
        rng=np.random.default_rng(0); xs=rng.uniform(-11,9,400); ys=rng.uniform(-16,13,400)
        for polygon in [config['allowed_lanes'][0]['polygon']]+[z['polygon'] for z in config['forbidden_zones']]:
            mine=inside_polygon(xs,ys,polygon); reference=np.array([inside((x,y),polygon) for x,y in zip(xs,ys)])
            self.assertTrue((mine==reference).all())


if __name__=='__main__': unittest.main()
