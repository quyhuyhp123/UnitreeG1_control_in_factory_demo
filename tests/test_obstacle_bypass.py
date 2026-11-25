import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from navigation.lane_map import LaneMap, box_distance
from navigation.obstacle_bypass import HUMAN_EXTRA_M, plan
from navigation.route_graph import CorridorGrid


def lane_config(width=4.0):
    return {'resolution_m': .1,
            'footprint': {'body_radius_m': .35, 'step_sway_m': .08, 'pose_error_m': .04, 'margin_m': .1},
            'allowed_lanes': [{'polygon': [[-width/2, -1], [width/2, -1], [width/2, 12], [-width/2, 12]]}],
            'forbidden_zones': []}


def person(x, y):
    return {'id': 'person', 'kind': 'human', 'bounds': [[x-.325, y-.275], [x+.325, y+.275]]}


class ObstacleBypassTest(unittest.TestCase):
    def factory(self, config):
        return lambda obstacles: LaneMap(config, obstacles)

    def test_staggered_barrier_on_rejoin_ramp_both_sides(self):
        for side in (-1,1):
            with self.subTest(side=side):
                cfg=lane_config(width=6.)
                # Limit one side for the whole passage; add a short shelf
                # across the preferred return ramp on the open side.
                def box(name,x0,x1,y0,y1):
                    xs=sorted((side*x0,side*x1))
                    return {'id':name,'kind':'static','bounds':[[xs[0],y0],[xs[1],y1]]}
                obstacles=[person(0,5),box('closed side',-2.8,-.55,4,6),
                           box('shelf',.8,1.,7.0,7.2)]
                lane=LaneMap(cfg,obstacles)
                result=plan((0,0),(0,10),(0,0),obstacles,self.factory(cfg),lane.radius)
                self.assertIsNotNone(result)
                self.assertGreater(side*result['points'][0][0],0)
                for a,b in zip(result['path'],result['path'][1:]):
                    for i in range(51):
                        self.assertTrue(lane.safe(tuple(a[j]+(b[j]-a[j])*i/50 for j in (0,1))))

    def test_person_on_centerline_is_bypassed_with_human_clearance(self):
        config = lane_config(); radius = LaneMap(config).radius
        human = person(0, 5)
        result = plan((0, 0), (0, 10), (0, 0), [human], self.factory(config), radius)
        self.assertIsNotNone(result)
        pass_a, pass_b, rejoin = result['points']
        # Constant side offset beside the person, then back on the centreline.
        self.assertAlmostEqual(pass_a[0], pass_b[0])
        self.assertAlmostEqual(rejoin[0], 0.)
        self.assertGreater(box_distance(pass_a, human['bounds']), radius + HUMAN_EXTRA_M - 1e-6)
        lo, hi = human['bounds']
        self.assertLess(pass_a[1], lo[1]); self.assertGreater(pass_b[1], hi[1])
        lane = LaneMap(config)
        for a, b in zip(result['path'], result['path'][1:]):
            for i in range(21):
                p = (a[0]+(b[0]-a[0])*i/20, a[1]+(b[1]-a[1])*i/20)
                self.assertTrue(lane.safe(p))
                self.assertGreater(box_distance(p, human['bounds']), radius-1e-6)

    def test_corridor_too_narrow_returns_none(self):
        config = lane_config(width=1.6); radius = LaneMap(config).radius
        self.assertIsNone(plan((0, 0), (0, 10), (0, 0), [person(0, 5)], self.factory(config), radius))

    def test_obstacle_on_checkpoint_is_not_skipped(self):
        config = lane_config(); radius = LaneMap(config).radius
        self.assertIsNone(plan((0, 0), (0, 10), (0, 0), [person(0, 9.6)], self.factory(config), radius))

    def test_passed_or_clear_obstacles_are_ignored(self):
        config = lane_config(); radius = LaneMap(config).radius
        side = {'id': 'box', 'kind': 'static', 'bounds': [[1.5, 4], [1.9, 4.4]]}
        self.assertIsNone(plan((0, 0), (0, 10), (0, 0), [side], self.factory(config), radius))
        self.assertIsNone(plan((0, 0), (0, 10), (0, 8), [person(0, 5)], self.factory(config), radius))

    def test_corridor_accepts_bypass_polyline(self):
        lane = LaneMap(lane_config())

        class Base:
            map = lane
        corridor = CorridorGrid(Base(), (0, 0), (0, 10), .4)
        self.assertFalse(corridor.contains((1.2, 5)))
        corridor.detour_path = [(0, 3), (1.2, 4), (1.2, 6), (0, 7)]
        self.assertTrue(corridor.contains((1.2, 5)))
        self.assertTrue(corridor.in_detour((.6, 3.5)))


class FactoryV2LaneTest(unittest.TestCase):
    def test_widened_lanes_keep_route_nodes_safe_and_free_map2_person_2(self):
        config = json.loads((ROOT/'configs/navigation_map_v2.json').read_text())
        routes = json.loads((ROOT/'configs/patrol_routes.json').read_text())
        lane = LaneMap(config)
        for name, point in routes['nodes'].items():
            if name not in ('ENTRY', 'EXIT'):
                self.assertTrue(lane.safe(point), name)
        # Person 2 of Map 2 stands in the left corridor; the red zones now hug
        # the narrowed island machines, leaving a lane beside it.
        prop = next(o for o in json.loads((ROOT/'assets/factory/factory_layout_v2/maps/map_2.json').read_text())['obstacles']
                    if o['name'] == 'shared_left_person')
        x, y, _ = prop['position_m']
        result = plan(routes['nodes']['LEFT_CORRIDOR'], routes['nodes']['TOP_LEFT_TURN'], routes['nodes']['LEFT_CORRIDOR'],
                      [person(x, y)], lambda obstacles: LaneMap(config, obstacles), lane.radius)
        self.assertIsNotNone(result)
        self.assertTrue(all(math.isfinite(v) for p in result['points'] for v in p))


if __name__ == '__main__':
    unittest.main()
