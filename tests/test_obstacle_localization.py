"""Synthetic-scene checks for RGB-D obstacle localisation. No simulator needed."""
import math,unittest
import numpy as np
from perception.obstacle_localizer import ObstacleLocalizer,load_config

W,H,FX=640,360,381.36
K={'fx':FX,'fy':FX,'cx':W/2,'cy':H/2}
PITCH=math.radians(15); CAMERA=np.array([.1,0.,1.25])


def camera_pose(yaw=0.,base=(0.,0.)):
    forward=np.array([math.cos(PITCH),0,-math.sin(PITCH)]); right=np.array([0,-1.,0]); down=np.cross(forward,right)
    T=np.eye(4); T[:3,:3]=np.stack([right,down,forward],axis=1); T[:3,3]=CAMERA
    c,s=math.cos(yaw),math.sin(yaw); R=np.array([[c,-s,0,base[0]],[s,c,0,base[1]],[0,0,1,0],[0,0,0,1]])
    return R@T


def render(T,boxes):
    """Ray-cast a floor at Z=0 and axis-aligned boxes [(lo, hi)] into a Z-depth image."""
    v,u=np.mgrid[0:H,0:W]
    rays=np.stack([(u+.5-K['cx'])/FX,(v+.5-K['cy'])/FX,np.ones((H,W))],axis=-1)@T[:3,:3].T
    origin=T[:3,3]; depth=np.full((H,W),np.inf)
    with np.errstate(divide='ignore',invalid='ignore'):
        t=-origin[2]/rays[...,2]; depth=np.where((rays[...,2]<0)&(t>0),t,depth)
        for lo,hi in boxes:
            a=(np.array(lo)-origin)/rays; b=(np.array(hi)-origin)/rays
            near=np.minimum(a,b).max(-1); far=np.maximum(a,b).min(-1)
            depth=np.where((near<=far)&(near>0)&(near<depth),near,depth)
    return depth.astype(np.float32)


def frame(boxes,yaw=0.,base=(0.,0.)):
    T=camera_pose(yaw,base); depth=render(T,boxes)
    return {'depth_m':depth,'valid':np.isfinite(depth)&(depth>.1)&(depth<20),'intrinsics':K,'T_world_optical':T,
            'base_position_m':[base[0],base[1],.79],'base_orientation_wxyz':[math.cos(yaw/2),0,0,math.sin(yaw/2)],
            'sim_time':1.,'index':1,'control_step':50,'pose_lag_ticks':2}


class ObstacleLocalizationTests(unittest.TestCase):
    def setUp(self): self.localizer=ObstacleLocalizer(load_config())

    def test_empty_floor_has_no_obstacles(self):
        result=self.localizer.process(frame([]),[])
        self.assertEqual(result['obstacles'],[]); self.assertGreater(result['points']['floor'],10000)

    def test_unknown_box_found_at_true_distance(self):
        result=self.localizer.process(frame([((3.,-.25,0.),(3.5,.25,.5))]),[])
        self.assertEqual(len(result['obstacles']),1); box=result['obstacles'][0]
        self.assertEqual((box['class_name'],box['source']),('unknown','depth'))
        self.assertAlmostEqual(box['distance_m'],3.,delta=.06)
        self.assertAlmostEqual(box['height_range_m'][1],.5,delta=.05)

    def test_yolo_box_uses_foreground_not_background(self):
        scene=[((3.,-.25,0.),(3.5,.25,1.)),((6.,-4.,0.),(6.2,4.,3.))]  # object in front of a wall
        f=frame(scene); T=np.linalg.inv(f['T_world_optical'])
        corners=[T@np.array([3.,y,z,1.]) for y in (-.6,.6) for z in (0.,1.4)]  # box drawn generously around the object
        us=[FX*p[0]/p[2]+K['cx'] for p in corners]; vs=[FX*p[1]/p[2]+K['cy'] for p in corners]
        detection={'class_id':0,'class_name':'person','confidence':.9,'bbox_xyxy':[min(us),max(0,min(vs)),max(us),min(H,max(vs))]}
        result=self.localizer.process(f,[detection])
        person=[o for o in result['obstacles'] if o['class_name']=='person']
        self.assertEqual(len(person),1); self.assertEqual(person[0]['kind'],'human')
        self.assertAlmostEqual(person[0]['distance_m'],3.,delta=.08); self.assertLess(person[0]['position_robot_m'][0],3.6)
        self.assertTrue(detection['localized'])

    def test_robot_frame_follows_rotation(self):
        yaw=math.radians(30); base=(2.,-1.)
        # A box 3 m straight ahead of the rotated robot, expressed in world coordinates (axis-aligned, so slightly larger).
        centre=np.array(base)+3.25*np.array([math.cos(yaw),math.sin(yaw)])
        result=self.localizer.process(frame([((centre[0]-.25,centre[1]-.25,0.),(centre[0]+.25,centre[1]+.25,.6))],yaw,base),[])
        box=result['obstacles'][0]
        self.assertAlmostEqual(box['position_robot_m'][1],0.,delta=.15); self.assertAlmostEqual(box['distance_m'],2.95,delta=.15)
        self.assertAlmostEqual(box['position_world_m'][0],centre[0],delta=.3); self.assertAlmostEqual(box['position_world_m'][1],centre[1],delta=.3)

    def test_points_on_the_robot_are_ignored(self):
        result=self.localizer.process(frame([((.25,-.1,.9),(.35,.1,1.1))]),[])  # something 0.3 m from the base axis
        self.assertEqual(result['obstacles'],[]); self.assertGreater(result['points']['robot_body'],0)

    def test_person_box_range_requires_every_depth_pixel_to_be_valid(self):
        f=frame([((9.,-3.,0.),(9.2,3.,3.))])
        box=[315,80,325,90]
        item={'class_id':0,'class_name':'person','confidence':.9,'bbox_xyxy':box}
        self.localizer.process(f,[item])
        self.assertFalse(item['localized'])
        self.assertGreater(item['nearest_box_depth_xy_m'],8.)
        f['valid'][82,317]=False
        self.localizer.process(f,[item])
        self.assertIsNone(item['nearest_box_depth_xy_m'])


if __name__=='__main__': unittest.main()
