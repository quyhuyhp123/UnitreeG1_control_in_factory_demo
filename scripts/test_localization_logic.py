"""Deterministic depth/transform checks; synthetic arrays, not simulation evidence."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(r'D:\unitree_g1')))
import numpy as np
from controllers.obstacle_localization import DepthLocalizer

def main():
    localizer=DepthLocalizer()
    T=np.eye(4);T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T[2,3]=1
    meta={'intrinsics_K':[[100,0,40],[0,100,30],[0,0,1]],'T_world_optical':T.tolist(),
          'T_world_robot':np.eye(4).tolist(),'frame_counter':1,'acquisition_utc':'test',
          'robot_self_aabbs_world_m':[]}
    depth=np.full((60,80),5.,dtype=np.float32);depth[15:45,20:60]=2
    depth[20,25]=np.nan;depth[21,26]=np.inf;depth[22,27]=0;depth[23,28]=-1
    result={'frame_counter':1,'acquisition_utc':'test','detections':[{'class_name':'fixture','confidence':.9,'bbox_xyxy':[10,5,70,55]}]}
    objects=localizer.semantic(depth,meta,result)
    assert objects and abs(objects[0]['optical_z_m']-2)<.01
    assert abs(objects[0]['position_robot_m'][0]-2)<.01
    masked=dict(meta,robot_self_aabbs_world_m=[[[0,-100,-100],[100,100,100]]])
    assert not localizer.geometry(depth,masked)
    floorT=np.array(T);floorT[2,3]=-10
    assert not localizer.geometry(depth,dict(meta,T_world_optical=floorT.tolist()))
    assert not localizer.geometry(np.full_like(depth,np.nan),meta)
    bad=dict(result,acquisition_utc='other')
    try:localizer.semantic(depth,meta,bad)
    except ValueError:pass
    else:raise AssertionError('Timestamp mismatch accepted')
    print('PASS: robust foreground, invalid depth, optical-to-robot transform, self mask, floor mask, timestamp rejection')

if __name__=='__main__':main()
