from pathlib import Path
import sys,json,time
sys.path.insert(0,r'D:\unitree_g1')
import numpy as np
from controllers.local_costmap import LocalCostmap
from controllers.obstacle_localization import DepthLocalizer
root=Path(r'D:\unitree_g1');folder=root/'logs/keyboard_runs/20261116_181241/rgbd/standing_1.5'
meta=json.loads((folder/'metadata.json').read_text());depth=np.load(folder/'depth_m.npy')
costmap=LocalCostmap();costmap.update(depth,meta,DepthLocalizer())
assert costmap.grid.shape==(60,60) and (costmap.grid==-1).any() and (costmap.grid==100).any()
assert (costmap.cost==80).any()
costmap.static_bounds=[[[-8.5,-14.,0],[-8.3,-13.8,2]]]
meta['acquisition_unix_s']+=3
costmap.update(np.full_like(depth,np.nan),meta,DepthLocalizer())
assert not (costmap.grid==0).any(),'Expired free evidence persisted'
assert (costmap.grid==100).any(),'Static restrictions expired'
costmap.save(root/'logs/costmap_logic')
print('PASS unknown/occupied/inflation/timeout/static restrictions; latency_ms',costmap.metadata['latency_ms'])
fixture=LocalCostmap();fixture.config['allowed_lanes']=[{'polygon':[[-100,-100],[100,-100],[100,100],[-100,100]]}];fixture.config['forbidden_zones']=[]
T=np.eye(4);T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T[2,3]=1
m={'intrinsics_K':[[100,0,0],[0,100,0],[0,0,1]],'T_world_optical':T.tolist(),'T_world_robot':np.eye(4).tolist(),
   'robot_self_aabbs_world_m':[],'acquisition_unix_s':100.,'acquisition_utc':'fixture'}
fixture.update(np.array([[2.]],np.float32),m,DepthLocalizer())
assert fixture.grid[30,50]==100,'Known endpoint not occupied'
assert fixture.grid[30,40]==0,'Visible ray not free'
assert fixture.grid[30,10]==-1,'Blind rear marked free'
m['acquisition_unix_s']=103
fixture.update(np.array([[np.nan]],np.float32),m,DepthLocalizer())
assert np.all(fixture.grid==-1),'Evidence did not expire to unknown'
print('PASS known 2m endpoint/free ray/rear unknown/full evidence expiry')
