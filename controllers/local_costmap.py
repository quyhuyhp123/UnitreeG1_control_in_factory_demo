"""Observation-only rolling world-aligned grid. Never sends navigation commands."""
import json,time
from pathlib import Path
import numpy as np
ROOT=Path(r'D:\unitree_g1')

class LocalCostmap:
    def __init__(self):
        self.resolution=.1;self.size=60;self.timeout=2.;self.history={};self.semantic={}
        self.config=json.loads((ROOT/'configs/navigation_map_v2.json').read_text())
        self.radius=sum(self.config['footprint'].values())
        self.window=None
        self.static_bounds=[]

    def update(self,depth,meta,localizer):
        import cv2
        start=time.perf_counter();now=meta['acquisition_unix_s']
        if hasattr(self,'last_time') and now<self.last_time:self.history.clear();self.semantic.clear()
        self.last_time=now
        self.semantic={k:v for k,v in self.semantic.items() if now-v.get('acquisition_unix_s',0)<=self.timeout}
        robot=np.array(meta['T_world_robot'])[:2,3]
        origin=np.floor(robot/.1).astype(int)-30;self.origin=origin*.1
        u,v=np.meshgrid(np.arange(0,depth.shape[1],16),np.arange(0,depth.shape[0],16))
        u=u.ravel();v=v.ravel()
        _,world,_,obstacle=localizer.points(depth,meta,u,v)
        z=depth[v,u];valid=np.isfinite(z)&(z>.15)&(z<10)
        # Floor returns can prove visible ray space; self returns cannot.
        for bounds in meta['robot_self_aabbs_world_m']:
            lo,hi=np.array(bounds);valid &= ~np.all((world>=lo-.025)&(world<=hi+.025),axis=1)
        camera=np.array(meta['T_world_optical'])[:2,3]
        occupied=set();free=set()
        for point,is_obstacle in zip(world[valid],obstacle[valid]):
            end=point[:2];length=np.linalg.norm(end-camera)
            # Only low obstacle endpoints count; high surfaces never clear space under them.
            if point[2]>2.:continue
            cells=np.floor((camera+np.linspace(0,1,max(2,int(length/.05)+1))[:,None]*(end-camera))/.1).astype(int)
            free.update(map(tuple,cells[:-1]))
            if is_obstacle:occupied.add(tuple(cells[-1]))
            else:free.add(tuple(cells[-1]))
        for cell in free-occupied:self.history[cell]=(now,0)
        for cell in occupied:self.history[cell]=(now,100)
        self.history={c:x for c,x in self.history.items() if now-x[0]<=self.timeout and np.all(np.abs(np.array(c)-origin-30)<40)}
        grid=np.full((60,60),-1,np.int16)
        for (x,y),(stamp,value) in self.history.items():
            ix,iy=x-origin[0],y-origin[1]
            if 0<=ix<60 and 0<=iy<60:grid[iy,ix]=value
        # Static map restrictions are authoritative and cannot be ray-cleared.
        blocked=np.zeros((60,60),bool)
        for y in range(60):
            for x in range(60):
                p=tuple(((origin+np.array([x+.5,y+.5]))*.1).tolist())
                allowed=any(cv2.pointPolygonTest(np.array(a['polygon'],np.float32),p,False)>=0 for a in self.config['allowed_lanes'])
                forbidden=any(cv2.pointPolygonTest(np.array(a['polygon'],np.float32),p,False)>=0 for a in self.config['forbidden_zones'])
                blocked[y,x]=not allowed or forbidden
        grid[blocked]=100
        ys,xs=np.mgrid[0:60,0:60];wx=(xs+origin[0]+.5)*.1;wy=(ys+origin[1]+.5)*.1
        for lo,hi in self.static_bounds:
            grid[(wx>=lo[0])&(wx<=hi[0])&(wy>=lo[1])&(wy<=hi[1])]=100
        r=int(np.ceil(self.radius/.1));yy,xx=np.mgrid[-r:r+1,-r:r+1]
        kernel=((xx*.1)**2+(yy*.1)**2<=self.radius**2).astype(np.uint8)
        inflated=cv2.dilate((grid==100).astype(np.uint8),kernel)>0
        self.grid=grid;self.cost=grid.copy();self.cost[inflated&(grid!=100)]=80
        self.metadata={'timestamp':meta['acquisition_utc'],'origin_world_xy_m':self.origin.tolist(),
            'frame':'world axes, rolling centered on pelvis','resolution_m':.1,'size_m':[6,6],
            'inflation_radius_m':self.radius,'timeout_s':self.timeout,'latency_ms':(time.perf_counter()-start)*1000}
        image=np.full((60,60,3),90,np.uint8);image[grid==0]=230;image[self.cost==80]=[255,170,0];image[grid==100]=[255,40,40]
        self.image=cv2.resize(np.flipud(image),(600,600),interpolation=cv2.INTER_NEAREST)
        if self.window is not None:
            preview=cv2.resize(self.image,(240,240),interpolation=cv2.INTER_NEAREST)
            rgba=np.concatenate([preview,np.full((240,240,1),255,np.uint8)],axis=2)
            self.provider.set_bytes_data(rgba.ravel().tolist(),[240,240])

    def add_semantics(self,objects):
        from datetime import datetime
        for obj in objects:
            obj=dict(obj,acquisition_unix_s=datetime.fromisoformat(obj['timestamp']).timestamp())
            self.semantic[tuple(np.floor(np.array(obj['position_world_m'][:2])/.1).astype(int))]=obj
        self.semantic={k:v for k,v in self.semantic.items() if self.last_time-v.get('acquisition_unix_s',self.last_time)<=self.timeout}

    def debug(self):
        import omni.ui as ui
        self.window=ui.Window('Local Costmap | observation only',width=610,height=640)
        self.provider=ui.ByteImageProvider()
        with self.window.frame:
            with ui.VStack():
                ui.Label('Gray unknown | white observed free | red occupied | amber inflated',height=24)
                ui.ImageWithProvider(self.provider)

    def save(self,directory):
        import cv2
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        np.save(directory/'occupancy.npy',self.grid);np.save(directory/'cost.npy',self.cost)
        cv2.imwrite(str(directory/'costmap.png'),cv2.cvtColor(self.image,cv2.COLOR_RGB2BGR))
        (directory/'metadata.json').write_text(json.dumps(self.metadata,indent=2))
        (directory/'semantics.json').write_text(json.dumps(list(self.semantic.values()),indent=2))
