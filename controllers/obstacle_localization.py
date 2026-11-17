"""Metric visible-surface localization only; no scene labels or motion commands."""
from pathlib import Path
import json
import numpy as np
ROOT=Path(r'D:\unitree_g1')

class DepthLocalizer:
    def __init__(self,config=None):
        self.cfg=config or json.loads((ROOT/'configs/obstacle_localization.json').read_text())
        if not (0<self.cfg['min_depth_m']<self.cfg['max_depth_m'] and self.cfg['stride']>=1 and
                self.cfg['voxel_m']>0 and self.cfg['depth_band_m']>0 and self.cfg['minimum_points']>=3):
            raise ValueError('Invalid obstacle localization sampling/depth configuration')

    def points(self,depth,meta,u,v):
        z=depth[v,u]
        valid=np.isfinite(z)&(z>=self.cfg['min_depth_m'])&(z<=self.cfg['max_depth_m'])
        z=np.where(valid,z,0.)
        K=np.array(meta['intrinsics_K'])
        optical=np.column_stack(((u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z))
        T=np.array(meta['T_world_optical'])
        world=optical@T[:3,:3].T+T[:3,3]
        R=np.linalg.inv(np.array(meta['T_world_robot']))
        robot=world@R[:3,:3].T+R[:3,3]
        valid &= world[:,2]>self.cfg['floor_z_m']+self.cfg['floor_clearance_m']
        for bounds in meta.get('robot_self_aabbs_world_m',[]):
            lo,hi=np.array(bounds)
            pad=self.cfg['self_padding_m']
            valid &= ~np.all((world>=lo-pad)&(world<=hi+pad),axis=1)
        return optical,world,robot,valid

    def record(self,optical,world,robot,meta,**extra):
        p=np.median(optical,axis=0);r=np.median(robot,axis=0);w=np.median(world,axis=0)
        return dict(class_name='unknown',confidence=None,position_camera_optical_m=p.tolist(),
            position_robot_m=r.tolist(),position_world_m=w.tolist(),
            distance_camera_m=float(np.linalg.norm(p)),optical_z_m=float(p[2]),
            nearest_surface_distance_m=float(np.min(np.linalg.norm(optical,axis=1))),
            frame_counter=meta['frame_counter'],timestamp=meta['acquisition_utc'],
            simulation_time_s=meta.get('acquisition_sim_time_s'),point_count=len(optical),
            position_kind='median visible surface; not object center',**extra)

    def semantic(self,depth,meta,result):
        # Exact packet identity, not nearest timestamp or current robot transform.
        if result['frame_counter']!=meta['frame_counter'] or result['acquisition_utc']!=meta['acquisition_utc']:
            raise ValueError('RGB/depth/YOLO frame or timestamp mismatch')
        objects=[]
        h,w=depth.shape
        for det in result['detections']:
            x1,y1,x2,y2=det['bbox_xyxy'];dx=(x2-x1)*.2;dy=(y2-y1)*.2
            us=np.arange(max(0,int(x1+dx)),min(w,int(x2-dx)),2)
            vs=np.arange(max(0,int(y1+dy)),min(h,int(y2-dy)),2)
            u,v=np.meshgrid(us,vs);u=u.ravel();v=v.ravel()
            p,world,robot,valid=self.points(depth,meta,u,v)
            p,world,robot=p[valid],world[valid],robot[valid]
            if len(p)<self.cfg['minimum_points']: continue
            # Nearest sufficiently supported depth band, followed by MAD trim.
            bins=np.floor(p[:,2]/self.cfg['depth_band_m']).astype(int)
            ids,counts=np.unique(bins,return_counts=True)
            supported=ids[counts>=max(self.cfg['minimum_points'],int(counts.max()*.12))]
            if not len(supported): continue
            keep=np.abs(p[:,2]-(supported[0]+.5)*self.cfg['depth_band_m'])<=self.cfg['depth_band_m']
            med=np.median(p[keep,2]);mad=np.median(np.abs(p[keep,2]-med))
            keep &= np.abs(p[:,2]-med)<=max(.025,3*1.4826*mad)
            if keep.sum()<self.cfg['minimum_points']: continue
            item=self.record(p[keep],world[keep],robot[keep],meta,source='yolo_rgb_depth',bbox_xyxy=det['bbox_xyxy'])
            item.update(class_name=det['class_name'],confidence=det['confidence'])
            objects.append(item)
        return objects

    def geometry(self,depth,meta):
        h,w=depth.shape;s=self.cfg['stride']
        u,v=np.meshgrid(np.arange(0,w,s),np.arange(0,h,s));u=u.ravel();v=v.ravel()
        p,world,robot,valid=self.points(depth,meta,u,v)
        p,world,robot=p[valid],world[valid],robot[valid]
        cells={}
        for i,key in enumerate(np.floor(robot/self.cfg['voxel_m']).astype(int)):
            cells.setdefault(tuple(key),[]).append(i)
        objects=[]
        while cells:
            key,indices=cells.popitem();stack=[key]
            while stack:
                a,b,c=stack.pop()
                for da,db,dc in [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]:
                    nxt=(a+da,b+db,c+dc)
                    if nxt in cells:
                        indices.extend(cells.pop(nxt));stack.append(nxt)
            if len(indices)>=self.cfg['minimum_points']:
                objects.append(self.record(p[indices],world[indices],robot[indices],meta,source='depth_geometry'))
        return objects

    def fuse(self,geometric,semantic):
        # Keep geometric coverage even when a classifier misses an object.
        # Attach semantic association without overwriting geometric extents.
        fused=[]
        for geo in geometric:
            item=dict(geo)
            matches=[s for s in semantic if np.linalg.norm(np.array(s['position_robot_m'])-geo['position_robot_m'])<.5]
            item['semantic_candidates']=[{'class_name':s['class_name'],'confidence':s['confidence']} for s in matches]
            fused.append(item)
        return {'geometric_obstacles':fused,'semantic_obstacles':semantic}
