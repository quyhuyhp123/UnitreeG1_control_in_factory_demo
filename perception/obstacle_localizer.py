"""3D obstacle localisation from one synchronised RGB-D frame plus YOLO detections.

Two sources are fused:
  * YOLO boxes -> robust depth inside the box -> 3D position of a named object;
  * depth alone -> floor/robot points removed -> occupancy grid -> clusters, so
    obstacles are found even when YOLO has no class for them.

Frames (metres):
  optical : camera, +Z forward, +X right, +Y down (OpenCV)
  robot   : origin on the floor under the base, +X forward, +Y left, +Z up,
            yaw-only (gravity aligned)
  world   : simulation world, Z up

Pure numpy/scipy: no Isaac Sim import, no commands, no navigation access. USD
ground truth is never read here; it is used only by the evaluation script.
"""
from pathlib import Path
import json,math,time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def load_config(overrides=None):
    config=json.loads((ROOT/'configs/obstacle_localization.json').read_text())
    config.update(overrides or {})
    if not .02<=config['grid_resolution_m']<=.5 or config['pixel_stride'] not in (1,2,3,4): raise ValueError('Invalid obstacle grid/stride')
    if not 0<config['min_range_m']<config['max_range_m']<=20: raise ValueError('Invalid obstacle range')
    if not 0<config['floor_tolerance_m']<.3 or not .5<config['max_height_m']<5: raise ValueError('Invalid floor/height band')
    return config


def robot_pose(position,orientation_wxyz,floor_z=0.):
    """Yaw-only robot frame on the floor under the base. Returns (T_world_robot, yaw)."""
    w,x,y,z=orientation_wxyz; yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)); c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0,position[0]],[s,c,0,position[1]],[0,0,1,floor_z],[0,0,0,1]],dtype=np.float64),yaw


def transform(T,points): return points@T[:3,:3].T+T[:3,3]


class ObstacleLocalizer:
    def __init__(self,config):
        self.config=config; self.grid=None; self.observation=None; self.last_ms=0.

    # ---- geometry --------------------------------------------------------
    def backproject(self,depth,intrinsics):
        """Depth image (Z along the optical axis, metres) -> HxWx3 points in the optical frame."""
        s=self.config['pixel_stride']; z=depth[::s,::s]
        v,u=np.mgrid[0:depth.shape[0]:s,0:depth.shape[1]:s]
        x=(u+.5-intrinsics['cx'])/intrinsics['fx']*z; y=(v+.5-intrinsics['cy'])/intrinsics['fy']*z
        return np.stack([x,y,z],axis=-1)

    def classify_points(self,robot,valid):
        """Boolean masks: obstacle candidates, floor, robot body, above the height band."""
        c=self.config; horizontal=np.hypot(robot[...,0],robot[...,1])
        usable=valid&(horizontal<=c['max_range_m'])&(horizontal>=c['min_range_m'])
        # Pitch/pose error lifts or sinks distant floor points, so the floor band widens with range.
        floor=usable&(robot[...,2]<c['floor_tolerance_m']+c['floor_tolerance_per_m']*horizontal)
        high=usable&(robot[...,2]>c['max_height_m'])
        body=valid&(horizontal<c['self_radius_m'])
        return usable&~floor&~high&~body,floor,body,high

    # ---- depth-only obstacles --------------------------------------------
    def cluster(self,robot,mask):
        """Occupancy grid in the robot frame, then 8-connected components. Returns (labels per point, count)."""
        from scipy import ndimage
        c=self.config; res=c['grid_resolution_m']; half=c['grid_half_width_m']
        nx=math.ceil(c['max_range_m']/res); ny=math.ceil(2*half/res)
        ix=np.floor(robot[...,0]/res).astype(np.int64); iy=np.floor((robot[...,1]+half)/res).astype(np.int64)
        inside=mask&(ix>=0)&(ix<nx)&(iy>=0)&(iy<ny)
        counts=np.bincount((ix[inside]*ny+iy[inside]),minlength=nx*ny).reshape(nx,ny)
        occupied=counts>=c['min_points_per_cell']
        cells,number=ndimage.label(occupied,structure=np.ones((3,3)))
        self.grid={'counts':counts,'occupied':occupied,'labels':cells,'resolution_m':res,'half_width_m':half}
        labels=np.zeros(mask.shape,dtype=np.int32); labels[inside]=cells[ix[inside],iy[inside]]
        return labels,number

    def describe(self,robot,optical,world,members):
        """Statistics of one obstacle from its member points (boolean mask over the point image)."""
        pr=robot[members]; horizontal=np.hypot(pr[:,0],pr[:,1]); ranges=np.linalg.norm(optical[members],axis=1)
        nearest=pr[np.argsort(horizontal)[:max(1,len(pr)//50)]].mean(0)  # mean of the closest 2 % of points
        centre=np.median(pr,axis=0); pw=world[members]
        return {'distance_m':float(np.percentile(horizontal,2)),'bearing_deg':float(math.degrees(math.atan2(nearest[1],nearest[0]))),
            'range_camera_m':float(np.percentile(ranges,2)),'depth_median_m':float(np.median(optical[members][:,2])),
            'position_robot_m':[float(v) for v in centre],'nearest_point_robot_m':[float(v) for v in nearest],
            'position_world_m':[float(v) for v in np.median(pw,axis=0)],
            'bounds_robot_m':[[float(pr[:,0].min()),float(pr[:,1].min())],[float(pr[:,0].max()),float(pr[:,1].max())]],
            'bounds':[[float(pw[:,0].min()),float(pw[:,1].min())],[float(pw[:,0].max()),float(pw[:,1].max())]],
            'height_range_m':[float(np.percentile(pr[:,2],1)),float(np.percentile(pr[:,2],99))],'points':int(len(pr))}

    # ---- YOLO boxes -------------------------------------------------------
    def object_points(self,box,optical,candidates):
        """Robust foreground selection inside a detection box. Returns a boolean mask or None.

        The box always contains background (wall behind a person, floor around the
        feet). Floor/robot points are already excluded from `candidates`; the
        nearest depth layer holding a meaningful share of the remaining pixels is
        taken as the object, and everything behind it is dropped.
        """
        c=self.config; s=c['pixel_stride']; x0,y0,x1,y1=box
        shrink=c['box_shrink_fraction']; dx,dy=(x1-x0)*shrink,(y1-y0)*shrink
        region=np.zeros(candidates.shape,dtype=bool)
        r0,r1=int((y0+dy)//s),int(math.ceil((y1-dy)/s)); c0,c1=int((x0+dx)//s),int(math.ceil((x1-dx)/s))
        region[max(0,r0):max(r0+1,r1),max(0,c0):max(c0+1,c1)]=True
        inner=region&candidates
        if inner.sum()<c['min_object_points']:
            # Small or thin object: fall back to the whole box.
            region[:]=False; region[max(0,int(y0//s)):int(math.ceil(y1/s)),max(0,int(x0//s)):int(math.ceil(x1/s))]=True
            inner=region&candidates
            if inner.sum()<c['min_object_points']: return None
        depth=optical[...,2][inner]; width=c['depth_bin_m']
        edges=np.arange(depth.min(),depth.max()+2*width,width); histogram,_=np.histogram(depth,bins=edges)
        strong=np.nonzero(histogram>=max(c['min_object_points']//2,c['foreground_fraction']*len(depth)))[0]
        if not len(strong): return None
        first=last=strong[0]
        # Extend through contiguous populated bins: the object's own depth extent.
        while last+1<len(histogram) and histogram[last+1]>=.02*len(depth) and (last+1-first)*width<c['max_object_depth_m']: last+=1
        return inner&(optical[...,2]>=edges[first])&(optical[...,2]<edges[last+1])

    # ---- one frame --------------------------------------------------------
    def process(self,frame,detections):
        """frame: dict from RgbdCamera.read(); detections: YOLO list for that same frame."""
        started=time.perf_counter(); c=self.config
        optical=self.backproject(frame['depth_m'],frame['intrinsics']); s=c['pixel_stride']
        valid=frame['valid'][::s,::s]&np.isfinite(optical[...,2])
        optical=np.where(valid[...,None],optical,0.)
        T_world_robot,yaw=robot_pose(frame['base_position_m'],frame['base_orientation_wxyz'],c['floor_z_m'])
        T_robot_optical=np.linalg.inv(T_world_robot)@frame['T_world_optical']
        robot=transform(T_robot_optical,optical); world=transform(frame['T_world_optical'],optical)
        candidates,floor,body,high=self.classify_points(robot,valid)
        labels,number=self.cluster(robot,candidates)
        stamp={'sim_time':frame['sim_time'],'frame_index':frame['index'],'control_step':frame['control_step']}
        clusters={}
        semantic=np.full(candidates.shape,-1,dtype=np.int16); score=np.zeros(candidates.shape,dtype=np.float32)
        for label in range(1,number+1):
            members=labels==label
            if members.sum()<c['min_cluster_points']: continue
            clusters[label]={'source':'depth','class_name':'unknown','confidence':None,'kind':'static','members':members,
                             'cells':int((self.grid['labels']==label).sum())}
        obstacles=[]
        for detection in sorted(detections,key=lambda d:-d['confidence']):
            members=self.object_points(detection['bbox_xyxy'],optical,candidates)
            if members is None:
                detection.update(distance_m=None,position_robot_m=None,localized=False); continue
            info=self.describe(robot,optical,world,members)
            detection.update(distance_m=info['distance_m'],position_robot_m=info['position_robot_m'],localized=True)
            kind='human' if detection['class_name']=='person' else 'static'
            votes=np.bincount(labels[members],minlength=number+1); owner=int(votes[1:].argmax())+1 if number and votes[1:].any() else 0
            # The object owns its cluster when it supplies most of the cluster's points; otherwise it is
            # reported by itself and the cluster (e.g. a wall or a pedestal it touches) stays 'unknown'.
            if owner in clusters and clusters[owner]['class_name']=='unknown' and votes[owner]>=.5*clusters[owner]['members'].sum():
                clusters[owner].update(source='yolo+depth',class_name=detection['class_name'],confidence=detection['confidence'],kind=kind,
                                       bbox_xyxy=detection['bbox_xyxy'])
                semantic[clusters[owner]['members']]=detection['class_id']; score[clusters[owner]['members']]=detection['confidence']
            elif owner in clusters and clusters[owner]['class_name']==detection['class_name']:
                pass  # a second, weaker box on an object that is already labelled
            else:
                semantic[members]=detection['class_id']; score[members]=detection['confidence']
                obstacles.append({'source':'yolo+depth','class_name':detection['class_name'],'confidence':detection['confidence'],'kind':kind,
                                  'bbox_xyxy':detection['bbox_xyxy'],'cells':None,**info,**stamp})
        for label,item in clusters.items():
            members=item.pop('members'); obstacles.append({**item,**self.describe(robot,optical,world,members),**stamp})
        obstacles.sort(key=lambda o:o['distance_m'])
        for index,o in enumerate(obstacles): o['id']=f"{o['class_name'].replace(' ','_')}_{index}"; o['dynamic']=False; o['velocity']=[0.,0.]
        self.last_ms=(time.perf_counter()-started)*1000
        # Per-point data for map building (world points, obstacle/floor masks, YOLO class per obstacle point).
        self.observation={'world':world,'obstacle':candidates,'floor':floor,'class_id':semantic,'confidence':score}
        return {'obstacles':obstacles,'robot_pose_world':{'x':float(T_world_robot[0,3]),'y':float(T_world_robot[1,3]),'yaw_rad':yaw},
            'T_robot_optical':T_robot_optical.tolist(),**stamp,
            'sync':{'rgb_depth_same_render_frame':True,'yolo_frame_index':frame['index'],'depth_frame_index':frame['index'],
                    'pose_sim_time':frame['sim_time'],'pose_lag_ticks':frame['pose_lag_ticks']},
            'points':{'valid':int(valid.sum()),'floor':int(floor.sum()),'robot_body':int(body.sum()),'above_height_band':int(high.sum()),
                      'obstacle':int(candidates.sum())},'localization_ms':self.last_ms}


def render_topdown(result,grid,ground_truth=None,size=560,span_m=9.):
    """Bird's-eye debug image in the robot frame: robot at the bottom centre, +X up, +Y left."""
    from PIL import Image,ImageDraw
    image=Image.new('RGB',(size,size),(18,18,22)); draw=ImageDraw.Draw(image); scale=size/span_m
    def pixel(x,y): return size/2-y*scale,size-20-x*scale
    for metres in range(1,int(span_m)+1):
        r=metres*scale; cx,cy=pixel(0,0); draw.arc([cx-r,cy-r,cx+r,cy+r],180,360,fill=(50,50,60))
        draw.text((cx+3,cy-r-11),f'{metres} m',fill=(90,90,100))
    if grid is not None:
        res=grid['resolution_m']
        for ix,iy in zip(*np.nonzero(grid['occupied'])):
            x0,y0=pixel(ix*res,iy*res-grid['half_width_m']); x1,y1=pixel((ix+1)*res,(iy+1)*res-grid['half_width_m'])
            draw.rectangle([min(x0,x1),min(y0,y1),max(x0,x1),max(y0,y1)],fill=(120,120,130))
    for box in ground_truth or []:
        corners=[pixel(x,y) for x,y in box['corners_robot_m']]; draw.polygon(corners,outline=(60,220,90))
        draw.text(corners[0],box['name'],fill=(60,220,90))
    for o in result['obstacles']:
        colour=(255,90,90) if o['kind']=='human' else ((255,190,40) if o['source']=='yolo+depth' else (90,170,255))
        (lx,ly),(hx,hy)=o['bounds_robot_m']; a=pixel(lx,ly); b=pixel(hx,hy)
        draw.rectangle([min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1])],outline=colour,width=2)
        px,py=pixel(*o['nearest_point_robot_m'][:2]); draw.ellipse([px-3,py-3,px+3,py+3],fill=colour)
        draw.text((min(a[0],b[0]),min(a[1],b[1])-11),f"{o['class_name']} {o['distance_m']:.2f} m",fill=colour)
    cx,cy=pixel(0,0); draw.polygon([(cx,cy-12),(cx-7,cy+6),(cx+7,cy+6)],fill=(255,255,255))
    draw.text((6,6),f"robot frame | t={result['sim_time']:.2f} s | {len(result['obstacles'])} obstacles | red person, orange YOLO, blue depth-only"
              +(', green USD ground truth' if ground_truth else ''),fill=(220,220,220))
    return np.asarray(image)
