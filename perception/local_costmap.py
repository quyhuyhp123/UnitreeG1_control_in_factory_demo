"""Rolling local obstacle costmap around the G1, built from RGB-D observations.

Frame: the grid is centred on the robot and axis-aligned with the WORLD frame
(a rolling window, like a ROS local costmap in the odom frame). Cell (ix, iy)
covers world [ix*res, (ix+1)*res) x [iy*res, (iy+1)*res). Keeping the axes
world-aligned means cells keep their identity while the robot turns or walks.

Per-cell observation state
  UNKNOWN   never seen, or the last observation is older than its memory
  FREE      the camera saw the floor there (and nothing above it) recently,
            or the robot is standing on it
  OCCUPIED  obstacle points seen there recently
A cell the camera cannot see (behind the robot, the blind zone at its feet, the
shadow behind an obstacle) is UNKNOWN, never FREE.

Cost layers (uint8, ROS convention)
  0         free of any obstacle influence
  1..252    inside the inflation radius, decaying with distance
  253       INSCRIBED: the robot centre here would touch an obstacle
  254       LETHAL: observed obstacle, or a Factory V2 area that is not walkable
  255       NO_INFORMATION

Output only. Nothing here commands the robot or feeds navigation.
"""
from pathlib import Path
import json,math,threading,time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
UNKNOWN,FREE,OCCUPIED=0,1,2
INSCRIBED,LETHAL,NO_INFORMATION=253,254,255
NEVER=-1e9


def load_config(overrides=None):
    config=json.loads((ROOT/'configs/local_costmap.json').read_text())
    config.update(overrides or {})
    if not 2<=config['size_m']<=20 or not .02<=config['resolution_m']<=.5: raise ValueError('Invalid costmap size/resolution')
    for key in ('occupied_memory_s','human_memory_s','free_memory_s'):
        if not 0<config[key]<=120: raise ValueError('Invalid costmap memory: '+key)
    return config


def inside_polygon(x,y,polygon):
    """Vectorised even-odd ray cast; same rule as navigation.lane_map.inside."""
    hit=np.zeros(np.shape(x),dtype=bool)
    for (ax,ay),(bx,by) in zip(polygon,polygon[1:]+polygon[:1]):
        crosses=(ay>y)!=(by>y)
        with np.errstate(divide='ignore',invalid='ignore'):
            hit^=crosses&(x<(bx-ax)*(y-ay)/(by-ay)+ax)
    return hit


class LocalCostmap:
    def __init__(self,config,navigation_map=None):
        self.config=config; self.res=float(config['resolution_m'])
        self.n=int(round(config['size_m']/self.res))
        self.navigation_map=navigation_map
        footprint=(navigation_map or {}).get('footprint',config['fallback_footprint'])
        # Same footprint numbers the factory navigation map uses.
        self.inscribed_radius=float(footprint['body_radius_m'])
        self.inflation_radius=sum(float(footprint[k]) for k in ('body_radius_m','step_sway_m','pose_error_m','margin_m'))
        self.human_extra=float(config['human_extra_m'])
        self.pad=math.ceil((self.inflation_radius+self.human_extra)/self.res)+1
        self.origin=None  # world cell index of array element [0, 0]
        shape=(self.n,self.n)
        self.occupied_time=np.full(shape,NEVER); self.free_time=np.full(shape,NEVER)
        self.class_id=np.full(shape,-1,dtype=np.int16); self.confidence=np.zeros(shape,dtype=np.float32)
        self.lock=threading.Lock(); self.snapshot=None; self.updates=0; self.last_ms=0.

    def reset(self):
        """Forget every observation (all cells UNKNOWN again)."""
        with self.lock:
            self.occupied_time[...]=NEVER; self.free_time[...]=NEVER; self.class_id[...]=-1; self.confidence[...]=0; self.snapshot=None

    # ---- window ----------------------------------------------------------
    def _roll(self,robot_xy):
        """Keep the window centred on the robot, shifting stored cells by whole cells."""
        origin=(int(math.floor(robot_xy[0]/self.res))-self.n//2,int(math.floor(robot_xy[1]/self.res))-self.n//2)
        if self.origin is None: self.origin=origin; return
        dx,dy=origin[0]-self.origin[0],origin[1]-self.origin[1]
        if not dx and not dy: return
        for array,fill in ((self.occupied_time,NEVER),(self.free_time,NEVER),(self.class_id,-1),(self.confidence,0)):
            moved=np.full_like(array,fill)
            sx0,sx1=max(0,dx),min(self.n,self.n+dx); sy0,sy1=max(0,dy),min(self.n,self.n+dy)
            if sx0<sx1 and sy0<sy1: moved[sx0-dx:sx1-dx,sy0-dy:sy1-dy]=array[sx0:sx1,sy0:sy1]
            array[...]=moved
        self.origin=origin

    def static_lethal(self,origin,n):
        """Factory V2 layer: True where the robot may never walk (outside the allowed area, or a forbidden zone)."""
        if self.navigation_map is None: return np.zeros((n,n),dtype=bool)
        centres=(np.arange(n)+.5)*self.res
        x=(origin[0]*self.res+centres)[:,None]*np.ones((1,n)); y=(origin[1]*self.res+centres)[None,:]*np.ones((n,1))
        allowed=np.zeros((n,n),dtype=bool)
        for lane in self.navigation_map['allowed_lanes']: allowed|=inside_polygon(x,y,lane['polygon'])
        forbidden=np.zeros((n,n),dtype=bool)
        for zone in self.navigation_map.get('forbidden_zones',[])+self.navigation_map.get('static_obstacles',[]):
            forbidden|=inside_polygon(x,y,zone['polygon'])
        return ~allowed|forbidden

    # ---- update ----------------------------------------------------------
    def update(self,observation,robot_xy,now):
        """observation: world points (..., 3) with boolean `obstacle`/`floor` masks and per-point class id/confidence."""
        started=time.perf_counter(); c=self.config
        with self.lock:
            self._roll(robot_xy)
            world=observation['world']
            ix=np.floor(world[...,0]/self.res).astype(np.int64)-self.origin[0]; iy=np.floor(world[...,1]/self.res).astype(np.int64)-self.origin[1]
            inside=(ix>=0)&(ix<self.n)&(iy>=0)&(iy<self.n)
            def count(mask):
                m=mask&inside
                return np.bincount(ix[m]*self.n+iy[m],minlength=self.n*self.n).reshape(self.n,self.n)
            obstacle_hits=count(observation['obstacle']); floor_hits=count(observation['floor'])
            occupied=obstacle_hits>=c['min_obstacle_points']
            # Floor seen and no obstacle point above it: observed free. A single stray obstacle point keeps it undecided.
            free=(floor_hits>=c['min_floor_points'])&(obstacle_hits==0)
            self.occupied_time[occupied]=now; self.free_time[free]=now
            # The robot itself stands on free ground.
            centres=(np.arange(self.n)+.5)*self.res
            cx=self.origin[0]*self.res+centres[:,None]; cy=self.origin[1]*self.res+centres[None,:]
            self.free_time[np.hypot(cx-robot_xy[0],cy-robot_xy[1])<=self.inscribed_radius]=now
            # Semantics: strongest YOLO class among the obstacle points of each occupied cell.
            self.class_id[occupied]=-1; self.confidence[occupied]=0
            labelled=observation['obstacle']&inside&(observation['class_id']>=0)
            if labelled.any():
                flat=ix[labelled]*self.n+iy[labelled]; order=np.argsort(observation['confidence'][labelled])
                classes=self.class_id.reshape(-1); scores=self.confidence.reshape(-1)
                classes[flat[order]]=observation['class_id'][labelled][order]; scores[flat[order]]=observation['confidence'][labelled][order]
            self.updates+=1
            self.snapshot=self._build(robot_xy,now)
            self.last_ms=(time.perf_counter()-started)*1000; self.snapshot['update_ms']=self.last_ms
            return self.snapshot

    def _build(self,robot_xy,now):
        from scipy import ndimage
        c=self.config; human_class=c['human_class_id']
        occupied_age=now-self.occupied_time; free_age=now-self.free_time
        memory=np.where(self.class_id==human_class,c['human_memory_s'],c['occupied_memory_s'])
        # The most recent observation of a cell decides, and only while it is fresh enough.
        occupied=(occupied_age<=memory)&(self.occupied_time>=self.free_time)
        free=(free_age<=c['free_memory_s'])&(self.free_time>self.occupied_time)
        state=np.full((self.n,self.n),UNKNOWN,dtype=np.uint8); state[free]=FREE; state[occupied]=OCCUPIED
        class_id=np.where(occupied,self.class_id,-1)
        # Inflate on a padded window so obstacles and forbidden areas just outside still push cost inwards.
        p=self.pad; size=self.n+2*p
        static=self.static_lethal((self.origin[0]-p,self.origin[1]-p),size)
        lethal=static.copy(); lethal[p:-p,p:-p]|=occupied
        human=np.zeros((size,size),dtype=bool); human[p:-p,p:-p]=occupied&(class_id==human_class)
        distance=ndimage.distance_transform_edt(~lethal)*self.res if lethal.any() else np.full((size,size),np.inf)
        if human.any():
            # People get extra clearance: treat them as if they were `human_extra` larger.
            distance=np.minimum(distance,np.maximum(ndimage.distance_transform_edt(~human)*self.res-self.human_extra,0.))
        span=max(self.inflation_radius-self.inscribed_radius,1e-6)
        cost=np.zeros((size,size),dtype=np.uint8)
        ring=(distance>self.inscribed_radius)&(distance<=self.inflation_radius)
        cost[ring]=np.clip(np.round(252*(1-(distance[ring]-self.inscribed_radius)/span)),1,252).astype(np.uint8)
        cost[distance<=self.inscribed_radius]=INSCRIBED; cost[lethal]=LETHAL
        cost=cost[p:-p,p:-p]; static=static[p:-p,p:-p]; distance=distance[p:-p,p:-p]
        cost[(state==UNKNOWN)&(cost==0)]=NO_INFORMATION
        # Traversable for the robot centre: actually observed free, walkable in the factory map, and outside every inflated obstacle.
        traversable=(state==FREE)&~static&(cost==0)
        return {'state':state,'cost':cost,'static_lethal':static,'class_id':class_id,'traversable':traversable,'distance_m':distance,
            'human_cells':np.argwhere(human[p:-p,p:-p]),
            'origin_cell':self.origin,'origin_world_m':[self.origin[0]*self.res,self.origin[1]*self.res],'resolution_m':self.res,
            'size_cells':self.n,'robot_xy':[float(robot_xy[0]),float(robot_xy[1])],'sim_time':float(now),
            'occupied_age_s':np.where(occupied,occupied_age,np.nan),'free_age_s':np.where(free,free_age,np.nan)}

    def latest(self):
        with self.lock: return self.snapshot

    # ---- queries ---------------------------------------------------------
    def cell(self,snapshot,x,y):
        ix=int(math.floor(x/self.res))-snapshot['origin_cell'][0]; iy=int(math.floor(y/self.res))-snapshot['origin_cell'][1]
        return (ix,iy) if 0<=ix<self.n and 0<=iy<self.n else None

    def summary(self,snapshot):
        state=snapshot['state']; cost=snapshot['cost']
        return {'sim_time':snapshot['sim_time'],'origin_world_m':snapshot['origin_world_m'],'robot_xy':snapshot['robot_xy'],
            'cells':{'unknown':int((state==UNKNOWN).sum()),'free':int((state==FREE).sum()),'occupied':int((state==OCCUPIED).sum()),
                     'static_lethal':int(snapshot['static_lethal'].sum()),'inscribed':int((cost==INSCRIBED).sum()),
                     'inflated':int(((cost>0)&(cost<INSCRIBED)).sum()),'traversable':int(snapshot['traversable'].sum()),
                     'labelled':int((snapshot['class_id']>=0).sum())},'update_ms':snapshot.get('update_ms',0.)}


def render(snapshot,robot_yaw=None,class_names=None,fov_deg=80.,scale=8,ground_truth=None):
    """Debug image, world-aligned: +X right, +Y up. Returns uint8 RGB."""
    from PIL import Image,ImageDraw
    n=snapshot['size_cells']; res=snapshot['resolution_m']; state=snapshot['state']; cost=snapshot['cost'].astype(np.float32)
    colour=np.zeros((n,n,3),dtype=np.uint8)
    colour[state==UNKNOWN]=(70,70,78); colour[state==FREE]=(232,238,232)
    ring=(cost>0)&(cost<INSCRIBED); t=(cost[ring]/252.)[:,None]
    colour[ring]=((1-t)*np.array([255,236,150])+t*np.array([255,150,40])).astype(np.uint8)
    colour[snapshot['cost']==INSCRIBED]=(225,95,60)
    # Unknown cells keep a dark tint under the inflation colours, so "not seen" stays visible everywhere.
    hidden=(state==UNKNOWN)&(snapshot['cost']>0)&(snapshot['cost']<=INSCRIBED)
    colour[hidden]=(.45*colour[hidden]+.55*np.array([70,70,78])).astype(np.uint8)
    colour[snapshot['static_lethal']]=(95,50,120)
    colour[state==OCCUPIED]=(20,20,20); colour[(state==OCCUPIED)&(snapshot['class_id']>=0)]=(0,90,200)
    human=(state==OCCUPIED)&(snapshot['class_id']==0); colour[human]=(200,0,0)
    # Array is [ix, iy]; image rows run top to bottom, so flip Y and swap axes.
    picture=np.repeat(np.repeat(colour.transpose(1,0,2)[::-1],scale,axis=0),scale,axis=1)
    side=n*scale; legend=150
    image=Image.new('RGB',(side+legend,side+18),(18,18,22)); image.paste(Image.fromarray(picture),(0,18)); draw=ImageDraw.Draw(image)
    ox,oy=snapshot['origin_world_m']
    def pixel(x,y): return (x-ox)/res*scale,18+side-(y-oy)/res*scale
    for k in range(1,int(n*res)):   # metre lines
        offset=(math.ceil(ox)+k-1-ox)/res*scale
        if 0<offset<side: draw.line([offset,18,offset,18+side],fill=(120,120,128)); draw.line([0,18+side-((math.ceil(oy)+k-1-oy)/res*scale),side,18+side-((math.ceil(oy)+k-1-oy)/res*scale)],fill=(120,120,128))
    for box in ground_truth or []:
        (lx,ly),(hx,hy)=box['low'],box['high']; a=pixel(lx,ly); b=pixel(hx,hy)
        draw.rectangle([min(a[0],b[0]),min(a[1],b[1]),max(a[0],b[0]),max(a[1],b[1])],outline=(40,230,90),width=2)
        draw.text((min(a[0],b[0]),min(a[1],b[1])-11),box['name'],fill=(40,230,90))
    if class_names:
        from scipy import ndimage
        for cid in np.unique(snapshot['class_id'][snapshot['class_id']>=0]):
            blobs,count=ndimage.label(snapshot['class_id']==cid,structure=np.ones((3,3)))
            for index,(cx,cy) in enumerate(ndimage.center_of_mass(blobs>0,blobs,range(1,count+1))):
                draw.text(pixel(ox+(cx+.5)*res+.15,oy+(cy+.5)*res+.25),class_names.get(int(cid),str(cid)),fill=(255,255,255))
    rx,ry=pixel(*snapshot['robot_xy'])
    if robot_yaw is not None:
        for sign in (-1,1):
            angle=robot_yaw+sign*math.radians(fov_deg/2); ex,ey=pixel(snapshot['robot_xy'][0]+3*math.cos(angle),snapshot['robot_xy'][1]+3*math.sin(angle))
            draw.line([rx,ry,ex,ey],fill=(60,160,255))
        hx,hy=pixel(snapshot['robot_xy'][0]+.45*math.cos(robot_yaw),snapshot['robot_xy'][1]+.45*math.sin(robot_yaw))
        draw.line([rx,ry,hx,hy],fill=(0,200,255),width=3)
    radius=.35/res*scale; draw.ellipse([rx-radius,ry-radius,rx+radius,ry+radius],outline=(0,200,255),width=2)
    draw.text((4,3),f"local costmap {n}x{n} @ {res:.2f} m | world-aligned, +X right +Y up | t={snapshot['sim_time']:.2f} s",fill=(230,230,230))
    entries=[((232,238,232),'free (seen)'),((70,70,78),'unknown'),((20,20,20),'occupied'),((200,0,0),'person'),((0,90,200),'YOLO class'),
             ((225,95,60),'inscribed'),((255,190,90),'inflation'),((95,50,120),'factory no-go'),((0,200,255),'robot + FOV'),((40,230,90),'ground truth')]
    for index,(swatch,name) in enumerate(entries):
        y=26+index*18; draw.rectangle([side+8,y,side+20,y+12],fill=swatch,outline=(200,200,200)); draw.text((side+26,y),name,fill=(230,230,230))
    return np.asarray(image)
