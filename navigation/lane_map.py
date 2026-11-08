"""Conservative circular footprint inside polygon lanes, outside obstacles."""
import math


def segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx*dx+dy*dy else 0
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def inside(p,polygon):
    hit=False; x,y=p
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]: hit=not hit
    return hit


def edge_distance(p,polygon):
    return min(segment_distance(p,a,b) for a,b in zip(polygon,polygon[1:]+polygon[:1]))


def box_distance(p,bounds):
    lo,hi=bounds
    return math.hypot(max(lo[0]-p[0],0,p[0]-hi[0]),max(lo[1]-p[1],0,p[1]-hi[1]))


class LaneMap:
    def __init__(self,config,obstacles=(),floor=None):
        self.config=config; self.obstacles=list(obstacles); self.floor=floor
        self.radius=sum(float(config['footprint'][k]) for k in ['body_radius_m','step_sway_m','pose_error_m','margin_m'])
        self.lanes=[p['polygon'] for p in config['allowed_lanes']]
        self.forbidden=[p['polygon'] for p in config.get('forbidden_zones',[])]
        self.forbidden += [p['polygon'] for p in config.get('static_obstacles',[])]
        if not self.lanes or not .05<=config['resolution_m']<=.1 or not .2<=self.radius<=1:
            raise ValueError('Invalid lane resolution/footprint')
        for poly in self.lanes+self.forbidden:
            if len(poly)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in poly):
                raise ValueError('Invalid polygon')
            area=abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(poly,poly[1:]+poly[:1])))/2
            if area<1e-6: raise ValueError('Degenerate polygon')

    def safe(self,p,extra=0):
        r=self.radius+extra
        if not all(math.isfinite(v) for v in p): return False
        if not any(inside(p,poly) and edge_distance(p,poly)>=r for poly in self.lanes): return False
        if self.floor and any(p[i]-r<self.floor[0][i] or p[i]+r>self.floor[1][i] for i in [0,1]): return False
        if any(inside(p,poly) or edge_distance(p,poly)<=r for poly in self.forbidden): return False
        return not any(box_distance(p,o['bounds'])<=r for o in self.obstacles)
