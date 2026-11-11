"""Constrain the existing grid/A* to one mandatory graph edge at a time."""
import math
from navigation.local_planner import wrap
from navigation.lane_map import segment_distance
class CorridorGrid:
    def __init__(self,base,a,b,allowance,start=None):
        self.base=base; self.map=base.map; self.a=a; self.b=b; self.allowance=allowance
        self.start=tuple(start) if start is not None else None
        self.start_cell=base.world_to_cell(self.start) if self.start is not None else None
    def __getattr__(self,name): return getattr(self.base,name)
    def contains(self,p): return segment_distance(p,self.a,self.b)<=self.allowance
    def cell_to_world(self,c): return self.start if c==self.start_cell else self.base.cell_to_world(c)
    def free(self,c):
        if c==self.start_cell: return self.map.safe(self.start) and self.contains(self.start)
        return self.base.free(c) and self.contains(self.base.cell_to_world(c))
    def point_free(self,p):
        if self.start is not None and math.dist(p,self.start)<1e-8: return self.map.safe(p) and self.contains(p)
        return self.base.point_free(p) and self.contains(p) and self.free(self.world_to_cell(p))
    def segment_free(self,a,b):
        # Exact full-radius checks avoid falsely rejecting a safe actual robot
        # pose merely because the *rest* of its occupancy cell is unsafe.
        n=max(1,math.ceil(math.dist(a,b)/(self.base.resolution/3)))
        for i in range(n+1):
            p=(a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n)
            if not self.contains(p) or not self.map.safe(p): return False
        return True
def heading(a,b): return math.atan2(b[1]-a[1],b[0]-a[0])
def is_corner(a,b,c): return abs(wrap(heading(b,c)-heading(a,b)))>.5
