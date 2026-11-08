"""Cells are free only if the whole cell plus inflated footprint is safe."""
import math
import numpy as np


class OccupancyGrid:
    def __init__(self,lane_map):
        self.map=lane_map; self.resolution=float(lane_map.config['resolution_m'])
        points=[p for polygon in lane_map.lanes for p in polygon]
        self.origin=(min(p[0] for p in points),min(p[1] for p in points))
        self.width=math.ceil((max(p[0] for p in points)-self.origin[0])/self.resolution)
        self.height=math.ceil((max(p[1] for p in points)-self.origin[1])/self.resolution)
        if self.width*self.height>100000: raise ValueError('Map too large for single-environment planner')
        self.blocked=np.ones((self.height,self.width),dtype=bool)
        margin=self.resolution*math.sqrt(2)/2
        for y in range(self.height):
            for x in range(self.width): self.blocked[y,x]=not lane_map.safe(self.cell_to_world((x,y)),margin)

    def world_to_cell(self,p):
        return tuple(math.floor((p[i]-self.origin[i])/self.resolution) for i in [0,1])

    def cell_to_world(self,c): return tuple(self.origin[i]+(c[i]+.5)*self.resolution for i in [0,1])

    def free(self,c):
        x,y=c
        return 0<=x<self.width and 0<=y<self.height and not self.blocked[y,x]

    def point_free(self,p): return self.free(self.world_to_cell(p)) and self.map.safe(p)

    def segment_free(self,a,b):
        n=max(1,math.ceil(math.dist(a,b)/(self.resolution/3)))
        previous=self.world_to_cell(a)
        for i in range(n+1):
            p=tuple(a[j]+(b[j]-a[j])*i/n for j in [0,1]); c=self.world_to_cell(p)
            if not self.free(c): return False
            if c[0]!=previous[0] and c[1]!=previous[1]:
                if not self.free((c[0],previous[1])) or not self.free((previous[0],c[1])): return False
            previous=c
        return True
