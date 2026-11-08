"""A* with no diagonal corner cutting and collision-checked line-of-sight smoothing."""
import heapq,math,time


class NoValidPath(ValueError): pass


class GlobalPlanner:
    def __init__(self,grid): self.grid=grid; self.latency_ms=0

    def plan(self,start,goal):
        begin=time.perf_counter(); g=self.grid
        if not g.point_free(start) or not g.point_free(goal): raise NoValidPath('NO_VALID_PATH: endpoint footprint outside lane or obstructed')
        source,target=g.world_to_cell(start),g.world_to_cell(goal)
        queue=[(0,source)]; costs={source:0}; parents={source:None}
        while queue:
            _,current=heapq.heappop(queue)
            if current==target: break
            for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nxt=(current[0]+dx,current[1]+dy)
                if not g.free(nxt): continue
                if dx and dy and (not g.free((current[0]+dx,current[1])) or not g.free((current[0],current[1]+dy))): continue
                cost=costs[current]+math.hypot(dx,dy)
                if cost>=costs.get(nxt,float('inf')): continue
                costs[nxt]=cost; parents[nxt]=current
                heapq.heappush(queue,(cost+math.dist(nxt,target),nxt))
        if target not in parents: raise NoValidPath('NO_VALID_PATH: inflated obstacles disconnect allowed lane')
        cells=[]; c=target
        while c is not None: cells.append(c); c=parents[c]
        points=[tuple(start)]+[g.cell_to_world(c) for c in reversed(cells)]+[tuple(goal)]
        smooth=[points[0]]; i=0
        while i<len(points)-1:
            j=len(points)-1
            while j>i+1 and not g.segment_free(points[i],points[j]): j-=1
            if not g.segment_free(points[i],points[j]): raise NoValidPath('NO_VALID_PATH: endpoint connector unsafe')
            smooth.append(points[j]); i=j
        self.latency_ms=(time.perf_counter()-begin)*1000
        return smooth
