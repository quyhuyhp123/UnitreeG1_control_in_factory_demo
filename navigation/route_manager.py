"""Ordered route graph; no fallback straight lines or alternate-branch shortcuts."""
import json,math
from pathlib import Path
from navigation.route_graph import CorridorGrid
from navigation.global_planner import GlobalPlanner,NoValidPath
from navigation.speed_profile import load_speed_profile
ROOT=Path(r'D:\unitree_g1')
class RouteManager:
    def __init__(self,factory_config,data=None):
        self.data=data if data is not None else json.loads((ROOT/'configs/patrol_routes.json').read_text())
        if self.data['factory_id']!='v2' or self.data['source_factory_sha256']!=factory_config['source_factory_sha256']:
            raise ValueError('Route factory identity/hash mismatch')
        if self.data.get('version')!=1 or self.data['factory_usd']!=factory_config['factory_usd']: raise ValueError('Route schema/factory path mismatch')
        if not 0<self.data['corridor_center_allowance_m']<=.5: raise ValueError('Invalid route corridor allowance')
        if set(self.data['routes'])!={'route_1_blue','route_2_brown'}: raise ValueError('Expected exactly the two supported routes')
        for point in self.data['nodes'].values():
            if len(point)!=2 or not all(math.isfinite(v) for v in point): raise ValueError('Invalid route coordinates')
        for rid,route in self.data['routes'].items():
            nodes=route['ordered_nodes']
            if any(n not in self.data['nodes'] for n in nodes): raise ValueError('Unknown route graph node')
            if any(math.dist(self.data['nodes'][a],self.data['nodes'][b])<1e-4 for a,b in zip(nodes,nodes[1:])): raise ValueError('Degenerate route edge')
            required='CENTER_CORRIDOR' if rid=='route_1_blue' else 'RIGHT_CORRIDOR'
            if nodes[:len(self.data['common_nodes'])]!=self.data['common_nodes'] or nodes[-1]!='EXIT' or required not in nodes:
                raise ValueError('Missing mandatory route checkpoints')
            if route['mandatory_segments']!=[a+'__'+b for a,b in zip(nodes,nodes[1:])]: raise ValueError('Route segment ID/order mismatch')
            if not .12<=route['speed_limit_m_s']<=load_speed_profile()['maximum_verified_m_s'] or not 0<route['turn_speed_limit_rad_s']<=.6 or not .08<=route['waypoint_tolerance_m']<=.2:
                raise ValueError('Invalid route velocity/tolerance')
            if any(k not in nodes or not math.isfinite(v) for k,v in route.get('heading_at_turns_rad',{}).items()): raise ValueError('Invalid optional turn heading')
        self.status={}; self.plans={}
    def route_id(self,mode,completed=0,first='route_1_blue'):
        if mode=='alternate':
            if first not in self.data['routes']: raise ValueError('Unknown first alternate route')
            return first if completed%2==0 else ('route_2_brown' if first=='route_1_blue' else 'route_1_blue')
        if mode not in self.data['routes']: raise ValueError('Unknown route')
        return mode
    def edge(self,grid,route_id,index,start=None):
        route=self.data['routes'][route_id]; nodes=route['ordered_nodes']; a=self.data['nodes'][nodes[index]]; b=self.data['nodes'][nodes[index+1]]
        constrained=CorridorGrid(grid,a,b,self.data['corridor_center_allowance_m'],start)
        try:
            planner=GlobalPlanner(constrained)
            if start is not None and math.dist(start,b)>.8:
                dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy)
                t=max(0.,min(1.,((start[0]-a[0])*dx+(start[1]-a[1])*dy)/(length*length)+.6/length))
                anchor=(a[0]+t*dx,a[1]+t*dy)
                path=planner.plan(start,anchor)+planner.plan(anchor,b)[1:]
            else: path=planner.plan(start or a,b)
        except NoValidPath as exc: raise NoValidPath('INVALID_ROUTE_SEGMENT '+route['mandatory_segments'][index]+': '+str(exc)) from exc
        return {'id':route['mandatory_segments'][index],'start_node':nodes[index],'end_node':nodes[index+1],'path':path,'grid':constrained,'goal':b}
    def validate(self,grid,route_id):
        route=self.data['routes'][route_id]; edges=[]
        try:
            for i in range(len(route['ordered_nodes'])-1): edges.append(self.edge(grid,route_id,i))
        except NoValidPath as exc:
            self.status[route_id]={'status':'INVALID_ROUTE_SEGMENT','error':str(exc)}; raise
        path=[edges[0]['path'][0]]+[p for e in edges for p in e['path'][1:]]
        plan={'route_id':route_id,'edges':edges,'path':path,'length_m':sum(math.dist(a,b) for a,b in zip(path,path[1:])),'waypoints':route['ordered_nodes']}
        self.plans[route_id]=plan; self.status[route_id]={'status':'VALID','length_m':plan['length_m'],'waypoints':len(plan['waypoints']),'estimated_travel_seconds':plan['length_m']/route['speed_limit_m_s'],'estimate_only':True}
        return plan
