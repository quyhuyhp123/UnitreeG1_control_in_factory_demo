"""Pure geometry and speed limits for V2 route tracking; no simulation APIs."""
import math
from navigation.local_planner import wrap
from navigation.route_graph import heading


def straight_run_end(nodes,coordinates,index,headings=()):
    end=index
    while end+2<len(nodes):
        a,b,c=(coordinates[nodes[j]] for j in (end,end+1,end+2))
        if nodes[end+1] in headings or abs(wrap(heading(b,c)-heading(a,b)))>1e-3: break
        end+=1
    return end


def densify(path,spacing=.2):
    result=[path[0]]
    for a,b in zip(path,path[1:]):
        count=max(1,math.ceil(math.dist(a,b)/spacing))
        result.extend([(a[0]+(b[0]-a[0])*j/count,a[1]+(b[1]-a[1])*j/count) for j in range(1,count+1)])
    return result


def remaining_length(position,path):
    """Arc length after the nearest projection onto the constrained polyline."""
    best=None; traversed=0.; total=0.
    for a,b in zip(path,path[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy)
        t=max(0.,min(1.,((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(length*length))) if length else 0.
        distance=math.hypot(position[0]-a[0]-t*dx,position[1]-a[1]-t*dy)
        candidate=(distance,-(traversed+t*length))
        if best is None or candidate<best: best=candidate
        traversed+=length; total+=length
    return max(0.,total+best[1]) if best is not None else 0.


def passed_waypoint(position,a,b,max_deviation):
    dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy)
    if not length: return False
    forward=((position[0]-b[0])*dx+(position[1]-b[1])*dy)/length
    lateral=abs((position[0]-a[0])*dy-(position[1]-a[1])*dx)/length
    return forward>=0 and lateral<=max_deviation


def lateral_clearance(grid,a,b):
    """Remaining full-footprint lateral room at the edge midpoint, in metres."""
    length=math.dist(a,b); normal=(-(b[1]-a[1])/length,(b[0]-a[0])/length)
    centre=((a[0]+b[0])/2,(a[1]+b[1])/2)
    if not grid.map.safe(centre): return 0.
    room=0.
    for index in range(1,21):
        distance=index*.01
        if not all(grid.map.safe((centre[0]+sign*normal[0]*distance,centre[1]+sign*normal[1]*distance)) for sign in (-1,1)): break
        room=distance
    return room


def speed_cap(requested,distance,safety,braking):
    approach=min(requested,.22)
    blend=max(0.,min(1.,(distance-1.2)/.8))
    curve=approach+(requested-approach)*blend
    t=safety['prediction_seconds']+safety['reaction_latency_seconds']
    room=distance-safety['stopping_extra_m']; a=braking['conservative_deceleration_m_s2']
    if not math.isfinite(a) or a<=0: raise ValueError('Invalid measured deceleration')
    envelope=0. if room<=0 else a*(-t+math.sqrt(t*t+2*room/a))
    return min(requested,curve,envelope)


def bound_policy_command(command,low,high,translation_cap):
    """Scale XY together, preserving direction under asymmetric policy bounds."""
    if not all(math.isfinite(v) for v in command): raise ValueError('Nonfinite policy command')
    if translation_cap<0 or not math.isfinite(translation_cap): raise ValueError('Invalid translation cap')
    scale=1.; length=math.hypot(*command[:2])
    if length: scale=min(scale,translation_cap/length)
    for i in (0,1):
        if not low[i]<=0<=high[i]: raise ValueError('Policy limits must contain zero')
        value=command[i]
        if value>high[i]: scale=min(scale,high[i]/value)
        elif value<low[i]: scale=min(scale,low[i]/value)
    # Runtime policy limits are numpy float32 arrays; keep telemetry JSON-safe.
    return [float(command[0]*scale),float(command[1]*scale),float(max(low[2],min(high[2],command[2])))]
