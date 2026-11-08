"""Bounded holonomic pose feedback for the verified biped velocity policy."""
import math


def wrap(value): return (value+math.pi)%(2*math.pi)-math.pi


def pose(state):
    w,x,y,z=state['base_orientation_wxyz']
    return (*state['base_position_m'][:2],math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


class LocalPlanner:
    def __init__(self,config): self.config=config; self.path=[]; self.index=1; self.settled=0
    def reset(self,path): self.path=path; self.index=min(1,len(path)-1); self.settled=0

    def command(self,state,goal,dt):
        x,y,yaw=pose(state); c=self.config; final=math.dist((x,y),goal[:2])
        while self.index<len(self.path)-1 and math.dist((x,y),self.path[self.index])<c['lookahead_m']: self.index+=1
        target=self.path[self.index]; dx,dy=target[0]-x,target[1]-y
        heading=math.atan2(dy,dx)
        # A configured heading is independent of travel direction: the biped can
        # walk backwards/laterally. This avoids repeatedly turning 180 degrees
        # when gait sway crosses the positional tolerance near the endpoint.
        desired_heading=goal[2] if goal[2] is not None else heading
        err=wrap(desired_heading-yaw)
        yaw_rate=max(-c['max_yaw_rate'],min(c['max_yaw_rate'],1.2*err))
        if final<c['position_tolerance_m']:
            vx=vy=0
            if goal[2] is None: yaw_rate=0
        elif abs(err)>.65 and final>.45:
            vx=vy=0
        else:
            vx=math.cos(yaw)*dx+math.sin(yaw)*dy; vy=-math.sin(yaw)*dx+math.cos(yaw)*dy
            length=math.hypot(vx,vy)
            speed=min(c['max_speed_m_s'],max(.12,.7*length))
            if final<.5: speed=min(speed,.12)
            vx,vy=vx/length*speed,vy/length*speed
        speed=math.hypot(*state['base_linear_velocity_world_m_s'][:2])
        angular=abs(state['base_angular_velocity_world_rad_s'][2])
        achieved=final<c['position_tolerance_m'] and (goal[2] is None or abs(wrap(goal[2]-yaw))<c['heading_tolerance_rad'])
        self.settled=self.settled+dt if achieved and speed<c['settle_speed_m_s'] and angular<c['settle_yaw_rate'] else 0
        return [vx,vy,yaw_rate],self.settled>=c['settle_seconds'],final
