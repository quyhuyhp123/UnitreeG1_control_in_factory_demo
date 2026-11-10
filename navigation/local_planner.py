"""Bounded holonomic pose feedback for the verified biped velocity policy."""
import math


def wrap(value): return (value+math.pi)%(2*math.pi)-math.pi


def pose(state):
    w,x,y,z=state['base_orientation_wxyz']
    return (*state['base_position_m'][:2],math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


class LocalPlanner:
    def __init__(self,config):
        self.config=config; self.reset([])
    def reset(self,path):
        self.path=path; self.index=min(1,len(path)-1); self.settled=0
        self.position_reached=False; self.travel_heading=None; self.turning=True

    def command(self,state,goal,dt):
        x,y,yaw=pose(state); c=self.config; final=math.dist((x,y),goal[:2])
        while self.index<len(self.path)-1 and math.dist((x,y),self.path[self.index])<c['lookahead_m']: self.index+=1
        target=self.path[self.index]; dx,dy=target[0]-x,target[1]-y
        heading=math.atan2(dy,dx)
        # Face the path before translating, including the B -> A return leg.
        # Waypoint heading is an arrival orientation, never a travel heading.
        tolerance=c['position_tolerance_m']
        if final<tolerance: self.position_reached=True
        elif final>max(.5,tolerance+.10): self.position_reached=False
        # Freeze the approach heading close to the endpoint so gait sway or a
        # small overshoot cannot repeatedly request a 180-degree turn.
        if self.travel_heading is None or final>max(.3,tolerance+.10):
            self.travel_heading=heading
        desired_heading=(goal[2] if goal[2] is not None else yaw) if self.position_reached else self.travel_heading
        err=wrap(desired_heading-yaw)
        yaw_rate=max(-c['max_yaw_rate'],min(c['max_yaw_rate'],1.2*err))
        if self.position_reached:
            vx=vy=0
            # Hold the arrival position while applying the optional final yaw.
            # Biped turning has real translational drift; a small holonomic
            # correction avoids leaving A and starting another half-turn.
            if final>tolerance*.6:
                gx,gy=goal[0]-x,goal[1]-y
                scale=min(.08,.4*final)/final
                vx=(math.cos(yaw)*gx+math.sin(yaw)*gy)*scale
                vy=(-math.sin(yaw)*gx+math.cos(yaw)*gy)*scale
        else:
            if abs(err)>.65: self.turning=True
            elif abs(err)<c['heading_tolerance_rad']: self.turning=False
            vx=math.cos(yaw)*dx+math.sin(yaw)*dy; vy=-math.sin(yaw)*dx+math.cos(yaw)*dy
            length=math.hypot(vx,vy)
            speed=min(c['max_speed_m_s'],max(.12,.7*length))
            if final<.5: speed=min(speed,.10)
            vx,vy=max(0.,vx/length*speed),vy/length*speed
            if self.turning or vx==0: vx=vy=0
        speed=math.hypot(*state['base_linear_velocity_world_m_s'][:2])
        angular=abs(state['base_angular_velocity_world_rad_s'][2])
        achieved=final<c['position_tolerance_m'] and (goal[2] is None or abs(wrap(goal[2]-yaw))<c['heading_tolerance_rad'])
        self.settled=self.settled+dt if achieved and speed<c['settle_speed_m_s'] and angular<c['settle_yaw_rate'] else 0
        return [vx,vy,yaw_rate],self.settled>=c['settle_seconds'],final
