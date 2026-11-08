"""Simulation-only monitor, not certified industrial safety. No pose writes."""
import math
from navigation.lane_map import box_distance
from navigation.local_planner import pose


class SafetySupervisor:
    def __init__(self,config,braking): self.config=config; self.braking=braking
    def stop_distance(self,speed):
        if not self.braking.get('validated'): raise ValueError('Measured braking calibration required')
        return speed*self.config['reaction_latency_seconds']+speed*speed/(2*self.braking['conservative_deceleration_m_s2'])+self.config['stopping_extra_m']
    def check(self,state,command,grid,obstacles,age,prediction_velocity=None):
        values=state['base_position_m']+state['base_orientation_wxyz']+state['base_linear_velocity_world_m_s']+state['base_angular_velocity_world_rad_s']
        if not all(math.isfinite(v) for v in values) or age>self.config['max_obstacle_age_seconds']: return 'FAULT','Missing/stale/nonfinite feedback'
        q=state['base_orientation_wxyz']; norm=math.sqrt(sum(v*v for v in q))
        if abs(norm-1)>.02: return 'FAULT','Invalid orientation'
        up=1-2*(q[1]*q[1]+q[2]*q[2])
        if up<math.cos(math.radians(self.config['max_tilt_deg'])) or state['base_position_m'][2]<.5: return 'FAULT','Robot unstable/fallen'
        x,y,yaw=pose(state); p=(x,y)
        if not grid.map.safe(p): return 'FAULT','Footprint outside lane or in obstacle'
        raw_speed=math.hypot(*state['base_linear_velocity_world_m_s'][:2])
        if raw_speed>self.config['max_actual_speed_m_s']: return 'FAULT','Actual velocity outside validated envelope'
        actual=prediction_velocity if prediction_velocity is not None else state['base_linear_velocity_world_m_s'][:2]
        speed=math.hypot(*actual)
        braking=self.stop_distance(speed)
        world=(math.cos(yaw)*command[0]-math.sin(yaw)*command[1],math.sin(yaw)*command[0]+math.cos(yaw)*command[1])
        travel=actual if speed>.05 else world
        for o in obstacles:
            extra=self.config['human_extra_m'] if o['kind']=='human' else 0
            d=box_distance(p,o['bounds'])-grid.map.radius
            lo,hi=o['bounds']
            nearest=[max(lo[i],min(hi[i],p[i])) for i in [0,1]]
            toward=sum((nearest[i]-p[i])*travel[i] for i in [0,1])>0
            if d<braking+extra and (o['kind']=='human' or toward or d<.05):
                return 'WAITING_FOR_CLEAR_PATH','Human near' if o['kind']=='human' else 'Obstacle in braking envelope'
            # Predict moving volumes through the stopping horizon, not just their current centers.
            horizon=self.config['prediction_seconds']
            lo,hi=o['bounds']; vx,vy=o.get('velocity',[0,0])
            swept=[[min(lo[0],lo[0]+vx*horizon),min(lo[1],lo[1]+vy*horizon)],
                   [max(hi[0],hi[0]+vx*horizon),max(hi[1],hi[1]+vy*horizon)]]
            # Include robot travel before braking as well as the obstacle sweep.
            if o.get('dynamic') and box_distance(p,swept)<grid.map.radius+braking+speed*horizon+extra:
                return 'WAITING_FOR_CLEAR_PATH','Predicted crossing obstacle/human'
        for velocity in [world,actual]:
            length=math.hypot(*velocity)
            end=(x+velocity[0]*self.config['prediction_seconds'],y+velocity[1]*self.config['prediction_seconds'])
            if length>1e-6: end=(end[0]+velocity[0]/length*braking,end[1]+velocity[1]/length*braking)
            if not grid.segment_free(p,end): return 'WAITING_FOR_CLEAR_PATH','Predicted trajectory/footprint blocked'
        return None,''
