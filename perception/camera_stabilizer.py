"""Virtual sensor gimbal; only camera transforms, no robot/control writes."""
import math
import numpy as np

class CameraStabilizer:
    def __init__(self,config):
        self.config=config;self.position=None;self.yaw=None
    def reset(self):self.position=None;self.yaw=None
    def step(self,raw_position,yaw,pitch_deg,dt):
        position=np.asarray(raw_position,dtype=float)
        if self.position is None:self.position=position.copy();self.yaw=yaw
        else:
            tau=np.array([self.config['position_tau_s'],self.config['position_tau_s'],self.config['height_tau_s']])
            self.position+=(1-np.exp(-max(0.,dt)/tau))*(position-self.position)
            delta=(yaw-self.yaw+math.pi)%(2*math.pi)-math.pi
            self.yaw+=(1-math.exp(-max(0.,dt)/self.config['yaw_tau_s']))*delta
            # Limit virtual translation lag near the physical mounting point.
            lag=self.position-position;length=np.linalg.norm(lag)
            if length>self.config['max_position_lag_m']:self.position=position+lag*self.config['max_position_lag_m']/length
        pitch=math.radians(pitch_deg);c,s=math.cos(self.yaw),math.sin(self.yaw)
        forward=np.array([math.cos(pitch)*c,math.cos(pitch)*s,-math.sin(pitch)])
        right=np.array([s,-c,0.]);up=np.cross(right,forward)
        T=np.eye(4);T[:3,:3]=np.column_stack([right,up,-forward]);T[:3,3]=self.position
        return T
