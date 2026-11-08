"""Offline command conversion; no planner/control loop integration authorized yet."""
import math
import numpy as np


class SonicCommandAdapter:
    # IDs from NVIDIA planner_onnx.md, not invented speed thresholds.
    MODES={'idle':0,'slowWalk':1,'walk':2,'run':3}

    def convert(self, velocity, yaw, dt, mode='slowWalk'):
        v=np.asarray(velocity,dtype=float)
        if v.shape!=(3,) or not np.isfinite(v).all() or not math.isfinite(yaw) or not 0<dt<=.1:
            raise ValueError('Invalid command/yaw/dt')
        if mode not in self.MODES: raise ValueError('Unknown planner mode')
        speed=float(np.linalg.norm(v[:2]))
        facing=yaw+float(v[2])*dt
        movement=np.array([math.cos(yaw)*v[0]-math.sin(yaw)*v[1],
                           math.sin(yaw)*v[0]+math.cos(yaw)*v[1],0.0])
        if speed>1e-8: movement/=speed
        # target_vel=0 would select mode default speed, so stopped commands use IDLE.
        selected='idle' if np.linalg.norm(v)<1e-8 else mode
        return {'mode':self.MODES[selected],'target_vel':speed if speed>1e-8 else -1.0,
                'movement_direction':movement.tolist(),
                'facing_direction':[math.cos(facing),math.sin(facing),0.0]}
