"""Sole velocity-command source, bounded by the real locomotion deploy contract."""
import math


class CommandSelector:
    def __init__(self,keyboard,low,high):
        self.keyboard=keyboard; self.low=low; self.high=high; self.mode='MANUAL'; self.previous=[0.,0.,0.]
    def select(self,mode):
        if mode not in ('MANUAL','AUTO PATROL'): raise ValueError('Unknown control mode')
        self.mode=mode; self.previous=[0.,0.,0.]; self.keyboard.clear('Control source changed: fresh keys required')
    def auto(self,command,dt,preserve_linear_direction=False):
        if len(command)!=3 or not all(math.isfinite(v) for v in command): raise ValueError('Invalid navigation command')
        if preserve_linear_direction:
            if any(not self.low[i]<=v<=self.high[i] for i,v in enumerate(command)): raise ValueError('Navigation velocity exceeds policy range')
            delta=[command[i]-self.previous[i] for i in (0,1)]; length=math.hypot(*delta)
            rate=.8 if math.hypot(*command[:2])<math.hypot(*self.previous[:2]) else .4
            alpha=min(1.,rate*dt/length) if length else 1.
            result=[self.previous[i]+alpha*delta[i] for i in (0,1)]
            result.append(self.previous[2]+max(-.5*dt,min(.5*dt,command[2]-self.previous[2])))
            self.previous=result; return list(result)
        result=[]
        for i,target in enumerate(command):
            if not self.low[i]<=target<=self.high[i]: raise ValueError('Navigation velocity exceeds policy range')
            current=self.previous[i]; limit=(.8 if abs(target)<abs(current) else .4) if i<2 else .5
            result.append(current+max(-limit*dt,min(limit*dt,target-current)))
        self.previous=result; return list(result)
    def stop(self): self.previous=[0.,0.,0.]; return [0.,0.,0.]
