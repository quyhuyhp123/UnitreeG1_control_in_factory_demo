"""Sole velocity-command source, bounded by the real locomotion deploy contract."""
import math


class CommandSelector:
    def __init__(self,keyboard,low,high):
        self.keyboard=keyboard; self.low=low; self.high=high; self.mode='MANUAL'; self.previous=[0.,0.,0.]
    def select(self,mode):
        if mode not in ('MANUAL','AUTO PATROL'): raise ValueError('Unknown control mode')
        self.mode=mode; self.previous=[0.,0.,0.]; self.keyboard.clear('Control source changed: fresh keys required')
    def auto(self,command,dt):
        if len(command)!=3 or not all(math.isfinite(v) for v in command): raise ValueError('Invalid navigation command')
        result=[]
        for i,target in enumerate(command):
            if not self.low[i]<=target<=self.high[i]: raise ValueError('Navigation velocity exceeds policy range')
            current=self.previous[i]; limit=(.8 if abs(target)<abs(current) else .4) if i<2 else .5
            result.append(current+max(-limit*dt,min(limit*dt,target-current)))
        self.previous=result; return list(result)
    def stop(self): self.previous=[0.,0.,0.]; return [0.,0.,0.]
