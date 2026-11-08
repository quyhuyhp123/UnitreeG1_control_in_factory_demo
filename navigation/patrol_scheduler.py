"""Nonblocking A-B-A scheduler. Intervals start at completed return, never overlap."""
import math


def validate_schedule(config):
    for name in ['start_delay_seconds','repeat_interval_seconds','dwell_at_b_seconds','movement_timeout_seconds','obstacle_wait_timeout_seconds']:
        v=config[name]
        if not isinstance(v,(int,float)) or not math.isfinite(v) or v<0: raise ValueError('Invalid '+name)
    if config['movement_timeout_seconds']<=0 or config['obstacle_wait_timeout_seconds']<=0: raise ValueError('Timeout must be positive')
    if type(config['max_cycles']) is not int or config['max_cycles']<0: raise ValueError('Invalid max_cycles')
    if type(config['repeat_enabled']) is not bool: raise ValueError('repeat_enabled must be boolean')
    if config['clock_mode'] not in ('simulation','wall'): raise ValueError('Invalid clock mode')


class PatrolScheduler:
    def __init__(self,config):
        validate_schedule(config); self.config=dict(config); self.state='IDLE'; self.completed=0; self.deadline=0; self.goal='B'; self.reason=''; self.events=[]
    def transition(self,state,now,reason=''):
        self.state=state; self.entered=now; self.reason=reason
        self.events.append({'state':state,'time':now,'reason':reason,'completed':self.completed})
    def start(self,now):
        self.completed=0; self.goal='B'; self.transition('WAITING_START',now); self.deadline=now+self.config['start_delay_seconds']
    def update(self,now):
        if self.state=='WAITING_START' and now>=self.deadline: self.transition('PLANNING',now)
        elif self.state=='WAITING_AT_B' and now>=self.deadline: self.goal='A'; self.transition('REPLANNING',now)
        elif self.state=='CYCLE_COMPLETE' and now>=self.deadline: self.goal='B'; self.transition('PLANNING',now)
    def arrived(self,now):
        if self.goal=='B':
            self.transition('WAITING_AT_B',now); self.deadline=now+self.config['dwell_at_b_seconds']
        else:
            self.completed+=1
            if not self.config['repeat_enabled'] or (self.config['max_cycles'] and self.completed>=self.config['max_cycles']):
                self.transition('STOPPED',now,'Configured cycles complete')
            else:
                self.transition('CYCLE_COMPLETE',now); self.deadline=now+self.config['repeat_interval_seconds']
    def pause(self,now):
        if self.state not in ('IDLE','STOPPED','FAULT','PAUSED'):
            self.saved_state=self.state; self.paused_at=now; self.transition('PAUSED',now,'Explicit Resume required')
    def resume(self,now):
        if self.state!='PAUSED': raise ValueError('Not paused')
        previous=self.saved_state
        if self.config['clock_mode']=='simulation': self.deadline+=now-self.paused_at
        self.transition('REPLANNING' if previous in ('MOVING_TO_A','MOVING_TO_B','WAITING_FOR_CLEAR_PATH','REPLANNING','PLANNING') else previous,now)
