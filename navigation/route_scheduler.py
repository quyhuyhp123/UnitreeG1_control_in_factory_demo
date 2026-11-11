"""Reuse clock/pause/limit handling; completion requires the verified exit."""
from navigation.patrol_scheduler import PatrolScheduler
class RouteScheduler(PatrolScheduler):
    def update(self,now):
        if self.state in ('WAITING_START','CYCLE_COMPLETE') and now>=self.deadline: self.transition('VALIDATING_ROUTE',now)
    def route_finished(self,now):
        if self.state!='MOVING_TO_EXIT': raise ValueError('Cycle completion requires verified active exit leg')
        self.completed+=1
        if not self.config['repeat_enabled'] or (self.config['max_cycles'] and self.completed>=self.config['max_cycles']):
            self.transition('STOPPED',now,'Verified route exit complete')
        else:
            self.transition('CYCLE_COMPLETE',now); self.deadline=now+self.config['repeat_interval_seconds']
    def resume(self,now):
        super().resume(now)
        if self.state not in ('WAITING_START','CYCLE_COMPLETE'): self.transition('REPLANNING',now)
