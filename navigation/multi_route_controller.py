"""Single velocity-command source for mandatory, ordered V2 route segments."""
import json,math,time
from collections import deque
from pathlib import Path
from navigation.patrol_controller import PatrolController,ROOT,validate_navigation
from navigation.route_manager import RouteManager
from navigation.route_scheduler import RouteScheduler
from navigation.route_graph import heading,CorridorChainGrid
from navigation.route_motion import straight_run_end,densify,remaining_length,passed_waypoint,speed_cap,bound_policy_command,lateral_clearance
from navigation.local_planner import LocalPlanner,pose,wrap
from navigation.global_planner import NoValidPath
from navigation.lane_map import segment_distance
from navigation.speed_profile import load_speed_profile

class MultiRouteController(PatrolController):
    def __init__(self,runtime):
        super().__init__(runtime)
        self.route_manager=RouteManager(runtime.factory_cfg)
        settings=json.loads((ROOT/'configs/patrol_route_settings.json').read_text())
        if settings.get('version')!=1 or settings.get('factory_id')!='v2': raise ValueError('Route settings factory/version mismatch')
        if settings.get('source_factory_sha256')!=runtime.factory_cfg['source_factory_sha256'] or settings.get('factory_usd')!=runtime.factory_cfg['factory_usd']: raise ValueError('Route settings/source identity mismatch')
        self.scheduler=RouteScheduler(settings['schedule']); self.event_index=0
        self.route_mode=self.route_manager.route_id(settings['route_mode'])
        self.active_route=None; self.segment_index=0; self.edge=None; self.route_plan=None; self.route_complete_history=[]
        self.position_recovery=False
        self.velocity_history=deque(maxlen=30)
        self.config['safety']['reaction_latency_seconds']=.65
        self.speed_profile=load_speed_profile(); self.speed_limit=self.speed_profile['maximum_verified_m_s']
        self.config['local']['max_speed_m_s']=self.speed_profile['default_m_s']
        self.config['safety']['max_actual_speed_m_s']=self.speed_profile['max_actual_speed_m_s']
        self.safety.swept_static_braking=runtime.factory_cfg.get('factory_id')=='v2'
        self.reason='Select route, PREVIEW / VALIDATE, then START. No automatic start.'
    def selected_route(self,completed=None):
        return self.route_manager.route_id(self.route_mode)
    def select_route(self,mode):
        if self.scheduler.state not in ('IDLE','STOPPED'): raise ValueError('STOP before changing route')
        if math.hypot(*self.runtime.last_state['base_linear_velocity_world_m_s'][:2])>.12: raise ValueError('Settle before route change')
        self.route_manager.route_id(mode)
        self.route_mode=mode; self.path=[]; self.edge=None; self.active_route=None; self.route_plan=None
    def validate_route(self):
        self.refresh_obstacles(force=True)
        ids=[self.selected_route()]
        for rid in ids: self.route_manager.validate(self.grid,rid)
        self.route_plan=self.route_manager.plans[self.selected_route()]; self.path=self.route_plan['path']
        self.reason='VALID route geometry; physical completion not implied'
        return self.route_plan
    def validate_points(self,a=None,b=None): return self.validate_route()
    def start(self):
        if self.scheduler.state=='FAULT': raise ValueError('Acknowledge FAULT before START')
        if not self.runtime.timeline.is_playing(): raise ValueError('Play simulation before START')
        if math.hypot(*self.runtime.last_state['base_linear_velocity_world_m_s'][:2])>.12: raise ValueError('Stop/settle before START')
        entry=self.route_manager.data['nodes']['ENTRY']
        if math.dist(pose(self.runtime.last_state)[:2],entry)>.2: raise ValueError('Robot must be at verified ENTRY')
        self.safety.stop_distance(0)
        if self.braking.get('checkpoint_sha256')!=self.runtime.loco_cfg['checkpoint_sha256']: raise ValueError('Braking checkpoint mismatch')
        self.validate_route(); self.selector.select('AUTO PATROL'); self.scheduler.start(self.now)
        self.active_route=None; self.edge=None; self.return_only=False; self.blocked_since=None
        self.route_complete_history=[]; self.cycle_durations=[]
        self.last_stop_generation=self.runtime.keys.stop_generation
    def begin_cycle(self,state):
        if math.dist(pose(state)[:2],self.route_manager.data['nodes']['ENTRY'])>.2:
            raise NoValidPath('ENTRY displaced before cycle start; return safely to ENTRY')
        self.active_route=self.selected_route(); self.route_plan=self.route_manager.validate(self.grid,self.active_route)
        self.segment_index=0; self.edge=None; self.cycle_start=self.now
        self.begin_edge(state)
    def begin_edge(self,state):
        begin=time.perf_counter()
        previous=self.edge
        edge=self.route_manager.edge(self.grid,self.active_route,self.segment_index,pose(state)[:2])
        holding=bool(previous and previous['id']==edge['id'] and self.local.position_reached and math.dist(pose(state)[:2],edge['goal'])<.5)
        if not holding: self.position_recovery=False
        self.edge=edge
        self.scheduler.goal=self.edge['end_node']
        route=self.route_manager.data['routes'][self.active_route]; nodes=route['ordered_nodes']; coordinates=self.route_manager.data['nodes']
        self.tracking_end_index=straight_run_end(nodes,coordinates,self.segment_index,route.get('heading_at_turns_rad',{}))
        tracking=list(self.edge['path'])
        for index in range(self.segment_index+1,self.tracking_end_index+1):
            tracking.extend(self.route_plan['edges'][index]['path'][1:])
        segments=[(coordinates[nodes[index]],coordinates[nodes[index+1]]) for index in range(self.segment_index,self.tracking_end_index+1)]
        self.motion_grid=CorridorChainGrid(self.grid,segments,self.route_manager.data['corridor_center_allowance_m'])
        self.lane_lateral_clearance_m=min(lateral_clearance(self.grid,a,b) for a,b in segments)
        self.lane_speed_cap_m_s=.22 if self.lane_lateral_clearance_m<.13 else self.speed_limit
        self.local.reset(densify(tracking)); self.path=self.route_plan['path']; self.movement_started=self.now
        self.local.position_reached=holding
        self.progress_time=self.now; self.best_distance=math.dist(pose(state)[:2],self.edge['goal']); self.progress_yaw=pose(state)[2]
        self.planner_latency_ms=(time.perf_counter()-begin)*1000
        self.scheduler.transition('MOVING_TO_EXIT' if self.return_only or self.segment_index==len(self.route_plan['edges'])-1 else 'MOVING_SEGMENT',self.now)
    def resume(self):
        if not self.runtime.timeline.is_playing(): raise ValueError('Play simulation first')
        self.scheduler.resume(self.now); self.selector.stop(); self.blocked_since=None
        self.movement_started=self.now; self.progress_time=self.now
        self.last_stop_generation=self.runtime.keys.stop_generation
    def return_to_a(self):
        if self.scheduler.state not in ('IDLE','STOPPED','PAUSED'): raise ValueError('STOP before RETURN TO ENTRY')
        if math.hypot(*self.runtime.last_state['base_linear_velocity_world_m_s'][:2])>.12: raise ValueError('Settle before returning')
        if math.dist(pose(self.runtime.last_state)[:2],self.route_manager.data['nodes']['ENTRY'])<.13:
            self.stop('Already at ENTRY; no cycle counted'); return
        if not self.edge or not self.route_plan: raise ValueError('No active verified route; use manual control to ENTRY')
        # Finish the remaining mandatory route; never shortcut through another branch.
        self.selector.select('AUTO PATROL'); self.return_only=True; self.last_stop_generation=self.runtime.keys.stop_generation
        self.scheduler.transition('REPLANNING',self.now,'Finish remaining route to ENTRY without counting a cycle')
    def finish_edge(self,state):
        self.segment_index+=1
        if self.segment_index<len(self.route_plan['edges']): self.begin_edge(state); return False
        if math.dist(pose(state)[:2],self.route_manager.data['nodes']['EXIT'])>self.config['local']['position_tolerance_m']:
            self.fail('Exit position not settled'); return True
        self.selector.stop()
        if self.return_only:
            self.scheduler.transition('STOPPED',self.now,'RETURN TO ENTRY complete'); self.return_only=False
        else:
            self.route_complete_history.append({'route_id':self.active_route,'completed_at':self.now,'ordered_segments':[e['id'] for e in self.route_plan['edges']]})
            self.cycle_durations.append(self.now-self.cycle_start); self.scheduler.route_finished(self.now)
        self.reason='Verified EXIT reached; route complete'; self.edge=None; self.active_route=None; self.route_plan=None; self.path=[]; return True
    def command(self,dt):
        if self.mode=='MANUAL': return self.record(self.runtime.last_state,self.runtime.keys.tick(dt))
        if self.runtime.keys.stop_generation!=self.last_stop_generation:
            self.last_stop_generation=self.runtime.keys.stop_generation; self.pause('Stop key; explicit Resume required')
        self.runtime.keys.command=[0.,0.,0.]
        state=self.runtime.policy.feedback.read_feedback(); p=pose(state)
        self.velocity_history.append(state['base_linear_velocity_world_m_s'][:2]); velocity=[sum(v[i] for v in self.velocity_history)/len(self.velocity_history) for i in (0,1)]
        # Full-stride mean below 3 cm/s is standing sway, not a travel direction.
        # Otherwise fixed stopping-extra projects jitter 8 cm sideways forever.
        # Raw pose/tilt/velocity and all footprint/braking margins stay checked.
        if math.hypot(*velocity)<.03: velocity=[0.,0.]
        self.refresh_obstacles(); self.scheduler.update(self.now)
        early,reason=self.safety.check(state,[0,0,0],self.grid,self.detector.obstacles,self.sim_time-self.detector.timestamp,velocity)
        if early=='FAULT': self.fail(reason)
        status=self.scheduler.state
        if status in ('IDLE','WAITING_START','CYCLE_COMPLETE','PAUSED','STOPPED','FAULT'): return self.record(state,self.selector.stop())
        try:
            if status=='VALIDATING_ROUTE': self.begin_cycle(state)
            elif status in ('REPLANNING','WAITING_FOR_CLEAR_PATH'):
                if self.blocked_since is not None and self.now-self.blocked_since>self.scheduler.config['obstacle_wait_timeout_seconds']:
                    self.fail('PATH_BLOCKED timeout'); return self.record(state,self.selector.stop())
                if status=='WAITING_FOR_CLEAR_PATH' and self.sim_time-self.last_replan<1.: return self.record(state,self.selector.stop())
                self.last_replan=self.sim_time
                # Preserve the original per-leg deadline during obstacle retries.
                if self.edge is None: self.begin_cycle(state)
                else:
                    started=self.movement_started; self.begin_edge(state); self.movement_started=started
        except NoValidPath as exc:
            self.wait(str(exc)); return self.record(state,self.selector.stop())
        route=self.route_manager.data['routes'][self.active_route]
        nodes=route['ordered_nodes']; coordinates=self.route_manager.data['nodes']
        while True:
            i=self.segment_index; goal=self.edge['goal']; distance=math.dist(p[:2],goal)
            straight=self.tracking_end_index>i
            passed=passed_waypoint(p[:2],coordinates[nodes[i]],goal,min(self.config['safety']['max_path_deviation_m'],self.route_manager.data['corridor_center_allowance_m']))
            if straight and self.motion_grid.contains(p[:2]) and (distance<route['waypoint_tolerance_m'] or passed):
                self.finish_edge(state) # Ordered edge event, no stop or cycle completion.
            else: break
        final=i==len(nodes)-2; corner=not final and not straight
        terminal=self.tracking_end_index; stop_goal=coordinates[nodes[terminal+1]]
        self.distance_to_turn_m=remaining_length(p[:2],self.local.path)
        requested=min(self.config['local']['max_speed_m_s'],route['speed_limit_m_s'],self.speed_limit,self.lane_speed_cap_m_s)
        self.active_speed_cap_m_s=speed_cap(requested,self.distance_to_turn_m,self.config['safety'],self.braking)
        self.braking_speed_cap_m_s=self.active_speed_cap_m_s; self.terminal_recovery_step=False
        self.local.config=dict(self.config['local'])
        self.local.config['max_speed_m_s']=self.active_speed_cap_m_s
        self.local.config['lookahead_m']=max(self.config['local']['lookahead_m'],self.active_speed_cap_m_s/.7+.05)
        self.local.config['max_yaw_rate']=route['turn_speed_limit_rad_s']; self.local.config['position_tolerance_m']=route['waypoint_tolerance_m']
        target_yaw=heading(stop_goal,coordinates[nodes[terminal+2]]) if terminal<len(nodes)-2 else 0.
        target_yaw=route.get('heading_at_turns_rad',{}).get(nodes[terminal+1],target_yaw)
        # This policy disables gait below command norm 0.1. Braking can reach
        # that deadband just outside position tolerance. Start the real corner
        # turn/position-hold phase once stopped nearby, rather than waiting for
        # an inactive forward command. Arrival still needs the original strict
        # position/yaw/velocity/settling conditions; no checkpoint is skipped.
        if self.active_speed_cap_m_s<.1 and math.dist(p[:2],stop_goal)<.25 and math.hypot(*velocity)<self.config['local']['settle_speed_m_s']:
            self.local.position_reached=True
        desired,arrived,_=self.local.command(state,[*stop_goal,target_yaw],dt); self.last_distance=distance
        pose_settled=arrived
        if self.local.position_reached:
            if distance>route['waypoint_tolerance_m']: self.position_recovery=True
            if distance<route['waypoint_tolerance_m']*.65: self.position_recovery=False
            if arrived: self.position_recovery=False
            if self.position_recovery:
                # The verified policy gait is inactive below command norm 0.1.
                # Use a real bounded corrective step, not an ineffectual 0.05
                # velocity forever just outside tolerance. Safety still vets it.
                dx,dy=goal[0]-p[0],goal[1]-p[1]
                desired[:2]=[.12*(math.cos(p[2])*dx+math.sin(p[2])*dy)/distance,.12*(-math.sin(p[2])*dx+math.cos(p[2])*dy)/distance]
                arrived=False
                if distance<.25 and self.active_speed_cap_m_s<min(requested,.12):
                    # The waypoint stopping envelope is a tracking constraint,
                    # not an obstacle. A sub-gait command cannot close the last
                    # centimetres. Permit one bounded corrective gait near the
                    # stopped corner only after the full obstacle/map stopping
                    # sweep accepts it. Never relax collision/human margins.
                    accepted=None
                    for step in (.12,.1):
                        recovery=bound_policy_command(desired,self.selector.low,self.selector.high,min(requested,step))
                        blocked,reason=self.safety.check(state,recovery,self.motion_grid,self.detector.obstacles,self.sim_time-self.detector.timestamp,velocity)
                        if not blocked: accepted=(recovery,min(requested,step)); break
                    if accepted is None and blocked!='FAULT' and distance<route['waypoint_tolerance_m']:
                        # Already inside the real waypoint tolerance: do not
                        # demand an unsafe extra centring step before turning.
                        # Rotation still goes through the normal safety check.
                        self.position_recovery=False; desired[:2]=[0.,0.]; arrived=pose_settled
                    elif accepted is None:
                        if blocked=='FAULT': self.fail(reason)
                        else: self.wait(reason)
                        return self.record(state,self.selector.stop())
                    else:
                        desired=accepted[0]; self.active_speed_cap_m_s=accepted[1]; self.terminal_recovery_step=True
        if arrived:
            finished=self.finish_edge(state)
            return self.record(state,self.selector.stop()) if finished else self.command(dt)
        self.deviation=min(segment_distance(p[:2],a,b) for a,b in zip(self.edge['path'],self.edge['path'][1:]))
        if self.now-self.movement_started>self.scheduler.config['movement_timeout_seconds']: self.fail('Movement timeout'); return self.record(state,self.selector.stop())
        if self.deviation>self.config['safety']['max_path_deviation_m']: self.fail('Route path deviation'); return self.record(state,self.selector.stop())
        if distance<self.best_distance-.04 or abs(wrap(p[2]-self.progress_yaw))>.08:
            self.best_distance=min(distance,self.best_distance); self.progress_time=self.now; self.progress_yaw=p[2]
        if self.now-self.progress_time>self.config['safety']['no_progress_seconds']: self.fail('No progress on mandatory segment'); return self.record(state,self.selector.stop())
        if not self.local.position_reached and not self.local.turning and math.hypot(*desired[:2])>.1:
            a=coordinates[nodes[i]]; b=coordinates[nodes[i+1]]; length=math.dist(a,b)
            ux,uy=(b[0]-a[0])/length,(b[1]-a[1])/length; nx,ny=-uy,ux
            error=(p[0]-a[0])*nx+(p[1]-a[1])*ny
            if self.distance_to_turn_m>1.2:
                # Do not accelerate out of a corner while close to a narrow
                # lane boundary. Re-centre with real holonomic feedback first.
                wx=math.cos(p[2])*desired[0]-math.sin(p[2])*desired[1]
                wy=math.sin(p[2])*desired[0]+math.cos(p[2])*desired[1]
                along=max(0.,wx*ux+wy*uy)
                if abs(error)>.04: along=min(along,.22)
                cross=max(-.15,min(.15,-1.2*error-.5*(velocity[0]*nx+velocity[1]*ny)))
                wx,wy=along*ux+cross*nx,along*uy+cross*ny
                desired[:2]=[math.cos(p[2])*wx+math.sin(p[2])*wy,-math.sin(p[2])*wx+math.cos(p[2])*wy]
        desired=bound_policy_command(desired,self.selector.low,self.selector.high,self.active_speed_cap_m_s)
        self.clearance_speed_cap_m_s=math.hypot(*desired[:2])
        if self.clearance_speed_cap_m_s>.12:
            # Reduce the requested future velocity before it would force a
            # hard safety stop. Actual feedback remains checked after ramping.
            original=list(desired); length=self.clearance_speed_cap_m_s; chosen=None
            for cap in (length,.75*length,.5*length,.25*length,.12):
                if cap<.12: continue
                candidate=[original[0]*cap/length,original[1]*cap/length,original[2]]
                future=[math.cos(p[2])*candidate[0]-math.sin(p[2])*candidate[1],math.sin(p[2])*candidate[0]+math.cos(p[2])*candidate[1]]
                blocked,probe_reason=self.safety.check(state,candidate,self.motion_grid,self.detector.obstacles,self.sim_time-self.detector.timestamp,future)
                if not blocked: chosen=candidate; break
            if chosen is None:
                self.wait(probe_reason); return self.record(state,self.selector.stop())
            desired=chosen; self.clearance_speed_cap_m_s=math.hypot(*desired[:2])
        command=self.selector.auto(desired,dt,preserve_linear_direction=True)
        command=bound_policy_command(command,self.selector.low,self.selector.high,self.active_speed_cap_m_s)
        self.selector.previous=list(command)
        safety,reason=self.safety.check(state,command,self.motion_grid,self.detector.obstacles,self.sim_time-self.detector.timestamp,velocity)
        if safety:
            if safety=='FAULT': self.fail(reason)
            else: self.wait(reason)
            return self.record(state,self.selector.stop())
        self.blocked_since=None
        phase='TURNING' if self.local.position_reached and corner else ('APPROACHING_TURN' if corner and distance<.6 else ('MOVING_TO_EXIT' if final else 'MOVING_SEGMENT'))
        if self.scheduler.state!=phase: self.scheduler.transition(phase,self.now)
        self.reason='Following '+self.edge['id']+(' | narrow lane cap 0.22 m/s' if self.lane_speed_cap_m_s==.22 else ''); return self.record(state,command)
    def snapshot(self):
        value=super().snapshot()
        if not hasattr(self,'route_manager'): return value
        rid=self.active_route or self.selected_route(); status=dict(self.route_manager.status.get(rid,{'status':'UNVALIDATED'}))
        if status.get('length_m'):
            speed=min(self.config['local']['max_speed_m_s'],self.route_manager.data['routes'][rid]['speed_limit_m_s'])
            status['estimated_travel_seconds']=status['length_m']/speed
        value.update(route_mode=self.route_mode,selected_route=rid,next_route=self.selected_route(self.scheduler.completed+1),route_validation=status,current_segment=self.edge['id'] if self.edge else None,current_waypoint=self.edge['end_node'] if self.edge else None,route_history=self.route_complete_history,route_length_m=self.route_plan['length_m'] if self.route_plan else None,
                     active_speed_cap_m_s=getattr(self,'active_speed_cap_m_s',None),braking_speed_cap_m_s=getattr(self,'braking_speed_cap_m_s',None),terminal_recovery_step=getattr(self,'terminal_recovery_step',False),clearance_speed_cap_m_s=getattr(self,'clearance_speed_cap_m_s',None),lane_speed_cap_m_s=getattr(self,'lane_speed_cap_m_s',None),lane_lateral_clearance_m=getattr(self,'lane_lateral_clearance_m',None),distance_to_turn_m=getattr(self,'distance_to_turn_m',None),unverified_user_override=self.speed_profile.get('unverified_user_override',False))
        return value
    def reset_route_configuration(self):
        if self.scheduler.state not in ('IDLE','STOPPED'): raise ValueError('STOP before resetting route settings')
        self.select_route('route_1_blue')
        self.route_manager=RouteManager(self.runtime.factory_cfg)
        settings=json.loads((ROOT/'configs/patrol_route_settings.json').read_text())
        self.scheduler=RouteScheduler(settings['schedule']); self.event_index=0
        self.route_mode=self.route_manager.route_id(settings['route_mode'])
        self.reason='Route defaults restored; validate before START'
    def on_robot_reset(self):
        self.edge=None; self.active_route=None; self.route_plan=None; self.path=[]; self.segment_index=0
        self.return_only=False; self.blocked_since=None; self.position_recovery=False; self.local.reset([]); self.velocity_history.clear()
        self.scheduler.goal='ENTRY'; self.reason='Robot reset to ENTRY; route cancelled, Manual takeover'
    def save(self,path=None):
        payload={'version':1,'factory_id':'v2','source_factory_sha256':self.runtime.factory_cfg['source_factory_sha256'],'route_mode':self.route_mode,'schedule':self.scheduler.config,'routes':self.route_manager.data,'navigation':self.config}
        path=Path(path) if path else ROOT/'configs/patrol_route_user.json'
        if not path.resolve().is_relative_to(ROOT): raise ValueError('Save must stay within project')
        if path.exists(): (self.directory/'route_config_previous.json').write_bytes(path.read_bytes())
        path.write_text(json.dumps(payload,indent=2)); return path
    def load(self,path=None):
        from navigation.patrol_scheduler import validate_schedule
        if self.mode!='MANUAL' or self.scheduler.state not in ('IDLE','STOPPED'): raise ValueError('STOP and Manual before loading')
        path=Path(path) if path else ROOT/'configs/patrol_route_user.json'
        if not path.resolve().is_relative_to(ROOT): raise ValueError('Route config must stay under project root')
        payload=json.loads(path.read_text()); validate_schedule(payload['schedule'])
        manager=RouteManager(self.runtime.factory_cfg,payload['routes'])
        if payload.get('factory_id')!='v2' or payload.get('source_factory_sha256')!=self.runtime.factory_cfg['source_factory_sha256']: raise ValueError('Wrong factory route file')
        config=payload.get('navigation',self.config); validate_navigation(config,self.speed_limit)
        if config['safety']['reaction_latency_seconds']<.65: raise ValueError('Multi-route stride filter requires reaction latency >= 0.65 s')
        for rid in manager.data['routes']: manager.validate(self.grid,rid)
        manager.route_id(payload['route_mode'])
        self.route_manager=manager; self.select_route(payload['route_mode']); self.scheduler=RouteScheduler(payload['schedule']); self.event_index=0
        self.config=config; self.local=LocalPlanner(config['local']); self.safety.config=config['safety']
