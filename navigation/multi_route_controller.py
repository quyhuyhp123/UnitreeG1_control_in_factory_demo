"""Single velocity-command source for mandatory, ordered V2 route segments."""
import json,math,time
from collections import deque
from pathlib import Path
from navigation.patrol_controller import PatrolController,ROOT,validate_navigation
from navigation.route_manager import RouteManager
from navigation.route_scheduler import RouteScheduler
from navigation.route_graph import heading,is_corner
from navigation.local_planner import LocalPlanner,pose,wrap
from navigation.global_planner import NoValidPath
from navigation.lane_map import segment_distance
from navigation.speed_profile import load_speed_profile
from navigation.dynamic_local_planner import DynamicLocalPlanner

TURN_SLOWDOWN_START_M=2.0
TURN_SLOWDOWN_END_M=1.2

class MultiRouteController(PatrolController):
    def __init__(self,runtime):
        super().__init__(runtime)
        self.route_manager=RouteManager(runtime.factory_cfg)
        settings=json.loads((ROOT/'configs/patrol_route_settings.json').read_text())
        if settings.get('version')!=1 or settings.get('factory_id')!='v2': raise ValueError('Route settings factory/version mismatch')
        if settings.get('source_factory_sha256')!=runtime.factory_cfg['source_factory_sha256'] or settings.get('factory_usd')!=runtime.factory_cfg['factory_usd']: raise ValueError('Route settings/source identity mismatch')
        self.scheduler=RouteScheduler(settings['schedule']); self.event_index=0
        self.route_mode=settings['route_mode']; self.alternate_first=settings['alternate_first']
        self.active_route=None; self.segment_index=0; self.edge=None; self.route_plan=None; self.route_complete_history=[]
        self.position_recovery=False
        self.velocity_history=deque(maxlen=30)
        self.config['safety']['reaction_latency_seconds']=.65
        self.safety.swept=True
        self.speed_profile=load_speed_profile(); self.speed_limit=self.speed_profile['maximum_verified_m_s']
        self.safety.simulation_braking_bypass=self.speed_profile.get('simulation_braking_bypass') is True
        self.config['local']['max_speed_m_s']=self.speed_profile['default_m_s']
        self.config['safety']['max_actual_speed_m_s']=self.speed_profile['max_actual_speed_m_s']
        self.dynamic_enabled=runtime.factory_cfg.get('dynamic_avoidance',False)
        self.dynamic=DynamicLocalPlanner(self.safety)
        self.dynamic_wait=False
        self.reason='Select route, PREVIEW / VALIDATE, then START. No automatic start.'
    def set_dynamic_avoidance(self, enabled):
        if self.scheduler.state not in ('IDLE','STOPPED','PAUSED','WAITING_FOR_CLEAR_PATH'):
            raise ValueError('Pause or STOP patrol before changing avoidance')
        if math.hypot(*self.runtime.last_state['base_linear_velocity_world_m_s'][:2])>.10:
            raise ValueError('Wait for G1 to settle before changing avoidance')
        source=self.runtime.yolo
        if enabled and (source is None or source.costmap is None or source.worker.error):
            raise ValueError('Costmap unavailable. Launch with -DynamicAvoidance or -YoloDebug.')
        self.dynamic_enabled=enabled
        self.dynamic.telemetry={}
        self.selector.stop()
        self.reason='Dynamic avoidance '+('ON' if enabled else 'OFF')

    def envelope_speed(self,distance):
        """Largest speed whose supervisor look-ahead (prediction + reaction + braking) fits in distance."""
        s=self.config['safety']; a=self.braking['conservative_deceleration_m_s2']
        t=s['prediction_seconds']+s['reaction_latency_seconds']; room=distance-s['stopping_extra_m']
        return 0. if room<=0 else a*(-t+math.sqrt(t*t+2*room/a))

    def chair_bypass_command(self, p, cap):
        """Begin the Map 2 bypass before the centered approach enters inflation."""
        guide=self.map_config.get('map_2_chair_bypass')
        if (not self.dynamic_enabled or self.runtime.factory_cfg.get('map_id')!=2 or not guide or
                self.edge['id']!=guide['edge'] or p[1]<guide['approach_y_m']):
            return None
        if not any(o['id'].endswith('/shared_left_chair/Collider') for o in self.detector.obstacles):
            return None
        index=self.edge.get('chair_bypass_index',0)
        points=guide['points']
        if index<len(points) and math.dist(p[:2],points[index])<.09:
            index+=1
            self.edge['chair_bypass_index']=index
            self.progress_time=self.now
        if index==len(points):
            return None
        x,y=points[index]
        err=wrap(math.atan2(y-p[1],x-p[0])-p[2])
        yaw=max(-.30,min(.30,1.2*err))
        if abs(err)>.35:
            return [0.,0.,yaw]
        guide_cap=cap if self.safety.simulation_braking_bypass else min(cap,.22)
        speed=min(guide_cap,max(.12,math.dist(p[:2],points[index])*.7))
        return [speed*math.cos(err),speed*math.sin(err),yaw]
    def selected_route(self,completed=None):
        return self.route_manager.route_id(self.route_mode,self.scheduler.completed if completed is None else completed,self.alternate_first)
    def select_route(self,mode,first=None):
        if self.scheduler.state not in ('IDLE','STOPPED'): raise ValueError('STOP before changing route')
        if math.hypot(*self.runtime.last_state['base_linear_velocity_world_m_s'][:2])>.12: raise ValueError('Settle before route change')
        if (first or self.alternate_first) not in self.route_manager.data['routes']: raise ValueError('Unknown first alternate route')
        self.route_manager.route_id(mode,0,first or self.alternate_first)
        self.route_mode=mode; self.alternate_first=first or self.alternate_first; self.path=[]; self.edge=None; self.active_route=None; self.route_plan=None
    def validate_route(self):
        self.refresh_obstacles(force=True)
        ids=['route_1_blue','route_2_brown'] if self.route_mode=='alternate' else [self.selected_route()]
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
        self.dynamic_wait=False
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
        dense=[self.edge['path'][0]]
        for a,b in zip(self.edge['path'],self.edge['path'][1:]):
            count=max(1,math.ceil(math.dist(a,b)/.2))
            dense.extend([(a[0]+(b[0]-a[0])*j/count,a[1]+(b[1]-a[1])*j/count) for j in range(1,count+1)])
        nodes=self.route_manager.data['routes'][self.active_route]['ordered_nodes']; N=self.route_manager.data['nodes']; i=self.segment_index
        if i<len(nodes)-2 and not is_corner(N[nodes[i]],N[nodes[i+1]],N[nodes[i+2]]):
            # Pass-through node: keep look-ahead targets on the next collinear
            # segment so the robot does not brake to a crawl at the checkpoint.
            b,c=N[nodes[i+1]],N[nodes[i+2]]; length=math.dist(b,c); reach=min(length,4.); count=math.ceil(reach/.2)
            dense.extend([(b[0]+(c[0]-b[0])*reach*j/count/length,b[1]+(c[1]-b[1])*reach*j/count/length) for j in range(1,count+1)])
        self.local.reset(dense); self.path=self.route_plan['path']; self.movement_started=self.now
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
        perception=None
        if self.dynamic_enabled:
            source=self.runtime.yolo
            result=source.result if source is not None and not source.worker.error else None
            perception=self.dynamic.observation(result,source.perception_epoch if source else None,self.sim_time,time.time())
            if perception is None:
                return self.wait_for_perception(state,'stale','RGB-D costmap missing or stale')
            # Unlocalized RGB detections are debug output. Valid geometric
            # costmap cells and localized people still constrain movement.
        try:
            if status=='VALIDATING_ROUTE': self.begin_cycle(state)
            elif status in ('REPLANNING','WAITING_FOR_CLEAR_PATH'):
                if self.blocked_since is not None and self.now-self.blocked_since>self.scheduler.config['obstacle_wait_timeout_seconds']:
                    self.fail('PATH_BLOCKED timeout'); return self.record(state,self.selector.stop())
                if status=='WAITING_FOR_CLEAR_PATH' and not self.dynamic_wait and self.sim_time-self.last_replan<1.: return self.record(state,self.selector.stop())
                self.last_replan=self.sim_time
                # Preserve the original per-leg deadline during obstacle retries.
                if self.edge is None: self.begin_cycle(state)
                elif not self.dynamic_wait:
                    started=self.movement_started; self.begin_edge(state); self.movement_started=started
        except NoValidPath as exc:
            self.wait(str(exc)); return self.record(state,self.selector.stop())
        route=self.route_manager.data['routes'][self.active_route]
        nodes=route['ordered_nodes']; i=self.segment_index; goal=self.edge['goal']
        final=i==len(nodes)-2; corner=not final and is_corner(self.route_manager.data['nodes'][nodes[i]],goal,self.route_manager.data['nodes'][nodes[i+2]])
        distance=math.dist(p[:2],goal)
        through=not final and not corner; N=self.route_manager.data['nodes']
        # Distance to the next node where the robot must stop/turn (corner or
        # EXIT), measured through any collinear pass-through checkpoints.
        stop_distance=distance; chain=[]; j=i
        while j<len(nodes)-2 and not is_corner(N[nodes[j]],N[nodes[j+1]],N[nodes[j+2]]):
            chain.append((N[nodes[j+1]],N[nodes[j+2]])); stop_distance+=math.dist(*chain[-1]); j+=1
        self.edge['grid'].extra=chain
        self.edge['grid'].detour=(self.map_config.get('local_detours',{}).get(self.edge['id'])
                                  if self.dynamic_enabled else None)
        requested=min(self.config['local']['max_speed_m_s'],route['speed_limit_m_s'],self.speed_limit)
        self.local.config=dict(self.config['local'])
        cap=requested
        if requested>.22 and not self.safety.simulation_braking_bypass:
            # Slow down from TURN_SLOWDOWN_START_M before a turn/EXIT, reaching the
            # verified 0.22 m/s approach speed TURN_SLOWDOWN_END_M before it.
            if stop_distance<TURN_SLOWDOWN_START_M:
                cap=.22+(requested-.22)*max(0.,stop_distance-TURN_SLOWDOWN_END_M)/(TURN_SLOWDOWN_START_M-TURN_SLOWDOWN_END_M)
            # Never command a speed whose predicted stopping point lies beyond
            # the turn; otherwise the supervisor would stop the robot instead.
            cap=min(cap,max(.12,self.envelope_speed(stop_distance-.2)))
        self.local.config['max_speed_m_s']=cap
        self.local.config['slow_endpoint']=not self.safety.simulation_braking_bypass
        # The tracker commands 0.7 x look-ahead distance, so high speed needs a
        # proportionally longer look-ahead.
        self.local.config['lookahead_m']=max(self.config['local']['lookahead_m'],cap/.7+.05)
        self.local.config['max_yaw_rate']=route['turn_speed_limit_rad_s']; self.local.config['position_tolerance_m']=route['waypoint_tolerance_m']
        if through:
            a=N[nodes[i]]; length=math.dist(a,goal)
            along=((p[0]-goal[0])*(goal[0]-a[0])+(p[1]-goal[1])*(goal[1]-a[1]))/length
            passed=along>=0
            if distance<route['waypoint_tolerance_m'] or (not self.dynamic_enabled and passed and distance<self.config['safety']['max_path_deviation_m']):
                self.finish_edge(state); return self.command(dt)
        target_yaw=heading(goal,self.route_manager.data['nodes'][nodes[i+2]]) if corner else (0. if final else None)
        if corner: target_yaw=route.get('heading_at_turns_rad',{}).get(nodes[i+1],target_yaw)
        desired,arrived,self.last_distance=self.local.command(state,[*self.local.path[-1],None] if through else [*goal,target_yaw],dt)
        if through: arrived=False; self.last_distance=distance
        cross_track=math.sqrt(max(0.,distance*distance-along*along)) if through else 0.
        if self.dynamic_enabled and through and along>-.2 and (passed or cross_track>route['waypoint_tolerance_m']*.75):
            # Finish a detour at the mandatory node instead of following the
            # look-ahead target that already lies on the next straight edge.
            dx,dy=goal[0]-p[0],goal[1]-p[1]
            scale=min(cap,.12)/distance
            desired[:2]=[scale*(math.cos(p[2])*dx+math.sin(p[2])*dy),
                         scale*(-math.sin(p[2])*dx+math.cos(p[2])*dy)]
        lateral=min(self.selector.high[1],-self.selector.low[1])
        if abs(desired[1])>lateral: desired[:2]=[v*lateral/abs(desired[1]) for v in desired[:2]]
        if self.local.position_reached:
            if distance>route['waypoint_tolerance_m']: self.position_recovery=True
            if distance<route['waypoint_tolerance_m']*.65: self.position_recovery=False
            if self.position_recovery:
                # The verified policy gait is inactive below command norm 0.1.
                # Use a real bounded corrective step, not an ineffectual 0.05
                # velocity forever just outside tolerance. Safety still vets it.
                dx,dy=goal[0]-p[0],goal[1]-p[1]
                desired[:2]=[.12*(math.cos(p[2])*dx+math.sin(p[2])*dy)/distance,.12*(-math.sin(p[2])*dx+math.cos(p[2])*dy)/distance]
                arrived=False
        if arrived:
            finished=self.finish_edge(state)
            return self.record(state,self.selector.stop()) if finished else self.command(dt)
        self.deviation=min(segment_distance(p[:2],a,b) for a,b in zip(self.edge['path'],self.edge['path'][1:]))
        if self.now-self.movement_started>self.scheduler.config['movement_timeout_seconds']: self.fail('Movement timeout'); return self.record(state,self.selector.stop())
        if self.deviation>self.config['safety']['max_path_deviation_m'] and not self.edge['grid'].in_detour(p[:2]):
            self.fail('Route path deviation'); return self.record(state,self.selector.stop())
        if distance<self.best_distance-.04 or abs(wrap(p[2]-self.progress_yaw))>.08:
            self.best_distance=min(distance,self.best_distance); self.progress_time=self.now; self.progress_yaw=p[2]
        if self.now-self.progress_time>self.config['safety']['no_progress_seconds']: self.fail('No progress on mandatory segment'); return self.record(state,self.selector.stop())
        bypass=None
        if self.dynamic_enabled:
            corridor=self.edge['grid']; tolerance=route['waypoint_tolerance_m']
            bypass=self.chair_bypass_command(p,cap)
            if bypass is not None: desired=bypass
            if not self.dynamic.trace(perception,corridor,p,[0.,0.,0.],goal,tolerance,actual=velocity):
                return self.wait_for_perception(state,'braking','Current velocity has no clear braking sweep')
            people=[*self.detector.obstacles,*result.get('obstacles',[])]
            def command_blocker(candidate):
                filtered=self.selector.auto(candidate,dt,preserve_linear_direction=True,preview=True)
                if not self.dynamic.trace(perception,corridor,p,filtered,goal,tolerance):
                    return self.dynamic.trace_blocker
                status, reason=self.safety.check(state,filtered,corridor,self.detector.obstacles,
                                                 self.sim_time-self.detector.timestamp,velocity)
                return reason if status else None
            desired=self.dynamic.choose(perception,corridor,p,desired,self.selector.low,self.selector.high,cap,goal,tolerance,
                                        people,allow_detour=not self.local.position_reached,command_blocker=command_blocker)
            if desired is None:
                return self.wait_for_perception(state,self.dynamic.telemetry['status'],'No clear detour inside the current route lane')
        command=self.selector.auto(desired,dt,preserve_linear_direction=True)
        if self.dynamic_enabled and (math.hypot(*command[:2])>requested+1e-9 or
                                     not self.dynamic.trace(perception,corridor,p,command,goal,tolerance)):
            return self.wait_for_perception(state,'braking','Filtered command has no clear braking sweep')
        safety,reason=self.safety.check(state,command,self.edge['grid'],self.detector.obstacles,self.sim_time-self.detector.timestamp,velocity)
        if safety:
            if safety=='FAULT': self.fail(reason)
            elif self.dynamic_enabled: return self.wait_for_perception(state,'safety_wait',reason)
            else: self.wait(reason)
            return self.record(state,self.selector.stop())
        self.blocked_since=None
        self.dynamic_wait=False
        phase='TURNING' if self.local.position_reached and corner else ('APPROACHING_TURN' if corner and distance<.6 else ('MOVING_TO_EXIT' if final else 'MOVING_SEGMENT'))
        if self.scheduler.state!=phase: self.scheduler.transition(phase,self.now)
        self.reason=('Map 2 chair bypass' if bypass is not None else 'Following '+self.edge['id'])
        return self.record(state,command)
    def wait_for_perception(self,state,status,reason):
        self.dynamic.telemetry.update(status=status,predicted_end_xy=None,predicted_obstacle_cell_distance_m=None)
        self.dynamic_wait=True
        if self.blocked_since is not None and self.now-self.blocked_since>self.scheduler.config['obstacle_wait_timeout_seconds']:
            self.fail('PATH_BLOCKED timeout: '+reason)
        elif self.scheduler.state!='WAITING_FOR_CLEAR_PATH' or self.reason!=reason:
            self.wait(reason)
        self.progress_time=self.now
        return self.record(state,self.selector.stop())
    def snapshot(self):
        value=super().snapshot()
        if not hasattr(self,'route_manager'): return value
        rid=self.active_route or self.selected_route(); status=dict(self.route_manager.status.get(rid,{'status':'UNVALIDATED'}))
        if status.get('length_m'):
            speed=min(self.config['local']['max_speed_m_s'],self.route_manager.data['routes'][rid]['speed_limit_m_s'])
            status['estimated_travel_seconds']=status['length_m']/speed
        value.update(route_mode=self.route_mode,selected_route=rid,next_route=self.selected_route(self.scheduler.completed+1),route_validation=status,current_segment=self.edge['id'] if self.edge else None,current_waypoint=self.edge['end_node'] if self.edge else None,route_history=self.route_complete_history,route_length_m=self.route_plan['length_m'] if self.route_plan else None)
        value['dynamic_avoidance']={'enabled':self.dynamic_enabled,**self.dynamic.telemetry}
        return value
    def reset_route_configuration(self):
        if self.scheduler.state not in ('IDLE','STOPPED'): raise ValueError('STOP before resetting route settings')
        self.select_route('route_1_blue')
        self.route_manager=RouteManager(self.runtime.factory_cfg)
        settings=json.loads((ROOT/'configs/patrol_route_settings.json').read_text())
        self.scheduler=RouteScheduler(settings['schedule']); self.event_index=0
        self.route_mode=settings['route_mode']; self.alternate_first=settings['alternate_first']
        self.reason='Route defaults restored; validate before START'
    def on_robot_reset(self):
        self.edge=None; self.active_route=None; self.route_plan=None; self.path=[]; self.segment_index=0
        self.return_only=False; self.blocked_since=None; self.position_recovery=False; self.local.reset([]); self.velocity_history.clear()
        self.dynamic_wait=False
        if self.runtime.yolo is not None: self.runtime.yolo.invalidate_costmap()
        self.scheduler.goal='ENTRY'; self.reason='Robot reset to ENTRY; route cancelled, Manual takeover'
    def save(self,path=None):
        payload={'version':1,'factory_id':'v2','source_factory_sha256':self.runtime.factory_cfg['source_factory_sha256'],'route_mode':self.route_mode,'alternate_first':self.alternate_first,'schedule':self.scheduler.config,'routes':self.route_manager.data,'navigation':self.config}
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
        self.route_manager=manager; self.select_route(payload['route_mode'],payload['alternate_first']); self.scheduler=RouteScheduler(payload['schedule']); self.event_index=0
        self.config=config; self.local=LocalPlanner(config['local']); self.safety.config=config['safety']
