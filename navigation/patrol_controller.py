"""Closed-loop patrol mission. Outputs velocities only; no joint/pose writes."""
import json,math,time
from collections import deque
from pathlib import Path
from navigation.lane_map import LaneMap,box_distance,segment_distance
from navigation.occupancy_grid import OccupancyGrid
from navigation.global_planner import GlobalPlanner,NoValidPath
from navigation.local_planner import LocalPlanner,pose
from navigation.patrol_scheduler import PatrolScheduler,validate_schedule
from navigation.safety_supervisor import SafetySupervisor
from navigation.command_adapter import CommandSelector
from navigation.obstacle_detector import SceneObstacleDetector
ROOT=Path(r'D:\unitree_g1')


def validate_navigation(config):
    for group in ['local','safety']:
        if not all(isinstance(v,(int,float)) and math.isfinite(v) and v>0 for v in config[group].values()):
            raise ValueError('Navigation limits must be finite and positive')
    if not .12<=config['local']['max_speed_m_s']<=.35 or config['local']['max_yaw_rate']>.6:
        raise ValueError('Navigation velocity outside tested limits')
    if config['safety']['reaction_latency_seconds']<config['obstacle_update_seconds']+.2:
        raise ValueError('Reaction latency must include obstacle scan and velocity-filter delay')
    if not .05<=config['obstacle_update_seconds']<=.25 or not .1<=config['replan_seconds']<=2:
        raise ValueError('Invalid navigation update interval')


class PatrolController:
    def __init__(self,runtime):
        self.runtime=runtime
        def load(name): return json.loads((ROOT/'configs'/name).read_text(encoding='utf-8-sig'))
        self.config=load('navigation.json'); self.map_config=load('navigation_map.json')
        validate_navigation(self.config)
        self.scheduler=PatrolScheduler(load('patrol_schedule.json'))
        braking_path=ROOT/'configs/navigation_braking.json'
        self.braking=json.loads(braking_path.read_text()) if braking_path.is_file() else {'validated':False}
        self.safety=SafetySupervisor(self.config['safety'],self.braking)
        self.selector=CommandSelector(runtime.keys,runtime.policy.command_low,runtime.policy.command_high)
        x,y,heading=pose(runtime.last_state)
        self.a=self.config['point_a'] or [x,y,heading]; self.b=self.config['point_b']
        self.detector=SceneObstacleDetector(runtime.stage,self.map_config)
        self.path=[]; self.grid=None; self.last_map_signature=None; self.last_scan=-1e9
        self.local=LocalPlanner(self.config['local']); self.last_replan=-1e9; self.now=0
        self.sim_time=0; self.wall_origin=time.perf_counter(); self.last_running=True
        self.last_stop_generation=runtime.keys.stop_generation
        self.reason='Select B and Validate Route before START'; self.last_command=[0.,0.,0.]
        self.last_distance=None; self.deviation=0.; self.planner_latency_ms=0; self.map_latency_ms=0
        self.progress_time=0; self.best_distance=float('inf'); self.cycle_start=None
        self.transitions=[]; self.previous_status=None; self.last_log=-1e9; self.last_flush=-1e9; self.last_status_clock=0; self.blocked_since=None
        self.event_index=0; self.return_only=False; self.cycle_durations=[]
        self.active_leg=None
        self.velocity_history=deque(maxlen=15)
        self.directory=ROOT/'logs/patrol_runs'/runtime.directory.name; self.directory.mkdir(parents=True,exist_ok=True)
        self.log=(self.directory/'telemetry.jsonl').open('w',encoding='utf-8')
        self.refresh_obstacles(force=True)
        (self.directory/'scene_map.json').write_text(json.dumps({'floor':self.detector.floor,'obstacles':self.detector.obstacles,'config':self.map_config},indent=2))

    @property
    def mode(self): return self.selector.mode
    def fail(self,reason):
        self.scheduler.transition('FAULT',self.now,reason); self.reason=reason; self.selector.stop(); self.path=[]
    def before_tick(self,running):
        self.sim_time=self.runtime.steps/self.runtime.loco_cfg['control_hz']
        self.now=self.sim_time if self.scheduler.config['clock_mode']=='simulation' else time.perf_counter()-self.wall_origin
        if self.mode=='AUTO PATROL' and not running and self.last_running: self.pause('Simulation paused; explicit patrol Resume required')
        self.last_running=running
        if self.runtime.fault and self.mode=='AUTO PATROL': self.fail(self.runtime.fault)
    def refresh_obstacles(self,force=False):
        if not force and self.sim_time-self.last_scan<self.config['obstacle_update_seconds']: return False
        begin=time.perf_counter(); self.detector.update(self.sim_time); self.last_scan=self.sim_time
        if self.detector.floor is None: raise ValueError('Factory floor bounds unavailable: navigation disabled')
        signature=tuple((o['id'],tuple(round(v,2) for b in o['bounds'] for v in b)) for o in self.detector.obstacles)
        changed=signature!=self.last_map_signature
        if changed and hasattr(self,'log'):
            previous={o['id']:o for o in getattr(self,'previous_obstacles',[])}
            updates=[o for o in self.detector.obstacles if o['id'] not in previous or o['bounds']!=previous[o['id']]['bounds']]
            current={o['id'] for o in self.detector.obstacles}
            self.log.write(json.dumps({'scene_update':{'sim_time':self.sim_time,'source':'SIM GROUND TRUTH',
                'objects':updates,'removed':[key for key in previous if key not in current]}})+'\n')
            self.previous_obstacles=list(self.detector.obstacles)
        if changed or self.grid is None:
            lane=LaneMap(self.map_config,self.detector.obstacles,self.detector.floor)
            self.grid=OccupancyGrid(lane); self.planner=GlobalPlanner(self.grid); self.last_map_signature=signature
        self.map_latency_ms=(time.perf_counter()-begin)*1000
        return changed
    def validate_points(self,a=None,b=None):
        a=self.a if a is None else a; b=self.b if b is None else b
        if a is None or b is None: raise ValueError('Set both A and B; B is not preselected')
        for p in [a,b]:
            if len(p)!=3 or not all(math.isfinite(v) for v in p[:2]) or (p[2] is not None and not math.isfinite(p[2])):
                raise ValueError('Invalid waypoint coordinates/heading')
        self.refresh_obstacles(force=True)
        path=self.planner.plan(a[:2],b[:2]); self.planner.plan(b[:2],a[:2])
        self.path=path; self.reason=f'Route valid, {sum(math.dist(x,y) for x,y in zip(path,path[1:])):.2f} m; SIM GROUND TRUTH'
        return path
    def set_point(self,key,value):
        if self.mode!='MANUAL' and self.scheduler.state not in ('STOPPED','IDLE'): raise ValueError('STOP patrol before editing waypoints')
        if not self.grid.point_free(value[:2]): raise NoValidPath('NO_VALID_PATH: waypoint footprint is blocked/outside lane')
        a=value if key=='A' else self.a; b=value if key=='B' else self.b
        if a is not None and b is not None: self.validate_points(a,b)
        if key=='A': self.a=list(value)
        else: self.b=list(value)
        self.path=[]
    def set_mode(self,mode):
        if self.scheduler.state=='FAULT': raise ValueError('FAULT latched: acknowledge before changing mode')
        if mode==self.mode: return
        self.stop('Explicit control mode change')
        self.selector.select(mode)
    def start(self):
        if self.scheduler.state=='FAULT': raise ValueError('FAULT latched: acknowledge on UI before START')
        if not self.runtime.timeline.is_playing(): raise ValueError('Resume simulation before starting patrol')
        if math.hypot(*self.runtime.last_state['base_linear_velocity_world_m_s'][:2])>.12: raise ValueError('Stop robot and wait before enabling AUTO')
        self.safety.stop_distance(0) # Requires measured calibration, never invents a braking constant.
        if self.braking.get('checkpoint_sha256')!=self.runtime.loco_cfg['checkpoint_sha256']:
            raise ValueError('Re-run braking calibration for the active policy checkpoint')
        speed=self.config['local']['max_speed_m_s']; yaw=self.config['local']['max_yaw_rate']
        if speed>min(self.selector.high[0],-self.selector.low[0],self.selector.high[1],-self.selector.low[1]) or yaw>min(self.selector.high[2],-self.selector.low[2]):
            raise ValueError('Navigation limits exceed active policy command ranges')
        self.validate_points()
        if math.dist(pose(self.runtime.last_state)[:2],self.a[:2])>self.config['local']['position_tolerance_m']:
            raise ValueError('Robot must be at A; use Set A from Robot, or RETURN TO A')
        self.selector.select('AUTO PATROL'); self.scheduler.start(self.now); self.reason='Waiting start delay'
        self.last_stop_generation=self.runtime.keys.stop_generation
        self.blocked_since=None
        self.cycle_start=None; self.best_distance=float('inf')
        self.return_only=False
        self.active_leg=None; self.velocity_history.clear()
    def stop(self,reason='STOP PATROL'):
        if self.scheduler.state=='FAULT':
            self.selector.stop(); self.path=[]; self.reason='FAULT latched: acknowledge before restart'; return
        self.scheduler.transition('STOPPED',self.now,reason); self.selector.stop(); self.path=[]; self.reason=reason
    def pause(self,reason='Patrol paused'):
        self.scheduler.pause(self.now); self.selector.stop(); self.reason=reason
    def resume(self):
        if not self.runtime.timeline.is_playing(): raise ValueError('Resume simulation first')
        self.validate_points(); self.scheduler.resume(self.now); self.selector.stop()
        self.last_stop_generation=self.runtime.keys.stop_generation; self.blocked_since=None
        self.active_leg=None
    def acknowledge_fault(self):
        if self.scheduler.state!='FAULT': return
        self.selector.select('MANUAL'); self.scheduler.transition('STOPPED',self.now,'FAULT acknowledged; start manually')
    def return_to_a(self):
        if self.scheduler.state=='FAULT': raise ValueError('Acknowledge FAULT first')
        if self.scheduler.state not in ('IDLE','STOPPED','PAUSED') or math.hypot(*self.runtime.last_state['base_linear_velocity_world_m_s'][:2])>.12:
            raise ValueError('STOP and settle before RETURN TO A')
        if not self.runtime.timeline.is_playing(): raise ValueError('Simulation paused')
        self.safety.stop_distance(0)
        if self.braking.get('checkpoint_sha256')!=self.runtime.loco_cfg['checkpoint_sha256']:
            raise ValueError('Re-run braking calibration for the active checkpoint')
        self.refresh_obstacles(force=True); self.planner.plan(pose(self.runtime.last_state)[:2],self.a[:2])
        self.selector.select('AUTO PATROL'); self.scheduler.goal='A'
        self.return_only=True; self.last_stop_generation=self.runtime.keys.stop_generation
        self.active_leg=None
        self.scheduler.transition('REPLANNING',self.now,'Explicit RETURN TO A')
    def plan_current(self,state):
        goal=self.b if self.scheduler.goal=='B' else self.a
        begin=time.perf_counter(); self.path=self.planner.plan(pose(state)[:2],goal[:2])
        leg=(self.scheduler.completed,self.scheduler.goal,self.return_only)
        hold=self.active_leg==leg and self.local.position_reached and math.dist(pose(state)[:2],goal[:2])<.5
        self.local.reset(self.path)
        self.local.position_reached=hold
        self.planner_latency_ms=(time.perf_counter()-begin)*1000
        self.scheduler.transition('MOVING_TO_'+self.scheduler.goal,self.now)
        if self.active_leg!=leg:
            self.movement_started=self.now; self.progress_time=self.now
            self.best_distance=math.dist(pose(state)[:2],goal[:2]); self.progress_yaw=pose(state)[2]
            self.active_leg=leg
        self.last_replan=self.sim_time
        if self.scheduler.goal=='B' and self.cycle_start is None: self.cycle_start=self.now
    def command(self,dt):
        if self.mode=='MANUAL': return self.record(self.runtime.last_state,self.runtime.keys.tick(dt))
        if self.runtime.keys.stop_generation!=self.last_stop_generation:
            self.last_stop_generation=self.runtime.keys.stop_generation; self.pause('SPACE pauses Auto; explicit Resume required')
        self.runtime.keys.command=[0.,0.,0.] # WASD never contributes to AUTO command.
        state=self.runtime.policy.feedback.read_feedback()
        self.velocity_history.append(state['base_linear_velocity_world_m_s'][:2])
        prediction_velocity=[sum(row[i] for row in self.velocity_history)/len(self.velocity_history) for i in [0,1]]
        self.refresh_obstacles()
        self.scheduler.update(self.now); status=self.scheduler.state
        if not self.braking.get('validated'):
            if status not in ('IDLE','STOPPED','PAUSED','FAULT'): self.fail('Measured braking calibration required')
            return self.record(state,self.selector.stop())
        early,_reason=self.safety.check(state,[0,0,0],self.grid,self.detector.obstacles,self.sim_time-self.detector.timestamp,prediction_velocity)
        if early=='FAULT': self.fail(_reason); return self.record(state,self.selector.stop())
        if status in ('IDLE','WAITING_START','WAITING_AT_B','CYCLE_COMPLETE','PAUSED','STOPPED','FAULT'):
            return self.record(state,self.selector.stop())
        if status=='WAITING_FOR_CLEAR_PATH':
            if self.now-self.wait_started>self.scheduler.config['obstacle_wait_timeout_seconds']:
                self.fail('Obstacle waiting timeout'); return self.record(state,self.selector.stop())
            if self.sim_time-self.last_replan<self.config['replan_seconds']: return self.record(state,self.selector.stop())
            self.last_replan=self.sim_time
        if status in ('PLANNING','REPLANNING','WAITING_FOR_CLEAR_PATH'):
            try: self.plan_current(state)
            except NoValidPath as error:
                self.wait(str(error)); return self.record(state,self.selector.stop())
        goal=self.b if self.scheduler.goal=='B' else self.a
        if self.now-self.movement_started>self.scheduler.config['movement_timeout_seconds']:
            self.fail('Movement timeout'); return self.record(state,self.selector.stop())
        p=pose(state)[:2]; self.deviation=min(segment_distance(p,a,b) for a,b in zip(self.path,self.path[1:]))
        if self.deviation>self.config['safety']['max_path_deviation_m']:
            self.fail('Path deviation exceeded'); return self.record(state,self.selector.stop())
        desired,arrived,self.last_distance=self.local.command(state,goal,dt)
        if arrived:
            if self.return_only:
                self.scheduler.transition('STOPPED',self.now,'RETURN TO A settled'); self.return_only=False
            else:
                self.scheduler.arrived(self.now)
                if self.scheduler.goal=='A' and self.cycle_start is not None: self.cycle_durations.append(self.now-self.cycle_start)
            self.reason='Goal settled'
            if self.scheduler.goal=='A': self.cycle_start=None
            return self.record(state,self.selector.stop())
        if self.last_distance<self.best_distance-.05:
            self.best_distance=self.last_distance; self.progress_time=self.now
        if self.now-self.progress_time>self.config['safety']['no_progress_seconds']:
            # Rotating in place also counts as progress; monitor heading while translation is stopped.
            yaw=pose(state)[2]
            if abs(yaw-getattr(self,'progress_yaw',yaw))<.08:
                self.fail('No progress to waypoint'); return self.record(state,self.selector.stop())
            self.progress_time=self.now; self.progress_yaw=yaw
        command=self.selector.auto(desired,dt)
        status,reason=self.safety.check(state,command,self.grid,self.detector.obstacles,self.sim_time-self.detector.timestamp,prediction_velocity)
        if status:
            if status=='FAULT': self.fail(reason)
            else: self.wait(reason)
            return self.record(state,self.selector.stop())
        self.blocked_since=None
        self.reason='Following collision-checked path'; return self.record(state,command)
    def wait(self,reason):
        if self.blocked_since is None: self.blocked_since=self.now
        self.wait_started=self.blocked_since
        self.scheduler.transition('WAITING_FOR_CLEAR_PATH',self.now,reason); self.reason=reason
    def snapshot(self):
        state=self.runtime.last_state; point=pose(state)[:2]
        return {'mode':self.mode,'state':self.scheduler.state,'backend':'Locomotion RL','source':self.detector.mode,
            'goal':self.scheduler.goal,'point_a':self.a,'point_b':self.b,'position':state['base_position_m'],
            'heading_rad':pose(state)[2],'command':self.last_command,'actual_velocity':state['base_linear_velocity_world_m_s'],
            'distance_to_goal':self.last_distance,'path_deviation_m':self.deviation,'active_waypoint':self.local.index,
            'arrival_position_hold':self.local.position_reached,
            'path_length_m':sum(math.dist(a,b) for a,b in zip(self.path,self.path[1:])),
            'completed_cycles':self.scheduler.completed,'cycle_index':self.scheduler.completed+1,
            'max_cycles':self.scheduler.config['max_cycles'],'repeat_enabled':self.scheduler.config['repeat_enabled'],
            'repeat_interval_seconds':self.scheduler.config['repeat_interval_seconds'],
            'max_speed_m_s':self.config['local']['max_speed_m_s'],
            'cycle_durations_seconds':list(self.cycle_durations),
            'next_cycle_seconds':max(0,self.scheduler.deadline-self.now) if self.scheduler.state in ('WAITING_START','CYCLE_COMPLETE','WAITING_AT_B') else None,
            'nearest_obstacle_m':min((box_distance(point,o['bounds'])-self.grid.map.radius for o in self.detector.obstacles),default=None),
            'humans_in_scene':sum(o['kind']=='human' for o in self.detector.obstacles),'reason':self.reason,
            'human_count':len({o.get('human_id',o['id']) for o in self.detector.obstacles if o['kind']=='human'}),
            'human_within_1_5m':any(o['kind']=='human' and box_distance(point,o['bounds'])<1.5 for o in self.detector.obstacles),
            'clock_mode':self.scheduler.config['clock_mode'],'clock_seconds':self.now,'sim_time':self.sim_time,
            'planner_latency_ms':self.planner_latency_ms,'map_update_ms':self.map_latency_ms,
            'loop_fps':self.runtime.fps,'policy_inference_ms':getattr(self.runtime,'last_inference_ms',None),
            'gpu_memory':self.runtime.gpu_samples[-1] if self.runtime.gpu_samples else None}
    def record(self,state,command):
        self.last_command=list(command)
        for event in self.scheduler.events[self.event_index:]:
            self.transitions.append(dict(event)); self.log.write(json.dumps({'transition':event})+'\n'); self.log.flush()
        self.event_index=len(self.scheduler.events)
        if self.sim_time-self.last_log>=.1:
            self.last_log=self.sim_time; self.log.write(json.dumps(self.snapshot())+'\n')
            if self.sim_time-self.last_flush>=1:
                self.log.flush(); self.last_flush=self.sim_time
        return self.last_command
    def save(self,path=None):
        payload={'version':1,'point_a':self.a,'point_b':self.b,'navigation':self.config,'map':self.map_config,'schedule':self.scheduler.config}
        path=Path(path) if path is not None else ROOT/'configs/patrol_user.json'
        if not path.resolve().is_relative_to(ROOT): raise ValueError('Configuration must stay under project root')
        if path.exists(): (self.directory/'patrol_user_previous.json').write_bytes(path.read_bytes())
        tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(payload,indent=2)); tmp.replace(path)
        return path
    def load(self,path=None):
        if self.mode!='MANUAL' or self.scheduler.state=='FAULT': raise ValueError('Switch to MANUAL before loading configuration')
        path=Path(path) if path is not None else ROOT/'configs/patrol_user.json'
        if not path.resolve().is_relative_to(ROOT): raise ValueError('Configuration must stay under project root')
        payload=json.loads(path.read_text())
        validate_schedule(payload['schedule'])
        config=payload['navigation']; lane=LaneMap(payload['map'],self.detector.obstacles,self.detector.floor); grid=OccupancyGrid(lane)
        validate_navigation(config)
        a,b=payload['point_a'],payload['point_b']
        if a is None or b is None: raise ValueError('Saved waypoints incomplete')
        GlobalPlanner(grid).plan(a[:2],b[:2]); GlobalPlanner(grid).plan(b[:2],a[:2])
        self.config=config; self.map_config=payload['map']; self.detector.config=self.map_config
        self.grid=grid; self.planner=GlobalPlanner(grid); self.last_map_signature=None
        self.scheduler=PatrolScheduler(payload['schedule']); self.event_index=0; self.local=LocalPlanner(config['local']); self.safety=SafetySupervisor(config['safety'],self.braking)
        self.a=a; self.b=b; self.reason='Configuration loaded and route validated'
    def close(self):
        self.selector.stop()
        try: self.record(self.runtime.last_state,[0.,0.,0.])
        finally: self.log.close()
