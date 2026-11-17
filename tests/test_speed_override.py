"""Logic-only regressions: no SimulationApp, robot, or network is started."""
import copy,json,math,unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from navigation.speed_profile import load_speed_profile,validate_speed_profile
from navigation.route_motion import straight_run_end,remaining_length,passed_waypoint,speed_cap,bound_policy_command,lateral_clearance
from navigation.route_graph import CorridorGrid
from navigation.local_planner import LocalPlanner
from navigation.lane_map import LaneMap
from navigation.occupancy_grid import OccupancyGrid
from navigation.safety_supervisor import SafetySupervisor,segment_box_distance
from navigation.multi_route_controller import MultiRouteController
from navigation.command_adapter import CommandSelector
from navigation.route_scheduler import RouteScheduler
from navigation.patrol_ui import PatrolPanel,speed_presets

ROOT=Path(__file__).resolve().parents[1]
def read(relative): return json.loads((ROOT/relative).read_text(encoding='utf-8-sig'))
def state(x=0.,y=0.,yaw=0.,velocity=(0.,0.,0.)):
    return {'base_position_m':[x,y,.78],'base_orientation_wxyz':[math.cos(yaw/2),0.,0.,math.sin(yaw/2)],
            'base_linear_velocity_world_m_s':list(velocity),'base_angular_velocity_world_rad_s':[0.,0.,0.]}


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile=read('configs/navigation_speed_v2.json'); self.cfg=read('configs/locomotion_mjlab.yaml'); self.factory=read('configs/factory_v2.json')
    def validate(self,p): return validate_speed_profile(p,self.cfg,self.factory,ROOT)
    def test_explicit_override_preserves_real_evidence(self):
        p=load_speed_profile(); self.assertEqual(p['default_m_s'],.3); self.assertEqual(p['maximum_verified_m_s'],1.)
        self.assertEqual(p['max_actual_speed_m_s'],1.25); self.assertIs(p['unverified_user_override'],True)
        audit=read(p['evidence']); self.assertEqual(audit['candidate_speed_m_s'],.3); self.assertEqual(audit['status'],'passed')
        for route in read('configs/patrol_routes.json')['routes'].values(): self.assertEqual(route['speed_limit_m_s'],1.)
    def test_without_override_mismatched_audit_rejected(self):
        for value in (False,None):
            p=copy.deepcopy(self.profile)
            if value is None: p.pop('unverified_user_override')
            else: p['unverified_user_override']=value
            with self.assertRaisesRegex(ValueError,'audit'): self.validate(p)
    def test_truthy_string_does_not_enable_override(self):
        p=copy.deepcopy(self.profile); p['unverified_user_override']='true'
        with self.assertRaisesRegex(ValueError,'boolean'): self.validate(p)
    def test_override_still_checks_status_identity_and_domains(self):
        for key,value in [('checkpoint_sha256','wrong'),('source_factory_sha256','wrong'),('factory_usd','wrong'),('default_m_s',1.1),('maximum_verified_m_s',1.01),('max_actual_speed_m_s',.9),('default_m_s',float('nan'))]:
            p=copy.deepcopy(self.profile); p[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError): self.validate(p)
        with patch('navigation.speed_profile.json.loads',return_value={'status':'failed','candidate_speed_m_s':.3}):
            with self.assertRaisesRegex(ValueError,'audit'): self.validate(self.profile)


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.safety=read('configs/navigation.json')['safety']; self.safety['reaction_latency_seconds']=.65
        self.braking=read('configs/navigation_braking.json')
    def test_lookahead_reaches_requested_speed_without_short_waypoint_stop(self):
        c=read('configs/navigation.json')['local']; c.update(max_speed_m_s=1.,lookahead_m=1./.7+.05)
        planner=LocalPlanner(c); planner.reset([(i*.2,0.) for i in range(41)])
        command,arrived,_=planner.command(state(),[8.,0.,None],.02)
        self.assertAlmostEqual(command[0],1.); self.assertFalse(arrived)
    def test_straight_chain_stops_at_real_corner_or_heading_override(self):
        nodes=['E','M','C','F']; xy={'E':(0,0),'M':(2,0),'C':(5,0),'F':(5,3)}
        self.assertEqual(straight_run_end(nodes,xy,0),1)
        self.assertEqual(straight_run_end(nodes,xy,1),1)
        self.assertEqual(straight_run_end(nodes,xy,0,{'M':1.}),0)
        self.assertAlmostEqual(remaining_length((1.8,.05),[(0,0),(2,0),(5,0)]),3.2)
        self.assertAlmostEqual(remaining_length((2,1),[(0,0),(2,0),(2,4)]),3.)
    def test_passed_plane_requires_forward_crossing_and_lateral_limit(self):
        self.assertTrue(passed_waypoint((2.1,.05),(0,0),(2,0),.4))
        self.assertFalse(passed_waypoint((1.99,0),(0,0),(2,0),.4))
        self.assertFalse(passed_waypoint((2.1,.41),(0,0),(2,0),.4))
        self.assertTrue(passed_waypoint((-2.1,.05),(0,0),(-2,0),.4))
    def test_corner_blend_and_exact_stopping_envelope(self):
        loose=dict(self.safety,prediction_seconds=.01,reaction_latency_seconds=.01,stopping_extra_m=0.)
        self.assertAlmostEqual(speed_cap(1.,2.,loose,{'conservative_deceleration_m_s2':10.}),1.)
        self.assertAlmostEqual(speed_cap(1.,1.6,loose,{'conservative_deceleration_m_s2':10.}),.61)
        self.assertAlmostEqual(speed_cap(1.,1.2,loose,{'conservative_deceleration_m_s2':10.}),.22)
        previous=-1.
        for distance in (0.,.08,.1,.5,1.2,1.6,2.,3.,5.,10.):
            cap=speed_cap(1.,distance,self.safety,self.braking)
            self.assertGreaterEqual(cap,previous); self.assertLessEqual(cap,1.); previous=cap
            if distance<=1.2: self.assertLessEqual(cap,.22)
            if distance<=self.safety['stopping_extra_m']: self.assertEqual(cap,0.)
            else:
                a=self.braking['conservative_deceleration_m_s2']; t=self.safety['prediction_seconds']+self.safety['reaction_latency_seconds']
                self.assertLessEqual(cap*t+cap*cap/(2*a),distance-self.safety['stopping_extra_m']+1e-9)
    def test_policy_scaling_preserves_direction_with_asymmetric_limits(self):
        low=[-.5,-.5,-1.]; high=[1.,.5,1.]
        for command in ([.9,.8,2.],[-.8,-.7,-2.],[.9,.8,0.]):
            result=bound_policy_command(command,low,high,.7)
            self.assertAlmostEqual(result[0]*command[1]-result[1]*command[0],0.)
            self.assertLessEqual(math.hypot(*result[:2]),.7+1e-9)
            self.assertTrue(all(low[i]<=result[i]<=high[i] for i in range(3)))
        import numpy as np
        result=bound_policy_command([.9,.8,2.],np.array(low,dtype=np.float32),np.array(high,dtype=np.float32),1.)
        self.assertTrue(all(type(v) is float for v in result)); json.dumps(result)
    def test_actual_factory_routes_do_not_extend_lookahead_into_wrong_branch(self):
        from scripts.check_route_geometry import build_grid
        from navigation.route_manager import RouteManager
        from navigation.route_graph import CorridorChainGrid
        grid=build_grid(); manager=RouteManager(read('configs/factory_v2.json'))
        xy=manager.data['nodes']
        self.assertLess(lateral_clearance(grid,xy['TOP_RIGHT_TURN'],xy['RIGHT_CORRIDOR']),.13)
        self.assertGreaterEqual(lateral_clearance(grid,xy['LEFT_BOTTOM'],xy['LEFT_CORRIDOR']),.13)
        for rid,expected_end in (('route_1_blue',4),('route_2_brown',5)):
            plan=manager.validate(grid,rid); route=manager.data['routes'][rid]; nodes=route['ordered_nodes']; xy=manager.data['nodes']
            end=straight_run_end(nodes,xy,4,route.get('heading_at_turns_rad',{}))
            self.assertEqual(end,expected_end)
            chain=CorridorChainGrid(grid,[(xy[nodes[j]],xy[nodes[j+1]]) for j in range(4,end+1)],.4)
            self.assertEqual(chain.segment_free((-.2,xy['TOP_CENTER_JUNCTION'][1]),(2.,xy['TOP_CENTER_JUNCTION'][1])),rid=='route_2_brown')
            self.assertGreater(plan['length_m'],60.)


def synthetic_navigation(position,requested=.3):
    """Real controller/planner/selector on a synthetic straight-then-turn graph."""
    s=state(*position); coordinates={'E':(0.,0.),'M':(2.,0.),'C':(5.,0.),'F':(5.,3.)}; nodes=list(coordinates)
    route={'ordered_nodes':nodes,'speed_limit_m_s':1.,'turn_speed_limit_rad_s':.3,'waypoint_tolerance_m':.13}
    paths=[[coordinates[a],coordinates[b]] for a,b in zip(nodes,nodes[1:])]
    lane=LaneMap({'resolution_m':.1,'footprint':dict(body_radius_m=.2,step_sway_m=.05,pose_error_m=.02,margin_m=.03),
                  'allowed_lanes':[{'polygon':[[-3,-3],[8,-3],[8,6],[-3,6]]}]})
    grid=OccupancyGrid(lane)
    def edge(base,rid,index,start):
        a,b=paths[index]
        return {'id':nodes[index]+'__'+nodes[index+1],'goal':b,'end_node':nodes[index+1],
                'path':[tuple(start),b],'grid':CorridorGrid(base,a,b,.4,start)}
    nav=object.__new__(MultiRouteController); nav.config=copy.deepcopy(read('configs/navigation.json'))
    nav.config['local']['max_speed_m_s']=requested; nav.config['safety']['reaction_latency_seconds']=.65
    nav.braking=read('configs/navigation_braking.json'); nav.local=LocalPlanner(nav.config['local'])
    nav.grid=grid; nav.route_manager=NS(data={'nodes':coordinates,'routes':{'synthetic':route},'corridor_center_allowance_m':.4},edge=edge)
    nav.route_plan={'path':[coordinates[n] for n in nodes],'edges':[{'path':p} for p in paths]}
    nav.active_route='synthetic'; nav.segment_index=0; nav.edge=None; nav.return_only=False; nav.position_recovery=False
    nav.scheduler=RouteScheduler(read('configs/patrol_route_settings.json')['schedule']); nav.now=0.; nav.sim_time=0.
    nav.selector=CommandSelector(NS(clear=lambda reason:None),[-.5,-.5,-1.],[1.,.5,1.]); nav.selector.select('AUTO PATROL'); nav.selector.previous=[requested,0.,0.]
    nav.speed_limit=1.; nav.runtime=NS(keys=NS(stop_generation=0,command=[0.,0.,0.]),policy=NS(feedback=NS(read_feedback=lambda:s)))
    nav.detector=NS(obstacles=[],timestamp=0.); nav.safety=SafetySupervisor(nav.config['safety'],nav.braking,True)
    nav.velocity_history=deque(maxlen=30); nav.last_stop_generation=0; nav.blocked_since=None; nav.last_replan=0.
    nav.refresh_obstacles=lambda:None; nav.record=lambda state,cmd:list(cmd)
    nav.route_complete_history=[]; nav.cycle_durations=[]
    nav.begin_edge(s)
    return nav,s


class ControllerIntegrationTests(unittest.TestCase):
    def test_tracking_and_safety_extend_past_straight_intermediate(self):
        nav,_=synthetic_navigation((1.8,0.)); command=nav.command(.02)
        self.assertEqual(nav.segment_index,0); self.assertGreater(command[0],.25)
        self.assertEqual(nav.local.path[-1],(5.,0.)); self.assertGreater(nav.distance_to_turn_m,3.)
        self.assertFalse(nav.edge['grid'].segment_free((1.8,0),(3.,0)))
        self.assertTrue(nav.motion_grid.segment_free((1.8,0),(3.,0)))
        self.assertFalse(nav.motion_grid.segment_free((4.8,0),(5.,1.)))
    def test_passed_intermediate_advances_without_zero_or_cycle_completion(self):
        nav,_=synthetic_navigation((2.1,.05)); command=nav.command(.02)
        self.assertEqual(nav.segment_index,1); self.assertGreater(command[0],.2)
        self.assertEqual(nav.scheduler.completed,0)
    def test_lateral_violation_cannot_skip_checkpoint(self):
        nav,_=synthetic_navigation((2.1,.46)); command=nav.command(.02)
        self.assertEqual(nav.segment_index,0); self.assertEqual(nav.scheduler.state,'WAITING_FOR_CLEAR_PATH')
        self.assertEqual(command[:2],[0.,0.])
    def test_corner_still_stops_and_turns(self):
        nav,_=synthetic_navigation((4.95,0.)); nav.segment_index=1; nav.edge=None
        nav.begin_edge(state(4.95,0.)); command=nav.command(.02)
        self.assertEqual(command[:2],[0.,0.]); self.assertGreater(command[2],0.)
        self.assertEqual(nav.segment_index,1); self.assertEqual(nav.scheduler.completed,0)
    def test_sub_gait_approach_starts_corner_turn_without_skipping_position_check(self):
        nav,_=synthetic_navigation((4.84,0.)); nav.segment_index=1; nav.edge=None
        nav.begin_edge(state(4.84,0.))
        for _ in range(20): command=nav.command(.02)
        self.assertLess(nav.braking_speed_cap_m_s,.1); self.assertTrue(nav.local.position_reached)
        self.assertTrue(nav.terminal_recovery_step); self.assertLessEqual(nav.active_speed_cap_m_s,.12)
        self.assertGreaterEqual(math.sqrt(sum(v*v for v in command)),.1)
        self.assertLessEqual(math.hypot(*command[:2]),nav.active_speed_cap_m_s+1e-9)
        self.assertEqual(nav.segment_index,1); self.assertEqual(nav.scheduler.completed,0)
    def test_recovery_reaches_corner_centre_then_settles_without_deadband(self):
        nav,feedback=synthetic_navigation((4.89,0.)); feedback.update(state(4.89,0.,math.pi/2))
        nav.segment_index=1; nav.edge=None; nav.begin_edge(feedback); nav.position_recovery=True
        command=nav.command(.02)
        self.assertTrue(nav.terminal_recovery_step); self.assertEqual(nav.segment_index,1)
        feedback.update(state(4.95,0.,math.pi/2))
        for _ in range(40):
            nav.command(.02)
            if nav.segment_index==2: break
        self.assertEqual(nav.segment_index,2); self.assertEqual(nav.scheduler.completed,0)
    def test_off_centre_straight_motion_reduces_forward_and_corrects_lateral(self):
        nav,_=synthetic_navigation((2.3,.1),.8); nav.selector.previous=[0.,0.,0.]
        for _ in range(50): command=nav.command(.02)
        self.assertLess(command[1],-.05); self.assertLessEqual(command[0],.22+1e-8)
        self.assertEqual(nav.segment_index,1); self.assertEqual(nav.scheduler.completed,0)
    def test_small_lateral_error_is_corrected_before_reaching_slowdown_threshold(self):
        nav,_=synthetic_navigation((2.3,.02),.8); nav.selector.previous=[0.,0.,0.]
        for _ in range(50): command=nav.command(.02)
        self.assertLess(command[1],-.009); self.assertGreater(command[0],.3)
    def test_blocked_optional_centring_does_not_prevent_turn_at_valid_waypoint(self):
        nav,_=synthetic_navigation((4.89,0.)); nav.segment_index=1; nav.edge=None
        nav.begin_edge(state(4.89,0.)); nav.position_recovery=True; original=nav.safety.check
        def guarded(feedback,command,*args):
            if math.hypot(*command[:2])>=.099: return 'WAITING_FOR_CLEAR_PATH','Test swept boundary'
            return original(feedback,command,*args)
        nav.safety.check=guarded; command=nav.command(.02)
        self.assertLessEqual(math.hypot(*command[:2]),nav.braking_speed_cap_m_s+1e-9)
        self.assertFalse(nav.terminal_recovery_step); self.assertGreater(command[2],0.)
        self.assertEqual(nav.scheduler.state,'TURNING'); self.assertEqual(nav.segment_index,1)
    def test_blocked_recovery_outside_tolerance_cannot_skip_corner(self):
        nav,_=synthetic_navigation((4.84,0.)); nav.segment_index=1; nav.edge=None
        nav.begin_edge(state(4.84,0.)); original=nav.safety.check
        def guarded(feedback,command,*args):
            if math.hypot(*command[:2])>=.099: return 'WAITING_FOR_CLEAR_PATH','Test swept boundary'
            return original(feedback,command,*args)
        nav.safety.check=guarded; command=nav.command(.02)
        self.assertEqual(command,[0.,0.,0.]); self.assertEqual(nav.scheduler.state,'WAITING_FOR_CLEAR_PATH')
        self.assertEqual(nav.segment_index,1); self.assertEqual(nav.scheduler.completed,0)


class SweptSafetyTests(unittest.TestCase):
    def check(self,obstacle,swept=True):
        cfg=read('configs/navigation.json')['safety']; cfg['max_actual_speed_m_s']=1.25
        monitor=SafetySupervisor(cfg,read('configs/navigation_braking.json'),swept)
        lane=LaneMap({'resolution_m':.1,'footprint':dict(body_radius_m=.2,step_sway_m=.05,pose_error_m=.02,margin_m=.03),
                      'allowed_lanes':[{'polygon':[[-5,-5],[5,-5],[5,5],[-5,5]]}]},[obstacle])
        class Grid:
            map=lane
            def segment_free(self,a,b):
                return all(lane.safe((a[0]+(b[0]-a[0])*i/100,a[1]+(b[1]-a[1])*i/100)) for i in range(101))
        return monitor.check(state(velocity=(.8,0,0)),[.8,0,0],Grid(),[obstacle],0.)
    def test_side_shelf_no_longitudinal_brake_false_positive_and_v1_unchanged(self):
        shelf={'kind':'static','dynamic':False,'bounds':[[.2,.9],[2.,1.1]]}
        self.assertIsNone(self.check(shelf)[0]); self.assertEqual(self.check(shelf,False)[0],'WAITING_FOR_CLEAR_PATH')
    def test_front_obstacle_human_dynamic_and_close_are_preserved(self):
        for obstacle in (
            {'kind':'static','dynamic':False,'bounds':[[1.,-.2],[1.2,.2]]},
            {'kind':'human','dynamic':False,'bounds':[[.2,.9],[2.,1.1]]},
            {'kind':'static','dynamic':True,'velocity':[0,-1.],'bounds':[[.2,.9],[2.,1.1]]},
            {'kind':'static','dynamic':False,'bounds':[[0.,.34],[.2,.5]]}):
            with self.subTest(obstacle=obstacle): self.assertEqual(self.check(obstacle)[0],'WAITING_FOR_CLEAR_PATH')
    def test_exact_capsule_distance_diagonal_and_degenerate(self):
        self.assertAlmostEqual(segment_box_distance((0,0),(2,2),[[1,1],[1.5,1.5]]),0.)
        self.assertAlmostEqual(segment_box_distance((0,0),(0,0),[[1,1],[2,2]]),math.sqrt(2))
        self.assertAlmostEqual(segment_box_distance((0,0),(2,0),[[.5,1],[1.5,2]]),1.)
    def test_directional_mode_retains_feedback_faults(self):
        cfg=read('configs/navigation.json')['safety']; cfg['max_actual_speed_m_s']=1.25
        monitor=SafetySupervisor(cfg,read('configs/navigation_braking.json'),True)
        grid=NS(map=NS(safe=lambda p:True),segment_free=lambda a,b:True)
        for feedback,age in ((state(),10.),(dict(state(),base_position_m=[0.,0.,.2]),0.),
                             (state(velocity=(1.3,0.,0.)),0.),(dict(state(),base_orientation_wxyz=[2.,0.,0.,0.]),0.)):
            with self.subTest(age=age,feedback=feedback): self.assertEqual(monitor.check(feedback,[0.,0.,0.],grid,[],age)[0],'FAULT')


class UiSpeedTests(unittest.TestCase):
    def panel(self,value,phase='STOPPED'):
        panel=object.__new__(PatrolPanel); cfg=read('configs/patrol_route_settings.json')['schedule']
        scheduler=RouteScheduler(cfg); scheduler.transition(phase,0.)
        panel.nav=NS(scheduler=scheduler,now=0.,speed_limit=1.,config=read('configs/navigation.json'),before_tick=lambda running:None)
        panel.nav.config['local']['max_speed_m_s']=.3
        panel.fields={key:NS(as_float=float(v)) for key,v in cfg.items() if key not in ('repeat_enabled','clock_mode')}
        panel.repeat=NS(model=NS(as_bool=False)); panel.clock=NS(model=NS(get_item_value_model=lambda:NS(as_int=0)))
        panel.speed=NS(model=NS(as_float=value)); panel.runtime=NS(timeline=NS(is_playing=lambda:True))
        panel.mode=NS(model=NS(get_item_value_model=lambda:NS(set_value=lambda v:None)))
        return panel
    def test_presets_and_start_apply_selected_value(self):
        self.assertEqual(speed_presets(1.),(.22,.3,.5,.8,1.)); self.assertEqual(speed_presets(.3),(.22,.3))
        panel=self.panel(.8); started=[]; panel.nav.start=lambda:started.append(panel.nav.config['local']['max_speed_m_s'])
        self.assertFalse(started); panel.start(); self.assertEqual(started,[.8])
    def test_start_applies_route2_without_separate_apply_button(self):
        panel=self.panel(1.); panel.nav.route_mode='route_1_blue'
        panel.route_choice=NS(model=NS(get_item_value_model=lambda:NS(as_int=1)))
        panel.nav.select_route=lambda mode:setattr(panel.nav,'route_mode',mode)
        panel.nav.selected_route=lambda:panel.nav.route_mode
        started=[]; panel.nav.start=lambda:started.append((panel.nav.route_mode,panel.nav.config['local']['max_speed_m_s']))
        panel.start(); self.assertEqual(started,[('route_2_brown',1.)])
    def test_start_plays_paused_simulation_then_starts_once_on_next_frame(self):
        panel=self.panel(.8); playing=[False]; requests=[]; started=[]
        panel.message=NS(text=''); panel.runtime.timeline=NS(is_playing=lambda:playing[0],play=lambda:requests.append('play'),pause=lambda:playing.__setitem__(0,False))
        panel.nav.start=lambda:started.append(panel.nav.config['local']['max_speed_m_s'])
        panel.start(); self.assertEqual(requests,['play']); self.assertFalse(started)
        self.assertTrue(panel.start_pending); playing[0]=True; panel.process_pending_start(); panel.process_pending_start()
        self.assertEqual(started,[.8]); self.assertFalse(panel.start_pending)
    def test_stop_input_cancels_pending_autoplay_start(self):
        panel=self.panel(.8); started=[]; panel.runtime.keys=NS(stop_generation=0)
        panel.runtime.timeline=NS(is_playing=lambda:False,play=lambda:None,pause=lambda:None)
        panel.nav.start=lambda:started.append(True)
        panel.start(); panel.runtime.keys.stop_generation=1; panel.process_pending_start()
        self.assertFalse(panel.start_pending); self.assertFalse(started)
    def test_fault_cannot_be_bypassed_by_start_autoplay(self):
        panel=self.panel(.8,'FAULT'); played=[]
        panel.runtime.timeline=NS(is_playing=lambda:False,play=lambda:played.append(True))
        with self.assertRaises(ValueError): panel.start()
        self.assertFalse(played)
    def test_moving_state_does_not_apply_speed_and_above_limit_rejected(self):
        panel=self.panel(.8,'MOVING_SEGMENT')
        with self.assertRaises(ValueError): panel.apply_schedule()
        self.assertEqual(panel.nav.config['local']['max_speed_m_s'],.3)
        with self.assertRaises(ValueError): self.panel(1.01).apply_schedule()
