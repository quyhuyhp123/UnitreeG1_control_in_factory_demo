import unittest,json,sys,copy
from pathlib import Path
ROOT=Path(r'D:\unitree_g1'); sys.path.append(str(ROOT/'scripts'))
from check_route_geometry import build_grid
from navigation.route_manager import RouteManager
from navigation.global_planner import NoValidPath
from navigation.route_scheduler import RouteScheduler
class MultiRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid=build_grid(); cls.cfg=json.loads((ROOT/'configs/factory_v2.json').read_text())
    def test_geometry_and_required_branches(self):
        m=RouteManager(self.cfg)
        for rid,required in [('route_1_blue','CENTER_CORRIDOR'),('route_2_brown','RIGHT_CORRIDOR')]:
            p=m.validate(self.grid,rid); self.assertIn(required,p['waypoints']); self.assertGreater(p['length_m'],60)
            self.assertTrue(all(e['grid'].segment_free(a,b) for e in p['edges'] for a,b in zip(e['path'],e['path'][1:])))
            self.assertEqual(p['waypoints'][-1],'EXIT')
    def test_center_block_does_not_use_right_route(self):
        g=copy.copy(self.grid); g.blocked=self.grid.blocked.copy()
        for y in range(g.height):
            for x in range(g.width):
                px,py=g.cell_to_world((x,y))
                if abs(px)<1 and abs(py)<.3: g.blocked[y,x]=True
        m=RouteManager(self.cfg)
        with self.assertRaisesRegex(NoValidPath,'INVALID_ROUTE_SEGMENT'): m.validate(g,'route_1_blue')
        self.assertEqual(m.validate(g,'route_2_brown')['route_id'],'route_2_brown')
    def test_identity_and_checkpoint_order(self):
        m=RouteManager(self.cfg); d=copy.deepcopy(m.data); d['source_factory_sha256']='invalid'
        with self.assertRaises(ValueError): RouteManager(self.cfg,d)
        d=copy.deepcopy(m.data); d['routes']['route_1_blue']['ordered_nodes'].remove('CENTER_CORRIDOR')
        with self.assertRaises(ValueError): RouteManager(self.cfg,d)
    def test_route_cannot_exceed_verified_speed_profile(self):
        from navigation.speed_profile import load_speed_profile
        m=RouteManager(self.cfg); d=copy.deepcopy(m.data)
        d['routes']['route_1_blue']['speed_limit_m_s']=load_speed_profile()['maximum_verified_m_s']+.01
        with self.assertRaisesRegex(ValueError,'velocity/tolerance'): RouteManager(self.cfg,d)
    def test_safe_actual_pose_can_connect_from_conservative_cell(self):
        start=(-7.11,-8.)
        self.assertTrue(self.grid.map.safe(start)); self.assertFalse(self.grid.point_free(start))
        m=RouteManager(self.cfg); index=m.data['routes']['route_1_blue']['ordered_nodes'].index('LEFT_BOTTOM')
        edge=m.edge(self.grid,'route_1_blue',index,start)
        self.assertEqual(tuple(edge['path'][0]),start)
        self.assertTrue(all(edge['grid'].segment_free(a,b) for a,b in zip(edge['path'],edge['path'][1:])))
    def test_alternate_only_completed_count(self):
        m=RouteManager(self.cfg)
        self.assertEqual([m.route_id('alternate',i) for i in range(4)],['route_1_blue','route_2_brown']*2)
        self.assertEqual(m.route_id('alternate',0,'route_2_brown'),'route_2_brown')
    def test_fault_zeros_command_telemetry_without_waiting_for_policy_tick(self):
        from types import SimpleNamespace
        from navigation.patrol_controller import PatrolController
        from navigation.command_adapter import CommandSelector
        scheduler=RouteScheduler(json.loads((ROOT/'configs/patrol_route_settings.json').read_text())['schedule'])
        selector=CommandSelector(None,[-.5,-.5,-1],[1,.5,1]); selector.auto([.2,0,0],.02)
        controller=SimpleNamespace(scheduler=scheduler,now=3.,selector=selector,path=[(0,0)],last_command=[.2,0,0])
        PatrolController.fail(controller,'Injected runtime fault')
        self.assertEqual(controller.last_command,[0,0,0]); self.assertEqual(scheduler.state,'FAULT')
        self.assertFalse(controller.path); self.assertEqual(scheduler.completed,0)
    def test_vector_acceleration_does_not_turn_safe_command_toward_wall(self):
        import math
        from navigation.command_adapter import CommandSelector
        s=CommandSelector(None,[-.5,-.5,-1],[1,.5,1]); target=[.22,.015,.1]
        command=s.auto(target,.02,preserve_linear_direction=True)
        self.assertAlmostEqual(command[0]*target[1]-command[1]*target[0],0)
        self.assertLessEqual(math.hypot(*command[:2]),.4*.02+1e-10)
        self.assertGreater(math.cos(1.494)*command[0]-math.sin(1.494)*command[1],0)
    def test_schedule_pause_and_repeat_no_false_completion(self):
        c=json.loads((ROOT/'configs/patrol_route_settings.json').read_text())['schedule']; c.update(repeat_enabled=True,max_cycles=2)
        s=RouteScheduler(c); s.start(0); s.update(5); s.transition('TURNING',6); s.pause(7); s.resume(8)
        self.assertEqual(s.completed,0); s.transition('MOVING_TO_EXIT',99); s.route_finished(100); s.update(219.9); self.assertEqual(s.state,'CYCLE_COMPLETE')
        s.update(220); self.assertEqual(s.state,'VALIDATING_ROUTE'); s.transition('MOVING_TO_EXIT',299); s.route_finished(300)
        self.assertEqual(s.completed,2); self.assertEqual(s.state,'STOPPED')
        with self.assertRaises(ValueError): s.route_finished(301)
        s.transition('FAULT',302)
        with self.assertRaises(ValueError): s.route_finished(303)
        self.assertEqual(s.completed,2)
