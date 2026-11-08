import unittest,math
from navigation.lane_map import LaneMap
from navigation.occupancy_grid import OccupancyGrid
from navigation.global_planner import GlobalPlanner,NoValidPath
from navigation.patrol_scheduler import PatrolScheduler,validate_schedule
from navigation.safety_supervisor import SafetySupervisor


def config():
    return {'resolution_m':.1,'footprint':{'body_radius_m':.2,'step_sway_m':.05,'pose_error_m':.02,'margin_m':.03},
            'allowed_lanes':[{'polygon':[[-2,-2],[2,-2],[2,2],[-2,2]]}],'forbidden_zones':[]}
def schedule():
    return {'start_delay_seconds':5,'repeat_interval_seconds':120,'dwell_at_b_seconds':10,'repeat_enabled':True,
            'max_cycles':2,'movement_timeout_seconds':100,'obstacle_wait_timeout_seconds':30,'clock_mode':'simulation'}


class PlanningTests(unittest.TestCase):
    def test_roundtrip(self):
        g=OccupancyGrid(LaneMap(config()))
        for c in [(2,2),(20,20),(35,35)]: self.assertEqual(g.world_to_cell(g.cell_to_world(c)),c)
    def test_boundary_and_footprint(self):
        g=OccupancyGrid(LaneMap(config()))
        self.assertFalse(g.point_free((1.9,0)))
        with self.assertRaises(NoValidPath): GlobalPlanner(g).plan((0,0),(1.9,0))
    def test_obstacle_detour(self):
        o={'bounds':[[-.3,-.5],[.3,.5]]}
        g=OccupancyGrid(LaneMap(config(),[o])); path=GlobalPlanner(g).plan((-1,0),(1,0))
        self.assertGreater(len(path),2)
        self.assertTrue(all(g.segment_free(a,b) for a,b in zip(path,path[1:])))
        self.assertFalse(g.segment_free((-1,0),(1,0)))
    def test_full_block(self):
        g=OccupancyGrid(LaneMap(config(),[{'bounds':[[-.2,-2],[.2,2]]}]))
        with self.assertRaises(NoValidPath): GlobalPlanner(g).plan((-1,0),(1,0))
    def test_narrow_lane_no_margin_reduction(self):
        c=config(); c['allowed_lanes'][0]['polygon']=[[-.2,-2],[.2,-2],[.2,2],[-.2,2]]
        with self.assertRaises(NoValidPath): GlobalPlanner(OccupancyGrid(LaneMap(c))).plan((0,-1),(0,1))
    def test_forbidden_polygon(self):
        c=config(); c['forbidden_zones']=[{'polygon':[[-.2,-2],[.2,-2],[.2,2],[-.2,2]]}]
        with self.assertRaises(NoValidPath): GlobalPlanner(OccupancyGrid(LaneMap(c))).plan((-1,0),(1,0))
    def test_no_diagonal_cut(self):
        g=OccupancyGrid(LaneMap(config())); g.blocked[:]=True
        g.blocked[10,10]=False; g.blocked[11,11]=False
        with self.assertRaises(NoValidPath): GlobalPlanner(g).plan(g.cell_to_world((10,10)),g.cell_to_world((11,11)))


class ScheduleTests(unittest.TestCase):
    def test_repeat_after_completed_return(self):
        s=PatrolScheduler(schedule()); s.start(0); s.update(4); self.assertEqual(s.state,'WAITING_START')
        s.update(5); self.assertEqual(s.state,'PLANNING')
        s.arrived(20); s.update(29); self.assertEqual(s.state,'WAITING_AT_B')
        s.update(30); self.assertEqual(s.goal,'A'); s.arrived(50)
        self.assertEqual(s.completed,1); s.update(169); self.assertEqual(s.state,'CYCLE_COMPLETE')
        s.update(170); self.assertEqual(s.state,'PLANNING'); s.goal='A'; s.arrived(200); self.assertEqual(s.state,'STOPPED')
    def test_pause_never_starts_on_wall_timer(self):
        c=schedule(); c['clock_mode']='wall'
        s=PatrolScheduler(c); s.start(0); s.pause(1); s.update(1000); self.assertEqual(s.state,'PAUSED')
        s.resume(1000); s.update(1000); self.assertEqual(s.state,'PLANNING')
    def test_auto_pause_freezes_simulation_schedule(self):
        s=PatrolScheduler(schedule()); s.start(0); s.pause(1); s.resume(11); s.update(14)
        self.assertEqual(s.state,'WAITING_START'); s.update(15); self.assertEqual(s.state,'PLANNING')
    def test_invalid_schedule(self):
        c=schedule(); c['repeat_interval_seconds']=float('nan')
        with self.assertRaises(ValueError): validate_schedule(c)
    def test_measured_braking_required(self):
        s=SafetySupervisor({'reaction_latency_seconds':.3,'stopping_extra_m':.08},{'validated':False})
        with self.assertRaises(ValueError): s.stop_distance(.2)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        import json
        from pathlib import Path
        c=json.loads(Path(r'D:\unitree_g1\configs\navigation.json').read_text())
        self.supervisor=SafetySupervisor(c['safety'],{'validated':True,'conservative_deceleration_m_s2':.32})
        self.state={'base_position_m':[0,0,.78],'base_orientation_wxyz':[1,0,0,0],
                    'base_linear_velocity_world_m_s':[.2,0,0],'base_angular_velocity_world_rad_s':[0,0,0]}
    def test_human_priority_stop(self):
        obstacle={'bounds':[[.8,-.2],[1.2,.2]],'kind':'human','dynamic':False}
        grid=OccupancyGrid(LaneMap(config(),[obstacle]))
        status,_=self.supervisor.check(self.state,[.2,0,0],grid,[obstacle],0)
        self.assertEqual(status,'WAITING_FOR_CLEAR_PATH')
    def test_predicted_crossing(self):
        o={'bounds':[[-1.4,-.2],[-1.,.2]],'kind':'static','dynamic':True,'velocity':[2,0]}
        grid=OccupancyGrid(LaneMap(config(),[o]))
        status,reason=self.supervisor.check(self.state,[.2,0,0],grid,[o],0)
        self.assertEqual(status,'WAITING_FOR_CLEAR_PATH'); self.assertIn('crossing',reason)
    def test_stale_and_fallen(self):
        grid=OccupancyGrid(LaneMap(config()))
        self.assertEqual(self.supervisor.check(self.state,[0,0,0],grid,[],10)[0],'FAULT')
        self.state['base_position_m'][2]=.2
        self.assertEqual(self.supervisor.check(self.state,[0,0,0],grid,[],0)[0],'FAULT')
    def test_prediction_cannot_leave_lane(self):
        grid=OccupancyGrid(LaneMap(config()))
        self.state['base_position_m']=[1.5,0,.78]
        self.assertEqual(self.supervisor.check(self.state,[.25,0,0],grid,[],0)[0],'WAITING_FOR_CLEAR_PATH')


class MissionBudgetTests(unittest.TestCase):
    def test_replan_preserves_leg_deadline(self):
        from navigation.patrol_controller import PatrolController
        from navigation.local_planner import LocalPlanner
        from types import SimpleNamespace
        nav=object.__new__(PatrolController)
        nav.a=[0,0,0]; nav.b=[0,1,0]; nav.scheduler=PatrolScheduler(schedule())
        nav.active_leg=None; nav.return_only=False; nav.cycle_start=None
        nav.local=LocalPlanner({}); nav.planner=SimpleNamespace(plan=lambda a,b:[a,b])
        state={'base_position_m':[0,0,.78],'base_orientation_wxyz':[1,0,0,0]}
        nav.now=1; nav.sim_time=1; nav.plan_current(state)
        nav.now=50; nav.sim_time=50; nav.plan_current(state)
        self.assertEqual(nav.movement_started,1)
        nav.scheduler.goal='A'; nav.plan_current(state)
        self.assertEqual(nav.movement_started,50)
    def test_stop_cannot_clear_fault(self):
        from navigation.patrol_controller import PatrolController
        from types import SimpleNamespace
        nav=object.__new__(PatrolController); nav.scheduler=PatrolScheduler(schedule()); nav.scheduler.transition('FAULT',0)
        nav.selector=SimpleNamespace(stop=lambda:None); nav.path=[(0,0)]; nav.stop()
        self.assertEqual(nav.scheduler.state,'FAULT')
        with self.assertRaises(ValueError): nav.set_mode('MANUAL')
