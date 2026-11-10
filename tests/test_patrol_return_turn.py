import json,math,unittest
from pathlib import Path
from navigation.local_planner import LocalPlanner

class ReturnTurnTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads(Path(r'D:\unitree_g1\configs\navigation.json').read_text())['local']
        self.planner=LocalPlanner(self.cfg)
        self.planner.reset([(0,1.5),(0,0)])
    def state(self,x,y,yaw):
        return {'base_position_m':[x,y,.78], 'base_orientation_wxyz':[math.cos(yaw/2),0,0,math.sin(yaw/2)],
                'base_linear_velocity_world_m_s':[0,0,0], 'base_angular_velocity_world_rad_s':[0,0,0]}
    def test_return_turns_before_walking_despite_same_endpoint_heading(self):
        goal=[0,0,math.pi/2]
        command,_,_=self.planner.command(self.state(0,1.5,math.pi/2),goal,.02)
        self.assertEqual(command[:2],[0,0]); self.assertGreater(abs(command[2]),0)
        command,_,_=self.planner.command(self.state(0,1.5,-math.pi/2),goal,.02)
        self.assertGreater(command[0],0); self.assertAlmostEqual(command[1],0)
    def test_endpoint_heading_applies_after_arrival(self):
        command,_,_=self.planner.command(self.state(0,.05,-math.pi/2),[0,0,math.pi/2],.02)
        self.assertEqual(command[:2],[0,0]); self.assertGreater(abs(command[2]),0)
    def test_endpoint_sway_does_not_restart_turn(self):
        self.planner.command(self.state(0,.1,-math.pi/2),[0,0,None],.02)
        command,_,_=self.planner.command(self.state(0,-.18,-math.pi/2),[0,0,None],.02)
        self.assertEqual(command[2],0)
        self.assertTrue(self.planner.position_reached)
        self.assertLessEqual(math.hypot(*command[:2]),.12)
    def test_close_return_still_turns(self):
        command,_,_=self.planner.command(self.state(0,.35,math.pi/2),[0,0,None],.02)
        self.assertEqual(command[:2],[0,0]); self.assertGreater(abs(command[2]),0)
    def test_reset_clears_arrival_latch(self):
        self.planner.command(self.state(0,.1,0),[0,0,None],.02)
        self.planner.reset([(0,0),(0,1.5)])
        command,_,_=self.planner.command(self.state(0,0,math.pi/2),[0,1.5,None],.02)
        self.assertGreater(command[0],0)
    def test_final_turn_corrects_drift_without_restarting_travel(self):
        self.planner.command(self.state(0,.05,-math.pi/2),[0,0,math.pi/2],.02)
        command,_,_=self.planner.command(self.state(0,.28,0),[0,0,math.pi/2],.02)
        self.assertTrue(self.planner.position_reached)
        self.assertLess(command[1],0)
        self.assertLessEqual(math.hypot(*command[:2]),.12)
