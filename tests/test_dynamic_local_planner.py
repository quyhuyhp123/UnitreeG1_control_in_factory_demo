import json
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from navigation.dynamic_local_planner import DynamicLocalPlanner
from navigation.command_adapter import CommandSelector
from navigation.lane_map import segment_distance
from navigation.route_graph import CorridorGrid
from navigation.safety_supervisor import SafetySupervisor


ROOT = Path(__file__).resolve().parents[1]


class DynamicPlannerTests(unittest.TestCase):
    def setUp(self):
        config = json.loads((ROOT/'configs/navigation.json').read_text())['safety']
        config['reaction_latency_seconds'] = .65
        self.safety = SafetySupervisor(config, {'validated': True, 'conservative_deceleration_m_s2': .5})
        self.planner = DynamicLocalPlanner(self.safety)
        self.goal = (2., 0.)
        self.pose = (.025, .025, 0.)
        ground = SimpleNamespace(radius=.55, safe=lambda p: abs(p[1]) <= .4)
        base = SimpleNamespace(map=ground, resolution=.1)
        self.corridor = CorridorGrid(base, (-1., 0.), self.goal, .4)
        self.low, self.high = (-.6, -.25, -.6), (1., .25, .6)
        self.grid = {
            'cost': np.zeros((60, 60), dtype=np.uint8),
            'traversable': np.ones((60, 60), dtype=bool),
            'distance_m': np.full((60, 60), 1.),
            'free_age_s': np.zeros((60, 60)),
            'origin_world_m': (-3., -3.), 'size_cells': 60, 'resolution_m': .1, 'sim_time': 4.,
        }

    def choose(self, people=()):
        return self.planner.choose(self.grid, self.corridor, self.pose, [.30, 0., 0.],
                                   self.low, self.high, .30, self.goal, .13, people)

    def barrier(self, rows, start=33):
        self.grid['cost'][start:48, rows] = 254
        self.grid['traversable'][start:48, rows] = False
        self.grid['distance_m'][start:48, rows] = .2

    def test_A_slow_for_static_obstacle_and_keep_stopping_room(self):
        self.barrier(slice(None), start=34)
        result = self.choose()
        self.assertIsNotNone(result)
        self.assertLess(math.hypot(*result[:2]), .30)
        self.assertAlmostEqual(result[1], 0.)
        self.assertLess(self.planner.telemetry['predicted_end_xy'][0], .4)

    def test_B_C_only_open_side_is_selected(self):
        for side in (1, -1):
            with self.subTest(side=side):
                self.setUp()
                self.barrier(slice(None, 31) if side == 1 else slice(30, None), start=31)
                result = self.choose()
                self.assertIsNotNone(result)
                self.assertGreater(side*result[1], 0.)
                self.assertLessEqual(abs(result[1]), .25)
                self.assertLessEqual(math.hypot(*result[:2]), .30)
                self.assertTrue(self.corridor.contains(self.planner.telemetry['predicted_end_xy']))

    def test_D_blocked_or_unknown_wait(self):
        self.grid['cost'][:, :] = 255
        self.grid['traversable'][:, :] = False
        self.assertIsNone(self.choose())

    def test_supervisor_veto_of_forward_command_selects_only_open_side(self):
        self.safety.swept = True
        selector = CommandSelector(None, self.low, self.high)
        obstacle = {'kind':'static','bounds':[[.645,-.10],[.90,.10]]}
        state = {'base_position_m':[*self.pose[:2],.78], 'base_orientation_wxyz':[1.,0.,0.,0.],
                 'base_linear_velocity_world_m_s':[0.,0.,0.], 'base_angular_velocity_world_rad_s':[0.,0.,0.]}
        def blocker(candidate):
            filtered = selector.auto(candidate,.02,preserve_linear_direction=True,preview=True)
            status, reason = self.safety.check(state,filtered,self.corridor,[obstacle],0.)
            return reason if status else None

        for side in (-1,1):
            with self.subTest(side=side):
                self.grid['cost'][:,:] = 0
                self.grid['traversable'][:,:] = True
                closed = slice(None,30) if side>0 else slice(31,None)
                self.grid['cost'][:,closed] = 255
                self.grid['traversable'][:,closed] = False
                self.assertEqual(blocker([.2,0.,0.]),'Obstacle in braking envelope')
                command = self.planner.choose(self.grid,self.corridor,self.pose,[.2,0.,0.],
                                              self.low,self.high,.2,self.goal,.13,(),command_blocker=blocker)
                self.assertIsNotNone(command)
                self.assertGreater(side*command[1],0.)
                self.assertIsNone(blocker(command))
                self.assertEqual(selector.previous,[0.,0.,0.])
        self.grid['cost'][:, :] = 254
        self.assertIsNone(self.choose())

    def test_high_requested_speed_still_samples_crawl_on_short_visible_path(self):
        self.grid['cost'][33:, :] = 255
        self.grid['traversable'][33:, :] = False
        command = self.planner.choose(self.grid, self.corridor, self.pose, [.8, 0., 0.],
                                      self.low, self.high, .8, self.goal, .13, ())
        self.assertIsNotNone(command)
        self.assertGreaterEqual(math.hypot(*command[:2]), .10)
        self.assertLess(math.hypot(*command[:2]), .22)
        self.assertEqual(self.planner.telemetry['nominal_blocker'],'floor_unseen_or_stale')

    def test_half_metre_floor_check_keeps_braking_and_known_obstacles(self):
        self.grid['cost'][36:, :] = 255
        self.grid['traversable'][36:, :] = False
        command = [.6, 0., 0.]
        self.assertIsNotNone(self.planner.trace(self.grid,self.corridor,self.pose,command,self.goal,.13))
        self.assertIsNone(self.planner.trace(self.grid,self.corridor,self.pose,command,self.goal,.13,actual=(.6,0.)))
        self.grid['cost'][38, :] = 254
        self.assertIsNone(self.planner.trace(self.grid,self.corridor,self.pose,command,self.goal,.13))

    def test_simulation_bypass_uses_short_collision_sweep(self):
        self.grid['cost'][37, :] = 254
        command = [.8, 0., 0.]
        self.assertIsNone(self.planner.trace(self.grid,self.corridor,self.pose,command,self.goal,.13))
        self.safety.simulation_braking_bypass = True
        self.grid['cost'][31:36, :] = 255
        self.grid['traversable'][31:36, :] = False
        end, _ = self.planner.trace(self.grid,self.corridor,self.pose,command,self.goal,.13)
        self.assertAlmostEqual(math.dist(end,self.pose[:2]),.5)
        selected=self.planner.choose(self.grid,self.corridor,self.pose,command,self.low,self.high,.8,self.goal,.13,())
        self.assertEqual(selected,command)
        self.grid['cost'][34, :] = 254
        self.assertIsNone(self.planner.trace(self.grid,self.corridor,self.pose,command,self.goal,.13))

    def test_E_F_rejoin_selected_route_segment_in_its_world_orientation(self):
        data = json.loads((ROOT/'configs/patrol_routes.json').read_text())
        for route_id, node in (('route_1_blue', 'CENTER_CORRIDOR'), ('route_2_brown', 'RIGHT_CORRIDOR')):
            with self.subTest(route=route_id):
                nodes = data['routes'][route_id]['ordered_nodes']
                i = nodes.index(node)
                a, b = (data['nodes'][n] for n in nodes[i-1:i+1])
                yaw = math.atan2(b[1]-a[1], b[0]-a[0])
                x, y = ((a[j]+b[j])/2 for j in (0, 1))
                self.pose = (x-.2*math.sin(yaw), y+.2*math.cos(yaw), yaw)
                self.grid['origin_world_m'] = (x-3, y-3)
                self.corridor = CorridorGrid(self.corridor.base, a, b, .4)
                self.corridor.map.safe = lambda p: self.corridor.contains(p)
                command = self.planner.choose(self.grid, self.corridor, self.pose, [.2, -.12, 0.],
                                              self.low, self.high, .30, b, .13, ())
                self.assertIsNotNone(command)
                end = self.planner.telemetry['predicted_end_xy']
                self.assertLess(segment_distance(end, a, b), segment_distance(self.pose[:2], a, b))
                self.assertEqual(self.corridor.b, b)

    def test_checkpoint_plane_and_corner_rotation(self):
        self.pose = (1.7, .22, 0.)
        self.assertIsNone(self.planner.trace(self.grid, self.corridor, self.pose, [.30, 0., 0.], self.goal, .13))
        self.assertIsNotNone(self.planner.trace(self.grid, self.corridor, (2., 0., 0.), [0., 0., .35], self.goal, .13))

    def test_override_and_measured_velocity_remain_bounded(self):
        goal = (2.9, 0.)
        corridor = CorridorGrid(self.corridor.base, self.corridor.a, goal, .4)
        for cap in (.30, .70, 1.):
            with self.subTest(cap=cap):
                command = self.planner.choose(self.grid, corridor, self.pose, [1., 0., 0.],
                                              self.low, self.high, cap, goal, .13, ())
                self.assertAlmostEqual(math.hypot(*command[:2]), cap)
        self.barrier(slice(None), start=34)
        self.assertIsNone(self.planner.trace(self.grid, self.corridor, self.pose, [0., 0., 0.],
                                             self.goal, .13, actual=(.30, 0.)))

    def test_G_stale_missing_invalid_or_old_epoch_is_rejected(self):
        now = time.time()
        result = {'perception_epoch': 2, '_costmap': self.grid, 'wall_time_capture': now, 'points': {'valid': 42}}
        self.assertIs(self.planner.observation(result, 2, 4.1, now), self.grid)
        self.assertIsNone(self.planner.observation(None, 2, 4.1, now))
        self.assertIsNone(self.planner.observation(result, 2, 4.7, now))
        self.assertIsNone(self.planner.observation(result, 2, 4.1, now+2.01))
        self.assertIsNone(self.planner.observation(result, 3, 4.1, now))
        self.assertIsNone(self.planner.observation({**result, 'points': {'valid': 0}}, 2, 4.1, now))
        self.grid['free_age_s'][:, :] = .8
        self.assertIsNone(self.choose())

    def test_H_safety_still_rejects_fallen_robot(self):
        state = {'base_position_m': [0., 0., .3], 'base_orientation_wxyz': [1., 0., 0., 0.],
                 'base_linear_velocity_world_m_s': [0., 0., 0.], 'base_angular_velocity_world_rad_s': [0., 0., 0.]}
        self.assertEqual(self.safety.check(state, [.2, 0., 0.], self.corridor, [], 0.)[0], 'FAULT')
        self.assertIsNone(self.choose([{'kind': 'human', 'bounds': [[.9, -.1], [1., .1]]}]))
        self.grid['human_cells'] = [(39, 30)]
        self.assertIsNone(self.choose())

if __name__ == '__main__':
    unittest.main()
