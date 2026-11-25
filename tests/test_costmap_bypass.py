"""Level-1 sensor-driven bypass: costmap blobs, early diagonal, checkpoint gates. No simulator."""
import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT/'tests'))

from navigation.costmap_obstacles import extract, load_config
from navigation.lane_map import LaneMap, box_distance
from navigation.obstacle_bypass import plan, path_clear
from perception.local_costmap import LocalCostmap, load_config as load_costmap
from test_local_costmap import observation

FOOTPRINT = {'body_radius_m': .35, 'step_sway_m': .08, 'pose_error_m': .04, 'margin_m': .1}
# A 4 m wide lane along +Y, with a forbidden machine block beside it.
LANE = {'resolution_m': .1, 'footprint': FOOTPRINT,
        'allowed_lanes': [{'polygon': [[-2, -1], [2, -1], [2, 20], [-2, 20]]}],
        'forbidden_zones': [{'polygon': [[2.5, 0], [4, 0], [4, 3], [2.5, 3]]}]}


def factory(items):
    return LaneMap(LANE, items)


def snapshot(obstacles, classes=None, robot=(0., 0.), floor=None, size=10.):
    costmap = LocalCostmap(load_costmap({'size_m': size}), LANE)
    floor = floor or [(-2., .4, 2., 4.9)]
    return costmap.update(observation(floor=floor, obstacles=obstacles, classes=classes), robot, 0.)


class CostmapObstacleTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()

    def test_box_in_lane_becomes_obstacle_grown_away_from_robot(self):
        # The camera sees only a 0.1 m deep front face of a 0.6 m wide box at y=3.
        grid = snapshot([(-.3, 3., .3, 3.1)])
        found = extract(grid, (0., 0.), self.cfg)
        self.assertEqual(len(found), 1)
        (lo, hi), = [o['bounds'] for o in found]
        self.assertEqual(found[0]['kind'], 'static'); self.assertEqual(found[0]['source'], 'costmap')
        self.assertAlmostEqual(lo[1], 3., places=6)                       # near face kept
        self.assertGreaterEqual(hi[1]-lo[1], self.cfg['min_object_depth_m']-1e-9)   # hidden depth added behind it
        self.assertLessEqual(lo[0], -.3+1e-9); self.assertGreaterEqual(hi[0], .3-1e-9)

    def test_person_label_speckle_machine_and_robot_cells(self):
        grid = snapshot([(-.2, 2., .2, 2.1), (1.2, 1.2, 1.3, 1.3), (2.6, 1., 2.9, 1.5), (-.1, .0, .1, .1)],
                        classes={0: 0}, floor=[(-2., -.5, 2., 4.9)])
        found = extract(grid, (0., 0.), self.cfg)
        self.assertEqual([o['kind'] for o in found], ['human'])          # speckle, machine zone, own feet dropped
        lo, hi = found[0]['bounds']
        self.assertGreaterEqual(hi[0]-lo[0], self.cfg['min_human_size_m']-1e-9)
        self.assertGreaterEqual(hi[1]-lo[1], self.cfg['min_human_size_m']-1e-9)

    def test_costmap_obstacle_drives_the_bypass_planner(self):
        """An object not in any USD scene list is bypassed purely from depth data."""
        grid = snapshot([(-.3, 4.5, .3, 4.6)])
        obstacles = extract(grid, (0., 0.), self.cfg)
        radius = LaneMap(LANE).radius
        result = plan((0., 0.), (0., 15.), (0., 0.), obstacles, factory, radius, ramp_slope=self.cfg['ramp_slope'])
        self.assertIsNotNone(result)
        lo, hi = obstacles[0]['bounds']
        pass_a, pass_b, _ = result['points']
        self.assertLess(pass_a[1], lo[1]); self.assertGreater(pass_b[1], hi[1])
        self.assertGreater(abs(result['offset_m']), radius)


class EarlyDiagonalTests(unittest.TestCase):
    def test_ramp_slope_starts_lane_change_earlier(self):
        radius = LaneMap(LANE).radius
        person = {'id': 'p', 'kind': 'human', 'bounds': [[-.3, 7.7], [.3, 8.3]]}
        steep = plan((0., 0.), (0., 15.), (0., 0.), [person], factory, radius)
        early = plan((0., 0.), (0., 15.), (0., 0.), [person], factory, radius, ramp_slope=.6)
        self.assertLess(early['start_s'], steep['start_s'])
        entry, first = early['path'][0], early['path'][1]
        self.assertLessEqual(abs(first[0]-entry[0])/(first[1]-entry[1]), .6+1e-6)   # gentle diagonal

    def test_path_clear_detects_new_obstacle_on_remaining_path(self):
        radius = LaneMap(LANE).radius
        person = {'id': 'p', 'kind': 'human', 'bounds': [[-.3, 7.7], [.3, 8.3]]}
        result = plan((0., 0.), (0., 15.), (0., 4.), [person], factory, radius, ramp_slope=.6)
        self.assertTrue(path_clear((0., 4.), result['points'], [person], factory))
        x, y = result['points'][1]
        blocker = {'id': 'b', 'kind': 'static', 'bounds': [[x-.2, y-1.2], [x+.2, y-.8]]}
        self.assertFalse(path_clear((0., 4.), result['points'], [person, blocker], factory))


class CheckpointGateTests(unittest.TestCase):
    def test_person_on_pass_through_checkpoint_is_passed_inside_the_gate(self):
        radius = LaneMap(LANE).radius
        person = {'id': 'p', 'kind': 'human', 'bounds': [[-.3, 9.7], [.3, 10.3]]}
        # Old behaviour: the checkpoint at s=10 is the edge end -> wait forever.
        self.assertIsNone(plan((0., 0.), (0., 10.), (0., 0.), [person], factory, radius))
        # Gate: plan to the next stop node (s=18) and cross the checkpoint plane off-centre.
        result = plan((0., 0.), (0., 18.), (0., 0.), [person], factory, radius, ramp_slope=.6, gates=[10.], gate_half_width=1.5)
        self.assertIsNotNone(result)
        self.assertLessEqual(abs(result['offset_m']), 1.5)
        lo, hi = person['bounds']
        self.assertGreater(box_distance(result['points'][0], person['bounds']), radius)
        # A gate narrower than the needed offset keeps the old wait behaviour.
        self.assertIsNone(plan((0., 0.), (0., 18.), (0., 0.), [person], factory, radius, gates=[10.], gate_half_width=.5))

    def test_turn_node_is_never_skipped(self):
        radius = LaneMap(LANE).radius
        person = {'id': 'p', 'kind': 'human', 'bounds': [[-.3, 17.7], [.3, 18.3]]}
        self.assertIsNone(plan((0., 0.), (0., 18.), (0., 0.), [person], factory, radius, gates=[10.], gate_half_width=1.5))


class ControllerHelperTests(unittest.TestCase):
    def controller(self, people, extra=()):
        from navigation.multi_route_controller import MultiRouteController
        corridor = SimpleNamespace(a=(0., 0.), b=(0., 10.), extra=list(extra))
        fake = SimpleNamespace(bypass_cfg=load_config(), edge={'grid': corridor}, sensor_obstacles=people)
        fake.bypass_frame = lambda: MultiRouteController.bypass_frame(fake)
        return fake, MultiRouteController

    def test_bypass_frame_extends_through_collinear_checkpoints(self):
        fake, cls = self.controller([], extra=[((0., 10.), (0., 18.)), ((0., 18.), (0., 25.))])
        a, end, gates = cls.bypass_frame(fake)
        self.assertEqual(end, (0., 25.)); self.assertEqual(gates, [10., 18.])
        fake, cls = self.controller([])
        self.assertEqual(cls.bypass_frame(fake), ((0., 0.), (0., 10.), []))

    def test_human_slowdown_is_gradual_and_ignores_people_behind(self):
        slow = load_config()['human_slowdown']
        def cap_at(y):
            fake, cls = self.controller([{'kind': 'human', 'bounds': [[-.25, y-.25], [.25, y+.25]]}])
            return cls.human_approach_cap(fake, (0., 0., math.pi/2), .6)
        self.assertEqual(cap_at(5.), .6)
        middle = cap_at(.25+(slow['start_m']+slow['end_m'])/2)
        self.assertTrue(slow['min_speed_m_s'] < middle < .6)
        self.assertAlmostEqual(cap_at(.25+slow['end_m']*.8), slow['min_speed_m_s'])
        self.assertEqual(cap_at(-1.5), .6)        # already passed

    def test_closed_loop_bypass_passes_person_and_rejoins_centreline(self):
        """Kinematic loop through the real controller method (no simulator)."""
        from navigation.multi_route_controller import MultiRouteController
        from navigation.route_graph import CorridorGrid
        lane = LaneMap(LANE)
        corridor = CorridorGrid(SimpleNamespace(map=lane, resolution=.1), (0., 0.), (0., 15.), .4)
        person = {'id': 'p', 'kind': 'human', 'bounds': [[-.25, 7.75], [.25, 8.25]]}
        events = []
        fake = SimpleNamespace(dynamic_enabled=True, edge={'id': 'E', 'grid': corridor}, bypass_cfg=load_config(),
                               sensor_obstacles=[person], map_config=LANE, detector=SimpleNamespace(floor=None),
                               sim_time=0., now=0., progress_time=0., log_event=events.append,
                               local=SimpleNamespace(path=[(0., y/5) for y in range(76)], index=0))
        for name in ('bypass_frame', 'sync_local_progress'):
            setattr(fake, name, getattr(MultiRouteController, name).__get__(fake))
        x, y, yaw = 0., 0., math.pi/2
        dt = .1; min_clear = math.inf; max_offset = 0.; activated = None
        for step in range(800):
            fake.sim_time = fake.now = step*dt
            grid = {'sim_time': fake.sim_time}
            command = MultiRouteController.obstacle_bypass_command(fake, (x, y, yaw), .3, grid)
            if command is None:
                command = [.3, 0., 0.]      # nominal tracker: straight along +Y
                if activated is not None and fake.edge.get('bypass') is None:
                    break
            elif activated is None:
                activated = y
            vx, vy, rate = command
            x += (math.cos(yaw)*vx-math.sin(yaw)*vy)*dt; y += (math.sin(yaw)*vx+math.cos(yaw)*vy)*dt; yaw += rate*dt
            min_clear = min(min_clear, box_distance((x, y), person['bounds']))
            max_offset = max(max_offset, abs(x))
        self.assertIsNotNone(activated)
        self.assertLess(activated, 6.)                      # lane change began well before the person
        self.assertGreater(y, 8.25)                         # passed the person
        self.assertLess(abs(x), .15)                        # back on the centreline
        self.assertGreater(min_clear, lane.radius)          # footprint never touched the person
        self.assertGreater(max_offset, lane.radius)
        self.assertTrue(any('obstacle_bypass' in e for e in events))


if __name__ == '__main__':
    unittest.main()
