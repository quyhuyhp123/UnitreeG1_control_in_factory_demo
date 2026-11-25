"""Turn RGB-D costmap cells into obstacle boxes for the bypass planner.

Input is a LocalCostmap snapshot (perception/local_costmap.py). Output has the
same shape as SceneObstacleDetector obstacles ({'id','kind','bounds',...}),
so navigation.obstacle_bypass.plan() can use sensor data instead of USD
ground truth. Only OCCUPIED cells inside walkable lanes are used: walls and
machines are already in the static lane map (static_lethal), and UNKNOWN is
never treated as an obstacle here (the costmap trace still refuses to enter it).
"""
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OCCUPIED = 2   # perception.local_costmap.OCCUPIED (kept local: no scipy import at load)
SOURCES = ('costmap', 'scene', 'both')


def load_config(overrides=None):
    config = json.loads((ROOT/'configs/obstacle_bypass.json').read_text())
    config.update(overrides or {})
    if config.get('version') != 1 or config['source'] not in SOURCES:
        raise ValueError('Invalid obstacle bypass config/source')
    if not 1 <= config['min_cells'] <= 50 or not .3 <= config['checkpoint_gate_half_width_m'] <= 3:
        raise ValueError('Invalid obstacle bypass cells/gate width')
    if not .1 <= config['ramp_slope'] <= 1.5 or not .05 <= config['replan_period_s'] <= 2:
        raise ValueError('Invalid obstacle bypass ramp/replan period')
    slow = config['human_slowdown']
    if not 0 < slow['end_m'] < slow['start_m'] <= 8 or not .12 <= slow['min_speed_m_s'] <= 1:
        raise ValueError('Invalid human slowdown')
    return config


def _grow(lo, hi, size, direction):
    """Grow each axis to at least `size`; away from the robot when the axis faces it."""
    for i in (0, 1):
        need = size - (hi[i] - lo[i])
        if need <= 0:
            continue
        if abs(direction[i]) >= .5:
            # Only the near face is observed: the hidden depth lies behind it.
            if direction[i] > 0: hi[i] += need
            else: lo[i] -= need
        else:
            lo[i] -= need/2; hi[i] += need/2


def extract(grid, robot_xy, config, human_class=0, ignore_radius=.35):
    """Connected OCCUPIED in-lane cells -> obstacle dicts with world AABB bounds."""
    from scipy import ndimage
    state = grid.get('state')
    if state is None:
        return []
    occupied = (state == OCCUPIED) & ~grid['static_lethal']
    if not occupied.any():
        return []
    res = grid['resolution_m']; ox, oy = grid['origin_world_m']; n = grid['size_cells']
    centres = (np.arange(n)+.5)*res
    # Cells under the robot body are its own limbs/feet, not obstacles.
    near = np.hypot((ox+centres)[:, None]-robot_xy[0], (oy+centres)[None, :]-robot_xy[1]) <= ignore_radius
    occupied &= ~near
    labels, count = ndimage.label(occupied, structure=np.ones((3, 3), dtype=bool))
    found = []
    for index, window in enumerate(ndimage.find_objects(labels), 1):
        if window is None:
            continue
        cells = labels[window] == index
        total = int(cells.sum())
        if total < config['min_cells']:
            continue
        human = bool((grid['class_id'][window][cells] == human_class).any())
        lo = [ox+window[0].start*res, oy+window[1].start*res]
        hi = [ox+window[0].stop*res, oy+window[1].stop*res]
        centre = ((lo[0]+hi[0])/2, (lo[1]+hi[1])/2)
        dx, dy = centre[0]-robot_xy[0], centre[1]-robot_xy[1]
        norm = math.hypot(dx, dy) or 1.
        direction = (dx/norm, dy/norm)
        _grow(lo, hi, config['min_object_depth_m'], direction)
        if human:
            _grow(lo, hi, config['min_human_size_m'], direction)
        kind = 'human' if human else 'static'
        found.append({'id': f'costmap:{kind}:{centre[0]:.1f},{centre[1]:.1f}', 'kind': kind, 'source': 'costmap',
                      'bounds': [lo, hi], 'cells': total, 'dynamic': human, 'velocity': [0., 0.],
                      'timestamp': grid.get('sim_time')})
    return found
