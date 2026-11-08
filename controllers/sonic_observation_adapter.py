"""Verified coordinate primitives only; full SONIC encoder adapter remains gated.

Planner qpos is world Z-up, xyz + normalized WXYZ + MuJoCo tree-order joints.
Do not replace the 1762-D encoder reference by velocity or padded observations.
"""
import numpy as np
from controllers.sonic_joint_adapter import reorder


def normalize_wxyz(value):
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid quaternion')
    return q / np.linalg.norm(q)


def wxyz_to_xyzw(q):
    return normalize_wxyz(q)[[1,2,3,0]]


def xyzw_to_wxyz(q):
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (4,):
        raise ValueError('Expected four quaternion components')
    return normalize_wxyz(q[[3,0,1,2]])


def state_to_mujoco_qpos(state, sdk_names):
    p = np.asarray(state['base_position_m'], dtype=np.float32)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError('Invalid base position')
    q = normalize_wxyz(state['base_orientation_wxyz'])
    joints = reorder(state['joint_positions_rad'], state['joint_names'], sdk_names)
    return np.concatenate([p, q, joints]).astype(np.float32)
