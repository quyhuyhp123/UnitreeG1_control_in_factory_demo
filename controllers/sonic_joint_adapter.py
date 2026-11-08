"""Audited SONIC index permutations. No Articulation writes in this module.

Source: NVIDIA policy_parameters.hpp at pinned source commit in configs/sonic.yaml.
SDK/MuJoCo order is the existing g1_robot.yaml sdk_joint_order; policy is breadth-first.
"""
import numpy as np

POLICY_FROM_SDK = np.array([0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,
                           16,23,5,11,17,24,18,25,19,26,20,27,21,28])
SDK_FROM_POLICY = np.argsort(POLICY_FROM_SDK)


def reorder(values, source_names, target_names):
    if len(source_names) != 29 or len(set(source_names)) != 29:
        raise ValueError('Expected 29 unique source joint names')
    if len(target_names) != 29 or set(source_names) != set(target_names):
        raise ValueError('Joint name sets differ')
    a = np.asarray(values, dtype=np.float32)
    if a.shape[-1] != 29 or not np.isfinite(a).all():
        raise ValueError('Invalid joint tensor')
    return a[..., [source_names.index(n) for n in target_names]]
