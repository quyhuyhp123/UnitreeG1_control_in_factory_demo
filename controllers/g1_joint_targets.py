"""Simulation-only joint target adapter. Radians, explicit names, no network I/O."""
import numpy as np

def as_numpy(value):
    if hasattr(value,'numpy'): return np.asarray(value.numpy())
    return np.asarray(value)

class G1JointTargets:
    def __init__(self, articulation, expected_names):
        self.articulation=articulation
        self.names=list(articulation.dof_names)
        self.index={name:i for i,name in enumerate(self.names)}
        if len(self.index)!=29 or set(self.names)!=set(expected_names):
            raise ValueError('Runtime joint names do not match the 29-DOF contract')
        lower,upper=articulation.get_dof_limits()
        self.lower=as_numpy(lower).reshape(-1)
        self.upper=as_numpy(upper).reshape(-1)

    def set_joint_targets(self, targets_rad, joint_names):
        """Set PD targets only; never teleports base or joint positions.

        A subset or any policy ordering is accepted when names are explicit.
        Unknown/duplicate names, NaN/Inf, wrong sizes and out-of-limit targets fail.
        """
        names=list(joint_names)
        values=np.asarray(targets_rad,dtype=np.float32)
        if values.ndim!=1 or len(values)!=len(names) or not names:
            raise ValueError('Expected one target per named joint')
        if len(set(names))!=len(names) or any(n not in self.index for n in names):
            raise ValueError('Duplicate or unknown joint name')
        indices=np.asarray([self.index[n] for n in names],dtype=np.int32)
        if not np.isfinite(values).all(): raise ValueError('Nonfinite joint target')
        if np.any(values<self.lower[indices]) or np.any(values>self.upper[indices]):
            raise ValueError('Joint target outside hard limits')
        self.articulation.set_dof_position_targets(values.tolist(),dof_indices=indices.tolist())
        return indices.tolist()

    def read_feedback(self):
        position,orientation=self.articulation.get_world_poses()
        linear,angular=self.articulation.get_velocities()
        return {'base_position_m':as_numpy(position).reshape(-1).tolist(),
                'base_orientation_wxyz':as_numpy(orientation).reshape(-1).tolist(),
                'base_linear_velocity_world_m_s':as_numpy(linear).reshape(-1).tolist(),
                'base_angular_velocity_world_rad_s':as_numpy(angular).reshape(-1).tolist(),
                'joint_positions_rad':as_numpy(self.articulation.get_dof_positions()).reshape(-1).tolist(),
                'joint_velocities_rad_s':as_numpy(self.articulation.get_dof_velocities()).reshape(-1).tolist(),
                'joint_names':self.names}
