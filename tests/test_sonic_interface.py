import unittest
import numpy as np
from controllers.sonic_observation_adapter import normalize_wxyz,wxyz_to_xyzw,xyzw_to_wxyz,state_to_mujoco_qpos
from controllers.sonic_joint_adapter import POLICY_FROM_SDK,SDK_FROM_POLICY,reorder
from controllers.sonic_command_adapter import SonicCommandAdapter
from controllers.g1_locomotion import inverse_rotate_wxyz


class SonicInterfaceTests(unittest.TestCase):
    def test_quaternion_roundtrip(self):
        q=normalize_wxyz([1,0,0,1])
        np.testing.assert_allclose(xyzw_to_wxyz(wxyz_to_xyzw(q)),q)
        np.testing.assert_allclose(inverse_rotate_wxyz(q,[0,1,0]),[1,0,0],atol=1e-7)

    def test_invalid_quaternions(self):
        for q in [[0,0,0,0],[1,2,3],[float('nan'),0,0,0]]:
            with self.assertRaises(ValueError): normalize_wxyz(q)

    def test_mapping_roundtrip(self):
        x=np.arange(29)
        np.testing.assert_equal(x[POLICY_FROM_SDK][SDK_FROM_POLICY],x)
        names=[str(i) for i in range(29)]
        np.testing.assert_equal(reorder(x,names,list(reversed(names))),x[::-1])
        with self.assertRaises(ValueError): reorder(x,names,['x']*29)

    def test_real_state_layout(self):
        names=[str(i) for i in range(29)]
        state={'base_position_m':[1,2,.8],'base_orientation_wxyz':[1,0,0,0],
               'joint_names':names[::-1],'joint_positions_rad':list(range(28,-1,-1))}
        q=state_to_mujoco_qpos(state,names)
        self.assertEqual(q.shape,(36,))
        np.testing.assert_equal(q[7:],np.arange(29))

    def test_command_coordinates_and_stop(self):
        adapter=SonicCommandAdapter()
        cmd=adapter.convert([.3,0,0],np.pi/2,.02)
        np.testing.assert_allclose(cmd['movement_direction'],[0,1,0],atol=1e-7)
        self.assertEqual(adapter.convert([0,0,0],0,.02)['mode'],0)
        self.assertEqual(adapter.convert([0,0,0],0,.02)['target_vel'],-1)
