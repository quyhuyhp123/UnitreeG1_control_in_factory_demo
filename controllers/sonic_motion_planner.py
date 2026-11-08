"""Default SONIC ONNX planner, offline validation only; never writes robot targets."""
import time
import numpy as np


class SonicMotionPlanner:
    def __init__(self, path):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), sess_options=options,
                                           providers=['CPUExecutionProvider'])
        self.last_latency_ms = None

    def predict_idle(self, context):
        """Four actual consecutive state samples, idle mode 0.

        Extra input defaults match localmotion_kplanner_onnx.hpp InitializeSpecific:
        no waypoints, height disabled, first six token horizons enabled.
        Seed 1234 matches LocalMotionPlannerConfig.initial_random_seed.
        """
        context = np.asarray(context, dtype=np.float32)
        if context.shape != (1,4,36) or not np.isfinite(context).all():
            raise ValueError('Expected finite [1,4,36] real-state context')
        if not np.allclose(np.linalg.norm(context[...,3:7], axis=-1), 1, atol=1e-4):
            raise ValueError('Planner context quaternions must be normalized WXYZ')
        w,x,y,z = context[0,-1,3:7]
        yaw = np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        feed = {
            'context_mujoco_qpos': context,
            'target_vel': np.array([-1],np.float32), 'mode': np.array([0],np.int64),
            'movement_direction': np.zeros((1,3),np.float32),
            'facing_direction': np.array([[np.cos(yaw),np.sin(yaw),0]],np.float32),
            'height': np.array([-1],np.float32), 'random_seed': np.array([1234],np.int64),
            'has_specific_target': np.zeros((1,1),np.int64),
            'specific_target_positions': np.zeros((1,4,3),np.float32),
            'specific_target_headings': np.zeros((1,4),np.float32),
            'allowed_pred_num_tokens': np.array([[1]*6+[0]*5],np.int64),
        }
        if {x.name for x in self.session.get_inputs()} != set(feed):
            raise ValueError('Planner input contract differs from audited default release')
        start = time.perf_counter()
        qpos, count = self.session.run(['mujoco_qpos','num_pred_frames'],feed)
        self.last_latency_ms = (time.perf_counter()-start)*1000
        n = int(np.asarray(count).item())
        if qpos.shape != (1,64,36) or not 1 <= n <= 64 or not np.isfinite(qpos[:,:n]).all():
            raise ValueError('Invalid planner output')
        if not np.allclose(np.linalg.norm(qpos[:,:n,3:7],axis=-1),1,atol=.01):
            raise ValueError('Planner returned invalid root quaternion')
        return qpos[:,:n].copy(), n
