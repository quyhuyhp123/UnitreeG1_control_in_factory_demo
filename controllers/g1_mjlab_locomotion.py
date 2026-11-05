"""Official Unitree Mjlab G1 velocity ONNX interface, ported to PhysX feedback.

Separate from the preserved IsaacLab policy. Sources are pinned next to checkpoint.
"""
import json,hashlib,math
import numpy as np
from controllers.g1_locomotion import G1Locomotion,inverse_rotate_wxyz,PROJECT_ROOT
from controllers.g1_joint_targets import G1JointTargets

class G1MjlabLocomotion(G1Locomotion):
    def __init__(self,articulation,config):
        import onnxruntime as ort
        self.config=config
        self.deploy=json.loads((PROJECT_ROOT/config['deploy_config']).read_text())
        self.policy_names=list(config['policy_joint_names'])
        self.feedback=G1JointTargets(articulation,self.policy_names)
        self.articulation=articulation
        self.indices=[self.feedback.index[n] for n in self.policy_names]
        if len(self.indices)!=29 or len(set(self.indices))!=29 or self.deploy['joint_ids_map']!=list(range(29)):
            raise ValueError('Mjlab 29 joint identity SDK mapping mismatch')
        self.default=np.asarray(self.deploy['default_joint_pos'],dtype=np.float32)
        self.actions_cfg=self.deploy['actions']['JointPositionAction']
        self.scale=np.asarray(self.actions_cfg['scale'],dtype=np.float32)
        self.offset=np.asarray(self.actions_cfg['offset'],dtype=np.float32)
        self.previous_action=np.zeros(29,dtype=np.float32)
        self.history={}; self.count=0; self.phase=0.0
        checkpoint=PROJECT_ROOT/config['checkpoint']
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=config['checkpoint_sha256']: raise ValueError('Checkpoint hash mismatch')
        options=ort.SessionOptions(); options.intra_op_num_threads=2; options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(checkpoint),sess_options=options,providers=['CPUExecutionProvider'])
        inputs,outputs=self.session.get_inputs(),self.session.get_outputs()
        if len(inputs)!=1 or inputs[0].shape!=[1,98] or outputs[0].shape!=[1,29]: raise ValueError('Mjlab policy contract must be [1,98] -> [1,29]')
        self.input_name=inputs[0].name; self.output_name=outputs[0].name
        ranges=self.deploy['commands']['base_velocity']['ranges']
        self.command_low=np.array([ranges[k][0] for k in ['lin_vel_x','lin_vel_y','ang_vel_z']],dtype=np.float32)
        self.command_high=np.array([ranges[k][1] for k in ['lin_vel_x','lin_vel_y','ang_vel_z']],dtype=np.float32)
        if abs(self.deploy['step_dt']-1/config['control_hz'])>1e-8: raise ValueError('Mjlab control timing mismatch')

    def reset(self):
        super().reset(); self.phase=0.0

    def make_observation(self,state,command):
        command=np.asarray(command,dtype=np.float32)
        if command.shape!=(3,) or not np.isfinite(command).all() or np.any(command<self.command_low) or np.any(command>self.command_high):
            raise ValueError('Velocity command outside Mjlab policy range')
        cfg=self.deploy['observations']
        self.phase=(self.phase+self.deploy['step_dt']/cfg['gait_phase']['params']['period'])%1.0
        phase=[math.sin(2*math.pi*self.phase),math.cos(2*math.pi*self.phase)] if np.linalg.norm(command)>=.1 else [0,0]
        order=[state['joint_names'].index(n) for n in self.policy_names]
        terms={'base_ang_vel':inverse_rotate_wxyz(state['base_orientation_wxyz'],state['base_angular_velocity_world_rad_s']),
            'projected_gravity':inverse_rotate_wxyz(state['base_orientation_wxyz'],[0,0,-1]),
            'velocity_commands':command,'gait_phase':phase,
            'joint_pos_rel':np.array(state['joint_positions_rad'])[order]-self.default,
            'joint_vel_rel':np.array(state['joint_velocities_rad_s'])[order],'last_action':self.previous_action.copy()}
        parts=[]
        for name,c in cfg.items():
            if c['history_length']!=1: raise ValueError('Unexpected Mjlab observation history')
            row=np.asarray(terms[name],dtype=np.float32)
            if c['clip'] is not None: row=np.clip(row,*c['clip'])
            parts.append(row*np.asarray(c['scale'],dtype=np.float32))
        obs=np.concatenate(parts).astype(np.float32)[None,:]
        if obs.shape!=(1,98) or not np.isfinite(obs).all(): raise ValueError('Invalid Mjlab observation')
        return obs
