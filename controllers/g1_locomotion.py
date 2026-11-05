"""Unitree v0 velocity ONNX port to Isaac Sim state/targets. No robot networking."""
from collections import deque
from pathlib import Path
import hashlib,json,time
import numpy as np
from controllers.g1_joint_targets import G1JointTargets,as_numpy

PROJECT_ROOT=Path(r'D:\unitree_g1')

def inverse_rotate_wxyz(quaternion,vector):
    q=np.asarray(quaternion,dtype=np.float64)
    length=np.linalg.norm(q)
    if not np.isfinite(length) or abs(length-1)>.05: raise ValueError('Invalid base quaternion')
    q=q/length
    v=np.asarray(vector,dtype=np.float64)
    return v-2*q[0]*np.cross(q[1:],v)+2*np.cross(q[1:],np.cross(q[1:],v))

class G1Locomotion:
    def __init__(self,articulation,config):
        import onnxruntime as ort
        self.config=config
        self.deploy=json.loads((PROJECT_ROOT/config['deploy_config']).read_text(encoding='utf-8'))
        self.policy_names=list(config['policy_joint_names'])
        self.feedback=G1JointTargets(articulation,self.policy_names)
        self.articulation=articulation
        self.indices=[self.feedback.index[n] for n in self.policy_names]
        if len(self.indices)!=29 or len(set(self.indices))!=29: raise ValueError('Invalid joint mapping')
        self.default=np.asarray(self.deploy['default_joint_pos'],dtype=np.float32)
        self.actions_cfg=self.deploy['actions']['JointPositionAction']
        self.scale=np.asarray(self.actions_cfg['scale'],dtype=np.float32)
        self.offset=np.asarray(self.actions_cfg['offset'],dtype=np.float32)
        self.previous_action=np.zeros(29,dtype=np.float32)
        self.history={}
        self.count=0
        checkpoint=PROJECT_ROOT/config['checkpoint']
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=config['checkpoint_sha256']:
            raise ValueError('Checkpoint hash mismatch')
        options=ort.SessionOptions()
        options.intra_op_num_threads=2
        options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(checkpoint),sess_options=options,providers=['CPUExecutionProvider'])
        inputs,outputs=self.session.get_inputs(),self.session.get_outputs()
        if len(inputs)!=1 or inputs[0].shape!=[1,480] or outputs[0].shape!=[1,29]: raise ValueError('Policy contract mismatch')
        self.input_name=inputs[0].name
        self.output_name=outputs[0].name
        self.command_low=np.array([self.deploy['commands']['base_velocity']['ranges'][k][0] for k in ['lin_vel_x','lin_vel_y','ang_vel_z']],dtype=np.float32)
        self.command_high=np.array([self.deploy['commands']['base_velocity']['ranges'][k][1] for k in ['lin_vel_x','lin_vel_y','ang_vel_z']],dtype=np.float32)

    def reset(self):
        self.previous_action[:]=0
        self.history.clear()
        self.count=0

    def make_observation(self,state,command):
        command=np.asarray(command,dtype=np.float32)
        if command.shape!=(3,) or not np.isfinite(command).all(): raise ValueError('Invalid velocity command')
        if np.any(command<self.command_low) or np.any(command>self.command_high): raise ValueError('Velocity command outside policy range')
        order=[state['joint_names'].index(n) for n in self.policy_names]
        terms={
            'base_ang_vel':inverse_rotate_wxyz(state['base_orientation_wxyz'],state['base_angular_velocity_world_rad_s']),
            'projected_gravity':inverse_rotate_wxyz(state['base_orientation_wxyz'],[0,0,-1]),
            'velocity_commands':command,
            'joint_pos_rel':np.asarray(state['joint_positions_rad'])[order]-self.default,
            'joint_vel_rel':np.asarray(state['joint_velocities_rad_s'])[order],
            'last_action':self.previous_action.copy(),
        }
        parts=[]
        for name,cfg in self.deploy['observations'].items():
            row=np.asarray(terms[name],dtype=np.float32)
            if cfg['clip'] is not None: row=np.clip(row,*cfg['clip'])
            row=row*np.asarray(cfg['scale'],dtype=np.float32)
            length=cfg['history_length']
            if name not in self.history:
                self.history[name]=deque([row.copy() for _ in range(length)],maxlen=length)
            else: self.history[name].append(row.copy())
            # Official default use_gym_history=false: all time slots of each
            # term, then the next term. Within each term oldest -> newest.
            parts.extend(list(self.history[name]))
        observation=np.concatenate(parts).astype(np.float32)[None,:]
        if observation.shape!=(1,480) or not np.isfinite(observation).all(): raise ValueError('Invalid observation')
        return observation

    def step(self,command):
        state=self.feedback.read_feedback()
        observation=self.make_observation(state,command)
        started=time.perf_counter()
        action=self.session.run([self.output_name],{self.input_name:observation})[0].reshape(29).astype(np.float32)
        inference_ms=1000*(time.perf_counter()-started)
        if not np.isfinite(action).all(): raise ValueError('Nonfinite policy action')
        target=action*self.scale+self.offset
        if self.actions_cfg['clip'] is not None:
            bounds=np.asarray(self.actions_cfg['clip'])
            target=np.clip(target,bounds[:,0],bounds[:,1])
        # No additional arbitrary action clipping: deploy clip is null. Physics
        # hard limits and torque limits remain active on the articulation.
        self.articulation.set_dof_position_targets(target.tolist(),dof_indices=self.indices)
        self.previous_action=action.copy()
        self.count+=1
        state.update({'command_body_m_s_rad_s':list(command),'action_policy_order':action.tolist(),
                      'target_policy_order_rad':target.tolist(),'observation':observation.reshape(-1).tolist(),
                      'inference_ms':inference_ms,'control_step':self.count,
                      'base_linear_velocity_body_m_s':inverse_rotate_wxyz(state['base_orientation_wxyz'],state['base_linear_velocity_world_m_s']).tolist()})
        return state
