"""Isaac Sim GUI runtime for bounded keyboard velocity -> existing ONNX policy."""
from pathlib import Path
import json,math,re,time
from datetime import datetime
import numpy as np
from controllers.keyboard_controller import KeyboardController,CarbKeyboardInput
from controllers.g1_locomotion import G1Locomotion,inverse_rotate_wxyz

PROJECT_ROOT=Path(r'D:\unitree_g1')

class G1KeyboardRuntime:
    def __init__(self,app,logical_test_keyboard=False,scene_setup=None,runtime_config=None):
        import omni.kit.app,omni.timeline,omni.appwindow
        import carb.input
        from isaacsim.core.simulation_manager import SimulationManager
        from omni.physx import get_physx_interface
        from g1_scene import build_g1_scene
        self.app=app
        self.timeline=omni.timeline.get_timeline_interface()
        self.manager=SimulationManager
        self.physx=get_physx_interface()
        self.test_mode=logical_test_keyboard
        self.synthetic_focus=True
        runtime_config=runtime_config or {}
        self.loco_cfg=json.loads((PROJECT_ROOT/runtime_config.get('locomotion_config','configs/locomotion.yaml')).read_text())
        self.loco_cfg.update(runtime_config.get('locomotion',{}))
        self.key_cfg=json.loads((PROJECT_ROOT/'configs/keyboard.yaml').read_text())
        self.key_cfg.update(runtime_config.get('keyboard',{}))
        self.deploy=json.loads((PROJECT_ROOT/self.loco_cfg['deploy_config']).read_text())
        robot=json.loads((PROJECT_ROOT/'configs/g1_robot.yaml').read_text())
        robot['physics_hz']=self.loco_cfg['physics_hz']
        robot['floor_size_m']=self.loco_cfg.get('floor_size_m',12)
        robot['scene_output']=runtime_config.get('scene_output','assets/robot/g1_keyboard_flat.usd')
        groups=robot['actuator_groups']
        new_groups={}
        for index,name in enumerate(robot['sdk_joint_order']):
            group=next(g for g in groups.values() if any(re.fullmatch(p,name) for p in g['joint_names_expr']))
            new_groups[name]={**group,'joint_names_expr':[re.escape(name)],'stiffness':self.deploy['stiffness'][index],'damping':self.deploy['damping'][index]}
        robot['actuator_groups']=new_groups
        robot['initial_joint_position_rules_rad']={re.escape(n):self.deploy['default_joint_pos'][i] for i,n in enumerate(self.loco_cfg['policy_joint_names'])}
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.pip.onnx',True)
        self.context=build_g1_scene(app,robot)
        self.stage=self.context['stage']
        if scene_setup is not None: scene_setup(self)
        self.initial_layer=self.stage.GetRootLayer().ExportToString()
        self.directory=PROJECT_ROOT/'logs/keyboard_runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
        self.directory.mkdir(parents=True,exist_ok=True)
        self.event_file=(self.directory/'events.jsonl').open('w',encoding='utf-8')
        self.telemetry_file=(self.directory/'telemetry.jsonl').open('w',encoding='utf-8')
        self.keys=KeyboardController(self.key_cfg,self.deploy)
        self.reset_events=[]
        self.steps=0
        self.settle_time=0.0
        self.policy_status='Initializing'
        self.fault=None
        self.last_running=True
        self.input=carb.input.acquire_input_interface()
        self.provider=self.input.get_input_provider()
        self.logical_keyboard=self.provider.create_keyboard('G1 automatic integration test') if self.test_mode else None
        self.hud=None
        self._initialize_policy()
        self._build_hud()
        focus=(lambda:self.synthetic_focus) if self.test_mode else self._surface_focused
        self.binding=CarbKeyboardInput(self.keys,keyboard=self.logical_keyboard,focus_provider=focus,event_logger=self._log_event)
        self.last_status_time=0
        self.closed=False
        self.update_status(force=True)

    def _initialize_policy(self):
        from isaacsim.core.experimental.prims import Articulation
        self.articulation=Articulation(self.context['root_path'])
        self.manager.set_physics_dt(1/self.loco_cfg['physics_hz'])
        self.timeline.play()
        self.app.update()
        self.timeline.set_auto_update(False)
        if self.loco_cfg.get('backend')=='unitree_mjlab':
            from controllers.g1_mjlab_locomotion import G1MjlabLocomotion
            self.policy=G1MjlabLocomotion(self.articulation,self.loco_cfg)
        else:
            self.policy=G1Locomotion(self.articulation,self.loco_cfg)
        self.policy.reset()
        self.last_state=self.policy.feedback.read_feedback()
        self.policy_status='RUNNING'

    def _build_hud(self):
        import omni.ui as ui
        compact=bool(getattr(self,'factory_cfg',{}).get('head_camera',{}).get('enabled',False))
        self.hud=ui.Window('G1 Keyboard Control',width=410,height=600)
        # Keep the HUD over the inspector column rather than over the robot.
        import omni.appwindow
        self.hud.position_x=max(16,omni.appwindow.get_default_app_window().get_width()-430)
        self.hud.position_y=64
        with self.hud.frame:
            with ui.VStack(spacing=5):
                ui.Label('G1 | WASD + ARROW KEYS',height=22 if compact else 28)
                self.label_keys=ui.Label('',height=20 if compact else 25)
                self.label_target=ui.Label('',height=36 if compact else 42,word_wrap=True)
                self.label_pose=ui.Label('',height=45 if compact else 55,word_wrap=True)
                self.label_state=ui.Label('',height=45 if compact else 55,word_wrap=True)
                self.label_note=ui.Label('',height=50 if compact else 75,word_wrap=True)
                with ui.HStack(height=30):
                    ui.Button('Pause / Resume',clicked_fn=self.toggle_pause)
                    ui.Button('Stop [X]',clicked_fn=self.stop_command)
                    ui.Button('Reset [R]',clicked_fn=self.keys.request_reset)
                ui.Button('Enable / Disable keyboard capture',height=30,clicked_fn=lambda:self.keys.set_enabled(not self.keys.enabled))
                turning=('A/Left and D/Right: turn in place.\nPure yaw uses vx=vy=0.' if self.loco_cfg.get('backend')=='unitree_mjlab' else
                    'A/D alone: in-place turning not achieved.\nHold W+A or W+D for walking turns.')
                follow_keys='/'.join(k for k,v in self.keys.mapping.items() if v=='follow_camera')
                stop_keys='/'.join(k for k,v in self.keys.mapping.items() if v=='stop')
                ui.Label('W/Up forward | S/Down backward\nA/Left yaw+ | D/Right yaw- | Q/E sideways\n'+turning+'\n'+follow_keys+': follow camera. '+stop_keys+': stop.',word_wrap=True,height=70 if compact else 90)
                if hasattr(self,'build_camera_controls'): self.build_camera_controls()
                if hasattr(self,'build_controller_controls'): self.build_controller_controls()

    def _surface_focused(self):
        import omni.appwindow
        from omni.kit.viewport.utility import get_active_viewport_window
        if not omni.appwindow.get_default_app_window().is_focused(): return False
        viewport=get_active_viewport_window()
        return bool((viewport is not None and viewport.focused) or (self.hud is not None and self.hud.focused))

    def _log_event(self,event):
        event['wall_time']=time.time()
        event['simulation_control_step']=self.steps
        self.event_file.write(json.dumps(event)+'\n')
        self.event_file.flush()

    def toggle_pause(self):
        if self.timeline.is_playing(): self.timeline.pause()
        else: self.timeline.play()

    def stop_command(self):
        self.keys.handle_key('X','PRESS')
        self.keys.handle_key('X','RELEASE')

    def inject(self,key,kind='PRESS'):
        """Test-only logical Carb device, not an OS key or fake policy."""
        if not self.test_mode: raise RuntimeError('Input injection is restricted to explicit self-test mode')
        import carb.input
        types={'PRESS':carb.input.KeyboardEventType.KEY_PRESS,'RELEASE':carb.input.KeyboardEventType.KEY_RELEASE,'REPEAT':carb.input.KeyboardEventType.KEY_REPEAT}
        self.provider.buffer_keyboard_key_event(self.logical_keyboard,types[kind],getattr(carb.input.KeyboardInput,key),0)
        self.provider.update_keyboard(self.logical_keyboard)
        self.input.distribute_buffered_events()

    def safe_reset(self,source='R'):
        state=self.last_state
        linear=np.linalg.norm(state['base_linear_velocity_world_m_s'])
        angular=np.linalg.norm(state['base_angular_velocity_world_rad_s'])
        paused=not self.timeline.is_playing()
        if not (paused or (linear<self.key_cfg['reset_max_linear_speed'] and angular<self.key_cfg['reset_max_angular_speed'] and self.settle_time>=self.key_cfg['reset_settle_seconds'])):
            self.policy_status='RESET WAIT: stopping first'
            return False
        was_running=self.timeline.is_playing()
        self.keys.clear('Reset in progress: release keys')
        self.policy_status='RESETTING'
        self.timeline.stop()
        self.app.update()
        self.policy=None
        self.articulation=None
        if not self.stage.GetRootLayer().ImportFromString(self.initial_layer): raise RuntimeError('Initial scene reset failed')
        self.app.update()
        self._initialize_policy()
        self.keys.reset_requested=False
        self.keys.clear('Reset complete: press keys again')
        self.settle_time=0
        self.fault=None
        record={'source':source,'paused_before_reset':paused,'safe_before_reset':paused or (linear<self.key_cfg['reset_max_linear_speed'] and angular<self.key_cfg['reset_max_angular_speed']),
                'position_after_reset':self.last_state['base_position_m'],'orientation_after_reset_wxyz':self.last_state['base_orientation_wxyz'],
                'joint_positions_after_reset':self.last_state['joint_positions_rad'],'history_terms_after_reset':len(self.policy.history),'policy_steps_after_reset':self.policy.count,
                'gait_phase_after_reset':getattr(self.policy,'phase',None)}
        self.reset_events.append(record)
        self._log_event({'reset':record})
        if not was_running:
            self.timeline.pause()
            self.app.update()
        self.last_running=self.timeline.is_playing()
        return True

    def tick(self):
        dt=1/self.loco_cfg['control_hz']
        self.binding.refresh_focus()
        running=self.timeline.is_playing()
        self.keys.set_running(running)
        if running and not self.last_running: self.policy.reset()
        self.last_running=running
        if self.keys.reset_requested:
            if self.safe_reset():
                self.update_status(force=True)
                return None
        if not running or self.fault:
            self.update_status()
            return None
        command=self.select_motion_command(dt)
        state=self.policy.step(command)
        up=inverse_rotate_wxyz(state['base_orientation_wxyz'],[0,0,1])
        tilt=math.degrees(math.acos(float(np.clip(up[2],-1,1))))
        if tilt>65 or state['base_position_m'][2]<.4:
            self.fault='Robot fallen: paused; use R to reset'
            self.keys.clear(self.fault)
            self.timeline.pause()
            self.last_state=state
            self.update_status(force=True)
            return None
        for _ in range(self.loco_cfg['decimation']): self.manager.step()
        self.physx.update_transformations(False,True,True,False)
        self.steps+=1
        self.last_state=self.policy.feedback.read_feedback()
        speed=np.linalg.norm(self.last_state['base_linear_velocity_world_m_s'])
        angular=np.linalg.norm(self.last_state['base_angular_velocity_world_rad_s'])
        if np.linalg.norm(command)<1e-6 and speed<self.key_cfg['reset_max_linear_speed'] and angular<self.key_cfg['reset_max_angular_speed']:
            self.settle_time+=dt
        else: self.settle_time=0
        if not self.keys.reset_requested: self.policy_status='RUNNING' if np.linalg.norm(command)>0 else 'STAND / BALANCE'
        state.update({'keys':sorted(self.keys.pressed),'desired_command':self.keys.desired(),'applied_command':command,
                      'keyboard_focused':self.keys.focused,'tilt_deg':tilt,'keyboard_step':self.steps})
        state.pop('observation',None)
        self.telemetry_file.write(json.dumps(state)+'\n')
        if self.steps%10==0: self.telemetry_file.flush()
        self.update_status()
        return state

    def select_motion_command(self,dt):
        """One command source; subclasses may select navigation before policy inference."""
        return self.keys.tick(dt)

    def snapshot(self):
        running=self.timeline.is_playing()
        return {'wall_time':time.time(),'pressed_keys':sorted(self.keys.held_keys),'desired_command':self.keys.desired(),'applied_command':self.keys.command,
                'position':self.last_state['base_position_m'],'orientation_wxyz':self.last_state['base_orientation_wxyz'],
                'policy_status':self.fault or (self.policy_status if running else 'PAUSED / STOPPED'),
                'simulation_state':'PLAYING' if running else ('STOPPED' if self.timeline.is_stopped() else 'PAUSED'),
                'focused':self.keys.focused,'capture_enabled':self.keys.enabled,'reason':self.keys.reason,'event_count':self.keys.event_count,
                'input_subscription_active':self.binding.subscription is not None,'reset_count':len(self.reset_events),
                'native_keyboard':not self.test_mode,'run_directory':str(self.directory),'turn_warning':self.turn_warning()}

    def turn_warning(self):
        if self.loco_cfg.get('backend')=='unitree_mjlab': return ''
        command=self.keys.desired()
        if abs(command[2])>0 and abs(command[0])+abs(command[1])<1e-6:
            return 'Policy: in-place turn not achieved. Use W+A / W+D.'
        return ''

    def update_status(self,force=False):
        now=time.perf_counter()
        if not force and now-self.last_status_time<.15: return
        self.last_status_time=now
        status=self.snapshot()
        self.label_keys.text='Keys: '+(', '.join(status['pressed_keys']) or '(none)')
        self.label_target.text='Target [vx, vy, yaw]: '+str([round(v,3) for v in status['applied_command']])+'\nDesired: '+str([round(v,3) for v in status['desired_command']])
        self.label_pose.text='World xyz: '+str([round(v,3) for v in status['position']])+'\nQuaternion WXYZ: '+str([round(v,3) for v in status['orientation_wxyz']])
        self.label_state.text='Policy: '+status['policy_status']+'\nSimulation: '+status['simulation_state']+' | Focus: '+str(status['focused'])+' | Capture: '+str(status['capture_enabled'])
        self.label_note.text=status['reason']+('\n'+status['turn_warning'] if status['turn_warning'] else '')
        (self.directory/'live_status.json').write_text(json.dumps(status,indent=2),encoding='utf-8')
        if not self.test_mode:
            (PROJECT_ROOT/'logs/keyboard_live_status.json').write_text(json.dumps(status,indent=2),encoding='utf-8')

    def close(self):
        if self.closed: return
        self.closed=True
        self.binding.close()
        self.timeline.stop()
        self.app.update()
        self.hud.visible=False
        self.hud.destroy()
        self.event_file.close()
        self.telemetry_file.close()
        if self.logical_keyboard is not None: self.provider.destroy_keyboard(self.logical_keyboard)
