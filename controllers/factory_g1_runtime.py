"""Compose the existing factory and G1 without editing either source asset."""
from pathlib import Path
import json,time,subprocess
import numpy as np
from controllers.g1_keyboard_runtime import G1KeyboardRuntime
ROOT=Path(r'D:\unitree_g1')

class FactoryG1Runtime(G1KeyboardRuntime):
    def build_controller_controls(self):
        import omni.ui as ui
        from controllers.controller_selector import ControllerSelector
        self.controller_selector=ControllerSelector()
        self.controller_window=ui.Window('G1 CONTROLLER MANAGER',width=350,height=350)
        self.controller_window.position_x=16
        self.controller_window.position_y=64
        with self.controller_window.frame:
            with ui.VStack(spacing=4):
                ui.Label('Keyboard Manual | Locomotion RL active',height=22)
                self.controller_choice=ui.ComboBox(0,'Locomotion RL','SONIC (blocked)',height=24)
                self.controller_message=ui.Label('Policy loaded | Planner: N/A\nSONIC motion modes unavailable until validated.',
                                                 height=130,word_wrap=True)
                self.controller_metrics=ui.Label('Waiting for runtime metrics',height=60,word_wrap=True)
                ui.Button('Apply Controller (restart required)',height=24,clicked_fn=self.apply_controller_selection)
                with ui.HStack(height=24):
                    ui.Button('Reset',clicked_fn=self.keys.request_reset)
                    ui.Button('Pause',clicked_fn=self.timeline.pause)
                    ui.Button('Resume',clicked_fn=self.timeline.play)

    def apply_controller_selection(self):
        index=self.controller_choice.model.get_item_value_model().as_int
        self.controller_message.text=self.controller_selector.request('sonic' if index else 'locomotion')

    def __init__(self,app,cfg,flat=False,self_test=False):
        self.factory_cfg=cfg
        self.flat=flat
        self.scene_checks={}
        self.contact_counts={}
        self.contact_examples=[]
        self.clock=time.perf_counter()
        self.last_metric_clock=self.clock
        self.last_metric_steps=0
        self.fps=0.0
        self.physics_rate=0.0
        self.last_gpu_clock=0
        self.gpu_samples=[]
        self.contact_subscription=None
        self.head_window=None
        self.head_widget=None
        self.last_stop_generation=0
        super().__init__(app,logical_test_keyboard=self_test,scene_setup=self._compose,
            runtime_config={'scene_output':'assets/robot/g1_factory_base.usd','locomotion_config':cfg['locomotion_config'],
                'keyboard':cfg.get('keyboard_overrides',{}),'locomotion':{
                'checkpoint':cfg['checkpoint'],'physics_hz':round(1/cfg['physics_dt']),
                'control_hz':cfg['control_hz'],'decimation':round(1/cfg['physics_dt']/cfg['control_hz'])}})
        from omni.physx import get_physx_simulation_interface
        self.contact_subscription=get_physx_simulation_interface().subscribe_contact_report_events(self._contacts)
        self.metric_file=(self.directory/'performance.jsonl').open('w',encoding='utf-8')
        self.stage.GetRootLayer().Export(str(self.directory/'factory_g1_composed.usda'))
        (self.directory/'scene_checks.json').write_text(json.dumps(self.scene_checks,indent=2),encoding='utf-8')
        self.update_camera()
        self._build_head_preview()
        self.navigation=None
        if not flat:
            from navigation.patrol_controller import PatrolController
            from navigation.patrol_ui import PatrolPanel
            self.navigation=PatrolController(self)
            self.patrol_panel=PatrolPanel(self.navigation)

    def select_motion_command(self,dt):
        nav=getattr(self,'navigation',None)
        return nav.command(dt) if nav is not None else super().select_motion_command(dt)

    def _compose(self,runtime):
        from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdLux,UsdShade
        stage=self.stage
        cfg=self.factory_cfg
        spawn=[0.0,0.0,0.0]
        source_transforms={}
        if not self.flat:
            source=ROOT/cfg['factory_usd']
            src=Usd.Stage.Open(str(source))
            if not src or not src.GetDefaultPrim(): raise RuntimeError('Missing USD default prim: factory')
            if UsdGeom.GetStageMetersPerUnit(src)!=1 or UsdGeom.GetStageUpAxis(src)!='Z': raise ValueError('Factory must be meters/Z-up')
            # Remove temporary flat builder objects before starting the timeline.
            for path in ['/World/Floor','/World/Lighting']:
                stage.RemovePrim(path)
            root=UsdGeom.Xform.Define(stage,'/World/FactoryScene').GetPrim()
            root.GetReferences().AddReference(str(source),src.GetDefaultPrim().GetPath())
            cache=UsdGeom.XformCache()
            prims=list(Usd.PrimRange(root))
            # Keep the flat-tested PhysicsScene path used by SimulationManager.
            # Deactivate only the referenced scene prim, never its colliders.
            for p in prims:
                if p.IsA(UsdPhysics.Scene): p.SetActive(False)
            self.scene_checks['referenced_physics_scene_deactivated']=True
            meshes=[p for p in prims if p.IsA(UsdGeom.Mesh)]
            if not meshes: raise RuntimeError('Missing USD factory meshes/reference dependency')
            for p in meshes:
                rel=p.GetPath().MakeRelativePath(root.GetPath())
                original=src.GetPrimAtPath(src.GetDefaultPrim().GetPath().AppendPath(rel))
                if not original or cache.GetLocalToWorldTransform(p)!=cache.GetLocalToWorldTransform(original):
                    raise RuntimeError('Factory names/transforms changed: '+str(p.GetPath()))
                source_transforms[str(p.GetPath())]=True
            markers=[p for p in prims if p.GetName().startswith(cfg['robot_spawn']['marker_prefix'])]
            if len(markers)!=1: raise ValueError('Robot_Spawn must resolve to exactly one USD prim: '+str([str(p.GetPath()) for p in markers]))
            spawn=list(cache.GetLocalToWorldTransform(markers[0]).ExtractTranslation())
            self.scene_checks['spawn_marker']=str(markers[0].GetPath())
            # Preserve existing factory lights. Supply ambient only if absent.
            if not any(p.IsA(UsdLux.DomeLight) for p in prims):
                UsdLux.DomeLight.Define(stage,'/World/FactoryAmbient').CreateIntensityAttr(500)
            # Keep the authored factory collisions/materials; never add a second floor.
            floors=[p for p in prims if p.GetName().startswith('Floor') and p.IsA(UsdGeom.Mesh)]
            if len(floors)!=1: raise ValueError('Expected exactly one factory floor mesh')
            bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
            bounds=bbox.ComputeWorldBound(floors[0]).ComputeAlignedRange()
            if abs(bounds.GetMax()[2])>1e-5: raise ValueError('Factory floor top must be Z=0')
            self.floor_path=str(floors[0].GetPath())
            apron=cfg.get('entrance_apron',{})
            if apron.get('enabled',False):
                w,l,t=[float(apron[k]) for k in ['width_m','length_m','thickness_m']]
                if not all(0<v<=10 for v in [w,l,t]): raise ValueError('Invalid entrance apron size')
                pad=UsdGeom.Cube.Define(stage,'/World/EntranceApron')
                pad.CreateSizeAttr(1)
                pad.AddTranslateOp().Set(Gf.Vec3d(0,float(bounds.GetMin()[1])-l/2,-t/2))
                pad.AddScaleOp().Set(Gf.Vec3d(w,l,t))
                UsdPhysics.CollisionAPI.Apply(pad.GetPrim())
                pxpad=PhysxSchema.PhysxCollisionAPI.Apply(pad.GetPrim())
                pxpad.CreateContactOffsetAttr(cfg['floor_contact']['contact_offset_m']); pxpad.CreateRestOffsetAttr(0)
                mat=UsdShade.Material(stage.GetPrimAtPath('/World/Materials/Floor'))
                UsdShade.MaterialBindingAPI.Apply(pad.GetPrim()).Bind(mat)
                UsdShade.MaterialBindingAPI.Apply(pad.GetPrim()).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
                self.scene_checks['entrance_apron']={'path':str(pad.GetPath()),'size_m':[w,l,t],
                    'top_z_m':0,'y_range_m':[float(bounds.GetMin()[1])-l,float(bounds.GetMin()[1])],
                    'overlaps_factory_floor':False}
            contact=cfg['floor_contact']
            if not 0<=contact['rest_offset_m']<contact['contact_offset_m']<=.05: raise ValueError('Invalid floor contact offsets')
            floor_px=PhysxSchema.PhysxCollisionAPI.Apply(floors[0])
            floor_px.CreateContactOffsetAttr(contact['contact_offset_m'])
            floor_px.CreateRestOffsetAttr(contact['rest_offset_m'])
            self.scene_checks['floor_contact_override']=contact
            self.scene_checks.update(factory_reference_loaded=True,factory_meshes=len(meshes),
                source_names_transforms_preserved=True,floor_top_z_m=float(bounds.GetMax()[2]),no_duplicate_ground=True)
        else:
            self.floor_path='/World/Floor'
            self.scene_checks['flat_comparison_mode']=True
        override=cfg['robot_spawn']['position_override_m']
        if override is not None: spawn=[float(x) for x in override]
        if len(spawn)!=3 or not np.isfinite(spawn).all() or abs(spawn[2])>1e-5: raise ValueError('Spawn must be finite xyz on Z=0 floor')
        heading=float(cfg['robot_spawn']['heading_deg'])
        if not np.isfinite(heading): raise ValueError('Spawn heading must be finite')
        self.spawn=spawn
        xf=UsdGeom.Xformable(self.context['robot'])
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(opSuffix='spawn').Set(Gf.Vec3d(spawn[0],spawn[1],self.context['height']+spawn[2]))
        xf.AddRotateZOp(opSuffix='spawnHeading').Set(heading)
        scenes=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes)!=1: raise RuntimeError('Expected one Physics Scene, found '+str(len(scenes)))
        ps=UsdPhysics.Scene(scenes[0])
        if abs(ps.GetGravityMagnitudeAttr().Get()-9.81)>1e-5 or list(ps.GetGravityDirectionAttr().Get())!=[0,0,-1]: raise ValueError('Wrong factory gravity')
        PhysxSchema.PhysxSceneAPI.Apply(scenes[0]).CreateTimeStepsPerSecondAttr(round(1/cfg['physics_dt']))
        blockers=[]
        bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'])
        for p in stage.Traverse():
            if str(p.GetPath()).startswith('/World/G1/') and p.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0)
            if self.flat or not str(p.GetPath()).startswith('/World/FactoryScene/') or not p.HasAPI(UsdPhysics.CollisionAPI): continue
            if str(p.GetPath())==self.floor_path: continue
            r=bbox.ComputeWorldBound(p).ComputeAlignedRange()
            lo,hi=r.GetMin(),r.GetMax()
            if hi[2]>.01 and lo[2]<2.1 and all(lo[i]<spawn[i]+1 and hi[i]>spawn[i]-1 for i in [0,1]): blockers.append(str(p.GetPath()))
        if blockers: raise ValueError('Robot_Spawn collider blockers: '+str(blockers))
        self.scene_checks.update(spawn_xyz_m=spawn,base_spawn_height_m=self.context['height'],spawn_blockers=blockers,
            spawn_heading_deg=heading,
            policy_backend=self.loco_cfg.get('backend','unitree_rl_lab'),checkpoint=self.loco_cfg['checkpoint'],
            physics_scene=str(scenes[0].GetPath()),gravity=[0,0,-9.81],physics_hz=round(1/cfg['physics_dt']),
            robot_count=1,robot_dof=29,flat_mode=self.flat)
        if cfg.get('head_camera',{}).get('enabled',False):
            import math
            bodies=[p for p in stage.Traverse() if p.GetName()=='torso_link' and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(bodies)!=1: raise ValueError('Head camera needs exactly one torso rigid body')
            hc=cfg['head_camera']; local_eye=Gf.Vec3d(*hc['torso_offset_m'])
            pitch=math.radians(hc['pitch_down_deg'])
            direction=Gf.Vec3d(math.cos(pitch),0,-math.sin(pitch))
            camera=UsdGeom.Camera.Define(stage,bodies[0].GetPath().AppendChild('HeadCamera'))
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(local_eye,local_eye+direction,Gf.Vec3d(0,0,1)).GetInverse())
            self.head_mount_matrix=camera.GetOrderedXformOps()[0].Get()
            camera.CreateHorizontalApertureAttr(20.955)
            camera.CreateFocalLengthAttr(20.955/(2*math.tan(math.radians(hc['horizontal_fov_deg'])/2)))
            camera.CreateClippingRangeAttr(Gf.Vec2f(.03,50))
            self.head_camera_path=str(camera.GetPath())
            self.scene_checks['head_camera']={'path':self.head_camera_path,'resolution':hc['resolution'],'horizontal_fov_deg':hc['horizontal_fov_deg']}

    def _build_head_preview(self):
        if not self.factory_cfg.get('head_camera',{}).get('enabled',False): return
        import omni.appwindow
        from omni.kit.viewport.utility import create_viewport_window
        self.head_window=create_viewport_window('G1 Head Camera | LIVE',width=410,height=240,
            position_x=max(16,omni.appwindow.get_default_app_window().get_width()-430),position_y=64,
            camera_path=self.head_camera_path,usd_drop_support=False)
        if self.head_window is None: raise RuntimeError('Head viewport creation failed')
        self.head_widget=self.head_window
        self.head_widget.viewport_api.resolution=tuple(self.factory_cfg['head_camera']['resolution'])
        self.hud.position_y=320
        self._clean_head_overlays()

    def _clean_head_overlays(self):
        """Camera preview is image-only, not a scene editing viewport."""
        if self.head_window is None: return
        import carb
        settings=carb.settings.get_settings()
        viewport_id=f'{self.head_window.name}/Viewport0'
        for key in ['guide/axis/visible','guide/grid/visible','guide/selection/visible']:
            path=f'/persistent/app/viewport/{viewport_id}/{key}'
            if settings.get(path) is not False: settings.set(path,False)
        hidden=[]
        # Kit 110's scene layer wraps each manipulator in a visible transform.
        # Hide that wrapper, including Prim Transform whose own visible setter
        # deliberately does nothing. Only this preview window is affected.
        for name in ['Prim Transform','Selection','Camera']:
            item=self.head_window._find_viewport_layer(name,'manipulator')
            if item is not None:
                if item.visible: item.visible=False
                hidden.append(name)
        self.head_hidden_overlays=hidden

    def capture_head_preview(self,path):
        if self.head_widget is None: return
        import asyncio
        from omni.kit.viewport.utility import capture_viewport_to_file
        async def wait_image():
            request=capture_viewport_to_file(self.head_widget.viewport_api,str(path))
            return await request.wait_for_result()
        task=asyncio.ensure_future(wait_image())
        for _ in range(200):
            self.app.update()
            if task.done(): break
        if not task.done(): raise RuntimeError('Head camera capture timeout')
        task.result()
        # Image encoding/file writing can finish after the GPU capture future.
        for _ in range(200):
            if path.is_file(): break
            self.app.update(); time.sleep(.005)
        if not path.is_file(): raise RuntimeError('Head camera image was not written')

    def _surface_focused(self):
        import omni.appwindow
        if not omni.appwindow.get_default_app_window().is_focused(): return False
        from omni.kit.viewport.window import get_viewport_window_instances
        main_focused=any(w.viewport_api==self.context['viewport'] and w.focused for w in get_viewport_window_instances())
        return bool(main_focused or (self.hud is not None and self.hud.focused) or (self.head_window is not None and self.head_window.focused)
                    or (getattr(self,'controller_window',None) is not None and self.controller_window.focused)
                    or (getattr(self,'patrol_panel',None) is not None and self.patrol_panel.window.focused
                        and self.navigation.mode=='AUTO PATROL'))

    def _contacts(self,headers,data):
        from pxr import PhysicsSchemaTools
        for h in headers:
            if h.num_contact_data==0: continue
            a,b=[str(PhysicsSchemaTools.intToSdfPath(v)) for v in [h.collider0,h.collider1]]
            if not (a.startswith('/World/G1/') or b.startswith('/World/G1/')): continue
            other=b if a.startswith('/World/G1/') else a
            if other.startswith('/World/G1/'): continue
            kind='floor' if other in [self.floor_path,'/World/EntranceApron'] else 'obstacle'
            self.contact_counts[kind]=self.contact_counts.get(kind,0)+int(h.num_contact_data)
            if kind=='obstacle' and len(self.contact_examples)<20:
                self.contact_examples.append({'colliders':[a,b],'contact_count':int(h.num_contact_data)})

    def update_camera(self):
        from pxr import Gf,UsdGeom
        cfg=self.factory_cfg['camera']
        # A free camera is initialized once; never overwrite mouse navigation
        # or a camera selection made by the user on subsequent physics ticks.
        if cfg['mode']=='free' and getattr(self,'free_camera_initialized',False): return
        if cfg['mode']=='follow':
            p=np.array(self.last_state['base_position_m'])
            target=np.array([p[0],p[1],cfg['look_height_m']])
            q=self.last_state['base_orientation_wxyz']
            import math
            yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))
            offset=np.array(cfg.get('follow_offset_body_m',[-2.5,-.8,1.4]))
            eye=target+np.array([math.cos(yaw)*offset[0]-math.sin(yaw)*offset[1],math.sin(yaw)*offset[0]+math.cos(yaw)*offset[1],offset[2]])
        elif cfg['mode']=='overview':
            eye=cfg['overview_eye_m']; target=cfg['overview_target_m']
        else:
            eye=cfg['free_eye_m']; target=cfg['free_target_m']
        camera=self.context['camera']
        self._set_camera_transform(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
        camera.CreateFocalLengthAttr(cfg['focal_length_mm'])
        # Reference loading may select the source USD's saved camera. Rebind
        # explicitly so the viewport actually uses the tracking transform.
        self.context['viewport'].camera_path=str(camera.GetPath())
        if not getattr(self,'viewport_resolution_set',False):
            self.context['viewport'].set_texture_resolution((self.factory_cfg['rendering']['width'],self.factory_cfg['rendering']['height']))
            self.viewport_resolution_set=True
        self.free_camera_initialized=True

    def _set_camera_transform(self,matrix):
        from pxr import UsdGeom
        # Mouse navigation may author translate/orient ops instead of the
        # original matrix op. Use the same matrix representation when taking
        # over again for Follow/Overview or restoring a saved reset view.
        xf=UsdGeom.Xformable(self.context['camera'])
        ops=xf.GetOrderedXformOps()
        if len(ops)==1 and ops[0].GetOpType()==UsdGeom.XformOp.TypeTransform:
            ops[0].Set(matrix)
        else:
            xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(matrix)

    def build_camera_controls(self):
        import omni.ui as ui
        self.hud.height=540 if self.factory_cfg.get('head_camera',{}).get('enabled',False) else 660
        with ui.HStack(height=28):
            for mode in ['free','follow','overview']:
                ui.Button('Camera: '+mode.title(),clicked_fn=lambda value=mode:self.set_camera_mode(value))
        self.label_camera=ui.Label('Camera: '+self.factory_cfg['camera']['mode'].upper()+' | mouse navigation',height=22)

    def set_camera_mode(self,mode):
        if mode not in ['free','follow','overview']: raise ValueError('Invalid camera mode')
        # Switching to Free keeps the current view rather than snapping it.
        self.factory_cfg['camera']['mode']=mode
        self.free_camera_initialized=True
        if mode!='free': self.update_camera()
        self.label_camera.text='Camera: '+mode.upper()+' | mouse navigation'

    def safe_reset(self,source='R'):
        top_view=str(self.context['viewport'].camera_path)=='/World/Navigation/TopCamera'
        nav=getattr(self,'navigation',None)
        if nav is not None:
            nav.stop('Reset cancels patrol'); nav.selector.select('MANUAL')
        from pxr import UsdGeom
        free=self.factory_cfg['camera']['mode']=='free'
        view=None
        if free:
            prim=self.stage.GetPrimAtPath(str(self.context['viewport'].camera_path))
            if prim and prim.IsA(UsdGeom.Camera):
                view=(UsdGeom.XformCache().GetLocalToWorldTransform(prim),UsdGeom.Camera(prim).GetFocalLengthAttr().Get())
        if top_view:
            # The navigation camera is session-only and disappears when the
            # initial layer is imported. Retire its viewport binding first.
            self.context['viewport'].camera_path=str(self.context['camera'].GetPath())
            self._bind_render_camera(self.context['viewport'])
            for _ in range(2): self.app.update()
        state=self.last_state
        safe=not self.timeline.is_playing() or (
            np.linalg.norm(state['base_linear_velocity_world_m_s'])<self.key_cfg['reset_max_linear_speed'] and
            np.linalg.norm(state['base_angular_velocity_world_rad_s'])<self.key_cfg['reset_max_angular_speed'] and
            self.settle_time>=self.key_cfg['reset_settle_seconds'])
        rebuild_head=safe and self.head_widget is not None
        if rebuild_head:
            # GPU resources cannot be restored by copying USD RenderProducts.
            # Retire the preview before resetting USD, then create a new one.
            self.head_window.visible=False
            self.head_widget.destroy(); self.head_widget=None; self.head_window=None
            for _ in range(5): self.app.update()
        result=super().safe_reset(source)
        if result and view is not None:
            camera=self.context['camera']
            self._set_camera_transform(view[0])
            camera.CreateFocalLengthAttr(view[1])
            self.context['viewport'].camera_path=str(camera.GetPath())
        if rebuild_head: self._build_head_preview()
        if result and top_view and getattr(self,'patrol_panel',None) is not None:
            self.patrol_panel.top_view()
        self._bind_render_camera(self.context['viewport'])
        return result

    def _bind_render_camera(self,viewport):
        from pxr import UsdRender,Sdf
        path=viewport.render_product_path
        if not path or not viewport.camera_path: return
        product=UsdRender.Product(self.stage.GetPrimAtPath(path))
        if product:
            camera=[Sdf.Path(str(viewport.camera_path))]
            if product.GetCameraRel().GetTargets()!=camera: product.CreateCameraRel().SetTargets(camera)

    def update_status(self,force=False):
        super().update_status(force)
        if hasattr(self,'controller_metrics'):
            gpu=self.gpu_samples[-1].get('gpu_device_total_used_mib') if self.gpu_samples else None
            self.controller_metrics.text=(f'Robot: {self.policy_status} | FPS(loop): {self.fps:.1f}\n'
                f'Inference: {getattr(self,"last_inference_ms",0):.2f} ms | GPU total used: {gpu} MiB\n'
                f'Configured: {self.loco_cfg["physics_hz"]} Hz physics / {self.loco_cfg["control_hz"]} Hz control')
        if self.hud is not None:
            warning=self.turn_warning()
            self.label_note.text=self.keys.reason+('\n'+warning if warning else '')+f'\nFPS(loop): {self.fps:.1f} | physics steps/s(wall): {self.physics_rate:.1f}'

    def snapshot(self):
        from pxr import UsdGeom
        value=super().snapshot()
        value.update(factory_mode=not self.flat,loop_fps=self.fps,physics_steps_per_wall_second=self.physics_rate,
            camera_path=str(self.context['viewport'].camera_path),camera_mode=self.factory_cfg['camera']['mode'],
            camera_world_transform=[list(row) for row in UsdGeom.XformCache().GetLocalToWorldTransform(self.context['camera'].GetPrim())])
        value['head_camera_path']=getattr(self,'head_camera_path',None)
        nav=getattr(self,'navigation',None)
        if nav is not None and nav.mode=='AUTO PATROL':
            value['applied_command']=nav.last_command
            value['desired_command']=nav.last_command
            value['navigation_state']=nav.scheduler.state
        return value

    def tick(self):
        nav=getattr(self,'navigation',None)
        if nav is not None: nav.before_tick(self.timeline.is_playing())
        try: result=super().tick()
        except Exception as error:
            if nav is not None: nav.fail('Policy/feedback/runtime error: '+str(error))
            raise
        if nav is not None and hasattr(self,'patrol_panel'): self.patrol_panel.update()
        if result is not None: self.last_inference_ms=result.get('inference_ms',0)
        if self.keys.stop_generation!=self.last_stop_generation:
            self.last_stop_generation=self.keys.stop_generation
            self.set_camera_mode('follow')
        self.update_camera()
        self._bind_render_camera(self.context['viewport'])
        desired=(self.factory_cfg['rendering']['width'],self.factory_cfg['rendering']['height'])
        if tuple(self.context['viewport'].resolution or ())!=desired: self.context['viewport'].resolution=desired
        if self.head_widget is not None:
            self._clean_head_overlays()
            from pxr import UsdGeom
            xf=UsdGeom.Xformable(self.stage.GetPrimAtPath(self.head_camera_path))
            ops=xf.GetOrderedXformOps()
            if len(ops)==1 and ops[0].GetOpType()==UsdGeom.XformOp.TypeTransform:
                if ops[0].Get()!=self.head_mount_matrix: ops[0].Set(self.head_mount_matrix)
            else:
                xf.ClearXformOpOrder(); xf.AddTransformOp().Set(self.head_mount_matrix)
            self.head_widget.viewport_api.camera_path=self.head_camera_path
            desired=tuple(self.factory_cfg['head_camera']['resolution'])
            if tuple(self.head_widget.viewport_api.resolution or ())!=desired: self.head_widget.viewport_api.resolution=desired
            self._bind_render_camera(self.head_widget.viewport_api)
        now=time.perf_counter()
        elapsed=now-self.last_metric_clock
        if elapsed>=1:
            # One app.update/render request per loop tick; not a GPU present-time profiler.
            frames=self.steps-self.last_metric_steps
            self.fps=frames/elapsed
            self.physics_rate=frames*self.loco_cfg['decimation']/elapsed
            self.last_metric_clock=now; self.last_metric_steps=self.steps
            sample={'wall_time':time.time(),'loop_fps':self.fps,'physics_steps_per_wall_second':self.physics_rate,
                'configured_physics_hz':self.loco_cfg['physics_hz'],'configured_control_hz':self.loco_cfg['control_hz'],
                'policy_status':self.policy_status,'contacts':self.contact_counts}
            if now-self.last_gpu_clock>=self.factory_cfg['gpu_sample_interval_s']:
                self.last_gpu_clock=now
                try:
                    process=subprocess.run(['nvidia-smi','--query-gpu=memory.used,memory.total,utilization.gpu','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=5,creationflags=subprocess.CREATE_NO_WINDOW)
                    if process.returncode: raise RuntimeError(process.stderr.strip())
                    values=[float(x.strip()) for x in process.stdout.splitlines()[0].split(',')]
                    sample['gpu_device_total_used_mib']=values[0]; sample['gpu_total_mib']=values[1]; sample['gpu_utilization_pct']=values[2]
                    self.gpu_samples.append(sample.copy())
                except Exception as error: sample['gpu_sample_error']=str(error)
            self.metric_file.write(json.dumps(sample)+'\n'); self.metric_file.flush()
        return result

    def close(self):
        self.contact_subscription=None
        if getattr(self,'patrol_panel',None) is not None: self.patrol_panel.close(); self.patrol_panel=None
        if getattr(self,'navigation',None) is not None: self.navigation.close(); self.navigation=None
        if getattr(self,'controller_window',None) is not None:
            self.controller_window.destroy(); self.controller_window=None
        if self.head_widget is not None: self.head_widget.destroy(); self.head_widget=None; self.head_window=None
        if hasattr(self,'metric_file'): self.metric_file.close()
        super().close()
