"""RGB-D camera acceptance: standing, turning, metric depth vs world ground truth.

Headless, Factory V2, manual mode only. Never starts patrol. World-space ground
truth (camera pose, test sphere position, floor plane) is used for evaluation
only; the sensor itself never reads it.

Run: ./scripts/isaac_python.sh scripts/test_rgbd_camera.py
"""
from pathlib import Path
import json,math,sys
ROOT=Path(__file__).resolve().parents[1]; sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':640,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':[
    '--/app/vulkan='+str(sys.platform.startswith('linux')).lower(),
    '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
    '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
runtime=None
TARGET='/World/RgbdTestTarget'; RADIUS=.15
try:
    import numpy as np
    from pxr import Gf,Sdf,UsdGeom,UsdShade
    from controllers.factory_g1_runtime import FactoryG1Runtime
    from controllers.factory_selector import load_factory
    from perception.rgbd_camera import load_config
    from test_keyboard import yaw
    cfg=load_factory('v2'); cfg['rgbd_camera']=load_config({'enabled':True})
    runtime=FactoryG1Runtime(app,cfg,self_test=True)
    camera=runtime.rgbd; stage=runtime.stage
    report={'status':'running','camera_prim':camera.path,'run_directory':str(runtime.directory),'image_directory':str(camera.directory),
            'config':cfg['rgbd_camera'],'tests':{}}
    def add(name,passed,**values):
        report['tests'][name]={'passed':bool(passed),**values}
        print('RGBD_TEST '+json.dumps({name:report['tests'][name]},default=float),flush=True)
    def advance(seconds):
        for _ in range(round(seconds*runtime.loco_cfg['control_hz'])):
            app.update(); runtime.tick()
            if runtime.fault: raise RuntimeError('Robot fault during camera test: '+runtime.fault)
    def release():
        for key in runtime.keys.mapping: runtime.inject(key,'RELEASE')
    def fresh():
        """Let the renderer catch up with a scene edit, then read a frame."""
        advance(.4); frame=camera.read()
        if frame is None: raise RuntimeError('No RGB-D frame')
        return frame
    def project(frame,point):
        """World point -> (u, v, optical xyz) using the frame's own pose and intrinsics."""
        p=np.linalg.inv(frame['T_world_optical'])@np.array([*point,1.]); k=frame['intrinsics']
        return k['fx']*p[0]/p[2]+k['cx'],k['fy']*p[1]/p[2]+k['cy'],p[:3]
    def patch(frame,u,v,half=1):
        r,c=int(math.floor(v)),int(math.floor(u))
        return float(np.median(frame['depth_m'][max(0,r-half):r+half+1,max(0,c-half):c+half+1]))
    def target_mask(frame):
        r,g,b=[frame['rgb'][:,:,i].astype(np.int32) for i in range(3)]
        return (r>110)&(b>110)&(g<.55*np.minimum(r,b))
    def place_target(position):
        prim=stage.GetPrimAtPath(TARGET)
        if not prim:
            sphere=UsdGeom.Sphere.Define(stage,TARGET); sphere.CreateRadiusAttr(RADIUS)
            sphere.AddTranslateOp().Set(Gf.Vec3d(*position))
            # Visual only: no collider, so physics and the navigation obstacle scan cannot see it.
            material=UsdShade.Material.Define(stage,TARGET+'_Material')
            shader=UsdShade.Shader.Define(stage,TARGET+'_Material/Shader'); shader.CreateIdAttr('UsdPreviewSurface')
            for name in ['diffuseColor','emissiveColor']: shader.CreateInput(name,Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1,0,1))
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(material)
        else: UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(*position))
    def sphere_check(frame,centre):
        """Expected Z-depth of the sphere surface on the ray through its centre."""
        u,v,p=project(frame,centre); distance=float(np.linalg.norm(p))
        expected=(distance-RADIUS)*p[2]/distance; measured=patch(frame,u,v)
        mask=target_mask(frame); rows,cols=np.nonzero(mask)
        centroid=[float(cols.mean())+.5,float(rows.mean())+.5] if len(cols)>20 else None
        return {'ground_truth_range_to_centre_m':distance,'expected_depth_m':float(expected),'measured_depth_m':measured,
                'error_m':measured-float(expected),'predicted_pixel':[float(u),float(v)],'target_centroid_pixel':centroid,
                'pixel_error':float(math.hypot(centroid[0]-u,centroid[1]-v)) if centroid else None,'target_pixels':int(mask.sum())}

    # ---- 1. standing ------------------------------------------------------
    advance(3)
    frame=fresh(); stem=camera.save(frame,'standing')
    k=frame['intrinsics']; T=frame['T_world_optical']; forward=T[:3,2]
    robot_yaw=yaw(frame['base_orientation_wxyz'])
    add('frame_contract',frame['rgb'].shape==(360,640,3) and frame['rgb'].dtype==np.uint8 and frame['depth_m'].shape==(360,640)
        and frame['depth_m'].dtype==np.float32 and frame['valid'].mean()>.9 and frame['rgb'].std()>5,
        rgb_shape=list(frame['rgb'].shape),depth_shape=list(frame['depth_m'].shape),depth_dtype=str(frame['depth_m'].dtype),
        valid_fraction=float(frame['valid'].mean()),depth_min_m=float(frame['depth_m'][frame['valid']].min()),
        depth_max_m=float(frame['depth_m'][frame['valid']].max()),rgb_std=float(frame['rgb'].std()),
        intrinsics=k,sim_time_s=frame['sim_time'],saved=str(stem))
    pitch=math.degrees(math.asin(-forward[2])); heading=math.atan2(forward[1],forward[0])
    add('mount_pose',abs(pitch-cfg['rgbd_camera']['pitch_down_deg'])<6 and abs(heading-robot_yaw)<.12 and 1.0<T[2,3]<1.5,
        camera_position_world_m=T[:3,3].tolist(),pitch_below_horizon_deg=pitch,nominal_pitch_deg=cfg['rgbd_camera']['pitch_down_deg'],
        camera_heading_rad=heading,robot_yaw_rad=robot_yaw,note='Measured pitch includes the torso lean of the standing policy')
    # Floor plane Z=0 is known world geometry: an object-free depth check.
    floor=[]
    for v in [230,290,345]:
        for u in [200,320,440]:
            ray=T[:3,:3]@np.array([(u+.5-k['cx'])/k['fx'],(v+.5-k['cy'])/k['fy'],1.])
            expected=-T[2,3]/ray[2]; measured=float(frame['depth_m'][v,u])
            floor.append({'pixel':[u,v],'expected_m':float(expected),'measured_m':measured,'error_m':measured-float(expected)})
    add('floor_depth',max(abs(row['error_m']) for row in floor)<.03,max_abs_error_m=max(abs(row['error_m']) for row in floor),samples=floor)
    frames=[]
    for _ in range(10): advance(.1); frames.append(camera.frame)
    times=[f['sim_time'] for f in frames]
    add('timestamps',all(b>a for a,b in zip(times,times[1:])) and abs((times[-1]-times[0])/9-.1)<.021,sim_times_s=times,
        configured_capture_hz=cfg['rgbd_camera']['capture_hz'])

    # ---- 2. depth vs world ground truth ------------------------------------
    origin=camera.world_pose()[:3,3]; rows=[]
    for distance in [1.,2.,3.,5.,8.]:
        centre=[origin[0]+distance+RADIUS,origin[1],.75]
        place_target(centre); frame=fresh(); row=sphere_check(frame,centre); row['nominal_surface_distance_m']=distance
        rows.append(row)
        if distance in (2.,5.): camera.save(frame,f'target_{distance:.0f}m')
    add('target_depth_standing',all(abs(r['error_m'])<max(.02,.01*r['expected_depth_m']) for r in rows)
        and all(r['pixel_error'] is not None and r['pixel_error']<3 for r in rows),
        max_abs_error_m=max(abs(r['error_m']) for r in rows),max_pixel_error=max(r['pixel_error'] or 99 for r in rows),samples=rows,
        ground_truth='USD world position of a collider-free test sphere and the camera world transform; evaluation only')

    # ---- 3. turning ---------------------------------------------------------
    centre=[origin[0]+2.5+RADIUS,origin[1],.9]; place_target(centre); advance(.5)
    before=camera.read(); camera.save(before,'turn_start'); yaw_before=yaw(runtime.last_state['base_orientation_wxyz'])
    runtime.inject('A'); lag_errors={lag:[] for lag in range(5)}; headings=[]; saved_mid=False
    for step in range(175):
        app.update(); runtime.tick()
        if runtime.fault: raise RuntimeError('Robot fault while turning: '+runtime.fault)
        frame=camera.read(); poses=camera.recent_poses()
        forward=frame['T_world_optical'][:3,2]
        headings.append(math.atan2(forward[1],forward[0])-yaw(frame['base_orientation_wxyz']))
        mask=target_mask(frame); rows_,cols_=np.nonzero(mask)
        # Use the target only while it is fully inside the image and the robot is really rotating.
        if len(cols_)>60 and cols_.min()>2 and cols_.max()<637 and step>40 and len(poses)>=5:
            for lag in lag_errors:
                trial=dict(frame); trial['T_world_optical']=poses[-1-lag]['T']@np.diag([1.,-1.,-1.,1.])
                lag_errors[lag].append(cols_.mean()+.5-project(trial,centre)[0])
        if step==85 and not saved_mid: camera.save(frame,'turn_mid'); saved_mid=True
    release(); advance(2.5)
    after=fresh(); camera.save(after,'turn_end'); yaw_after=yaw(runtime.last_state['base_orientation_wxyz'])
    turned=(yaw_after-yaw_before+math.pi)%(2*math.pi)-math.pi
    camera_turned=math.atan2(after['T_world_optical'][1,2],after['T_world_optical'][0,2])-math.atan2(before['T_world_optical'][1,2],before['T_world_optical'][0,2])
    camera_turned=(camera_turned+math.pi)%(2*math.pi)-math.pi
    lag_summary={lag:{'mean_abs_px':float(np.mean(np.abs(v))),'mean_px':float(np.mean(v)),'samples':len(v)} for lag,v in lag_errors.items() if v}
    best=min(lag_summary,key=lambda lag:lag_summary[lag]['mean_abs_px']) if lag_summary else None
    difference=float(np.abs(after['rgb'].astype(np.int32)-before['rgb'].astype(np.int32)).mean())
    add('turning_follow',abs(turned)>.6 and abs(camera_turned-turned)<.1 and difference>8 and after['valid'].mean()>.5
        and max(abs(h) for h in headings)<.25,
        robot_yaw_change_rad=turned,camera_heading_change_rad=camera_turned,mean_abs_rgb_change=difference,
        max_camera_minus_base_heading_rad=max(abs(h) for h in headings),valid_fraction_after=float(after['valid'].mean()))
    add('pose_image_sync',best is not None and best==cfg['rgbd_camera']['pose_lag_ticks'] and lag_summary[best]['mean_abs_px']<2.5,
        configured_pose_lag_ticks=cfg['rgbd_camera']['pose_lag_ticks'],best_lag_ticks=best,horizontal_error_by_lag=lag_summary,
        meaning='Which stored camera pose best explains where the world-fixed target appears while the robot rotates')
    # Depth again at the new heading, target placed along the camera's horizontal forward
    # (short range: after the turn a wall and an operator stand about 2.3 m ahead).
    T=camera.world_pose(); forward=-T[:3,2]; horizontal=np.array([forward[0],forward[1],0.])/math.hypot(forward[0],forward[1])
    rows=[]
    for distance in [1.,1.5]:
        centre=(T[:3,3]+horizontal*(distance+RADIUS)).tolist(); centre[2]=.75
        place_target(centre); frame=fresh(); row=sphere_check(frame,centre); row['nominal_surface_distance_m']=distance; rows.append(row)
    camera.save(frame,'target_after_turn')
    add('target_depth_after_turn',all(abs(r['error_m'])<max(.02,.01*r['expected_depth_m']) and (r['pixel_error'] or 99)<3 for r in rows),
        max_abs_error_m=max(abs(r['error_m']) for r in rows),samples=rows)

    # ---- 4. reset rebuilds the sensor, patrol untouched ----------------------
    stage.RemovePrim(TARGET); stage.RemovePrim(TARGET+'_Material')
    runtime.timeline.pause(); app.update()
    reset_ok=runtime.safe_reset(source='rgbd_camera_test'); runtime.timeline.play(); app.update(); runtime.keys.set_running(True)
    advance(2); frame=fresh(); camera.save(frame,'after_reset')
    add('reset_rebuild',reset_ok and frame['valid'].mean()>.9 and frame['rgb'].std()>5,valid_fraction=float(frame['valid'].mean()))
    nav=runtime.navigation
    add('patrol_not_started',nav.mode=='MANUAL' and nav.scheduler.state in ('IDLE','STOPPED') and not nav.scheduler.completed,
        navigation_mode=nav.mode,scheduler_state=nav.scheduler.state)
    report['annotator_read_ms']=camera.read_ms; report['frames_read']=camera.frame_index; report['files_saved']=camera.saved
    report['failed_tests']=[name for name,value in report['tests'].items() if not value['passed']]
    report['status']='passed' if not report['failed_tests'] else 'failed'
    payload=json.dumps(report,indent=2,default=float)
    (camera.directory/'rgbd_camera_results.json').write_text(payload)
    (ROOT/'docs/perception').mkdir(parents=True,exist_ok=True)
    (ROOT/'docs/perception/rgbd_camera_results.json').write_text(payload)
    print('RGBD_RESULT '+json.dumps({'status':report['status'],'failed_tests':report['failed_tests'],'images':str(camera.directory)}),flush=True)
finally:
    if runtime: runtime.close()
    app.close()
