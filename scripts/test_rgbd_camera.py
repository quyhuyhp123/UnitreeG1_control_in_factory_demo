"""Camera-only PhysX evaluation. Never starts Patrol or changes robot poses."""
from pathlib import Path
import sys, json, time, math, argparse
ROOT=Path(r'D:\unitree_g1')
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--gui',action='store_true')
    args,_=parser.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':not args.gui,'width':960,'height':640,
        'renderer':'RaytracedLighting','multi_gpu':False,'extra_args':[
        '--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
        '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        import numpy as np
        import carb
        from pxr import UsdGeom,Gf
        from controllers.factory_selector import load_factory
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.rgbd_camera import RobotRgbdCamera
        settings=carb.settings.get_settings()
        settings.set('/app/asyncRendering',False)
        for key in ['/rtx/reflections/enabled','/rtx/indirectDiffuse/enabled','/rtx/ambientOcclusion/enabled']:
            settings.set(key,False)
        cfg=load_factory('v2',None)
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        camera_cfg=json.loads((ROOT/'configs/rgbd_camera.json').read_text())
        camera_cfg.update(enabled=True,debug=args.gui,capture_hz=1000)
        runtime.rgbd=RobotRgbdCamera(runtime.stage,runtime.directory,camera_cfg)
        camera=runtime.rgbd
        report={'run_directory':str(runtime.directory),'tests':{},'patrol_started':False}
        start=time.perf_counter()
        def advance(n):
            for _ in range(n):
                app.update(); runtime.tick()
                if runtime.fault or camera.error:
                    raise RuntimeError(runtime.fault or camera.error)
        advance(100)
        if camera.latest is None: raise RuntimeError('No camera frames after 100 app updates')
        camera.save('standing')
        report['tests']['standing']={'frames':camera.frames,'policy_fault':runtime.fault}
        box=UsdGeom.Cube.Define(runtime.stage,'/World/RgbdEvaluationTarget')
        box.CreateSizeAttr(1.)
        box.CreateDisplayColorAttr([Gf.Vec3f(.1,.7,.9)])
        move=box.AddTranslateOp()
        box.AddScaleOp().Set(Gf.Vec3f(.2,1.2,1.2))
        depth_results=[]
        for distance in [1.5,2.5]:
            T=np.array(camera.latest[2]['T_world_optical'])
            center=T[:3,3]+T[:3,2]*distance
            move.Set(Gf.Vec3d(*center))
            advance(15)
            rgb,depth,meta=camera.latest
            T=np.array(meta['T_world_optical']); K=np.array(meta['intrinsics_K'])
            errors=[]; measured=[]; expected=[]
            lo=center-np.array([.1,.6,.6]); hi=center+np.array([.1,.6,.6])
            for v in range(177,183):
                for u in range(317,323):
                    direction=T[:3,:3]@np.array([(u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],1.])
                    with np.errstate(divide='ignore'):
                        a=(lo-T[:3,3])/direction; b=(hi-T[:3,3])/direction
                    near=max(np.minimum(a,b)); far=min(np.maximum(a,b))
                    if 0<near<=far and np.isfinite(depth[v,u]):
                        measured.append(float(depth[v,u]));expected.append(float(near));errors.append(abs(float(depth[v,u])-near))
            item={'target_distance_nominal_m':distance,'samples':len(errors),
                'median_measured_z_m':float(np.median(measured)) if measured else None,
                'median_expected_z_m':float(np.median(expected)) if expected else None,
                'maximum_absolute_error_m':max(errors,default=None),
                'passed':bool(bool(errors) and max(errors)<.05)}
            depth_results.append(item)
            camera.save('target_'+str(distance))
        report['tests']['depth_ground_truth']=depth_results
        runtime.stage.RemovePrim('/World/RgbdEvaluationTarget')
        before=np.array(camera.latest[2]['T_world_optical'])
        runtime.inject('A'); advance(100);runtime.inject('A','RELEASE');advance(50)
        after=np.array(camera.latest[2]['T_world_optical'])
        parent=runtime.stage.GetPrimAtPath('/World/G1/torso_link')
        parent_T=np.array(camera.latest[2]['T_world_mount_parent'])
        actual_mount=np.linalg.inv(parent_T)@np.array(camera.latest[2]['T_world_usd_camera'])
        yaw_change=math.degrees(math.atan2(after[1,2],after[0,2])-math.atan2(before[1,2],before[0,2]))
        mount_error=float(np.max(np.abs(actual_mount-np.array(camera.mount).T)))
        report['tests']['physical_rotation']={'yaw_change_deg':yaw_change,'mount_max_error':mount_error,
                                             'passed':bool(abs(yaw_change)>10 and mount_error<1e-5)}
        camera.save('rotation')
        previous=camera.frames
        camera.disable(); advance(5)
        disabled_frames=camera.frames
        camera.enable(); advance(15)
        report['tests']['toggle']={'passed':disabled_frames==previous and camera.frames>previous}
        runtime.timeline.pause();app.update()
        reset_ok=runtime.safe_reset('RGBD evaluation')
        runtime.timeline.play();advance(25)
        report['tests']['reset']={'passed':bool(reset_ok and camera.sensor is not None and camera.error is None)}
        report.update(elapsed_wall_s=time.perf_counter()-start,acquired_frames=camera.frames,
            loop_fps=runtime.fps,gpu_samples=runtime.gpu_samples,
            scope='Real PhysX + policy, injected logical keyboard; not native keyboard test. FPS is app loop; VRAM whole GPU.')
        report['passed']=all(x['passed'] for x in depth_results) and all(report['tests'][k]['passed'] for k in ['physical_rotation','toggle','reset'])
        (runtime.directory/'rgbd_test_results.json').write_text(json.dumps(report,indent=2,default=str),encoding='utf-8')
        print('RGBD_RESULT '+json.dumps(report),flush=True)
        if not report['passed']: raise RuntimeError('Camera evaluation failed; see report')
    except BaseException:
        import traceback
        failure=traceback.format_exc()
        print(failure,flush=True)
        if runtime: (runtime.directory/'rgbd_test_error.txt').write_text(failure)
        raise
    finally:
        if runtime: runtime.close()
        app.close()

if __name__=='__main__': main()
