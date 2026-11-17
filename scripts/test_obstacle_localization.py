"""PhysX camera evaluation; known USD cube geometry is evaluation-only."""
from pathlib import Path
import sys,json,math
ROOT=Path(r'D:\unitree_g1');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                      '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        import numpy as np,carb
        from pxr import UsdGeom,Gf
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.factory_selector import load_factory
        from controllers.rgbd_camera import RobotRgbdCamera
        from controllers.yolo_detection import AsyncYoloDetector,load_config
        carb.settings.get_settings().set('/app/asyncRendering',False)
        runtime=FactoryG1Runtime(app,load_factory('v2',None),self_test=True)
        cfg=json.loads((ROOT/'configs/rgbd_camera.json').read_text());cfg.update(enabled=True,debug=False,capture_hz=1000)
        runtime.rgbd=RobotRgbdCamera(runtime.stage,runtime.directory,cfg)
        runtime.yolo=AsyncYoloDetector(runtime.directory,load_config())
        camera=runtime.rgbd;rows=[]
        def advance(n):
            for _ in range(n):
                app.update();runtime.tick()
                if runtime.fault or camera.error or runtime.yolo.error: raise RuntimeError(runtime.fault or camera.error or runtime.yolo.error)
        advance(80)
        initial=np.array(camera.latest[2]['T_world_optical'])
        for turn in [False,True]:
            if turn:
                runtime.inject('A');advance(100);runtime.inject('A','RELEASE');advance(30)
            for distance in [1.5,2.5]:
                T=np.array(camera.latest[2]['T_world_optical']);center=T[:3,3]+T[:3,2]*distance
                cube=UsdGeom.Cube.Define(runtime.stage,'/World/LocalizationEvaluationCube')
                cube.CreateSizeAttr(1.);cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3f(.2,1.2,1.2))
                advance(20)
                rgb,depth,meta=camera.latest
                # Synthetic bbox is ONLY a projection/math test, never reported as YOLO detection.
                synthetic={'frame_counter':meta['frame_counter'],'acquisition_utc':meta['acquisition_utc'],
                    'detections':[{'class_name':'evaluation_roi_not_AI','confidence':1.,'bbox_xyxy':[308,168,332,192]}]}
                semantic=camera.localizer.semantic(depth,meta,synthetic)
                if not semantic: raise RuntimeError('Evaluation ROI has no valid depth')
                estimated=np.array(semantic[0]['position_world_m'])
                lo=center-np.array([.1,.6,.6]);hi=center+np.array([.1,.6,.6])
                surface_error=float(np.linalg.norm(np.maximum(lo-estimated,0)+np.maximum(estimated-hi,0)))
                # Ground truth ray to the same visible surface point, not cube center.
                T=np.array(meta['T_world_optical']);ray=estimated-T[:3,3]
                with np.errstate(divide='ignore'):
                    a=(lo-T[:3,3])/ray;b=(hi-T[:3,3])/ray
                near=max(np.minimum(a,b));ground_truth=T[:3,3]+near*ray
                error=float(np.linalg.norm(estimated-ground_truth))
                geometry=camera.localizer.geometry(depth,meta)
                candidates=[g for g in geometry if np.linalg.norm(np.array(g['position_world_m'])-center)<.9]
                if not candidates: raise RuntimeError('Unclassified cube missed by depth geometry')
                camera.save(('rotated' if turn else 'standing')+'_'+str(distance))
                rows.append({'rotated':turn,'cube_center_world_m':center.tolist(),'roi_surface':semantic[0],
                    'ground_truth_surface_world_m':ground_truth.tolist(),'distance_error_m':error,
                    'surface_aabb_error_m':surface_error,'unknown_geometry':candidates,
                    'passed':bool(error<.03 and surface_error<.03)})
                runtime.stage.RemovePrim('/World/LocalizationEvaluationCube');advance(5)
        # Timestamp mismatch must be rejected instead of silently matching current depth.
        bad=dict(synthetic);bad['frame_counter']=-1
        try: camera.localizer.semantic(depth,meta,bad);rejected=False
        except ValueError: rejected=True
        final=np.array(camera.latest[2]['T_world_optical'])
        yaw=math.degrees(math.atan2(final[1,2],final[0,2])-math.atan2(initial[1,2],initial[0,2]))
        report={'run_directory':str(runtime.directory),'measurements':rows,'yaw_change_deg':yaw,
            'timestamp_mismatch_rejected':rejected,'actual_yolo_results':runtime.yolo.completed,
            'policy_fault':runtime.fault,'patrol_mode':runtime.navigation.mode,
            'passed':bool(all(x['passed'] for x in rows) and rejected and abs(yaw)>10 and runtime.yolo.completed>0),
            'scope':'ROI tests use explicit synthetic evaluation bbox, not AI. Unknown depth geometry requires no YOLO box. Actual YOLO outputs logged separately.'}
        (runtime.directory/'localization_results.json').write_text(json.dumps(report,indent=2))
        print('LOCALIZATION_RESULT '+json.dumps(report),flush=True)
        if not report['passed']: raise RuntimeError('Localization test failed')
    except BaseException:
        import traceback
        err=traceback.format_exc();print(err,flush=True)
        if runtime:(runtime.directory/'localization_error.txt').write_text(err)
        raise
    finally:
        if runtime:runtime.close()
        app.close()

if __name__=='__main__':main()
