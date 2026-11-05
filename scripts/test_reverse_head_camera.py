"""Regression: lost W release, reverse, SPACE follow, mounted head preview."""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    p=argparse.ArgumentParser(); p.add_argument('--hold-seconds',type=float,default=0); args,_=p.parse_known_args()
    cfg=json.loads((ROOT/'configs/factory_g1.json').read_text())
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
            '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        import numpy as np
        from pxr import UsdGeom
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.g1_locomotion import inverse_rotate_wxyz
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        report={'input':'Carb logical keyboard and simulated physical release query; actual ONNX/PhysX',
            'run_directory':str(runtime.directory),'cases':[]}
        def advance(seconds):
            for _ in range(round(seconds*50)): app.update(); runtime.tick()
        def reset():
            for k in runtime.keys.mapping: runtime.inject(k,'RELEASE')
            runtime.timeline.pause(); app.update(); runtime.safe_reset(source='reverse_preview_regression')
            runtime.timeline.play(); app.update(); runtime.keys.set_running(True); advance(1)
        for key in ['S','DOWN']:
            reset()
            # W key-up intentionally missing; physical query correctly reports it up.
            runtime.inject('W'); advance(.6)
            runtime.inject(key)
            cancelled=runtime.keys.desired()[0]==0
            runtime.keys.reconcile_released_keys(lambda k:k==key,runtime._log_event)
            before=runtime.last_state
            advance(3.5)
            after=runtime.last_state
            delta=inverse_rotate_wxyz(before['base_orientation_wxyz'],np.array(after['base_position_m'])-before['base_position_m'])
            command=list(runtime.keys.command)
            runtime.inject(key,'RELEASE'); advance(2)
            case={'key':key,'opposed_before_reconcile':cancelled,'held_command':command,'body_forward_delta_m':float(delta[0]),
                'stale_W_removed':'W' not in runtime.keys.pressed,'fault':runtime.fault,
                'passed':bool(delta[0]<-.2 and command[0]<-.3 and not runtime.fault)}
            report['cases'].append(case); print('REVERSE_TEST '+json.dumps(case),flush=True)
        reset(); runtime.set_camera_mode('free'); runtime.inject('SPACE'); runtime.tick()
        report['space_stops_and_locks_camera']=runtime.keys.command==[0,0,0] and runtime.factory_cfg['camera']['mode']=='follow'
        runtime.inject('SPACE','RELEASE')
        camera=runtime.stage.GetPrimAtPath(runtime.head_camera_path)
        local_before=UsdGeom.Xformable(camera).GetLocalTransformation()
        world_before=UsdGeom.XformCache().GetLocalToWorldTransform(camera)
        runtime.inject('A'); advance(2); runtime.inject('A','RELEASE'); advance(2)
        world_after=UsdGeom.XformCache().GetLocalToWorldTransform(camera)
        report['head_camera_attached_to_torso']=str(camera.GetParent().GetName())=='torso_link'
        report['head_camera_follows_robot']=world_before!=world_after and local_before==UsdGeom.Xformable(camera).GetLocalTransformation()
        report['head_preview_resolution']=list(runtime.head_widget.viewport_api.resolution)
        runtime.context['capture'](runtime.directory/'reverse_main.png')
        runtime.capture_head_preview(runtime.directory/'head_camera.png')
        report['head_image_saved']=(runtime.directory/'head_camera.png').is_file()
        report['status']='passed' if all(c['passed'] for c in report['cases']) and all(report[k] for k in [
            'space_stops_and_locks_camera','head_camera_attached_to_torso','head_camera_follows_robot','head_image_saved']) else 'failed'
        (ROOT/'logs/reverse_head_camera_result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('REVERSE_HEAD_RESULT '+json.dumps(report),flush=True)
        print('PREVIEW_READY',flush=True)
        until=time.perf_counter()+args.hold_seconds
        while app.is_running() and time.perf_counter()<until:
            app.update(); runtime.tick(); time.sleep(.02)
        if report['status']!='passed': raise RuntimeError('Reverse/head camera regression failed')
    except BaseException:
        import traceback
        print(traceback.format_exc(),flush=True)
        raise
    finally:
        if runtime: runtime.close()
        app.close()

if __name__=='__main__': main()
