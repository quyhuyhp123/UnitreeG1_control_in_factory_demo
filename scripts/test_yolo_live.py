"""Real G1 RGB-D -> isolated YOLO. Manual mode only; no patrol start."""
from pathlib import Path
import sys,json,time,argparse,statistics
ROOT=Path(r'D:\unitree_g1')
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--headless',action='store_true')
    args,_=parser.parse_known_args()
    from controllers.yolo_detection import load_config
    yolo_cfg=load_config()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.headless,'width':960,'height':640,
        'renderer':'RaytracedLighting','multi_gpu':False,'extra_args':[
            '--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
            '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        import numpy as np
        import carb
        from controllers.factory_selector import load_factory
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.rgbd_camera import RobotRgbdCamera
        from controllers.yolo_detection import AsyncYoloDetector
        settings=carb.settings.get_settings();settings.set('/app/asyncRendering',False)
        for key in ['/rtx/reflections/enabled','/rtx/indirectDiffuse/enabled','/rtx/ambientOcclusion/enabled']:
            settings.set(key,False)
        runtime=FactoryG1Runtime(app,load_factory('v2',None),self_test=True)
        camera_cfg=json.loads((ROOT/'configs/rgbd_camera.json').read_text())
        camera_cfg.update(enabled=True,debug=not args.headless)
        runtime.rgbd=RobotRgbdCamera(runtime.stage,runtime.directory,camera_cfg)
        runtime.yolo=AsyncYoloDetector(runtime.directory,yolo_cfg)
        if not args.headless: runtime.yolo.build_debug()
        yolo=runtime.yolo
        new_classes=set();last_result=0
        start=time.perf_counter();step_times=[]
        def advance(count):
            nonlocal last_result
            for _ in range(count):
                begin=time.perf_counter()
                app.update();runtime.tick()
                step_times.append((time.perf_counter()-begin)*1000)
                if runtime.fault or runtime.rgbd.error or yolo.error:
                    raise RuntimeError(runtime.fault or runtime.rgbd.error or yolo.error)
                with yolo.lock: latest=yolo.latest
                if latest and latest[1]['received_perf_counter']!=last_result:
                    last_result=latest[1]['received_perf_counter']
                    for det in latest[1]['detections']:
                        if det['class_name'] not in new_classes:
                            yolo.save('first_'+det['class_name'].replace(' ','_'))
                            new_classes.add(det['class_name'])
        advance(100)
        runtime.rgbd.save('yolo_standing');yolo.save('standing')
        runtime.inject('A');advance(100);runtime.inject('A','RELEASE');advance(100)
        runtime.rgbd.save('yolo_rotated');yolo.save('rotated')
        if yolo.completed<5: raise RuntimeError('No sustained live-camera inference results')
        data=[json.loads(line) for line in (yolo.output/'detections.jsonl').read_text().splitlines()]
        start_ts=data[0]['received_perf_counter'];end_ts=data[-1]['received_perf_counter']
        count=yolo.completed
        yolo.set_enabled(False);advance(8)
        disabled_ok=yolo.completed==count and yolo.latest is None
        yolo.set_enabled(True);advance(25)
        toggle_ok=disabled_ok and yolo.completed>count
        report={'run_directory':str(runtime.directory),'model':yolo_cfg['model'],'device':yolo_cfg['device'],
            'imgsz':yolo_cfg['imgsz'],'target_hz':yolo_cfg['target_hz'],'camera_wh':camera_cfg['resolution_wh'],
            'detected_classes':sorted(new_classes),'target_classes_not_detected':sorted(set(['person','chair','bottle'])-new_classes),
            'inference_results':len(data),'inference_fps':(len(data)-1)/(end_ts-start_ts),
            'latency_ms_median':statistics.median(x['inference_total_ms'] for x in data),
            'latency_ms_p95':float(np.percentile([x['inference_total_ms'] for x in data],95)),
            'ipc_roundtrip_ms_p95':float(np.percentile([x['ipc_roundtrip_ms'] for x in data],95)),
            'frame_age_ms_p95':float(np.percentile([x['frame_age_ms'] for x in data],95)),
            'app_loop_ms_median':statistics.median(step_times),'loop_fps_last_sample':runtime.fps,
            'gpu_peak_device_total_mib':max([x['gpu_device_total_used_mib'] for x in runtime.gpu_samples],default=None),
            'gpu_samples':runtime.gpu_samples,'toggle_passed':toggle_ok,'policy_fault':runtime.fault,
            'patrol_started':False,'elapsed_wall_s':time.perf_counter()-start,
            'passed':bool(toggle_ok and len(data)>=5 and not runtime.fault),
            'scope':'Live RGB sensor inference, not USD labels. Empty detections permitted; not proof of detection recall. GPU memory is whole device.'}
        (runtime.directory/'yolo_live_results.json').write_text(json.dumps(report,indent=2))
        print('YOLO_LIVE_RESULT '+json.dumps({k:v for k,v in report.items() if k!='gpu_samples'}),flush=True)
        if not report['passed']: raise RuntimeError('Live camera integration checks failed')
    except BaseException:
        import traceback
        error=traceback.format_exc();print(error,flush=True)
        if runtime: (runtime.directory/'yolo_live_error.txt').write_text(error)
        raise
    finally:
        if runtime:runtime.close()
        app.close()

if __name__=='__main__':main()
