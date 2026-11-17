"""Offline image inference and real asynchronous IPC benchmark; no Isaac needed."""
from pathlib import Path
import sys, os, json, time, statistics
from datetime import datetime,timezone
ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(ROOT))
(ROOT/'logs/yolo_settings').mkdir(parents=True,exist_ok=True)
os.environ['YOLO_CONFIG_DIR']=str(ROOT/'logs/yolo_settings')
os.environ['YOLO_AUTOINSTALL']='False'

def main():
    import cv2, numpy as np
    from controllers.yolo_worker import Detector
    from controllers.yolo_detection import AsyncYoloDetector,load_config
    cfg=load_config()
    output=ROOT/'logs/yolo_offline'/datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True,exist_ok=False)
    detector=Detector(cfg)
    images=[ROOT/'logs/yolo_samples/bus.jpg']+sorted((ROOT/'logs/yolo_samples/coco128').glob('*.jpg'))
    # Include the camera images saved in phase 2: sim domain is reported separately.
    saved_camera=ROOT/'logs/keyboard_runs/20261116_175328/rgbd/standing/rgb.png'
    if saved_camera.exists(): images.append(saved_camera)
    all_results=[]; best={};times=[]
    for index,path in enumerate(images):
        bgr=cv2.imread(str(path))
        if bgr is None: raise RuntimeError('Cannot read image '+str(path))
        rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        meta={'source':str(path),'frame_counter':index,'acquisition_utc':datetime.now(timezone.utc).isoformat()}
        result=detector.predict(rgb,meta)
        all_results.append(result);times.append(result['inference_total_ms'])
        for det in result['detections']:
            name=det['class_name']
            if name in ['person','chair','bottle'] and (name not in best or det['confidence']>best[name]['detection']['confidence']):
                best[name]={'detection':det,'source':str(path),'result':result,'rgb':rgb.copy()}
    for name,item in best.items():
        for det in item['result']['detections']:
            x1,y1,x2,y2=map(int,det['bbox_xyxy'])
            cv2.rectangle(item['rgb'],(x1,y1),(x2,y2),(0,255,80),2)
            cv2.putText(item['rgb'],f"{det['class_name']} {det['confidence']:.2f}",(max(x1,0),max(y1-5,16)),
                cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,80),2)
        cv2.imwrite(str(output/(name+'_detections.jpg')),cv2.cvtColor(item['rgb'],cv2.COLOR_RGB2BGR))
        del item['rgb']
    (output/'image_results.json').write_text(json.dumps(all_results,indent=2))
    # Verify actual subprocess pipe, bounded latest-frame queue, timestamps,
    # confidence/bbox outputs and nonblocking producer under load.
    client=AsyncYoloDetector(output,cfg)
    frame=cv2.cvtColor(cv2.resize(cv2.imread(str(images[0])),(640,360)),cv2.COLOR_BGR2RGB)
    submit_times=[];start=time.perf_counter();counter=0
    try:
        while time.perf_counter()-start<10:
            counter+=1
            now=time.perf_counter()
            client.submit(frame,{'frame_counter':counter,'acquisition_utc':datetime.now(timezone.utc).isoformat(),
                'acquisition_unix_s':time.time(),'source':'offline bus resized 640x360'})
            submit_times.append((time.perf_counter()-now)*1000)
            if client.error: raise RuntimeError(client.error)
            time.sleep(.02)
        if client.completed<5: raise RuntimeError('Too few async inference results')
        client.save('async_bus')
        with client.lock: result=client.latest[1].copy()
        count_before=client.completed
        client.set_enabled(False);time.sleep(.4)
        disabled_ok=client.latest is None and client.completed==count_before
        client.set_enabled(True)
        client.submit(frame,{'frame_counter':counter+1,'acquisition_utc':datetime.now(timezone.utc).isoformat()})
        deadline=time.perf_counter()+3
        while client.latest is None and time.perf_counter()<deadline: time.sleep(.02)
        enabled_ok=client.latest is not None
        ipc={'completed_results':count_before,'last_result':result,'dropped_frames':client.dropped,
             'submit_ms_p95':float(np.percentile(submit_times,95)),'submit_ms_max':max(submit_times),
             'bounded_pending_frames':client.pending.maxsize,'toggle_passed':disabled_ok and enabled_ok}
    finally: client.close()
    report={'output_directory':str(output),'model':'official COCO pretrained yolo11n.pt',
        'imgsz':cfg['imgsz'],'device':cfg['device'],'images_tested':len(images),
        'detected_target_classes':best,'missing_target_classes':sorted(set(['person','chair','bottle'])-set(best)),
        'latency_ms_median':statistics.median(times),'latency_ms_p95':float(np.percentile(times,95)),
        'unpaced_inference_fps':1000/statistics.mean(times),'async_ipc':ipc,
        'passed':not(set(['person','chair','bottle'])-set(best)) and ipc['toggle_passed']}
    (output/'report.json').write_text(json.dumps(report,indent=2))
    print('YOLO_OFFLINE_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ['detected_target_classes','async_ipc']}),flush=True)
    if not report['passed']: raise RuntimeError('Offline detection checks failed')

if __name__=='__main__':main()
