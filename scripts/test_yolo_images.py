"""Offline YOLO check on stored images and a stored frame sequence. No simulation.

1. This project's pre/post-processing vs Ultralytics' own, same ONNX model, on reference photos.
2. Stored G1 camera frames (logs/rgbd_runs) replayed as a 10 Hz video through
   the background worker.
3. Latency benchmark: imgsz 320/640 on CPU and on each GPU.

Run: ./scripts/isaac_python.sh scripts/test_yolo_images.py
"""
from pathlib import Path
import glob,json,os,sys,time
ROOT=Path(__file__).resolve().parents[1]; sys.path[:0]=[str(ROOT)]
# onnxruntime-gpu and its CUDA runtime ship inside Isaac Sim extensions.
isaac=os.environ.get('ISAAC_SIM_ROOT')
if isaac: sys.path+=[f'{isaac}/exts/isaacsim.pip.onnx/pip_prebundle',f'{isaac}/exts/isaacsim.pip.nv/pip_prebundle']
import numpy as np
from PIL import Image
from perception.yolo_detector import YoloOnnx,YoloWorker,annotate,load_config

IMAGES=ROOT/'assets/perception/test_images'
MODELS={320:('checkpoints/yolo/yolo11n.onnx','39aef0b882b27eb843e2e004d67b7aeebe598d44cd0dfda6d8232b7455944ff5'),640:('checkpoints/yolo/yolo11n_640.onnx','ce2c9408f475e758629ae369381535f2869426f38fa53037beaf705130f327dc')}

def config_for(imgsz,provider='cuda',device=0):
    config=load_config({'imgsz':imgsz,'provider':provider,'cuda_device_id':device})
    config['model'],config['model_sha256']=MODELS[imgsz]
    return config

def iou(a,b):
    w=max(0,min(a[2],b[2])-max(a[0],b[0])); h=max(0,min(a[3],b[3])-max(a[1],b[1]))
    return w*h/((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-w*h+1e-9)

def main():
    out=ROOT/'logs/yolo_runs'/('offline_'+time.strftime('%Y%m%d_%H%M%S')); out.mkdir(parents=True,exist_ok=True)
    report={'status':'running','output_directory':str(out),'tests':{}}
    detector=YoloOnnx(config_for(320)); detector640=YoloOnnx(config_for(640))
    report['provider']=detector.provider

    # ---- 1. reference photos: ONNX vs PyTorch ----------------------------
    reference=json.loads((IMAGES/'reference_ultralytics_onnx_320.json').read_text())['images']; rows=[]; ok=True
    for path in sorted(IMAGES.glob('*.jpg')):
        rgb=np.asarray(Image.open(path).convert('RGB')); detections,timing=detector.detect(rgb)
        Image.fromarray(annotate(rgb,detections,f'YOLO11n 320 | {path.name}')).save(out/f'photo_{path.stem}_320.png')
        wide,_=detector640.detect(rgb)
        Image.fromarray(annotate(rgb,wide,f'YOLO11n 640 | {path.name}')).save(out/f'photo_{path.stem}_640.png')
        row={'image':path.name,'size_wh':[rgb.shape[1],rgb.shape[0]],
             'onnx_320':[(d['class_name'],round(d['confidence'],3)) for d in detections],
             'onnx_640':[(d['class_name'],round(d['confidence'],3)) for d in wide]}
        if path.name in reference:
            expected=reference[path.name]; matched=0; worst=0.
            for e in expected:
                best=max((d for d in detections if d['class_name']==e['class_name']),key=lambda d:iou(d['bbox_xyxy'],e['bbox_xyxy']),default=None)
                if best and iou(best['bbox_xyxy'],e['bbox_xyxy'])>.9:
                    matched+=1; worst=max(worst,abs(best['confidence']-e['confidence']))
            row.update(ultralytics_320=[(e['class_name'],e['confidence']) for e in expected],matched=matched,max_confidence_difference=worst)
            ok&=matched==len(expected)==len(detections) and worst<.02
        rows.append(row)
    report['tests']['photos_vs_ultralytics_reference']={'passed':bool(ok),'images':rows}
    found={name for row in rows for name,_ in row['onnx_320']}; found640={name for row in rows for name,_ in row['onnx_640']}
    report['tests']['target_classes_on_photos']={'passed':{'person','chair'}<=found and 'bottle' in found640,
        'person_320':'person' in found,'chair_320':'chair' in found,'bottle_320':'bottle' in found,'bottle_640':'bottle' in found640,
        'classes_320':sorted(found),'classes_640':sorted(found640)}

    # ---- 2. stored G1 camera frames as a 10 Hz video ----------------------
    frames=sorted(glob.glob(str(ROOT/'logs/rgbd_runs/*/*_rgb.png')))
    if frames:
        images=[np.asarray(Image.open(f).convert('RGB')) for f in frames]
        worker=YoloWorker(detector,10); results={}; slowest=0.
        for cycle in range(5):  # replay the stored frames five times
            for index,image in enumerate(images):
                started=time.perf_counter()
                worker.submit(image,{'frame_index':cycle*len(images)+index,'sim_time':(cycle*len(images)+index)*.1,'source':Path(frames[index]).name})
                slowest=max(slowest,time.perf_counter()-started)
                time.sleep(.1); result=worker.latest()
                if result: results[result['source']]=result
        time.sleep(.3); stats=worker.stats(); worker.stop()
        summary={}
        for name,result in sorted(results.items()):
            summary[name]=[(d['class_name'],round(d['confidence'],3)) for d in result['detections']]
            Image.fromarray(annotate(result['rgb'],result['detections'],f'YOLO11n 320 | {name}')).save(out/('sim_'+name))
        report['tests']['stored_sim_frames_video']={'passed':stats['processed']>=.9*stats['submitted'] and slowest<.002,
            'frames':len(images),'worker':stats,'slowest_submit_call_ms':slowest*1000,'detections':summary,
            'note':'submit() is what the physics loop calls; it must return immediately'}
    else:
        report['tests']['stored_sim_frames_video']={'passed':None,'reason':'No stored frames; run scripts/test_rgbd_camera.py first'}

    # ---- 3. latency benchmark on a 640x360 camera frame -------------------
    frame=np.asarray(Image.open(frames[0]).convert('RGB')) if frames else np.zeros((360,640,3),dtype=np.uint8)
    bench=[]
    for imgsz in (320,640):
        for provider,device in (('cuda',0),('cuda',1),('cpu',0)):
            try: model=YoloOnnx(config_for(imgsz,provider,device))
            except Exception as error:
                bench.append({'imgsz':imgsz,'provider':provider,'device':device,'error':repr(error)[:200]}); continue
            for _ in range(20): model.detect(frame)
            timings=[model.detect(frame)[1] for _ in range(200)]
            bench.append({'imgsz':imgsz,'requested':f'{provider}:{device}','actual_provider':model.provider,
                **{key:{'mean':float(np.mean([t[key] for t in timings])),'p95':float(np.percentile([t[key] for t in timings],95))}
                   for key in ('preprocess_ms','inference_ms','postprocess_ms','total_ms')},
                'max_rate_hz':1000/float(np.mean([t['total_ms'] for t in timings]))})
    report['benchmark']=bench
    report['failed_tests']=[k for k,v in report['tests'].items() if v['passed'] is False]
    report['status']='passed' if not report['failed_tests'] else 'failed'
    payload=json.dumps(report,indent=2)
    (out/'yolo_offline_results.json').write_text(payload)
    (ROOT/'docs/perception').mkdir(parents=True,exist_ok=True); (ROOT/'docs/perception/yolo_offline_results.json').write_text(payload)
    for name,test in report['tests'].items(): print('YOLO_TEST '+json.dumps({name:test})[:1500],flush=True)
    for row in bench: print('YOLO_BENCH '+json.dumps(row),flush=True)
    print('YOLO_RESULT '+json.dumps({'status':report['status'],'failed_tests':report['failed_tests'],'images':str(out)}),flush=True)

if __name__=='__main__': main()
