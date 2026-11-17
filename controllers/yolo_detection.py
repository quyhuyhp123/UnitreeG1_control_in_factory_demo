"""Nonblocking RGB-only inference client. No navigation/locomotion dependencies."""
from pathlib import Path
import json, os, queue, struct, subprocess, threading, time
from datetime import datetime
ROOT=Path(r'D:\unitree_g1')

def load_config():
    cfg=json.loads((ROOT/'configs/yolo_detection.json').read_text())
    if not (32<=cfg['imgsz']<=640 and cfg['imgsz']%32==0 and 0<cfg['target_hz']<=10):
        raise ValueError('Invalid YOLO imgsz or target_hz')
    if not (0<cfg['confidence']<1 and 0<cfg['iou']<1 and 1<=cfg['cpu_threads']<=4):
        raise ValueError('Invalid YOLO threshold or CPU thread limit')
    for key in ['python','model']:
        if not (ROOT/cfg[key]).is_file():
            raise FileNotFoundError(f'YOLO {key} missing: {ROOT/cfg[key]}; run scripts/setup_yolo.ps1')
    return cfg

class AsyncYoloDetector:
    def __init__(self,output,config=None):
        self.cfg=config or load_config()
        self.output=Path(output)/'yolo';self.output.mkdir(parents=True,exist_ok=True)
        self.pending=queue.Queue(maxsize=1)
        self.stop=threading.Event()
        self.enabled=True
        self.ready=False
        self.error=None
        self.latest=None
        self.completed=0
        self.dropped=0
        self.generation=0
        self.lock=threading.Lock()
        self.window=None
        self.last_display=0
        self.started=time.perf_counter()
        env=os.environ.copy()
        # Kit sets PYTHONPATH/PYTHONHOME to its own extensions. Never leak them
        # into the independent interpreter or torch/NumPy DLL resolution.
        for key in ['PYTHONPATH','PYTHONHOME','PYTHONSTARTUP','PYTHONEXE']:
            env.pop(key,None)
        env['PYTHONNOUSERSITE']='1'
        settings_dir=ROOT/'logs/yolo_settings'
        settings_dir.mkdir(parents=True,exist_ok=True)
        env['YOLO_CONFIG_DIR']=str(settings_dir)
        env['OMP_NUM_THREADS']=str(self.cfg['cpu_threads'])
        env['MKL_NUM_THREADS']=str(self.cfg['cpu_threads'])
        env['YOLO_AUTOINSTALL']='False'
        self.stderr=(self.output/'worker_stderr.log').open('wb')
        self.process=subprocess.Popen([str(ROOT/self.cfg['python']),'-u',str(ROOT/'controllers/yolo_worker.py')],
            cwd=str(ROOT),env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.stderr,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),bufsize=0)
        self.thread=threading.Thread(target=self._run,name='yolo-pipe-worker',daemon=True)
        self.thread.start()

    def _receive(self):
        line=self.process.stdout.readline()
        if not line: raise RuntimeError(f'YOLO worker exited (code {self.process.poll()}); see worker_stderr.log')
        payload=json.loads(line)
        if payload['type']=='error': raise RuntimeError(payload['message'])
        return payload

    def _run(self):
        try:
            hello=self._receive()
            if hello['type']!='ready': raise RuntimeError('Invalid YOLO startup response')
            self.ready=True
            last_dispatch=0
            with (self.output/'detections.jsonl').open('w',encoding='utf-8') as log:
                while not self.stop.is_set():
                    # Wait before selecting a frame, so the latest slot is used.
                    if self.stop.wait(max(0,1/self.cfg['target_hz']-(time.perf_counter()-last_dispatch))): break
                    try: item=self.pending.get(timeout=.1)
                    except queue.Empty: continue
                    if not self.enabled: continue
                    rgb,meta,generation,depth,depth_meta=item
                    start=time.perf_counter()
                    header=json.dumps({'shape_hw':list(rgb.shape[:2]),**meta}).encode('utf-8')
                    packet=struct.pack('<I',len(header))+header+rgb.tobytes()
                    # Pipe backpressure is confined to this background thread.
                    view=memoryview(packet)
                    while view:
                        written=self.process.stdin.write(view)
                        if not written: raise RuntimeError('YOLO frame pipe broken')
                        view=view[written:]
                    self.process.stdin.flush()
                    last_dispatch=start
                    result=self._receive()
                    if depth is not None:
                        from controllers.obstacle_localization import DepthLocalizer
                        localizer=DepthLocalizer()
                        if localizer.cfg['enabled']:
                            semantic=localizer.semantic(depth,depth_meta,result)
                            result['obstacle_localization']=localizer.fuse(localizer.geometry(depth,depth_meta),semantic)
                    result['ipc_roundtrip_ms']=(time.perf_counter()-start)*1000
                    result['frame_age_ms']=(time.time()-meta['acquisition_unix_s'])*1000
                    result['received_perf_counter']=time.perf_counter()
                    result['dropped_frames_total']=self.dropped
                    if generation==self.generation and self.enabled:
                        with self.lock:
                            self.latest=(rgb,result)
                            self.completed+=1
                        log.write(json.dumps(result)+'\n');log.flush()
        except Exception as error:
            if not self.stop.is_set(): self.error=str(error)

    def submit(self,rgb,metadata,depth=None):
        if not self.enabled or self.error or self.stop.is_set(): return
        meta={'frame_counter':metadata['frame_counter'],
              'acquisition_utc':metadata['acquisition_utc'],
              'acquisition_unix_s':metadata.get('acquisition_unix_s',time.time()),
              'acquisition_sim_time_s':metadata.get('acquisition_sim_time_s'),
              'source':metadata.get('source','G1 RGB-D camera')}
        item=(rgb.copy(),meta,self.generation,None if depth is None else depth.copy(),dict(metadata))
        try: self.pending.put_nowait(item)
        except queue.Full:
            try: self.pending.get_nowait();self.dropped+=1
            except queue.Empty: pass
            try: self.pending.put_nowait(item)
            except queue.Full: self.dropped+=1

    def set_enabled(self,value):
        self.enabled=bool(value);self.generation+=1
        with self.lock: self.latest=None
        while True:
            try: self.pending.get_nowait()
            except queue.Empty: break

    def clear(self):
        self.set_enabled(self.enabled)

    def annotated(self):
        import cv2
        with self.lock: latest=self.latest
        if latest is None: return None
        rgb,result=latest
        image=rgb.copy()
        for det in result['detections']:
            x1,y1,x2,y2=[int(round(v)) for v in det['bbox_xyxy']]
            cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,80),2)
            cv2.putText(image,f"{det['class_name']} {det['confidence']:.2f}",
                (max(0,x1),max(15,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,80),1,cv2.LINE_AA)
        return image,result

    def build_debug(self):
        import omni.ui as ui
        self.window=ui.Window('G1 YOLO11n | RGB detections only',width=660,height=440)
        self.provider=ui.ByteImageProvider()
        with self.window.frame:
            with ui.VStack():
                self.status=ui.Label('Starting isolated YOLO worker...',height=46,word_wrap=True)
                ui.ImageWithProvider(self.provider,height=360)
                with ui.HStack(height=26):
                    ui.Button('Enable YOLO',clicked_fn=lambda:self.set_enabled(True))
                    ui.Button('Disable YOLO',clicked_fn=lambda:self.set_enabled(False))
                    ui.Button('Save detections',clicked_fn=self.save)

    def update_debug(self):
        if self.window is None or not self.window.visible: return
        status='ERROR: '+self.error if self.error else ('disabled' if not self.enabled else ('ready' if self.ready else 'loading'))
        with self.lock: latest=self.latest
        if latest is None:
            self.status.text='YOLO: '+status+' | no detection frame yet'
            if self.last_display:
                self.provider.set_bytes_data([0,0,0,255],[1,1])
                self.last_display=0
            return
        result=latest[1]
        age=time.perf_counter()-result['received_perf_counter']
        self.status.text=(f"YOLO: {status} | {result['inference_total_ms']:.1f} ms | "
                          f"{len(result['detections'])} objects | result age {age:.2f}s"+
                          (' | STALE' if age>self.cfg['stale_after_s'] else ''))
        if self.last_display==result['received_perf_counter']: return
        self.last_display=result['received_perf_counter']
        import numpy as np
        image,_=self.annotated();h,w=image.shape[:2]
        rgba=np.concatenate([image,np.full((h,w,1),255,dtype=np.uint8)],axis=2)
        self.provider.set_bytes_data(rgba.ravel().tolist(),[w,h])

    def save(self,name=None):
        item=self.annotated()
        if item is None: return None
        import cv2
        image,result=item
        directory=self.output/(name or datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        directory.mkdir(parents=True,exist_ok=False)
        cv2.imwrite(str(directory/'detections.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
        (directory/'detections.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        return directory

    def close(self):
        self.stop.set()
        if self.process.poll() is None:
            self.process.terminate()
        self.thread.join(timeout=3)
        if self.process.poll() is None: self.process.kill()
        self.process.wait(timeout=3)
        self.process.stdin.close();self.process.stdout.close();self.stderr.close()
        if self.window is not None: self.window.destroy();self.window=None
