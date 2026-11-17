"""YOLO11 object detection on the G1 RGB camera (ONNX Runtime, background thread).

Output only: class name, confidence, pixel bounding box and timestamps. This
module never issues velocity commands and is not read by navigation,
locomotion or the safety supervisor. Detections come from the RGB image
alone; USD semantic labels are never used.

`YoloOnnx` is pure numpy + onnxruntime and also runs outside Isaac Sim.
"""
from pathlib import Path
import json,threading,time
import numpy as np
from perception.yolo_config import load_config,verify_model,ROOT

PALETTE=[(255,56,56),(72,249,10),(0,194,255),(255,178,29),(207,210,49),(146,204,23),(61,219,134),(26,147,52),(0,212,187),(255,149,200),(132,56,255),(82,0,133)]


def resize(image,width,height):
    try:
        import cv2
        return cv2.resize(image,(width,height),interpolation=cv2.INTER_LINEAR)
    except ImportError:
        from PIL import Image
        return np.asarray(Image.fromarray(image).resize((width,height),Image.BILINEAR))


def nms(boxes,scores,threshold):
    order=scores.argsort()[::-1]; keep=[]
    area=(boxes[:,2]-boxes[:,0])*(boxes[:,3]-boxes[:,1])
    while order.size:
        i=order[0]; keep.append(i)
        rest=order[1:]
        width=np.clip(np.minimum(boxes[i,2],boxes[rest,2])-np.maximum(boxes[i,0],boxes[rest,0]),0,None)
        height=np.clip(np.minimum(boxes[i,3],boxes[rest,3])-np.maximum(boxes[i,1],boxes[rest,1]),0,None)
        overlap=width*height
        order=rest[overlap/(area[i]+area[rest]-overlap+1e-9)<=threshold]
    return keep


def preload_cuda_runtime():
    """Make the CUDA 12 / cuDNN 9 libraries from the nvidia-* pip packages visible to ONNX Runtime.

    Isaac Sim ships them in `isaacsim.pip.nv` but not on the loader path, so the
    CUDA provider cannot find libcudnn.so.9 by itself. Loading them by full path
    changes nothing on disk and nothing in Isaac Sim's own packages.
    """
    import ctypes,sys
    loaded=[]
    for base in sys.path:
        root=Path(base)/'nvidia'
        if not (root/'cudnn/lib').is_dir(): continue
        for package in ['cuda_runtime','nvjitlink','cublas','cufft','curand','cusparse','cusolver','cuda_nvrtc','cudnn']:
            for library in sorted((root/package/'lib').glob('*.so*')):
                try: ctypes.CDLL(str(library),mode=ctypes.RTLD_GLOBAL); loaded.append(library.name)
                except OSError: pass
        break
    return loaded


class YoloOnnx:
    def __init__(self,config):
        import onnxruntime as ort
        self.config=config; self.imgsz=config['imgsz']
        self.names=json.loads((ROOT/config['class_names']).read_text())
        self.names={int(k):v for k,v in self.names.items()}
        model=verify_model(config)
        providers=['CPUExecutionProvider']
        if config['provider']=='cuda':
            preload_cuda_runtime()
            providers=[('CUDAExecutionProvider',{'device_id':int(config['cuda_device_id'])}),'CPUExecutionProvider']
        options=ort.SessionOptions(); options.intra_op_num_threads=2; options.inter_op_num_threads=1; options.log_severity_level=3
        self.session=ort.InferenceSession(str(model),sess_options=options,providers=providers)
        self.provider=self.session.get_providers()[0]
        entry=self.session.get_inputs()[0]; self.input_name=entry.name
        shape=entry.shape; fixed=[d for d in shape[2:] if isinstance(d,int)]
        if fixed and fixed!=[self.imgsz,self.imgsz]: raise ValueError(f'Model input {shape} does not match imgsz {self.imgsz}')
        output=self.session.get_outputs()[0].shape
        if output[1]!=4+len(self.names): raise ValueError(f'Unexpected YOLO output shape {output}')
        wanted=config.get('classes'); self.class_filter=None
        if wanted:
            index={v:k for k,v in self.names.items()}
            self.class_filter=np.array([index[name] for name in wanted])
        for _ in range(3): self.session.run(None,{self.input_name:np.zeros((1,3,self.imgsz,self.imgsz),dtype=np.float32)})

    def preprocess(self,rgb):
        """Letterbox to imgsz x imgsz (grey 114 padding), RGB, 0-1, NCHW."""
        height,width=rgb.shape[:2]; gain=min(self.imgsz/height,self.imgsz/width)
        new_w,new_h=round(width*gain),round(height*gain)
        pad_w,pad_h=(self.imgsz-new_w)/2,(self.imgsz-new_h)/2
        left,top=round(pad_w-.1),round(pad_h-.1)
        canvas=np.full((self.imgsz,self.imgsz,3),114,dtype=np.uint8)
        canvas[top:top+new_h,left:left+new_w]=resize(rgb,new_w,new_h) if (new_w,new_h)!=(width,height) else rgb
        tensor=np.ascontiguousarray(canvas.transpose(2,0,1)[None].astype(np.float32)/255.)
        return tensor,(gain,left,top)

    def postprocess(self,output,transform,shape):
        gain,left,top=transform; height,width=shape[:2]
        prediction=output[0].T  # (anchors, 4 + classes)
        scores=prediction[:,4:]
        if self.class_filter is not None:
            masked=np.zeros_like(scores); masked[:,self.class_filter]=scores[:,self.class_filter]; scores=masked
        classes=scores.argmax(1); confidence=scores[np.arange(len(scores)),classes]
        keep=confidence>self.config['confidence_threshold']
        if not keep.any(): return []
        xywh=prediction[keep,:4]; classes=classes[keep]; confidence=confidence[keep]
        boxes=np.stack([xywh[:,0]-xywh[:,2]/2,xywh[:,1]-xywh[:,3]/2,xywh[:,0]+xywh[:,2]/2,xywh[:,1]+xywh[:,3]/2],axis=1)
        # Class-aware NMS: shift each class into its own coordinate range.
        selected=nms(boxes+classes[:,None]*4096.,confidence,self.config['iou_threshold'])[:self.config['max_detections']]
        boxes=boxes[selected]; boxes[:,[0,2]]=(boxes[:,[0,2]]-left)/gain; boxes[:,[1,3]]=(boxes[:,[1,3]]-top)/gain
        boxes[:,[0,2]]=boxes[:,[0,2]].clip(0,width); boxes[:,[1,3]]=boxes[:,[1,3]].clip(0,height)
        return [{'class_id':int(c),'class_name':self.names[int(c)],'confidence':float(p),'bbox_xyxy':[float(v) for v in b]}
                for c,p,b in zip(classes[selected],confidence[selected],boxes)]

    def detect(self,rgb):
        """rgb: HxWx3 uint8. Returns (detections, timing in ms)."""
        t0=time.perf_counter(); tensor,transform=self.preprocess(rgb)
        t1=time.perf_counter(); output=self.session.run(None,{self.input_name:tensor})[0]
        t2=time.perf_counter(); detections=self.postprocess(output,transform,rgb.shape)
        t3=time.perf_counter()
        return detections,{'preprocess_ms':(t1-t0)*1000,'inference_ms':(t2-t1)*1000,'postprocess_ms':(t3-t2)*1000,'total_ms':(t3-t0)*1000}


def annotate(rgb,detections,footer=None):
    """Draw boxes, class names and confidences. Returns a new uint8 image."""
    from PIL import Image,ImageDraw
    image=Image.fromarray(rgb); draw=ImageDraw.Draw(image)
    for d in detections:
        colour=PALETTE[d['class_id']%len(PALETTE)]; x0,y0,x1,y1=d['bbox_xyxy']
        draw.rectangle([x0,y0,x1,y1],outline=colour,width=2)
        label=f"{d['class_name']} {d['confidence']:.2f}"
        if d.get('distance_m') is not None: label+=f" {d['distance_m']:.2f}m"
        box=draw.textbbox((0,0),label); w,h=box[2]-box[0]+4,box[3]-box[1]+4; y=y0-h if y0>=h else y0
        draw.rectangle([x0,y,x0+w,y+h],fill=colour); draw.text((x0+2,y+1),label,fill=(255,255,255))
    if footer:
        box=draw.textbbox((0,0),footer); draw.rectangle([0,image.height-box[3]-6,box[2]+6,image.height],fill=(0,0,0))
        draw.text((3,image.height-box[3]-3),footer,fill=(255,255,255))
    return np.asarray(image)


class YoloWorker:
    """Runs detection off the simulation thread. Keeps only the newest pending frame."""
    def __init__(self,detector,target_hz):
        self.detector=detector; self.period=1./target_hz
        self.lock=threading.Lock(); self.wake=threading.Event(); self.running=True
        self.pending=None; self.result=None; self.error=None
        self.post=None  # optional callable(payload, detections) -> dict merged into the result, run on this thread
        self.submitted=0; self.processed=0; self.dropped=0; self.history=[]
        self.thread=threading.Thread(target=self._loop,name='yolo-inference',daemon=True); self.thread.start()

    def submit(self,rgb,stamp,payload=None):
        """Constant time: store a reference and return. Never waits for inference."""
        with self.lock:
            if self.pending is not None: self.dropped+=1
            self.pending=(rgb,stamp,time.perf_counter(),payload); self.submitted+=1
        self.wake.set()

    def latest(self):
        with self.lock: return self.result

    def _loop(self):
        last=0.
        while self.running:
            if not self.wake.wait(.2): continue
            wait=self.period-(time.perf_counter()-last)
            if wait>0: time.sleep(wait)
            with self.lock:
                job=self.pending; self.pending=None; self.wake.clear()
            if job is None: continue
            rgb,stamp,submitted,payload=job; last=time.perf_counter(); extra={}
            try:
                detections,timing=self.detector.detect(rgb)
                if self.post is not None and payload is not None:
                    extra=self.post(payload,detections); timing['localization_ms']=extra.get('localization_ms',0.)
            except Exception as error:  # reported by the owner on the main thread
                self.error=repr(error); self.running=False; return
            done=time.perf_counter()
            timing['queue_ms']=(last-submitted)*1000; timing['submit_to_result_ms']=(done-submitted)*1000
            result={**stamp,**extra,'result_index':self.processed+1,'wall_time_result':time.time(),'detections':detections,'timing':timing,'rgb':rgb}
            with self.lock:
                self.result=result; self.processed+=1; self.history.append((done,timing['inference_ms'],timing['submit_to_result_ms']))
                del self.history[:-200]

    def stats(self):
        with self.lock: history=list(self.history); counts=(self.submitted,self.processed,self.dropped)
        if len(history)<2: return {'submitted':counts[0],'processed':counts[1],'dropped':counts[2]}
        span=history[-1][0]-history[0][0]
        return {'submitted':counts[0],'processed':counts[1],'dropped':counts[2],'inference_hz':(len(history)-1)/span if span>0 else 0.,
                'inference_ms_mean':float(np.mean([h[1] for h in history])),'inference_ms_p95':float(np.percentile([h[1] for h in history],95)),
                'submit_to_result_ms_mean':float(np.mean([h[2] for h in history])),'submit_to_result_ms_p95':float(np.percentile([h[2] for h in history],95))}

    def stop(self):
        self.running=False; self.wake.set(); self.thread.join(timeout=5)


class YoloPerception:
    """Glue between the RGB-D camera frames and the detector inside the Isaac Sim runtime."""
    def __init__(self,runtime,config):
        import omni.kit.app
        manager=omni.kit.app.get_app().get_extension_manager()
        for name in ['isaacsim.pip.onnx','isaacsim.pip.nv']: manager.set_extension_enabled_immediate(name,True)
        self.runtime=runtime; self.config=config
        self.detector=YoloOnnx(config); self.worker=YoloWorker(self.detector,config['target_hz'])
        self.directory=ROOT/config['save_directory']/runtime.directory.name; self.directory.mkdir(parents=True,exist_ok=True)
        self.log=(self.directory/'detections.jsonl').open('w',encoding='utf-8')
        self.logged=0; self.saved=0; self.result=None
        # Optional RGB-D obstacle localisation, run on the inference thread with the very frame YOLO saw.
        self.localizer=None; self.obstacle_log=None
        if runtime.factory_cfg.get('obstacle_localization',{}).get('enabled',False):
            from perception.obstacle_localizer import ObstacleLocalizer
            settings=runtime.factory_cfg['obstacle_localization']
            self.localizer=ObstacleLocalizer(settings); self.worker.post=self._localize
            self.obstacle_directory=ROOT/settings['save_directory']/runtime.directory.name; self.obstacle_directory.mkdir(parents=True,exist_ok=True)
            self.obstacle_log=(self.obstacle_directory/'obstacles.jsonl').open('w',encoding='utf-8')
        # Optional local costmap, updated on the same thread from each localised frame.
        self.costmap=None; self.costmap_log=None
        if runtime.factory_cfg.get('local_costmap',{}).get('enabled',False):
            if self.localizer is None: raise ValueError('The local costmap needs obstacle localisation enabled')
            from perception.local_costmap import LocalCostmap
            settings=runtime.factory_cfg['local_costmap']; map_file=runtime.factory_cfg.get('navigation_map')
            navigation_map=json.loads((ROOT/map_file).read_text(encoding='utf-8-sig')) if map_file and not runtime.flat else None
            self.costmap=LocalCostmap(settings,navigation_map)
            self.costmap_directory=ROOT/settings['save_directory']/runtime.directory.name; self.costmap_directory.mkdir(parents=True,exist_ok=True)
            self.costmap_log=(self.costmap_directory/'costmap.jsonl').open('w',encoding='utf-8')
        (self.directory/'model.json').write_text(json.dumps({'model':config['model'],'sha256':config['model_sha256'],'imgsz':config['imgsz'],
            'provider':self.detector.provider,'cuda_device_id':config['cuda_device_id'],'target_hz':config['target_hz']},indent=2))

    def submit(self,frame):
        """Hand the newest camera frame to the worker. Does not block the physics loop."""
        self.worker.submit(frame['rgb'],{'frame_index':frame['index'],'sim_time':frame['sim_time'],'control_step':frame['control_step'],
                                         'wall_time_capture':frame['wall_time'],'image_size_wh':[frame['rgb'].shape[1],frame['rgb'].shape[0]]},
                           payload=frame if self.localizer is not None else None)

    def _localize(self,frame,detections):
        """Inference thread: depth and pose come from the same frame object as the RGB image."""
        located=self.localizer.process(frame,detections)
        located['_grid']=self.localizer.grid
        if self.costmap is not None:
            pose=located['robot_pose_world']
            snapshot=self.costmap.update(self.localizer.observation,(pose['x'],pose['y']),located['sim_time'])
            located['_costmap']=snapshot; located['costmap']=self.costmap.summary(snapshot)
        return located

    def poll(self):
        """Main thread, every tick: pick up a finished result and log it."""
        if self.worker.error: raise RuntimeError('YOLO inference thread failed: '+self.worker.error)
        result=self.worker.latest()
        if result is None or result['result_index']==self.logged: return None
        self.logged=result['result_index']; self.result=result
        self.log.write(json.dumps(self.record(result))+'\n')
        if self.obstacle_log is not None and 'obstacles' in result:
            self.obstacle_log.write(json.dumps({k:result[k] for k in ('sim_time','frame_index','control_step','robot_pose_world','obstacles','points','sync')})+'\n')
            if self.logged%20==0: self.obstacle_log.flush()
        if self.costmap_log is not None and 'costmap' in result:
            self.costmap_log.write(json.dumps(result['costmap'])+'\n')
            if self.logged%20==0: self.costmap_log.flush()
        if self.logged%20==0: self.log.flush()
        return result

    @staticmethod
    def record(result): return {k:v for k,v in result.items() if k!='rgb' and not k.startswith('_')}

    def costmap_image(self,result=None,ground_truth=None):
        """Debug picture of the newest costmap (None until the first update)."""
        from perception.local_costmap import render
        result=result or self.result
        if result is None or '_costmap' not in result: return None
        return render(result['_costmap'],result['robot_pose_world']['yaw_rad'],self.detector.names,
                      self.runtime.factory_cfg['rgbd_camera']['horizontal_fov_deg'],ground_truth=ground_truth)

    def save_costmap(self,result=None,tag='costmap',ground_truth=None):
        """Costmap picture, the camera view it came from, raw layers (.npz) and a JSON summary."""
        from PIL import Image
        result=result or self.result
        if result is None or '_costmap' not in result: raise RuntimeError('No costmap available yet')
        self.saved+=1; stem=self.costmap_directory/f'{self.saved:03d}_{tag}'; snapshot=result['_costmap']
        Image.fromarray(self.costmap_image(result,ground_truth)).save(str(stem)+'_costmap.png')
        Image.fromarray(annotate(result['rgb'],result['detections'],f"t={result['sim_time']:.2f} s")).save(str(stem)+'_rgb.png')
        np.savez_compressed(str(stem)+'_layers.npz',state=snapshot['state'],cost=snapshot['cost'],static_lethal=snapshot['static_lethal'],
                            class_id=snapshot['class_id'],traversable=snapshot['traversable'])
        Path(str(stem)+'.json').write_text(json.dumps({**result['costmap'],'robot_pose_world':result['robot_pose_world'],
            'layers':'state: 0 unknown, 1 free, 2 occupied | cost: 0 free, 1-252 inflation, 253 inscribed, 254 lethal, 255 no information',
            'array_indexing':'[ix, iy]; cell covers world origin + [ix, ix+1) * resolution along X, same for Y',
            'resolution_m':snapshot['resolution_m'],'size_cells':snapshot['size_cells']},indent=2))
        return stem

    def save_obstacles(self,result=None,tag='obstacles',ground_truth=None):
        """Annotated RGB (class, confidence, distance), bird's-eye view and JSON for one localised frame."""
        from PIL import Image
        from perception.obstacle_localizer import render_topdown
        result=result or self.result
        if result is None or 'obstacles' not in result: raise RuntimeError('No obstacle localisation result available yet')
        self.saved+=1; stem=self.obstacle_directory/f'{self.saved:03d}_{tag}'
        footer=f"t={result['sim_time']:.2f} s | {len(result['obstacles'])} obstacles | localisation {result['timing']['localization_ms']:.1f} ms"
        Image.fromarray(annotate(result['rgb'],result['detections'],footer)).save(str(stem)+'_rgb.png')
        Image.fromarray(render_topdown(result,result.get('_grid'),ground_truth)).save(str(stem)+'_topdown.png')
        Path(str(stem)+'.json').write_text(json.dumps(self.record(result),indent=2))
        return stem

    def overlay(self,rgb):
        """Debug-window overlay: latest detections drawn on the given image."""
        result=self.result
        if result is None: return rgb
        stats=self.worker.stats()
        footer=f"YOLO {self.config['imgsz']} | {len(result['detections'])} det | {result['timing']['inference_ms']:.1f} ms | {stats.get('inference_hz',0):.1f} Hz | frame {result['frame_index']}"
        return annotate(rgb,result['detections'],footer)

    def save(self,result=None,tag='detections'):
        """Write the exact RGB image the detector saw with its boxes, plus JSON."""
        from PIL import Image
        result=result or self.result
        if result is None: raise RuntimeError('No YOLO result available yet')
        self.saved+=1; stem=self.directory/f'{self.saved:03d}_{tag}'
        footer=f"YOLO11n {self.config['imgsz']} | t={result['sim_time']:.2f} s | {result['timing']['inference_ms']:.1f} ms"
        Image.fromarray(annotate(result['rgb'],result['detections'],footer)).save(str(stem)+'.png')
        Path(str(stem)+'.json').write_text(json.dumps(self.record(result),indent=2))
        return stem

    def close(self):
        self.worker.stop(); self.log.close()
        if self.obstacle_log is not None: self.obstacle_log.close()
        if self.costmap_log is not None: self.costmap_log.close()
