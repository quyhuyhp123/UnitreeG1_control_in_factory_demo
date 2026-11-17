"""Standalone CPU inference process. stdin binary RGB packets, stdout JSONL.

Never import this module into Isaac. No networking, USD labels or control output.
"""
from pathlib import Path
import contextlib, json, os, struct, sys, time
ROOT=Path(r'D:\unitree_g1')

def read_exact(stream, length):
    chunks=[]
    while length:
        block=stream.read(length)
        if not block: raise EOFError('Frame pipe closed')
        chunks.append(block); length-=len(block)
    return b''.join(chunks)

class Detector:
    def __init__(self,cfg):
        import torch
        from ultralytics import YOLO
        self.cfg=cfg
        if cfg['device']!='cpu':
            raise ValueError('This isolated worker is validated for CPU only; CUDA backend requires separate validation')
        torch.set_num_threads(cfg['cpu_threads'])
        self.model=YOLO(str(ROOT/cfg['model']),task='detect')
        import numpy as np
        self.predict(np.zeros((360,640,3),dtype=np.uint8),{})

    def predict(self,rgb,meta):
        start=time.perf_counter()
        # Ultralytics numpy source expects BGR; camera input is explicitly RGB.
        result=self.model.predict(source=rgb[...,::-1].copy(),imgsz=self.cfg['imgsz'],
            conf=self.cfg['confidence'],iou=self.cfg['iou'],max_det=self.cfg['max_detections'],
            device=self.cfg['device'],verbose=False,save=False)[0]
        detections=[]
        for box in result.boxes:
            cls=int(box.cls.item())
            detections.append({'class_id':cls,'class_name':self.model.names[cls],
                'confidence':float(box.conf.item()),'bbox_xyxy':box.xyxy[0].tolist()})
        return {'type':'result','frame_counter':meta.get('frame_counter'),
            'acquisition_utc':meta.get('acquisition_utc'),
            'acquisition_sim_time_s':meta.get('acquisition_sim_time_s'),
            'source':meta.get('source','G1 RGB-D camera'),
            'completed_unix_s':time.time(),'inference_total_ms':(time.perf_counter()-start)*1000,
            'ultralytics_speed_ms':result.speed,'image_shape_hw':list(rgb.shape[:2]),
            'detections':detections,'model':'yolo11n.pt','device':self.cfg['device'],
            'imgsz':self.cfg['imgsz']}

def main():
    # Only protocol JSON uses stdout; third-party startup output belongs to stderr.
    protocol_out=sys.stdout
    def send(value):
        protocol_out.write(json.dumps(value)+'\n');protocol_out.flush()
    try:
        cfg=json.loads((ROOT/'configs/yolo_detection.json').read_text())
        with contextlib.redirect_stdout(sys.stderr): detector=Detector(cfg)
        send({'type':'ready','model':cfg['model'],'device':cfg['device']})
        import numpy as np
        while True:
            prefix=sys.stdin.buffer.read(4)
            if not prefix: break
            if len(prefix)!=4: raise ValueError('Truncated frame prefix')
            size=struct.unpack('<I',prefix)[0]
            if size>65536: raise ValueError('Invalid metadata length')
            meta=json.loads(read_exact(sys.stdin.buffer,size))
            h,w=meta['shape_hw']
            if not (0<h<=1080 and 0<w<=1920): raise ValueError('Invalid RGB dimensions')
            rgb=np.frombuffer(read_exact(sys.stdin.buffer,h*w*3),dtype=np.uint8).reshape(h,w,3)
            with contextlib.redirect_stdout(sys.stderr): result=detector.predict(rgb,meta)
            send(result)
    except EOFError:
        pass
    except BaseException as error:
        import traceback
        traceback.print_exc(file=sys.stderr)
        send({'type':'error','message':str(error)})
        raise SystemExit(1)

if __name__=='__main__': main()
