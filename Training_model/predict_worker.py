"""Tiến trình chạy thử mô hình cho tab "Kiểm tra". Do giao diện khởi chạy qua ml_python.sh.

Giao tiếp bằng từng dòng JSON: nhận yêu cầu ở stdin, trả kết quả ở stdout.
Mô hình được giữ trong bộ nhớ giữa các ảnh nên ảnh sau chạy nhanh.
"""
import json,os,sys,time

protocol=os.fdopen(os.dup(1),'w')   # kênh trả lời riêng
os.dup2(2,1)                         # mọi thứ thư viện in ra đi sang stderr, không làm hỏng JSON


def reply(payload):
    protocol.write(json.dumps(payload,ensure_ascii=False)+'\n'); protocol.flush()


def main():
    model=None; loaded=None
    reply({'ready':True})
    for line in sys.stdin:
        line=line.strip()
        if not line: continue
        try:
            request=json.loads(line)
            if request.get('quit'): break
            key=(request['model'],os.path.getmtime(request['model']))
            if key!=loaded:
                from ultralytics import YOLO
                model=YOLO(request['model']); loaded=key
            started=time.perf_counter()
            result=model.predict(request['image'],conf=float(request.get('conf',.25)),iou=float(request.get('iou',.45)),
                                 imgsz=int(request.get('imgsz',640)),device=str(request.get('device','0')),verbose=False)[0]
            elapsed=(time.perf_counter()-started)*1000
            height,width=result.orig_shape
            detections=[{'class_id':int(c),'class_name':result.names[int(c)],'confidence':float(p),'bbox_xyxy':[float(v) for v in b]}
                        for c,p,b in zip(result.boxes.cls.tolist(),result.boxes.conf.tolist(),result.boxes.xyxy.tolist())]
            reply({'id':request.get('id'),'detections':detections,'width':int(width),'height':int(height),'time_ms':elapsed,
                   'classes':{str(k):v for k,v in result.names.items()}})
        except Exception as error:
            reply({'id':None,'error':f'{type(error).__name__}: {error}'})


if __name__=='__main__': main()
