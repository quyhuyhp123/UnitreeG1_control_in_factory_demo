"""Xuất best.pt sang ONNX (định dạng dự án mô phỏng G1 dùng). Do giao diện khởi chạy."""
from pathlib import Path
import hashlib,json,shutil,sys


def main():
    run_dir=Path(sys.argv[1]); imgsz=int(sys.argv[2]); best=run_dir/'weights/best.pt'
    if not best.is_file(): raise SystemExit('Chưa có best.pt trong '+str(run_dir))
    from ultralytics import YOLO
    model=YOLO(str(best))
    exported=Path(model.export(format='onnx',imgsz=imgsz,opset=17,simplify=False,dynamic=False))
    target=run_dir/'weights'/f'best_{imgsz}.onnx'; shutil.move(str(exported),target)
    names=run_dir/'weights'/'class_names.json'
    names.write_text(json.dumps({str(k):v for k,v in model.names.items()},ensure_ascii=False,indent=1),encoding='utf-8')
    digest=hashlib.sha256(target.read_bytes()).hexdigest()
    info={'onnx':str(target),'class_names':str(names),'sha256':digest,'imgsz':imgsz,
        'cach_dung_cho_robot_mo_phong':['Chép file .onnx và class_names.json vào factory_v2/checkpoints/yolo/',
            'Sửa trong factory_v2/configs/yolo.json: "model", "model_sha256" (giá trị sha256 ở trên), "class_names", "imgsz"',
            'Lưu ý: perception của robot coi class id 0 là "người" (configs/local_costmap.json: human_class_id)']}
    (run_dir/'weights'/f'best_{imgsz}_export.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    print('EXPORT_OK '+json.dumps(info,ensure_ascii=False),flush=True)


if __name__=='__main__': main()
