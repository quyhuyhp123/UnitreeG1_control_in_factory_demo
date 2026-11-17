"""Tiến trình huấn luyện YOLO. Được giao diện (app.py) khởi chạy, không chạy trực tiếp.

Đọc cấu hình từ một file JSON, chuyển dữ liệu LabelMe sang YOLO rồi gọi
Ultralytics. Mọi thứ in ra màn hình được giao diện hiển thị ở khung "Nhật ký".
"""
from pathlib import Path
import json,sys,time,traceback

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))


def main():
    config=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
    run_dir=Path(config['run_dir']); status_path=run_dir/'status.json'
    def status(state,**extra):
        status_path.write_text(json.dumps({'state':state,'time':time.time(),**extra},ensure_ascii=False,indent=2),encoding='utf-8')
    try:
        status('preparing')
        from labelme_to_yolo import convert
        print(f'[1/3] Chuyển nhãn LabelMe sang YOLO: {config["data_dir"]}',flush=True)
        stats=convert(config['data_dir'],run_dir/'dataset',config['class_names'],config.get('include_background',True))
        (run_dir/'dataset_stats.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf-8')
        for split,info in stats['splits'].items():
            print(f'      {split}: {info["images"]} ảnh, {info["boxes"]} khung, {info["background_images"]} ảnh nền, bỏ qua {info["skipped_images"]}',flush=True)
        if stats['ignored_labels']: print(f'      Nhãn bị bỏ qua (không nằm trong danh sách): {stats["ignored_labels"]}',flush=True)
        if not stats['splits']['train']['boxes']: raise RuntimeError('Không có khung nào trong tập train sau khi chuyển đổi. Kiểm tra lại tên nhãn.')

        print('[2/3] Nạp Ultralytics và mô hình gốc...',flush=True)
        import torch
        from ultralytics import YOLO
        print(f'      torch {torch.__version__} | CUDA: {torch.cuda.is_available()} | thiết bị: {config["device"]}',flush=True)
        resume=config.get('resume_from')
        model=YOLO(resume if resume else config['base_model'])
        status('training',epochs=config['epochs'])
        print(f'[3/3] Bắt đầu huấn luyện: {config["epochs"]} epoch, batch {config["batch"]}, imgsz {config["imgsz"]}',flush=True)
        if resume:
            model.train(resume=True)
        else:
            model.train(data=stats['data_yaml'],epochs=int(config['epochs']),batch=int(config['batch']),imgsz=int(config['imgsz']),
                device=config['device'],project=str(run_dir.parent),name=run_dir.name,exist_ok=True,
                patience=int(config.get('patience',50)),workers=int(config.get('workers',8)),
                optimizer=config.get('optimizer','auto'),lr0=float(config.get('lr0',.01)),cos_lr=bool(config.get('cos_lr',False)),
                cache=bool(config.get('cache',False)),seed=int(config.get('seed',0)),plots=True,verbose=True)
        best=run_dir/'weights/best.pt'
        status('finished',best=str(best) if best.is_file() else None)
        print('HOÀN TẤT. Mô hình tốt nhất: '+str(best),flush=True)
    except BaseException as error:
        traceback.print_exc()
        status('failed',error=f'{type(error).__name__}: {error}')
        raise SystemExit(1)


if __name__=='__main__': main()
