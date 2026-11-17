# Training_model — Huấn luyện lại YOLO với dữ liệu LabelMe

Giao diện Python (Tkinter) để train lại YOLO11 phát hiện vật thể theo nhãn riêng của bạn.
Thư mục này độc lập với mô phỏng: không khởi động Isaac Sim, không điều khiển robot,
không sửa cấu hình nào của dự án.

## Mở giao diện

```bash
cd Training_model
./setup_env.sh     # chỉ cần chạy MỘT lần (đã chạy sẵn trên máy này)
./run_ui.sh        # mở cửa sổ; chạy trong Terminal của desktop Ubuntu / remote desktop
```

## Chuẩn bị dữ liệu

Gán nhãn bằng LabelMe (hình chữ nhật là tốt nhất), rồi chia ảnh vào 2 thư mục:

```
du_lieu_cua_toi/
├── train/   anh_001.jpg  anh_001.json  anh_002.jpg  anh_002.json ...
└── val/     anh_101.jpg  anh_101.json ...
```

- File `.json` của LabelMe nằm cạnh ảnh và cùng tên với ảnh.
- Nên để khoảng 10–20% số ảnh ở `val`.
- Ảnh không có `.json` được coi là ảnh nền (không có vật thể).
- Polygon/circle vẫn dùng được: chương trình lấy khung bao quanh.
- Dữ liệu gốc không bị sửa; bản YOLO được tạo trong `runs/<tên>/dataset` bằng liên kết tới ảnh gốc.

## Các tab

| Tab | Việc làm |
|---|---|
| 1. Dữ liệu | Chọn thư mục chứa `train/` và `val/`, bấm **Kiểm tra dữ liệu**: đếm ảnh, khung, liệt kê nhãn và cảnh báo |
| 2. Huấn luyện | Nhập tên nhãn (mỗi dòng một nhãn), epoch, batch, imgsz, mô hình gốc, GPU → **Bắt đầu train** / **Dừng**; xem tiến độ và nhật ký |
| 3. Biểu đồ loss | Loss train, loss val, precision/recall/mAP, learning rate theo epoch; tự cập nhật khi đang train |
| 4. Kiểm tra mô hình | Mở ảnh hoặc thư mục ảnh, xem khung phát hiện, đổi ngưỡng tin cậy, lưu ảnh kết quả |
| 5. Kết quả & Xuất | Danh sách các lần train, ảnh đánh giá (ma trận nhầm lẫn, đường PR...), **Train tiếp**, **Xuất ONNX cho robot**, xoá |

## Lưu ý về tham số

- **Tên nhãn** phải giống hệt trong LabelMe (kể cả hoa/thường). Thứ tự dòng là id của lớp (dòng đầu = 0).
  Nhãn có trong dữ liệu nhưng không nằm trong danh sách sẽ bị bỏ qua.
- **imgsz**: camera của robot là 640×360, nên 640 là lựa chọn hợp lý.
- **batch**: 16 là mặc định; `-1` để Ultralytics tự chọn theo VRAM.
- **Mô hình gốc**: `yolo11n.pt` có sẵn. Chọn `s/m/l` thì Ultralytics tự tải về (cần mạng).
- **Patience**: dừng sớm nếu mAP không cải thiện sau N epoch.

## Kết quả nằm ở đâu

```
runs/<tên lần train>/
├── weights/best.pt, last.pt      mô hình tốt nhất / gần nhất
├── results.csv, results.png      số liệu từng epoch
├── train.log, config.json        nhật ký và tham số đã dùng
└── dataset/                      dữ liệu YOLO đã chuyển từ LabelMe
```

## Dùng mô hình mới cho robot mô phỏng

Ở tab 5 bấm **Xuất ONNX cho robot** (chọn imgsz 640). Sau đó tự tay:

1. Chép `best_640.onnx` và `class_names.json` vào `factory_v2/checkpoints/yolo/`.
2. Sửa `model`, `model_sha256`, `class_names`, `imgsz` trong `factory_v2/configs/yolo.json`
   (giá trị sha256 hiện trong hộp thoại và trong file `best_640_export.json`).
3. Perception của robot coi **class id 0 là người** (`configs/local_costmap.json: human_class_id`).
   Nếu mô hình của bạn có nhãn người, hãy đặt nó ở dòng đầu tiên; nếu không, sửa `human_class_id`.

Giao diện không tự sửa cấu hình của dự án mô phỏng.

## Các file

| File | Vai trò |
|---|---|
| `training_ui.py` | Giao diện (Tkinter, chỉ dùng thư viện chuẩn + Pillow) |
| `run_ui.sh` | Mở giao diện |
| `setup_env.sh` | Cài Ultralytics vào `.deps` (bản `--no-deps`) và tải `yolo11n.pt` |
| `ml_python.sh` | Chạy Python với PyTorch/CUDA của Isaac Sim + `.deps` (không mở Isaac Sim) |
| `labelme_to_yolo.py` | Kiểm tra và chuyển nhãn LabelMe → YOLO |
| `train_worker.py`, `predict_worker.py`, `export_worker.py` | Tiến trình train / chạy thử / xuất ONNX |

Môi trường: giao diện chạy bằng `python3` của hệ thống; train dùng PyTorch 2.11 + CUDA có sẵn trong
Isaac Sim và Ultralytics cài riêng trong `.deps`. Không có gì được cài vào Isaac Sim; xoá `.deps` là gỡ sạch.

Trọng số YOLO11 và Ultralytics theo giấy phép AGPL-3.0.
