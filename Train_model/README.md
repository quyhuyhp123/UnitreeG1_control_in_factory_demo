# Train_model — Labelme → Dataset → Training

Mở D:\unitree_g1\Train_model\open_app.bat. Nếu cửa sổ bản cũ còn mở, đóng rồi mở lại để nạp giao diện mới.

Nhãn được làm bằng Labelme bên ngoài. Công cụ này không có chức năng vẽ/sửa box, gán ảnh nền thủ công hay tạo nhãn từ manifest; chỉ hiển thị box đọc-only để kiểm tra dữ liệu training. Không tự cài Labelme vào môi trường Isaac hoặc inference.

## Luồng dùng

1. Chốt class, dùng rectangle trong Labelme. Lưu JSON và giữ ảnh imagePath đi kèm; imageData nhúng cũng được hỗ trợ nếu thiếu file ảnh. Polygon/line bị từ chối, không tự chuyển thành box.
2. Trong giao diện chọn Class để khai báo đúng các tên class Labelme. Khi đã import nhãn, mapping bị khóa để tránh đổi class ID. Không đổi class một cách tùy tiện sau khi đã chuẩn bị dataset.
3. Import Labelme JSON hoặc Import folder Labelme. Mỗi thư mục nên là một phiên quay; nhập group tương ứng. Các frame cùng video/phiên phải dùng chung group. Import folder chỉ đọc JSON ở cấp hiện tại, không đệ quy các phiên khác.
4. Ảnh thiếu JSON không được coi là background. JSON shapes=[] chỉ nhận khi bạn xác nhận chúng là ảnh nền đã gán nhãn đầy đủ. JSON kích thước sai, ảnh thiếu, class lạ, box ngoài ảnh/NaN/diện tích bằng 0 bị từ chối.
5. Preview chỉ đọc. Muốn sửa: mở Labelme, sửa JSON gốc rồi import lại cùng group. JSON nguồn giữ nguyên; labelme_sources lưu bản sao theo timestamp cùng hash để truy vết. Ảnh trùng group khác bị chặn; JSON khác cho cùng ảnh bị chặn để tránh ghi đè nhãn mâu thuẫn. Dữ liệu cũ được giữ nguyên, không tự xóa hoặc chuyển đổi nhãn đã có.
6. Audit dataset. Cần ít nhất ba group độc lập và có nhãn cho các class. Duplicate hash không phát hiện được mọi ảnh gần trùng; cần tự kiểm tra tương đồng giữa các phiên.
7. Prepare training: chọn imgsz, epochs, batch dự kiến. Xuất exports/<timestamp>/images, labels YOLO, data.yaml, classes.json, split_manifest.json, custom_yolo11n.yaml và training_plan.json. Split theo group với seed42, ~70/15/15; plan cảnh báo class thiếu ở từng split. Đây chỉ là snapshot chuẩn bị, không bắt đầu học.
8. Dừng và quyết định khi nào training. Nút TRAIN vẫn khóa theo yêu cầu trước. Kiến trúc nano khởi tạo ngẫu nhiên, pretrained=false, không nạp .pt weights. Môi trường hiện có CPU-only; GPU training RTX5070 và export/evaluation sau khi có model chưa được thực hiện. Không tạo ONNX/checkpoint giả.

## Cấu trúc

- dataset.json: metadata, mapping class, group, nguồn nhãn và box đã import.
- images/: ảnh PNG chuẩn hóa; labelme_sources/: bản sao JSON nguồn.
- exports/: dataset YOLO và kế hoạch train có version theo timestamp.
- backups/: dữ liệu cũ trước khi ghi, code/tài liệu giao diện trước khi chuyển workflow.
- plans/readiness.json: trạng thái chuẩn bị hiện tại, không phải kết quả training.

Không có thao tác Labelme nào sửa Locomotion/Navigation/camera. Không thay detector đang dùng; không commit/push. Labelme là phần mềm bên ngoài, chưa cài trong tác vụ này.

Kiểm thử:

```powershell
& D:\unitree_g1\.venv\yolo\Scripts\python.exe -X utf8 D:\unitree_g1\Train_model\test_preparation.py
```

Tests dùng ảnh fixture tạm, kiểm tra import Labelme, giữ nguyên JSON, chuẩn hóa hai góc rectangle, từ chối polygon/empty không xác nhận, chống trùng khác group, YOLO export/split, plan không training và GUI preview. Dataset thật không được tự bổ sung bằng các fixture.
