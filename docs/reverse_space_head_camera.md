# Lùi, SPACE và head camera — 2026-11-05

Đã đọc log native logs/keyboard_runs/20261105_232247/events.jsonl: khi nhận DOWN, pressed còn [DOWN,W], desired [0,0,0]. Đây là bằng chứng lệnh đối nghịch triệt tiêu trong state, không chứng minh người dùng đang thật sự giữ W. Bộ input đã thêm đối chiếu release-only với GetAsyncKeyState trên Windows: phím đã thả nhưng mất KEY_RELEASE sẽ được loại bỏ. Polling không tự tạo PRESS hoặc di chuyển robot. Nếu thật sự giữ đồng thời phím tiến và lùi thì vẫn phải triệt tiêu, theo quy tắc đã đặt.

SPACE giữ chức năng zero target velocity và thêm sự kiện chuyển camera chính sang Follow, theo vị trí và heading robot. Nút Stop [SPACE] trên HUD cũng đi qua cùng xử lý. Chọn Camera: Free để tự di chuyển góc nhìn trở lại. Không dùng world pose để tạo chuyển động robot.

Đã thêm USD HeadCamera dưới torso_link (đầu G1 cố định tương đối với torso), offset local [0.1,0,0.52] m, nhìn về +X local và chúc xuống 10°, FOV ngang 80°. Khung G1 Head Camera | LIVE ở góc phải trên, texture 320×180. Camera chính 960×640, TAA thay DLSS để tránh upscale không cần thiết ở preview nhỏ. Không có depth/LiDAR; ảnh preview không được đưa vào observation locomotion.

Khung preview dùng ViewportWindow được đăng ký với Kit/RenderingManager. Khi reset robot, giải phóng preview trước rồi tạo lại sau reset; rebind camera relationship của render product chính. Cách này đã xử lý mất render product, preview đen và camera chính nhìn sai sau reset. Các lần thử lỗi được giữ trong logs/reverse_head_camera_initial_reset_error.log, reverse_head_camera_render_copy_crash.log, reverse_head_camera_bare_widget_test.log và reverse_head_camera_file_write_race.log. Không dùng những lần đó làm bằng chứng thành công.

## Kết quả cuối

scripts/test_reverse_head_camera.py, logs/reverse_head_camera_test.log, run logs/keyboard_runs/20261105_234140. Exit 0, không Error/Fatal/Traceback; còn warning hydratexture khi giải phóng viewport. Kết quả máy đọc: logs/reverse_head_camera_result.json.

- Cố tình giữ W trong state mà bỏ KEY_RELEASE, sau đó nhấn S: state đối nghịch ban đầu vx=0; release reconciliation bỏ W đã thả, target=-0.35 m/s và robot lùi body khoảng 0.684 m, không ngã.
- Cùng bài với DOWN: kết quả tương đương, lùi khoảng 0.684 m.
- SPACE: command [0,0,0], camera mode=follow.
- Head camera giữ local transform, world transform đổi theo robot khi quay; texture 320×180 và ảnh đã ghi hoàn tất.
- Đã kiểm tra ảnh và cửa sổ GUI trực tiếp: camera chính thấy G1, preview góc phải trên hiển thị nhà xưởng đúng góc nhìn đầu. head_camera.png và reverse_main.png nằm trong run_directory.
- Unit checks xác nhận hai phím đối nghịch thực sự vẫn triệt tiêu và trường hợp W mất release không chặn lùi sau khi reconciliation.

Input trong bài physics là Carb logical keyboard, trạng thái physical-down được mô phỏng để tái tạo lỗi mất release. Đây không phải kiểm thử giữ S trên bàn phím OS sau bản sửa; log native ban đầu đã được dùng để chẩn đoán. Cần người dùng xác nhận lại trên bàn phím thật sau khi đóng/mở chương trình. FPS loop trong test hai viewport khoảng 18–20 theo wall time, không đạt realtime policy 50 Hz; physics dt vẫn 0.005 s theo simulation time. Không tăng giới hạn lệnh hoặc thay checkpoint để che vấn đề input.

## Sử dụng

```powershell
& 'D:\unitree_g1\run_sim.bat'
```

Đóng phiên cũ trước để nạp mã mới. Thả W/↑ rồi giữ S/↓ để lùi; HUD phải chỉ hiển thị phím đang giữ và Desired vx âm. SPACE dừng lệnh và khóa camera chính theo robot; Free mở lại điều khiển chuột. Preview đầu luôn là ảnh từ camera gắn trên G1, không phải vùng nhìn dự đoán/AI.

File sửa: controllers/keyboard_controller.py, controllers/g1_keyboard_runtime.py, controllers/factory_g1_runtime.py, scripts/run_factory_g1.py, scripts/test_keyboard_contract.py và configs/factory_g1.json. File test mới scripts/test_reverse_head_camera.py. README.md cập nhật điều khiển và hai viewport.
