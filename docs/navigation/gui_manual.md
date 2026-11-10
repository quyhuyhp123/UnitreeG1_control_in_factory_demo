# Sử dụng G1 PATROL NAVIGATION

Chạy bằng PowerShell:

```powershell
Set-Location D:\unitree_g1
.\run_sim.bat -Controller locomotion
```

Mặc định robot vẫn ở MANUAL, gần cửa `[0, -4]`, nhìn theo +Y. Điểm A lấy từ feedback thực; B chưa được đặt. Không tự bắt đầu nhiệm vụ và không tự load file cấu hình cũ.

1. Trong **G1 PATROL NAVIGATION**, nhập tọa độ A/B hoặc dùng **Set A/B from Robot**. Heading nhập bằng độ; checkbox cạnh heading bật yêu cầu hướng cuối. Giá trị trong file JSON dùng radian.
2. Bộ điểm đã thử: A gần `(0, -4)`, heading `90°`; B `(0, -2.5)`, có thể chọn heading `90°`. Đây là tọa độ thử đã qua kiểm tra trên USD hiện tại, không được tự ghi làm B mặc định. Robot quay mặt theo đường đi rồi mới đi tiến ở cả hai chiều A→B và B→A. Heading tùy chọn chỉ dùng khi đã đến điểm cuối; nếu bật heading `90°` tại A, robot sẽ quay về hướng đó sau khi đi tới A. Tắt checkbox heading nếu muốn giữ hướng vừa đi về.

   Khi đã vào vùng đích, bộ điều khiển có thể phát hiệu chỉnh vị trí nhỏ tối đa 0.08 m/s để bù độ trôi khi quay và giữ thăng bằng. Đây là bước giữ điểm cuối, không phải đi lùi suốt chặng B→A. Thay đổi planner cần đóng và khởi động lại chương trình để có hiệu lực.
3. Nhấn **Apply A/B + Validate Route**. B chỉ được xác nhận sau thao tác này hoặc Set B from Robot. Nếu footprint hoặc đường hai chiều không hợp lệ, UI hiện `NO_VALID_PATH`. STOP nhiệm vụ trước khi sửa điểm.
4. Nhấn **Top View** để xem làn, footprint và path. Camera này dùng viewport chính, cắt mái bằng clipping; không sửa hoặc ẩn geometry nguồn. Free/Follow/Overview và Head Camera vẫn dùng được. Nút Show/Hide Path bật/tắt overlay; khi ở Manual có thể tạo lại preview từ A/B đã xác nhận.
5. Mở **Schedule / limits**: Start Delay, Interval, Dwell, Repeat, Max Cycles, Movement Timeout, Obstacle Wait Timeout, Clock Mode và Max Speed. Giới hạn tốc độ UI 0.12–0.35 m/s; mặc định 0.35. Đây là vận tốc mục tiêu trong thời gian mô phỏng; robot vẫn giảm tốc khi gần đích/quay và chịu kiểm tra khoảng phanh.
6. Nhấn **START PATROL** khi robot đã đứng tại A. Mặc định chờ 5 s, đến B chờ 10 s, về A rồi nghỉ 5 s trước vòng tiếp. Bật Repeat và đặt Max Cycles = 10 để chạy mười vòng A→B→A. Giá trị cũ 120 s vẫn được giữ nếu chủ động Load cấu hình cũ. Trong CYCLE_COMPLETE, sửa thời gian chờ/tốc độ rồi nhấn **Apply schedule / speed (while waiting)** để cập nhật mà không mất số vòng đã hoàn tất. Khi dùng Simulation Time, đồng hồ chờ chạy theo simulation, có thể chậm hơn nhiều so với thời gian thực; panel có ước tính wall s. Muốn chờ theo đồng hồ thật, STOP rồi chọn Wall Time.

**STOP** hủy nhiệm vụ và đưa mục tiêu về 0, vẫn giữ AUTO để WASD không vô tình di chuyển robot. Muốn điều khiển tay: chọn MANUAL, nhấn **Apply Mode / explicit Manual takeover**, rồi nhấn phím mới. Nhập số trong panel không được coi là focus điều khiển tay.

**SPACE** chỉ chuyển camera sang Follow, không xóa phím đang giữ, không đổi lệnh vận tốc và không PAUSE AUTO. **X** đưa lệnh tay về 0 hoặc PAUSE AUTO; dùng **RESUME Auto** để tiếp tục. Pause timeline vẫn yêu cầu Resume riêng. Mất focus vẫn xóa phím manual bị giữ.

**FAULT**: đọc lý do, loại bỏ nguyên nhân, nhấn **Acknowledge FAULT**, rồi chủ động START lại. STOP hoặc chọn mode khác không bỏ qua latch. **RESET ROBOT** dùng cơ chế reset an toàn hiện có và hủy nhiệm vụ; không dùng để vượt vật cản. **RETURN TO A** chỉ dùng sau STOP/PAUSE và khi robot đã chậm, có path hợp lệ; chỉ quay về A, không tự khởi động thêm chu trình.

**Save config / Load config** lưu A/B, lane, forbidden/static polygons, speed, safety settings và schedule vào `configs/patrol_user.json`. File trước được sao lưu trong thư mục log của lượt chạy. Load yêu cầu MANUAL và kiểm tra route trước khi áp dụng. Làn polygon sửa trong JSON; chưa có công cụ click chọn điểm hay vẽ polygon bằng chuột.

Panel luôn ghi **SIM GROUND TRUTH**. Số người/near-human lấy từ collider và metadata scene, không phải camera AI. SONIC bị vô hiệu hóa vì chưa có backend PhysX đã kiểm chứng. Không bật LiDAR/depth hay kết nối robot thật.

Smoke test callback GUI đã chạy. Helper desktop không kết nối được trong phiên triển khai, nên người dùng vẫn cần thử native mouse/keyboard: sửa số không làm robot đi, Apply/START/STOP, WASD/arrow manual, SPACE Auto, Pause/Resume, R, camera và Save/Load.

Chạy lại các bài kiểm thử bằng đúng Python Isaac Sim đang cài:

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_patrol.py --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_patrol_obstacles.py --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_patrol_gui.py --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\summarize_patrol_tests.py --no-ros-env
```

`test_patrol.py --calibrate-only` chỉ đo phanh, lưu báo cáo riêng; không thay thế báo cáo chu trình đầy đủ. Các script test dùng Carb logical input và tự đóng Isaac khi xong. Muốn thử phím thật, dùng launcher thông thường, không dùng self-test.
