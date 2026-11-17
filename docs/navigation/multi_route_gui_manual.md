# Multi-route GUI — Factory V2

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Controller locomotion
```

Không tự chạy AUTO. Space chỉ Follow camera, X dừng/tạm dừng AUTO, R reset;
WASD/arrow/QE manual giữ nguyên. Camera đầu theo cấu hình hiện có, không camera AI.

1. Mở **G1 PATROL NAVIGATION → ROUTE SELECTION**.
2. Chọn Route 1 Blue/Center hoặc Route 2 Brown/Right. Đã bỏ Alternate và lựa chọn tuyến đầu.
3. START tự áp dụng tuyến đang chọn; PREVIEW/VALIDATE cũng áp dụng lựa chọn đó.
   Đổi tuyến khi đang chạy bị từ chối: STOP, đợi đứng chậm và trở về ENTRY trước START.
4. **PREVIEW ROUTE / VALIDATE ROUTE** kiểm tra mọi mandatory edge. Lỗi ghi rõ đoạn
   `INVALID_ROUTE_SEGMENT`; không có đường thẳng dự phòng.
5. **Top View**: đỏ chung + xanh hoặc nâu theo tuyến đã chọn, footprint và target.
6. **START PATROL** khi đang ở ENTRY; tự Play nếu mô phỏng đang Pause, không cần bấm Play riêng.
   Mặc định mới: delay 0 s, Repeat off,
   Max Cycles=1, interval 120 s theo Simulation Time. Lịch A/B của V1 không đổi.
7. Để lặp chính tuyến đang chọn, bật Repeat và Max Cycles ≥2. Chỉ tăng cycle khi về EXIT và đứng
   ổn định. 120 giây simulation có thể lâu hơn 120 giây thực; panel có clock/wait.
8. PAUSE/RESUME/STOP như cũ. FAULT cần acknowledge, không tự chạy lại. Vật cản làm
   chờ/replan trong corridor hiện tại, không tự chuyển sang nhánh khác.
9. **RETURN TO ENTRY** sau STOP/PAUSE và đứng chậm: hoàn tất phần còn lại của tuyến
   đã xác minh, không shortcut, không tính thêm cycle. Chưa có active route thì dùng Manual.
10. Save/Load dùng `configs/patrol_route_user.json`, khác A/B. Load cần STOP+Manual;
    identity/hash và cả hai tuyến được kiểm tra. Reset route configuration khôi phục
    các file mặc định rồi cần validate lại.

Tốc độ lệnh dùng min(Max Navigation Speed, route speed limit, profile ceiling), mặc định
0.30 m/s; user override mở trần 1.0 m/s **chưa kiểm chứng hoàn tất hai tuyến**.
Trong Schedule / limits có nút 0.22 / 0.30 / 0.50 / 0.80 / 1.0 m/s (lọc theo trần).
Chọn rồi Apply khi trạng thái cho phép; START áp dụng mức đang chọn, không tự chạy.
Look-ahead đi qua waypoint thẳng; cap giảm dần trong 2.0–1.2 m trước góc/đích,
đạt 0.22 m/s tại 1.2 m, đồng thời bị giới hạn thêm bởi envelope prediction + phanh.
Ngoại lệ gần góc: bước hiệu chỉnh 0.10–0.12 m/s có thể được dùng để vượt deadband gait,
chỉ sau kiểm tra đầy đủ vùng quét/phanh. Nếu đã trong tolerance nhưng không thể bước
căn giữa an toàn, giữ vị trí/quay; không bỏ qua điều kiện vị trí và settled.
Đoạn thẳng có phản hồi căn giữa liên tục và giảm tốc tiến khi lệch tâm.
Yaw cap 0.30 rad/s. ETA=length/speed chỉ ước tính, chưa cộng quay/chờ/dừng.
`configs/navigation_speed_v2.json` giữ default, trần user override và evidence audit,
gắn với checkpoint và Factory V2. `unverified_user_override=true` chỉ bỏ kiểm tra
audit khớp tốc độ trần; không biến audit 0.30 thành kết quả 1.0 m/s.
Xem [cập nhật/kiểm tra logic](speed_override_update.md), [kết quả đo trước đó](speed_test_report.md).
Load config cũ có thể khôi phục mức
0.22; chọn lại preset và Apply nếu muốn tăng.

Chỉnh JSON khi dừng; giữ đúng mandatory order và world coordinates. Không dùng
waypoint V1. Nếu USD đổi, tạo lại physics/map/hash trước. Hành lang 1.56–1.63 m
còn hẹp: planner VALID không bảo đảm PhysX hoàn tất. EXIT là marker phía trong
cửa; chưa có sàn ngoài để đi hẳn ra ngoài. Người là SIM GROUND TRUTH, không YOLO/RGB-D.

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\check_route_geometry.py --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_multi_route.py --mode route_2_brown --cycles 2 --no-ros-env
```

Xem `multi_route_test_report.md` trước khi coi hai tuyến đã qua kiểm thử vật lý.
