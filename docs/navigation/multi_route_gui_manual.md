# Multi-route GUI — Factory V2

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Controller locomotion
```

Không tự chạy AUTO. Space chỉ Follow camera, X dừng/tạm dừng AUTO, R reset;
WASD/arrow/QE manual giữ nguyên. Camera đầu theo cấu hình hiện có, không camera AI.

1. Mở **G1 PATROL NAVIGATION → ROUTE SELECTION**.
2. Chọn Route 1 Blue/Center, Route 2 Brown/Right hoặc Alternate; chọn tuyến đầu cho Alternate.
3. STOP và đợi đứng chậm, rồi **Apply route**. Đổi tuyến khi đang chạy bị từ chối.
4. **PREVIEW ROUTE / VALIDATE ROUTE** kiểm tra mọi mandatory edge. Lỗi ghi rõ đoạn
   `INVALID_ROUTE_SEGMENT`; không có đường thẳng dự phòng.
5. **Top View**: đỏ chung + xanh hoặc nâu, footprint và target. Alternate hiển thị
   route hiện tại/sắp chạy và ID route cycle kế tiếp.
6. **START PATROL** khi đang ở ENTRY. Mặc định mới: delay 5 s, Repeat off,
   Max Cycles=1, interval 120 s theo Simulation Time. Lịch A/B của V1 không đổi.
7. Để luân phiên, bật Repeat và Max Cycles ≥2. Chỉ tăng cycle khi về EXIT và đứng
   ổn định. 120 giây simulation có thể lâu hơn 120 giây thực; panel có clock/wait.
8. PAUSE/RESUME/STOP như cũ. FAULT cần acknowledge, không tự chạy lại. Vật cản làm
   chờ/replan trong corridor hiện tại, không tự chuyển sang nhánh khác.
9. **RETURN TO ENTRY** sau STOP/PAUSE và đứng chậm: hoàn tất phần còn lại của tuyến
   đã xác minh, không shortcut, không tính thêm cycle. Chưa có active route thì dùng Manual.
10. Save/Load dùng `configs/patrol_route_user.json`, khác A/B. Load cần STOP+Manual;
    identity/hash và cả hai tuyến được kiểm tra. Reset route configuration khôi phục
    các file mặc định rồi cần validate lại.

Tốc độ lệnh dùng min(Max Navigation Speed, route speed limit), mặc định/giới hạn
đã kiểm thử mới là 0.30 m/s. Trong Schedule / limits có nút 0.22 / 0.25 / 0.30 m/s;
chọn rồi Apply khi STOP hoặc trước START. Trong 1.2 m trước góc/đích, cap giảm
về 0.22 m/s rồi giảm thêm để dừng/quay; ra đoạn thẳng khôi phục mức đã chọn.
Yaw cap 0.30 rad/s. ETA=length/speed chỉ ước tính, chưa cộng quay/chờ/dừng.
`configs/navigation_speed_v2.json` giữ mức đã xác minh và evidence audit, gắn với
checkpoint và Factory V2. Không nâng file này chỉ dựa vào giới hạn policy 1 m/s.
Xem [kết quả thử tốc độ](speed_test_report.md). Load config cũ có thể khôi phục mức
0.22; chọn lại preset và Apply nếu muốn tăng.

Chỉnh JSON khi dừng; giữ đúng mandatory order và world coordinates. Không dùng
waypoint V1. Nếu USD đổi, tạo lại physics/map/hash trước. Hành lang 1.56–1.63 m
còn hẹp: planner VALID không bảo đảm PhysX hoàn tất. EXIT là marker phía trong
cửa; chưa có sàn ngoài để đi hẳn ra ngoài. Người là SIM GROUND TRUTH, không YOLO/RGB-D.

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\check_route_geometry.py --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_multi_route.py --mode alternate --cycles 2 --no-ros-env
```

Xem `multi_route_test_report.md` trước khi coi hai tuyến đã qua kiểm thử vật lý.
