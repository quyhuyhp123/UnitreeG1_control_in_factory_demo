# Dừng robot và giảm margin Factory V2 — 11/11/2026

Nguyên nhân phiên GUI 20261111_215017 dừng: FAULT `No progress on mandatory segment`.
Tại ENTRY_INNER robot cách waypoint khoảng 16 cm nhưng cap phanh còn 0.041 m/s,
dưới ngưỡng gait 0.1 của policy. Đã sửa tiếp cận góc bằng bước hiệu chỉnh 0.10–0.12 m/s
chỉ khi full footprint/quãng phanh cho phép; giữ điều kiện vị trí/yaw/settled thực.
Nếu đã trong tolerance nhưng bước căn giữa thêm bị chặn, giữ/quay thay vì bắt buộc bước đó.
Thêm căn giữa liên tục, giảm tốc khi lệch và cap riêng theo clearance của đoạn hẹp.

Đã có hai replay PhysX độc lập đạt trước khi giảm margin:
- `corner_stall_fix_results.json`: tái hiện gần cửa, chuyển segment 0 → 1, cycle không tăng.
- `bottom_corner_fix_results.json`: tái hiện góc dưới, chuyển segment 7 → 8, cycle không tăng.
Đây không phải bằng chứng hoàn thành toàn tuyến.

Các lượt Route 1 → Route 2 ở cap chọn 0.8 đã tìm ra thêm vấn đề tracking/clearance;
đã giữ các attempt 1..4 trong docs/navigation. Có lượt Route 1 hoàn thành, nhưng Route 2
bị footprint FAULT ở hành lang phải. Đã thêm cap đoạn rất hẹp 0.22 m/s và chạy lại.
Lượt cuối bị dừng theo yêu cầu người dùng trước khi đổi margin, không coi là PASS hai tuyến.
Xem `corner_fix_alternate_080_results.json` để biết trạng thái và progress cuối thực tế.

Theo yêu cầu mới nhất: giảm margin dự phòng, không chạy lại:
- `configs/navigation_map_v2.json`: margin_m 0.10 → 0.05 m.
- Body radius 0.45, step sway 0.08, pose error 0.04 giữ nguyên; tổng footprint 0.67 → 0.62 m.
- `configs/factory_v2.json`: spawn clearance radius 0.62 m để đồng bộ map.
- `blender/build_factory_v2_physics.py`: lần dựng lại dùng margin 0.05 và cộng các thành phần
  để tính spawn radius; chưa chạy builder, không sửa geometry/USD.
- Không giảm human extra 0.30, stopping extra 0.08, thời gian phản ứng hay giới hạn tilt.

Bộ logic trước thay margin: 83 tests, 81 đạt, 2 SONIC skips (`logs/corner_stall_logic.log`).
**Không chạy lại test hoặc mô phỏng sau thay margin theo yêu cầu người dùng.**
Chưa xác nhận toàn bộ hai tuyến với cấu hình margin 0.05. Không thay audit 0.30 m/s,
không chứng nhận 1.0 m/s, không commit/push, không điều khiển robot thật.

Khởi động lại Isaac Sim để đọc map mới:
`D:\unitree_g1\run_sim.bat -Factory v2 -Controller locomotion`
Muốn chạy cả hai tuyến: Alternate Routes, Repeat bật, Max Cycles >= 2; robot chờ theo
Repeat Interval giữa hai vòng. Default speed vẫn 0.30; trần 1.0 là user override chưa kiểm chứng.
