# Multi-Route Patrol — kết quả kiểm thử thực tế

Ghi chú 11/11/2026: đây là baseline 0.22 m/s. Cấu hình tốc độ mới và thử nghiệm
được ghi riêng trong [speed_test_report.md](speed_test_report.md).

Ngày 10/11/2026, Windows 11, RTX 5070 12 GB, Isaac Sim 6.1.0-rc.26, Python 3.12.13.
**PASS trong phạm vi kiểm thử dưới đây.** Dữ liệu tổng hợp: [multi_route_acceptance.json](multi_route_acceptance.json).

## Khởi chạy

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Controller locomotion
```

Trong **G1 PATROL NAVIGATION → ROUTE SELECTION**, chọn Route 1, Route 2 hoặc Alternate;
Apply → Preview/Validate → START PATROL. Chạy luân phiên: Alternate, bật Repeat,
Max Cycles ≥ 2. Mặc định delay 5 s, interval 120 s, Repeat tắt, một vòng, Simulation Time.
Space chỉ follow camera; X pause Auto; R reset. Manual WASD/mũi tên/QE giữ nguyên,
không cộng vào Auto. Chi tiết: [hướng dẫn GUI](multi_route_gui_manual.md).

## Kết quả từng bài

| Bài | Kết quả và bằng chứng |
|---|---|
| 1. Geometry | Hai tuyến VALID_GEOMETRY; A* từng cạnh giữ corridor và footprint 0,67 m. `multi_route_geometry.json` |
| 2. Route 1 | PhysX đi thực đủ 10 đoạn, qua trái → trên → giữa → dưới → cửa; hoàn thành simulation 374,44 s |
| 3. Route 2 | PhysX đi thực đủ 12 đoạn, qua trái → trên → phải → dưới → cửa; hoàn thành 819,24 s |
| 4. Turning | Trace có stop/turn-in-place; 69/68 mẫu TURNING với yaw command >0,1 rad/s và linear command <0,01 m/s cho Route 1/2. Các góc được vượt qua thực tế |
| 5. No shortcut | Đối chiếu trace độc lập: 10/10 và 12/12 mandatory edges đúng thứ tự. 4.086 mẫu toàn run không vi phạm footprint/map |
| 6. Alternate | Route 1 → Route 2 tự động, completed_cycles=2, kết thúc STOPPED, runtime fault=null |
| 7. Repeat interval | Thử vật lý rút ngắn còn 2 s: CYCLE_COMPLETE 374,44 → VALIDATING_ROUTE 376,44. Giá trị 120 s production kiểm bằng unit test, chưa chờ đủ 120 s trong phiên vật lý |
| 8. Obstacle | Đặt Cube collider trước robot đang đi: WAITING_FOR_CLEAR_PATH, command zero, `Obstacle in braking envelope`; gỡ ra tiếp tục cùng Route 1 |
| 9. Blocked route | Chặn kín giữa: Validate/Start Route 1 bị từ chối, cycle=0, không chuyển Route 2. Route 2 vẫn valid; gỡ blocker Route 1 valid lại |
| 10. Manual | W/S/A/D, UP/DOWN/LEFT/RIGHT, Q/E có command đúng dấu và chuyển động/yaw đo được >0,02 m hoặc rad; không fault. Space không dừng; X pause; Resume rõ ràng |
| 11. V1 regression | Một vòng A→B→A đạt; quay tại chỗ rồi đi tiến về A, không đi lùi chặng về; kết thúc STOPPED |
| 12. Reset/Fault | Tiêm software runtime fault: Auto FAULT, command zero, STOP không gỡ latch. R lúc pause về (-8,85,-13,75), clear policy/history, Manual, không tự restart. Không gọi đây là thử ngã thật |
| 13. Performance | Số liệu bên dưới; giữ log/telemetry từng run |

Route 1/2 kiểm trong một phiên Alternate liên tục, không phải hai phiên standalone riêng.
Lựa chọn route riêng và Alternate đã kiểm qua GUI callbacks. GUI được mở thật,
input qua Carb logical keys; chưa tự động click/nhấn native Windows.
Ảnh viewport đã mở kiểm tra bố cục: `logs/keyboard_runs/20261110_223027/multi_route_top.png`;
ảnh không bao gồm panel omni.ui hoặc overlay UI.

## Hiệu năng

Hai vòng headless, một G1, một Factory V2, policy hiện có, không training;
tắt head preview riêng trong phép đo. Phiên GUI riêng bật head preview.

| Chỉ số | Kết quả |
|---|---:|
| Geometry length Route 1 / 2 | 65,896 / 79,796 m |
| Route command speed / yaw tối đa | 0,22 m/s / 0,30 rad/s |
| Loop FPS trung bình tại GPU samples | 40,85 |
| Physics steps / wall second trung bình | 160,16 |
| Physics / policy frequency cấu hình | 200 / 50 Hz simulation |
| VRAM toàn GPU đỉnh | 4.378 MiB / 12.227 MiB |
| Planner latency cạnh cuối | 8,41 ms |
| Path deviation cao nhất Route 1 / 2 | 0,1384 / 0,1386 m |

FPS là vòng lặp, không phải GPU-present; VRAM là tổng toàn GPU, không riêng process.
Simulation có thể chậm hơn thời gian thực. Bốn log final physics/GUI/reset/V1 không có
`[Error]`, `[Fatal]` hoặc Python traceback; vẫn có warning Kit/Fabric/DLSS.

## File triển khai và bằng chứng

- Graph/controller: `navigation/route_graph.py`, `route_manager.py`, `route_scheduler.py`, `multi_route_controller.py`.
- Tích hợp: `controllers/factory_g1_runtime.py`, `navigation/patrol_ui.py`, `navigation/command_adapter.py`, `navigation/patrol_controller.py` (FAULT telemetry zero).
- Config: `configs/patrol_routes.json`, `configs/patrol_route_settings.json`, `configs/factory_v2.json`; builder V2 giữ cờ multi-route khi dựng lại.
- Docs: `multi_route_architecture.md` có bảng world coordinates/hành lang, `multi_route_gui_manual.md`, README.
- Scripts/tests: `tests/test_multi_route.py`, `scripts/check_route_geometry.py`, `test_multi_route.py`, `test_multi_route_ui.py`, `test_multi_route_reset.py`, `test_route_corner.py`, `summarize_multi_route.py`.
- Final JSON: `multi_route_physics_results.json`, `multi_route_ui_results.json`, `multi_route_reset_results.json`, `multi_route_v1_regression.json`, `multi_route_acceptance.json`.
- Logs: `logs/multi_route_physics_final.log`, `multi_route_ui.log`, `multi_route_reset.log`, `multi_route_v1_regression.log`, `multi_route_unit_final.log`.
- Trace: `logs/patrol_runs/20261110_221305/multi_route_trace.json`; telemetry/performance: `logs/keyboard_runs/20261110_221305/`.

Unit suite cuối: **56 bài, 54 đạt, 2 SONIC skips**; compileall và git diff --check đạt.
SHA256 so baseline xác nhận không đổi source Factory V2 USD, factory V1, robot USD,
policy ONNX, adapter locomotion và config V1. Scene V1 generated đã backup/restore
sau regression. Không commit/push ở bước này.

## Giới hạn và lịch sử sửa lỗi

EXIT là marker phía trong cửa; chưa có sàn/allowed area ngoài để đi hẳn ra sân.
Hành lang hẹp 1,56–1,63 m; hai vòng đạt không bảo đảm mọi speed/layout/nhiễu đều đạt.
Perception SIM GROUND TRUTH, chưa thử người di chuyển hoặc sensor. Footprint audit
lấy mẫu, không chứng minh swept-mesh liên tục. RETURN TO ENTRY giữ tuyến còn lại;
chưa kiểm hành trình Return riêng từ giữa xưởng. Native GUI clicks, ngã vật lý thật
và chờ đủ 120 s production còn là checklist thủ công.

Các lần phát triển trước không hoàn thành, giữ thành `multi_route_dev_attempt_*`:
source-cell quá bảo thủ, target xa, ramp XY đổi hướng, corrective command dưới gait
threshold. Đã sửa bằng exact safe-pose connector, waypoint dày/recovery anchor,
vector ramp và corrective step có giới hạn; không giảm footprint hoặc đổi policy/geometry.
Lần reset/blocker đầu (`multi_route_reset_attempt_1.json`) lỗi vì assert quá sớm và
telemetry FAULT còn command cũ. Final sau sửa đã đạt; không trình bày lần lỗi là thành công.
