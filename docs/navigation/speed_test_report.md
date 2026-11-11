# Factory V2 — báo cáo thử tốc độ, 11/11/2026

Đã đặt mặc định và trần lệnh Automatic Patrol V2 thành **0.30 m/s**.
Đây là mức cao nhất đã kiểm thử đạt đầy đủ cả hai tuyến trong đợt này,
không phải khẳng định giới hạn vật lý tuyệt đối của G1.

## Cách dùng

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Controller locomotion
```

Mở **G1 PATROL NAVIGATION → Schedule / limits**. Chọn nhanh **0.22 / 0.25 / 0.30 m/s**,
rồi Apply khi STOP hoặc trước START. FloatField vẫn cho nhập liên tục trong giới hạn;
0.25 là lựa chọn trung gian, chưa có một lượt đầy đủ riêng ở đúng mức đó.
Load cấu hình cũ có thể đưa về 0.22; chọn lại preset nếu cần.

Mỗi tuyến có cap 0.30; profile mặc định cũng 0.30. Trước góc/đích 1.2 m, controller
hạ cap về 0.22 rồi giảm thêm để dừng/quay. Ra đoạn thẳng khôi phục mức người dùng chọn.
Giới hạn yaw giữ 0.30 rad/s. Space chỉ Follow camera; X pause Auto. Thay đổi này áp dụng
Multi-Route V2; không nâng tốc độ Manual hay legacy A/B V1 lên các mức thử nghiệm cao.

## Thử nghiệm sàn phẳng và nhà xưởng

Cùng robot/policy ONNX thật, PhysX 200 Hz, policy 50 Hz. Không thay checkpoint,
joint mapping, PD, footprint hoặc geometry. Các trial sàn phẳng ramp 0.4 m/s²,
giữ target khoảng 3 s và quan sát dừng 3 s; reset chỉ giữa các trial, không dùng để đi bộ.
Vận tốc trung bình bảng dưới lấy cửa sổ steady cuối, không phải tốc độ tức thời theo gait.

| Lệnh m/s | Tiến trung bình trên sàn trống m/s | Kết quả Factory V2 |
|---|---:|---|
| 0.22 | 0.227 | Baseline hai tuyến đạt ngày 10/11 |
| **0.30** | **0.303** | **Đạt Route 1 → Route 2, completed=2, STOPPED, runtime fault=null** |
| 0.35 | 0.350 | Lần 1 hoàn thành Route 1 nhưng FAULT footprint ở vòng 2; lần 2 tăng lookahead bị dừng thử vì nhiều safety stops và throughput thấp; không chứng nhận |
| 0.50 | 0.495 | FAULT footprint, chưa hoàn thành vòng |
| 0.65 | 0.627 | FAULT footprint, chưa hoàn thành vòng |
| 0.80 | 0.753 | FAULT footprint, chưa hoàn thành vòng |
| 1.00 | 0.947 | FAULT footprint, chưa hoàn thành vòng |

1 m/s đạt trial đi thẳng ngắn: nghiêng tối đa 4.56°, dịch chuyển lớn nhất sau lệnh zero
0.356 m. Điều này không chứng nhận đi lâu, quay, tránh người hoặc toàn tuyến ở tốc độ đó.
Ở 0.30 m/s, góc nghiêng trial flat tối đa 2.51°, dịch chuyển sau zero 0.094 m.
Các FAULT footprint là lỗi kiểm tra vùng an toàn, không đồng nghĩa robot bị ngã;
không giảm margin để vượt kiểm thử. Không kiểm hết mọi giá trị liên tục giữa các mức.

## Kết quả cuối ở 0.30 m/s

- Route 1 hoàn thành simulation 327.22 s; Route 2 hoàn thành 726.22 s.
- 3,621 mẫu trace; đối chiếu độc lập đúng 10 mandatory edges Route 1 và 12 edges Route 2.
- Không mẫu nào vi phạm footprint/map. Đây là audit lấy mẫu, không phải chứng minh swept-mesh liên tục.
- Max forward command đo được 0.3000 m/s; 903 mẫu command vượt 90% target.
- Mean actual speed ở các mẫu đang dịch chuyển: 0.277 m/s; tốc độ giảm khi gần góc/đích.
- Tổng 726.22 s so baseline 819.24 s: giảm khoảng **11.4% thời gian mô phỏng**.
  Tăng trần lệnh từ 0.22 lên 0.30 là +36.4%, không có nghĩa cả tuyến nhanh hơn 36.4%.
- Test repeat interval rút xuống 2 s; production 120 s không đổi.
- Headless test tắt head preview, render mỗi 5 control ticks; physics vẫn 200 Hz, policy 50 Hz.
  Mean loop rate tại GPU samples 53.8 ticks/s, peak whole-GPU 4,041 MiB, planner cạnh cuối 8.59 ms.
  Không so trực tiếp FPS này với baseline render mỗi tick; giây simulation dùng để so tuyến.
- GUI mở thật sau khi áp dụng cấu hình mới: 12 checks đạt, gồm default 0.30, reject trên cap,
  Save/Load, Alternate, X pause, Space follow không stop, blocked-route không đổi nhánh,
  WASD/arrows/QE có chuyển động đúng. Input qua Carb/callbacks, không phải native OS clicks.
- Unit suite: 57 bài, 55 đạt, 2 SONIC skips; compileall và git diff --check đạt.

M/s ở đây là theo simulation time. Nếu GUI chạy physics chậm hơn 200 bước mỗi giây thực,
robot nhìn sẽ chậm hơn theo wall time. Headless throughput không bảo đảm FPS GUI giống nhau.
Một lượt hai tuyến đạt chưa bảo đảm mọi nhiễu, layout hoặc người di chuyển; perception vẫn SIM GROUND TRUTH.

## Code và cấu hình

- `configs/navigation_speed_v2.json`: default/max 0.30, raw actual-speed envelope 0.6 như cũ;
  gắn factory identity, source hash, checkpoint hash và evidence audit.
- `configs/patrol_routes.json`: cap cả hai tuyến 0.30.
- `navigation/speed_profile.py`: kiểm tra identity, giá trị và audit PASS trước khi mở cap >0.22.
- `navigation/multi_route_controller.py`: đọc profile, giảm tốc sớm trước góc;
  local tracking config dùng bản sao để cap tạm thời không làm mất tốc độ người dùng chọn.
- `navigation/route_manager.py`: route không được vượt mức đã xác minh.
- `navigation/patrol_ui.py`: label cap theo controller, preset, xử lý sai số float32.
  Legacy V1 giữ giới hạn cũ qua default parameter của `validate_navigation`.
- README và `multi_route_gui_manual.md` cập nhật cách dùng; báo cáo baseline giữ riêng.

## Bằng chứng và lệnh kiểm tra

- `docs/navigation/speed_sweep_results.json`, `speed_sweep_extra.json`.
- `docs/navigation/factory_speed_0.30_results.json`, `factory_speed_0.30_audit.json`.
- `docs/navigation/speed_ui_results.json`.
- `logs/factory_speed_0.30.log`, `speed_ui.log`, `speed_unit_final.log`.
- Trace cuối: `logs/patrol_runs/20261111_070918/multi_route_trace.json`.
- Failed/aborted attempts giữ tại `factory_speed_0.35_attempt_1.json`,
  `factory_speed_0.35_attempt_2.json`, và `factory_speed_0.50/0.65/0.80/1.00_results.json`.
- Các nhánh tracking tốc độ cao thử nghiệm không được đưa vào đường chạy production;
  source archive nằm ở `logs/speed_experimental_controller_20261111.py`.
- Backup trước khi nâng cap: `logs/before_speed_promotion_20261111_072141/`.

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat scripts/test_factory_speed.py --speed 0.30 --no-ros-env
& C:\isaacsim\python.bat scripts/audit_factory_speed.py --speed 0.30
& C:\isaacsim\python.bat scripts/test_multi_route_ui.py --output-name speed_ui_results.json --no-ros-env
& C:\isaacsim\python.bat -m unittest discover -s tests -v
```

Đọc status/checks JSON để kết luận, không chỉ exit code của python.bat.
Giữ nguyên source Factory USD và policy; không commit/push trong yêu cầu này.
