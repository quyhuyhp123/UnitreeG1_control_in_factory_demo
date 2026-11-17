# Cập nhật tốc độ Factory V2 — user override, 11/11/2026

Đã sửa trực tiếp project hiện tại `D:\unitree_g1`, được xác định từ working directory,
README, config và remote repository. Không tạo thư mục factory_v2 mới, không đổi branch,
không commit/push. HEAD vẫn `d1b64da79268f2be7e43bca3b05699fdff9e9458`.
Không tìm thấy AGENTS.md ở root, thư mục cha hoặc các thư mục code đã kiểm tra.

## Trạng thái và cách chọn tốc độ

- Trần profile (khóa legacy `maximum_verified_m_s`): **1.0 m/s theo user override**, không phải kết quả chứng nhận mới.
- Mặc định **0.30 m/s**; giới hạn vận tốc thực trong supervisor **1.25 m/s**.
- `unverified_user_override=true`; `override_note` ghi rõ audit hoàn tất gần nhất là 0.30 m/s,
  chưa xác nhận hoàn tất hai tuyến ở 1.0 m/s. Giữ nguyên checkpoint/factory/evidence path.
- Audit vẫn phải tồn tại, nằm trong project và có status PASS. Chỉ bỏ phép so tốc độ audit
  với trần khi flag là boolean true rõ ràng. Flag false/thiếu vẫn bắt buộc khớp.

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Controller locomotion
```

Mở **G1 PATROL NAVIGATION → Schedule / limits**. Chọn **0.22 / 0.30 / 0.50 / 0.80 / 1.0 m/s**;
các nút chỉ điền giá trị, không tự chạy. Apply chỉ cho IDLE/STOPPED/CYCLE_COMPLETE như trước.
START áp dụng mức đang chọn rồi mới bắt đầu theo lịch. Khi mở mới vẫn mặc định 0.30 m/s.
Space/Follow và X/Pause không bị thay đổi.

Vận tốc yêu cầu là min(lựa chọn người dùng, cap tuyến, cap profile), còn cap tức thời thấp hơn
khi tới góc hoặc thiếu quãng phanh. Load config người dùng cũ có thể khôi phục cap riêng của
route thấp hơn; không sửa/ghi đè file cấu hình người dùng đã lưu. Reset route configuration
khi STOP sẽ nạp lại `configs/patrol_routes.json` hiện tại.

## Các file sửa và tạo

| File | Thay đổi |
|---|---|
| `configs/navigation_speed_v2.json` | Trần 1.0, default 0.3, actual envelope 1.25, explicit override và ghi chú chưa kiểm chứng |
| `configs/patrol_routes.json` | Chỉ nâng hai speed_limit_m_s lên 1.0; giữ nguyên waypoint, heading, turn rate, tolerance, hình học |
| `navigation/speed_profile.py` | Override chỉ bỏ so khớp tốc độ audit; giữ identity, domain, path và status; đọc UTF-8 trên Windows |
| `navigation/multi_route_controller.py` | Nối tracking qua checkpoint thẳng; plane crossing; khoảng cách dọc tuyến; speed cap và policy scaling; bật swept-static riêng V2 |
| `navigation/route_motion.py` (mới) | Hàm hình học, projection/arc distance, curve/braking envelope, co XY cùng tỷ lệ |
| `navigation/route_graph.py` | CorridorChainGrid chỉ hợp các cạnh thẳng bắt buộc đã validate; không nối nhánh qua góc |
| `navigation/safety_supervisor.py` | Capsule quét footprint theo hướng actual/command cho vật cản tĩnh; giữ người/dynamic/near collision/feedback faults; mode mặc định cũ cho V1 |
| `navigation/route_manager.py` | Đọc JSON route bằng UTF-8, tránh phụ thuộc codepage Windows |
| `navigation/patrol_ui.py` | Preset theo yêu cầu, lọc theo trần, ghi chú override; giữ state gating và START áp dụng giá trị |
| `tests/test_speed_override.py` (mới) | Kiểm tra logic profile, route motion/controller, safety, UI |
| `README.md`, `docs/navigation/multi_route_gui_manual.md` | Hướng dẫn mới, phân biệt override và evidence |
| `docs/navigation/speed_override_update.md` (mới) | Báo cáo này |

Backup trước sửa: `logs/before_speed_override_20261111_204036/`.
Không thay asset, checkpoint, PD/joint mapping hoặc các báo cáo kết quả cũ.

## Cơ chế điều khiển

Tracking path kéo dài qua các graph waypoint cùng hướng, dừng ở góc thật hoặc waypoint có
heading bắt buộc. Current segment vẫn cập nhật theo thứ tự, không tăng cycle sớm và không
đổi route khi bị chặn. Waypoint thẳng được chuyển khi tới tolerance hoặc vượt mặt phẳng
vuông góc phía trước, đồng thời còn trong lateral deviation/corridor/map cho phép.

Khoảng cách đến góc/đích là chiều dài còn lại trên polyline đã plan, qua các checkpoint thẳng.
Lookahead = max(lookahead cấu hình, cap / 0.7 + 0.05). LocalPlanner nhận góc/đích kế tiếp làm
mục tiêu dừng, nên không bị ngưỡng chậm 0.5 m của checkpoint thẳng làm dừng không cần thiết.

Curve cap nội suy từ requested ở 2.0 m về min(requested, 0.22) ở 1.2 m. Áp dụng thêm:

```
t = prediction_seconds + reaction_latency_seconds
room = distance - stopping_extra_m
a = conservative_deceleration_m_s2
envelope_speed = 0                              nếu room <= 0
                 a * (-t + sqrt(t*t + 2*room/a)) nếu room > 0
cap = min(requested, curve_cap, envelope_speed)
```

XY được co cùng tỷ lệ theo cap và miền policy bất đối xứng, cả trước/sau ramp; yaw được clamp
riêng. Dừng/quay và điều kiện settled tại góc/đích vẫn được giữ, không sửa world pose.

V2 static safety kiểm tra khoảng cách chính xác từ segment dự báo/phanh đến AABB so với
bán kính footprint (capsule), thay vì dùng bán kính quãng phanh quanh robot cho kệ tĩnh bên hông.
Vẫn kiểm cả actual và command, giữ sát-vật-cản 5 cm, human extra, dynamic prediction,
map/footprint, dữ liệu stale/nonfinite, quaternion/tilt, ngã và actual-speed limit. V1 mặc định
`swept_static_braking=False`, tiếp tục nhánh xử lý cũ.

## Kiểm tra đã chạy — LOGIC ONLY

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat -m unittest discover -s tests -v
& C:\isaacsim\python.bat -m compileall -q navigation tests/test_speed_override.py
```

Kết quả: **77 tests; 75 đạt, 2 SONIC skips**. Log: `logs/speed_override_logic.log`.
Compileall và `git diff --check` đạt.

Các ca mới kiểm tra:
- Flag phải là boolean true; audit mismatch chỉ được bỏ trong override; status fail, sai hash,
  sai factory, NaN hoặc vượt miền vẫn bị từ chối.
- Hai route thực validate được với cap mới; look-ahead Route 1 không đi tiếp vào nhánh nâu,
  Route 2 đi tiếp qua nút giữa đúng corridor; không thay hình học.
- Controller/planner/selector thật trên graph tổng hợp: qua waypoint thẳng vẫn có lệnh tiến;
  vượt mặt phẳng thì chuyển segment, quá lệch ngang thì không chuyển; không tăng cycle.
- Look-ahead cho phép lệnh 1.0 trong phép tính logic; không diễn giải thành robot đã đạt 1.0.
- Curve 2.0/1.6/1.2 m; bất đẳng thức stopping envelope; room không dương trả zero.
- Co XY giữ hướng và tuân thủ các giới hạn dương/âm/yaw của policy.
- Kệ tĩnh bên hông cho qua nếu ngoài capsule; vật cản phía trước, người, dynamic và gần footprint vẫn dừng.
  V1 vẫn dừng theo nhánh radial cũ trong cùng ca đối chiếu; V2 giữ stale/fall/speed/quaternion faults.
- Preset được lọc; START dùng giá trị đang chọn; đang MOVING không Apply được; nhập trên 1.0 bị từ chối.

**Không chạy SimulationApp/PhysX hoặc GUI trong đợt kiểm tra này; không chạy robot thật.**
Các fixture/state tổng hợp không phải telemetry vật lý. Chưa xác nhận độ ổn định hay hoàn tất
hai tuyến ở 1.0 m/s, cũng chưa đo lại tốc độ với controller mới. Audit 0.30 m/s là kết quả lịch sử
của bản trước, không chứng nhận tự động cho cơ chế mới.

Audit và policy giữ nguyên SHA256 trước/sau:
- `factory_speed_0.30_audit.json`: `72bbf8fac4d26548e306f7b2c876fc928f004ca2c944cf80cf1b806fcf30061d`.
- `policy.onnx`: `2a66ca6336eadb3c0b34b557763f3e06d01ff8fcf6260dd4cedbd69d6093fc28`.
