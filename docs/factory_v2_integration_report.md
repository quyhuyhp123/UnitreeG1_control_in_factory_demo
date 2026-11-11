# Factory V2 integration — 2026-11-10

Đã tích hợp và kiểm thử thực tế trên Isaac Sim 6.1 / PhysX. Branch `codex/factory-v2-integration` được tạo trước khi sửa. Trước tác vụ chỉ có source/asset Blender V2/reference chưa tracked, không có tracked modifications; không ghi đè các file đó. Chưa commit/push tác vụ này.

## Chạy và rollback

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Controller locomotion
# Mặc định cũng chọn V2
.\run_sim.bat
# Rollback ở phiên mới
.\run_sim.bat -Factory v1 -Controller locomotion
```

Cả -Factory v1/v2 và default V2 đã chạy qua launcher thật: exit 0, completed, không error log. Chọn factory yêu cầu restart; chỉ một factory được reference tại /World/FactoryScene.

## Physics và collision

Source giữ nguyên: assets/factory/factory_layout_v2/factory_layout_v2.usd. Layer mới factory_layout_v2_physics.usd tham chiếu source tương đối, metersPerUnit=1, Z-up. Runtime giữ /World/PhysicsScene và deactivate scene trong reference; gravity (0,0,-9.81), dt 0.005 s / 200 Hz, policy 50 Hz/decimation 4, CPU PhysX như cũ.

42 static Cube colliders được tạo từ đúng world bounds của proxy: sàn 1, tường 4, máy 14, bàn 2, ghế 2, người 16, mái 1, lintel 1, cửa cuốn đang mở 1. Không collider trùng trên mesh hiển thị hoặc proxy nguồn; không collider trên vạch, green overlay hoặc marker. Không thêm ground plane/apron V1. Sàn /World/FactoryScene/PhysicsColliders/Floor, top Z=0, friction static/dynamic 0.8/0.6, restitution 0; contact offset 0.02/rest 0. Invisible collider bounds được đọc với BBoxCache positional ignoreVisibility=True theo API OpenUSD thực tế.

Mái ẩn hiển thị để quan sát nội thất nhưng collider và source USD giữ nguyên. Giữ đèn nguồn, thêm DomeLight 500 khi chưa có dome. Cập nhật preview: dùng camera lighting cho hai viewport; không reset/import lại stage khi dựng GUI. Không có pose update để tạo bước đi.

## Spawn / policy / reset

Marker Robot_Entry_Spawn được đọc world transform sau reference: **(-8.85,-13.75,0) m**, heading **0°**, robot forward **+X**, hướng vào cửa tây nam. Không dùng (0,-4). Spawn nằm trong allowed polygon, radius clearance 0.67 m không có blocker; map mới xác minh footprint free. Base height do FK standing từ deploy.default_joint_pos, không đặt pelvis Z=0.
Reset root thực tế [-8.850000381469727, -13.75, 0.7854665517807007]; quaternion WXYZ [1.0, -3.293137290638626e-11, 0.0, -2.4071916335244303e-12]. History terms và policy step count về 0. Initial layer V2 khôi phục pose/joints/velocity; lệnh cũ bị xóa, patrol về MANUAL, dùng safety settle/paused reset hiện có. Không kinematic/khóa chân/teleport để đi.

Checkpoint/PD/joint mapping/observation/timing giữ nguyên. Không sửa SONIC, không training hoặc mạng robot thật.

**Phím theo xác nhận cuối của người dùng:** cả V1/V2 Space chỉ khóa camera Follow, X dừng lệnh/tạm dừng AUTO, R reset; V2 giữ F làm phím Follow phụ. Space không xóa phím đang giữ hoặc phát stop event. Đã cập nhật cả cấu hình V2 và script dựng lại. WASD/Arrow/QE không đổi.

## Navigation / camera

configs/navigation_map_v2.json chuyển outer thành allowed lane, hai holes thành forbidden zones tương đương. Không sửa A*, LocalPlanner hoặc SafetySupervisor. Live obstacle bounds từ collider, người có navigation:kind=human (SIM GROUND TRUTH, không AI). A lấy từ spawn mới, B UNSET; START từ chối nếu chưa chọn B/validate. Config save riêng configs/patrol_v2_user.json; factory_id và source USD SHA256 bảo vệ map/source và từ chối patrol V1. Chưa chạy Auto Patrol thực tế trong V2/Multi-Waypoint.

Overview bao nhà xưởng 20x30; Follow theo body frame, Free giữ điều hướng chuột. Top View V2 dùng camera chính perspective gần thẳng đứng tại (0,-3,45), focal 19 mm, tránh camera session/orthographic đen trên cài đặt thực tế. Ảnh cuối Top/Overview đã được xem và xác nhận đầy đủ. V2 tắt head-camera preview: một viewport, không RGB-D/LiDAR/camera AI. RTX real-time D3D12/TAA, low quality, không path tracing. V1 giữ camera cũ.

## Kết quả kiểm thử

Run V2 cuối: D:\unitree_g1\logs\keyboard_runs\20261110_195314; log logs/factory_v2_acceptance_final.log.

| Kiểm tra | Kết quả |
|---|---|
| factory_reference | PASS |
| collider_count | PASS |
| spawn_heading | PASS |
| standing | PASS |
| navigation_new_spawn | PASS |
| auto_unset_b_rejected | PASS |
| islands_forbidden | PASS |
| drop_floor | PASS |
| drop_door | PASS |
| drop_machine | PASS |
| key_W | PASS |
| key_UP | PASS |
| key_S | PASS |
| key_DOWN | PASS |
| key_A | PASS |
| key_LEFT | PASS |
| key_D | PASS |
| key_RIGHT | PASS |
| key_Q | PASS |
| key_E | PASS |
| space_binding | PASS |
| reset_new_spawn | PASS |
| camera_modes | PASS |

Đứng 3 s simulation: tilt tối đa 0.900°, floor contacts 10836. W trong 3 s tiến 0.835 m theo body +X. Cube 0.2 m rơi sàn/ở cửa dừng tâm Z≈0.10 m, trên máy Z≈2.43 m. Tất cả 23 kiểm tra V2 PASS; V1 rollback 19 kiểm tra PASS. Unit suite 45 PASS, 2 SONIC SKIP.

Startup 17.18 s; mean loop FPS 41.74; physics steps/wall second 166.97 (configured 200 Hz, không khẳng định realtime). Peak whole-GPU VRAM 4411 MiB, không process-only. Mẫu RAM từ acceptance V2 trước chỉnh camera: working set 7839 MiB, peak working set 8009 MiB, CPU time 263.36 s, không phải utilization. Không OOM trong các lượt đã chạy; không bảo đảm mọi tình huống trên 12GB.

Các log cuối có 0 Error/Fatal/Traceback; vẫn có warning USD/Fabric/hydratexture/gizmo. Lần đầu gặp BBoxCache keyword không tương thích, ảnh Top đen rồi overexposed khi thử extra light; đã sửa và test lại, không dùng lượt lỗi làm bằng chứng thành công.

**NOT TESTED:** native OS mouse/keyboard, GUI docking, tuần tra V2, mọi kiểu G1 đâm từng máy/tường hoặc chứng minh không penetration trong mọi contact. Input acceptance dùng logical Carb keyboard, physics thật; collision probe kiểm tra sàn/cửa/máy. Hành lang 1.56–1.63 m còn hẹp; geometry map hợp lệ không bảo đảm mọi góc cua locomotion.

## File tạo / sửa

Tạo configs/factory_v2.json, configs/factory_selector.json, configs/navigation_map_v2.json; controllers/factory_selector.py; blender/build_factory_v2_physics.py; physics USD và physics_layer_manifest.json; scripts/test_factory_v2.py; tests/test_factory_selector.py; các báo cáo acceptance/launcher/summary trong docs.

Sửa controllers/factory_g1_runtime.py, controllers/g1_keyboard_runtime.py, navigation/obstacle_detector.py, navigation/patrol_controller.py, navigation/patrol_ui.py, scripts/run_factory_g1.py, scripts/run_sim.ps1, README.md. run_sim.bat, source Blender/USD, factory_physics.usd V1, configs/factory_g1.json, robot asset và policy/checkpoint không đổi nội dung; hash/chứng cứ nằm trong factory_v2_integration_summary.json. Khác line endings CRLF/LF của config V1 được normalize khi so sánh, Git không ghi nhận thay đổi file này.

Checklist GUI: mở V2; kiểm tra G1 ở cửa; W/Up vào xưởng; S/Down lùi ngắn; A/D quay; Space Follow không dừng; X dừng; Free/Overview/Top hoạt động; R sau dừng về cửa; mở phiên riêng -Factory v1; không load A/B V1 sang V2.

Chạy lại acceptance:

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_factory_v2.py --factory v2 --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_factory_v2.py --factory v1 --no-ros-env
```

Ảnh camera khởi động thực tế: logs/keyboard_runs/20261110_200007/factory_g1_gui.png. Đã xem ảnh: G1 hiện rõ qua cửa, nhìn vào +X; launcher mặc định exit 0, error_lines 0. Kết quả riêng trong factory_v2_entry_launcher_results.json.

Cập nhật phím cuối: kiểm tra unit với cấu hình V2 thật xác minh giữ W + Space không đổi command/stop_generation, follow_generation tăng; X dừng đúng. Bài acceptance PhysX phía trên được ghi trước đổi phím (Space khi đó là stop); chưa chạy lại toàn bộ PhysX chỉ vì thay mapping.

## Camera đầu được bật lại theo yêu cầu tiếp theo

V2 head_camera.enabled=True, giữ preview G1 Head Camera | LIVE ở góc trên phải, 320×180. Cập nhật cả script tạo config để không bị tắt khi dựng lại. Đây là viewport phụ xem RGB, không AI/depth/LiDAR và không thay observation policy. Các số VRAM/acceptance phía trên được đo khi preview còn tắt; không dùng chúng làm số đo riêng của bản bật preview. Cần khởi động lại phiên đang mở để nạp cấu hình.

Kiểm tra camera đầu bản cuối: GUI run 20261110_204045, exit 0, error_lines 0; ảnh 320×180 đã xem có nội dung đúng hướng đầu robot. Dùng Kit action set_lighting_mode_camera (đã kiểm tra API cài đặt) để tránh ảnh đen khi hai viewport dùng stage lighting. Không thêm đèn/collider; không đổi physics, policy, spawn hoặc phím Space/X. Bỏ reset stage khi khởi tạo GUI để tránh stage widget truy cập prim vừa bị xóa. Các lần thử ảnh đen và một lỗi invalid null prim được giữ trong log cũ, không tính là kết quả cuối. Native OS mouse/keyboard chưa kiểm thử riêng.
