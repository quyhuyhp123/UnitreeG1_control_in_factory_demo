# G1_factory_keyboard

## Factory V2 — nhà xưởng Blender mới

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Controller locomotion
```

Mặc định launcher chọn V2. G1 xuất hiện tại cửa tây nam, marker
`(-8.85, -13.75, 0)` m, hướng +X vào xưởng; base đứng khoảng Z=0.785 m theo FK
của policy hiện có. Một factory, một G1, viewport chính và cửa sổ
**G1 Head Camera | LIVE** góc trên phải (320×180). Mái ẩn hiển thị để xem bên trong,
collider mái vẫn có. Camera đầu chỉ xem trực tiếp, không đưa vào policy hay AI.

Theo lựa chọn đã xác nhận: **Space chỉ khóa camera Follow, X dừng, R reset** ở cả V1/V2.
V2 còn cho phép F khóa camera. Space không ngắt di chuyển hoặc AUTO. WASD/arrow/Q/E không đổi.
Camera Free, Follow, Overview và Top View có trên panel.

V2 dùng `configs/navigation_map_v2.json` và Multi-Route trong `configs/patrol_routes.json`.
Trong **ROUTE SELECTION**, chọn Route 1 (giữa), Route 2 (phải), hoặc Alternate;
Apply khi STOP/đứng chậm, PREVIEW/VALIDATE rồi START. Mỗi tuyến có checkpoint bắt buộc,
không tìm đường tắt qua nhánh khác. Mặc định mới: delay 5 s, interval 120 s,
Repeat off, Max Cycles=1; bật Repeat và >=2 cycles để kiểm tra Alternate.
Cấu hình route lưu riêng `configs/patrol_route_user.json`. Không load A/B V1 vào V2.
Tốc độ tuần tra V2 mặc định và giới hạn đã kiểm thử là **0.30 m/s** (11/11/2026).
Trong **Schedule / limits**, chọn nhanh **0.22 / 0.25 / 0.30 m/s**, rồi Apply khi STOP
hoặc trước START. Robot giảm về tối đa 0.22 m/s trong 1.2 m trước góc/đích,
và tiếp tục giảm tốc để dừng/quay. Các mức cao hơn chưa đạt kiểm thử toàn xưởng;
giới hạn 1 m/s của policy chỉ đạt bài đi thẳng ngắn trên sàn trống.
Xem [báo cáo thử tốc độ](docs/navigation/speed_test_report.md).
Legacy A/B V2 còn được giữ qua `multi_route_enabled=false`; khi đó B vẫn UNSET.
Các ví dụ A/B gần `(0,-4)` trong tài liệu V1 phía dưới không áp dụng cho V2.

Rollback bằng `./run_sim.bat -Factory v1 -Controller locomotion`; chọn factory
chỉ có hiệu lực khi khởi động phiên mới. Không chuyển môi trường khi đang chạy.
Xem [báo cáo tích hợp V2](docs/factory_v2_integration_report.md).
Hướng dẫn [Multi-Route GUI](docs/navigation/multi_route_gui_manual.md),
[kiến trúc](docs/navigation/multi_route_architecture.md),
[kết quả kiểm thử từng tuyến](docs/navigation/multi_route_test_report.md).

## Automatic Patrol A/B — V1 / legacy

Chạy `./run_sim.bat -Factory v1 -Controller locomotion`, mở panel **G1 PATROL NAVIGATION**.
Mặc định vẫn MANUAL, A lấy từ robot, B chưa đặt. Nhập A/B, nhấn **Apply A/B + Validate Route**,
cấu hình lịch rồi **START PATROL**. Bộ điểm đã thử: A gần `(0,-4)`, B `(0,-2.5)`.
Mặc định chờ 5 giây, đến B chờ 10 giây, về A rồi nghỉ 5 giây trước lượt tiếp theo.
Bật Repeat và đặt Max Cycles = 10 để chạy mười vòng A→B→A. Max Speed mặc định 0.35 m/s,
cho phép 0.12–0.35 m/s; vẫn giảm tốc gần đích. Thời gian chờ dùng clock đã chọn;
simulation chậm sẽ kéo dài thời gian chờ ngoài đời. Khi đang chờ tại A có thể sửa
Interval/Speed rồi nhấn **Apply schedule / speed (while waiting)**.
SPACE chỉ khóa camera Follow, không dừng robot hoặc tuần tra. X dừng lệnh tay / tạm dừng AUTO; Resume phải chủ động trên UI.

Patrol dùng A* và footprint trong làn cho phép, phản hồi PhysX và Locomotion RL hiện tại.
Vật cản/người được đọc từ **SIM GROUND TRUTH**, chưa có AI perception. SONIC vẫn bị khóa.
Xem [hướng dẫn GUI](docs/navigation/gui_manual.md), [kiến trúc](docs/navigation/architecture.md),
[lịch tuần tra](docs/navigation/patrol_schedule.md), [kết quả kiểm thử](docs/navigation/navigation_test_report.md).
Các test dùng Python Isaac Sim: `scripts/test_patrol.py`, `test_patrol_obstacles.py`, `test_patrol_gui.py`.

## SONIC integration status (2026-11-08)

**PARTIALLY IMPLEMENTED. SONIC does not yet control G1 in PhysX.**
Default `run_sim.bat` continues using the verified Locomotion RL backend.
`run_sim.bat -Controller locomotion` selects it explicitly.
`run_sim.bat -Controller sonic` currently rejects before simulation startup with
`CONTROLLER_UNAVAILABLE`; model sessions and offline idle planner inference are
verified, but full observation/reference and actuator parity are unresolved.
The G1 CONTROLLER MANAGER panel reports this limitation and blocks hot switching.

See [run instructions](docs/sonic_integration/run_instructions.md),
[compatibility](docs/sonic_integration/compatibility_report.md),
[baseline evidence](docs/sonic_integration/baseline_report.md), and
[test report](docs/sonic_integration/test_report.md).
Large SONIC weights stay local and are reproducibly downloaded with
`scripts/download_sonic.py`; existing locomotion weights and factory assets are preserved.

Giao diện G1 Keyboard Control/Head Camera là **Isaac Sim**. Blender dùng để sửa mô hình tại blender/factory.blend, sau đó xuất USD để Isaac chạy mô phỏng. Hướng dẫn hai ứng dụng và cập nhật mô hình: docs/blender_isaac_workflow.md. Gizmo biên tập được ẩn riêng khỏi khung head camera.

Một nhà xưởng Blender 10×10 m, một Unitree G1 29-DOF, policy velocity ONNX chính thức của Unitree và điều khiển bàn phím trên NVIDIA Isaac Sim. Chỉ mô phỏng bằng PhysX; không robot thật, ROS, Navigation hoặc training.

**Cập nhật:** robot bắt đầu tại cửa, world xyz sàn [0,-4,0], quay mặt +Y vào nhà xưởng. Đã port và thử policy G1 velocity chính thức từ Unitree RL Mjlab: A/← và D/→ quay tại chỗ bằng joint targets + PhysX với vx=vy=0. Policy Unitree RL Lab cũ được giữ nguyên để đối chiếu. Phím vật lý Windows vẫn cần checklist thủ công; bài tự động dùng Carb logical keyboard. Xem docs/entrance_in_place_turning.md cho hợp đồng policy và kết quả hiện hành.

Policy mới đã quay trái khoảng +155.82° và phải -128.30° trong 5 giây, độ dịch chuyển ngang tối đa khoảng 0.10/0.12 m. docs/turning_check.md là kết quả lịch sử của policy cũ, không phải policy mặc định hiện tại.

## Mở bằng một lệnh

Trong PowerShell:

```powershell
& 'D:\unitree_g1\run_sim.bat'
```

Hoặc mở trực tiếp run_sim.bat trong Explorer. Launcher dùng installation đã xác minh từ configs/environment.json: C:\isaacsim, Isaac Sim 6.1.0-rc.26, Python 3.12.13 đi kèm. Không chạy bằng Python mặc định của Windows. Lần đầu có thể cần đợi shader/cache; cửa sổ console cho biết đường log.

Khi cửa sổ mở, click viewport hoặc bảng **G1 Keyboard Control**. Simulation tự Play, robot dùng policy giữ thăng bằng. Thả hết phím rồi nhấn mới nếu vừa đổi focus/Pause/Resume.

| Phím | Chức năng |
|---|---|
| W / ↑ | Tiến |
| S / ↓ | Lùi |
| A / ← | Quay trái tại chỗ, yaw target +0.6 rad/s |
| D / → | Quay phải tại chỗ, yaw target -0.6 rad/s |
| Q / E | Ngang trái / phải |
| W+A hoặc ↑+→ | Đi và quay |
| SPACE | Khóa camera chính về bám robot, không dừng di chuyển |
| X | Dừng lệnh tay / tạm dừng AUTO |
| R | Yêu cầu reset an toàn |

Giữ phím để đi; nhả để giảm target về 0. Phím đối nghịch triệt tiêu. Robot vẫn có quán tính; target=0 không phải phanh cơ học tức thì. Không có tránh chướng ngại tự động. Đổ thì runtime Pause; dùng R, Resume và nhấn mới. Reset dễ kiểm tra nhất bằng Pause → R → Resume. Đóng cửa sổ để kết thúc.

## Camera, chế độ so sánh và test

Mặc định **Free Camera**, không tự kéo camera theo robot. Dùng chuột phải + rê để nhìn quanh, giữ chuột giữa + rê để pan, lăn chuột để tiến/lùi góc nhìn. WASD/mũi tên/Q/E vẫn dành cho robot. HUD có nút Camera: Free / Follow / Overview; chọn Free để ngừng khóa theo robot, Follow để bám lại. Free giữ góc nhìn khi reset robot. Camera đầu có khung xem riêng ở góc phải. Cần đóng/mở lại chương trình cũ để nạp thay đổi.

**Cập nhật camera đầu:** góc trên bên phải có G1 Head Camera | LIVE, camera gắn trên đầu qua torso_link, hướng theo robot, ảnh RGB xem trực tiếp 320×180. SPACE đưa camera chính về Follow ở phía sau robot; chọn Camera: Free để di chuyển lại bằng chuột. Ảnh head camera chưa được dùng trong observation của policy, không có depth/LiDAR hay vision AI. Hiện có viewport chính và một viewport nhỏ head camera theo yêu cầu mới; các ghi chú một viewport/không camera ở báo cáo cũ là lịch sử.

**S/↓ bị triệt tiêu do W kẹt:** log native từng ghi nhận W còn trong pressed_keys khi bấm lùi, làm vx=0. Đã thêm release-only polling GetAsyncKeyState trên Windows để loại phím đã thả bị mất KEY_RELEASE; không tạo PRESS từ polling. Nếu thật sự giữ cả W và S/↓, lệnh vẫn triệt tiêu đúng. Khi lùi hãy thả phím tiến, kiểm tra HUD Keys và Desired vx âm. X xóa lệnh giữ cũ.

Thao tác chuột theo [Omniverse viewport navigation](https://docs.omniverse.nvidia.com/extensions/latest/ext_core/ext_viewport/navigation.html). Alt+chuột trái cũng orbit được, nhưng shortcut Alt hiện xóa lệnh robot đang giữ; thả rồi nhấn lại phím robot sau thao tác đó. Để vừa điều khiển robot vừa nhìn quanh, dùng chuột phải/giữa và bánh xe, không dùng Alt.

```powershell
# Góc tổng quan cố định
& 'D:\unitree_g1\run_sim.bat' -Overview
# Cùng G1/policy/keyboard nhưng chỉ sàn phẳng
& 'D:\unitree_g1\run_sim.bat' -Flat
# Các bài physics A–J, input tự động qua Carb
& 'D:\unitree_g1\run_sim.bat' -SelfTest -Headless
# Bài so sánh flat, H không áp dụng
& 'D:\unitree_g1\run_sim.bat' -Flat -SelfTest -Headless
# GUI tự đóng sau 15 giây và lưu ảnh, phục vụ smoke test
& 'D:\unitree_g1\run_sim.bat' -Duration 15
```

Self-test phải đọc test_status/failed_tests trong JSON để biết kết quả từng bài. Crash/Fatal/exception hoặc thiếu marker kết thúc khiến launcher báo lỗi. Không coi exit code của python.bat là bằng chứng duy nhất.

## Cấu hình và file

```text
D:\unitree_g1\
├── run_sim.bat
├── blender\                 # factory.blend và script dựng lại
├── assets\factory\          # factory.usd, factory_physics.usd, manifest
├── assets\robot\            # USD G1 chính thức và scene test sinh lại
├── checkpoints\             # policy thật và deploy/provenance
├── configs\factory_g1.json  # factory, spawn, checkpoint, timing, render/camera
├── configs\keyboard.yaml    # mapping, tốc độ, ramp, safe reset
├── configs\locomotion_mjlab.yaml # contract policy mới, 98 obs -> 29 actions
├── configs\locomotion.yaml  # contract policy cũ được giữ nguyên
├── controllers\             # policy, joint adapter, keyboard, runtime
├── scripts\                 # entry point, launcher PowerShell, test
├── logs\                    # stdout/stderr, telemetry, performance, ảnh
└── docs\                    # báo cáo môi trường và các bước kiểm thử
```

configs/factory_g1.json cấu hình đường factory/checkpoint, spawn override [0,-4,0], heading 90°, timestep, control rate và render. Physics dt=0.005 s, policy 50 Hz là contract hiện đã kiểm chứng; thay timing tùy ý bị từ chối. keyboard_overrides.max_yaw_rate=0.6 áp dụng cho policy mới, được ưu tiên hơn giá trị trong keyboard.yaml. Checkpoint thay thế phải đúng hash/interface tương ứng, không chỉ đổi tên file. Đoạn sàn nối EntranceApron bên ngoài cửa hỗ trợ lùi ra ngoài; không đè lên sàn nhà xưởng.

Renderer RaytracedLighting, TAA, chất lượng thấp, viewport chính 960×640 và preview đầu 320×180, một G1/một environment, không depth/LiDAR/path tracing. Backend mặc định Direct3D12 trên Windows để tránh lỗi shader Vulkan đã gặp với driver hiện có; có thể chọn vulkan trong config để chẩn đoán, chưa coi là ổn định. Không sửa cài đặt hệ thống hoặc driver.

Mọi script dùng project root D:\unitree_g1 bằng pathlib khi quản lý file Python. Kit/Warp có thể dùng cache mặc định trên C:. Không có bản sao project ở Desktop/Documents/Downloads. README cũ đã được lưu trong logs/README_before_factory_<timestamp>.md.

## Log và kiểm tra thủ công

Launcher in đường logs/factory_g1_<timestamp>.log; logs/factory_g1_run_summary.json ghi exit/errors/status. Mỗi runtime lưu ở logs/keyboard_runs/<timestamp>/:

- telemetry.jsonl: pose, velocities, joints, action/targets và command.
- events.jsonl: phím/focus/reset, cảnh báo UNKNOWN.
- performance.jsonl: loop FPS, physics steps/s theo wall time, GPU tổng used/total và utilization.
- scene_checks.json, factory_g1_composed.usda: kiểm tra composition và scene phục vụ review.
- integration_results.json khi SelfTest, ảnh GUI khi dùng Duration.

logs/factory_g1_results.json và factory_g1_flat_results.json là kết quả self-test gần nhất; lỗi cũ không bị xóa. FPS là tốc độ loop/render request, không phải GPU presentation profiler; VRAM đo toàn GPU. 200 Hz physics là simulation time, chưa bảo đảm realtime trên máy này.

Sau khi chạy, cần tự kiểm tra giữ/nhả W và ↑, S và ↓, Q/E, W+A, SPACE, Pause/Resume, chuyển ứng dụng khi giữ phím, reset và va chạm. Checklist A–J đầy đủ tại docs/factory_g1_integration.md. Công cụ tự nhấn phím Windows trước đây gửi UNKNOWN, nên chưa xác nhận native keyboard end-to-end.

Nếu mũi tên điều khiển camera: click viewport/HUD, kiểm tra Focus=True/Capture=True, thả và nhấn lại; xem events.jsonl. Callback mapped keys chặn listener phía sau. Extension giành shortcut trước đó cần bỏ shortcut trùng; hướng dẫn chi tiết docs/keyboard_report.md.

## Xử lý lỗi

- Missing USD/checkpoint: đọc đường lỗi, kiểm tra đủ reference dependencies và checkpoint thật. Nguồn/provenance tại docs/locomotion_report.md.
- Joint mapping hoặc policy dimension mismatch: dừng, đối chiếu configs/g1_robot.yaml, locomotion.yaml và deploy; không đoán lại thứ tự joint.
- Isaac API incompatibility: kiểm tra VERSION và đường cài từ environment.json. Không dùng pip Python bên ngoài để chạy Sim.
- Vulkan shader crash `Unsupported ... extractelement` / nvgpucomp64.dll: giữ backend d3d12; xem log, không tự nâng/hạ driver. Crash ngoài Python vẫn được launcher phát hiện qua Fatal/marker.
- CUDA out of memory: giảm resolution, đóng ứng dụng dùng GPU khác, thử -Flat; không tự cài lại phần mềm.
- Runtime exception: đọc logs/factory_g1_error.json và log timestamp hiện tại; file lỗi cũ có thể còn tồn tại, phải đối chiếu thời gian/run.

RTX 5070 12 GB dưới mức VRAM tối thiểu chính thức nêu trong environment_report.md. Scene này đã có số liệu VRAM thực chạy; không suy ra mọi scene khác sẽ phù hợp.

## Mở mã nguồn và báo cáo

```powershell
code 'D:\unitree_g1'
nvidia-smi
Get-Content 'D:\unitree_g1\logs\factory_g1_run_summary.json'
```

Các báo cáo lịch sử: docs/environment_report.md, factory_scene.md, factory_physics.md, g1_articulation_report.md, g1_joint_indices.md, locomotion_report.md, keyboard_report.md và factory_g1_integration.md. Trạng thái policy/spawn mới ở docs/entrance_in_place_turning.md. Không cần Blender chạy để mở USD đã tạo. Không training lại RL; đã đổi sang checkpoint velocity có sẵn chính thức của Unitree sau kiểm thử port.

Chi tiết sửa lùi/SPACE/head preview và bằng chứng: docs/reverse_space_head_camera.md.
