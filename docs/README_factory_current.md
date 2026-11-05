# G1_factory_keyboard

Một nhà xưởng Blender 10×10 m, một Unitree G1 29-DOF, policy velocity ONNX chính thức của Unitree và điều khiển bàn phím trên NVIDIA Isaac Sim. Chỉ mô phỏng bằng PhysX; không robot thật, ROS, Navigation hoặc training.

**Trạng thái:** đã chạy kiểm thử tích hợp physics, đứng/tiến/lùi/vừa đi vừa quay/reset và contact với bàn. A/D quay tại chỗ chưa đạt với checkpoint hiện có. Phím vật lý Windows cần checklist thủ công; test tự động sử dụng Carb logical keyboard. Xem docs/factory_g1_integration.md và JSON kết quả để biết giới hạn, không coi toàn bộ yêu cầu đã đạt.

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
| A / ← | Lệnh quay trái; policy chưa quay tại chỗ đạt yêu cầu |
| D / → | Lệnh quay phải; policy chưa quay tại chỗ đạt yêu cầu |
| Q / E | Ngang trái / phải |
| W+A hoặc ↑+→ | Đi và quay |
| SPACE | Đưa target velocity về 0 ngay |
| R | Yêu cầu reset an toàn |

Giữ phím để đi; nhả để giảm target về 0. Phím đối nghịch triệt tiêu. Robot vẫn có quán tính; target=0 không phải phanh cơ học tức thì. Không có tránh chướng ngại tự động. Đổ thì runtime Pause; dùng R, Resume và nhấn mới. Reset dễ kiểm tra nhất bằng Pause → R → Resume. Đóng cửa sổ để kết thúc.

## Camera, chế độ so sánh và test

Mặc định camera theo vị trí robot, một viewport. Không có camera sensor. Góc nhìn có thể bị tường/vật cản che khi robot đi xa.

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

Self-test có thể exit 0 dù bài quay tại chỗ chưa đạt; phải đọc test_status/failed_tests trong JSON. Crash/Fatal/exception hoặc thiếu marker kết thúc khiến launcher báo lỗi. Không coi exit code của python.bat là bằng chứng duy nhất.

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
├── configs\locomotion.yaml  # contract policy đã kiểm chứng
├── controllers\             # policy, joint adapter, keyboard, runtime
├── scripts\                 # entry point, launcher PowerShell, test
├── logs\                    # stdout/stderr, telemetry, performance, ảnh
└── docs\                    # báo cáo môi trường và các bước kiểm thử
```

configs/factory_g1.json cấu hình đường factory/checkpoint, marker spawn, timestep, control rate và render. Physics dt=0.005 s, policy 50 Hz là contract hiện đã kiểm chứng; thay timing tùy ý bị từ chối. Spawn heading hiện hỗ trợ 0°. Checkpoint thay thế phải đúng hash/interface tương ứng, không chỉ đổi tên file.

Renderer RaytracedLighting, thấp, 960×640, một G1/một environment, không RTX sensor/path tracing. Backend mặc định Direct3D12 trên Windows để tránh lỗi shader Vulkan đã gặp với driver hiện có; có thể chọn vulkan trong config để chẩn đoán, chưa coi là ổn định. Không sửa cài đặt hệ thống hoặc driver.

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

Các báo cáo: docs/environment_report.md, factory_scene.md, factory_physics.md, g1_articulation_report.md, g1_joint_indices.md, locomotion_report.md, keyboard_report.md và factory_g1_integration.md. Không cần Blender chạy để mở USD đã tạo. Chưa training lại RL hoặc thay checkpoint để xử lý hạn chế quay tại chỗ.
