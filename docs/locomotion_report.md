# G1 locomotion — policy verification and simulation results

Ngày 05/11/2026. **Đã chạy G1 tự giữ thăng bằng, bước đi thật, quay khi đang đi và dừng bằng policy ONNX chính thức. Quay tại chỗ với vx=0 chưa đạt; command vx=0,2 m/s cũng chưa đạt bài kiểm tra bước đi.** Không training, không teleport/animation hoặc kết nối robot thật.

## Phiên bản và phương án tích hợp

Isaac Sim thực tế vẫn là `6.1.0-rc.26+release.49347.2d230af4.gl`, Python 3.12.13, Windows 11, RTX 5070 12 GB. Isaac Lab và Unitree RL Lab chưa được cài thành môi trường chạy trong máy này.

[Bảng tương thích Isaac Lab chính thức](https://github.com/isaac-sim/IsaacLab) ghi tag `v3.0.0-EA` phù hợp Sim 6.1; main/release 3.0 hiện hướng tới Sim 6.2. Unitree RL Lab commit đang dùng khai báo package 0.1.0 và dependencies Isaac Lab không ghim phiên bản; **chưa xác minh native Unitree RL Lab + Lab 3.0 EA** và không nhận là đã chạy workflow Isaac Lab.

Phương án đã kiểm thử là **port frozen ONNX và deploy contract sang standalone Isaac Sim 6.1/PhysX**. Không cần cài Isaac Lab/Torch hoặc thay version Sim để inference. Dùng extension có sẵn `isaacsim.pip.onnx` 1.0.1, ONNX Runtime **1.26.0**, CPUExecutionProvider, 2 inference threads. Không cài hoặc nâng/gỡ phần mềm, không chạy chương trình deploy DDS/SDK của Unitree.

## Checkpoint có thật và tải được

Repository: [unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab), commit ghim **4960b84732b0c2ec593dccbfe963fda1bcd7b1e3**.

- [policy.onnx chính thức](https://github.com/unitreerobotics/unitree_rl_lab/blob/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0/exported/policy.onnx): tải thành công, **1.658.204 bytes**, không phải Git LFS pointer.
- [deploy.yaml đi kèm](https://github.com/unitreerobotics/unitree_rl_lab/blob/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml): 2.403 bytes, tải cùng commit. Bản JSON chuyển bằng `yaml.safe_load` để runtime không cần parser YAML mới.
- ONNX SHA256: `610c27e463a8f666aa50a06346678c00b4df3859f10b54bcc1f817c28251406f`.
- ORT đã load và inference thật: input **obs [1,480] float32**, output **actions [1,29] float32**, output hữu hạn. Không tạo network/random weights. Package `onnx` checker không có sẵn, nên không nhận đã chạy onnx.checker; dùng ORT load/infer và physics tests để xác minh thực tế.
- Thư mục velocity v0 ở commit này cung cấp ONNX + deploy.yaml, không có `model_*.pt` hoặc `env.yaml` cho run huấn luyện gốc. Không nhận đây là checkpoint để resume training. Frozen export đủ cho luồng inference đang dùng.
- File tải trong `checkpoints\unitree_g1_velocity_v0`; `provenance.json` ghi commit/hash/bytes. Mã C++ observation/history/action/deploy được lưu dưới `source` để kiểm tra hợp đồng, không build hoặc thực thi controller mạng.

## Hợp đồng observation/action đã port

Không coi ONNX MuJoCo là plug-and-play cho Isaac Lab. Đã đối chiếu `observation_manager.h`, `manager_term_cfg.h`, `observations.h`, `unitree_articulation.h`, `joint_actions.h`, `manager_based_rl_env.h`, cấu hình velocity environment và PPO ở cùng commit.

### Joint mapping

`deploy.joint_ids_map` ánh xạ **policy index → SDK index**:

```text
[0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,16,23,5,11,17,24,18,25,19,26,20,27,21,28]
```

Policy names được suy ra bằng SDK names chính thức, sau đó map **theo tên** sang indices runtime. Trong test này policy order trùng thứ tự runtime PhysX đã đo, nhưng controller vẫn map theo tên và không dựa vào giả định đó. Xem `configs\g1_joint_indices.json` và `configs\locomotion.yaml`.

### Observation — 96 giá trị hiện tại × history 5 = 480

| Thứ tự term | Vector hiện tại | Scale | Vùng trong input 480 |
|---|---|---|---|
| base_ang_vel | 3 angular velocities trong body frame | 0,2 | [0:15] |
| projected_gravity | 3 thành phần q⁻¹·(0,0,-1) | 1 | [15:30] |
| velocity_commands | vx, vy, yaw mục tiêu | 1 | [30:45] |
| joint_pos_rel | 29 q − default_q theo policy order | 1 | [45:190] |
| joint_vel_rel | 29 dq theo policy order | 0,05 | [190:335] |
| last_action | 29 action raw của lần inference trước | 1 | [335:480] |

History **term-major**, mỗi term có 5 mẫu từ cũ nhất tới mới nhất. Không ghép 5 frame đầy đủ theo frame-major. Reset history lặp lại observation đầu cho đủ 5 mẫu, raw action trước ban đầu bằng 0. Theo deploy clip=null, không thêm clip/white noise ở inference. Nguồn PPO ghi empirical_normalization=false; không thêm mean/std normalizer bên ngoài ONNX. Frozen model được chạy nguyên trạng như C++ ORT runner.

Feedback Isaac Sim trả quaternion **WXYZ** và angular velocity world-frame; port đổi bằng inverse quaternion sang body-frame. Không đưa base linear velocity vào actor input vì deploy không có term này; linear velocity vẫn được ghi để đánh giá.

`scripts\check_locomotion_contract.py` đã kiểm tra số học bằng fixture xác định: SDK mapping, vị trí term, scale, reset/shift history, previous raw action, WXYZ rotation, lệnh ngoài range và các biên float32. Đây là unit checks, tách biệt khỏi chứng cứ physics.

### Action, default pose và PD

Target q = **offset + 0,25 × raw_action**, 29 giá trị theo policy order. Offset trùng `deploy.default_joint_pos`; không lấy tư thế FixStand khác trong FSM để thay offset. Config đầy đủ tại `checkpoints\unitree_g1_velocity_v0\deploy.json`.

Default pose: hip_pitch -0,1; knee 0,3; ankle_pitch -0,2; shoulder_pitch 0,3; shoulder_roll trái/phải ±0,25; elbow 0,97; wrist_roll trái/phải ±0,15 rad, các khớp khác 0. Scale/offset/null clipping giữ đúng deploy, không đổi joint mapping để tìm kết quả đẹp.

PD stiffness/damping trong deploy là **SDK order**, khác policy order. Script map theo SDK name trước khi tạo drive. Kp: hip 100, knee 150, waist 200, ankle/arm/wrist 40 Nm/rad. Kd: hip 2, knee 4, ankle 2, waist 5, arms/wrists 10 Nm·s/rad. Đây là deploy gains, không mặc định giữ arm damping=1 của cấu hình robot trước. Runtime readback gains được kiểm tra bằng `Articulation.get_dof_gains()`; effort/velocity hard limits và armature giữ từ cấu hình robot đã xác minh. Không bổ sung tùy tiện action clip khi deploy clip=null; PhysX vẫn có hard joint/torque limits.

### Clock và ranges

- Physics **200 Hz**, dt=0,005 s; inference **50 Hz**, step_dt=0,02 s; decimation **4**. Khớp cấu hình environment nguồn.
- Timeline auto-update tắt khi chạy vòng lặp manual; không để app.update cộng thêm physics steps. Callback PhysX kiểm tra số bước và dt thực tế.
- Ranges: vx **[-0,5;1,0] m/s**, vy **[-0,3;0,3] m/s**, yaw **[-0,2;0,2] rad/s**. Controller từ chối lệnh vượt range. Không có bảo đảm policy bám tốt ở mọi giá trị trong range: 0,2 m/s và pure-yaw là các ví dụ chưa đạt đã đo.

## Scene và kết quả thực tế

Một robot, một mặt sàn 12×12 m, không nhà xưởng hoặc robot khác. Sàn rộng hơn test trước để đủ quãng đường; vẫn một environment. Dùng asset, mass/inertia và collision của bước G1 trước: 33 collider gốc + 12 box proxy. Proxy bổ sung không tự va chạm với robot; collider gốc giữ self-collision. Không khóa chân hoặc base, không kinematic.

Tính FK và chiều cao ban đầu trước Play: pelvis Z≈0,7862 m, đáy collider bàn chân gần sàn. Sau reset, **không sửa world pose hoặc joint positions**; chỉ gửi drive targets từ ONNX. Chứng cứ bước đi gồm dịch chuyển base, vận tốc tensor, biến thiên joint states và nâng/hạ ankle link thực tế từ PhysX, không animation.

### Chuỗi GUI đạt: 20 giây mô phỏng

Log: `logs\locomotion_20261105_190355.log`. Kết quả giữ riêng tại `logs\locomotion_gui_walking_turns_pass.json`. 1.000 inference, **4.000 physics steps**, callback dt≈0,005 s, không ngã. Exit 0, không Error/Fatal; còn warning startup/prototype material như bước trước.

Hồi quy headless cuối sau khi thêm kiểm tra runtime PD gains: `logs\locomotion_20261105_190519.log`, exit 0, 0 Error/Fatal, 5 Warning, cả 6 phase đạt, `runtime_pd_gains_verified=true`. Kết quả latest ở `logs\locomotion_result.json`, run directory `logs\locomotion_runs\20261105_190519`. Metrics chuyển động trùng lần GUI trong bảng bên dưới.

| Bài test | Command vx,vy,yaw | Kết quả đo |
|---|---|---|
| Đứng yên 3 s | 0,0,0 | Tốc độ cuối trung bình ≈0,002 m/s, max tilt ≈0,94° |
| Đi tiến 6 s | 0,35;0;0 | Đi ≈1,87 m, vx trung bình nửa sau ≈0,329 m/s; ankle nâng/hạ ≈3,2/3,4 cm |
| Dừng sau tiến 2 s | 0,0,0 | Tốc độ nửa sau ≈0,018 m/s |
| Quay trái khi đi 3 s | 0,35;0;0,2 | Đổi yaw ≈+0,408 rad; yaw rate nửa sau ≈+0,142 rad/s |
| Quay phải khi đi 3 s | 0,35;0;-0,2 | Đổi yaw ≈−0,392 rad; yaw rate nửa sau ≈−0,138 rad/s |
| Dừng cuối 3 s | 0,0,0 | Tốc độ nửa sau ≈0,0053 m/s |

Một probe độc lập command 0,5 m/s cho quãng đường ≈1,81 m/4 s, vx nửa sau ≈0,489 m/s, ankle nâng/hạ ≈4 cm. Kết quả tại `logs\locomotion_forward_probe_pass.json`, không thay cho chuỗi đầy đủ.

**Quay tại chỗ chưa đạt.** Đã chạy vx=0 với yaw ±0,15 và ±0,2 trong 3 giây mỗi phía: yaw thay đổi chỉ khoảng 0,01 rad, không đủ tiêu chí. Giữ kết quả này tại `logs\locomotion_in_place_turns_result.json` và run directory cũ. Không tăng command vượt range, không tự thêm vx trong controller để giả pure-yaw thành công. Main test hiện chọn **walking arc** bằng command vx=0,35 được ghi rõ trong config; tùy chọn `-InPlaceTurns` chạy lại chẩn đoán pure-yaw và có thể trả failure.

Command vx=0,2 m/s ban đầu cũng chỉ làm robot dịch nhẹ/đổi tư thế, chưa bước rõ. Mức 0,35 m/s đã được kiểm thử, không khẳng định dead zone cố định cho mọi scene. Không nhận toàn bộ command envelope đã được chứng nhận.

### Telemetry và tiêu chí

Mỗi run nằm trong `logs\locomotion_runs\<timestamp>`:

- `telemetry.jsonl`: mỗi inference có simulation time, command, base pose/quaternion, world/body velocity, joint names/q/dq, foot link positions, tilt, observation 480 chiều, raw action, q target, inference time.
- `result.json`: metrics từng phase, physics step counts, pass/fail và exception nếu có.
- `initial.png`, `forward.png`, `turn_left.png`, ... khi bật capture: viewport Isaac Sim thực tế.

Report latest: `logs\locomotion_result.json`; log summary: `logs\locomotion_run_summary.json`. Main acceptance: không ngã, nghiêng <45°, base >0,5 m; đi tiến vx>0,05, quãng đường >0,25 m và cả hai ankle có biến thiên cao >1,5 cm; quay đúng dấu với |delta_yaw|>0,15 và |yaw_rate|>0,03; dừng có tốc độ trung bình nửa sau <0,15 m/s. Tiêu chí giữ nguyên khi test chưa đạt, không hạ ngưỡng để đổi failure thành success.

Các lần failed ban đầu được giữ riêng. Một lần dừng vì lỗi so sánh biên float32 ±0,2 với float64; đã sửa kiểm tra range và thêm unit check biên. Wrapper kiểm tra trạng thái report lẫn log và exit code, vì fastShutdown của Kit có thể trả 0 sau exception.

## Chạy lại / đặt checkpoint

Checkpoint đã đặt đúng chỗ. Để kiểm tra hash hoặc tải lại file bị thiếu:

```powershell
Set-Location -LiteralPath 'D:\unitree_g1'
& 'D:\unitree_g1\scripts\download_locomotion_policy.ps1'
```

Script ghim commit/hash, bỏ qua file hợp lệ và dừng nếu file có sẵn khác hash. Không cần tài khoản/token cho các file công khai đã tải. Giữ `deploy.json`, `deploy.yaml`, `policy.onnx` cùng bộ; không thay checkpoint khác rồi dùng nguyên contract này.

```powershell
# Chuỗi đứng/đi/quay khi đang đi/dừng; tự đóng khi hoàn tất
& 'D:\unitree_g1\scripts\run_locomotion.ps1' -Gui

# Headless, đầy đủ telemetry, bỏ ảnh để chạy kiểm tra nhanh hơn
& 'D:\unitree_g1\scripts\run_locomotion.ps1' -NoCapture

# Chỉ bài đứng yên
& 'D:\unitree_g1\scripts\run_locomotion.ps1' -Gui -StandOnly

# Chẩn đoán quay tại chỗ, hiện biết chưa đạt
& 'D:\unitree_g1\scripts\run_locomotion.ps1' -Gui -InPlaceTurns
```

File `assets\robot\g1_locomotion_flat.usd` chỉ lưu setup physics/tư thế đầu; **mở USD riêng không chạy ONNX**. Phải dùng script để policy cập nhật 50 Hz. Config `configs\locomotion.yaml` dùng JSON subset hợp lệ của YAML 1.2. Chưa gắn bàn phím hoặc ghép nhà xưởng trong bước này.

RTX 5070 12 GB đã chạy test này nhưng chưa đo peak VRAM hoặc benchmark FPS. Chưa kiểm thử robot trong nhà xưởng, vật cản, trượt sàn hoặc push recovery; không suy ra đã đáp ứng mọi yêu cầu locomotion.
