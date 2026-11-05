# Unitree G1 29-DOF — báo cáo tích hợp

Kiểm tra ngày 05/11/2026 trên Windows 11, RTX 5070 12 GB, Isaac Sim `6.1.0-rc.26+release.49347.2d230af4.gl`, Python 3.12.13 đi kèm. **Articulation và joint target đã chạy thực tế; chưa có standing balance hoặc walking policy. Robot ngã trong bài test.**

## Nguồn và tải asset

- [Unitree RL Lab chính chủ](https://github.com/unitreerobotics/unitree_rl_lab): đối chiếu `UNITREE_G1_29DOF_CFG` tại commit `4960b84732b0c2ec593dccbfe963fda1bcd7b1e3`, gồm đường dẫn USD, tư thế đầu, SDK joint order và actuator groups. Bản cấu hình tham khảo được giữ trong `assets\robot\upstream\unitree_rl_lab_robot_config.py`, giữ header bản quyền nguồn. Không clone/cài/chạy toàn bộ RL Lab hoặc Isaac Lab; chỉ phân tích cấu hình dữ liệu bằng AST.
- [Asset Unitree chính thức](https://huggingface.co/datasets/unitreerobotics/unitree_model/tree/323e350252b9c3aee9c40acbfdad84f6ce46a5ac/G1/29dof): revision `323e350252b9c3aee9c40acbfdad84f6ce46a5ac` được ghim. API xác nhận `private=false`, `gated=false`; tải HTTP công khai thành công, không cần token. Metadata không khai báo license, README không nêu giấy phép asset riêng; không suy diễn giấy phép của RL Lab sang dataset.
- Có USD 23-DOF và 29-DOF riêng. Dự án chọn **G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd** đúng theo cấu hình RL Lab. Không cần importer URDF.
- Tải đủ main USD (1.603 bytes), base (28.337.485 bytes), physics (10.366 bytes), sensor (656 bytes), cùng README. Giữ nguyên cây thư mục dưới `D:\unitree_g1\assets\robot\unitree_model`; không cắt riêng main USD vì nó dùng các layer phụ thuộc. Chọn variant `Physics=PhysX`, `Sensor=None` khi nạp, không tạo sensor robot.
- Hash SHA256, URL có revision và số bytes nằm trong `assets\robot\upstream\download_manifest.json`. Đã chạy lại `scripts\download_g1_assets.ps1` để xác nhận toàn bộ file khớp hash; nếu file cũ khác hash, script dừng để kiểm tra, không ghi đè.

## Scene và mô hình vật lý

`assets\robot\g1_flat.usd` là scene riêng, tham chiếu USD robot bằng đường dẫn tương đối. Chỉ có một G1 tự do, một sàn 6×6 m mặt trên Z=0 và một Physics Scene; thêm đèn và camera viewport để quan sát. Không ghép nhà xưởng, không ground plane thứ hai, không sensor/ROS/SDK điều khiển robot thật.

- Root: `/World/G1/pelvis`, đúng một ArticulationRootAPI.
- 30 rigid bodies, 29 revolute joints; graph nối thành một cây với một đế nổi tự do. Không fixed joint xuống world, không link kinematic.
- Tổng mass đọc từ asset: **33,341142 kg**. Tất cả link có mass dương, diagonal inertia hữu hạn/dương, thỏa bất đẳng thức tam giác; center of mass và principal axes giữ từ nguồn. Đây là xác minh dữ liệu vật lý, không phải kiểm định độ chính xác khối lượng robot thật.
- Limits vị trí lấy từ asset, kiểm tra lower < upper và tư thế đầu nằm trong giới hạn. Báo cáo JSON liệt kê limits cho từng khớp.
- Force drives lấy tham số từ cấu hình Unitree RL Lab: effort 88/139/25/5 Nm và velocity 32/20/37/22 rad/s theo actuator group; stiffness/damping và armature 0,01 ghi trong YAML/JSON. USD angular gains dùng đơn vị theo degree nên được chuyển từ Nm/rad bằng hệ số π/180; target position và max velocity đổi sang degree ở lớp USD. API runtime/adapter dùng radian.
- PhysX CPU, TGS, 120 Hz, gravity -Z 9,81 m/s²; renderer RaytracedLighting, một GPU. Không cài Torch hoặc Isaac Lab để chạy bài test này.

### Collision gốc và bổ sung

Nguồn có **33 collider**, nhiều shape là instance proxy và được ẩn khỏi render. Phải dùng TraverseInstanceProxies và BBoxCache bỏ qua visibility mới kiểm tra đúng. Gồm convex hull cùng các primitive box/sphere/cylinder, không dùng triangle mesh động phức tạp.

12 link không có collider riêng trong asset gốc: hai hip_roll, hai ankle_pitch, waist_yaw, waist_roll, hai shoulder_roll, hai elbow, hai wrist_pitch. Dự án bổ sung 12 box proxy theo local bounding box của hình học, tạo **45 collider** tổng. Các collider này được đánh dấu guide/invisible, không thay visual mesh hoặc file nguồn; kích thước và đường dẫn có trong `added_box_proxies` của báo cáo JSON.

Box bổ sung có thể giao vỏ khớp lân cận, nên lọc self-collision của riêng proxy với các rigid body trong cùng robot; chúng vẫn va chạm sàn/vật thể ngoài. Các shape gốc vẫn giữ self-collision bật và thiết lập cặp khớp từ Unitree. Proxy là xấp xỉ ban đầu, cần đánh giá lại trước ghép locomotion policy; không khẳng định đây là bộ collider đã được Unitree chứng nhận cho mọi tác vụ.

## Tư thế đầu và kết quả thực tế

Tư thế đầu theo Unitree RL Lab: hip_pitch -0,1; knee 0,3; ankle_pitch -0,2; shoulder_pitch 0,3; shoulder_roll trái/phải ±0,25; elbow 0,97; wrist_roll trái/phải ±0,15 rad, các khớp còn lại 0.

Script tính FK **một lần trước khi bắt đầu physics**, ghi transform khởi tạo link và joint state nhất quán. Tính chiều cao từ đáy collider bàn chân thay vì giả định pelvis Z=0,8 là đúng cho mọi asset. Pelvis ban đầu **Z=0,786202 m**, điểm thấp nhất mỗi bàn chân Z=0,002 m để không khởi tạo xuyên sàn và tiếp xúc khi trọng lực bắt đầu. Hướng trước robot +X, trục lên +Z, orientation ban đầu identity WXYZ=(1,0,0,0).

Không dùng set_world_pose/set_world_poses để di chuyển robot trong vòng lặp. Không reset pose liên tục, không tắt gravity, không khóa chân hoặc đế. Sau khi Play chỉ gửi joint drive targets. Camera có thể được điều chỉnh để quan sát trạng thái cuối; điều đó không thay pose robot.

Đã chạy **headless và GUI** bằng runtime thực tế:

| Kiểm tra | Kết quả |
|---|---|
| DOF | 29, không thiếu/trùng/thừa tên |
| Body graph | 30 bodies, một free-base tree |
| Collision | Mỗi body có collider sau khi bổ sung proxy |
| Initial joint state | Sai lệch lớn nhất sau lần app update đầu ≈0,0191 rad |
| Feedback | Pose đế, angular/linear velocity, q và dq đều hữu hạn |
| Thứ tự runtime | Khác SDK order; đã tạo mapping theo tên |
| Joint target | right_wrist_yaw target 0,05 rad; readback 0,0500000007 rad |
| Phản hồi chuyển động | Sau 60 updates, joint position ≈0,0508014 rad |
| Adapter input | Từ chối đủ 5 ca NaN, duplicate name, unknown name, sai chiều và ngoài limits |
| Thời gian | 360 app updates, khoảng 6,0167 giây timeline; physics 120 Hz |
| Kết quả thăng bằng | **Robot ngã**, nghiêng cuối khoảng 101,35°, pelvis Z≈0,0994 m |
| Log GUI cuối | `logs\g1_flat_20261105_184753.log`: exit 0, 0 Error/Fatal, 5 Warning |
| File nguồn | Giữ nguyên, đã kiểm tra SHA256 |

**“passed” nghĩa là bài kiểm tra cấu trúc, dữ liệu và API đạt, không có nghĩa robot đứng ổn định.** Joint PD đang giữ góc mục tiêu nhưng không phải controller cân bằng toàn thân. Ngã trong trường hợp này chưa đủ để kết luận asset lỗi. Chưa kiểm thử walking, đẩy nhiễu, ma sát chân hoặc policy RL.

Warning còn có: OpenUSD build của asset converter, deprecated Semantics, material configuration và material binding trong prototype của asset nguồn. Không có Error/Fatal ở lần đạt; chưa sửa mesh/material nguồn chỉ để xóa warning. Các lần lỗi phát triển trước được giữ lại, không coi là test đạt.

## Dữ liệu và giao diện policy

- `configs\g1_joint_indices.json`: runtime index ↔ name ↔ SDK index.
- `docs\g1_joint_indices.md`: bảng 29 tên/index đã đo khi physics chạy. Không dùng thứ tự duyệt USD hoặc SDK order thay cho runtime order.
- `logs\g1_articulation_report.json`: báo cáo máy đọc gồm joint limits/drives, bodies mass/inertia, collider paths, kết quả test, feedback đầu/cuối.
- `logs\g1_feedback.json`: mẫu phản hồi mỗi 30 app updates; q/dq theo `joint_names` runtime. Orientation **WXYZ** theo API experimental.prims bản 6.1 này; velocity đế theo **world frame**. Policy đòi body-frame hoặc XYZW phải đổi rõ ràng ở adapter policy sau này.
- `controllers\g1_joint_targets.py`: `G1JointTargets.set_joint_targets(targets_rad, joint_names)` nhận subset hoặc một mảng theo bất kỳ thứ tự tên rõ ràng; kiểm tra limits/NaN/tên rồi map sang runtime indices và gọi `Articulation.set_dof_position_targets`. Không sửa trực tiếp q/base pose.

Ví dụ bên trong process Isaac Sim sau khi Articulation đã khởi tạo:

```python
adapter = G1JointTargets(articulation, config['sdk_joint_order'])
adapter.set_joint_targets(policy_joint_positions_rad, policy_joint_names)
feedback = adapter.read_feedback()
```

Chưa giả định policy output là absolute joint target hay delta, chưa tạo/checkpoint AI. Bước policy phải chốt đúng observation layout, action scale, default pose, timestep/decimation và joint order của checkpoint thật.

## Lệnh chạy PowerShell

```powershell
Set-Location -LiteralPath 'D:\unitree_g1'
# Kiểm tra/tải lại đúng asset ghim hash (file hợp lệ có sẵn sẽ được bỏ qua)
& 'D:\unitree_g1\scripts\download_g1_assets.ps1'

# Chạy headless, chụp ảnh đầu/cuối, kiểm tra rồi tự thoát
& 'D:\unitree_g1\scripts\run_g1_flat.ps1'

# Mở GUI, chạy bài test rồi tiếp tục mô phỏng cho tới khi đóng cửa sổ
& 'D:\unitree_g1\scripts\run_g1_flat.ps1' -Gui

# GUI chạy test và tự đóng (đã thực thi để kiểm tra)
& 'D:\unitree_g1\scripts\run_g1_flat.ps1' -Gui -AutoClose

# Tùy chọn không joint PD; chưa chạy riêng chế độ này trong lần bàn giao
& 'D:\unitree_g1\scripts\run_g1_flat.ps1' -Gui -Passive
```

`configs\g1_robot.yaml` dùng cú pháp JSON — tập con hợp lệ của YAML 1.2 — để đọc bằng thư viện chuẩn và không phải cài PyYAML. Không đổi software environment cũ. Scene output có sẵn được backup trong `logs\g1_backups`; các report latest cập nhật mỗi lần chạy, bản headless đạt được giữ ở `logs\g1_headless_pass.json`.

Ảnh `logs\g1_initial_isaac.png` và `g1_final_isaac.png` là viewport Isaac Sim thực tế, không phải render Blender. Chưa đo peak VRAM hoặc FPS; giới hạn 12 GB vẫn cần theo dõi ở bước ghép robot + policy + nhà xưởng.
