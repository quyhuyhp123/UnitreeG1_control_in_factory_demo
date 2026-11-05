# Spawn tại cửa và quay tại chỗ — cấu hình hiện hành

Ngày kiểm thử 2026-11-05. Đã đổi policy mặc định sau khi port và kiểm thử thật trên Isaac Sim 6.1/PhysX. Không training, không sửa world pose để tạo chuyển động, không thêm vx ẩn khi A/D.

## Vị trí xuất phát

configs/factory_g1.json: robot_spawn.position_override_m=[0,-4,0], heading_deg=90. Mặt robot +X local được đặt hướng +Y world, nhìn từ cửa phía Y âm vào nhà xưởng. Base cao theo FK/foot geometry khoảng 0.7862 m, điểm sàn ở z=0. Rotation được đặt một lần ở parent /World/G1 trước Play, toàn bộ articulation giữ nguyên quan hệ khớp; reset khôi phục cùng transform. Không cập nhật pose robot trong loop.

Marker Blender gốc ở [0,0,0] được giữ nguyên; runtime override có chủ đích ưu tiên vị trí cửa mới. Kiểm tra không có collider trong vùng spawn. Đã thêm EntranceApron 3×2 m, y=[-7,-5], top Z=0, static box collider, nối mép sàn cũ tại y=-5, không chồng lên sàn nhà xưởng. Bài lùi đầu tiên ngã vì bước ra khỏi mép sàn; kết quả đó lưu tại logs/factory_g1_mjlab_before_entrance_apron.json. Sau thêm apron, lùi và dừng đạt.

Camera vẫn Free; góc đầu từ ngoài cửa nhìn vào. WASD/mũi tên/Q/E dành cho G1; mouse navigation không bị runtime bám theo robot. Có các nút Free/Follow/Overview.

## Policy mới và giao diện

Nguồn: [Unitree RL Mjlab chính thức](https://github.com/unitreerobotics/unitree_rl_mjlab/tree/1425b15f73bd4095f0df53709d7c389c3eb9e790), commit 1425b15f73bd4095f0df53709d7c389c3eb9e790. Policy gốc dùng MuJoCo; không giả định chạy trực tiếp trên Isaac. Đã tạo adapter riêng controllers/g1_mjlab_locomotion.py cho feedback và joint-target API PhysX hiện có.

Checkpoint: checkpoints/unitree_g1_mjlab_velocity_v0/policy.onnx, 878421 bytes; SHA256 2a66ca6336eadb3c0b34b557763f3e06d01ff8fcf6260dd4cedbd69d6093fc28. File deploy.yaml/json, mã observations.h, observation_manager.h, velocity_env_cfg.py, g1_constants.py và XML nguồn được lưu cạnh checkpoint. provenance.json ghi commit, hash và run kiểm thử.

| Interface | Giá trị đã đối chiếu |
|---|---|
| Joint order | 29 joint theo SDK, identity joint_ids_map; so sánh đủ 29 tên và thứ tự với XML nguồn chính thức, khớp |
| ONNX | [1,98] → [1,29], CPUExecutionProvider |
| Observation | angular velocity body 3, gravity body 3, command 3, gait phase 2, relative q 29, dq 29, previous raw action 29 |
| History/scaling | history=1, scale=1 cho các term theo deploy mới; không dùng scale/history của policy cũ |
| Gait phase | period 0.6 s; cập nhật phase mỗi 0.02 s, sin/cos; đưa hai giá trị về 0 khi norm(command)<0.1; reset phase=0 |
| Action | default offset + raw action × scale từng joint trong deploy; không dùng scale 0.25 của policy cũ |
| PD/default q | lấy đúng arrays stiffness/damping/default q từ deploy theo SDK order |
| Physics/control | 200 Hz / 50 Hz, decimation 4, theo simulation time |
| Command ranges | vx [-0.5,1], vy [-0.5,0.5], yaw [-1,1] theo deploy |
| Keyboard | vx ±0.35, vy ±0.3, yaw ±0.6; factory keyboard_overrides ưu tiên hơn keyboard.yaml |

Dynamics/collision dùng USD G1 chính thức và cấu hình articulation của dự án đã kiểm chứng; không khẳng định mọi thông số MuJoCo đã được tái tạo y hệt PhysX. Kiểm thử bên dưới xác nhận phạm vi lệnh đang dùng. controllers/g1_locomotion.py và checkpoint/config Unitree RL Lab cũ được giữ nguyên; entry point bàn phím cũ còn dùng profile cũ. Chương trình nhà xưởng run_sim.bat dùng profile mới configs/locomotion_mjlab.yaml.

## Kết quả thực tế

Nguồn input tự động: Carb logical keyboard, không phải phím OS Windows. Có inference ONNX và physics thật. Không animation hoặc world-pose motion.

Run tập trung: logs/keyboard_runs/20261105_231222; logs/spawn_turning_result.json, logs/spawn_turn_mjlab_candidate.log. Tất cả 7 case đạt; exit 0, không Error/Fatal/Traceback.

| Case | Kết quả |
|---|---|
| Spawn | xy≈[0,-4], yaw≈89.9996°, không blocker |
| Đứng | Không ngã; drift khoảng 1 mm trong 3 s |
| W | Tiến body khoảng 0.988 m trong 3.5 s, hướng vào nhà xưởng |
| S | Lùi body khoảng 0.950 m trong 3.5 s |
| A / LEFT | +155.82° trong 5 s; command [0,0,+0.6], drift ngang tối đa 0.098 m |
| D / RIGHT | -128.30° trong 5 s; command [0,0,-0.6], drift ngang tối đa 0.117 m |

Run tích hợp sau thêm apron: logs/keyboard_runs/20261105_231617; log logs/factory_g1_20261105_231601.log. **passed_automated_checks, failed_tests=[]**, exit 0, error_lines=0, stderr diagnostics=0, finished marker có mặt.

A/B/C mở/spawn/đứng đạt; W tiến body 0.996 m, S lùi body 0.955 m; thả phím target=0, vận tốc thân khoảng 0.048 m/s sau ổn định. A/D tại chỗ đạt; các tổ hợp vừa đi vừa quay đạt. H có 300 điểm contact được báo qua nhiều physics steps với vật cản, không fault. Reset trả base về [0,-4,0.78547], quaternion WXYZ≈[0.707107,0,0,0.707107], phase=0. Log hiệu năng: 70 mẫu, loop trung bình 44.29 Hz, physics steps/wall second≈177.14; peak tổng GPU 4529/12227 MiB. Các số này không bảo đảm realtime 200 Hz hoặc mọi góc va chạm.

Lùi vào ngoài apron hoặc đi hết sàn vẫn có thể ngã. Quay tại chỗ có dịch chuyển nhỏ của thân/chân, không giữ base XY cứng. Chưa có navigation/tránh chướng ngại. Bàn phím native Windows vẫn cần kiểm tra trực tiếp; test Carb không thay thế xác nhận focus và phím OS.

## Chạy

Đóng cửa sổ cũ rồi mở lại để nạp profile và spawn mới:

```powershell
& 'D:\unitree_g1\run_sim.bat'
```

W/↑ đi vào, S/↓ lùi, A/← quay trái tại chỗ, D/→ quay phải tại chỗ, Q/E đi ngang, SPACE zero target. R reset về cửa hướng vào; Free Camera được giữ. Không cần giữ W để A/D quay nữa. Hướng dẫn camera tại README.md.

Các file chính: configs/factory_g1.json, configs/locomotion_mjlab.yaml, controllers/g1_mjlab_locomotion.py; cập nhật lifecycle/profile dispatch trong controllers/g1_keyboard_runtime.py và controllers/factory_g1_runtime.py. scripts/test_spawn_turning.py chạy bài tập trung; scripts/run_factory_g1.py tính displacement theo body frame để test đúng khi spawn yaw=90°. Chưa cài/nâng cấp phần mềm.

Hồi quy Free Camera với profile mới: logs/free_camera_mjlab_test.log, run 20261105_232004, exit 0 và không Error/Fatal/Traceback. Camera chỉnh qua USD không bị ghi đè trong lúc robot tiến body khoảng 1.138 m, giữ góc qua reset và đổi Free/Follow đạt. Chưa thay thế được test kéo chuột và phím native Windows trực tiếp.

GUI smoke với profile mới: logs/factory_g1_20261105_231820.log, run 20261105_231836; exit 0, error_lines=0, không policy fault. Đã kiểm tra ảnh thấy G1 đứng ở cửa nhìn vào; góc camera sau đó được lùi thêm và đặt target ngay robot để tránh cắt chân.

GUI cuối sau chỉnh góc mở đầu: logs/factory_g1_20261105_232056.log, run 20261105_232111; exit 0, Error/Fatal=0, stderr diagnostics=0, finished marker có mặt. Ảnh factory_g1_gui.png đã kiểm tra trực quan: thấy đủ toàn thân G1, đứng ở cửa và nhìn vào nhà xưởng.
