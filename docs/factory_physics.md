# Nạp nhà xưởng vào Isaac Sim 6.1

Module dùng installation thực tế `C:\isaacsim`, VERSION `6.1.0-rc.26+release.49347.2d230af4.gl`, Python 3.12.13 đi kèm. Không cài thêm dependencies, không sửa installation hoặc USD Blender nguồn.

## Lệnh chạy PowerShell

Kiểm tra headless, tạo/cập nhật scene physics, thả cube và tự thoát:

```powershell
Set-Location -LiteralPath 'D:\unitree_g1'
& 'D:\unitree_g1\scripts\run_factory.ps1'
```

Mở cửa sổ Isaac Sim tại camera toàn cảnh, giữ cửa sổ để quan sát:

```powershell
& 'D:\unitree_g1\scripts\run_factory.ps1' -Gui
```

Mở GUI, chạy kiểm tra rơi rồi tự đóng sau 60 lần cập nhật bổ sung:

```powershell
& 'D:\unitree_g1\scripts\run_factory.ps1' -Gui -Validate -Frames 60
```

Wrapper đọc đường dẫn installation từ `configs\environment.json`, dùng `PYTHONEXE` chỉ trong process và khôi phục sau chạy; không yêu cầu sửa PATH. `--no-ros-env` ở sau script để bỏ setup ROS. Nếu policy máy chặn ps1, xử lý theo chính sách của máy; script không thay execution policy.

## Scene được tạo

`assets\factory\factory_physics.usd` là composition layer, chứa USD reference tương đối `./factory.usd` tại `/World/Factory` trỏ tới `/Factory` nguồn. Không flatten hoặc sao chép mesh. Tên prim con và transform world từ USD nguồn giữ nguyên; prefix đường dẫn bổ sung là `/World`. Tên được Blender chuyển thành USD identifier từ trước được giữ nguyên, không đổi lại tên Blender.

- `/World/PhysicsScene`: PhysX, Z-up, đơn vị mét, gravity (0,0,-1) × 9,81 m/s²; 120 Hz, TGS, CCD. CPU dynamics và broadphase MBP cho scene tĩnh nhẹ; chỉ một nhà xưởng.
- 192 static colliders, `UsdPhysics.CollisionAPI` + `MeshCollisionAPI` approximation `boundingCube`. Không RigidBodyAPI trên factory. Các collider nằm trên mesh bộ phận, không bọc cả nhóm bàn/kệ bằng một hộp lớn.
- Nhóm bắt buộc: Floor 1, Wall 5 mesh (tường cửa tách hai), Machine 2, Table parts 10, Shelf parts 16. Bổ sung collider đơn giản cho kết cấu, ống, cửa cuốn, pallet/thùng, người và mái. Vạch sơn, chữ/biển, đèn, Empty Robot_Spawn không có collider.
- `/World/Materials/FloorPhysics`: static friction 0,8; dynamic friction 0,6; restitution 0. Đây là giá trị khởi đầu cấu hình được, chưa hiệu chỉnh cho policy G1.
- Mặt trên sàn Z=0, sàn duy nhất dày 0,2 m; không thêm ground plane. Bounding box spawn 2×2×2,1 m không giao static collider, ngoại trừ sàn bên dưới chân. Dầm/mái trên cao không chặn vùng này.
- `/World/OverviewCamera`: camera viewport thông thường, không sensor robot hoặc render product dành cho thu thập dữ liệu. Góc nhìn cao bên trong xưởng để mái không che nội thất; chỉnh eye/target trong config.
- Renderer của launcher: `RaytracedLighting`, không `PathTracing` hoặc `RealTimePathTracing`; 960×640, một GPU. Nguồn dùng vật liệu màu đơn giản, không thêm texture/sensor RTX. Mở USD thủ công trong một app khác cần tự chọn renderer tương ứng, vì renderer là cấu hình runtime của launcher.

## Cấu hình

### Sửa lỗi viewport tối — 05/11/2026

Đã tái hiện bằng ảnh chụp viewport Isaac Sim (không dùng ảnh Blender để xác nhận). Đèn trần USD từ Blender có intensity khoảng 31,83, dome 0,5; hai đèn chính ở trên mái kín. Bổ sung rig dưới `/World/Lighting`: 8 RectLight dưới mái tại Z=3,9 m, intensity 35.000 (normalize=false), dome 500. Chỉnh qua mục `lighting` trong config. Không sửa USD nguồn hoặc bật path tracing. Camera binding được ghi vào customLayerData của scene output.

Đã chụp và kiểm tra ảnh sau sửa tại `logs\factory_isaac_preview.png`: nhìn rõ sàn xanh, vạch vàng, máy và công nhân. Test physics sau sửa vẫn đạt, exit 0 và không có Error/Fatal trong log `factory_physics_20261105_183250.log`. Script `scripts\check_factory_render.py` dùng viewport sẵn có, không thêm sensor robot. Khi mở lại, dùng `factory_physics.usd` hoặc launcher `run_factory.ps1 -Gui`; mở riêng `factory.usd` nguồn sẽ không có rig bổ sung này.

Chỉnh `configs\factory_physics.json`, sau đó chạy lại wrapper để rebuild lớp physics. Gravity, Hz, friction/restitution, loại object có collider, vùng spawn, camera và cube test đều có cấu hình. Input/output phải khác nhau; output được giới hạn trong project root. Scene output có sẵn được backup dưới `logs\factory_physics_backups` trước khi thay.

## Kiểm tra thực tế

Đã chạy headless trên RTX 5070 12 GB, reopen chính file output và kiểm tra 263 mesh nguồn vẫn nạp đủ. Reference, tên/world transform, mặt sàn, số sàn, collider bắt buộc và vùng spawn đều đạt.

Cube cạnh 0,3 m, khối lượng 1 kg, thả với tâm ban đầu Z=2 m tại (0,0). Trong lần headless đạt, 480 app updates tương ứng khoảng 8 giây timeline; physics scene cấu hình 120 Hz (không đồng nhất một app update với một physics step). Tâm cube cuối Z=0,150000006 m, đúng nửa cạnh; đáy thấp nhất khoảng -8,94×10⁻⁹ m (sai số float), độ dao động 60 mẫu cuối bằng 0. Không xuyên sàn theo ngưỡng kiểm thử 0,025 m, kiểm tra điểm nghỉ trong 0,012 m.

Cube chỉ tạo runtime sau khi save scene sạch, rồi xóa; không có cube trong scene lưu và không chiếm Robot_Spawn sau kiểm thử. Không thêm G1. Bài test chứng minh va chạm sàn tại một điểm; không thay thế kiểm tra robot, ma sát trượt hoặc va chạm từng mặt máy/tường.

Chứng cứ: `logs\factory_physics_headless_pass.json`, `factory_physics_result.json`, `factory_run_summary.json`, log timestamp `factory_physics_*.log`. Wrapper kiểm tra cả exit code, error log và trạng thái JSON, vì Kit fastShutdown có thể trả 0 kể cả Python đã thất bại. Các lần thử lỗi đầu được giữ lại để truy vết: transform cube trong session layer đã che writeback PhysX về root USD; sửa sang root edit trong RAM sau khi lưu, không save cube. Không xem các lần đó là đạt.

Đã chạy thêm **GUI + Validate + Frames 60** ngày 05/11/2026 lúc 18:14 (UTC+7), chọn camera sau khi reopen scene. Lần GUI cũng đạt toàn bộ checks, exit code 0, log `logs\factory_physics_20261105_181411.log` có 0 dòng Error/Fatal. Vị trí nghỉ cube vẫn Z=0,150000006 m. GUI đã tự đóng theo Frames 60; dùng lệnh `-Gui` không có Frames để giữ cửa sổ. Không lấy CSV GPU sau shutdown làm bằng chứng VRAM khi GUI đang chạy; chưa đo peak VRAM toàn bộ phiên hoặc benchmark FPS.

Startup RaytracedLighting lần đầu chờ shader compile khoảng 114 giây; các lần cache sau nhanh hơn. Có warning startup về OpenUSD/asset converter, deprecated Semantics và material config; không đồng nghĩa không có warning khi báo không có Error. Dùng summary của lần chạy cuối để xác định log thực tế. Cache/log Kit và Warp có thể dùng đường dẫn mặc định trên C: như báo cáo môi trường; tất cả module/config/output dự án ở D:\unitree_g1.

## API đã đối chiếu

- `SimulationApp`, `SimulationManager.switch_physics_engine`, `set_device`, `enable_fabric`: kiểm tra trong mã bản cài 6.1.
- `pxr.Usd`, `UsdGeom`, `UsdPhysics`, `UsdShade`, `PhysxSchema`: reference, physics scene, collider/material.
- Timeline + `app.update()` của Kit 110 để chạy test; USD readback bật, Fabric tắt trong workflow kiểm tra này.
- Hướng dẫn local `C:\isaacsim\skills\physics-simulation\SKILL.md` và [Scene Setup Snippets 6.1](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/python_scripting/environment_setup.html).

12 GB VRAM vẫn dưới cấu hình chính thức 16 GB của Sim 6.1. Kết quả này chỉ áp dụng nhà xưởng tĩnh và một cube; chưa có kết luận cho policy G1.
