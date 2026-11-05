# Blender và Isaac Sim trong dự án này

Giao diện có bảng G1 Keyboard Control và G1 Head Camera | LIVE là **Isaac Sim**, không phải Blender. Blender chỉ tạo/sửa mesh, vật liệu và bố cục nhà xưởng. Isaac Sim nạp USD, thêm collision/Articulation và chạy policy + PhysX + keyboard. Không cần Blender đang mở khi mô phỏng; chưa có đồng bộ live giữa hai ứng dụng.

## Mở và sửa mô hình bằng Blender

```powershell
& 'D:\Blender\blender.exe' 'D:\unitree_g1\blender\factory.blend'
```

Trong Blender có thể sửa máy, bàn, kệ, người và sàn; lưu factory.blend. Giữ hệ mét, scale 1, Z hướng lên, mặt sàn Z=0, tên/loại object hiện có để module collision tiếp tục nhận diện. Robot/policy trong Isaac không được tạo bởi file .blend này.

Sau khi sửa, dùng **File → Export → Universal Scene Description** để xuất vào D:\unitree_g1\assets\factory\factory.usd. Giữ root prim /Factory, đơn vị mét, vật liệu, không xuất animation/camera trình bày Blender vào asset nhà xưởng. Sao lưu USD cũ trước khi xuất. Script generate_factory.py dùng để dựng lại bố cục từ code, không dùng nó để xuất lại các chỉnh sửa thủ công vừa lưu.

Tiếp đó cần tạo lại lớp collision/physics vì factory_physics.usd là lớp tham chiếu có collider đã tính từ hình học cũ. Lệnh thủ công, chỉ chạy sau khi đã xuất USD mới:

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& 'C:\isaacsim\python.bat' 'D:\unitree_g1\scripts\load_factory.py' --validate --no-ros-env --/app/vulkan=false
```

Đường Isaac/Blender trên là đường đã kiểm tra trên máy; cấu hình nằm ở configs/environment.json. load_factory.py sao lưu lớp physics có sẵn trước khi ghi lại. Đọc logs/factory_physics_result.json và log để xác nhận validate đạt; đừng chỉ dựa vào launcher exit 0. Vị trí spawn cửa hiện dùng override trong configs/factory_g1.json, không lấy marker Blender làm vị trí cuối.

Cuối cùng đóng phiên mô phỏng cũ, mở lại:

```powershell
& 'D:\unitree_g1\run_sim.bat'
```

Thay đổi geometry chưa tự động cập nhật collider hoặc tiến trình Isaac đang chạy. Các lệnh export/rebuild trên là hướng dẫn cho lần bạn sửa Blender, không phải thao tác đã tự chạy trong lần sửa giao diện này.

## Trục tọa độ trong ảnh camera đầu

Mũi tên XYZ lớn trong ảnh là **transform gizmo** của object được chọn (ảnh người dùng đang chọn Spawn_Border_X_02_001). Nó là lớp biên tập UI của viewport Isaac, không phải vật thể thật, không phải camera nhìn thấy một trục có collision. Trục nhỏ ở góc là orientation guide, cũng là UI.

Head preview hiện tắt Prim Transform, Selection, Camera manipulators và axis/grid/selection guides riêng cho viewport này. Chọn object trong Stage không được làm gizmo xuất hiện trong head preview. Viewport chính vẫn giữ giao diện biên tập bình thường. Không xóa object hoặc sửa mesh để loại gizmo.

Đã chạy scripts/test_head_overlay.py: chọn Spawn_Border_X_01_001, head transform gizmo visible=False, main transform visible=True, ba lớp preview bị ẩn đúng. logs/head_overlay_result.json ghi passed, run 20261105_235205; exit 0, không Error/Fatal/Traceback. Đây là kiểm tra trạng thái lớp UI khi chọn object; ảnh capture render chỉ chứa ảnh scene, không dùng ảnh render để chứng minh overlay UI đã ẩn. Chưa chạy export/rebuild Blender trong lần sửa này.
