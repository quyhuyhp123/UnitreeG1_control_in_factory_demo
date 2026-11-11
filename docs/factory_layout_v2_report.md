# Factory layout v2 — máy đồng kích thước, người đứng và bao che

Đã chỉnh theo yêu cầu tiếp theo và dựng/xuất lại thực tế bằng Blender 5.2.2 LTS. Blender MCP chưa kết nối, dùng bpy background. Không chạy Isaac Sim, không sửa factory_physics.usd, G1, checkpoint hoặc Navigation đang hoạt động.

## Bố cục và kích thước

Nhà xưởng 20 × 30 m, tường cao 4 m, sàn mặt trên Z=0. Giữ nguyên tâm từng máy trong bản trước, không đổi vị trí hai đảo, hai cụm bàn hoặc làn xanh/vạch vàng.

- **8 máy giữa:** thân cơ sở 2.40 × 2.40 × 2.15 m, bằng nhau; đã đối chiếu mesh Base trong manifest cho cả 8.
- **6 máy hai bên:** thân cơ sở 1.55 × 5.20 × 2.15 m, bằng nhau; đã đối chiếu cả 6.
- Kích thước trên là thân/base; đèn báo nâng chiều cao assembly lên 2.32 m, panel/HMI nhô ra nhẹ và được tính trong proxy.
- **14 người đứng:** mỗi máy một operator low-poly có mũ bảo hộ. Người ở máy ngoài đứng phía đầu ngắn trong dải xám; người tại máy giữa đứng trong đảo máy, tránh làn robot. Giữ 2 người ngồi tại bàn, tổng **16 người**.
- Thêm **mái kín**, giữ đủ bốn tường, thêm khung/jamb, lintel, ray và **cửa cuốn mở nâng lên** ở cửa phía tây nam. Lỗ cửa rộng 2.5 m, cao thông thủy 2.8 m. Không có collider chắn ngang lối vào.
- Spawn vẫn (-8.85,-13.75,0), yaw 0° hướng +X. Đây là marker sàn; chiều cao base G1 phải xử lý ở bước tích hợp.

Kích thước và lựa chọn bao che nằm trong `factory_dimensions.json`; từng object có tọa độ/rotation/dimensions/bounds trong `object_manifest.json`. Cửa là trạng thái mở tĩnh, chưa có animation/controller đóng mở.

## Kết quả cuối

- 429 objects, 400 mesh objects, **8,298 tam giác**, 15 material, 0 texture.
- 42 box proxies, gồm mái, cửa và người đứng mới. USD chứa proxy ẩn purpose=proxy; **chưa author PhysX collider API**.
- Mở lại blend PASS. USD/GLB nhập lại PASS; không thiếu visual object ID hoặc sai vị trí.
- USD metersPerUnit=1.0, Z-up; manifest prim path đã xác minh bằng OpenUSD cho 429 object IDs. GLB chuẩn Y-up, Blender importer khôi phục Z-up; không chứa proxy ẩn.
- Toàn bộ object có semantic metadata. Metadata chưa phải Isaac semantic label đã đăng ký.
- Grid 0.10 m, footprint radius 0.67 m: 9325 ô an toàn, tất cả liên thông với spawn; mọi marker A–E đạt footprint và kết nối. Bounds mái/lintel/cửa mở ở trên 1.9 m được bỏ qua trong kiểm tra obstacle 2D; vẫn có trong danh sách static bounds.
- Hành lang trái/trung tâm 1.627 m, phải 1.563 m, bắc 2.434 m, nam 6.057 m. Vẫn chưa đạt mục tiêu 2–2.5 m ở các làn dọc. Có kiểm tra hình học với người đứng nhưng **chưa thử locomotion/va chạm động lực**.
- Không scale bất thường/nonfinite/mesh rỗng. Không chứng nhận exhaustive triangle intersection; các bộ phận lắp ráp người/máy có vùng tiếp giáp/chồng lắp có chủ ý.
- Build và verification exit 0. Log cuối không traceback/error nghiêm trọng; còn cảnh báo deprecation Material.use_nodes từ Blender 5.2.2.
- RAM/VRAM bản thêm người/mái **NOT TESTED**; `memory_samples_initial_layout.json` là đo bản đầu trước chỉnh sửa, không dùng làm chứng minh VRAM bản mới. Không khẳng định 12GB đủ cho Isaac Sim chỉ từ render Blender.

## Source và preview

- `blender/factory_layout_v2.blend`: đầy đủ mái, tường và cửa; mặc định mái đang hiện.
- `blender/generate_factory_layout_v2.py`: script dựng lại.
- `blender/validate_factory_layout_v2.py`: mở lại source và kiểm tra USD/GLB.
- `assets/factory/factory_layout_v2/factory_layout_v2.usd` và `.glb`: đã cập nhật.
- `preview_top.png`: tạm ẩn mái để đọc bố cục.
- `preview_isometric.png`: cutaway, tạm ẩn mái và tường nam/đông để thấy máy/người.
- `preview_exterior.png`: ngoại thất có đầy đủ mái/tường/cửa.

Hai ảnh cutaway không có nghĩa là source thiếu tường/mái. Trong Blender có thể bật/tắt `Factory_Roof` ở collection Building; ảnh ngoại thất chứng minh bao che đầy đủ. Preview Workbench 1280 × 720, không Cycles/path tracing.

Bản trước khi chỉnh đã sao lưu trong `assets/factory/factory_layout_v2/backups/before_uniform_workers_*`, gồm blend, config, manifest, script và báo cáo cũ. Ảnh tham chiếu từ Google Docs vẫn nằm tại `references/factory_layout_reference_1.jpeg`; việc đổi kích thước máy là thay đổi có chủ ý theo yêu cầu mới.

Schema navigation có polygon holes, chưa được nạp trực tiếp vào code Navigation cũ. Bước sau cần import scene mới riêng, kích hoạt proxy, khai báo semantic labels, xử lý mái/ánh sáng và thử G1. Chưa tự thay môi trường đang chạy.

```powershell
& D:\Blender\blender.exe -b --python-exit-code 1 --python D:\unitree_g1\blender\generate_factory_layout_v2.py
& D:\Blender\blender.exe -b --python-exit-code 1 --python D:\unitree_g1\blender\validate_factory_layout_v2.py
```
