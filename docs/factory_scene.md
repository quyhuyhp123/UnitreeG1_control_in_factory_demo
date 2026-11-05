# Nhà xưởng Blender — 05/11/2026

## Bản thiết kế lại theo ảnh tham khảo — revision 2

File chính `blender\factory.blend` và `assets\factory\factory.usd` đã cập nhật bằng Blender MCP. Sàn xanh lá, vạch sơn vàng, cột/dầm thép lặp lại, xà gồ và mái hai dốc, đèn treo, đường ống, ống thông gió, cửa cuốn phía sau, biển khu vực, thùng/phuy/pallet. Thêm 3 công nhân low-poly đội mũ bảo hộ: hai người thao tác bàn, một người thao tác bảng máy. Các công nhân là mesh tĩnh, chưa rig hoặc animation.

Giữ sàn 10×10 m, Z=0, hành lang 2,2 m và vùng trống 2×2 m cho G1. Để tạo dáng nhà xưởng cao như ảnh, giữ panel tường dưới cao 3 m, bổ sung phần vách trên tới 4,6 m và đỉnh mái tới 5,15 m. Không sao chép asset từ đường dẫn hiển thị trong ảnh tham khảo.

Revision 2 có 263 mesh / 3.519 polygon trước triangulation. Kiểm tra mesh rỗng, scale, transform, vùng spawn và hành lang không có lỗi. Khớp người và liên kết thép/ống có giao hình học chủ ý, không khẳng định mọi cặp mesh hoàn toàn không giao nhau. USD đã import lại kiểm tra đủ mesh, có chi tiết 3 công nhân và mặt sàn Z=0; xem báo cáo JSON hiện tại. Chưa cấu hình collider hay chạy scene mới trong Isaac Sim.

Ảnh chính: `assets\factory\factory_interior.png`. Ảnh `factory_overview.png` tạm ẩn hai tấm mái để xem bố cục; mái vẫn có trong BLEND/USD. Camera trong file Blender là góc nhìn nội thất.

Script `blender\generate_factory.py` gọi `generate_factory_v2.py`, dùng builder giữ nguyên tại `generate_factory_base.py`. Cần giữ cả ba file để dựng lại. Output cũ được sao lưu trong `logs\factory_backups`; script cũ cũng được sao lưu trước thay đổi.

**Phần bên dưới mô tả revision 1, giữ lại làm lịch sử; thông số và kết quả mới nhất là revision 2 ở trên.**

Đã dựng qua Blender MCP trong Blender 5.2.2 LTS, scene mới; không xóa scene trước đó. File chính: `D:\unitree_g1\blender\factory.blend`. Script dựng lại: `D:\unitree_g1\blender\generate_factory.py`.

## Bố cục

- Lòng nhà xưởng và sàn 10×10 m, gốc giữa sàn, Z hướng lên. Sàn dày 0,2 m, mặt trên Z=0. Tường dày 0,2 m nằm ngoài biên sàn, nên kích thước phủ bì tường 10,4×10,4 m.
- Tường cao 3 m; mặt phía Y âm có cửa rộng 2 m, không ngưỡng, mở hết chiều cao. Tường thứ tư gồm hai mesh Wall_04_Left/Right.
- Hai Machine_01/02 có thân 1,5×1×1,8 m, tại X=±3,15; Y=2,8. Panel gắn mặt trước, tiếp xúc thân và không xuyên thân.
- Hai bàn Table_01/02 cao đúng 0,8 m ở X=±3,15; Y=0. Tên gốc là Empty nhóm các mesh mặt bàn/chân riêng.
- Hai Shelf_01/02 cao 2,2 m ở X=±3,15; Y=-2,8. Mỗi giá có bốn tầng, bốn trụ và hai hộp nhẹ; bộ phận là mesh riêng, nhóm bằng Empty.
- Hành lang chính X từ -1,1 đến 1,1: rộng 2,2 m; cửa là điểm hẹp 2 m. Vạch vàng nằm ngoài lòng hành lang.
- Robot_Spawn là Empty tại (0,0,0), không phải hình học hoặc collider. Vùng 2×2 m bên trong không có mesh trên sàn; đường viền đặt ngoài vùng này.
- Collections tách Architecture, Machines, Worktables, Storage, Floor_Markings, Markers, Lighting, Presentation. Không texture ngoài, không ghép toàn bộ thành một mesh.

## Kiểm tra đã làm

56 mesh, tổng 336 polygon trước triangulation. Không mesh rỗng, không scale khác (1,1,1), không transform vô hạn/NaN. Kiểm tra AABB toàn bộ cặp mesh không phát hiện giao nhau có thể tích; cho phép tiếp xúc mặt để chân bàn/kệ đặt trên sàn và chi tiết ráp với nhau. Vùng spawn không bị mesh xâm lấn, ngoại trừ chính sàn nằm dưới Z=0.

USD exporter trả FINISHED; `assets\factory\factory.usd` đã được import lại bằng Blender USD importer vào scene tạm để kiểm tra 56 mesh, kích thước sàn 10×10×0,2 m và mặt sàn Z=0. Xem `validation_report.json` và `usd_roundtrip_report.json`. Chưa mở USD này trong Isaac Sim; kiểm tra roundtrip không thay thế kiểm tra physics trong Sim.

USD hiện là asset hình học/vật liệu/ánh sáng, **chưa có static collision schemas**. Trước chạy G1 cần gán static colliders cho sàn, tường, máy, bàn và kệ trong Isaac Sim; không gán collider cho vạch sơn, camera, đèn hoặc Robot_Spawn. Không thêm robot ở bước này.

`object_manifest.json` ghi vị trí world, kích thước, bounding box, scale, loại, parent và collection. Empty nhóm bàn/kệ có origin ở gốc scene; kích thước manifest tính từ mesh con. Mesh có origin tại tâm hình học và scale đã bake trực tiếp vào vertices.

## Dựng lại

```powershell
& 'D:\Blender\blender.exe' --background --python 'D:\unitree_g1\blender\generate_factory.py'
```

Script tạo scene mới, không xóa scene cũ; sao lưu output có sẵn vào `D:\unitree_g1\logs\factory_backups\<timestamp>` trước ghi lại. Nếu USD exporter lỗi, script xuất GLB và ghi hướng dẫn dùng Isaac Sim Asset Converter sang USD trong export_status.txt. Lần chạy hiện tại USD thành công nên không cần fallback.

Ảnh `assets\factory\factory_overview.png` là render EEVEE 1400×1400 từ camera trên cao; đủ bốn tường được giữ nguyên. Các file vẫn hoàn toàn trong D:\unitree_g1.
