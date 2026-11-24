# Giai đoạn 6 — Né vật cản cục bộ trong Factory V2

## Chạy trên Windows

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Map 2 -DynamicAvoidance
```

Trong Navigation, chọn Route 1 hoặc Route 2, nhập tốc độ mong muốn vào **Max speed m/s** rồi bấm START. Mặc định 0.30 m/s, trần cấu hình hiện tại 1.0 m/s. Có thể đổi `-Map 2` thành `-Map 3`. Thêm `-CostmapDebug` nếu cần xem costmap. Bỏ `-DynamicAvoidance` để chạy chế độ mặc định. Công tắc dòng lệnh này tự bật RGB-D, YOLO, định vị và costmap. Nó giữ hành vi START thủ công.

## Hành vi

- Giữ trình tự checkpoint của tuyến. Bộ điều khiển hiện có tạo lệnh bám tuyến. Nếu quỹ đạo đó bị chặn, bộ lập kế hoạch lấy mẫu tốc độ thấp hơn và hướng lệch hai bên. Khi lệnh bám tuyến an toàn trở lại, bộ điều khiển ưu tiên lệnh đó.
- Mỗi mẫu `[vx, vy, yaw_rate]` được kiểm tra trong hành lang của đoạn hiện tại, với footprint trong vùng xanh và với costmap đã inflation. Quỹ đạo gồm phần dự đoán và phần quãng phanh tính từ hiệu chuẩn hiện có. Cả vận tốc thực tế và lệnh sau bộ lọc gia tốc đều được kiểm tra. Safety Supervisor tiếp tục có quyền dừng.
- Trần lệnh dịch chuyển lấy theo mức thấp nhất giữa tốc độ đã chọn, giới hạn tuyến và speed profile. Mẫu lệnh giữ trong miền vận tốc của policy. Người đã định vị có thể khiến robot giảm tốc hoặc dừng chờ. Nhãn YOLO chưa định vị chỉ dùng để quan sát. Robot có thể tiếp tục khi costmap còn hợp lệ và vùng phanh đã được xác nhận trống. Trần 1.0 m/s là cấu hình theo yêu cầu người dùng, chưa được kiểm chứng bằng tuần tra né vật cản trong PhysX.
- Snapshot quá 0.60 giây mô phỏng hoặc 2 giây thực bị từ chối. Ô mới đi qua phải được quan sát trống trong 0.60 giây mô phỏng gần nhất. Đổi map hoặc reset yêu cầu một thế hệ quan sát mới. Hành lang hẹp, vùng mù hoặc ô trống chưa đủ bằng chứng có thể khiến robot chờ dù cảnh nhìn qua viewport có vẻ thoáng.

## Kiểm thử

| Bài | Logic | G1 tuần tra trong PhysX |
|---|---|---|
| A. Vật cản tĩnh phía trước | PASS, giảm tốc với sweep phanh | NOT RUN |
| B. Chỉ hở bên trái | PASS, chọn lệnh lệch trái trong lưới tổng hợp | NOT RUN |
| C. Chỉ hở bên phải | PASS, chọn lệnh lệch phải trong lưới tổng hợp | NOT RUN |
| D. Chặn toàn bộ | PASS, không chọn lệnh đi | NOT RUN |
| E. Nhập lại Route 1 | PASS, mẫu tiến gần đoạn Route 1 từ vị trí lệch | NOT RUN |
| F. Nhập lại Route 2 | PASS, mẫu tiến gần đoạn Route 2 từ vị trí lệch | NOT RUN |
| G. Mất dữ liệu camera | PASS, từ chối dữ liệu vắng, hết hạn hoặc Depth vô hiệu | NOT RUN |
| H. Ngã hoặc Safety Fault | PASS, Safety trả FAULT với trạng thái ngã tổng hợp | NOT RUN |

50 bài unittest về planner mới, multi-route, navigation và costmap đã qua với Python của Isaac Sim. Các bài logic E và F kiểm tra một quyết định nhập tuyến trên đúng hình học đoạn tuyến. Quá trình nhập tuyến hoàn chỉnh vẫn cần đo bằng G1.

Smoke test Map 2 chạy 12 giây thực ở chế độ headless, G1 ở ENTRY và Patrol ở MANUAL/IDLE. Isaac Sim kết thúc mã 0, không có dòng Error/Fatal trong log, có cảnh báo USD/Fabric. YOLO ONNX chạy CPU, xử lý 56 frame ở 4.90 Hz. Từ submit đến result trung bình 53.5 ms, P95 61.0 ms. Vòng lặp trung bình 22.0 FPS. VRAM toàn GPU cao nhất theo mẫu là 4916 MiB trên 12227 MiB. Phát lại snapshot ở ENTRY 100 lần cho P95 khoảng 11.4 ms riêng ở bộ lấy mẫu và chọn được lệnh có độ lớn 0.15 m/s. Giá trị phát lại này chưa qua bộ lọc lệnh, Safety Supervisor và PhysX.

Thời gian phản ứng thực tế, khoảng hở nhỏ nhất khi đi qua vật cản và quỹ đạo né thực tế đều **NOT RUN**. Với cấu hình control 50 Hz, bộ điều khiển có thể yêu cầu vận tốc bằng 0 ở lần gọi kế tiếp sau khi nhận ra lỗi hoặc dữ liệu quá hạn, tương ứng bước lịch 20 ms mô phỏng. Độ trễ từ phát hiện đến lúc G1 dừng còn phụ thuộc camera, suy luận, tốc độ mô phỏng và quãng phanh.

Bằng chứng chạy có trong `logs/factory_g1_20261124_195628.log`, `logs/costmap_runs/20261124_195643/` và `logs/keyboard_runs/20261124_195643/`. Lượt đi thật sẽ ghi vị trí, vận tốc, trạng thái và khoảng hở mô phỏng theo mẫu vào `logs/patrol_runs/<run>/telemetry.jsonl`. Điểm cuối quỹ đạo dự đoán và khoảng cách tới ô vật cản được ghi riêng trong trường `dynamic_avoidance`.
