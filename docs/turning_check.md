# Kiểm tra A/D và mũi tên trái/phải — 2026-11-05

**Lịch sử policy Unitree RL Lab cũ.** Mặc định hiện đã chuyển sang policy Unitree RL Mjlab có quay tại chỗ qua physics; xem entrance_in_place_turning.md. Các failure dưới đây được giữ làm bằng chứng, không áp dụng cho policy mới.

Đã chạy scripts/test_turning.py trên factory + G1 + ONNX + PhysX thật, D3D12 headless. Nguồn phím là Carb logical keyboard, không phải bàn phím vật lý. Exit 0, không Error/Fatal/Traceback. Run: logs/keyboard_runs/20261105_225522. Chi tiết logs/turning_diagnostic.json và logs/turning_diagnostic_20261105.log.

| Phím | Lệnh [vx,vy,yaw] | Thời gian giữ | Góc quay đo được | Kết luận |
|---|---|---|---|---|
| A / LEFT | [0,0,+0.2] | 6 s | +0.53° | Lệnh đúng, chưa quay tại chỗ đạt yêu cầu |
| D / RIGHT | [0,0,-0.2] | 6 s | -0.34° | Lệnh đúng, chưa quay tại chỗ đạt yêu cầu |
| W+A | [0.35,0,+0.2] | 3.5 s | +23.42° | Đi và quay trái qua physics, đạt |
| W+D | [0.35,0,-0.2] | 3.5 s | -21.34° | Đi và quay phải qua physics, đạt |

Callback nhận đúng một PRESS cho mỗi phím; desired và applied yaw đều đạt đúng dấu/giá trị. Điều này xác nhận mapping và đường truyền lệnh trong test Carb, không xác nhận phím native Windows. Nếu HUD hiện yaw ±0.2 khi người dùng giữ A/D nhưng robot không xoay, biểu hiện phù hợp với giới hạn pure-yaw đã đo. Nếu HUD không đổi thì kiểm tra Focus/Capture và events.jsonl trước khi quy lỗi cho policy.

Chưa sửa được quay tại chỗ của policy hiện tại. Không tăng yaw vượt range ±0.2, không tự thêm vx vào A/D, không sửa world pose hoặc tạo animation. Cách đã kiểm chứng để đổi hướng hiện nay là giữ W+A/W+D hoặc Up+Left/Up+Right; robot di chuyển trên một cung tròn, cần khoảng trống.

Đã cập nhật HUD: thông báo khi yêu cầu pure-yaw và hướng dẫn W+A/W+D. Cần đóng/mở lại chương trình để nạp thay đổi; không tự sửa nóng tiến trình đang chạy. Thay đổi chỉ làm rõ trạng thái, không phải sửa thành công khả năng quay tại chỗ. Logic keyboard unit tests vẫn đạt; Python compile checks đạt. Chưa kiểm tra trực quan bố cục HUD mới trong GUI.
