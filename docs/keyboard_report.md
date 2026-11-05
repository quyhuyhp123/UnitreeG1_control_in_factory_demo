# G1 keyboard control — kiểm thử 2026-11-05

Đã triển khai lớp bàn phím → velocity command → policy ONNX hiện có → joint targets → PhysX. Chưa thể xác nhận toàn bộ yêu cầu đã hoàn thành: quay tại chỗ chưa đạt với checkpoint hiện tại và bàn phím vật lý cần kiểm tra thủ công trên cửa sổ Windows.

## Chạy

```powershell
Set-Location -LiteralPath 'D:\unitree_g1'
& 'D:\unitree_g1\scripts\run_keyboard.ps1'
```

Lệnh mở một G1 trên sàn phẳng, không nạp nhà xưởng. Click viewport hoặc bảng **G1 Keyboard Control**. Không nhập phím khi con trỏ đang ở ô sửa thuộc tính. Đóng cửa sổ để kết thúc.

```powershell
# Kiểm tra logic, không cần Isaac Sim
python 'D:\unitree_g1\scripts\test_keyboard_contract.py'
# Kiểm thử tích hợp Carb input + ONNX + physics, headless
& 'D:\unitree_g1\scripts\run_keyboard.ps1' -SelfTest
# Cùng bài test, có cửa sổ quan sát
& 'D:\unitree_g1\scripts\run_keyboard.ps1' -SelfTest -Gui
```

Wrapper đọc đường cài đã xác minh từ configs/environment.json: Sim 6.1.0-rc.26, Python 3.12.13 tại C:\isaacsim. Không cài Isaac Lab/phần mềm mới. RTX real-time, không path tracing, một environment, không sensor. Cache Kit/Warp mặc định có thể nằm trên C:.

## Phím và vận tốc

| Phím | Lệnh |
|---|---|
| W / ↑ | vx = +0.35 m/s |
| S / ↓ | vx = -0.35 m/s |
| A / ← | yaw = +0.20 rad/s |
| D / → | yaw = -0.20 rad/s |
| Q / E | vy = +0.30 / -0.30 m/s |
| SPACE | Đưa target về 0 ngay |
| R | Yêu cầu reset an toàn |

Có thể giữ và kết hợp phím. W+↑ không nhân đôi vận tốc. W+S, ←+→, Q+E triệt tiêu trục tương ứng. Vận tốc là lệnh trong hệ thân robot, không phải vận tốc thực đo. Thả phím làm target giảm theo giới hạn cấu hình; robot vẫn có quán tính và bước giữ thăng bằng.

configs/keyboard.yaml dùng JSON hợp lệ theo YAML 1.2. Acceleration [0.6,0.5,0.5], deceleration [1.0,1.0,0.8], đơn vị m/s² cho hai trục tịnh tiến và rad/s² cho yaw. Controller kiểm tra cấu hình với command ranges trong deploy.json của policy; giá trị vượt giới hạn gây lỗi. Không tự thêm vận tốc tiến để giả quay tại chỗ.

## API và trạng thái

Carb acquire_input_interface().subscribe_to_input_events, lọc DeviceType.KEYBOARD, KEY_PRESS/KEY_RELEASE/KEY_REPEAT. Native input lắng nghe keyboard trong app và được chặn bởi focus viewport/HUD. order=0 chèn đầu danh sách callback; mapped key được xử lý trả False để chặn listener camera phía sau. Khi đóng runtime, subscription/observers được hủy; đăng ký lặp là idempotent.

Focus dùng omni.appwindow và viewport.focused/HUD.focused; sự kiện mất focus và timeline Pause/Stop xóa lệnh ngay. Resume yêu cầu nhấn mới. Phím đang giữ bị chặn đến khi thả; repeat không tích lũy. Ctrl/Alt/Super shortcut không phát lệnh di chuyển. CHAR và UNKNOWN không được suy đoán thành phím robot.

Physics 200 Hz, policy/keyboard 50 Hz, decimation 4. Giữ nguyên controllers/g1_locomotion.py, joint mapping, normalization, history và checkpoint. SHA256 checkpoint: 610c27e463a8f666aa50a06346678c00b4df3859f10b54bcc1f817c28251406f.

R đặt command=0 và chờ tốc độ tuyến tính <0.15 m/s, góc <0.5 rad/s liên tục 0.3 giây, hoặc timeline đã Pause. Reset stop timeline, khôi phục snapshot scene ban đầu, khởi tạo lại Articulation/policy và xóa observation history. Nếu khó đạt trạng thái đứng yên, dùng Pause rồi R rồi Resume. Đây là reset simulation rõ ràng, không dùng để tạo bước đi. Khi phát hiện đổ, runtime pause và yêu cầu R.

HUD hiển thị phím giữ, desired/applied command, xyz, quaternion WXYZ, policy, simulation, focus và capture. Camera đứng cố định; đi xa có thể ra ngoài khung nhìn.

## Kết quả thực tế

Nguồn test tự động là **logical keyboard của Carb InputProvider**, đi qua cùng callback/controller/policy và physics thật. Test giữ phím 3.5 giây, sau đó nhả 1 giây, reset giữa các bài độc lập trong trạng thái Pause. Đây không phải xác nhận phím vật lý Windows. Chi tiết số liệu: logs/keyboard_test_results.json; telemetry và events ở run_directory ghi trong file đó.

| Test | Kết quả lệnh | Kết quả chuyển động/kiểm tra |
|---|---|---|
| 1 W | Đạt | Tiến khoảng 0.904 m |
| 2 ↑ | Đạt | Giống W, khoảng 0.904 m |
| 3 S | Đạt | Lùi khoảng 0.845 m |
| 4 ↓ | Đạt | Giống S, khoảng 0.845 m |
| 5 A | Đạt yaw dương | **Chưa đạt quay tại chỗ**, yaw chỉ khoảng +0.0105 rad |
| 6 ← | Đạt yaw dương | **Chưa đạt**, giống A |
| 7 D | Đạt yaw âm | **Chưa đạt quay tại chỗ**, yaw chỉ khoảng -0.0049 rad |
| 8 → | Đạt yaw âm | **Chưa đạt**, giống D |
| 9 Q / E | Đạt | Ngang trái/phải khoảng +0.601 / -0.601 m |
| 10 W+A | Đạt | Tiến và quay trái qua physics |
| 11 ↑+→ | Đạt | Tiến và quay phải qua physics |
| 12 W+S | Đạt | vx command = 0; thêm ←+→ và Q+E cũng triệt tiêu |
| 13 Nhả phím | Đạt | Target giảm về [0,0,0] |
| 14 SPACE | Đạt | Target 0 ngay, không tự chạy lại từ phím giữ cũ |
| 15 R | Đạt trong Pause | Base/joints khôi phục, z≈0.78547 m; history và policy step = 0 |
| 16 Mất focus | Đạt với focus gate giả lập | Xóa command cũ; bắt buộc nhấn mới; cần xác nhận native focus thủ công |
| 17 Pause/Resume | Đạt | Không giữ command cũ sau Resume |
| 18 Subscription | Đạt | start lần hai không tạo callback trùng |
| 19 Capture mũi tên | Đạt với Carb | Listener phía sau không nhận ↑, phím X không liên quan vẫn đi qua |

Unit checks còn kiểm tra W+Q, giữ/repeat, giới hạn cấu hình và aliases. logs/keyboard_contract_result.json ghi kết quả. Không dùng test logic để kết luận robot thực sự di chuyển.

Hồi quy cuối sau sửa xử lý CHAR/UNKNOWN và vị trí HUD: run_directory logs/keyboard_runs/20261105_194331, log logs/keyboard_20261105_194320.log, kết thúc 19:44:56 +07:00, exit 0, error_lines=0. all_command_tests_passed=true, all_motion_tests_passed=false; motion limitations vẫn là test 5–8. Không diễn giải exit 0 thành mọi chuyển động đều đạt.

## File tạo/cập nhật

- controllers/keyboard_controller.py: trạng thái phím, giới hạn lệnh và Carb subscription.
- controllers/g1_keyboard_runtime.py: ghép policy/PhysX/HUD/reset và log.
- configs/keyboard.yaml: mapping, vận tốc, ramp và điều kiện reset.
- scripts/test_keyboard.py: entry point GUI và bài test tích hợp.
- scripts/test_keyboard_contract.py: test logic độc lập Isaac Sim.
- scripts/run_keyboard.ps1: launcher dùng đường cài đã xác minh.
- scripts/probe_keyboard_input.py: probe Carb phục vụ chẩn đoán, không cần để chạy thường ngày.
- assets/robot/g1_keyboard_flat.usd: scene test được runtime sinh lại.
- docs/keyboard_report.md và README.md: hướng dẫn, kết quả và giới hạn.
- logs/keyboard_test_results.json, keyboard_contract_result.json, keyboard_run_summary.json và keyboard_runs/: bằng chứng chạy thực tế; các lần thử cũ được giữ lại.

GUI native đã mở được, G1 tự giữ thăng bằng với ONNX và HUD hiển thị bình thường. Lần chạy logs/keyboard_20261105_193924.log kết thúc exit 0, không có dòng Error/Fatal/Traceback. Khi công cụ Computer Use gửi ↑, Carb nhận KEY_PRESS/KEY_RELEASE nhưng input=UNKNOWN, event_count mapped vẫn 0. Vì vậy **chưa xác nhận điều khiển bằng bàn phím vật lý**, không báo native keyboard đã đạt. Không sửa UNKNOWN thành một hướng di chuyển bất kỳ. Lần thử cũ có lỗi CHAR input đã được sửa và không phải kết quả cuối.

## Nếu mũi tên điều khiển camera hoặc không chạy

1. Click viewport/HUD, kiểm tra Focus=True, Capture=True, Simulation=PLAYING. Thả hết phím rồi nhấn lại sau Pause/Resume hoặc chuyển ứng dụng.
2. Kiểm tra HUD Keys/Desired và logs/keyboard_runs/<timestamp>/events.jsonl. Không có mapped event: kiểm tra focus/subscription và mã phím; UNKNOWN là sự cố nguồn input, chưa phải lỗi policy.
3. Có event nhưng camera vẫn xoay: listener khác có thể đăng ký trước. Tắt keyboard capture rồi bật lại; nếu extension khác vẫn giành phím, tắt shortcut điều hướng trùng trong extension đó. Test 19 chỉ chứng minh listener phía sau bị chặn, không bảo đảm mọi extension bên thứ ba.
4. Có Desired nhưng Applied chưa đạt: xem ramp, opposing keys, SPACE, pending reset và Pause. Có Applied nhưng không quay tại chỗ: đây là giới hạn policy đã đo, không đổi mapping hoặc teleport để che lỗi.
5. Cần người dùng thử giữ W/↑ vài giây trên bàn phím vật lý, nhả, đổi sang ứng dụng khác khi đang giữ và xác nhận target về 0. Không kết luận native focus an toàn trước bước này. Việc target=0 không đồng nghĩa vận tốc thân lập tức bằng 0.

Chưa ghép nhà xưởng trong bước này. Chưa đáp ứng đầy đủ tiêu chí quay tại chỗ; cần checkpoint/interface được xác minh có khả năng đó ở bước riêng, giữ policy hiện tại nguyên vẹn.
