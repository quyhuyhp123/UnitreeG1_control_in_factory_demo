# G1_factory_keyboard — tích hợp và kiểm thử

**Báo cáo lịch sử với policy Unitree RL Lab cũ.** Spawn/camera/policy mặc định hiện đã cập nhật: robot đứng ở cửa nhìn vào, Free Camera và policy Unitree RL Mjlab quay tại chỗ được. Xem entrance_in_place_turning.md và kết quả factory_g1_results.json mới nhất; không đọc các failure quay tại chỗ dưới đây như kết quả hiện hành.

Ngày kiểm thử: 2026-11-05. Isaac Sim cài thực tế: 6.1.0-rc.26, Python 3.12.13; Windows, RTX 5070 12227 MiB, driver 610.88. Không dùng Isaac Lab runtime, ROS, mạng robot hoặc training.

## Kiến trúc và vòng đời

scripts/run_factory_g1.py kiểm tra file/checkpoint/hash/timing trước khi tạo SimulationApp; API theo bộ cài C:\isaacsim. Bật PhysX, CPU dynamics, tắt Fabric để nhận state qua USD/tensor như các bài test trước. Một USD stage, một G1, một viewport, RTX real-time 960×640; không path tracing hay sensor.

Runtime tái sử dụng builder G1 đã kiểm chứng, rồi bỏ sàn và đèn tạm trước Play. Nạp toàn bộ default prim của factory_physics.usd bằng reference tại /World/FactoryScene, giữ nguyên hierarchy/tên/transform bên trong. Đường dẫn prim có thêm namespace reference; không đổi tên object nguồn.

Giữ /World/PhysicsScene của builder, vô hiệu hóa duy nhất prim PhysicsScene trong reference bằng override. Chỉ có một PhysicsScene hoạt động. Đây là sửa lifecycle quan trọng: phiên bản thử dùng PhysicsScene lồng trực tiếp khiến bài D ngã; cùng bài flat đạt. Sau khi giữ đường dẫn scene đã kiểm chứng, bài D đạt. Không sửa factory_physics.usd hoặc checkpoint.

Sàn factory có mặt trên Z=0, static collider và physics material được giữ. Không tạo thêm ground plane. Contact/rest offset của riêng sàn được override cấu hình 0.02/0 m để đồng bộ margin với flat setup; friction vẫn 0.8/0.6. Thử chỉ đổi offset chưa giải quyết được lỗi D; không quy kết thành công cho thay đổi đó.

Robot_Spawn_001 trong USD resolve thành /World/FactoryScene/Factory/Robot_Spawn_001, world xyz=[0,0,0]. Base được đặt cao khoảng 0.7862 m theo FK/foot geometry lúc tạo scene. Không có collider xâm phạm vùng ±1 m tại spawn trong cao độ 0–2.1 m. Chỉ hỗ trợ spawn heading 0° đã kiểm chứng; cấu hình khác bị từ chối rõ ràng.

Sau khi compose xong mới lưu snapshot reset, tạo Articulation, set physics dt, Play/app.update khởi tạo PhysX, tắt timeline auto update, rồi khởi tạo ONNX policy và keyboard. Mỗi control tick: poll event/focus → ramp velocity → đọc feedback → inference → joint target → 4 physics steps → transform readback → camera/HUD/log. Physics 200 Hz theo simulation time, policy 50 Hz; tốc độ wall time có thể thấp hơn.

Không sửa controllers/g1_locomotion.py, thứ tự joint hoặc checkpoint. ONNX CPUExecutionProvider, input [1,480], output [1,29], observation history 5 theo từng term. G1 có 29 DOF/30 bodies. Reset chỉ thao tác pose ở sự kiện reset rõ ràng, không dùng để bước đi. Điều kiện an toàn và fresh-key/focus/Pause guards giữ nguyên module keyboard.

## Kết quả tự động A–J

Chạy `scripts/run_sim.ps1 -SelfTest -Headless`. Input là logical keyboard qua Carb InputProvider đi vào cùng callback, policy và physics; không phải bàn phím vật lý Windows. Mỗi bài độc lập reset trong Pause; reset không góp vào quãng đường đo.

Kết quả cuối của bài physics factory trên D3D12: logs/keyboard_runs/20261105_224933/integration_results.json; log logs/factory_g1_20261105_224917.log. Exit 0, error_lines=0, native_stderr_diagnostics=0, FACTORY_FINISHED có mặt; status **partial_with_failed_tests**, không phải tất cả bài đều đạt. Lần Vulkan sau sửa PhysicsScene (20261105_224227) cũng đạt các bài tương tự nhưng backend đó có crash trong GUI nên không dùng mặc định.

| Bài | Kết quả thực chạy |
|---|---|
| A Mở nhà xưởng | Đạt: reference resolve 263 meshes, tên và world transforms giữ nguyên |
| B Spawn G1 | Đạt: gần [0,0,0.7838] sau khởi tạo physics; marker mặt sàn [0,0,0] |
| C Đứng | Đạt: 4 giây simulation, độ nghiêng lớn nhất khoảng 0.95°, có contact với sàn |
| D W tiến | Đạt qua Carb: +0.904 m theo X, không fault |
| E S lùi | Đạt qua Carb: -0.841 m theo X, không fault |
| F A/D tại chỗ | **Chưa đạt**: yaw chỉ +0.0104/-0.0049 rad; đúng giới hạn checkpoint đã biết |
| F thêm W+A / Up+Right | Đạt chuyển động vừa đi vừa quay; không tự thêm vx vào A/D |
| G Nhả phím | Đạt: target [0,0,0], tốc độ thân sau ổn định khoảng 0.0032 m/s |
| H Va chạm vật cản | Đạt kiểm tra contact/trajectory: 407 điểm contact G1 với Table_02_Top; base dừng gần [2.240,-0.080,0.788], không fault |
| I Reset | Đạt: base/joint states về trạng thái đầu, observation history và policy step bằng 0 |
| J FPS/VRAM/log | D3D12 ghi 67 mẫu performance; loop trung bình 36.26 Hz, physics 145.03 steps/s theo wall time; GPU tổng dùng cao nhất 4503/12227 MiB |

H xác nhận tiếp xúc vật lý và base không vượt ra ngoài sàn trong bài thử này; không phải chứng minh tuyệt đối không xuyên mọi mesh ở mọi tốc độ/góc va chạm. Robot có thể ngã khi đẩy vào vật cản; policy không có navigation/tránh chướng ngại. Supplemental collision proxies của G1 vẫn là box xấp xỉ từ bước trước.

So sánh flat D3D12 cuối: logs/factory_g1_flat_results.json, run 20261105_225113 (xem run_directory chính xác trong JSON). D tiến khoảng 0.904 m; chỉ F_left/F_right không đạt; H không áp dụng do flat không có vật cản. Các lần factory lỗi ban đầu được giữ tại logs/factory_g1_initial_results.json và factory_g1_contact_offset_results.json, không dùng chúng làm kết quả thành công.

Các mẫu FPS là vòng app.update/control/render-request theo wall time, không phải GPU presentation profiler. Self-test không pacing. VRAM là toàn GPU gồm cả ứng dụng khác, không phải riêng tiến trình Sim. Thông số 200 Hz physics/50 Hz policy trong config là simulation time, không cam kết realtime.

## GUI và camera

Lệnh `.\run_sim.bat -Duration 15` đã mở GUI, policy giữ G1 đứng, camera theo vị trí robot cho thấy đầy đủ robot trên sàn xanh/vạch vàng. Ảnh: logs/keyboard_runs/20261105_224350/factory_g1_gui.png. Camera default offset world [0,-2.7,1.7], target cao 0.65 m; camera theo vị trí, không tự xoay theo heading và chưa tránh tường che khuất. Dùng `-Overview` để chọn góc tổng quan cố định.

Vulkan không ổn định trên máy này: lần GUI đầu xuất native stderr `Unsupported: ... extractelement ...`; lần lặp lại logs/factory_g1_20261105_224502.log có Fatal, 0x80000003 và stack nvgpucomp64.dll/nvrtum64.dll. python.bat vẫn trả 0, nên wrapper dùng cả Error/Fatal và marker FACTORY_FINISHED để phát hiện crash, không chỉ exit code. PowerShell 5.1 stderr cũng được giữ lại mà không dừng wrapper trước khi ghi summary.

Đã đổi riêng runtime sang Direct3D12 bằng --/app/vulkan=false, không sửa installation/driver. Bộ cài local có sẵn backend/cache D3D12. [Issue Isaac Sim #912](https://github.com/isaac-sim/IsaacSim/issues/912) báo một lỗi Vulkan/driver khác ở Sim 5.1 và gợi ý D3D12; đây chỉ là cơ sở thử workaround, không chứng minh cùng nguyên nhân. Kết quả kiểm thử trên bộ cài 6.1 hiện tại mới là bằng chứng áp dụng được.

GUI D3D12 cuối: logs/factory_g1_20261105_224713.log, run 20261105_224853. Exit 0, error_lines=0, native_stderr_diagnostics=0, FACTORY_FINISHED có mặt và không policy fault. Khởi tạo lần đầu mất khoảng 100 giây; loop khi đã chạy khoảng 30–33 Hz, trung bình cả startup/settle khoảng 27.79 Hz; peak GPU tổng dùng 4411 MiB. Ảnh camera đã kiểm tra trực quan: logs/keyboard_runs/20261105_224853/factory_g1_gui.png, nhìn thấy đầy đủ G1 trong nhà xưởng. Không coi ảnh GUI là bằng chứng tất cả phím vật lý hoạt động; một lần smoke test cũng không bảo đảm mọi lần khởi động tương lai.

Ảnh capture headless của một số lần thử vẫn cho góc bị mái che, dù transform camera đọc lại đúng; chỉ ảnh GUI được dùng để xác nhận góc nhìn. Không suy diễn kết quả render GUI từ các ảnh headless đó.

## Checklist thủ công còn bắt buộc

- Chạy run_sim.bat, click viewport/HUD, xác nhận PLAYING, Focus=True, Capture=True.
- Giữ W rồi ↑ riêng từng lần: robot tiến; nhả: target giảm về 0 và robot ổn định. Giữ S/↓ để lùi. Theo dõi va chạm khi đi về phía bàn.
- Thử Q/E; W+A và ↑+→. A/D riêng phát yaw command nhưng policy hiện chưa quay tại chỗ đạt yêu cầu.
- Thử W+S, ←+→, Q+E: trục tương ứng phải về 0. SPACE đưa target về 0 ngay, không đồng nghĩa vận tốc thân dừng tức thì.
- Đổi sang ứng dụng khác khi giữ W: target phải về 0. Quay lại, thả rồi nhấn mới; Pause/Resume cũng phải yêu cầu nhấn lại.
- Pause rồi R: reset đúng, sau đó Resume và nhấn phím mới. Nếu đang đi, R chờ robot giảm tốc đến ngưỡng an toàn.
- Kiểm tra HUD vị trí/target/FPS/physics rate/policy, cùng telemetry và performance.jsonl của run hiện tại.

Native input chưa được xác nhận bởi automation: ở bước trước công cụ Windows gửi phím thành KeyboardInput.UNKNOWN; runtime bỏ qua UNKNOWN, không suy đoán hướng đi. Mũi tên được capture đầu danh sách callback; nếu vẫn điều khiển camera, kiểm tra Focus/Capture, event log và extension đăng ký shortcut trước đó. Chi tiết docs/keyboard_report.md.

## Lỗi và giới hạn

Missing USD/checkpoint, hash/timing sai được chặn preflight; joint mapping và dimensions được kiểm tra bởi module policy hiện có. Exception ghi logs/factory_g1_error.json với category/traceback. Launcher lưu stdout/stderr, exit code, Error/Fatal/Traceback và số native stderr diagnostics; lỗi crash ngoài Python phải xem process exit và log. Không cố ý tạo CUDA OOM/crash để thử; các nhánh phân loại đó là xử lý đã lập trình, chưa fault-inject trên GPU.

Nếu thiếu file, dùng nguồn asset/checkpoint chính thức và provenance trong docs/locomotion_report.md; không tạo checkpoint giả. Nếu không đủ VRAM, đóng app dùng GPU khác, giảm resolution trong config hoặc dùng -Flat để cô lập lỗi. Không tự nâng driver, đổi phiên bản hoặc cài lại phần mềm.

RTX 5070 12 GB dưới mức VRAM tối thiểu chính thức đã nêu trong báo cáo môi trường. Các test này vừa VRAM nhưng không chứng minh mọi nhà xưởng/cấu hình khác sẽ chạy được. Cache Kit/Warp mặc định có thể trên C:, toàn bộ project/artifact ở D:\unitree_g1.

## File tạo/cập nhật

Cập nhật camera sau kiểm thử tích hợp: configs/factory_g1.json mặc định mode=free. Runtime chỉ đặt góc nhìn lúc khởi tạo, không ghi đè camera hoặc lựa chọn camera mỗi frame trong Free. HUD có Free/Follow/Overview; chuyển sang Free giữ góc hiện tại. Reset robot giữ lại góc nhìn Free. Hướng dẫn thao tác chuột hiện hành tại README.md. scripts/test_free_camera.py kiểm tra chỉnh camera qua USD tồn tại qua quá trình robot đi bằng policy, reset và chuyển chế độ; đây không phải test kéo chuột Windows tự động. Các số liệu/ảnh camera Follow ở phần GUI phía trên là lịch sử của lần thử trước cập nhật Free.

- scripts/run_factory_g1.py: preflight, SimulationApp, loop và A–J tự động.
- controllers/factory_g1_runtime.py: USD composition, spawn, camera follow, contact reports, performance/HUD.
- configs/factory_g1.json: đường asset/checkpoint, timing, floor contact, graphics API và camera.
- scripts/run_sim.ps1 và run_sim.bat: launcher Windows một lệnh, phân tích log/marker/crash.
- controllers/g1_keyboard_runtime.py: thêm hook scene_setup và runtime_config tùy chọn trước khởi tạo physics; entry point bàn phím cũ vẫn dùng mặc định.
- README.md, docs/factory_g1_integration.md: hướng dẫn hiện hành và bằng chứng. README cũ đã backup trong logs/README_before_factory_20261105_224621.md.
- assets/robot/g1_factory_base.usd: base scene builder sinh lại; scene tích hợp review được lưu tại run_directory/factory_g1_composed.usda.
- logs/factory_g1_results.json, factory_g1_flat_results.json, factory_g1_run_summary.json, factory_g1_preflight_checks.json và các run_directory/log timestamp.

Đã trực tiếp thử preflight từ chối Missing USD, Missing checkpoint và control frequency sai; chi tiết factory_g1_preflight_checks.json. Không tạo file giả cho các bài này. SHA256 policy vẫn khớp contract đã xác minh; không cài/nâng cấp phần mềm hay sửa asset nhà xưởng gốc.

Hồi quy Flat cuối dùng đúng launcher .bat/PowerShell 5.1 đã sửa: logs/factory_g1_20261105_225057.log, exit 0, Error/Fatal=0, stderr diagnostics=0, finished marker có mặt. Loop trung bình 42.77 Hz, peak GPU tổng 4460 MiB; chỉ F_left/F_right chưa đạt. Contact point count trong H là tổng số điểm được callback báo qua nhiều physics steps, không phải số điểm tiếp xúc duy nhất.
