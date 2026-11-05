# Báo cáo môi trường Unitree G1 — Prompt 1

Ngày kiểm tra: 05/11/2026, khoảng 17:32–17:36 (UTC+7). Project root: `D:\unitree_g1`.

## Kết luận

**Isaac Sim đã chạy được smoke test scene trống headless. Chưa kiểm thử G1, policy, GUI hoặc nhà xưởng.** RTX 5070 12 GB dưới mức VRAM tối thiểu chính thức 16 GB của Sim 6.1. Có thể tiếp tục thử nghiệm nhỏ từng bước, nhưng chưa thể cam kết tốc độ, độ ổn định hoặc đủ VRAM cho mục tiêu cuối.

Không cài, nâng cấp hoặc gỡ phần mềm. Thư mục dự án ban đầu trống, không có AGENTS.md tại project root; không sửa repository cũ. Chưa tạo nhà xưởng, import robot hoặc tải/tạo checkpoint.

## Kết quả kiểm tra thực tế

| Thành phần | Kết quả / đường dẫn |
|---|---|
| Windows | Microsoft Windows 11 Pro 25H2, x64, build 26200.9711; xác nhận bằng CIM và log Kit |
| Lưu ý Windows | Registry ProductName vẫn ghi Windows 10 Pro; build 26200, CIM và Kit xác nhận Windows 11. Không dùng ProductName riêng lẻ để kết luận |
| CPU | Intel Core i5-14600KF, 14 cores (CIM) |
| RAM | 31,78 GiB khả dụng theo phần cứng báo cáo; tương đương máy lắp 32 GB |
| GPU | NVIDIA GeForce RTX 5070, WDDM, Blackwell sm_120 được Warp nhận diện |
| VRAM | nvidia-smi: 12.227 MiB tổng; baseline 2.191 MiB lúc kiểm tra đầu |
| Driver | NVIDIA 610.88 |
| CUDA driver | nvidia-smi báo CUDA UMD 13.3; đây không phải xác nhận đã cài CUDA Toolkit 13.3 |
| CUDA Toolkit riêng | Không tìm thấy nvcc trên PATH hoặc thư mục chuẩn NVIDIA GPU Computing Toolkit; chưa xác nhận toolkit độc lập ở vị trí tùy chỉnh |
| Python mặc định | 3.10.0, `C:\Users\Admin\AppData\Local\Programs\Python\Python310\python.exe`; không tìm thấy torch, isaacsim, isaaclab trong interpreter này |
| Python riêng | 3.13.15, `C:\Users\Admin\AppData\Local\Programs\Python\Python313\python.exe` |
| PyTorch Python 3.13 | 2.14.0+cu130, CUDA runtime 13.0; `torch.cuda.is_available()` True; nhận RTX 5070; chưa thử phép tính tensor |
| Python launcher | `C:\Windows\py.exe`; `py -0p` báo không có Python đăng ký mặc dù hai interpreter tồn tại |
| Blender | **5.2.2 LTS**, build hash d13f752e3b9c; chạy `--version` thực tế thành công |
| blender.exe | `D:\Blender\blender.exe` (không có trên PATH) |
| Isaac Sim | `C:\isaacsim`; VERSION: **6.1.0-rc.26+release.49347.2d230af4.gl**; experience khởi động ghi 6.1.0. Giữ nguyên bản đã cài; lưu ý metadata là RC |
| Isaac Sim Python | `C:\isaacsim\kit\python\python.exe`, **3.12.13** |
| PyTorch trong Sim Python | Import trực tiếp sau setup launcher báo `ModuleNotFoundError: No module named 'torch'`; môi trường hiện tại chưa sẵn sàng inference Torch theo cách này |
| Isaac Lab | Chưa tìm thấy installation/repository trong phạm vi kiểm tra; chưa có đường dẫn/version xác minh |
| Unitree RL Lab | Chưa tìm thấy installation/repository trong phạm vi kiểm tra; chưa có đường dẫn/version xác minh |
| VS Code | `C:\Users\Admin\AppData\Local\Programs\Microsoft VS Code\bin\code.cmd` có trên PATH |
| Dung lượng trống | D: khoảng 292,8 GiB; C: khoảng 43,8 GiB tại thời điểm kiểm tra; chưa xác minh loại SSD |

Phạm vi tìm kiếm: PATH, uninstall registry, thư mục chương trình và Python theo user, các vị trí Omniverse chuẩn; tìm launcher/pyvenv trong D:, C:\isaacsim và một số vị trí source/Documents/Downloads. D:\AI\unitreeG1 và D:\code\G1 có tồn tại nhưng không thấy nội dung ở cấp kiểm tra. Không coi “chưa tìm thấy” là bằng chứng tuyệt đối không tồn tại trong mọi user, WSL, ổ đĩa hoặc môi trường khác. Không sửa các thư mục đó.

## Smoke test đã thực hiện

- Script: `D:\unitree_g1\scripts\smoke_test.py`.
- Dùng Python đi kèm bản Sim đã xác minh; `PYTHONEXE` trỏ tới python.exe hiện hữu để launcher không cần copy tạo kit.exe trong installation. Dùng `--no-ros-env` ở sau script để bỏ setup ROS tự động.
- Headless, cấu hình render 640×480, stage USD mới, không robot, không sensor, không asset tải ngoài và không physics scene được script tạo. Chạy **120 app.update()**. Đây là kiểm tra khởi động/update, không phải benchmark physics hoặc frame rate.
- Log: startup complete tại khoảng **26,59 giây**, shutdown khoảng **27,25 giây** theo đồng hồ Kit. Process trả **exit code 0**.
- JSON ghi `status=stage_updates_passed`, `updates=120`. Kit dùng fastShutdown nên có thể kết thúc process ngay trong app.close(); vì vậy không dựa vào marker sau close để phán đoán. Kết luận dựa trên JSON trước close, log shutdown và exit code.
- VRAM toàn GPU quan sát riêng: 2.546 MiB lúc startup; CSV lấy mẫu khoảng 2 giây/lần ghi tối đa **2.577 MiB**, rồi về khoảng 2.169–2.180 MiB sau shutdown. Đây là đỉnh **trong mẫu**, không phải peak liên tục, không phải VRAM riêng của process; chuỗi CSV không bao phủ toàn bộ startup.
- Warp 1.16.0 nhận RTX 5070 sm_120; log Warp báo CUDA Toolkit runtime 12.9 / Driver 13.3. Không suy ra CUDA Toolkit độc lập đã cài.
- Không thấy dòng `[Error]` trong log test thành công. Có cảnh báo OpenUSD build/ABI của asset converter, deprecated pxr.Semantics và material configuration. Chưa kiểm tra asset conversion; cần lưu ý cảnh báo ABI trước bước đó.
- Thử đầu trong sandbox bị chặn CIM/executable và launcher không copy được kit.exe. Chạy kiểm tra được cấp quyền ngoài sandbox đã xác nhận Blender/Windows và chạy smoke test. Một lệnh probe đặt `--no-ros-env` trước `-c` bị Python từ chối; lần sau đặt ở cuối đúng cách. Đây là lỗi probe, không phải bằng chứng Sim không chạy.

Chứng cứ giữ tại `logs\smoke_stdout.log`, `smoke_result.json`, `smoke_exit_code.txt`, `smoke_vram.csv`, `environment_inventory.json`.

## Đối chiếu tương thích chính thức (đã tra cứu, chưa cài)

1. [Isaac Sim 6.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html): Windows 11; 32 GB RAM; tối thiểu RTX 4080 / 16 GB VRAM; driver Windows thử nghiệm 595.97. Máy dùng đúng Windows 11, RAM ở mức tối thiểu; 610.88 mới hơn phiên bản thử nghiệm nhưng không đồng nghĩa mọi tổ hợp đã được NVIDIA chứng nhận. RTX 5070 12 GB không đạt cấu hình GPU/VRAM tối thiểu niêm yết. Không đề xuất tự động thay driver hoặc phần mềm dựa trên số phiên bản.
2. [Isaac Lab compatibility table](https://github.com/isaac-sim/IsaacLab) và [hướng dẫn v3.0.0-EA](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/setup/installation/index.html): tag **v3.0.0-EA** là mốc phù hợp Sim **6.1**, Python **3.12**. Release notes EA nêu PyTorch **2.11**. Không lấy main/release branch đang di chuyển rồi mặc định vẫn hỗ trợ 6.1. Python 3.13/PyTorch 2.14 hiện có không phải stack đã xác minh cho mục tiêu này.
3. [Unitree RL Lab chính chủ](https://github.com/unitreerobotics/unitree_rl_lab) hỗ trợ G1-29dof, task `Unitree-G1-29dof-Velocity`, và có play.py cho agent đã huấn luyện. [extension.toml](https://raw.githubusercontent.com/unitreerobotics/unitree_rl_lab/main/source/unitree_rl_lab/config/extension.toml) khai báo version 0.1.0 nhưng dependencies Isaac Lab không ghim phiên bản. Vì vậy **chưa xác minh được bộ ba Unitree RL Lab + Lab 3.0 EA + Sim 6.1 trên Windows**. Tài liệu dùng shell .sh; chưa chứng minh wrapper đó chạy native PowerShell.
4. [Isaac Lab release notes](https://github.com/isaac-sim/IsaacLab/releases): 3.0 thay đổi API đáng kể, gồm quaternion và dữ liệu asset. Cần kiểm tra commit Unitree cụ thể, observation/action, thứ tự joint, PD gains, timestep và chuẩn quaternion trước inference. Không tự đổi version Sim đang dùng để né kiểm tra này.

## Các thành phần còn thiếu / chưa kiểm thử

- Isaac Lab đúng tag/commit cho Sim 6.1, môi trường Python 3.12 với Torch và RL dependencies tương ứng; chưa cài.
- Unitree RL Lab commit được kiểm thử với stack trên và cách chạy Windows; chưa clone/cài.
- Checkpoint locomotion G1 29-DOF có nguồn, giấy phép, cấu hình observation/action và joint mapping khớp. Chưa tải và chưa xác nhận checkpoint phù hợp có sẵn; thư mục checkpoints để trống.
- Asset robot, scene nhà xưởng, controller bàn phím chưa tạo đúng phạm vi Prompt 1.
- Chưa chạy NVIDIA Compatibility Checker, GUI, tải asset online, physics robot hoặc policy inference. Chưa có bằng chứng FPS/real-time của mục tiêu cuối.

## Đề xuất cho bước tiếp theo (chưa thực hiện)

Giữ Sim hiện tại. Chốt commit Lab v3.0.0-EA và đánh giá mã Unitree trước khi cài trong môi trường tách biệt thuộc D:\unitree_g1. Nếu Unitree chưa tương thích, báo rõ phương án port adapter hoặc stack khác để người dùng lựa chọn; không tự nâng/hạ Sim. Chỉ sau đó chuẩn bị checkpoint thật và chạy một G1 trên mặt phẳng. Đối với 12 GB VRAM, thử một environment, render nhỏ, vật liệu/texture nhẹ và không sensor; ghi VRAM/FPS từng bước. Diện tích 10×10 m không đủ để dự đoán VRAM: mesh, texture và renderer mới là các yếu tố quan trọng.

Project không được tạo trên C:. Installation có sẵn vẫn tại C:\isaacsim. Runtime có thể ghi log/cache mặc định trên C:, gồm `%LOCALAPPDATA%\ov`, `%LOCALAPPDATA%\NVIDIA\warp\Cache\1.16.0`, và cache/log portable trong installation; log thực tế có đường dẫn Warp trên. Không di chuyển hoặc xóa cache. Các file do dự án chủ động tạo đều nằm trong D:\unitree_g1.
