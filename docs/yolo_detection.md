# YOLO11n RGB detection — stage 3

Project root `D:\unitree_g1`. Detection only: no planner, depth fusion, velocity commands, policy changes, USD semantic-label detections or training.

## Runtime

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Yolo
# Explicitly disabled, RGB-D still active:
.\run_sim.bat -Factory v2 -Rgbd -NoYolo
```

`-Yolo` enables RGB-D automatically; combining it with `-NoRgbd` is rejected. Default detection is disabled. UI **G1 YOLO11n | RGB detections only** has Enable YOLO, Disable YOLO and Save detections. Disable stops submission and clears queued/results display; the loaded worker remains idle for fast re-enable. Quit simulation to release the worker process. Re-enable does not start Patrol. Closing only the window hides debug, not inference.

Debug displays the exact RGB image belonging to the inference result, not stale boxes over a newer image. Label shows latency, object count and result age; stale results are marked after 1 second. Saving creates `logs/keyboard_runs/<run>/yolo/<timestamp>/detections.png` plus JSON. Every result is logged in `yolo/detections.jsonl`; worker errors/startup diagnostics go to `yolo/worker_stderr.log`.

Configuration: `configs/yolo_detection.json`. Default imgsz 320, confidence 0.25, IoU 0.45, target rate 6 Hz, CPU threads 2. Inference device is deliberately CPU; GPU inference has not been validated and the worker rejects other backends. Retain this backend while evaluating detection quality: it meets the requested 5–10 Hz in the measured live run without consuming inference CUDA VRAM.

## Isolation and frame exchange

Installed Isaac Sim is 6.1.0-rc.26, Python 3.12.13, with its own extension-managed NumPy/CUDA/ONNX dependencies. System Python was 3.10.0 without torch/torchvision/ultralytics. No packages were installed into either global Python or Isaac Sim.

Separate environment: `D:\unitree_g1\.venv\yolo\Scripts\python.exe` (Python 3.10.0), torch 2.7.1+cpu, torchvision 0.22.1+cpu, Ultralytics 8.3.221, NumPy 2.2.6, OpenCV 5.0.0. CUDA is None and torch.cuda.is_available() is false in this environment. `uv pip check` passed. Full actual versions: `configs/yolo_environment_snapshot.json`. Model: official COCO pretrained **yolo11n.pt**, 5,613,764 bytes, SHA256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`; download URL/hash in `checkpoints/yolo/source_manifest.json`.

Flow:

1. `RobotRgbdCamera.poll` returns a new RGB frame with frame counter and acquisition timestamps.
2. `AsyncYoloDetector.submit` copies it into a bounded, latest-only queue (one pending frame). Old pending frames are dropped, not accumulated.
3. A background IPC thread sends RGB bytes plus length-prefixed JSON metadata through stdin to the isolated Python worker. There is one request in flight. Neither pipe I/O nor inference runs in the Physics Loop.
4. `yolo_worker.py` converts RGB to BGR for Ultralytics, performs actual pretrained inference and returns JSONL through stdout. Third-party logs use stderr, so they cannot corrupt the protocol. Kit PYTHONPATH/PYTHONHOME are removed from worker environment.
5. Background thread stores the matching image/result. Main UI draws boxes; Navigation and Locomotion RL receive no detections. No socket server, ROS, robot network, semantic annotator or USD labels are involved.

Result fields: class_id/name, confidence, bbox_xyxy in original RGB pixels, acquisition UTC and simulation acquisition time, frame_counter, completion time, model/device/imgsz, inference_total_ms, native preprocessing/inference/postprocessing timing, IPC roundtrip and frame age. Empty detections are valid outputs, not invented objects. Initial model loading/warm-up is excluded from inference latency statistics. Physics may still be slower than real time due to rendering and CPU contention; asynchronous inference is not a guarantee of real-time simulation.

Reset clears pending results and invalidates in-flight results by generation; sensor lifecycle remains owned by RGB-D. Standalone YOLO has no robot pose/actuation access. Camera acquisition errors are handled by the existing RGB-D runtime; worker errors appear in its debug status/log and do not generate any stop/motion command.

## Reproduce dependencies and tests

No setup is needed on this machine after this task. To recreate the isolated environment if missing (uses existing `uv` and chosen base Python):

```powershell
cd D:\unitree_g1
.\scripts\setup_yolo.ps1
# Then offline, before live:
.\.venv\yolo\Scripts\python.exe .\scripts\test_yolo_offline.py
# Close any existing Isaac session, launch only one test process:
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_yolo_live.py --no-ros-env
```

Setup installs only into `.venv/yolo`; it does not change Isaac or install CUDA. Existing uv download cache and the initial Ultralytics import used normal user cache/settings on C:. Subsequent YOLO settings are routed to `D:\unitree_g1\logs\yolo_settings\Ultralytics`; downloaded weights, samples and project files remain on D:. Initial fallback settings were moved under `logs/yolo_initial_settings`. No training/download-dataset execution is invoked; only JPEG reference samples are extracted from the official COCO128 archive. No image classification output is substituted for detection.

Offline test evaluates official sample photos and a saved G1 RGB image, benchmarks actual IPC under 50 Hz producer load, checks bounded queue, nonblocking submit and enable/disable. Live test uses G1 standing and policy-driven A rotation in Factory V2, no world-pose teleport and no Patrol start. Pass means the integration/transport worked; it does not mean factory person recall passed. See measured results separately.

Official sources: [YOLO11 models](https://docs.ultralytics.com/models/yolo11/), [Ultralytics pinned dependency metadata](https://github.com/ultralytics/ultralytics/blob/v8.3.221/pyproject.toml), [PyTorch CPU installation pairs](https://pytorch.org/get-started/previous-versions/). Ultralytics is distributed under its stated AGPL/Enterprise licensing; model sources remain traceable in the manifest.
