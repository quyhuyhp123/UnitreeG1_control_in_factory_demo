# Actual YOLO detection results — 2026-11-16

## Outcome

**Pipeline integration and asynchronous inference passed. Person detection on the low-poly Factory V2 workers did not pass.** There is no perception-based avoidance in this stage. No training, policy modification, planner, velocity command, auto Patrol, commit or push.

### Saved-image tests

Source report: `D:\unitree_g1\logs\yolo_offline\20261116_180308\report.json`; per-image detections: `image_results.json`. Official YOLO11n pretrained, CPU, imgsz=320, conf=0.25. 130 images tested: official bus photo, 128 COCO sample images and saved G1 camera RGB.

| Target class | Best observed confidence | Source COCO sample |
|---|---:|---|
| person | 0.9634 | 000000000436.jpg |
| chair | 0.9041 | 000000000164.jpg |
| bottle | 0.9081 | 000000000283.jpg |

All three target classes were detected on real photos. This is a sample smoke test, not a validation-set mAP/recall evaluation. Offline latency median 20.62 ms, p95 25.54 ms; unpaced average throughput 46.83 images/s, excluding warm-up. Actual asynchronous pipe test: 52 completed results during 10 s including startup; producer about 50 Hz, one pending slot. Submission p95 0.2705 ms, maximum 0.8228 ms; 429 superseded frames dropped. Enable/disable passed. Confidence and bboxes came from neural inference, not reference annotations.

Screenshots: `logs/yolo_offline/20261116_180308/person_detections.jpg`, `chair_detections.jpg`, `bottle_detections.jpg`; copies under `docs/yolo_screenshots`.

### Live G1 RGB-D camera

Report: `D:\unitree_g1\logs\keyboard_runs\20261116_180436\yolo_live_results.json`. Logs: `logs/yolo_live_test.log`, run `yolo/detections.jsonl` and `worker_stderr.log`. Camera: 640×360, poll 10 Hz. YOLO target 6 Hz, CPU threads 2, imgsz320. Actual G1 stood and rotated through the existing policy/PhysX. GUI RGB-D and YOLO windows were active.

| Measurement | Actual |
|---|---:|
| Inference results in measured interval | 91 |
| Inference rate | 5.786 Hz |
| Full prediction median / p95 | 24.37 / 26.16 ms |
| IPC roundtrip p95 | 49.99 ms |
| Frame acquisition-to-result age p95 | 167.65 ms |
| Last measured app loop FPS | 17.96 |
| Whole-GPU peak sampled memory | 4126 MiB (~4.03 GiB) |
| Policy fault | null |
| Enable/disable | passed |

GPU usage includes Isaac rendering and other applications; YOLO uses CPU tensors and allocates no inference CUDA memory. No process-only VRAM delta or matched baseline was measured. App loop FPS is not GPU present-time profiling or proof of real-time 200 Hz PhysX.

**Live detections need improvement:** the only output class was `sports ball`, visibly a false positive on the yellow head of a block-style worker (confidence about 0.50). The workers were not detected as person at the tested settings, even though visible. Chair and bottle were not detected in the observed views; these are not established false negatives without confirming their presence. We did not replace these missing detections with USD labels, lower thresholds until fake success, add photographic billboards, or train the network.

Live screenshots: run `yolo/first_sports_ball/detections.png` documents the false positive; `yolo/rotated/detections.png` documents no person boxes despite visible workers. Copies in `docs/yolo_screenshots`. RGB/depth snapshots remain in run `rgbd/yolo_standing` and `rgbd/yolo_rotated`.

No error/traceback in the successful live test. Existing USD/Fabric/head-preview DLSS warnings remain. Main launcher smoke test: `scripts/run_sim.ps1 -Factory v2 -Yolo -Headless -Duration 6`, log `logs/factory_g1_20261116_180559.log`, exit0, zero Error lines, finished marker present. No automatic Patrol.

## Files changed in this stage

Created: `configs/yolo_detection.json`, `configs/yolo_environment_snapshot.json`, `controllers/yolo_detection.py`, `controllers/yolo_worker.py`, `scripts/setup_yolo.ps1`, `scripts/prepare_yolo_assets.py`, `scripts/test_yolo_offline.py`, `scripts/test_yolo_live.py`, `checkpoints/yolo/source_manifest.json`, this report, `docs/yolo_detection.md`, `docs/yolo_screenshots/*`. Downloaded local checkpoint and `.venv/yolo` are excluded from Git.

Modified: `controllers/rgbd_camera.py` (Unix acquisition timestamp only), `controllers/factory_g1_runtime.py` (submit new RGB, UI update/cleanup and invalidate results at reset), `scripts/run_factory_g1.py`, `scripts/run_sim.ps1` (optional YOLO flags), README and `.gitignore`. Navigation Python files, speed profile and locomotion modules were hash-compared with pre-stage contents and unchanged; all earlier user changes preserved.

Next work remains a separate stage: first improve/validate appearance and detection quality on the simulated people, then evaluate depth association. Current results are **not ready to be relied on for person avoidance**. No subsequent stage was implemented.
