# Windows compatibility audit — 2026-11-18

Project: D:\unitree_g1; branch AI_UnitreeG1_current. Base a823a61. Existing Linux support retained. No training, package/driver upgrade, auto Patrol, commit or push.

## Run on this Windows machine

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Check
.\run_sim.bat -Factory v2
# Camera + YOLO and debug images:
.\run_sim.bat -Factory v2 -YoloDebug
# Camera without YOLO:
.\run_sim.bat -Factory v2 -RgbdDebug -NoYolo
# Training UI only, does not start training:
.\Training_model\run_ui.bat
```

PowerShell launcher now forwards Check, Rgbd/RgbdDebug/NoRgbd, Yolo/YoloDebug/NoYolo, Obstacles/ObstaclesDebug/NoObstacles, Costmap/CostmapDebug/NoCostmap to the existing Python CLI. Check correctly recognizes PREFLIGHT_OK without requiring FACTORY_FINISHED. Existing Windows environment.json (C:\isaacsim) stays unchanged. Core ROOT paths already use pathlib relative to source; no Linux project path was copied onto Windows.

## Actual bugs fixed

1. Training UI previously launched bash/ml_python.sh for train/predict/export, used start_new_session and os.killpg, and only offered Linux setup. Added platform_runtime.py: Windows isolated interpreter, sanitized environment, UTF8 log decoding, CREATE_NO_WINDOW/process groups, termination of owned process tree using taskkill. Linux continues to use its original bash launcher/group handling. Windows setup button gives an environment explanation, not an unexpected installer.
2. Training environment readiness no longer claims the Linux .deps/Isaac CUDA setup is available on Windows. GUI checks the selected Windows interpreter; GPU device choices are withheld when torch.cuda.is_available() is false. Hardware nvidia-smi visibility alone does not mean PyTorch CUDA support. Current default interpreter is .venv/yolo/Scripts/python.exe (CPU torch2.7.1), overridable with G1_TRAINING_PYTHON or run_ui.ps1 -Python. No installs were made.
3. Labelme conversion used os.symlink, requiring special Windows privileges. On Windows it now copies source images to the dataset snapshot; originals remain unchanged. Linux retains symlinks. Windows conversion therefore uses more disk space.
4. YOLO startup enabled isaacsim.pip.nv, an extension absent in installed Windows Isaac6.1. That caused extension dependency resolution failure/registry sync. Windows now enables only isaacsim.pip.onnx and uses ONNX CPU by default, explicitly logging the fallback. Linux keeps its CUDA path. G1_YOLO_CUDA=1 is an opt-in unvalidated override, NOT a claim that CUDA DLLs are installed or correctly configured. No CUDA/PyTorch inside Isaac was modified.

## Verified on this machine

- 74 unit tests ran, 72 passed and 2 existing SONIC closed-loop tests intentionally skipped. Log logs/windows_unit_tests_final.log. New Windows tests cover worker start/stop and Labelme fixture conversion without symlink permission; no neural training was run.
- Actual Training_model.training_ui.App opened/updated/destroyed without invoking training. Dummy sleeping worker was launched/stopped natively. Actual predict_worker.py ran in the selected separate Windows Python and detected five objects on the provided bus sample; log logs/windows_predict_worker_test.json. This validates prediction protocol, not model quality or a training run.
- Main launcher -Check passed: logs/factory_g1_20261118_065933.log.
- Actual basic headless Factory V2 + G1/policy run passed, exit0, zero Error lines, policy_fault=null: logs/factory_g1_20261118_065633.log, logs/keyboard_runs/20261118_065701.
- Actual GUI -YoloDebug -Duration6 passed, exit0, zero Error/traceback, policy_fault=null: logs/factory_g1_20261118_065853.log, logs/keyboard_runs/20261118_065959. RGB-D output logs/rgbd_runs/20261118_065959/001_duration_end, YOLO output logs/yolo_runs/20261118_065959/001_duration_end. CPUExecutionProvider, 24/24 frames processed, 0 dropped, inference4.575Hz, mean36.29ms, p9537.57ms at640 input. Rate is measured wall time and depends on camera/render rate.

The first YOLO smoke attempt before the extension fix failed resolving isaacsim.pip.nv and was stopped; log logs/factory_g1_20261118_065747.log. Subsequent run succeeded. Existing robot USD/Fabric, head-preview DLSS and startup network/cache warnings remain. Cold GUI startup in the successful run took ~65s; no claim of instant startup or that every warning is eliminated.

## Limits / not tested

Training GPU backend on RTX5070 is NOT prepared/validated. Current separate PyTorch is CPU-only; no training was started. Existing .venv/yolo has no guaranteed ONNX export toolchain; export after real training must be checked separately. ONNX GPU inference not validated. Full patrol completion, all collision cases, native keyboard focus, obstacle/costmap GUI modes and long sessions were not rerun. This task did not change route geometry, speed, locomotion or safety logic, nor did it restore the prior branch's simplified UI. Source branch runtime behavior otherwise remains as checked out.

Modified: scripts/run_sim.ps1; perception/yolo_detector.py; Training_model/training_ui.py and labelme_to_yolo.py; README, Training_model/README.md, this guide. Added Training_model/platform_runtime.py, run_ui.ps1, run_ui.bat and tests/test_windows_compatibility.py. Original data and ignored environments remain local.
