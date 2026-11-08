# Run and rollback
From PowerShell in D:\unitree_g1:
```powershell
.\run_sim.bat
.\run_sim.bat -Controller locomotion
.\run_sim.bat -Controller locomotion -Flat -SelfTest -Headless
.\run_sim.bat -Controller locomotion -SelfTest -Headless
```
Default preserves factory, Mjlab ONNX, keyboard and both viewports. W/Up forward, S/Down reverse, A/Left and D/Right pure yaw, Q/E lateral, SPACE zero command + Follow, R controlled reset. Free camera remains available.

```powershell
.\run_sim.bat -Controller sonic
```
Currently EXPECTED TO REJECT before SimulationApp: CONTROLLER_UNAVAILABLE. This is not a command for a working SONIC robot. G1 CONTROLLER MANAGER displays the same limitation and requires restart for a different backend. Restart .\run_sim.bat to return to locomotion. No dual-policy writes.

Download/verify minimal models and run offline validation with the verified Isaac interpreter:
```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\download_sonic.py --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\validate_sonic_models.py --telemetry D:\unitree_g1\logs\keyboard_runs\20261108_121312\telemetry.jsonl --no-ros-env
& C:\isaacsim\python.bat -c "import unittest; r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.discover('tests')); raise SystemExit(not r.wasSuccessful())" --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_controller_ui.py --no-ros-env
```
Use another fresh run's telemetry if the recorded local log directory is absent. Runtime logs/cache remain local; Isaac's default C: application caches are allowed, with no project copy there. Baseline JSON evidence is preserved under docs for repository review.

Branch feature/g1-sonic-controller isolates changes from main. No push was requested for this task. To return to original source, preserve/commit or stash current work before switching to main; do not use reset --hard. Operational fallback requires no Git change: run_sim.bat defaults to locomotion.

Manual acceptance checklist: launch default; check manager text and head preview; focus viewport; W/S/A/D/Q/E and arrows; release; SPACE follows robot and zeroes commands; Free/Follow/Overview; Pause/Resume requires fresh press; R safe reset; select SONIC and Apply must show restart/blocker while Locomotion RL remains current. Automated Carb input and programmatic widget tests do not replace this physical keyboard/mouse checklist.
