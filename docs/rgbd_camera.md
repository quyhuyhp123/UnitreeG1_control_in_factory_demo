# G1 front RGB-D — camera stage only

Project root: `D:\unitree_g1`. Installed Isaac Sim: `C:\isaacsim`, 6.1.0-rc.26. Camera API: `isaacsim.sensors.experimental.rtx.CameraSensor` (installed extension 1.9.0). No package installation, YOLO, training, policy changes or automatic Patrol.

## Use

```powershell
Set-Location D:\unitree_g1
.\run_sim.bat -Factory v2 -Rgbd
# Sensor disabled (default):
.\run_sim.bat -Factory v2 -NoRgbd
```

Debug window: RGB on left, color depth on right; black means invalid depth. `Save RGB + depth + metadata` saves the latest acquired frame under the current `logs/keyboard_runs/<run>/rgbd/<timestamp>/`. `Disable sensor` destroys the sensor render product/annotators; `Enable sensor` recreates them. Closing only the debug window does not disable acquisition. Existing Head Camera preview remains separate.

Configuration: `configs/rgbd_camera.json`. Resolution `[width,height]=[640,360]`; API receives `(360,640)`. Capture polling defaults to 10 Hz wall time, limited by app rendering FPS. This limits CPU readback/debug work, not the independent render product's rendering frequency. Set `debug=false` to avoid image UI updates. No path tracing or additional environments.

## Mount and data conventions

Camera: `/World/G1/torso_link/RgbdCamera`. Parent: actual rigid torso link. Local eye `[0.15,0,0.52]` meters; forward robot +X, pitch down 10 degrees; local up +Z. A USD camera's viewing axis is -Z, converted to optical +Z in saved matrices. Near/far 0.1/30 m; horizontal FOV 80 degrees. No camera collider, mass or robot asset edits.

`controllers/rgbd_camera.py` supplies `latest=(rgb,depth_m,metadata)` and `poll(sim_time_s)`. RGB: uint8 H×W×3 RGB; depth: float32 H×W **distance_to_image_plane**, optical Z, not Euclidean ray range. Stage must use metersPerUnit=1; adapter rejects other scales rather than silently misconvert depth. Installed annotator returns meters in this stage; verified against world geometry. Nonfinite or <=0 is invalid.

Saved outputs: `rgb.png`, `depth_m.npy` (authoritative float meters), `depth_mm.png` (uint16 millimeters, invalid=0, saturated at 65535), `debug_rgb_depth.png`, `metadata.json`. Metadata includes K, camera world matrix, parent world matrix, optical world matrix, mount, acquisition UTC, simulation acquisition time and annotator information. Matrices use column vectors. Optical frame: X right, Y down, Z forward. Principal point uses pixel centers `(319.5,179.5)`, fx≈fy≈381.361 px.

Runtime polls immediately after synchronous `app.update`, before the next manual PhysX batch. Timestamp is **host acquisition time**, not a measured hardware exposure timestamp; native annotator info is preserved when supplied. No claim of hardware synchronization or calibrated real-camera noise. Depth is ideal simulation depth; missing geometry/material effects must be considered separately before perception control.

Runtime integration is limited to `controllers/factory_g1_runtime.py`, `scripts/run_factory_g1.py` and `scripts/run_sim.ps1`. Navigation, locomotion and speed profiles remain unchanged. Reset detaches sensor GPU resources before USD layer restoration and recreates the sensor afterwards. Installed 6.1 API has no public close; teardown is isolated in the adapter using installed SensorRuntime's `_invalidate_sensor` method, requiring recheck on Isaac upgrade.

## Repeat actual camera evaluation

Close the existing Isaac session first; launch one process:

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_rgbd_camera.py --gui --no-ros-env
```

Omit `--gui` for headless. Test runs real policy/PhysX and injects the existing test keyboard A command; robot pose is never teleported. It does not start Patrol. A temporary visual cube (not a navigation obstacle) supplies two known front faces for a ray/AABB depth comparison across 36 central pixels. Cube is removed before rotation. World truth is used only for evaluation, never as perception/controller input. Evaluation uses every-loop acquisition to stress the debug path, instead of default 10 Hz polling.

Reports and debug images are saved with each run. See final measured results in `docs/rgbd_camera_results.md`. Native physical keyboard operation is a manual check; test input uses the project's logical keyboard device.
