# Stage 4 — RGB-D obstacle localization

Run from PowerShell:

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Yolo
# Depth geometry without classification:
.\run_sim.bat -Factory v2 -Rgbd -NoYolo
```

`configs/obstacle_localization.json` defaults enabled. Set enabled=false before launching to disable localization. No motion commands or navigation integration. Existing detection UI still shows 2D boxes; 3D data are written to JSONL, not an avoidance overlay.

## Synchronization

RGB/depth are fetched together after synchronous app.update. Camera metadata records K, T_world_optical and **T_world_robot for pelvis** at that acquisition. YOLO queue owns a copy of that depth and metadata with its original RGB, one pending and one in-flight frame. These remain in Isaac's client process; worker receives RGB only. On response, both frame_counter and exact acquisition_utc must match; mismatches raise rather than using latest depth. Reset invalidates generation and outstanding output. UTC is host acquisition time, not a native exposure timestamp. This provides packet alignment under the verified synchronous renderer, not a claim of hardware exposure synchronization.

Robot coordinates refer to `/World/G1/pelvis`, +X forward, +Y left, +Z up. Optical coordinates: +X right, +Y down, +Z forward. For each valid pixel: X=(u-cx)Z/fx, Y=(v-cy)Z/fy. Apply T_robot_optical = inverse(T_world_robot) @ T_world_optical. Never apply the robot's current pose to an old inference result.

## Depth processing and fusion

Reject nonfinite, zero, negative and out-of-range depth (0.15–10 m). Bounding-box ROI uses central 60% and step 2 sampling; select nearest sufficiently supported 0.15 m depth band, then trim with MAD (minimum tolerance 0.025 m). This limits background bias, but **a 2D box is not an object segmentation mask**: overlapping foreground, thin objects, large depth gradients and small objects can still be wrong or rejected.

Remove floor using configured world Z=0 and 0.06 m clearance. This is calibrated floor geometry, not an AI class or obstacle ground-truth lookup. It assumes the existing flat factory floor; sloped floors/stairs and low obstacles below clearance are limitations. Self rejection uses acquisition-time AABBs of G1 rigid links plus 0.025 m padding. It uses robot kinematic geometry only, no factory semantics. Boxes overapproximate link shape, so near-body obstacles may be removed; very fine geometry or rendering latency can leave residual self pixels. Not a complete articulated mesh ray-casting self-filter.

Geometric branch samples every 8 pixels, voxelizes robot-frame points at 0.2 m and groups six-connected voxels. Components need >=6 samples. No YOLO class needed: source=depth_geometry, class=unknown, confidence=null. Adjacent objects/walls can merge; thin/small or distant obstacles may be missed. This is visible-surface clustering, not object tracking or a complete occupancy map.

Semantic output retains actual YOLO class/confidence, even if classifier is wrong. Fusion attaches nearby semantic candidates (<0.5 m) without deleting unknown geometry. This simple spatial association can be ambiguous, does not replace geometric bounds and is not a validated tracker. No semantic labels from USD enter detection output.

Outputs include class/confidence, position_camera_optical_m, position_robot_m, position_world_m, optical_z_m, distance_camera_m, nearest_surface_distance_m, timestamp/frame, point_count and source. **Position is median visible surface, not physical object center.** Camera distance is Euclidean norm to that representative surface; optical Z is reported separately. Nearest surface distance is minimum accepted sample range, not a guaranteed collision clearance.

`rgbd/geometric_obstacles.jsonl` receives depth-only output even if YOLO is off. `yolo/detections.jsonl` contains obstacle_localization with semantic_obstacles and geometric_obstacles for the same retained frame. YOLO Save includes those fields. Full depth/transforms remain in RGB-D Save metadata; raw native annotator information remains preserved. Geometry runs during acquisition; semantic fusion runs in background inference thread. No new packages.

## Verification

```powershell
.\.venv\yolo\Scripts\python.exe .\scripts\test_localization_logic.py
# Close any running Isaac session first:
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_obstacle_localization.py --no-ros-env
```

Logic tests cover foreground/background separation, invalid pixels, floor/self masks, coordinate transform and timestamp mismatch. PhysX test evaluates known cubes at two distances before/after real policy-driven rotation. The small synthetic ROI used to isolate projection math is explicitly labeled evaluation_roi_not_AI, never claimed as a YOLO detection. Actual YOLO inference runs simultaneously and logs independently. USD bounds and cube placement are used only inside this evaluation script, not localization modules. Ideal simulated depth is not evidence of real RGB-D sensor accuracy; stage 3's low-poly person detection failure remains unresolved.

Created: controllers/obstacle_localization.py, configs/obstacle_localization.json, scripts/test_localization_logic.py, scripts/test_obstacle_localization.py, this guide and measured results report. Modified: controllers/rgbd_camera.py (robot/self transforms and depth geometry), controllers/yolo_detection.py (retain matching depth and fuse in worker thread), controllers/factory_g1_runtime.py (pass new frame depth), README. Multi-Route Navigation, Speed Profile and locomotion were not edited. No commit/push or automatic Patrol.
