# Actual localization evaluation — 2026-11-16

Final run: `D:\unitree_g1\logs\keyboard_runs\20261116_181241\localization_results.json`. Log: `D:\unitree_g1\logs\localization_test_final.log`. Isaac Sim 6.1 PhysX, one Factory V2/G1, existing policy; Patrol mode MANUAL, no motion output from localization. 89 real YOLO inference results were processed concurrently. Combined checks passed; timestamp mismatch was rejected. Zero Error/traceback lines in final log. Earlier run emitted invalid-matmul warnings for infinity; filtering was corrected before the final run.

## Measured visible surface

Meters throughout. Robot coordinates are relative to pelvis at acquisition. ROI math test uses a deliberately synthetic evaluation box, **not a claimed YOLO detection**. Error is Euclidean discrepancy to known cube surface along the estimated representative viewing ray; not center-position error or a detector mAP score.

| Orientation / nominal center range | Cube center world XYZ | Estimated surface robot XYZ | Camera Euclidean range | Surface error |
|---|---|---|---:|---:|
| Initial / 1.5 m | (-7.23992,-13.81183,1.07634) | (1.52658,-0.04608,0.29797) | 1.397784 m | 0.000000226 m |
| Initial / 2.5 m | (-6.25697,-13.84791,0.89613) | (2.50815,-0.07379,0.10911) | 2.397655 m | 0.000010794 m |
| Rotated / 1.5 m | (-8.16727,-12.23473,1.07358) | (1.24112,-0.05076,0.35313) | 1.107533 m | 0.010205 m |
| Rotated / 2.5 m | (-7.67145,-11.36274,0.90864) | (2.35624,-0.04376,0.14622) | 2.241425 m | 0.008290 m |

Actual policy-driven camera yaw change: 64.8157 degrees. Cube dimensions 0.2×1.2×1.2 m, world-axis aligned; after rotation a different face is seen, so near-face distance is not nominal center distance minus 0.1 m. Median points spanning several faces may lie slightly inside the object, explaining centimeter-scale discrepancy. Do not interpret ideal simulated micrometer errors in initial views as real sensor precision.

## Objects with no YOLO class

Each of the four views produced a matching independent depth-geometry cluster, class unknown, confidence null, with surface coordinates/distance in `measurements[*].unknown_geometry`. These clusters were computed without the synthetic ROI or YOLO boxes. Actual classifier outputs remain separate in run `yolo/detections.jsonl`, including `obstacle_localization`. There is no claim that YOLO recognized the cube or that its known USD class was inferred. Full geometry results are also in run `rgbd/geometric_obstacles.jsonl`.

Images and exact K/transforms/timestamps: run `rgbd/standing_1.5`, `standing_2.5`, `rotated_1.5`, `rotated_2.5`. World-space USD bounds were used only in `scripts/test_obstacle_localization.py` for error evaluation. Production localizer accepts only depth, calibration/robot transforms and actual detection packets.

## Logic vs simulation scope

`scripts/test_localization_logic.py` passed synthetic array checks for invalid-depth rejection, nearest supported foreground band, optical-to-robot conversion, full self mask, floor mask and timestamp rejection. These logic checks are not a measured segmentation recall test. The PhysX test measured camera depth against cubes before/after real robot rotation; it did not audit all human occlusions, glass/transparent objects, moving objects, every robot self-occlusion or rough/sloped floors.

Remaining limits: host acquisition timestamps rather than measured native exposure synchronization; no instance masks/tracking; voxel merging of adjacent obstacles; floor clearance can hide small low objects; conservative self AABBs can hide very close obstacles; YOLO's low-poly person failure remains. No performance/VRAM benchmark specific to this new geometry stage was completed. Depth-only geometry currently runs in acquisition thread; no guarantee of real-time loop rate. This stage is localization only and must not be treated as validated avoidance.

Files and run instructions: `docs/obstacle_localization.md`. Existing Multi-Route Navigation, speed profile and locomotion hashes remained unchanged; no commit/push.
