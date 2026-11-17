# Actual RGB-D evaluation — 2026-11-16

Completed on Windows home workstation, RTX 5070 12227 MiB, Isaac Sim 6.1.0-rc.26. Source/report: `D:\unitree_g1\logs\keyboard_runs\20261116_175328\rgbd_test_results.json`. Log: `D:\unitree_g1\logs\rgbd_camera_test_final.log`.

Real Factory V2 stage, one G1, existing ONNX policy and PhysX. GUI debug images were updated through ByteImageProvider. No Patrol started, no YOLO, no real robot commands.

| Check | Actual result |
|---|---|
| Standing | 98 frames before target tests; policy fault null |
| Target near | measured optical Z 1.398230910 m; world expected 1.398245602 m; max absolute error over 36 pixels 0.000015976 m |
| Target far | measured optical Z 2.398300529 m; world expected 2.398302501 m; max absolute error over 36 pixels 0.000002699 m |
| Rotation | Actual policy-driven yaw change 63.045 degrees; camera stays mounted, matrix max error 0.000001907 |
| Sensor disable/enable | Pass: no acquired frame count change while disabled; acquisition resumes after enable |
| Stage reset | Pass: sensor recreated and acquisition resumed with no camera error |
| Combined report | passed=true, 315 frames acquired |

Target center nominal placement 1.5/2.5 m; expected depths differ because the 0.2 m thick box's front face is nearer than its center, and camera pitches slightly with the physical torso. Comparisons use per-pixel optical rays and actual camera transforms, not nominal center distance. These are ideal simulated surfaces, not evidence of real depth sensor precision.

Stress run acquired/debugged every app loop: about 12–15 loop FPS after startup, approximately 48–62 physical steps per wall second despite configured physics 200 Hz in simulation time. Peak sampled whole-device VRAM 4087 MiB (~4.0 GiB); includes other apps, not process-only allocation. This is not evidence of real-time 200 Hz performance. Normal configuration polls at 10 Hz.

No `[Error]` or traceback in final test log. Warnings remain from robot USD/Fabric attributes, existing low-resolution Head Camera DLSS, long transform history during reset and hydratexture interfaces on shutdown. Earlier development tests had UI enum / NumPy JSON serialization errors; these were corrected before the final passing run. The initial sandbox launch did not reach startup and was terminated before the successful GPU-enabled runs.

Images and metadata: `D:\unitree_g1\logs\keyboard_runs\20261116_175328\rgbd\standing`, `target_1.5`, `target_2.5`, `rotation`. Each contains RGB/depth/metadata and a side-by-side debug image. Camera transform and intrinsics are captured with acquisition timestamps; hardware exposure synchronization was not measured.

Protected Navigation Python files, speed profile and existing locomotion modules were hash-compared against pre-task contents and unchanged. Existing unrelated Git edits were preserved. No branch, commit, push or package installation.

Launcher smoke test also completed with `scripts/run_sim.ps1 -Factory v2 -Rgbd -Headless -Duration 4`, default acquisition 10 Hz: exit code 0, zero error lines, FACTORY_FINISHED marker present, policy fault null. Log: `D:\unitree_g1\logs\factory_g1_20261116_175425.log`. This verifies the normal launcher path as well as the separate evaluation script.
