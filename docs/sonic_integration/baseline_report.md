# Baseline, 2026-11-08
Initial worktree clean; branch main at 77929fb97e0365b17df17e11693c5766f35d54d2. Created feature/g1-sonic-controller before changes. No reset, deletion, driver or simulator installation.

First flat self-test failed at reset assertion: missing import math in scripts/run_factory_g1.py. Fixed the import; this was a test harness error, not a policy failure.

Rerun flat: passed_automated_checks; failed tests []. Run: D:\unitree_g1\logs\keyboard_runs\20261108_121312.
Factory: passed_automated_checks; failed tests []. Run: D:\unitree_g1\logs\keyboard_runs\20261108_121530.

Actual PhysX standing, forward, reverse, left/right pure yaw, walking turns, command release and reset passed. Factory obstacle contact passed. Inputs used a Carb logical device, not physical OS keyboard. Headless viewport rendering used; not a manual GUI acceptance test.
Logs: logs/factory_g1_20261108_121256.log and logs/factory_g1_20261108_121512.log: exit 0, zero Error/Fatal/Traceback, FACTORY_FINISHED seen.

Full evidence: baseline_flat.json, baseline_factory.json and baseline_snapshot.json. Snapshot made after baseline runs; each protected file's Git blob was compared with the original HEAD blob and matched. Snapshot is not represented as a pre-run file backup.

Runtime audit: build_g1_scene authors G1 and static floor; experimental Articulation is initialized after timeline play/app update, with manual timeline advancement. G1JointTargets reads world WXYZ pose, world linear/angular velocity and DOF state; explicit named mapping. G1MjlabLocomotion creates 98-D observation and applies existing position targets. 200 Hz PhysX, 50 Hz policy, decimation 4. Keyboard uses press/release, focus guards and release reconciliation. Pause clears commands; reset stops timeline, reloads initial stage and initializes fresh policy. FactoryG1Runtime adds USD reference under /World/FactoryScene, preserves source transforms and disables redundant referenced PhysicsScene.
