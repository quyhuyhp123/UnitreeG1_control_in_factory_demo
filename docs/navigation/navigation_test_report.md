# Navigation test report — 2026-11-08

Status: navigation implemented and verified for the recorded simulation scenarios. **Native desktop mouse/keyboard acceptance is BLOCKED** because the Computer Use native pipe could not connect (retry and kernel reset also failed). Programmatic GUI callbacks and actual viewport rendering passed. SONIC is still blocked and was not used.

| Test | Actual result and scope |
|---|---|
| 1 Manual regression | Fresh factory A–J self-test passed, zero failed checks; Carb logical W/S/A/D and arrow aliases, standing, release, reset, obstacle contact. Native OS keys still require checklist. |
| 2 A→B | Real Mjlab policy + PhysX, test A near [0,-4], validated B [0,-2.5]. No robot teleport during patrol. |
| 3 Return to A | Two complete A→B→A cycles passed. Final position error 0.141 m, tolerance 0.16 m plus velocity/heading settle. |
| 4 Lane boundary | Inflated footprint/grid unit tests and logged physical trajectory passed; max recorded path deviation 0.166 m. |
| 5 Static obstacle | A* detour verified on algorithm test map. Real factory full-lane collider triggered waiting; physical detour around a static obstacle was not separately demonstrated. |
| 6 Dynamic obstacle | Actual kinematic collision actor crossed; stop/wait and clear-path resume passed. Min inflated clearance 0.236 m. |
| 7 Human crossing | Semantic human collider proxy crossed; stop/wait and resume passed, min inflated clearance 0.393 m. This tests scene truth metadata, not an AI detector or animated human-mesh tracker. Existing workers remain scene objects. |
| 8 Fully blocked lane | WAITING_FOR_CLEAR_PATH, zero target, then replanning after actual collider removal passed. |
| 9 Schedule | Two physical cycles passed with shortened 2 s interval and 1 s dwell. Unit tests cover default 120 s and pause semantics; a full-duration 120 s physical waiting experiment was not run. |
| 10 Pause/resume | Final physical test and separate GUI tests passed: sim clock frozen, wall clock advances, no physics steps while paused, no automatic patrol resume. |
| 11 Manual takeover | Explicit cancellation/source selection and fresh manual command passed; AUTO ignored D/WASD contribution, SPACE paused Auto. |
| 12 Fault | Stale-data injection produced latched FAULT; fresh data did not restart, acknowledgement passed. Fault-stop ownership/budget unit tests passed. Falling robot, corrupted policy, GPU crash and every timeout permutation were not physically injected. |
| 13 Persistence | Actual JSON save/load roundtrip of A/B/map/speed/schedule passed using a log-directory test file, preserving user configuration. |
| 14 Performance | Measured table below. These are loop requests/whole-device VRAM, not presentation FPS or process-only memory. |

Unit suite: 30 total, 28 passed, 2 explicitly skipped (pre-existing blocked SONIC flat/factory tests), 0 failures, 0 errors.

| Measurement | Headless patrol | Manual regression |
|---|---:|---:|
| Mean loop FPS | 34.34 | 32.59 |
| Physics steps / wall second | 137.37 | 130.37 |
| Mean policy inference ms | 0.122 | 0.120 |
| Peak whole-GPU MiB | 6109.0 | 6179.0 |

Configured physics remains 200 Hz, policy 50 Hz. Patrol head preview was disabled for headless testing; manual/GUI production preview was enabled, so this is not a controlled renderer A/B benchmark. Patrol recorded planner maximum 0.698 ms and map-update maximum 39.52 ms. Bounding-volume/map updates add CPU cost and wall-clock real-time 200 Hz is not achieved. Cycle durations in simulation seconds: [24.200000000000003, 23.099999999999994]. Approximate sampled base travel 6.55 m, including lateral gait motion. No fall was reported in the accepted patrol run.

Measured braking: raw initial speed about 0.460 m/s, maximum post-zero base displacement 0.115 m; factor-two distance conservatism plus reaction/extra margin. One straight-floor setup is not worst-case validation for all gait phases, terrain, slopes or impacts. The initial actor test introduced a crossing too near the inflated stopping envelope and failed with FAULT; its log is retained. Accepted crossing tests use initial separation calculated from measured braking and footprint, actual moving USD/PhysX colliders, and explicit assertions—not hardcoded detector outputs.

Evidence: physics_test_results.json, obstacle_test_results.json, gui_test_results.json, manual_regression_results.json, unit_test_results.json, performance_results.json, acceptance_summary.json. Raw JSONL and images stay under the run directories recorded in those files, outside Git. Earlier heading-oscillation and sway-prediction failures and a one-frame pause assertion failure were preserved under logs/patrol_baseline; final physical suite passed all checks after fixes.

Source preservation: original factory USD, robot source USD, checkpoint, keyboard mapping/controller, joint adapter, locomotion implementation, timing configs and run_sim.bat match original Git blobs. Previous uncommitted SONIC work was carried into feature/g1-automatic-patrol and not reset or removed. Modified integration files retain that work. The generated g1_factory_base.usd test output was backed up and restored after testing; no factory regeneration, driver/software installation, sensor AI, training or robot network was used. Isaac/graphics software may use its existing default C: caches; project files stay on D:.

New code: navigation package, configs/navigation*.json and patrol_schedule.json, scripts/test_patrol*.py, summarize_patrol_tests.py and tests/test_navigation.py. Integration edits: factory_g1_runtime.py, g1_keyboard_runtime.py command hook, run_factory_g1.py Kit pacing, and README navigation entry. Existing launcher arguments and the default manual command remain valid. Code has not been committed or pushed for this task.
