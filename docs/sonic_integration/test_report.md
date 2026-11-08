# Test report: PARTIALLY IMPLEMENTED
2026-11-08. Locomotion remains functional; SONIC is not an operational PhysX controller.

- Flat baseline: passed_automated_checks, zero failed tests; after correcting missing math import. See baseline_flat.json.
- Factory baseline: passed_automated_checks, zero failed tests, actual obstacle contacts; see baseline_factory.json.
- Unit suite: 12 discovered, 10 passed, 2 explicitly skipped (SONIC flat/factory prerequisites not met). Conversion, permutation, default/unknown selection, restart-only ownership and blocked SONIC construction tested. Delegation test uses a fake recording object only to test wrapper dispatch, not as a policy or physics evidence.
- Existing keyboard contract regression passed including lost-release reconciliation and opposing-key cancellation.
- Real official model sessions: 3 loaded. Offline planner: 3 real-state idle inference calls passed. Encoder/decoder inference not executed.
- Programmatic GUI test: ui_validation.json; visible app runtime, SONIC Apply blocked, policy object unchanged, head viewport present, camera modes callable; no physical mouse/key automation claimed. logs/sonic_audit/controller_ui.log has zero Error/Fatal/Traceback. Some existing USD/material, viewport resource and DLSS preview warnings remain; not zero-warning certification.
- Post-rejection default run_sim.bat -Duration 3 -Headless completed with zero error lines and FACTORY_FINISHED; see default_recovery.json. This short recovery run is not a performance benchmark.
- Explicit -Controller sonic rejected before SimulationApp and articulation creation. Launcher status controller_unavailable; no fabricated successful finish marker. Recovery path is restart default locomotion.

Not executed: SONIC walking/turning/collision/reset, online planner replenishment, v1.1/low-latency model inference, Linux deployment, TensorRT/CUDA inference, full A/B stability comparison, manual native keyboard/mouse acceptance. No model, driver, simulator or OS installation and no robot network I/O.

Files added: controllers/controller_interface.py, controller_selector.py, g1_sonic_controller.py (gate), sonic_joint_adapter.py, sonic_observation_adapter.py (primitives), sonic_command_adapter.py, sonic_motion_planner.py (offline idle); configs/sonic.yaml; scripts/download_sonic.py, validate_sonic_models.py, test_controller_ui.py; five tests/test_*.py; checkpoints/sonic metadata/license/provenance and local ignored ONNX; this documentation/evidence directory.
Files modified: .gitignore, README.md, scripts/run_sim.ps1, scripts/run_factory_g1.py, controllers/g1_keyboard_runtime.py (HUD extension hook), controllers/factory_g1_runtime.py (manager panel/metrics).
Preserved: run_sim.bat, factory USDs/collisions, robot source USD, configs/factory_g1.json, configs/g1_robot.yaml, configs/locomotion_mjlab.yaml, configs/keyboard.yaml, both locomotion implementations/checkpoints, keyboard controller and joint-target adapter. Protected hashes match original Git blobs in baseline_snapshot.json.

Completion criteria NOT met: SONIC encoder/decoder real-input inference, whole-body PhysX control and SONIC keyboard locomotion. See compatibility_report.md for exact remaining porting tasks; no claim that SONIC is ready.
