# Space camera separation — 2026-11-10

Previous behavior bound SPACE to stop_generation, zeroed/cleared manual input,
paused AUTO PATROL, and switched the factory camera to Follow. This caused the
reported stop when locking the camera.

Current controls:

- SPACE: request camera Follow only. Does not clear held movement keys, change
  velocity, increment stop_generation, or pause patrol.
- X / Stop [X]: zero manual commands or pause AUTO (explicit Resume required).
- R: existing controlled reset. UI STOP/PAUSE and focus-loss behavior remain.

Camera follows its own follow_generation event. Keyboard mapping, runtime HUD,
patrol hint, active README/manual and test input scripts were updated. Historical
reports describing Space as stop refer to the earlier behavior.

Actual validation:

- Keyboard contract: holding W while pressing/repeating/releasing SPACE preserves
  velocity and W state; one follow event, zero stop events. X still stops and
  requires a fresh movement press.
- Unit suite: 39 passed, two existing SONIC skips.
- Real headless Isaac Sim 6.1 / PhysX, run 20261110_190037: SPACE injected through
  the logical Carb keyboard during MOVING_TO_B. Camera changed Free→Follow;
  patrol remained MOVING_TO_B, forward command continued, no stop event.
- The full A→B→A cycle subsequently completed without runtime fault. All nine
  assertions passed, process exit 0, no Error/Fatal/Traceback matches. Existing
  USD/Fabric warnings remain. Native OS keyboard/mouse was not tested.
- Evidence: `docs/navigation/space_camera_test_results.json`;
  log: `logs/space_camera_physics.log`.

Restart `run_sim.bat -Controller locomotion` to load the new mapping and runtime.
Regression: Isaac Python runs `scripts/test_patrol_return_turn.py --test-space
--cycles 1 --no-ros-env`.
