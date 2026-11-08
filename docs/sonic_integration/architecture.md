# Architecture and implemented boundary
run_sim.bat forwards arguments to run_sim.ps1 -> run_factory_g1.py. Default locomotion follows the existing runtime and checkpoint path unchanged. --controller is validated before SimulationApp creation.

ControllerSelector owns the session selection; Apply in G1 CONTROLLER MANAGER never replaces runtime.policy. A different selection reports restart required. SONIC selection fails explicitly with CONTROLLER_UNAVAILABLE; it does not silently run locomotion under a SONIC label. Restarting without parameters selects locomotion.

controller_interface.py defines the minimal existing policy reset/step protocol and a delegating LocomotionRLBackend. The live locomotion runtime continues using its original policy directly; no observation/action rewrite was introduced. SonicBackend intentionally raises ControllerUnavailable. This is a gate, not an implemented whole-body controller.

Offline implemented: pinned/hash-verified minimal model download, model tensor inspection, real-state-to-MuJoCo conversion, joint permutation helpers, command direction conversion, CPU idle planner inference. sonic_observation_adapter.py contains coordinate primitives, not a complete 1762-D encoder. sonic_motion_planner.py does not implement online trajectory replenishment/history or body FK. No SONIC actions reach Articulation.

The manager panel is separate from existing keyboard HUD and head preview. Reset/Pause/Resume call existing lifecycle functions. Existing Free/Follow/Overview and SPACE follow behavior stay intact. Panel shows current policy status, loop FPS, inference time and whole-GPU memory sample when available. Unsupported SONIC modes are not selectable as working modes.

Future path: keyboard -> SonicCommandAdapter -> Kinematic Planner -> validated reference/FK buffer -> encoder -> 64-D token -> decoder with proprioceptive history -> verified action/PD adapter -> sole Articulation owner. This path is not connected yet.
