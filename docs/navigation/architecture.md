# Automatic Patrol Navigation

Navigation adds one velocity-command source above the existing Mjlab ONNX policy. It never writes robot poses, joint positions, actuator parameters or checkpoint contents. Existing PhysX timing remains 0.005 s / 200 Hz, policy 50 Hz, decimation 4. These are simulation-time rates, not a claim of real-time wall-clock performance.

`PatrolScheduler → A/B goal → GlobalPlanner → LocalPlanner → CommandSelector ramp → SafetySupervisor → existing policy.step([vx,vy,yaw_rate]) → Articulation → feedback`.

`G1KeyboardRuntime.select_motion_command()` is the single selection hook. MANUAL delegates to the original keyboard controller. AUTO ignores WASD and arrows as motion sources. One policy object owns joint targets; SONIC remains unavailable and is not advertised as a working backend.

Modules:

- `lane_map.py`: polygons, forbidden zones and conservative circular footprint. Body radius 0.45 m + gait sway 0.08 + pose error 0.04 + margin 0.10 = 0.67 m. This exceeds the measured initial robot bounding-box corner radius, about 0.32 m. A circle is invariant to heading and covers turns in place.
- `occupancy_grid.py`: 0.10 m cells; requires the complete cell plus footprint inside an allowed polygon. Floor bounds are read from USD. Polygon erosion is conservative: one polygon must contain the entire footprint; overlapping narrow polygons may require a combined polygon at a junction.
- `global_planner.py`: eight-connected A*, no diagonal corner cutting, collision-checked connectors and smoothing. Both A→B and B→A must validate. No margin reduction on failure.
- `local_planner.py`: world-to-body pose feedback, bounded translation/yaw, slower final approach, positional/heading/actual-velocity settle criteria. A specified goal heading is independent of direction of travel; returning backwards is supported by the real policy.
- `obstacle_detector.py`: reads enabled scene colliders and world bounding boxes. Records bounds, velocity estimate and simulation timestamp; identifies current Worker/Human names or explicit `navigation:kind = "human"` metadata. It is **SIM GROUND TRUTH**, not AI perception. Conservative AABBs may block more space than actual geometry. The configured vertical band excludes floor markings and overhead geometry; this is not a mesh-level footstep planner.
- `safety_supervisor.py`: validates raw state/tilt/velocity, footprint, data age, obstacle sweeps, projected trajectory and braking distance. A 15-sample / 0.30 s velocity mean is used for trajectory prediction so lateral gait oscillation is not treated as sustained vehicle motion. Raw position, tilt and velocity limits remain monitored. Reaction allowance includes scan/filter delay.
- `patrol_scheduler.py`: nonblocking timers and A→B→A lifecycle. No waiting sleeps and no overlapping missions.
- `patrol_controller.py`: lifecycle, replanning, latched faults, persistence and JSONL logging.
- `patrol_ui.py`: `omni.ui` panel, numeric A/B controls, top-view camera and non-colliding USD overlays. Green lane, red forbidden zones, orange collider outlines, blue dashed path and robot footprint/heading. Original factory USD is unchanged.

Braking calibration is required. `configs/navigation_braking.json` records real PhysX stopping measurements and the policy hash. Stop distance is `v * reaction_latency + v² / (2 * measured_conservative_deceleration) + extra_margin`. Planner projections check both commanded and measured mean velocity. Dynamic obstacle sweeps include robot travel before braking. This is a simulation monitor, not certified industrial safety. An actor appearing inside a physically unavoidable stopping envelope can cause FAULT; collision avoidance is not guaranteed for arbitrary injected actors.

FAULT cannot be cleared by START, STOP, Resume or changing the command source. Explicit acknowledgement is required. Reset cancels the mission and uses the existing controlled robot-reset lifecycle; it never fakes a completed patrol. Exceptions propagate to the launcher with navigation fault context.

Source/API references: [USD BBoxCache](https://openusd.org/release/api/class_usd_geom_b_box_cache.html), [Isaac Sim 6.1 experimental Articulation](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html). Kit rate-limit settings were verified in the installed `C:/isaacsim/apps/*.kit`; the main loop uses Kit pacing instead of Python timer sleeps.
