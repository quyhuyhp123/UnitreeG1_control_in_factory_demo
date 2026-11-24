# Fast map switching — measured 2026-11-24

Cause of the pause: old set_obstacle_map removed/referenced the USD subtree, opened a temporary composed USD stage to rewrite the reset state, called app.update repeatedly inside the GUI action, then forced a full occupancy-grid rebuild although only the scenario props changed. This blocked the UI thread.

Changes:

- One resident pool of nine chair/person/box props, loaded once at scene startup from map_2's asset references. Map3 updates transforms to its existing seeded manifest, not references/meshes/materials. Map1 sets invisible and physics:collisionEnabled=false on all scenario colliders. Existing original factory and routes stay unchanged.
- Reset layer changes use direct Sdf opinions; no temporary Usd.Stage asset recomposition. Robot initial pose/joints are not overwritten with current state.
- Removed nested render/update loops inside switching; the next normal app iteration processes changes. Explicit START already commits timeline Play before checking AUTO state.
- Baseline navigation grid is reused when non-scenario planning obstacles and floor bounds are unchanged. Full detector list, scene update logging and SafetySupervisor still see active props. Actual planning obstacle changes invalidate/rebuild the grid. Map safety restrictions STOP/settle/ENTRY remain.

Measured same GUI machine, six switches 1→2→3→1→2→3→1:

| Metric | Before | After |
|---|---:|---:|
| Synchronous map transaction | 1064–1387 ms | 20–33 ms |
| Transaction + next app/physics update | 1097–1476 ms | 56–68 ms |
| Navigation grid reused | no | yes, all six |

Raw reports: logs/map_switch_baseline.json and logs/map_switch_optimized.json. Both policy_fault=null, mode MANUAL. These are short local timings, not a worst-case frame-time guarantee. Native UI callback was also measured around 22–24 ms in the first pool regression. Cold asset/shader startup is shifted to initial scene load and cannot be guaranteed hitch-free on every driver. A single nine-object pool remains resident on Map1, trading some startup/resource residency for fast switching; two separate 18-object pools are not loaded.

Physics regression logs/map_pool_tests.log: map2/3 probe rests on a box at z≈0.75 m; map1 probe at same cached-object position reaches floor z≈0.10 m (hidden props do not collide). All four map changes preserve active prop counts after reset and leave policy healthy. Extended final regression logs/map_pool_tests_final.log also checks seeded collider centers after reset and real non-scenario obstacle invalidation. Full route completion and robot braking trials are not part of this performance test.

Use existing Map dropdown / Apply Map / START; no new command required. Close old Isaac session and relaunch to load the new code. No commit, push, training, route-point edits or stage6 AI avoidance.

Files changed for optimization: controllers/factory_maps.py, controllers/factory_g1_runtime.py, navigation/patrol_controller.py; scripts/test_factory_maps.py (commit timeline in direct-controller test); new benchmark_map_switch.py/test_map_pool.py. Existing unrelated local changes preserved.
