# Map variants — actual checks (2026-11-24)

Final test: logs/factory_maps_test_final.log; report logs/keyboard_runs/20261124_111131/factory_maps_results.json. One GUI Isaac Sim session, real G1/PhysX/policy; no training or sensor-based avoidance. Unit suite:76 tests,74 passed,two existing SONIC checks skipped.

| Map | Test props in supervisor | Route1 displacement after START | Route2 displacement after START |
|---|---:|---:|---:|
| 1 original | 0 | 0.3441 m | 0.3468 m |
| 2 seed5102 | 9 | 0.3441 m | 0.3468 m |
| 3 seed5103 | 9 | 0.3441 m | 0.3468 m |

All six combinations reached MOVING_SEGMENT with no policy fault. Progress is measured near ENTRY before reaching the props, not evidence of passing a blocked lane. Route1 length65.8961m and Route2 length79.8674m remain equal across maps. Reset preserved the selected map. Props have static colliders; no robot pose/kinematic override. A real dynamic cube dropped onto a test box settled at Z0.75m (box top0.65m + cube half-height0.1m). Person/chair collision coverage was not separately exhaustively audited.

Focused SafetySupervisor test used synthetic feedback near a box, returned WAITING_FOR_CLEAR_PATH / Obstacle in braking envelope. Props are NOT ignored by Safety despite exclusion from baseline route validation. This is not a measured robot braking-distance test. No completed blocked route or automatic avoidance was claimed.

Main launcher smoke: `scripts/run_sim.ps1 -Factory v2 -Map 2 -CostmapDebug -Duration 4`. Log logs/factory_g1_20261124_111250.log; snapshot logs/keyboard_runs/20261124_111307/gui_snapshot.json confirms factory_map_id=2. Exit0, zero Error/traceback, policy_fault=null, RGB-D/YOLO/localization/costmap saved. CPU YOLO14/14 frames, mean37.46ms, rate4.31Hz. This proves pipeline operation, not recognition of all props. Cardboard box is not a COCO class; depth unknown geometry remains necessary.

Images: docs/factory_map_screenshots/map_1.png, map_2.png, map_3.png. Final test retained existing USD/Fabric/DLSS/hydratexture warnings, no Error/traceback. Earlier development test exposed deferred Play before START; corrected, final rerun passed.

Base Blender USD, physics USD, route JSON, speed profile and factory_v2 config are unchanged. Prior local Windows edits preserved. No commit/push. Usage/stage5 boundary: docs/factory_map_variants.md.
