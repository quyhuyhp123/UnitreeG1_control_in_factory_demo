# Factory V2: Maps 1 / 2 / 3 (stage 5 scenarios)

**Performance update:** runtime now keeps one shared nine-prop pool resident and switches transforms/visibility/collision, rather than replacing USD references each click. Map1 disables hidden prop collision. The standalone map USD wrappers remain valid. Reset metadata is edited without recomposing assets; baseline navigation grid is reused when only test props change. See [measured map-switch timings](map_switch_performance.md).

Original Blender export and factory_physics.usd are unchanged. Three lightweight USD wrappers in assets/factory/factory_layout_v2/maps reference the same base factory and add static test props. Runtime references only each wrapper's Obstacles subtree under /World/MapObstacles; it does not load a duplicate factory/floor. Map1 adds no new lane props; existing factory machines/workers remain.

Map2 seed5102 and Map3 seed5103 each add nine props: chair, worker and box on shared left corridor, Route1 center branch and Route2 right branch. Positions/yaw are seeded-random within long straight lane interiors, away from ENTRY/door/corners. Both variants are intentionally reproducible; no obstacle appears suddenly under a moving robot. Exact coordinates/types/seeds in map_2.json / map_3.json. Chair/person use existing realistic USD assets; simple static box proxies supply collision. Cardboard box is a low-poly cube. No robot pose writes or kinematic robot tricks.

## Usage

```powershell
cd D:\unitree_g1
.\run_sim.bat -Factory v2 -Map 1
.\run_sim.bat -Factory v2 -Map 2 -YoloDebug
.\run_sim.bat -Factory v2 -Map 3 -CostmapDebug
```

GUI: FACTORY MAP → choose Map1/2/3 → Apply Map at STOP and ENTRY. Select Route1 or Route2 → START PATROL. START applies the selected map and route, and plays the timeline if paused. It never auto-starts merely because the map selection changed. For live switch, STOP, let the robot settle at ENTRY; if away from ENTRY use RESET first. Map switch does not teleport the robot. Reset retains the selected map by updating only map reference opinions in the initial USD layer, not copying current robot pose/joints. Restart defaults to Map1 unless --map is supplied; selection is session-local.

## Stage boundary and safety

This is still stage5: AI perception/costmap does not command Navigation. To allow a mandatory route to start with a deliberate obstacle on its centerline, only the newly tagged test props are excluded from baseline route-planning grid construction. They are NOT excluded from physics, scene detector, sensor rendering or SafetySupervisor: full obstacle list still enters braking/human/dynamic checks. Existing machines/walls remain in the planning map. No route nodes, speed profile or locomotion weights changed.

AUTO can start and progress, but may stop in WAITING_FOR_CLEAR_PATH and eventually timeout/FAULT before a static obstruction. It will not invent an AI detour. Clearing the obstacle requires switching to Map1 at ENTRY, manual recovery/reset or a later stage6 implementation. Do not disable Safety merely to force route completion. Each route's geometric length/checkpoints remain unchanged. Supervisor probe in tests uses synthetic feedback near a known test box; it is not a measured physical braking trial against every prop.

Detection: run with -YoloDebug for person/chair boxes, or -CostmapDebug for unclassified depth geometry. Detection depends on visibility, distance and lighting; no guaranteed per-prop recognition. COCO has no generic cardboard-box class; stage5 depth should represent it as unknown geometry. USD navigation tags identify scenario objects for map/safety plumbing, not AI output. If performance is poor, use -RgbdDebug -NoYolo or disable extra debug windows. No new AI training or downloads were performed.

## Assets / tests

Generate from source (once; existing matching maps are preserved):

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\build_factory_maps.py --no-ros-env
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_factory_maps.py --no-ros-env
```

Builder opens SimulationApp only for bundled USD libraries, never starts a robot or training. Files: controllers/factory_maps.py and three USD/JSON map pairs; map selection hooks in factory_g1_runtime.py, patrol_ui.py and CLI/PowerShell; detector marks scenario props and patrol_controller builds the baseline grid without them; tests/test_factory_maps.py and scripts/test_factory_maps.py. Tests include seed/distance sanity, six route starts, reset persistence, static collider check, physical progress and a drop probe. Full route completion and exhaustive person/chair detection are explicitly not claimed. Prior Windows changes remain local; no commit/push.
