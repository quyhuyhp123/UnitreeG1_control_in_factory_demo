# Forward return patrol — 2026-11-10

The previous planner used the configured endpoint heading throughout each leg.
With both A and B facing +Y, the B-to-A leg therefore commanded reverse walking.
The planner now faces the path, turns in place until heading error is below the
configured tolerance, and then commands forward travel. Optional final waypoint
orientation applies only after entering the destination tolerance.

Near the endpoint, the approach direction is held to avoid repeated half-turns
from gait sway. Arrival position correction is bounded to 0.08 m/s; it can include
small lateral/reverse settling corrections, unlike the forward-only transit leg.
The arrival phase survives safety-triggered replanning on the same leg. Final
approach speed is capped at 0.10 m/s. Manual keyboard reverse is unchanged.

## Actual verification

- Isaac Sim 6.1 / PhysX, existing factory, existing trained policy, one G1.
- A captured near (0, -4.013), B (0, -2.5), both with final heading about +90°.
- One full A→B→A cycle completed and settled; all seven physical assertions passed.
- A return turn occurred before forward travel; no reverse transit command.
- Maximum return travel heading error: 0.181733 rad (about 10.4°).
- Final position error at A: 0.127757 m, below the configured 0.16 m tolerance.
- No runtime fault; final log has no Error/Fatal/Traceback matches. USD/Fabric warnings remain.
- Run: `logs/patrol_runs/20261110_062217`; trace: `return_turn_trace.json`.
- Log: `logs/patrol_return_turn_v4.log`.
- Machine-readable result: `docs/navigation/return_turn_test_results.json`.
- Unit suite: 34 passed, 2 existing SONIC skips.

The final-orientation turn near the entrance required repeated safety stops and
replanning; the complete physical test took about 75.7 simulation seconds after
startup. Disabling optional endpoint heading avoids an additional final half-turn.
Earlier development runs were interrupted while refining endpoint drift handling;
their v1–v3 logs are preserved and are not successful acceptance runs.
This was automated headless PhysX testing, not physical OS keyboard/mouse testing.

Restart the simulation to load the code change:

```powershell
Set-Location D:\unitree_g1
.\run_sim.bat -Controller locomotion
```

Select A/B and start AUTO PATROL as before. To repeat the physical regression:

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_patrol_return_turn.py --no-ros-env
```
