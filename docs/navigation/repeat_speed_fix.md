# Repeat scheduling and speed — 2026-11-10

## Observed cause

The actual user session `logs/patrol_runs/20261110_064232/telemetry.jsonl`
showed one completed A→B→A cycle, state `CYCLE_COMPLETE`, and about 14 simulation
seconds remaining at simulation time 240.36. The original rest interval was
120 simulation seconds. At about 23 control steps per real second versus a
configured 50 Hz control loop, this rest takes roughly 261 real seconds.
Later telemetry reached `MOVING_TO_A`, completed_cycles=1, cycle_index=2,
simulation time 291.84: the second cycle actually started with the old code.
Max Cycles is a completion limit, not an override of the rest timer. Repeat must
also remain enabled.

## Changes

- New default rest after A: 5 simulation seconds; explicitly loaded saved
  configurations retain their values.
- Default Max Cycles is now 10 as requested; 0 still allows unlimited repetition.
- UI shows completed/limit, Repeat, clock and approximate real-time wait.
- Apply schedule/speed while waiting at A without resetting completed cycles.
  Shortened rest is calculated from the original completion time; clock changes
  require STOP.
- Max Speed range 0.12–0.35 m/s; default 0.35 instead of 0.22 (59% higher target).
- Existing acceleration, braking, tilt, obstacle and policy command-range checks
  remain active. Near-goal speed stays capped at 0.10 m/s and arrival correction
  at 0.08 m/s. Speed refers to simulation time; low rendering/physics throughput
  can still make motion appear slow in real time.

## Validation status

Scheduler tests exercise exactly ten cycles and check stopping after cycle ten,
five-second repeats, editing a waiting interval without losing completed count,
clock-change rejection, Repeat disabled, and the new speed boundary.
The full unit suite has 39 passing tests and two existing SONIC skips.

A first two-cycle PhysX attempt remained in extension startup while the user's
Isaac Sim session was running and was cancelled. After the user closed that
session, a single-instance headless PhysX test completed successfully:

- Run `logs/patrol_runs/20261110_072202`.
- All eight assertions passed, including two complete A→B→A cycles, turn before
  return travel, forward-only transit commands, and completion at cycle limit.
- First completion at simulation time 69.52 s; second start at 74.52 s: exactly
  5.00 simulation seconds of waiting.
- Final state STOPPED with completed_cycles=2 at 116.84 s.
- Maximum forward command 0.3499997 m/s; final A error 0.1053385 m (<0.16 m).
- No runtime fault; process exit 0; zero Error/Fatal/Traceback matches in
  `logs/patrol_repeat_speed_single.log`. Existing USD/Fabric warnings remain.
- A near (0,-4.013), B=(0,-2.5); optional final headings about +90 degrees.
  Final orientation near the entrance still required safety stops/replanning.
- No physical OS keyboard/mouse test was performed.

```powershell
$env:PYTHONEXE='C:\isaacsim\kit\python\python.exe'
& C:\isaacsim\python.bat D:\unitree_g1\scripts\test_patrol_return_turn.py --cycles 2 --speed 0.35 --interval 5 --no-ros-env
```

Evidence is recorded in `docs/navigation/repeat_speed_test_results.json`. Ten
full physical cycles have not been run; the ten-cycle limit is covered by the
scheduler unit test. This acceptance run used two physical cycles.

Restart to load the changes, choose A/B, enable Repeat, Max Cycles=10,
Wait after return A=5, Max Speed=0.35, and START PATROL. Saved older configurations
may restore the old 120-second wait; edit and apply/save explicitly if loaded.
