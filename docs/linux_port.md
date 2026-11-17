# Factory V2 Linux port — 2026-11-11

Source branch: `codex/factory-v2-integration`, commit `d1b64da`.

Added executable `run_sim.sh` and `scripts/isaac_python.sh`, which use Isaac Sim's
Linux `python.sh`, detect ISAAC_SIM_ROOT, forward arguments and save logs.
The launcher checks completion markers and propagates process failures.
Project paths in Python controllers/navigation/scripts/tests/Blender generators
now resolve from source locations. Main and regression launchers use Vulkan on
Linux. GPU telemetry uses platform-compatible subprocess flags. Added `--check`
for selected-factory preflight, and explicit failure on policy/integration faults.

Default factory remains V2, including its multi-route UI and source speed profile
of 0.30 m/s. Robot/policy/factory source assets and route/safety parameters were
not changed. Original Windows launchers and environment settings are retained.

## Verification

Ubuntu 22.04, Isaac Sim 6.1 bundled Python, RTX 4090, Vulkan, PhysX.

- Unit suite: 57 run, 55 passed, 2 existing SONIC skips.
- Factory V1 and V2 asset/config/checkpoint preflight: passed.
- Locomotion numeric contract and keyboard contract: passed.
- Python compilation, Bash syntax and Git whitespace checks: passed.
- `./run_sim.sh --headless --duration 5`: passed. Factory V2 loaded,
  FACTORY_READY and FACTORY_FINISHED recorded, no policy fault.
- Factory and head-camera PNGs and GPU telemetry were produced.

See [runtime results](linux_runtime_results.json). Local logs:
`logs/linux_unit_tests.log`, `logs/linux_v2_runtime.log`, and
`logs/keyboard_runs/20261111_132619/`.

This is a launcher/platform verification. A full multi-route physics cycle and
native desktop keyboard interaction were not rerun. SONIC retains the source
branch's closed-loop control limitation.

## Start

```bash
cd /home/g1sim/projects/UnitreeG1_control_in_factory_demo/factory_v2
export ISAAC_SIM_ROOT=/home/g1sim/projects/isaacsim-6.1.0
./run_sim.sh
```

Use `./run_sim.sh --factory v1` for the old factory. For a server without a
desktop, use `./run_sim.sh --headless --duration 10`.
