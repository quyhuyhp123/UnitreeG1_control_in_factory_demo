#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -eq 0 ]]; then
    echo "Usage: $0 script.py [arguments ...]" >&2
    exit 2
fi
isaac_root="${ISAAC_SIM_ROOT:-}"
if [[ -z "$isaac_root" ]]; then
    for candidate in "$project_dir"/../isaacsim* "$project_dir"/../../isaacsim* "$HOME/isaacsim" "$HOME/isaac-sim" /opt/isaacsim /opt/isaac-sim /isaac-sim; do
        if [[ -f "$candidate/python.sh" ]]; then
            isaac_root="$candidate"
            break
        fi
    done
fi
if [[ ! -f "$isaac_root/python.sh" ]]; then
    echo 'Isaac Sim Linux not found. Set ISAAC_SIM_ROOT to the directory containing python.sh.' >&2
    exit 2
fi
export ISAAC_SIM_ROOT="$isaac_root"
cd "$project_dir"
mkdir -p logs
log="logs/linux_$(date +%Y%m%d_%H%M%S)_$$.log"
echo "Isaac Sim log: $project_dir/$log"
bash "$isaac_root/python.sh" "$@" --no-ros-env 2>&1 | tee "$log"
# Kit can terminate the process during shutdown; require the application marker.
needs_completion_marker=false
if [[ "$(basename -- "$1")" == run_factory_g1.py ]]; then
    needs_completion_marker=true
    for arg in "$@"; do
        if [[ "$arg" == --help || "$arg" == -h ]]; then needs_completion_marker=false; fi
    done
fi
if [[ "$needs_completion_marker" == true ]]; then
    if ! grep -Eq '^(FACTORY_FINISHED |PREFLIGHT_OK )' "$log"; then
        echo "Simulation did not complete. See $project_dir/$log" >&2
        exit 1
    fi
fi
