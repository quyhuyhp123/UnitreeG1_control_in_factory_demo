#!/usr/bin/env bash
# Run a Python script with the OpenUSD libraries bundled in Isaac Sim, WITHOUT starting the simulator.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-}"
if [[ -z "$isaac_root" ]]; then
    for candidate in "$project_dir"/../isaacsim* "$project_dir"/../../isaacsim* "$HOME/isaacsim" /opt/isaacsim; do
        if [[ -d "$candidate/kit/python" ]]; then isaac_root="$candidate"; break; fi
    done
fi
usd_libs="$(ls -d "$isaac_root"/extscache/omni.usd.libs-* 2>/dev/null | head -1)"
pip_archive="$(ls -d "$isaac_root"/extscache/omni.kit.pip_archive-*/pip_prebundle 2>/dev/null | head -1)"
if [[ -z "$usd_libs" ]]; then echo 'OpenUSD libraries not found. Set ISAAC_SIM_ROOT.' >&2; exit 2; fi
cd "$project_dir"
PYTHONPATH="$usd_libs:$pip_archive${PYTHONPATH:+:$PYTHONPATH}" LD_LIBRARY_PATH="$usd_libs/bin:$isaac_root/kit${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    exec "$isaac_root/kit/python/bin/python3" "$@"
