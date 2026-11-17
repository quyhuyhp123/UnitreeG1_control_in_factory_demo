#!/usr/bin/env bash
# Run with Isaac Sim's bundled Python and Linux environment.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$project_dir/scripts/isaac_python.sh" "$project_dir/scripts/run_factory_g1.py" "$@"
