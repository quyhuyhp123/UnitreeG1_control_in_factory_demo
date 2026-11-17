#!/usr/bin/env bash
# Chạy một script Python với PyTorch + CUDA có sẵn trong Isaac Sim và Ultralytics cài riêng
# trong Training_model/.deps. KHÔNG khởi động Isaac Sim và không cài gì vào Isaac Sim.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-}"
if [[ -z "$isaac_root" ]]; then
    for candidate in "$here"/../../../isaacsim* "$here"/../../isaacsim* "$HOME/isaacsim" /opt/isaacsim; do
        if [[ -d "$candidate/kit/python" ]]; then isaac_root="$(cd "$candidate" && pwd)"; break; fi
    done
fi
if [[ ! -x "$isaac_root/kit/python/bin/python3" ]]; then
    echo 'Không tìm thấy Isaac Sim. Đặt biến ISAAC_SIM_ROOT trỏ tới thư mục cài Isaac Sim.' >&2; exit 2
fi
pip_archive="$(ls -d "$isaac_root"/extscache/omni.kit.pip_archive-*/pip_prebundle 2>/dev/null | head -1)"
# Thư viện của Isaac đứng trước, .deps đứng cuối: torch/numpy/opencv của Isaac luôn được ưu tiên.
export PYTHONPATH="$isaac_root/extsDeprecated/omni.isaac.ml_archive/pip_prebundle:$isaac_root/exts/isaacsim.pip.nv/pip_prebundle:$isaac_root/exts/omni.pip.compute/pip_prebundle:$pip_archive:$isaac_root/exts/omni.isaac.core_archive/pip_prebundle:$isaac_root/exts/omni.pip.cloud/pip_prebundle:$here/.deps"
export YOLO_CONFIG_DIR="$here/.ultralytics"   # cài đặt của Ultralytics nằm trong thư mục này, không ghi vào ~/.config
export YOLO_AUTOINSTALL=false                  # cấm Ultralytics tự pip install vào Python của Isaac Sim
export ISAAC_SIM_ROOT="$isaac_root" MPLBACKEND=Agg PYTHONUNBUFFERED=1
mkdir -p "$YOLO_CONFIG_DIR"
exec "$isaac_root/kit/python/bin/python3" "$@"
