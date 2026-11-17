#!/usr/bin/env bash
# Cài một lần: Ultralytics (bản --no-deps) vào Training_model/.deps và tải trọng số gốc YOLO11n.
# Không cài/sửa gì trong Isaac Sim; xoá thư mục .deps là gỡ sạch.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"
free_kb=$(df -Pk "$here" | awk 'NR==2{print $4}')
if (( free_kb < 700000 )); then echo "Ổ đĩa còn $((free_kb/1024)) MB trống, cần ít nhất ~700 MB." >&2; exit 1; fi
isaac_root="${ISAAC_SIM_ROOT:-$(ls -d "$here"/../../../isaacsim* 2>/dev/null | head -1)}"
python="$isaac_root/kit/python/bin/python3"
echo "[1/3] Cài Ultralytics vào $here/.deps (không kèm phụ thuộc, không đụng tới Isaac Sim)"
"$python" -m pip install --quiet --no-deps --upgrade --target "$here/.deps" \
    ultralytics ultralytics-thop onnx protobuf ml_dtypes polars polars-runtime-32 tqdm
echo "[2/3] Tải trọng số gốc"
mkdir -p weights
for model in yolo11n.pt; do
    [[ -f "weights/$model" ]] || curl -fL --progress-bar -o "weights/$model" "https://github.com/ultralytics/assets/releases/download/v8.4.0/$model"
done
echo "[3/3] Kiểm tra"
./ml_python.sh -c "
import torch, ultralytics, numpy
print('ultralytics', ultralytics.__version__, '| torch', torch.__version__, '| numpy', numpy.__version__)
print('CUDA:', torch.cuda.is_available(), [torch.cuda.get_device_name(i) for i in range(torch.cuda.device_count())])
"
echo "Xong. Mở giao diện bằng: ./run_ui.sh"
