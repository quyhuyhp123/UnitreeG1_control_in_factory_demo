#!/usr/bin/env bash
# Mở giao diện huấn luyện YOLO (cửa sổ Tkinter). Chạy từ Terminal trong màn hình desktop.
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${DISPLAY:-}" ]]; then echo 'Không có màn hình (DISPLAY). Hãy chạy lệnh này trong Terminal của desktop Ubuntu / phiên remote desktop.' >&2; exit 2; fi
exec python3 "$here/training_ui.py" "$@"
