#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/nvidia/Desktop/yolo_fisheye_calibration_jetson
BENCH="$ROOT/nv_gpu_apriltags_bench"
CALIB="${CALIB:-$ROOT/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json}"
OUT_JSON="${OUT_JSON:-$ROOT/output/apriltag_latest_jetson.json}"

printf 'nvidia\n' | sudo -S systemctl stop jetson-vision.service >/dev/null 2>&1 || true
printf 'nvidia\n' | sudo -S jetson_clocks >/dev/null 2>&1 || true
printf 'nvidia\n' | sudo -S systemctl restart nvargus-daemon >/dev/null 2>&1 || true
mkdir -p "$(dirname "$OUT_JSON")"

cd "$BENCH"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
exec ./nv_gpu_apriltag_bench \
  --mode 0 \
  --sensor 3264x2464 \
  --sensor-fps 21 \
  --out 1280x960 \
  --seconds 0 \
  --warmup 8 \
  --gui \
  --preprocess "${PREPROCESS:-gray_blur_gamma07}" \
  --calib-json "$CALIB" \
  --output-json "$OUT_JSON"
