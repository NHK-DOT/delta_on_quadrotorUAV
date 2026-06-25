#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/nvidia/Desktop/yolo_fisheye_calibration_jetson
BENCH="$ROOT/nv_gpu_apriltags_bench"
CALIB="${CALIB:-$ROOT/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json}"
OUT_JSON="${OUT_JSON:-$ROOT/output/apriltag_latest_jetson.json}"
OUT_SIZE="${OUT_SIZE:-1280x960}"
GUI_SCALE="${GUI_SCALE:-1.0}"
GUI_EVERY="${GUI_EVERY:-1}"
EXTRA_ARGS=()
if [[ -n "${EXPOSURE_COMPENSATION:-}" ]]; then
  EXTRA_ARGS+=(--exposure-compensation "$EXPOSURE_COMPENSATION")
fi
if [[ -n "${EXPOSURETIMERANGE:-}" ]]; then
  EXTRA_ARGS+=(--exposuretimerange "$EXPOSURETIMERANGE")
fi
if [[ -n "${GAINRANGE:-}" ]]; then
  EXTRA_ARGS+=(--gainrange "$GAINRANGE")
fi
if [[ -n "${ISPDIGITALGAINRANGE:-}" ]]; then
  EXTRA_ARGS+=(--ispdigitalgainrange "$ISPDIGITALGAINRANGE")
fi
if [[ -n "${TNR_MODE:-}" ]]; then
  EXTRA_ARGS+=(--tnr-mode "$TNR_MODE")
fi
if [[ -n "${TNR_STRENGTH:-}" ]]; then
  EXTRA_ARGS+=(--tnr-strength "$TNR_STRENGTH")
fi
if [[ -n "${NVVIDCONV_INTERPOLATION:-}" ]]; then
  EXTRA_ARGS+=(--nvvidconv-interpolation "$NVVIDCONV_INTERPOLATION")
fi

EXISTING_PIDS="$(pgrep -f '^./nv_gpu_apriltag_bench( |$)' || true)"
if [[ -n "$EXISTING_PIDS" ]]; then
  kill $EXISTING_PIDS >/dev/null 2>&1 || true
  sleep 1
  for pid in $EXISTING_PIDS; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
  done
fi

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
  --out "$OUT_SIZE" \
  --seconds 0 \
  --warmup 8 \
  --gui \
  --gui-scale "$GUI_SCALE" \
  --gui-every "$GUI_EVERY" \
  --preprocess "${PREPROCESS:-gray_blur_gamma07}" \
  --gui-hold-ms "${GUI_HOLD_MS:-0}" \
  --output-hold-ms "${OUTPUT_HOLD_MS:-0}" \
  --calib-json "$CALIB" \
  --output-json "$OUT_JSON" \
  "${EXTRA_ARGS[@]}"
