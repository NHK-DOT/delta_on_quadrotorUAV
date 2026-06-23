#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/nvidia/Desktop/yolo_fisheye_calibration_jetson
cd "$ROOT"

COLS="${COLS:-10}"
ROWS="${ROWS:-7}"
SQUARE_SIZE_M="${SQUARE_SIZE_M:-0.020}"
OUT="$ROOT/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json"
CAPTURE_DIR="$ROOT/calibration/captures_apriltag_fullfov_1280x960"
mkdir -p "$(dirname "$OUT")"

PIPELINE="nvarguscamerasrc sensor-id=0 sensor-mode=0 ! video/x-raw(memory:NVMM),width=3264,height=2464,framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw,width=1280,height=960,format=I420 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"

python3 calibrate_fisheye_camera.py \
  --source gstreamer \
  --gst-pipeline "$PIPELINE" \
  --cols "$COLS" \
  --rows "$ROWS" \
  --square-size-m "$SQUARE_SIZE_M" \
  --output-file "$OUT" \
  --capture-dir "$CAPTURE_DIR"
