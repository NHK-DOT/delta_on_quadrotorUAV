#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/nvidia/Desktop/78arm
VISION_ROOT=/home/nvidia/Desktop/yolo_fisheye_calibration_jetson
SCRIPT="$ROOT/Delta_Gcode_Servo/real_machine_test/jetson_py36/jetson_opencv_equalized_apriltag_producer.py"
CALIB="${CALIB:-$VISION_ROOT/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json}"
OUT_JSON="${OUT_JSON:-$VISION_ROOT/output/apriltag_latest_jetson.json}"
OUT_IMAGE="${OUT_IMAGE:-$VISION_ROOT/output/apriltag_opencv_equalized_annotated.jpg}"

printf 'nvidia\n' | sudo -S systemctl stop jetson-vision.service >/dev/null 2>&1 || true
printf 'nvidia\n' | sudo -S systemctl restart nvargus-daemon >/dev/null 2>&1 || true
mkdir -p "$(dirname "$OUT_JSON")"

exec python3 "$SCRIPT" \
  --sensor-id 0 \
  --sensor-mode 0 \
  --sensor-size 3264x2464 \
  --output-size 1280x960 \
  --fps 21 \
  --preprocess "${PREPROCESS:-equalize}" \
  --tag-size-m "${TAG_SIZE_M:-0.0305}" \
  --calib-json "$CALIB" \
  --output-json "$OUT_JSON" \
  --output-image "$OUT_IMAGE"
