#!/usr/bin/env bash
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"

PORT="${PORT:-/dev/ttyUSB0}"
HAND_TAG_ID="${HAND_TAG_ID:-3}"
VISION_MODE="${VISION_MODE:-snapshot}"
BASE_CAMERA_SNAPSHOT="${BASE_CAMERA_SNAPSHOT:-/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json}"
APRILTAG_INTRINSICS="${APRILTAG_INTRINSICS:-/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json}"
CALIBRATION="${CALIBRATION:-$ROOT/Dual_Camera_HandEye/output/calibration_result.json}"
SERVO_CONFIG="${SERVO_CONFIG:-$ROOT/lx225_tool_demo/config/lx225_tool.demo.toml}"

cd "$THIS_DIR"
exec python3 jetson_structure_calibration_sampler_py36.py \
  --port "$PORT" \
  --hand-tag-id "$HAND_TAG_ID" \
  --vision-mode "$VISION_MODE" \
  --base-camera-snapshot "$BASE_CAMERA_SNAPSHOT" \
  --apriltag-intrinsics "$APRILTAG_INTRINSICS" \
  --calibration "$CALIBRATION" \
  --servo-config "$SERVO_CONFIG" \
  "$@"
