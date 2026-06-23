#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="$(date +%Y%m%d_%H%M%S)"

PORT="${PORT:-/dev/ttyUSB0}"
HAND_TAG_ID="${HAND_TAG_ID:-}"
OUT_DIR="${OUT_DIR:-$ROOT/Delta_Gcode_Servo/real_machine_test/apriltag_workspace_samples/$RUN_ID}"
BASE_CAMERA_SNAPSHOT="${BASE_CAMERA_SNAPSHOT:-/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json}"
APRILTAG_LAUNCH="${APRILTAG_LAUNCH:-/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench/run_fullfov_1280x960_gui.sh}"
CALIBRATION="${CALIBRATION:-$ROOT/Dual_Camera_HandEye/output/calibration_result.json}"
IMU_SNAPSHOT="${IMU_SNAPSHOT:-$ROOT/IMU/wt61c_latest.json}"
GAMEPAD_CONFIG="${GAMEPAD_CONFIG:-$ROOT/bt_8bitdo_min/config/gamepad_8bitdo_bt.json}"

cd "$ROOT/Delta_Gcode_Servo/real_machine_test"

args=(
  --port "$PORT"
  --output-dir "$OUT_DIR"
  --base-camera-snapshot "$BASE_CAMERA_SNAPSHOT"
  --apriltag-launch "$APRILTAG_LAUNCH"
  --calibration "$CALIBRATION"
  --imu-snapshot "$IMU_SNAPSHOT"
  --gamepad-config "$GAMEPAD_CONFIG"
)

if [[ -n "$HAND_TAG_ID" ]]; then
  args+=(--hand-tag-id "$HAND_TAG_ID")
fi

exec python3 apriltag_gamepad_workspace_sampler.py "${args[@]}" "$@"
