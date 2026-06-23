#!/usr/bin/env bash
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
RUN_ID="$(date +%Y%m%d_%H%M%S)"

PORT="${PORT:-/dev/ttyUSB0}"
HAND_TAG_ID="${HAND_TAG_ID:-3}"
OUT_DIR="${OUT_DIR:-$THIS_DIR/samples/$RUN_ID}"
BASE_CAMERA_SNAPSHOT="${BASE_CAMERA_SNAPSHOT:-/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json}"
APRILTAG_LAUNCH="${APRILTAG_LAUNCH:-/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench/run_fullfov_1280x960_gui.sh}"
CALIBRATION="${CALIBRATION:-$ROOT/Dual_Camera_HandEye/output/calibration_result.json}"
SERVO_CONFIG="${SERVO_CONFIG:-$ROOT/lx225_tool_demo/config/lx225_tool.demo.toml}"
GAMEPAD_CONFIG="${GAMEPAD_CONFIG:-$ROOT/bt_8bitdo_min/config/gamepad_8bitdo_bt.json}"

cd "$THIS_DIR"

APRILTAG_PID=""
cleanup() {
  if [[ -n "$APRILTAG_PID" ]] && kill -0 "$APRILTAG_PID" 2>/dev/null; then
    kill "$APRILTAG_PID" 2>/dev/null || true
    wait "$APRILTAG_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${NO_AUTOSTART_APRILTAG:-0}" != "1" ]]; then
  mkdir -p "$OUT_DIR/logs"
  if [[ -x "$APRILTAG_LAUNCH" || -f "$APRILTAG_LAUNCH" ]]; then
    echo "== starting Jetson 3K AprilTag =="
    OUT_JSON="$BASE_CAMERA_SNAPSHOT" bash "$APRILTAG_LAUNCH" > "$OUT_DIR/logs/jetson_apriltag3k.log" 2>&1 &
    APRILTAG_PID="$!"
    echo "AprilTag pid: $APRILTAG_PID"
    for _ in $(seq 1 60); do
      if python3 - "$BASE_CAMERA_SNAPSHOT" <<'PY'
import json, os, sys, time
path = sys.argv[1]
try:
    payload = json.load(open(path, "r"))
    ts = payload.get("timestamp_unix")
    if isinstance(ts, (int, float)) and (time.time() - float(ts)) < 2.0:
        raise SystemExit(0)
except Exception:
    pass
raise SystemExit(1)
PY
      then
        break
      fi
      sleep 0.5
    done
  else
    echo "warning: AprilTag launch script not found: $APRILTAG_LAUNCH"
  fi
fi

echo "== 78arm Jetson py36 preflight =="
python3 jetson_workspace_preflight.py \
  --port "$PORT" \
  --hand-tag-id "$HAND_TAG_ID" \
  --base-camera-snapshot "$BASE_CAMERA_SNAPSHOT" \
  --calibration "$CALIBRATION" \
  --servo-config "$SERVO_CONFIG" \
  --gamepad-config "$GAMEPAD_CONFIG" \
  --report "$OUT_DIR/preflight_report.json"

echo ""
echo "== 78arm Jetson py36 sampler =="
python3 jetson_apriltag_workspace_sampler_py36.py \
  --port "$PORT" \
  --hand-tag-id "$HAND_TAG_ID" \
  --output-dir "$OUT_DIR" \
  --base-camera-snapshot "$BASE_CAMERA_SNAPSHOT" \
  --apriltag-launch "$APRILTAG_LAUNCH" \
  --calibration "$CALIBRATION" \
  --servo-config "$SERVO_CONFIG" \
  --gamepad-config "$GAMEPAD_CONFIG" \
  --no-autostart-apriltag \
  "$@"
