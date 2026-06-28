#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/nvidia/Desktop/78arm}"
PORT="${PORT:-/dev/ttyUSB0}"
BAUDRATE="${BAUDRATE:-9600}"
RATE_HZ="${RATE_HZ:-10}"
OUTPUT_JSON="${OUTPUT_JSON:-${REPO_DIR}/Dual_Camera_HandEye/output/base_tool_from_servo_latest.json}"
HOLD_TARGET_RAW="${HOLD_TARGET_RAW:-}"
HOLD_REFRESH_SEC="${HOLD_REFRESH_SEC:-1.0}"
HOLD_TOLERANCE_TICKS="${HOLD_TOLERANCE_TICKS:-3}"
HOLD_MAX_LEAD_TICKS="${HOLD_MAX_LEAD_TICKS:-10}"
HOLD_MOVE_MS="${HOLD_MOVE_MS:-900}"

cd "${REPO_DIR}"
mkdir -p Dual_Camera_HandEye/output Delta_Gcode_Servo/real_machine_test/jetson_py36/logs

PID_FILE="Delta_Gcode_Servo/real_machine_test/jetson_py36/logs/base_tool_feedback_publisher.pid"
LOG_FILE="Delta_Gcode_Servo/real_machine_test/jetson_py36/logs/base_tool_feedback_publisher.log"

if [ -f "${PID_FILE}" ]; then
  OLD_PID="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [ -n "${OLD_PID}" ]; then
    kill "${OLD_PID}" 2>/dev/null || true
  fi
fi

ARGS=(
  python3 Delta_Gcode_Servo/real_machine_test/jetson_py36/publish_base_tool_from_servo_feedback_py36.py
  --port "${PORT}"
  --baudrate "${BAUDRATE}"
  --rate-hz "${RATE_HZ}"
  --output "${OUTPUT_JSON}"
  --hold-refresh-sec "${HOLD_REFRESH_SEC}"
  --hold-tolerance-ticks "${HOLD_TOLERANCE_TICKS}"
  --hold-max-lead-ticks "${HOLD_MAX_LEAD_TICKS}"
  --hold-move-ms "${HOLD_MOVE_MS}"
)

if [ -n "${HOLD_TARGET_RAW}" ]; then
  ARGS+=(--hold-target-raw "${HOLD_TARGET_RAW}")
fi

nohup "${ARGS[@]}" > "${LOG_FILE}" 2>&1 < /dev/null &

echo "$!" > "${PID_FILE}"
echo "base_tool_json=${OUTPUT_JSON}"
