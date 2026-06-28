#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/nvidia/Desktop/78arm}"
PORT="${PORT:-/dev/ttyUSB0}"
BAUDRATE="${BAUDRATE:-9600}"
RATE_HZ="${RATE_HZ:-10}"
OUTPUT_JSON="${OUTPUT_JSON:-${REPO_DIR}/Dual_Camera_HandEye/output/base_tool_from_servo_latest.json}"

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

nohup python3 Delta_Gcode_Servo/real_machine_test/jetson_py36/publish_base_tool_from_servo_feedback_py36.py \
  --port "${PORT}" \
  --baudrate "${BAUDRATE}" \
  --rate-hz "${RATE_HZ}" \
  --output "${OUTPUT_JSON}" \
  > "${LOG_FILE}" 2>&1 < /dev/null &

echo "$!" > "${PID_FILE}"
echo "base_tool_json=${OUTPUT_JSON}"
