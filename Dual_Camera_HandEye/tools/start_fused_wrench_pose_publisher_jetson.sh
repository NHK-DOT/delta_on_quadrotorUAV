#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/nvidia/Desktop/78arm}"
CONTROL_UDP_HOST="${CONTROL_UDP_HOST:-}"
CONTROL_UDP_PORT="${CONTROL_UDP_PORT:-0}"
BASE_TOOL_JSON="${BASE_TOOL_JSON:-${REPO_DIR}/Dual_Camera_HandEye/output/base_tool_from_servo_latest.json}"
RATE_HZ="${RATE_HZ:-10}"
MAX_SOURCE_AGE_SEC="${MAX_SOURCE_AGE_SEC:-0.75}"

cd "${REPO_DIR}"
mkdir -p Dual_Camera_HandEye/output Dual_Camera_HandEye/Log

if [ -f Dual_Camera_HandEye/Log/fused_wrench_pose_publisher.pid ]; then
  OLD_PID="$(cat Dual_Camera_HandEye/Log/fused_wrench_pose_publisher.pid 2>/dev/null || true)"
  if [ -n "${OLD_PID}" ]; then
    kill "${OLD_PID}" 2>/dev/null || true
  fi
fi

ARGS=(
  python3 Dual_Camera_HandEye/tools/publish_fused_wrench_pose.py
  --latest-url http://127.0.0.1:8090/latest.json
  --calibration Dual_Camera_HandEye/output/calibration_result.json
  --calibration-tool-key object_camera
  --output Dual_Camera_HandEye/output/fused_wrench_pose_latest.json
  --rate-hz "${RATE_HZ}"
  --max-source-age-sec "${MAX_SOURCE_AGE_SEC}"
)

if [ -n "${BASE_TOOL_JSON}" ] && [ -f "${BASE_TOOL_JSON}" ]; then
  ARGS+=(--base-tool-json "${BASE_TOOL_JSON}")
else
  if [ -n "${CONTROL_UDP_HOST}" ] && [ "${CONTROL_UDP_PORT}" != "0" ]; then
    echo "Refusing UDP publish without BASE_TOOL_JSON for real base_T_tool." >&2
    exit 2
  fi
  echo "Warning: BASE_TOOL_JSON missing; using simulated base_T_tool for visualization only." >&2
  ARGS+=(--base-tool-rpy 0 0 -0.28 0 0 0)
fi

if [ -n "${CONTROL_UDP_HOST}" ] && [ "${CONTROL_UDP_PORT}" != "0" ]; then
  ARGS+=(--udp-host "${CONTROL_UDP_HOST}" --udp-port "${CONTROL_UDP_PORT}")
fi

nohup "${ARGS[@]}" > Dual_Camera_HandEye/Log/fused_wrench_pose_publisher.log 2>&1 < /dev/null &
echo "$!" > Dual_Camera_HandEye/Log/fused_wrench_pose_publisher.pid
echo "fused_pose_json=${REPO_DIR}/Dual_Camera_HandEye/output/fused_wrench_pose_latest.json"
echo "udp=${CONTROL_UDP_HOST}:${CONTROL_UDP_PORT}"
