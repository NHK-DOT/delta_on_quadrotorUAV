#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/nvidia/Desktop/78arm}"
RATE_HZ="${RATE_HZ:-2}"
CONTROL_UDP_HOST="${CONTROL_UDP_HOST:-}"
CONTROL_UDP_PORT="${CONTROL_UDP_PORT:-0}"
FUSED_POSE_JSON="${FUSED_POSE_JSON:-Dual_Camera_HandEye/output/fused_wrench_pose_latest.json}"
OUTPUT_JSON="${OUTPUT_JSON:-Dual_Camera_HandEye/output/wrench_grasp_sequence_latest.json}"

cd "${REPO_DIR}"
mkdir -p Dual_Camera_HandEye/output Dual_Camera_HandEye/Log

if [ -f Dual_Camera_HandEye/Log/wrench_grasp_planner.pid ]; then
  OLD_PID="$(cat Dual_Camera_HandEye/Log/wrench_grasp_planner.pid 2>/dev/null || true)"
  if [ -n "${OLD_PID}" ]; then
    kill "${OLD_PID}" 2>/dev/null || true
  fi
fi

INTERVAL="$(python3 - <<PY
rate = float("${RATE_HZ}")
print(1.0 / rate if rate > 0 else 0.5)
PY
)"

(
  while true; do
    ARGS=(
      python3 Dual_Camera_HandEye/tools/plan_wrench_grasp_sequence.py
      --fused-pose "${FUSED_POSE_JSON}"
      --output "${OUTPUT_JSON}"
    )
    if [ -n "${CONTROL_UDP_HOST}" ] && [ "${CONTROL_UDP_PORT}" != "0" ]; then
      ARGS+=(--udp-host "${CONTROL_UDP_HOST}" --udp-port "${CONTROL_UDP_PORT}")
    fi
    "${ARGS[@]}" || true
    sleep "${INTERVAL}"
  done
) > Dual_Camera_HandEye/Log/wrench_grasp_planner.log 2>&1 < /dev/null &

echo "$!" > Dual_Camera_HandEye/Log/wrench_grasp_planner.pid
echo "grasp_sequence_json=${REPO_DIR}/${OUTPUT_JSON}"
echo "udp=${CONTROL_UDP_HOST}:${CONTROL_UDP_PORT}"
