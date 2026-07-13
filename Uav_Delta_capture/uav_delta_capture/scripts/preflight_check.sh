#!/usr/bin/env bash

PROFILE="${1:-full}"
TMP_DIR="/tmp/uav_preflight_check"
UWB_LOG="/tmp/uwb_aoa_driver_preflight.log"
UWB_STARTED=0
NOT_READY=0

mkdir -p "${TMP_DIR}"

source /opt/ros/humble/setup.bash
source /workspace/uav_delta_capture/install/setup.bash
set -u

cleanup() {
  if [ "${UWB_STARTED}" -eq 1 ]; then
    pkill -f '[u]wb_aoa_driver_node' 2>/dev/null || true
  fi
}
trap cleanup EXIT

ok() {
  echo "OK: $1"
}

warn() {
  echo "WAIT: $1"
  NOT_READY=1
}

fail() {
  echo "FAIL: $1"
  NOT_READY=1
}

have_topic_once() {
  local topic="$1"
  local out="$2"
  timeout 5 ros2 topic echo "${topic}" --once >"${out}" 2>&1
}

have_topic_once_best_effort() {
  local topic="$1"
  local out="$2"
  timeout 5 ros2 topic echo "${topic}" --qos-reliability best_effort --once >"${out}" 2>&1
}

check_uwb() {
  if ! ros2 topic list 2>/dev/null | grep -q '^/uwb_aoa/data$'; then
    ros2 run uwb_driver uwb_aoa_driver_node --ros-args \
      -p serial_port:=/dev/ttySTM1 \
      -p serial_baud:=115200 >"${UWB_LOG}" 2>&1 &
    UWB_STARTED=1
    sleep 2
  fi

  if have_topic_once "/uwb_aoa/data" "${TMP_DIR}/uwb.txt"; then
    DISTANCE_M=$(awk '/^distance_m:/ {print $2; exit}' "${TMP_DIR}/uwb.txt")
    AZIMUTH_DEG=$(awk '/^azimuth_deg:/ {print $2; exit}' "${TMP_DIR}/uwb.txt")
    if [ -n "${DISTANCE_M:-}" ] && awk -v d="${DISTANCE_M}" 'BEGIN {exit !(d > 0.0)}'; then
      ok "UWB sample distance=${DISTANCE_M}m azimuth=${AZIMUTH_DEG:-unknown}deg"
    else
      warn "UWB topic readable but distance is ${DISTANCE_M:-unknown}m; check tag power/range before flight"
    fi
    awk '/distance|azimuth|range|angle|data:/ {print "INFO: " $0}' "${TMP_DIR}/uwb.txt" | head -20 || true
  else
    warn "UWB sample not readable; check tag power, serial /dev/ttySTM1, and ${UWB_LOG}"
    tail -40 "${UWB_LOG}" 2>/dev/null || true
  fi
}

check_full_stack() {
  if pgrep -f 'test_mission_node.py|flight_commander_node|flight_state_machine_node|fcu_link_monitor_node' >/dev/null 2>&1; then
    fail "mission/helper nodes are already running; stop them before a new test"
  else
    ok "no mission/helper residual processes"
  fi

  if ! pgrep -f 'mavros_node' >/dev/null 2>&1; then
    fail "mavros_node is not running; start /workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh"
  else
    ok "mavros_node running"
  fi

  STATE_OUT="${TMP_DIR}/mavros_state.txt"
  if have_topic_once "/mavros/state" "${STATE_OUT}"; then
    if grep -q 'connected: true' "${STATE_OUT}"; then
      ok "FCU connected"
    else
      fail "FCU not connected"
    fi
    if grep -q 'armed: false' "${STATE_OUT}"; then
      ok "FCU disarmed"
    else
      fail "FCU is armed"
    fi
    if grep -q 'manual_input: true' "${STATE_OUT}"; then
      ok "RC/manual input present"
    else
      warn "RC/manual input not present"
    fi
  else
    fail "cannot read /mavros/state"
  fi

  if timeout 10 ros2 topic hz /mavros/local_position/pose >"${TMP_DIR}/local_pose_hz.txt" 2>&1; then
    if grep -q 'average rate:' "${TMP_DIR}/local_pose_hz.txt"; then
      ok "local_position publishing"
    else
      fail "local_position has no measured rate"
    fi
  else
    fail "local_position hz check failed"
  fi

  if have_topic_once_best_effort "/mavros/local_position/pose" "${TMP_DIR}/local_pose_once.txt"; then
    ok "local_position sample readable"
  else
    fail "local_position sample not readable"
  fi

  if have_topic_once "/mavros/rangefinder_pub" "${TMP_DIR}/rangefinder.txt"; then
    RANGE_VALUE=$(awk '/^range:/ {print $2; exit}' "${TMP_DIR}/rangefinder.txt")
    ok "rangefinder sample ${RANGE_VALUE:-unknown}m"
  else
    fail "rangefinder sample not readable"
  fi

  if have_topic_once "/mavros/optical_flow/raw/optical_flow" "${TMP_DIR}/optical_flow.txt"; then
    FLOW_QUALITY=$(awk '/^quality:/ {print $2; exit}' "${TMP_DIR}/optical_flow.txt")
    if [ -n "${FLOW_QUALITY:-}" ] && [ "${FLOW_QUALITY}" -gt 0 ] 2>/dev/null; then
      ok "optical_flow sample quality=${FLOW_QUALITY}"
    else
      warn "optical_flow sample quality=${FLOW_QUALITY:-unknown}"
    fi
  else
    fail "optical_flow sample not readable"
  fi

  if ros2 service list 2>/dev/null | grep -q '^/mavros/set_mode$'; then
    ok "MAVROS set_mode service available"
  else
    fail "MAVROS set_mode service missing"
  fi
}

echo "========== UAV PREFLIGHT CHECK (${PROFILE}) =========="

case "${PROFILE}" in
  uwb_only)
    check_uwb
    ;;
  full)
    check_full_stack
    check_uwb
    ;;
  *)
    echo "Usage: $0 [full|uwb_only]" >&2
    exit 2
    ;;
esac

echo "========================================="
if [ "${NOT_READY}" -eq 0 ]; then
  echo "RESULT: READY"
  exit 0
fi

echo "RESULT: NOT_READY"
exit 1