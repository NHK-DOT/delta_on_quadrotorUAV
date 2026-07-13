#!/usr/bin/env bash
set -eo pipefail

FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
LOCAL_POSITION_HZ="${LOCAL_POSITION_HZ:-10}"
MAVROS_LOG="${MAVROS_LOG:-/tmp/mavros.log}"

source /opt/ros/humble/setup.bash
source /workspace/uav_delta_capture/install/setup.bash
set -u

pkill -f "ros2 launch mavros apm.launch" 2>/dev/null || true
pkill -f "mavros_node" 2>/dev/null || true

ros2 launch mavros apm.launch "fcu_url:=${FCU_URL}" > "${MAVROS_LOG}" 2>&1 &
MAVROS_PID=$!

cleanup() {
  kill "${MAVROS_PID}" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 45); do
  if ros2 service list 2>/dev/null | grep -q '^/mavros/set_message_interval$'; then
    break
  fi
  sleep 1
done

for attempt in $(seq 1 5); do
  echo "Requesting MAVROS local position stream, attempt ${attempt}..."

  timeout 10 ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
    "{stream_id: 6, message_rate: ${LOCAL_POSITION_HZ}, on_off: true}" || true

  timeout 10 ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
    "{message_id: 32, message_rate: ${LOCAL_POSITION_HZ}.0}" || true

  if timeout 8 ros2 topic echo /mavros/local_position/pose --once >/tmp/mavros_local_position_check.log 2>&1; then
    echo "MAVROS is running; /mavros/local_position/pose is publishing at requested ${LOCAL_POSITION_HZ} Hz."
    trap - EXIT
    wait "${MAVROS_PID}"
    exit 0
  fi

  sleep 2
done

echo "ERROR: /mavros/local_position/pose did not publish after requesting LOCAL_POSITION_NED" >&2
cat /tmp/mavros_local_position_check.log >&2 || true
exit 1
