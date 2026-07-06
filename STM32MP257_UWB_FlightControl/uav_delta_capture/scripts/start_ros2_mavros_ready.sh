#!/bin/sh
set -eu

CONTAINER=ros2humble
LOG=/tmp/mlog
DOCKER_LOG=/tmp/dockerd.manual.log
STATE_OUT=/tmp/mavros_state_once.txt

echo '[1/6] ensure dockerd'
if ! pgrep dockerd >/dev/null 2>&1; then
  nohup dockerd > "$DOCKER_LOG" 2>&1 &
  sleep 5
fi
if ! docker ps >/dev/null 2>&1; then
  echo 'ERROR: docker daemon is not ready. Log:'
  tail -80 "$DOCKER_LOG" 2>/dev/null || true
  exit 1
fi

echo '[2/6] ensure runc runtime dir'
mkdir -p /run/user/1000
chmod 700 /run/user/1000

echo '[3/6] check serial devices'
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttySTM* 2>/dev/null || true

echo '[4/6] start container'
docker start "$CONTAINER" >/dev/null 2>&1 || true
docker exec "$CONTAINER" true

echo '[5/6] start MAVROS and request local_position'
docker exec "$CONTAINER" pkill -f 'ros2 launch mavros|mavros_node|start_mavros_with_local_position' >/dev/null 2>&1 || true
sleep 1
docker exec -d "$CONTAINER" bash -lc "/workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh > $LOG 2>&1"

echo '[6/6] wait for MAVROS state'
ok=0
for i in 1 2 3 4 5 6 7 8; do
  sleep 5
  rm -f "$STATE_OUT"
  docker exec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && timeout 8 ros2 topic echo /mavros/state --once" > "$STATE_OUT" 2>&1 || true
  cat "$STATE_OUT"
  if grep -q 'connected:' "$STATE_OUT"; then
    ok=1
    break
  fi
  echo "waiting for /mavros/state... attempt $i"
done
if [ "$ok" != 1 ]; then
  echo 'ERROR: /mavros/state not available. MAVROS log:'
  docker exec "$CONTAINER" tail -100 "$LOG" 2>/dev/null || true
  exit 1
fi

echo 'local_position hz:'
docker exec "$CONTAINER" bash -lc 'source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /mavros/local_position/pose' || true

echo 'READY: docker, container, MAVROS checked. Log: /tmp/mlog'
