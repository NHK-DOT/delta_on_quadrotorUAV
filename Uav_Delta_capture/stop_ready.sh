#!/bin/sh
docker exec ros2humble sh -lc "
  pkill -f '[s]tart_mavros_with_local_position.sh' || true
  pkill -f '[m]avros_node' || true
  pkill -f '[r]os2 launch mavros' || true
  pkill -f '[r]os2 service call /mavros' || true
"
echo 'READY stopped: MAVROS/start_ready helper nodes cleaned.'
