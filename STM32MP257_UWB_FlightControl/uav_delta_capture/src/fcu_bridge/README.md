# fcu_bridge

Bridge package between project nodes and MAVROS.

## Nodes

- `fcu_state_node`: publishes `fcu_state` from MAVROS state, battery, estimator, and local pose.
- `flight_commander_node`: exposes `flight_command` and forwards project `cmd_vel` setpoints to MAVROS.
- `fcu_link_monitor_node`: monitors `/mavros/local_position/pose` freshness and publishes `fcu_link/status`.
- `mock_mavros_pose_node`: publishes synthetic local pose for mock tests.

## Velocity Safety

`flight_commander_node` clamps velocity setpoints and publishes a zero setpoint if the latest project `cmd_vel` is older than `vel_timeout_sec`.

Default topics:

- input: `cmd_vel`
- output: `/mavros/setpoint_velocity/cmd_vel`

## Manual Takeover

Manual intervention is expected to happen at the FCU mode/RC layer. Mission code should stop publishing motion commands when FCU mode leaves the configured autonomous mode list.

For the current Python mission, `auto_modes` defaults to `GUIDED`. Switching the FCU out of `GUIDED` should make the mission publish zero velocity and enter `PAUSED_MANUAL`.

## Build

Run from the Windows project root:

```bash
python sync_to_board.py

python ssh2board.py "docker exec ros2humble bash -lc 'pkill -f [m]avros || true; pkill -f [f]cu_state_node || true; pkill -f [f]light_commander_node || true; pkill -f [f]light_state_machine_node || true; pkill -f [f]cu_link_monitor_node || true; sleep 2'"

python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace/uav_delta_capture && colcon build --packages-select fcu_bridge --parallel-workers 2'"

python ssh2board.py "docker restart ros2humble && sleep 5"
```

## Start MAVROS

MAVROS must be running before real `fcu_bridge` nodes can talk to the FCU.

```bash
# Start MAVROS in the background.
python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 > /tmp/mavros.log 2>&1'"

# Confirm FCU connection.
python ssh2board.py "docker exec ros2humble bash -lc 'grep -E \"HEARTBEAT|connected\" /tmp/mavros.log | tail -5'"

# Inspect raw MAVROS state.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/state --once'"
```

## Run Nodes Manually

Normally these nodes are started by `uwb_navigation` launch files. For isolated debugging:

```bash
python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge fcu_state_node > /tmp/fcu_state_node.log 2>&1'"

python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge fcu_link_monitor_node > /tmp/fcu_link_monitor_node.log 2>&1'"

python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge flight_commander_node --ros-args -p skip_ekf_check:=true -p vel_timeout_sec:=0.5 > /tmp/flight_commander_node.log 2>&1'"

python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge flight_state_machine_node > /tmp/flight_state_machine_node.log 2>&1'"
```

Mock pose publisher for software-only tests:

```bash
python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge mock_mavros_pose_node > /tmp/mock_mavros_pose_node.log 2>&1'"
```

## Monitor FCU Bridge

```bash
# Project-level FCU state.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /fcu_state --once'"

# Link monitor status.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /fcu_link/status --once'"

# Simple bridge state machine.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /uav_bridge/flight_state --once'"

# MAVROS local pose from optical-flow/rangefinder fusion.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/local_position/pose --once'"

# Project velocity input and MAVROS velocity output.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /cmd_vel --once'"
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once'"

# Useful rates.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /fcu_state'"
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/local_position/pose'"
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/setpoint_velocity/cmd_vel'"
```

## Manual FCU Commands

Use these only during no-propeller or otherwise controlled testing.

```bash
# Set ALT_HOLD then ARM. Keep RC throttle at the bottom before this.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{base_mode: 0, custom_mode: ALT_HOLD}\" && sleep 2 && ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\"'"

# DISARM.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: false}\"'"

# Publish a single zero setpoint.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \"{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}\"'"
```

## Stop And Clean Up

```bash
# Stop FCU bridge nodes.
python ssh2board.py "docker exec ros2humble bash -lc 'pkill -f [f]cu_state_node || true; pkill -f [f]light_commander_node || true; pkill -f [f]light_state_machine_node || true; pkill -f [f]cu_link_monitor_node || true; pkill -f [m]ock_mavros_pose_node || true'"

# Stop MAVROS.
python ssh2board.py "docker exec ros2humble bash -lc 'pkill -f [r]os2\ launch\ mavros || true; pkill -f [m]avros_node || true'"

# Check leftovers.
python ssh2board.py "docker exec ros2humble bash -lc 'ps -eo pid,ppid,cmd | grep -E \"mavros|fcu_state|flight_commander|flight_state_machine|fcu_link_monitor|mock_mavros_pose|ros2 launch\" | grep -v grep || true'"

# Hard cleanup if a node is stuck.
python ssh2board.py "docker exec ros2humble bash -lc 'pkill -9 -f [m]avros || true; pkill -9 -f [f]cu_state_node || true; pkill -9 -f [f]light_commander_node || true; pkill -9 -f [f]light_state_machine_node || true; pkill -9 -f [f]cu_link_monitor_node || true; pkill -9 -f [m]ock_mavros_pose_node || true'"
```
