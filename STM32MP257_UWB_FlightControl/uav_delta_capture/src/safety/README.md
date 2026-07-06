# safety

Safety utilities package.

## Current Role

The active UWB mission path currently handles critical checks in:

- `uwb_navigation/test_mission_node.py`
- `fcu_bridge/flight_commander_node.cpp`

`failsafe_manager_node` remains available for future target-point safety gating, but it is not the primary controller for the current UWB mission launch files.

## Current Runtime Checks

For the current UWB mission, use these commands from the Windows project root:

```bash
# Mission state.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /test_mission/state --once'"

# Mission events.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /test_mission/event --once'"

# FCU link status used by the mission recovery logic.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /fcu_link/status --once'"

# Aggregated FCU state: connected, armed, mode, battery, estimator, local pose.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /fcu_state --once'"

# Velocity command that the mission publishes.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /cmd_vel --once'"

# Velocity command forwarded to MAVROS after clamping/stale timeout handling.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once'"
```

## Simulate Link Loss And Recovery

These commands are useful in mock or no-propeller bench tests:

```bash
# Simulate FCU/local-position link loss.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String \"{data: LOST}\"'"

# Simulate link recovery.
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String \"{data: OK}\"'"
```

## Manual Takeover Check

The active mission treats FCU mode takeover as the manual override path. In bench testing, switch the FCU out of `GUIDED` and confirm:

```bash
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /test_mission/state --once'"
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /cmd_vel --once'"
```

Expected behavior: state becomes `PAUSED_MANUAL`, and the mission publishes zero velocity. It does not auto-resume; restart the mission launch after manual intervention.

## Clean Up Safety-Relevant Processes

```bash
python ssh2board.py "docker exec ros2humble bash -lc 'pkill -f [t]est_mission_node.py || true; pkill -f [f]light_commander_node || true; pkill -f [f]cu_link_monitor_node || true; pkill -f [f]light_state_machine_node || true; pkill -f [m]avros || true'"

python ssh2board.py "docker exec ros2humble bash -lc 'ps -eo pid,ppid,cmd | grep -E \"test_mission|flight_commander|fcu_link_monitor|flight_state_machine|mavros\" | grep -v grep || true'"
```
