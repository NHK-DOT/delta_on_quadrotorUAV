#!/bin/bash
# Start mock FCU bridge nodes - keep them alive after script exits
source /opt/ros/humble/setup.bash
source /workspace/uav_delta_capture/install/setup.bash

# Kill any existing instances
pkill -f fcu_state_node 2>/dev/null
pkill -f flight_commander 2>/dev/null
pkill -f fcu_link_monitor 2>/dev/null
pkill -f flight_state_machine 2>/dev/null
pkill -f mock_mavros_pose 2>/dev/null
sleep 1

echo "Starting mock FCU nodes (persistent)..."
echo "Logs at /tmp/mock_fcu_*.log"

nohup ros2 run fcu_bridge mock_mavros_pose_node --ros-args -p mock_altitude:=0.0 > /tmp/mock_fcu_pose.log 2>&1 &
echo "mock_mavros_pose_node PID=$!"

nohup ros2 run fcu_bridge fcu_state_node --ros-args -p use_mock:=true -p mock_armed:=true -p mock_altitude:=0.0 > /tmp/mock_fcu_state.log 2>&1 &
echo "fcu_state_node PID=$!"

nohup ros2 run fcu_bridge flight_commander_node --ros-args -p use_mock:=true > /tmp/mock_flight_commander.log 2>&1 &
echo "flight_commander_node PID=$!"

nohup ros2 run fcu_bridge fcu_link_monitor_node --ros-args -p use_mock:=true > /tmp/mock_fcu_link.log 2>&1 &
echo "fcu_link_monitor_node PID=$!"

nohup ros2 run fcu_bridge flight_state_machine_node > /tmp/mock_flight_state.log 2>&1 &
echo "flight_state_machine_node PID=$!"

sleep 3
echo ""
echo "=== Checking nodes ==="
ros2 topic list | grep -E "fcu|mavros"
echo ""
echo "=== Checking flight_command service ==="
timeout 3 ros2 service list | grep flight_command && echo "OK: service available" || echo "Waiting for service..."
echo ""
echo "Nodes started. They will keep running after script exits."
