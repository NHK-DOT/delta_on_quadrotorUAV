# test_mission_node 测试指南

## 前提

```bash
systemctl start docker
docker start ros2humble
```

## 一键启动 Mock 测试

```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission.launch.py
"
```

这会替代以前那4行 `docker exec -d`，同时启动 mock_mavros_pose + fcu_state + flight_commander + flight_state_machine + test_mission_node。

Mock 阶段时间已延长（hover_stable=5s, grasp=15s），方便你在飞行途中手动操作。

## 手动触发命令

开另一个终端，在板子上执行：

```bash
# 发 LOST → 触发 RECOVERING
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String '{data: LOST}'"

# 发 OK → 恢复飞行
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String '{data: OK}'"

# 发 RESET → 解锁 FAILSAFE
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /uav_bridge/flight_reset std_msgs/msg/String '{data: RESET}'"

# 看飞行状态机
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /uav_bridge/flight_state --once"

# 看任务状态
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test_mission/state --once"
```

**建议测试流程**：启动 launch → 看到 "HOVER_ABOVE"或"HOVER_FINAL" → 发 LOST → 观察 RECOVERING → 发 OK → 观察恢复继续

## 杀后台节点

```bash
docker exec ros2humble bash -c "pkill -f mock_mavros; pkill -f fcu_state_node; pkill -f flight_commander; pkill -f flight_state_machine; pkill -f fcu_link_monitor"
```

---

# 测试2: 真实 FCU + 真实 UWB, 不上桨叶

- [X] 不上桨叶! 遥控器在手!

## 启动

```bash
# 1. 接硬件
ls /dev/ttyACM0        # FCU
ls /dev/ttyACM1        # UWB

# 2. 启动 MAVROS
docker exec -d ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
"

# 3. 确认连通
docker exec ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic echo /mavros/state --once
"
# 预期: connected: true

# 4. 启动桥接 + UWB 驱动
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge fcu_state_node'
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge flight_commander_node'
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run fcu_bridge flight_state_machine_node'
docker exec -d ros2humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run uwb_driver uwb_aoa_driver_node --ros-args -p serial_port:=/dev/ttyACM1 -p baud_rate:=115200'

# 5. 确认数据就绪
docker exec ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic echo /fcu_state --once      # connected=true
  ros2 topic echo /uwb_aoa/data --once   # signal_valid=true
"

# 6. 启动 test_mission (真实模式)
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  python3 /workspace/uav_delta_capture/install/uwb_navigation/lib/uwb_navigation/test_mission_node.py \
    --ros-args --params-file /workspace/uav_delta_capture/install/uwb_navigation/share/uwb_navigation/test_mission.yaml
"
```

## 监控 (开第三个终端)

```bash
# cmd_vel 看 PID 输出
docker exec ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic echo /cmd_vel
"

# UWB 原始数据
docker exec ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  ros2 topic echo /uwb_aoa/data
"
```

## 手动操作验证

进入 MOVE_ABOVE 阶段后:
- 标签左移 → azimuth 变化 → cmd_vel.angular.z 负值（右旋纠正）
- 标签近移 → distance 变小 → cmd_vel.linear.x 正值（前进）

## 杀所有

```bash
docker exec ros2humble bash -c "pkill -f ros2; pkill -f python3"
```

---

## 手动触发命令速查

```bash
# LOST (触发RECOVERING)
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String '{data: LOST}'"

# OK (恢复)
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String '{data: OK}'"

# RESET (解锁FAILSAFE)
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /uav_bridge/flight_reset std_msgs/msg/String '{data: RESET}'"

# 看状态
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /uav_bridge/flight_state --once"
```





启动docker：
  nohup dockerd > /tmp/dockerd.log 2>&1 & sleep 5
  docker start ros2humble
  