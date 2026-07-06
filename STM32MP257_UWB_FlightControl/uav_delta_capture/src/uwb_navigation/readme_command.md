# uwb_navigation 常用命令

## 0. Start Docker/MAVROS

Use this first. It starts/fixes Docker, starts the ros2humble container, starts MAVROS, requests local_position, then runs read-only checks.

```bash
cd /usr/local/Uav_Delta_capture
./start_ready.sh
```

Success marker:

```text
READY: docker, container, MAVROS checked. Log: /tmp/mlog
```

If it fails, check logs:

```bash
tail -80 /tmp/dockerd.manual.log
docker exec ros2humble tail -100 /tmp/mlog
```

Manual read-only checks:

```bash
# FCU state
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"

# local_position rate
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/local_position/pose"

# rangefinder
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/rangefinder_pub sensor_msgs/msg/Range --qos-profile sensor_data --once"

# optical flow
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/optical_flow/raw/optical_flow"
```

## 1. 一键预检

用途：只读检查，不 ARM，不起飞。

```bash
# 只连 UWB、没连 FCU 时使用。
docker exec ros2humble bash -lc "/workspace/uav_delta_capture/scripts/preflight_check.sh uwb_only"

# FCU/MAVROS/UWB 都连接后使用。
docker exec ros2humble bash -lc "/workspace/uav_delta_capture/scripts/preflight_check.sh full"
```

通过标志：

```text
RESULT: READY
```

## 2. 纯 mock 测试

用途：不连真实 FCU，不上桨，只测 ROS 状态机。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission.launch.py
"
```

## 3. 桌面 bench 测试

用途：去桨，连真实 FCU，测试 ARM、速度指令、DISARM 和传感器链路。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_bench.launch.py
"
```

通过标志：

```text
BENCH RESULT: PASS
```

## 4. 上桨简单起降测试

用途：上桨后首次自主短测，只做起飞、悬停、降落。当前版本使用 MAVROS takeoff 起飞；为避免飞控拒绝低目标高度，takeoff 服务目标约为 `current_local_z + 0.2m`，任务达高仍按 rangefinder 相对高度 `0.6m` 判断，并在降落后等待落地/解锁确认。MAVROS takeoff 尚未达高时，任务节点只观察高度，不向 `/mavros/setpoint_velocity/cmd_vel` 连续发送零速度。

前置：bench 已 PASS，且已完成手动 `ALT_HOLD/LOITER` 短悬停。

遥控接管：

- 空中异常：模式开关最终切到 `ALT_HOLD` 接管；如果开关已经在 `ALT_HOLD` 位，先拨到 `LOITER` 再拨回 `ALT_HOLD`。
- 只拨右手摇杆不等于取消自主任务。
- 已经落地且桨还在转：油门最低，ARM 开关先拨到解锁位，再拨回未解锁位停桨。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_land.launch.py
"
```

通过标志：

```text
TAKEOFF_LAND RESULT: PASS
service_target
Landing complete
```

### 4.1 上桨 LOITER 悬停对比测试

用途：在简单起降基础上，对比飞控 `LOITER` 定点悬停效果。流程是 `GUIDED` takeoff 起飞，到高度后先在 `GUIDED` 低高度稳定约 1.5 秒，再自动切 `LOITER` 悬停 5 秒，最后切回 `GUIDED` 并执行 `LAND`。如果 `LOITER` 悬停期间高度掉到近地阈值以下，会判失败并进入安全处理，不再继续计时判 PASS。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_loiter_land.launch.py
"
```

关键通过标志：

```text
Takeoff OK
service_target
FCU mode confirmed: LOITER
FCU mode confirmed: GUIDED
Landing complete
TAKEOFF_LOITER_LAND RESULT: PASS
```

### 4.2 上桨 GUIDED 定位前进降落测试

用途：UWB 暂不可用时，先验证飞控本地位置 `/mavros/local_position/pose` 是否能支持短距离自主位移。流程是 `GUIDED` takeoff 起飞，到 `0.7m` 相对高度后先悬停稳定，再把 MAVROS setpoint velocity 切到 `BODY_NED`，按机体系 X 正方向前进；前进停止条件不是固定时间，而是 local_position 水平位移达到约 `1.0m`，随后在前方点悬停 2 秒并自动 `LAND`。

前置：简单起降已 PASS，`/mavros/local_position/pose`、测距、光流均为 OK。该模式不启动 UWB driver，不读取 `/dev/ttySTM1`，不要求 `UWB=OK`。

飞行前建议确认 MAVROS 速度坐标系已经被 launch 切成机体系：

```bash
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 param get /mavros/setpoint_velocity mav_frame"
```

期望输出：

```text
String value is: BODY_NED
```

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_forward_land.launch.py
"
```

关键通过标志：

```text
Takeoff OK
Phase: HOVER_TAKEOFF -> FORWARD
Forward moving:
Forward target reached
Forward target hover stable, landing
TAKEOFF_FORWARD_LAND RESULT: PASS
```

如果 `FORWARD timeout`，说明 local_position 位移没有按预期增长，先检查光流、测距、EKF 和地面纹理，不要加大速度硬飞。日志里的 `cmd_body=(0.40,0.00,...)` 表示机体系前向速度；如果仍然不随机头方向飞，先确认 `mav_frame` 是否仍是 `BODY_NED`。

### 4.3 上桨 GUIDED 航点往返降落测试

用途：在不依赖 UWB 的情况下，验证飞控本地坐标能否支持“记录起点、按机体方向飞出、目标点下降悬停、复飞、返回起点、降落”。去程把 MAVROS `setpoint_velocity` 切到 `BODY_NED`，所以 `waypoint_dx/waypoint_dy` 表示机体系前/右；去程停止条件是 `/mavros/local_position/pose` 水平位移达到目标距离。目标点悬停后先下降到 `descend_altitude`，低位悬停 `low_hover_time`，再复飞到 `takeoff_altitude`，稳住后把 `mav_frame` 切到 `LOCAL_NED`，按 local 坐标闭环平移回起点附近。

默认参数：起飞相对高度 `0.7m`，目标点低位高度 `0.5m`，低位悬停 `4.0s`；目标点偏移 `waypoint_dx=1.0m`、`waypoint_dy=0.0m`，含义是“起飞时机头前方 1m、右方 0m”；目标点和起点容差 `0.20m`，最大水平速度 `0.30m/s`。如果要测斜对角，先确认一维往返稳定，再把 `test_mission_takeoff_waypoint_return_land.yaml` 里的 `waypoint_dy` 改成 `1.0` 并重新编译。

飞行前建议确认 MAVROS 速度坐标系初始已经被 launch 切成机体系：

```bash
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 param get /mavros/setpoint_velocity mav_frame"
```

期望输出：

```text
String value is: BODY_NED
```

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_waypoint_return_land.launch.py
"
```

关键通过标志：

```text
Takeoff OK
Phase: HOVER_TAKEOFF -> WAYPOINT_OUTBOUND
Waypoint outbound moving:
Waypoint outbound reached
Phase: HOVER_WAYPOINT -> WAYPOINT_DESCEND
Waypoint descending:
Phase: WAYPOINT_DESCEND -> HOVER_WAYPOINT_LOW
Waypoint low hover holding
Phase: HOVER_WAYPOINT_LOW -> WAYPOINT_RECLIMB
Waypoint reclimbing:
Phase: WAYPOINT_RECLIMB -> HOVER_WAYPOINT_RECLIMB
MAVROS setpoint_velocity mav_frame confirmed: LOCAL_NED
Waypoint return moving:
Waypoint return reached
Home hover stable, landing
TAKEOFF_WAYPOINT_RETURN_LAND RESULT: PASS
```

## 5. 上桨 UWB 接近降落精简测试

用途：在简单 GUIDED 起降通过后，先验证“起飞、UWB 接近 tag 正上方、悬停、原地降落”。这个模式不做抓取、复飞、返航、投放，是完整任务前的上桨精简版本。

前置：简单起降已 PASS，UWB tag 放在地上，场地只做低高度、短距离测试。不要使用 LOITER 版本作为这个测试的前置。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_uwb_approach_land.launch.py
"
```

关键日志：

```text
UWB approach-land preflight
Takeoff OK
Phase: HOVER_TAKEOFF -> MOVE_ABOVE
UWB approach:
Above target
Phase: HOVER_ABOVE -> LAND
UWB_APPROACH_LAND RESULT: PASS
```

通过后再考虑恢复 `test_mission_real_full.launch.py` 的完整抓取、返航、投放流程。

## 6. 日志查看

```bash
# MAVROS 日志
docker exec ros2humble bash -lc "tail -160 /tmp/mavros.log"

# bench 后台日志
docker exec ros2humble bash -lc "grep -E 'Bench preflight|BENCH RESULT|Core links|Sensor links|Bench warnings|Phase|ERROR|WARN' /tmp/mission_bench.log | tail -160"

# 简单起降后台日志
docker exec ros2humble bash -lc "grep -E 'Takeoff-land preflight|TAKEOFF_LAND RESULT|Core links|Sensor links|Phase|Takeoff OK|Landing complete|LAND_WAIT|Takeoff|Land|FAILSAFE|ERROR|WARN' /tmp/mission_takeoff_land.log | tail -160"

# UWB 接近降落精简任务后台日志
docker exec ros2humble bash -lc "grep -E 'UWB approach-land preflight|UWB_APPROACH_LAND RESULT|Core links|Sensor links|Phase|Takeoff OK|UWB approach|Above target|Landing complete|LAND_WAIT|FAILSAFE|ERROR|WARN' /tmp/mission_uwb_approach_land.log | tail -180"

# GUIDED 定位前进降落后台日志
docker exec ros2humble bash -lc "grep -E 'Takeoff-forward-land preflight|TAKEOFF_FORWARD_LAND RESULT|Core links|Sensor links|Phase|Takeoff OK|Forward moving|Forward target|Landing complete|LAND_WAIT|FAILSAFE|ERROR|WARN' /tmp/mission_takeoff_forward_land.log | tail -180"
```
