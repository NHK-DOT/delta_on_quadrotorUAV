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

## 4.4 UWB 27 点交互检测/校准

用途：只读取 UWB 和测距仪数据，不 ARM、不切模式、不发布速度。按提示采集 3 个高度层级，每层 8 个方向加中心点，共 27 点。角度约定是无人机正前方 `0deg`、右侧 `90deg`、后方 `180deg`、左侧 `270deg`。当前标定后 `uwb_lateral_sign=-1.0`，CSV 中的 `body_*` 字段表示已校准到 BODY_NED 的机体系结果。每个点按回车后采样 3 秒，并输出原始 AOA、当前任务几何转换后的机体系数据和测距仪高度。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation uwb_calibration_recorder.launch.py
"
```

当前 tag 放地面时保持默认 `tag_height_m:=0.0`。后续 tag 放在平台上时，只改运行参数，不要把平台高度写进任务代码：

```bash
ros2 launch uwb_navigation uwb_calibration_recorder.launch.py tag_height_m:=0.35
```

输出会实时打印 `raw=(d, az, el)`、`body=(az, el, fwd, lat, h)` 和 `range=`。采完后会保存两个文件：`/tmp/uwb_calibration_<timestamp>_summary.csv` 和 `/tmp/uwb_calibration_<timestamp>_raw.csv`。后续分析以 raw CSV 为准，用 summary CSV 快速看均值、标准差和异常点；不要只根据单点均值直接改飞行参数。

UWB 接近任务现在使用基于 27 点标定特征的相对区间判别：`FRONT_APPROACH` 只有在严格前方窗口内稳定后才锁定直线接近，远距离仍保持横向 `vy=0`，避免追随 UWB 横向噪声斜飞；进入近中心高俯仰角、小水平距离窗口后才允许很小的横向 P 修正，日志会显示 `lat_trim=true/false`。`NEAR_CENTER_HOLD` 在已直线锁定且仍有前向余量时低速补前到预降落前向门限，同时允许极小横向 P 微调；如果高俯仰角、小水平距离和预降落门限已经稳定，则直接进入 `HOVER_ABOVE`，避免到 tag 正上方附近后仍像远距离搜索一样反复找。近中心保护区内禁止负向 `vx` 后退追 tag，UWB 方位角跳到侧后方时也优先保持中心确认。`CENTER_CAPTURE` 要求高俯仰角、小水平分量、更小的前后分量和独立左右分量门限，并优先按 center capture 连续计时，稳定后进入预降落阶段；预降落阶段会参考 27 点低高度中心特征继续校验 UWB raw elevation、水平分量、前后分量和左右分量，中心确认稳定后才切 `LAND` 或完整任务的下降抓取阶段；`SIDE_REAR_SCAN` 才原地偏航扫描，扫描方向进入后固定不再左右翻转，扫描锁定也要求角度进入严格前方窗口；`INVALID_HOLD` 先悬停等待，若当前任务配置为 `SCAN`，超时后也进入偏航扫描。正下方附近的 UWB 方位角可能发散，所以已经前向稳定接近后，主要用高机体系俯仰角和小水平分量判断近中心，不再强制要求 `body_azimuth` 接近 0。完整抓取返航入口额外使用 `takeoff_transition_tolerance=0.20m` 作为起飞后进入 UWB 接近阶段的独立门槛，日志里的 `threshold=...m` 是实际过渡门槛；这不会放宽后续下降、复飞、返航的高度容差。完整流程预降落阶段的横向微调按 `-kp*lateral_dist` 修正；预降确认超时后会继续悬停并小幅修正 `uwb_preland_timeout_hold_sec=4.0s`，仍无法稳定则进入 `FAILSAFE LAND`。

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
UWB approach BODY_NED: region=FRONT_APPROACH ...
UWB region=NEAR_CENTER_HOLD ...
UWB region=NEAR_CENTER_HOLD: near center held, hovering above target ...
UWB region=CENTER_CAPTURE center target captured ...
Above target
Phase: HOVER_ABOVE -> LAND
UWB_APPROACH_LAND RESULT: PASS
```

通过后再考虑恢复 `test_mission_real_full.launch.py` 的完整抓取、返航、投放流程。

### 5.1 上桨 UWB 抓取返航完整流程测试

用途：完全连接真实 FCU、UWB、光流/测距和机械臂模块，发送真实起飞命令。前半段复用当前 UWB staged 接近逻辑，飞到 tag 正上方并完成中心确认后不原地降落，而是从 `takeoff_altitude=0.75m` 降到 `descend_altitude=0.55m` 悬停，发布抓取命令，等待机械臂完成信号；随后复飞到 `0.75m`、切到 `LOCAL_NED`，用 `/mavros/local_position/pose` 闭环返回起飞点上方，发布投放命令，等待完成信号后自动 `LAND`。当前完整流程的起飞高度容差为 `0.12m`，UWB 接近超时为 `40s`，用于给近中心小幅微调和稳定确认留出时间。

机械臂接口：

```text
/grasp_command std_msgs/msg/String: start_grasp
/grasp_done    std_msgs/msg/String: done
/drop_command  std_msgs/msg/String: start_drop
/drop_done     std_msgs/msg/String: done
```

备用手动完成信号：

```bash
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 topic pub --once /grasp_done std_msgs/msg/String '{data: done}'"
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 topic pub --once /drop_done std_msgs/msg/String '{data: done}'"
```

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_uwb_grasp_return_land.launch.py
"
```

真实机械臂联调隔离入口，基于上面稳定版复制，后续机械臂话题或节点集成只改 arm-dev 配置/launch：

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_uwb_grasp_return_land_arm_dev.launch.py
"
```

关键日志：

```text
UWB grasp-return-land preflight
UWB region=CENTER_CAPTURE center target captured ...
UWB preland center confirmed, descending for grasp ...
Phase: HOVER_ABOVE -> DESCEND
Phase: HOVER_FINAL -> WAIT_GRASP
Publishing grasp command
grasp_complete
Phase: HOVER_CLIMB -> WAYPOINT_RETURN
MAVROS setpoint_velocity mav_frame confirmed: LOCAL_NED
Waypoint return reached
Publishing drop command
drop_complete
UWB_GRASP_RETURN_LAND RESULT: PASS
```

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

# UWB 抓取返航完整任务后台日志
docker exec ros2humble bash -lc "grep -E 'UWB grasp-return-land preflight|UWB_GRASP_RETURN_LAND RESULT|Core links|Sensor links|Phase|Takeoff OK|UWB approach|UWB preland|Publishing grasp|grasp_complete|Waypoint return|Publishing drop|drop_complete|Landing complete|LAND_WAIT|FAILSAFE|ERROR|WARN' /tmp/mission_uwb_grasp_return_land.log | tail -220"

# UWB 抓取返航真实机械臂联调入口后台日志
docker exec ros2humble bash -lc "grep -E 'UWB grasp-return-land preflight|UWB_GRASP_RETURN_LAND RESULT|Core links|Sensor links|Phase|UWB yaw scan|UWB approach|UWB preland|Publishing grasp|Received grasp_done|grasp_done accepted|Waypoint return|Publishing drop|Received drop_done|drop_done accepted|LAND_WAIT|FAILSAFE|ERROR|WARN' /tmp/mission_uwb_grasp_return_land_arm_dev.log | tail -260"

# GUIDED 定位前进降落后台日志
docker exec ros2humble bash -lc "grep -E 'Takeoff-forward-land preflight|TAKEOFF_FORWARD_LAND RESULT|Core links|Sensor links|Phase|Takeoff OK|Forward moving|Forward target|Landing complete|LAND_WAIT|FAILSAFE|ERROR|WARN' /tmp/mission_takeoff_forward_land.log | tail -180"
```
