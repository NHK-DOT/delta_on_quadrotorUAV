# uwb_navigation 包说明

`uwb_navigation` 是当前 UWB 无人机抓取项目的主任务包，负责状态机编排、UWB 引导、返航控制、mock 测试、桌面 bench 测试和完整任务入口。

当前推荐使用 Python 节点 `test_mission_node.py` 作为主线。C++ 节点 `uwb_mission_planner_node.cpp` 保留为早期规划器/备用实现，不作为当前调试主入口。

## 运行环境

板子上的 ROS 2 环境运行在 Docker 容器 `ros2humble` 中。下面命令默认在板子系统 shell 中执行；如果已经进入容器，则只执行容器内的 `source` 和 `ros2 ...` 命令即可。

```bash
# 启动 Docker 服务和 ROS 2 容器
systemctl start docker
docker start ros2humble

# 进入容器
docker exec -it ros2humble bash

# 容器内加载 ROS 2 和项目环境
source /opt/ros/humble/setup.bash
source /workspace/uav_delta_capture/install/setup.bash
```

如果修改了 launch、config 或源码，需要在容器内重新编译相关包：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/uav_delta_capture
colcon build --packages-select fcu_bridge uwb_driver uwb_navigation --parallel-workers 2
source /workspace/uav_delta_capture/install/setup.bash
```

## 任务模式

| 模式 | Launch 文件 | 配置文件 | 是否需要真实硬件 | 用途 |
|---|---|---|---|---|
| `mock_full` | `test_mission.launch.py` | `test_mission_mock.yaml` | 否 | 全软件状态机和 ROS 链路测试 |
| `bench_velocity` | `test_mission_bench.launch.py` | `test_mission_bench.yaml` | FCU 必需，UWB/测距/光流/本地位置仅监测 | 不上桨叶，ARM 后发送 Z 轴速度曲线再 DISARM |
| `takeoff_forward_land` | `test_mission_takeoff_forward_land.launch.py` | `test_mission_takeoff_forward_land.yaml` | FCU、测距、光流、本地位置 | 低高度 GUIDED 起飞、按起飞时机头方向移动约 1.0m、悬停、降落 |
| `takeoff_waypoint_return_land` | `test_mission_takeoff_waypoint_return_land.launch.py` | `test_mission_takeoff_waypoint_return_land.yaml` | FCU、测距、光流、本地位置 | 低高度 GUIDED 起飞、按起飞时机头方向定义目标点、返回起点、降落 |
| `uwb_approach_land` | `test_mission_uwb_approach_land.launch.py` | `test_mission_uwb_approach_land.yaml` | 完整硬件 | 低高度速度起飞、UWB 接近 tag 上方；目标不在前向扇区时原地偏航扫描，捕获后悬停降落 |
| `uwb_approach_grasp_return_land` | `test_mission_uwb_grasp_return_land.launch.py` | `test_mission_uwb_grasp_return_land.yaml` | 完整硬件+机械臂 | 复用 staged UWB 接近，tag 上方下降等待抓取，复飞后用 local_position 返航，投放后降落 |
| `uwb_approach_grasp_return_land` | `test_mission_uwb_grasp_return_land_arm_dev.launch.py` | `test_mission_uwb_grasp_return_land_arm_dev.yaml` | 真实机械臂联调 | 从稳定完整流程复制出的隔离入口，后续机械臂话题/节点联调只改这一版 |
| `real_full` | `test_mission_real_full.launch.py` | `test_mission_real.yaml` | 完整硬件 | 完整起飞、UWB 接近、下降、抓取占位、返航、投放占位、降落 |

`test_mission_real.launch.py` 目前是兼容旧命名的 bench 入口，实际等价于 `test_mission_bench.launch.py`。

`uwb_navigation_system.launch.py` 会启动 `real_full` 任务并包含视觉坐标转换 launch，适合后续系统集成。

## 状态机流程

```text
INIT
-> ARM
-> TAKEOFF
-> HOVER_TAKEOFF
-> MOVE_ABOVE
-> HOVER_ABOVE
-> DESCEND
-> HOVER_FINAL
-> WAIT_GRASP
-> CLIMB
-> HOVER_CLIMB
-> RETURN
-> HOVER_RETURN
-> WAIT_DROP
-> LAND
-> DONE
```

`bench_velocity` 模式只使用 `INIT -> ARM -> BENCH_VELOCITY -> DONE`。其中 `BENCH_VELOCITY` 会按“上升速度、零速度保持、下降速度、零速度保持”的顺序发送速度 setpoint，验证导航节点到飞控的速度链路，并在结束时输出 `BENCH RESULT` 总结。

异常状态：

- `PAUSED_MANUAL`：飞控模式离开 `auto_modes` 后进入，表示遥控器或地面站人工接管。
- `RECOVERING`：`fcu_link/status` 变为 `LOST` 后进入，链路恢复为 `OK` 后回到悬停阶段。
- `FAILSAFE`：飞控断连、低电量、恢复超时等严重异常时进入，节点会持续发零速度并尝试降落。

## 导航策略

起飞后，无人机先在 `takeoff_altitude` 高度悬停。`MOVE_ABOVE` 阶段使用 `/uwb_aoa/data` 的距离和方位角信息生成水平速度，让无人机移动到 tag 上方。到达目标上方后进入悬停，再缓慢下降到 `descend_altitude`。

抓取完成后，节点复飞到 `takeoff_altitude`。返航阶段不使用 UWB，而是使用 `/mavros/local_position/pose` 和 `INIT/TAKEOFF` 阶段记录的本地原点。到达原点附近后等待投放完成信号，再执行降落。

该节点不做 SLAM，不直接读取 UTF01。光流/测距数据应由飞控融合后通过 MAVROS 的本地位置话题输出。

## 话题和服务

### 订阅

| 名称 | 类型 | 作用 |
|---|---|---|
| `uwb_aoa/data` | `uav_delta_msgs/msg/UwbAoa` | UWB 距离、方位角、俯仰角和信号有效性 |
| `fcu_state` | `uav_delta_msgs/msg/FcuState` | 飞控连接、解锁、模式、电池、本地高度等状态 |
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | 飞控融合后的本地位置，用于起点记录和返航 |
| `fcu_link/status` | `std_msgs/msg/String` | 飞控本地位置链路状态，常用值为 `OK`/`LOST` |
| `grasp_done` | `std_msgs/msg/String` | 抓取完成信号 |
| `drop_done` | `std_msgs/msg/String` | 投放完成信号 |

### 发布

| 名称 | 类型 | 作用 |
|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/TwistStamped` | 给 `flight_commander_node` 的速度指令 |
| `test_mission/state` | `std_msgs/msg/String` | 当前任务状态 |
| `test_mission/event` | `std_msgs/msg/String` | 关键任务事件 |
| `grasp_command` | `std_msgs/msg/String` | 进入抓取阶段时发布 `start_grasp` |
| `drop_command` | `std_msgs/msg/String` | 进入投放阶段时发布 `start_drop` |
| `uav_bridge/flight_reset` | `std_msgs/msg/String` | 降落完成后重置桥接状态 |

### 服务

| 名称 | 类型 | 作用 |
|---|---|---|
| `flight_command` | `uav_delta_msgs/srv/FlightCommand` | ARM、DISARM、TAKEOFF、LAND 等飞控命令 |
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | ARM 前必要时切换飞控模式 |

`grasp_done` 和 `drop_done` 接受这些完成值：`true`、`ok`、`done`、`complete`、`success`、`1`，大小写不敏感。完整 UWB 抓取返航入口默认 `fake_grasp: false`、`fake_drop: false`，会主动发布 `grasp_command`/`drop_command` 并等待外部完成信号。

## Mock 全流程测试

用途：不接 FCU、不接 UWB，只验证 ROS 节点、状态机和恢复逻辑。

启动：

```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission.launch.py
"
```

这个 launch 会同时启动：

- `mock_mavros_pose_node`
- `fcu_state_node`
- `flight_commander_node`
- `fcu_link_monitor_node`
- `flight_state_machine_node`
- `test_mission_node.py`

另开一个终端查看状态：

```bash
# 查看任务状态
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test_mission/state --once"

# 查看任务事件
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test_mission/event --once"

# 查看飞行状态机
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /uav_bridge/flight_state --once"

# 查看速度指令
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /cmd_vel --once"
```

手动触发链路恢复流程：

```bash
# 发送 LOST，触发 RECOVERING
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String '{data: LOST}'"

# 发送 OK，恢复任务
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /fcu_link/status std_msgs/msg/String '{data: OK}'"
```

建议测试流程：启动 launch，等状态进入 `HOVER_ABOVE` 或 `HOVER_FINAL`，发送 `LOST`，确认进入 `RECOVERING`，再发送 `OK`，确认任务恢复。

## 桌面 bench_velocity 测试

用途：不上桨叶，验证真实 FCU、MAVROS、ARM、速度 setpoint、DISARM 链路，同时观察 UWB、测距、光流和本地位置是否在线。该模式不会让无人机真实起飞，只会在 ARM 后发送 Z 轴速度曲线，完成后自动打印 `BENCH RESULT`，并关闭本次 bench launch 启动的辅助节点。

注意：bench 模式不会因为 UWB、测距、光流或 `/mavros/local_position/pose` 暂时没有数据而卡住。节点会在日志中打印 `Bench preflight` 和最终 `Sensor links`，把 FCU、RC、UWB、rangefinder、optical_flow、local_pose、set_mode 服务分别标成 `OK` 或 `WAIT`。其中 FCU 连接、ARM、速度曲线和 DISARM 是核心判据；传感器项是桌面联通性观察项。完整飞行 `real_full` 仍然要求本地位置可用。

结果判读：

- `BENCH RESULT: PASS`：核心 ARM、速度曲线、DISARM 和全部观察项都正常。可以进入下一阶段的去桨真实任务预检。
- `BENCH RESULT: PASS_WITH_WARNINGS`：核心 ARM、速度曲线、DISARM 成功，但至少一个观察项是 `WAIT`。只说明桌面速度链路通过，不能直接进入 `real_full`。
- `BENCH RESULT: FAIL`：核心链路失败，停止后续测试，先看 `Core links`、`Bench warnings` 和 `/tmp/mavros.log`。

当前通过标准示例：

```text
BENCH RESULT: PASS
Core links: ARM=OK velocity_profile=OK DISARM=OK
Sensor links: FCU=OK mode=GUIDED armed=false RC=OK UWB=OK rangefinder=OK optical_flow=OK local_pose=OK set_mode_srv=OK
```

看到这个结果后，可以认为 FCU、遥控器、MAVROS 本地位置、UWB、测距、光流、模式服务、ARM/速度/DISARM 链路已经完成去桨 bench 验证。

默认速度曲线：

| 阶段 | 时长 | 指令 |
|---|---:|---|
| `bench_climb` | 4 秒 | `linear.z = +0.20 m/s` |
| `bench_hold` | 3 秒 | `linear.z = 0.00 m/s` |
| `bench_descend` | 4 秒 | `linear.z = -0.20 m/s` |
| `bench_zero` | 2 秒 | `linear.z = 0.00 m/s` |

电机体感不作为通过标准。通过标准以终端日志、任务状态机和 `/mavros/setpoint_velocity/cmd_vel` 话题为准。

硬件要求：

- FCU 通过 USB 连接到板子，默认 `/dev/ttyACM0`，波特率 `921600`。
- UWB 经 CP2102 USB-TTL 连接到板子 USB，当前默认 `/dev/ttyUSB0`，波特率 `115200`。
- 遥控器已连接飞控，ARM 前油门杆保持最低。
- 不安装桨叶。

推荐每次手动 bench 前先清场：

```bash
docker restart ros2humble
```

确认没有任务残余进程。下面命令必须保持单行执行，不要在正则的 `|` 后换行：

```bash
docker exec ros2humble bash -c "ps -eo pid,ppid,stat,cmd | grep -E 'mavros|test_mission|uwb_aoa|fcu_state|flight_commander|flight_state_machine|fcu_link_monitor|ros2 topic|ros2 launch' | grep -v grep || true"
```

正常情况下这条命令没有输出。刚重启容器后，`sleep infinity` 是容器 PID 1，不是 ROS 任务残留；如果只看到 `ps -eo ...` 自己，也不是任务残留。看到 `mavros_node`、`test_mission_node.py`、`flight_commander_node` 等才说明还有相关进程。

启动 MAVROS，并请求飞控持续下发本地位置：

```bash
docker exec -d ros2humble bash -lc "/workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh > /tmp/start_mavros_with_local_position.log 2>&1"
```

这个脚本会启动 MAVROS，并调用 `/mavros/set_message_interval` 请求 `LOCAL_POSITION_NED`，也就是 MAVLink `message_id=32`，默认 `10Hz`。如果只需要在已经启动的 MAVROS 上临时恢复本地位置，可以单独执行：

```bash
docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 10.0}"'
```

确认 MAVROS，并按需要观察本地位置：

```bash
# 确认飞控 heartbeat
docker exec ros2humble bash -c "grep -E 'HEARTBEAT|connected' /tmp/mavros.log | tail -5"

# 查看 MAVROS 状态
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"

# 查看本地位置。bench 不会硬等这条，但 real_full 必须依赖它。
# 该话题是 BEST_EFFORT QoS，echo 时显式指定 best_effort 更稳定。
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/local_position/pose --qos-reliability best_effort --once"
```

可选观察 MAVROS 侧传感器链路：

```bash
# 测距，当前 MAVROS 转发话题通常是 /mavros/rangefinder_pub
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/rangefinder_pub --once"

# 光流
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/optical_flow/raw/optical_flow --once"
```

UWB 的 `/uwb_aoa/data` 不是 MAVROS 发布的，而是 `uwb_aoa_driver_node` 发布的。`test_mission_bench.launch.py` 会自动启动这个节点，所以最推荐直接跑 bench，然后看最终 `Sensor links` 里的 `UWB=OK/WAIT`。如果要在 bench 前单独预检 UWB，先临时启动驱动：

```bash
docker exec -d ros2humble bash -c "source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run uwb_driver uwb_aoa_driver_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baud:=115200 > /tmp/uwb_aoa_driver.log 2>&1"

docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /uwb_aoa/data --once"

docker exec ros2humble bash -c "tail -80 /tmp/uwb_aoa_driver.log"

docker exec ros2humble bash -c "pkill -f [u]wb_aoa_driver_node || true"
```

启动 bench：

```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_bench.launch.py
"
```

后台运行并保存日志：

```bash
docker exec -d ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_bench.launch.py > /tmp/mission_bench.log 2>&1
"
```

查看 bench 日志：

```bash
docker exec ros2humble bash -c "grep -E 'Bench preflight|BENCH RESULT|Core links|Sensor links|Bench warnings|Phase|bench|Arm|Disarm|PAUSED|FAILSAFE|ERROR|WARN' /tmp/mission_bench.log | tail -120"
```

一次完整手动测试顺序：

```bash
# 1. 清场
docker restart ros2humble

# 2. 验证无残余任务进程。正常应无输出。
docker exec ros2humble bash -c "ps -eo pid,ppid,stat,cmd | grep -E 'mavros|test_mission|uwb_aoa|fcu_state|flight_commander|flight_state_machine|fcu_link_monitor|ros2 topic|ros2 launch' | grep -v grep || true"

# 3. 启动 MAVROS，并自动请求本地位置
docker exec -d ros2humble bash -lc "/workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh > /tmp/start_mavros_with_local_position.log 2>&1"

# 4. 等 10 秒左右后确认飞控已连接
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"

# 5. 可选：检查 MAVROS 侧光流。看到 quality > 0 表示 MAVROS 收到光流消息。
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/optical_flow/raw/optical_flow --once"

# 6. 启动 bench，等待它自动打印 BENCH RESULT 并退出。UWB 驱动会由这个 launch 自动启动并随 launch 关闭。
docker exec -it ros2humble bash -c "source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 launch uwb_navigation test_mission_bench.launch.py"
```

监测速度链路：

```bash
# 导航节点发布的速度
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /cmd_vel"

# flight_commander 转发给 MAVROS 的速度
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/setpoint_velocity/cmd_vel"
```

## bench PASS 后的下一步

bench 已经 `PASS` 后，不要直接上来就跑完整任务。按下面顺序推进，每一步只验证一个风险点。

### 1. 去桨静态复核

```bash
# 清场
docker restart ros2humble

# 启动 MAVROS，并自动请求 /mavros/local_position/pose
docker exec -d ros2humble bash -lc "/workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh > /tmp/start_mavros_with_local_position.log 2>&1"

# 确认 MAVROS 连接
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"

# 确认本地位置 10Hz 左右
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/local_position/pose"

# 确认 UWB 有数据
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && timeout 5 ros2 topic echo /uwb_aoa/data --once"

# 确认测距和光流
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/rangefinder_pub --once"
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/optical_flow/raw/optical_flow --once"
```

### 2. 去桨 real_full 预跑

用途：验证完整状态机、话题、服务和日志，不验证真实升力。保持去桨、低风险环境、遥控器可随时接管。

```bash
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_real_full.launch.py
"
```

后台保存日志：

```bash
docker exec -d ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_real_full.launch.py > /tmp/mission_real_full.log 2>&1
"

# 观察关键日志
docker exec ros2humble bash -lc "grep -E 'Real preflight|Phase|UWB|local_pose|rangefinder|optical_flow|ARM|GUIDED|LAND|DONE|FAILSAFE|ERROR|WARN' /tmp/mission_real_full.log | tail -160"
```

### 3. 上桨前安全检查和手动短悬停

用途：确认装桨后的电机方向、桨叶方向、姿态控制、高度控制和遥控接管都正常。不要把第一次装桨测试直接交给自主脚本。

必须满足：

- bench 已经 `PASS`，并且 `Sensor links` 里 `local_pose=OK`、`rangefinder=OK`、`optical_flow=OK`。
- 飞控当前 `armed=false`。如果刚跑过 bench，模式可能停在 `GUIDED`，先用遥控器或地面站切回 `ALT_HOLD` 或 `LOITER`。
- 油门杆保持最低。若日志出现 `Arm: Throttle (RC3) is not neutral`，先把油门杆打到最低；如果仍失败，再检查 RC3 校准、通道反向和遥控器油门曲线。
- 遥控器可随时切出 `GUIDED`，并且现场已确认急停/上锁动作。
- 场地清空，电池固定，机架、桨叶和旋向检查完成。

上桨后先手动短悬停：

1. 手动切到 `ALT_HOLD` 或 `LOITER`。
2. 手动解锁，缓慢起飞到 `0.3m` 到 `0.5m`。
3. 悬停几秒，确认没有明显漂移、翻转趋势或高度异常。
4. 手动降落并上锁。
5. 手动短悬停正常后，再进入下一节的自主起降脚本。

### 4. 第一次上桨自主起降测试

用途：只验证自动 ARM、低高度起飞、短悬停、自动 LAND，不验证 UWB 接近、抓取、返航。首次默认起飞高度 `0.6m`，悬停 `5s`。该 launch 会在 `test_mission_node` 结束后自动 shutdown 辅助节点；上桨测试不跳过 EKF 检查。

前置条件：bench 已经 `PASS`，并且手动短悬停正常。遥控器全程准备切出 GUIDED 或接管。

```bash
# 清场
docker restart ros2humble

# 启动 MAVROS，并自动恢复 /mavros/local_position/pose
docker exec -d ros2humble bash -lc "/workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh > /tmp/start_mavros_with_local_position.log 2>&1"

# 确认飞控已连接、未解锁。上桨测试前建议先让模式回到 ALT_HOLD 或 LOITER。
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"

# 确认本地位置、测距、光流。本地位置话题使用 BEST_EFFORT QoS。
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/local_position/pose"
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/local_position/pose --qos-reliability best_effort --once"
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/rangefinder_pub --once"
docker exec ros2humble bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/optical_flow/raw/optical_flow --once"

# 前台运行，方便随时 Ctrl+C 看日志
docker exec -it ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_land.launch.py
"
```

后台保存日志：

```bash
docker exec -d ros2humble bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_takeoff_land.launch.py > /tmp/mission_takeoff_land.log 2>&1
"

docker exec ros2humble bash -lc "grep -E 'Takeoff-land preflight|TAKEOFF_LAND RESULT|Core links|Sensor links|Phase|Takeoff|Land|FAILSAFE|ERROR|WARN' /tmp/mission_takeoff_land.log | tail -160"
```

通过标准：

```text
TAKEOFF_LAND RESULT: PASS
Core links: ARM=OK TAKEOFF=OK HOVER=OK LAND=OK
Sensor links: FCU=OK ... RC=OK ... rangefinder=OK ... optical_flow=OK local_pose=OK set_mode_srv=OK
```

通过后再进入小范围 GUIDED 定位前进测试，然后跑 `uwb_approach_land`，最后才跑低高度短距离 `real_full`。

LOITER 对比测试使用 `test_mission_takeoff_loiter_land.launch.py`。它沿用简单起降的 MAVROS takeoff 逻辑，到达目标相对高度后先在 `GUIDED` 稳定约 1.5 秒，再切 `LOITER` 悬停，随后切回 `GUIDED` 执行 `LAND`。这个测试用于比较飞控 `LOITER` 定点保持效果，不验证 UWB 接近。

GUIDED 定位前进测试使用 `test_mission_takeoff_forward_land.launch.py`。它沿用简单起降的 MAVROS takeoff 逻辑，到达低高度后保持 `GUIDED`，先把 MAVROS `setpoint_velocity` 的 `mav_frame` 切到 `BODY_NED`，再发送机体系 X 正方向速度，并用 `/mavros/local_position/pose` 的水平位移达到约 `1.0m` 作为停止条件；随后悬停约 2 秒并 `LAND`。这个测试不依赖 UWB，适合 UWB 串口暂不可用时先验证飞控本地位置和短距离速度控制。

GUIDED 航点往返测试使用 `test_mission_takeoff_waypoint_return_land.launch.py`。它沿用同一套 MAVROS takeoff 逻辑，到达低高度后先把 MAVROS `setpoint_velocity` 的 `mav_frame` 切到 `BODY_NED`，让 `waypoint_dx/waypoint_dy` 按机体系前/右解释；去程用 local_position 水平位移判定到点。目标点悬停后先下降到 `descend_altitude`，低位悬停 `low_hover_time`，再复飞到 `takeoff_altitude` 并稳住；随后节点把 `mav_frame` 切到 `LOCAL_NED`，按 local 坐标闭环平移回起点附近，最后 `LAND`。该测试不主动调转机头，用于验证光流/测距融合得到的 local_position 是否能支撑“去目标点、降落接近、复飞、回起点”。

UWB 接近降落精简测试使用 `test_mission_uwb_approach_land.launch.py`。它在 `GUIDED` 下用测距相对高度执行低速起飞，避免 MAVROS 本地高度偏置导致先升高再回落；随后用 UWB 的直线距离、水平角和俯仰角解算机体系相对区间，再决定动作。UWB base 前下俯安装可通过 `uwb_mount_pitch_down_deg` 做机体系补偿，当前 staged 标定使用 `uwb_lateral_sign=-1.0` 把 UWB 横向方向校准到 BODY_NED。任务会按 27 点标定观察到的特征把目标分为 `FRONT_APPROACH`、`NEAR_CENTER_HOLD`、`CENTER_CAPTURE`、`SIDE_REAR_SCAN` 和 `INVALID_HOLD`：只有目标在严格前方窗口内稳定后才锁定直线接近，远距离保持机体系 X 正方向接近且 `vy=0`；进入近中心高俯仰角、小水平距离窗口后，才允许一个极小上限的横向 P 修正，日志中以 `lat_trim=true/false` 标记。已经前向稳定接近后，若读数进入高俯仰角、小水平分量且前后/左右分量也足够小的近中心区间，则慢速补前到预降落前向门限、少量横向修正或保持高度并允许捕获，不再因为正下方方位角发散而自旋；`NEAR_CENTER_HOLD` 也会使用同一套极小横向 P 修正，稳定后可直接进入 `HOVER_ABOVE`，并且近中心保护区内禁止负向 `vx` 后退追 tag；捕获后进入预降落稳定阶段，参考 27 点低高度中心特征继续校验 UWB raw elevation、水平分量、前后分量和左右分量，中心确认稳定后精简入口切 `LAND`，完整抓取返航入口则进入下降抓取；侧后方、前方但未对准，或几何俯仰短时无效但配置为 `SCAN` 时，进入固定方向偏航扫描，扫描锁定也要求角度进入严格前方窗口。精简入口不做抓取、复飞、返航、投放，是完整任务前的上桨过渡入口。

完整抓取返航入口 `test_mission_uwb_grasp_return_land.launch.py` 使用更清晰的高度层级：`takeoff_altitude=0.75m` 用于起飞、UWB 接近、复飞和返航，中心确认后下降到 `descend_altitude=0.55m` 悬停并等待抓取完成信号；当前高度容差为 `0.12m`，UWB 接近阶段超时为 `40s`。起飞后进入 UWB 接近阶段使用独立的 `takeoff_transition_tolerance=0.20m`，避免测距高度在 0.57-0.65m 附近抖动时反复清零起飞悬停稳定计时；下降、复飞、返航等后续高度判断仍使用较严格的 `altitude_tolerance`。完整流程的预降落中心确认使用更宽的 `0.35m/0.18m/0.25m` 水平、前后、横向门限，横向微调按 `-kp*lateral_dist` 把误差拉回中心；近中心进入 `HOVER_ABOVE` 使用同一套预降落门限，避免 tag 上方附近长时间卡在 `MOVE_ABOVE`。若预降确认超时，会继续悬停并小幅修正 `uwb_preland_timeout_hold_sec=4.0s`，仍无法稳定则进入 `FAILSAFE LAND`。

如果失败：

- `Arm: Throttle (RC3) is not neutral`：油门杆没有在飞控认可的最低/中立位置，先调整油门杆，再查 RC3 校准。
- `Takeoff OK` 后只转桨但 rangefinder 相对高度不涨：检查 MAVROS takeoff 等待阶段是否有连续 `/mavros/setpoint_velocity/cmd_vel` 零速度；低高度起飞不应一边用 CommandTOL 起飞一边持续发零速度。
- `local_pose=WAIT`：不要上桨继续测，先回到 MAVROS 本地位置和 EKF origin 排查。
- `rangefinder=WAIT` 或 `optical_flow=WAIT`：不要跑自主起降，先确认测距、光流供电、安装方向和 MAVROS 话题。

### 5. 低高度 UWB 接近降落与 real_full

第一步先执行 `uwb_approach_land`：真实起飞，UWB 接近 tag 上方，悬停，原地降落。该模式不做抓取、复飞、返航和投放，用来验证 UWB 接近方向、速度和安全边界。

第二步才执行 `real_full`：完整任务状态机，有真实起飞、移动、下降、返航和降落动作，只能在去桨检查、传感器检查、遥控接管验证、场地安全确认之后分阶段测试。

当前硬门槛：`real_full` 必须等到 `/mavros/local_position/pose` 有连续数据才会继续。不要把 `require_local_pose_ready` 改成 `false` 绕过这条保护；没有本地位置时，GUIDED 速度控制和返航都不可靠。

室内真实导航依赖飞控用光流和测距融合出本地位置。bench 中 `UWB=OK rangefinder=OK optical_flow=OK local_pose=WAIT` 的含义是：传感器和上位机链路已经通了，但飞控 EKF 还没有给 MAVROS 输出可用本地位置。

飞控侧检查顺序：

1. 用 Mission Planner 或等价地面站备份当前参数。
2. 确认光流、测距模块方向和安装方向正确，光流朝下，测距朝下。
3. 按 ArduPilot 光流/测距室内定位方案检查 EKF3 source 参数：

```text
EK3_SRC1_POSXY = 0
EK3_SRC1_VELXY = 5
EK3_SRC1_POSZ  = 1
EK3_SRC1_VELZ  = 0
EK3_SRC1_YAW   = 1
EK3_SRC_OPTIONS = 0
```

4. 室内无 GPS 时，用地面站设置 EKF Origin。Mission Planner 可在地图上右键设置 EKF origin。
5. 重新启动 MAVROS 后，确认 `/mavros/local_position/pose` 连续输出，再进入真实任务。

启动前必须确认：

```bash
# UWB 数据有效
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /uwb_aoa/data --once"

# 飞控状态正常
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /fcu_state --once"

# 本地位置正常
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/local_position/pose --qos-reliability best_effort --once"

# 遥控器能切出 GUIDED 并接管
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state --once"
```

定位链路专项检查：

```bash
# 光流。quality > 0 说明 MAVROS 收到光流消息。
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/optical_flow/raw/optical_flow --once"

# 测距。range 应在 min/max 之间，并随高度变化。
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/rangefinder_pub --once"

# EKF 状态。至少要有姿态、水平速度、垂直速度估计。
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/estimator_status --once"

# 本地位置。real_full 前必须连续输出。
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/local_position/pose"
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /mavros/local_position/pose --qos-reliability best_effort --once"
```

启动 UWB 接近降落精简任务：

```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_uwb_approach_land.launch.py
"
```

`uwb_approach_land` 启动后会打印 `UWB approach-land preflight`，其中：

- `UWB=OK`、`rangefinder=OK`、`optical_flow=OK` 表示上位机能看到对应链路。
- `local_pose=WAIT` 表示飞控还没有输出本地位置，任务会继续等待，不会进入 ARM。

分阶段真实飞行建议：

1. 去桨 bench 保持 `PASS` 或只剩 `local_pose=WAIT` 这类已知 warning。
2. 解决 `/mavros/local_position/pose` 后，再跑一次 bench，目标是 `local_pose=OK`。
3. 上桨后先手动 `ALT_HOLD/LOITER` 短悬停，验证高度、光流、测距稳定。
4. 再验证 GUIDED 小速度 setpoint，观察 `/mavros/rc/out` 和机体响应。
5. 跑 `uwb_approach_land`，验证 UWB 接近方向、速度和降落。
6. 最后低高度、短距离跑 `real_full`，先保留 `fake_grasp=true`、`fake_drop=true`。

后台运行并保存日志：

```bash
docker exec -d ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation test_mission_uwb_approach_land.launch.py > /tmp/mission_uwb_approach_land.log 2>&1
"
```

系统集成入口：

```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash
  source /workspace/uav_delta_capture/install/setup.bash
  ros2 launch uwb_navigation uwb_navigation_system.launch.py
"
```

## 抓取和投放占位信号

默认配置下，`fake_grasp` 和 `fake_drop` 为 `true`，节点会用计时器自动推进。如果改成 `false`，需要外部模块发布完成信号。

```bash
# 通知抓取完成
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /grasp_done std_msgs/msg/String '{data: done}'"

# 通知投放完成
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /drop_done std_msgs/msg/String '{data: done}'"
```

## 状态和话题监测

```bash
# 节点列表
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 node list"

# 话题列表
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# 任务状态
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test_mission/state --once"

# 任务事件
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /test_mission/event --once"

# UWB 数据
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /uwb_aoa/data --once"

# 飞控聚合状态
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /fcu_state --once"

# 本地位置
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/local_position/pose --once"

# 速度 setpoint 频率
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/setpoint_velocity/cmd_vel"

# UWB 频率
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /uwb_aoa/data"

# 本地位置频率
docker exec ros2humble bash -c "source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /mavros/local_position/pose"
```

## 清理残留进程

优先温和清理：

```bash
docker exec ros2humble bash -c "
  pkill -f [r]os2\ launch\ uwb_navigation || true
  pkill -f [t]est_mission_node.py || true
  pkill -f [u]wb_aoa_driver_node || true
  pkill -f [f]cu_state_node || true
  pkill -f [f]light_commander_node || true
  pkill -f [f]light_state_machine_node || true
  pkill -f [f]cu_link_monitor_node || true
"
```

停止 MAVROS：

```bash
docker exec ros2humble bash -c "
  pkill -f [r]os2\ launch\ mavros || true
  pkill -f [m]avros_node || true
"
```

查看残留：

```bash
docker exec ros2humble bash -c "ps -eo pid,ppid,stat,cmd | grep -E 'mavros|test_mission|uwb_aoa|fcu_state|flight_commander|flight_state_machine|fcu_link_monitor|ros2 topic|ros2 launch' | grep -v grep || true"
```

这条命令要保持单行。正常无输出；`sleep infinity` 是容器 PID 1，不代表任务残留；`ros2-daemon` 也不控制飞控。

卡死时强制清理：

```bash
docker exec ros2humble bash -c "
  pkill -9 -f [m]avros || true
  pkill -9 -f [t]est_mission_node.py || true
  pkill -9 -f [u]wb_aoa_driver_node || true
  pkill -9 -f [f]cu_state_node || true
  pkill -9 -f [f]light_commander_node || true
  pkill -9 -f [f]light_state_machine_node || true
  pkill -9 -f [f]cu_link_monitor_node || true
"
```

最彻底的清场是重启容器：

```bash
docker restart ros2humble
```

## 遥控器接管

当前任务节点用飞控模式判断人工接管。`auto_modes` 默认只有 `GUIDED`。如果飞控模式离开 `GUIDED`，节点会发布零速度并进入 `PAUSED_MANUAL`，不会自动恢复任务。

测试方法：

1. 启动 bench、`uwb_approach_land` 或 real_full。
2. 用遥控器或地面站把飞控切出 `GUIDED`。
3. 查看 `/test_mission/state`，应变为 `PAUSED_MANUAL`。
4. 查看 `/cmd_vel`，应接近零速度。
5. 需要恢复自主任务时，先清理残留进程，再重新启动对应 launch。

## 关键参数

| 参数 | 说明 |
|---|---|
| `mission_mode` | `mock_full`、`bench_velocity`、`takeoff_hover_land`、`takeoff_forward_land`、`uwb_approach_land` 或 `real_full` |
| `require_uwb_ready` | `INIT` 阶段是否等待 UWB 数据有效 |
| `require_local_pose_ready` | `INIT` 阶段是否等待本地位置有效 |
| `takeoff_altitude` | 起飞和返航阶段使用的固定高度 |
| `descend_altitude` | 抓取前下降到的高度 |
| `max_vel_xy` | 水平最大速度 |
| `max_vel_z` | 垂直最大速度 |
| `velocity_slew_rate` | 速度变化限幅，用于平滑指令 |
| `forward_target_distance` | `takeoff_forward_land` 前进阶段目标 local_position 水平位移 |
| `forward_velocity` | `takeoff_forward_land` 前进阶段机体系 X 正方向速度，要求 MAVROS `mav_frame=BODY_NED` |
| `forward_timeout_sec` | `takeoff_forward_land` 前进阶段最长持续时间，超时后进入安全降落 |
| `target_hover_time` | `takeoff_forward_land` 到达目标位移后的悬停时间 |
| `move_above_timeout_sec` | UWB 接近阶段最长持续时间，超时后进入安全降落 |
| `uwb_missing_timeout_sec` | UWB 接近阶段允许 UWB 数据连续丢失的最长时间 |
| `uwb_region_classifier_enabled` | `uwb_approach_land` 是否启用基于 27 点标定特征的 UWB 相对区间判别 |
| `uwb_center_hold_hdist_m` / `uwb_center_capture_hdist_m` | 已经前向稳定接近后，近中心悬停和捕获使用的水平分量阈值 |
| `uwb_front_line_lock_deg` | `uwb_approach_land` 前方直线锁定和扫描锁定使用的严格前方角度窗口 |
| `uwb_center_creep_speed_mps` | 已直线锁定且近中心但仍在 tag 前方时允许的低速前向补偿速度 |
| `uwb_center_raw_elevation_min_deg` / `uwb_center_raw_elevation_max_deg` | 预降落中心确认参考的低高度中心 raw elevation 窗口 |
| `uwb_preland_hdist_m` / `uwb_preland_max_abs_forward_m` / `uwb_preland_max_abs_lateral_m` | 切 LAND 或进入抓取前要求稳定满足的中心水平、前后和左右分量阈值 |
| `uwb_preland_lateral_enable_hdist_m` / `uwb_preland_lateral_min_body_elevation_deg` | 允许近中心横向微调的水平距离和机体系俯仰角门限 |
| `uwb_preland_lateral_deadband_m` / `uwb_preland_lateral_kp` / `uwb_preland_max_lateral_speed_mps` | 近中心横向 P 修正的死区、比例系数和速度上限；机体系命令按 `-kp*lateral_dist` 修正 |
| `uwb_preland_stable_sec` / `uwb_preland_timeout_sec` / `uwb_preland_timeout_hold_sec` | 预降落中心确认稳定时间、首次超时时间和超时后的悬停微调尝试时间；完整流程仍不稳定则进入 `FAILSAFE LAND` |
| `uwb_scan_settle_sec` | 进入 UWB 偏航扫描后先发布零 yaw/零 XY 的稳定等待时间 |
| `uwb_center_max_abs_forward_m` / `uwb_center_max_abs_lateral_m` | 近中心判别允许的最大前后/左右分量，用来避免把偏离中心的高俯仰角误判成正上方 |
| `uwb_center_min_body_elevation_deg` / `uwb_center_capture_body_elevation_deg` | 近中心悬停和捕获要求的机体系俯仰角阈值 |
| `bench_velocity_z` | bench 阶段 Z 轴速度指令幅值 |
| `bench_climb_sec` / `bench_hold_sec` / `bench_descend_sec` / `bench_zero_sec` | bench 速度曲线各阶段时长 |
| `bench_sensor_timeout` | bench 自检判断传感器消息是否新鲜的超时时间 |
| `auto_modes` | 允许自主任务继续运行的飞控模式列表 |
| `fake_grasp` / `fake_drop` | 是否用计时器代替真实抓取/投放完成信号 |
| `grasp_timeout_sec` / `drop_timeout_sec` | `fake_grasp=false` 或 `fake_drop=false` 时等待外部完成信号的超时时间，超时进入 `FAILSAFE` |
| `grasp_command_topic` / `drop_command_topic` | 完整 UWB 抓取返航任务进入抓取/投放阶段时发布命令的话题 |

## 常见问题

1. MAVROS 没有 heartbeat：检查 FCU USB、`/dev/ttyACM0`、波特率和飞控供电。
2. 本地位置没有数据：检查光流/测距是否被飞控正常融合，室内无 GPS 时这条链路很关键。
3. bench 日志里 UWB 是 `WAIT`：桌面速度链路测试仍可继续；检查 `/uwb_aoa/data` 是否有数据，UWB USB 串口应为 `/dev/ttyUSB0`，参数名是 `serial_baud`。
4. bench 日志里 local_pose 是 `WAIT`：桌面速度链路测试仍可继续；这说明飞控还没输出本地位置，后续 real_full 前必须单独解决。
5. bench 结果是 `PASS_WITH_WARNINGS`：核心 ARM/速度/DISARM 成功，但至少一个传感器或模式服务未就绪；可以继续桌面验证，但不能直接进入 real_full。
6. bench 结果是 `FAIL`：核心链路失败，先看 `Core links` 和 `/tmp/mavros.log`。
7. ARM 被拒：查看 `/tmp/mavros.log` 中的 `PreArm` 或 `Arm` 信息，常见原因是油门不在最低、模式不可解锁、传感器预检失败。
8. 遥控接管后任务不继续：这是当前设计，进入 `PAUSED_MANUAL` 后需要重启任务节点。
9. 改了 launch/config 但没生效：确认已经重新编译并重新 source `/workspace/uav_delta_capture/install/setup.bash`。
