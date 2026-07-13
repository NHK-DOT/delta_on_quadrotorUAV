# UWB 无人机抓取项目

本项目用于验证一套资源受限条件下的室内无人机自主抓取流程。系统以 STM32MP257F-DK 作为板端计算平台，CUAVv5 飞控负责姿态、定高、本地位置估计和基础飞行控制，UWB AOA 模块提供目标相对距离与角度信息，视觉和机械臂模块负责最后阶段的精定位与抓取。

当前主线目标不是做完整 SLAM，而是在有限场地内先验证“起飞、悬停、UWB 引导接近目标、下降、等待抓取、复飞、返航、投放、降落”这条任务链路是否可行。

## 目录关系

```text
/usr/local/Uav_Delta_capture/              # 板子上的项目根目录
├── docker-compose.yml                     # 把本目录挂载到容器 /workspace
├── scripts/                               # 板子网络等辅助脚本
├── archive_legacy/                        # 历史记录和独立实验脚本归档
├── models/                                # 视觉实验模型，保留原路径
├── uwb-reference/                         # UWB 参考资料，保留原路径
└── uav_delta_capture/                     # ROS 2 colcon 工作区
    ├── scripts/                           # MAVROS 启动、预检等板端脚本
    └── src/                               # ROS 2 packages
```

Docker 容器 `ros2humble` 的 `/workspace` 是这个根目录的 bind mount；容器内 `/workspace/uav_delta_capture` 就是板子上的 `uav_delta_capture/` 工作区，不是另一份代码。

## 系统组成

| 模块 | 作用 |
|---|---|
| STM32MP257F-DK | 运行 ROS 2、任务状态机、UWB 数据链路、飞控桥接节点 |
| CUAVv5 飞控 | 执行飞行控制、ARM/DISARM、模式切换、速度 setpoint、降落命令 |
| UTF01 光流测距一体模块 | 接入飞控，由飞控融合后输出室内本地位置估计 |
| UWB AOA 模块 | 单基站挂在无人机上，目标物体放置 tag，输出距离、方位角、俯仰角 |
| 视觉模块 | 目标上方近距离精定位，当前由队友模块接入 |
| 机械臂/抓取模块 | 执行抓取和投放，当前导航侧预留完成信号接口 |

## 导航思路

无人机起飞后先在固定高度悬停。接近目标阶段主要使用 UWB tag 的相对观测信息，控制无人机平滑移动到目标上方；下降到抓取高度后，导航节点悬停并等待视觉/抓取模块完成。抓取完成后，无人机复飞到安全高度，返航阶段使用飞控发布的 `/mavros/local_position/pose` 和起飞时记录的本地原点，不再依赖 UWB。到达起飞点上方后进入投放占位阶段，最后降落。

该方案依赖飞控在室内能通过光流/测距维持可用的本地位置估计。MAVROS 默认不一定持续收到 `LOCAL_POSITION_NED`，所以当前使用 `uav_delta_capture/scripts/start_mavros_with_local_position.sh` 启动 MAVROS，并主动请求本地位置流。UWB 当前只承担目标相对引导，不承担全局建图或完整定位。

## 代码框架

```text
uav_delta_capture/src/
├── uav_delta_msgs/   # 项目自定义消息和服务：FcuState、UwbAoa、FlightCommand 等
├── fcu_bridge/       # 项目节点与 MAVROS/飞控之间的桥接
├── uwb_driver/       # UWB AOA 串口采集、协议解析、滤波和话题发布
├── uwb_navigation/   # 当前主任务状态机、UWB 导航、mock/bench/takeoff/real launch
├── safety/           # 安全检查和预留 failsafe 工具
├── vision_bridge/    # 视觉坐标转换和视觉侧桥接，当前待队友模块接入
├── vision_test/      # 视觉/NPU 实验测试，非当前 UWB 飞行主线
└── delta_kinematics/ # Delta 机械臂运动学保留包，机械臂侧后续再接入
```

## 包职责

### `uwb_navigation`

当前自主任务主线。核心节点是 Python 版本的 `test_mission_node.py`，负责四种运行模式：

- `mock_full`：纯软件 mock，全流程状态机连通性测试。
- `bench_velocity`：桌面级去桨验证，FCU 连接是硬条件，UWB、测距、光流和本地位置做在线状态监测；ARM 后发送 Z 轴速度曲线，再 DISARM，并在日志中输出 `BENCH RESULT`。
- `takeoff_hover_land`：第一次上桨小范围自主起降测试；流程为 ARM、TAKEOFF、悬停、LAND，不进入 UWB 接近和抓取。
- `real_full`：完整自主任务流程，抓取和投放可以先用假信号或计时器占位。

具体运行命令、话题监测、状态机说明见：

- `uav_delta_capture/src/uwb_navigation/README.md`
- `uav_delta_capture/src/uwb_navigation/readme_command.md`

### `uwb_driver`

负责读取 UWB AOA 模块串口数据，解析距离、方位角、俯仰角等字段，滤波后发布 `uwb_aoa/data`。当前板子默认串口是 `/dev/ttySTM1`，波特率 `115200`。导航节点只消费该话题，不直接操作串口。

### `fcu_bridge`

负责把 MAVROS 的飞控状态、本地位置、电池和估计器信息整理成项目内部状态，并把导航节点发布的 `cmd_vel` 转发到 MAVROS 速度 setpoint。该包也提供 `flight_command` 服务，用于 ARM、DISARM、起飞、降落等飞控命令。

### `safety`

当前主线安全逻辑主要在 `uwb_navigation` 和 `fcu_bridge` 内完成。`safety` 包保留后续更完整的 failsafe 管理能力。

### `vision_bridge`、`vision_test`、`delta_kinematics`

这些包和视觉、Jetson、机械臂相关。当前 UWB 导航主线只预留 `grasp_done`、`drop_done` 完成信号，不在导航侧直接实现视觉识别、手眼标定或机械臂控制。

## 关键数据流

```text
UWB 模块
  -> uwb_driver
  -> /uwb_aoa/data
  -> uwb_navigation/test_mission_node.py
  -> /cmd_vel
  -> fcu_bridge/flight_commander_node
  -> /mavros/setpoint_velocity/cmd_vel
  -> CUAVv5 飞控
```

```text
CUAVv5 + 光流/测距融合
  -> MAVROS
  -> /mavros/local_position/pose
  -> fcu_bridge + uwb_navigation
  -> 起飞点记录、返航、链路健康判断
```

## 常用入口

```bash
# 启动 MAVROS，并请求 local_position 数据流
docker exec -d ros2humble bash -lc "/workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh > /tmp/start_mavros_with_local_position.log 2>&1"

# 飞前预检，按场景选择 full / fcu_only / uwb_only
docker exec ros2humble bash -lc "/workspace/uav_delta_capture/scripts/preflight_check.sh full"

# 查看最精简命令清单
docker exec ros2humble bash -lc "sed -n '1,220p' /workspace/uav_delta_capture/src/uwb_navigation/readme_command.md"
```

## 当前验证顺序

1. `mock_full`：先验证状态机逻辑和事件输出。
2. `bench_velocity`：不上桨叶，验证 FCU、ARM、速度 setpoint、DISARM 链路，同时观察 UWB、测距、光流、本地位置。
3. `takeoff_hover_land`：上桨后第一次小范围自主起飞、悬停、降落。
4. `real_full`：低高度、短距离完整任务，首次保留 `fake_grasp=true`、`fake_drop=true`。
5. 视觉和机械臂模块接入后，把 `grasp_done`、`drop_done` 从占位信号替换成真实完成信号。

根目录 README 只维护项目整体说明；具体命令以 `uwb_navigation/readme_command.md` 和对应包 README 为准。
