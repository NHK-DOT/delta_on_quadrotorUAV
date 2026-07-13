# uav_delta_capture 架构说明

## 项目概述

本项目是一个基于 ROS 2 的无人机 Delta 机械臂抓取系统，实现从视觉感知到末端执行器运动控制的完整闭环。目标平台为搭载 STM32MP257F NPU 的嵌入式设备，通过 MAVROS 与飞控（FCU）通信。

---

## 目录结构

```
uav_delta_capture/
├── src/
│   ├── perception_logic/       # 视觉感知节点（Python）
│   ├── delta_kinematics/       # Delta 机械臂运动学节点（C++）
│   ├── uav_bridge/             # UAV 桥接、安全守护、状态机节点（C++）
│   └── uav_delta_msgs/         # 自定义消息与服务定义
├── build/                      # colcon 构建产物（忽略）
└── .devcontainer/              # VSCode 开发容器配置
```

---

## ROS 2 包说明

### 1. `perception_logic`（Python）

**节点：** `perception_node`

视觉感知层，订阅相机图像，输出目标在图像坐标系中的像素偏移量。

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/camera/image_raw` | `sensor_msgs/Image` |
| 发布 | `vision/target_offset` | `geometry_msgs/PointStamped` |

- `infer_npu()` 为 NPU 推理预留接口（OpenVINO / ONNX Runtime），当前返回图像中心作为占位实现。
- 输出的 `point.x/y` 为目标相对图像中心的像素偏移，`point.z = 0`。

---

### 2. `delta_kinematics`（C++）

**节点：** `delta_kinematics_node`

Delta 并联机械臂运动学求解层，将三维目标点转换为三个关节角度。

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `target_point` | `geometry_msgs/PointStamped` |
| 发布 | `joint_angles` | `std_msgs/Float64MultiArray` |

- `DeltaKinematics` 类封装正/逆运动学（依赖 Eigen）。
- 参数：`L1`、`L2`（连杆长度）、`R`（基座半径）、`r`（末端半径），单位 mm。
- 以 20ms 定时器（50Hz）驱动输出。

---

### 3. `uav_bridge`（C++）

包含四个节点，负责 UAV 状态桥接、坐标变换、安全守护与飞行状态管理。

#### 3.1 `uav_bridge_node`

坐标变换桥接节点，将视觉偏移从相机坐标系变换到 Delta 机械臂基座坐标系。

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` |
| 订阅 | `vision/target_offset` | `geometry_msgs/PointStamped` |
| 发布 | `target_point` | `geometry_msgs/PointStamped` |

- 使用 `tf2_ros` 将视觉偏移从 `camera_optical_frame` 变换到 `delta_base_link`。
- 通过串口（默认 `/dev/ttySTM0`，921600 baud）与飞控通信（接口预留）。

#### 3.2 `fcu_link_monitor_node`

FCU 链路健康监测节点，监控 MAVROS 位姿话题的心跳，输出链路状态。

| 状态 | 含义 |
|------|------|
| `WAIT_FCU` | 尚未收到位姿数据 |
| `OK` | 链路正常 |
| `LOST` | 超时未收到数据 |

发布到 `fcu_link/status`（`std_msgs/String`）。

#### 3.3 `flight_state_machine_node`

飞行状态机节点，根据链路状态驱动系统状态转换。

```
INIT → WAIT_FCU → TRACKING
                ↘ FAILSAFE
```

发布到 `uav_bridge/flight_state`（`std_msgs/String`）。

#### 3.4 `failsafe_manager_node`

安全守护节点，在链路正常且目标数据新鲜时透传目标点，否则输出预设安全位置。

| 方向 | Topic | 类型 |
|------|-------|------|
| 订阅 | `fcu_link/status` | `std_msgs/String` |
| 订阅 | `target_point` | `geometry_msgs/PointStamped` |
| 发布 | `target_point_safe` | `geometry_msgs/PointStamped` |

- 默认安全位置：`(0, 0, 0.25)` in `delta_base_link`。
- 输入超时阈值：0.4s；输出频率：20Hz。

---

### 4. `uav_delta_msgs`

自定义消息与服务。

| 类型 | 名称 | 字段 |
|------|------|------|
| msg | `GraspTarget` | `geometry_msgs/Point position`, `int32 target_id` |
| srv | `SetArmStatus` | 请求：`int32 status`（IDLE=0/GRASP=1/RESET=2）；响应：`bool success`, `string message` |

---

## 数据流

```
相机图像
  └─► perception_node
        └─► vision/target_offset (像素偏移, camera frame)
              └─► uav_bridge_node  ──(TF变换)──►  target_point (delta_base_link)
                    └─► failsafe_manager_node
                          ├─ [链路OK + 数据新鲜] → target_point_safe (透传)
                          └─ [链路异常 / 超时]   → target_point_safe (安全位置)
                                └─► delta_kinematics_node
                                      └─► joint_angles (3个关节角, rad)

/mavros/local_position/pose
  ├─► uav_bridge_node (缓存UAV位姿)
  └─► fcu_link_monitor_node
        └─► fcu_link/status
              └─► flight_state_machine_node → uav_bridge/flight_state
              └─► failsafe_manager_node
```

---

## Launch 文件

| 文件 | 用途 |
|------|------|
| `uav_delta_system.launch.py` | 完整系统启动入口，支持所有参数覆盖 |
| `uav_delta_with_fcu.launch.py` | 连接真实 FCU（启用 MAVROS + FCU 守护） |
| `uav_delta_with_mock_fcu.launch.py` | 仿真模式（启用 mock_mavros_pose_node） |

关键启动参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_mock_fcu` | `false` | 是否使用模拟位姿 |
| `start_mavros` | `false` | 是否启动 MAVROS 节点 |
| `start_fcu_guard` | `true` | 是否启动安全守护组（monitor + FSM + failsafe） |
| `delta_target_topic` | `target_point_safe` | Delta 节点订阅的目标话题 |

---

## 依赖

- ROS 2（Humble / Iron）
- `rclcpp`, `rclpy`
- `geometry_msgs`, `sensor_msgs`, `std_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `Eigen3`
- `mavros`（可选，真实飞控场景）
