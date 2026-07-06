# uav_delta_capture ROS 2 工作区

这是板子项目的 ROS 2 Humble colcon 工作区。

路径关系：

```text
板子真实路径：/usr/local/Uav_Delta_capture/uav_delta_capture
容器内路径：  /workspace/uav_delta_capture
```

`ros2humble` 容器通过 `docker-compose.yml` 把 `/usr/local/Uav_Delta_capture` 挂载到 `/workspace`。因此容器内的 `/workspace/uav_delta_capture` 和板子上的 `uav_delta_capture/` 是同一份文件。

## 当前包结构

```text
src/
├── uav_delta_msgs/   # 自定义消息和服务
├── fcu_bridge/       # MAVROS/FCU 状态、命令和速度 setpoint 桥接
├── uwb_driver/       # UWB AOA 串口驱动和 /uwb_aoa/data 发布
├── uwb_navigation/   # 当前主任务状态机和 mock/bench/takeoff/real launch
├── safety/           # 预留安全管理工具
├── vision_bridge/    # 视觉侧桥接，待队友模块接入
├── vision_test/      # 视觉/NPU 测试代码，非当前 UWB 飞行主线
└── delta_kinematics/ # Delta 机械臂运动学保留包
```

## 常用命令

```bash
# 构建当前主线包
docker exec ros2humble bash -lc "cd /workspace/uav_delta_capture && source /opt/ros/humble/setup.bash && colcon build --packages-select uav_delta_msgs fcu_bridge uwb_driver uwb_navigation"

# 启动 MAVROS，并请求 local_position 数据流
docker exec -d ros2humble bash -lc "/workspace/uav_delta_capture/scripts/start_mavros_with_local_position.sh > /tmp/start_mavros_with_local_position.log 2>&1"

# 飞前预检
docker exec ros2humble bash -lc "/workspace/uav_delta_capture/scripts/preflight_check.sh full"
```

## 文档入口

- 项目整体说明：`/usr/local/Uav_Delta_capture/README.md`
- 当前任务主线：`src/uwb_navigation/README.md`
- 精简命令清单：`src/uwb_navigation/readme_command.md`
- FCU 桥接说明：`src/fcu_bridge/README.md`
- UWB 驱动说明：`src/uwb_driver/README.md`

当前飞行主线以 `uwb_navigation` 为准；视觉、Jetson、机械臂相关包先保留接口和实验代码，后续再接入。
