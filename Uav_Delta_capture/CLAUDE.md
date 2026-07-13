# Uav_Delta_capture

ROS 2 Humble 无人机 + Delta 机械臂协同抓取项目。平台：STM32MP257F-DK (aarch64)。

## 项目结构

```text
uav_delta_capture/src/
├── uav_delta_msgs/      # 自定义消息 (GraspTarget.msg, SetArmStatus.srv)
├── delta_kinematics/    # Delta 机构运动学节点 (C++, Eigen)
├── perception_logic/    # 视觉感知原型 (Python, 预留 NPU)
├── uav_bridge/          # MAVROS/TF2 桥接 (C++)
└── vision_test/         # 视觉推理测试 (Python, stai_mpu NPU / onnxruntime)
```

## 构建和运行

在 Docker 容器内构建（容器已挂载本目录到 /workspace）：

```bash
docker exec -it ros2humble bash -c "
  cd /workspace/uav_delta_capture &&
  source /opt/ros/humble/setup.bash &&
  colcon build --symlink-install --parallel-workers 1
"
```

运行（容器内）：
```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  ros2 launch uav_bridge uav_delta_system.launch.py use_mock_fcu:=true start_mavros:=false
"
```

## 关键配置

- delta_kinematics: src/delta_kinematics/config/delta_kinematics.yaml (L1, L2, R, r)
- uav_bridge: src/uav_bridge/config/uav_bridge.yaml (serial_port, baudrate)
- mock_mavros_pose: src/uav_bridge/config/mock_mavros_pose.yaml
- failsafe: src/uav_bridge/config/failsafe.yaml (safe_x, safe_y, safe_z)

## 依赖

rclcpp, geometry_msgs, tf2, tf2_ros, mavros, eigen3, sensor_msgs, std_msgs, stai_mpu (vision_test)

## vision_test 视觉推理测试

构建并运行：
```bash
docker exec -it ros2humble bash -c "
  cd /workspace/uav_delta_capture &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select vision_test --symlink-install --parallel-workers 1 &&
  source install/setup.bash &&
  ros2 launch vision_test vision_test.launch.py
"
```

直接在板子上运行（不依赖 Docker）：
```bash
cd /usr/local/Uav_Delta_capture/uav_delta_capture
python3 -m vision_test.bench_node --ros-args -p use_npu:=true
```

参数：
- `mode:=synthetic`（默认）合成图像测试 | `mode:=subscribe` 订阅真实相机
- `input_size:=320` 推理输入分辨率
- `use_npu:=true`（默认）使用 stai_mpu NPU 加速
- `num_iterations:=100` 推理轮数
- `model_path:=` 模型路径（默认 .nb NPU 模型）

## 注意事项

- 构建用 --parallel-workers 1，板子内存有限
- 不要修改 /usr/local 下的其他文件
- Docker 容器名：ros2humble
