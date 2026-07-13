# NX 视觉与机械臂桥接包

本文件为中文独立入口。NX 只发送视觉/抓取观测，STM32MP257 保持对 ROS 2、UWB、MAVROS 与所有飞控决策的唯一控制权。

## 分工

- **NX**：扳手/AprilTag 视觉推理、手眼标定、Delta 机械臂、蓝牙手柄和本地诊断。
- **STM32MP257**：ROS 2、UWB、MAVROS、飞行状态机、FCU 指令、返航与降落。

## 启动

```bash
source /home/nvidia/.venvs/78arm-py38/bin/activate
export MP257_HOST=<STM32MP257_IP>
python3 /home/nvidia/Desktop/78arm/Uav_Delta_capture/nx_arm_bridge/nx_vision_arm_bridge.py
```

首次联调必须先使用 `--dry-run --once` 检查包格式，再由 MP257 在不连接飞控的 ROS mock/bench 模式下验证话题。
