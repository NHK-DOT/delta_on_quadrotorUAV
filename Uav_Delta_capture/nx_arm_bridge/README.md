# NX Vision and Arm Bridge

## English

This package is the only NX-to-STM32MP257 integration boundary for the UAV mission. NX owns camera inference, hand-eye calibration, Delta-arm execution, and Bluetooth diagnostics. STM32MP257 owns ROS 2, UWB, MAVROS, flight-state transitions, arming, velocity setpoints, return, and landing.

The bridge sends versioned UDP observations to the MP257 vision bridge. A packet contains only a target offset, optional distance, arm state, and health/timestamp metadata. It never listens for commands and never sends FCU, arm/disarm, flight-mode, or velocity messages.

## 中文

本包是无人机任务中唯一允许的 NX→STM32MP257 集成边界。NX 负责相机推理、手眼标定、Delta 机械臂和蓝牙诊断；STM32MP257 负责 ROS 2、UWB、MAVROS、飞行状态机、解锁、速度指令、返航和降落。

桥接程序只发送带版本号的 UDP 观测包，其中仅包含目标偏移、距离、机械臂状态和健康/时间戳元数据；它不监听控制命令，绝不发送飞控、解锁、模式切换或速度指令。
