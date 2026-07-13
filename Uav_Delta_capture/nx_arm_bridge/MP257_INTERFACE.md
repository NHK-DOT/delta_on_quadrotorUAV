# NX–STM32MP257 Interface Contract

## English

**Transport:** NX UDP sender to MP257 UDP listener, default port `5005`.

**Protocol:** `78arm.nx-arm-bridge/v1`. The data is observation-only: target offset, confidence, distance, arm state, health, timestamp, and sequence. The MP257 must reject unknown versions, malformed values, stale timestamps, and packets that encode control commands.

**Forbidden NX data:** FCU modes, arm/disarm requests, velocity setpoints, waypoint updates, landing commands, and any authority-changing request.

**MP257 behavior:** `vision_bridge` republishes validated observations into ROS 2. `uwb_navigation` alone decides whether a grasp wait is satisfied and whether flight continues. Timeout, `FAILED`, or invalid input must hold or abort under the MP257 mission safety policy.

## 中文

**传输方式：** NX UDP 发送端到 MP257 UDP 监听端，默认端口 `5005`。

**协议：** `78arm.nx-arm-bridge/v1`。数据仅为观测：目标偏移、置信度、距离、机械臂状态、健康状态、时间戳和序号。MP257 必须拒绝未知版本、格式错误、过期时间戳和任何携带控制命令的包。

**NX 禁止发送：** 飞控模式、解锁/上锁、速度设定点、航点、降落命令，以及任何改变飞行控制权限的请求。

**MP257 行为：** `vision_bridge` 将已校验观测重新发布为 ROS 2 话题；只有 `uwb_navigation` 决定抓取等待是否满足和任务是否继续。超时、`FAILED` 或非法输入必须由 MP257 的任务安全策略保持/中止。
