# uwb_driver - UWB 数据采集包

UWB AOA 模块数据采集、解析与滤波。

## 功能

- 从 ALX-AOA-FIT UWB 模块读取 AOA 数据
- 解析 37 字节帧协议（命令 0x2001）
- 卡尔曼滤波平滑距离和角度数据
- 发布标准化 UwbAoa 消息

## 节点

### uwb_aoa_driver_node

UWB 数据采集主节点。

**发布话题：**
- `uwb_aoa/data`（`UavDeltaMsgs/UwbAoa`）- 滤波后的 UWB 数据

**参数：**
- `serial_port`（string）- 串口路径，代码默认 `/dev/ttyUSB0`
- `serial_baud`（int）- 波特率，默认 `115200`
- `uwb_aoa_topic`（string）- 输出话题，默认 `uwb_aoa/data`
- `signal_loss_timeout_sec`（double）- 信号超时阈值，默认 `0.2`
- `kalman_Q`（double）- 过程噪声，默认 `0.1`
- `kalman_R`（double）- 测量噪声，默认 `0.1`

## 使用

从 Windows 项目根目录执行：

```bash
# 上传并编译
python sync_to_board.py
python ssh2board.py "docker exec ros2humble bash -lc 'pkill -f [u]wb_aoa_driver_node || true; source /opt/ros/humble/setup.bash && cd /workspace/uav_delta_capture && colcon build --packages-select uwb_driver --parallel-workers 2'"

# 单独启动 UWB driver。当前板子上 UWB 经 CP2102 USB-TTL 接入，设备节点是 /dev/ttyUSB0。
python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 run uwb_driver uwb_aoa_driver_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baud:=115200 > /tmp/uwb_aoa_driver.log 2>&1'"

# 或使用 launch/config 启动
python ssh2board.py "docker exec -d ros2humble bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/uav_delta_capture/install/setup.bash && ros2 launch uwb_driver uwb_aoa_driver.launch.py > /tmp/uwb_aoa_driver.log 2>&1'"
```

在容器里手动执行时：

```bash
source /opt/ros/humble/setup.bash
source /workspace/uav_delta_capture/install/setup.bash
ros2 run uwb_driver uwb_aoa_driver_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baud:=115200
```

## 监测

```bash
# 查看驱动日志
python ssh2board.py "docker exec ros2humble bash -lc 'tail -80 /tmp/uwb_aoa_driver.log'"

# 看 UWB 数据内容
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /uwb_aoa/data --once'"

# 看 UWB 数据频率
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && timeout 10 ros2 topic hz /uwb_aoa/data'"

# 看话题信息
python ssh2board.py "docker exec ros2humble bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic info /uwb_aoa/data'"

# 查看串口设备
python ssh2board.py "ls -l /dev/ttySTM* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null"
```

如果 `/uwb_aoa/data` 一直有消息但 `signal_valid: false` 或 `quality: 0.0`，说明驱动在发布超时/无效状态，先检查 UWB 模块供电、串口、tag 是否在可测范围内。

## 停止和清残留

```bash
# 温和停止
python ssh2board.py "docker exec ros2humble bash -lc 'pkill -f [u]wb_aoa_driver_node || true'"

# 强制停止
python ssh2board.py "docker exec ros2humble bash -lc 'pkill -9 -f [u]wb_aoa_driver_node || true'"

# 检查残留
python ssh2board.py "docker exec ros2humble bash -lc 'ps -eo pid,ppid,cmd | grep -E \"uwb_aoa_driver_node|uwb_driver\" | grep -v grep || true'"
```

## 配置

配置文件：`config/uwb_aoa_driver.yaml`

关键参数：

- `serial_port`: 串口路径，当前实机是 CP2102 USB-TTL 的 `/dev/ttyUSB0`
- `serial_baud`: 波特率，当前是 `115200`
- `uwb_aoa_topic`: 输出话题，默认 `uwb_aoa/data`
- `signal_loss_timeout_sec`: 多久没收到新帧后标记为无效
- `kalman_Q` / `kalman_R`: 距离和方位角滤波参数

## UWB 协议

帧格式（37 字节）：
```
帧头(2B) + 长度(2B) + 命令(2B) + 数据(28B) + 校验(1B) + 帧尾(2B)
```

数据字段偏移：
- [6:10] Distance - uint32, cm
- [10:12] Azimuth - int16, 度（±90°）
- [12:14] Elevation - int16, 度（±30°）
