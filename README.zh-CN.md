# 78arm 中文说明

English README: [README.md](README.md)

## 项目定位

这个仓库用于整理无人机挂载轻量 Delta 机械臂相关的代码和资料，包括机械臂仿真、舵机控制、总线舵机调试、IMU、AprilTag 视觉识别，以及当前正在使用的 8BitDo 蓝牙手柄实机控制包。

当前重点是 `bt_8bitdo_min/`。它不是单纯读取手柄状态的演示程序，而是一条可以控制真实舵机的链路：机载 Ubuntu 18.04 Nano 读取蓝牙手柄输入，把操作量转换成机械臂末端 XYZ 运动，再经过 Delta 机械臂逆运动学、舵机映射和串口协议，最终通过幻尔 xArm 1.6 舵机驱动板驱动物理总线舵机。

## 为什么使用 8BitDo Ultimate 2 Wireless

一开始尝试过 Xbox Series 无线手柄，但在这块 Nano 板子的 Ubuntu 18.04 蓝牙环境里没有稳定解决兼容问题。实际表现是手柄会在连接和断开之间反复跳变，无法作为实机控制输入使用。

因此当前改用 8BitDo Ultimate 2 Wireless 手柄。这个方案不依赖 pygame 或 SDL 的手柄映射，而是直接读取 Linux 的 `/dev/input/eventX` 事件设备，并通过配置文件维护按键和轴映射。这样更适合 Ubuntu 18.04 Nano 这种旧系统和嵌入式部署环境。

## 机械臂实物照片

| Delta 机械臂整体 | 执行机构侧 |
| --- | --- |
| <img src="images/1.jpg" alt="Delta 机械臂整体" width="420"> | <img src="images/884b798faf516a24bb9bb0af58b4d616.jpg" alt="Delta 机械臂执行机构侧" width="420"> |

## 实际硬件控制链路

```text
8BitDo Ultimate 2 Wireless 手柄
  -> Ubuntu 18.04 Nano 蓝牙输入设备 /dev/input/eventX
  -> bt_8bitdo_min 的 evdev 读取器
  -> 实时 Delta 机械臂控制器
  -> 末端 XYZ 目标和逆运动学解算
  -> LX 总线舵机原始位置值
  -> USB 转串口模块
  -> 幻尔 xArm 1.6 舵机驱动板
  -> 实物 LX 总线舵机
```

需要注意：程序最终会通过幻尔 xArm 1.6 舵机驱动板向舵机发送实际运动命令。也就是说，`run_control_bt.sh` 不是只读测试脚本，它可以让机械臂真实运动。运行前必须确认机械臂处在安全参考位，周围没有障碍物，舵机供电和串口连接正常。

## 目录说明

- `bt_8bitdo_min/`：当前最小 8BitDo 蓝牙手柄控制包，包含只读测试、映射检查、串口预检和实机控制入口。
- `Delta_Gcode_Servo/`：Delta 机械臂 G-code、逆运动学和总线舵机控制工具。
- `Delta-Robot/`：原始 Delta 机械臂仿真和模型资源。
- `lx225_tool_demo/`：LX-225 总线舵机工具和 demo 配置。
- `IMU/`：WT61C IMU 读取、可视化和 JSON 快照输出工具。
- `AprilTag_Vision/`：AprilTag 视觉识别、相机标定和定位输出工具。
- `Bus_Servo/`：总线舵机示例和二维云台相关工具。

## 推荐运行流程

进入当前手柄控制包：

```bash
cd ~/Desktop/78arm/bt_8bitdo_min
```

安装依赖和权限规则：

```bash
bash deploy/install_ubuntu18.sh
```

第一次安装后建议退出登录再重新登录，使 `input` 组权限生效。

如果脚本提示没有配置手柄 MAC 地址，需要先手动配对一次：

```bash
bluetoothctl
power on
agent on
default-agent
scan on
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
quit
```

然后把实际 MAC 写入 `config/bluetooth_mac.conf`。

## 只读测试流程

先做一次手柄日志采集：

```bash
bash deploy/run_log_test.sh 30
```

30 秒内按一遍方向键、右摇杆 Y 轴、`A/B/X/Y/LB/RB`。采集结果会覆盖写入：

- `logs/gamepad_once.log`
- `logs/gamepad_once.json`

检查采集结果是否覆盖实机控制需要的输入：

```bash
bash deploy/run_mapping_check.sh
```

如果有项目显示 `MISSING`，重新采集并补按对应按键或摇杆。

也可以打开只读实时状态显示：

```bash
bash deploy/run_show_state.sh
```

这个脚本只读取手柄，不打开舵机串口，因此不会移动机械臂。

## 串口和舵机预检

在实机控制前，先读取舵机驱动板和舵机反馈：

```bash
bash deploy/run_serial_check.sh --port /dev/ttyUSB0
```

这个脚本会打开串口，读取 1/2/3 号舵机位置和电压信息，但不会发送运动命令。

需要做小幅写入动作检查时，再运行：

```bash
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0
```

它会读取当前舵机位置，小幅移动，然后返回起点。这个步骤已经属于写命令测试，运行前要确认机械臂处在安全空间内。

## 实机控制

只有在手柄映射检查和串口预检都通过后，才运行：

```bash
bash deploy/run_control_bt.sh --port /dev/ttyUSB0
```

默认控制映射：

- 方向键 X/Y：控制机械臂末端 X/Y 方向移动
- 右摇杆 Y：控制机械臂末端 Z 方向移动
- `A`：退出
- `B`：记录当前点
- `X`：切换安全扫描模式
- `Y`：切换传感器坐标系模式
- `LB/RB`：末端工具舵机闭合/打开

程序会自动寻找当前的 `/dev/input/eventX`，不要把设备号固定写死成 `event8`。如果只是日常使用，保持 `config/gamepad_8bitdo_bt.json` 里的 `device.device_path` 为空即可。

## 安全注意事项

- `run_log_test.sh`、`run_mapping_check.sh`、`run_show_state.sh` 是只读路径，不会移动舵机。
- `run_serial_check.sh` 会打开串口并读取反馈，但不发送运动命令。
- `run_serial_move_check.sh` 和 `run_control_bt.sh` 会发送实际舵机运动命令。
- 运行实机控制前，必须确认机械臂在安全参考姿态，舵机 1/2/3 在线，周围无障碍物。
- 如果出现逆运动学失败、串口异常、舵机反馈异常或手柄断连，应停止运行并重新检查硬件状态。

## Git 同步注意

仓库中存在厂家资料包、视频、安装包、运行日志和缓存文件，这些不适合直接提交到 GitHub。根目录 `.gitignore` 已经忽略 `LX-225 串行总线舵机/`、运行日志、缓存、压缩包、安装包和视频文件。提交前建议先运行：

```bash
git status --short --ignored
git add -n .
```

确认不会把大文件或本地运行产物加入暂存区后，再正式提交。
