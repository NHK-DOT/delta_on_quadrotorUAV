# 78arm 中文说明

English README: [README.md](README.md)

开源协议：GNU GPL v3.0，见 [LICENSE](LICENSE)。上游 MIT 项目的版权声明保留在
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 项目定位

这个仓库整理无人机挂载轻量 Delta 机械臂相关的代码和资料，包括机械臂仿真、舵机控制、总线舵机调试、IMU、AprilTag 视觉识别、双相机手眼链路，以及当前正在使用的 8BitDo 蓝牙手柄实机控制包。

当前实机控制重点是 `bt_8bitdo_min/`：机载 Ubuntu 18.04 Nano 读取蓝牙手柄输入，把操作量转换成机械臂末端 XYZ 运动，再经过 Delta 机械臂逆运动学、舵机映射和串口协议，最终通过幻尔 xArm 1.6 舵机驱动板驱动物理总线舵机。

本项目的 Delta 机械臂结构思路和早期建模/控制方向借鉴自 Isaac Chasteau 的 MIT 开源项目 [isaac879/Delta-Robot](https://github.com/isaac879/Delta-Robot)。本仓库里的建模、硬件布局、控制代码、传感器链路和部署方式已经按当前机械臂做了修改，但仍保留对上游项目的来源声明和 MIT 版权声明。

## 机械臂实物与模型照片

| 实机框架与电子部分 | 手持检查连杆机构 |
| --- | --- |
| <img src="images/1.jpg" alt="Delta 机械臂实机框架与电子部分" width="420"><br>实机框架、连杆、控制板和机载走线。 | <img src="images/884b798faf516a24bb9bb0af58b4d616.jpg" alt="Delta 机械臂手持检查连杆机构" width="420"><br>轻量 Delta 机械臂装配后进行手持检查。 |
| <img src="images/9b5124927711c6a065732a5374151702.jpg" alt="Delta 机械臂连杆与改版打印件" width="420"><br>连杆/末端侧，展示已安装的改版打印件。 | <img src="images/0cb198a8a6041f6031b36bc2a0e89fff.jpg" alt="改版连杆座 CAD 概念" width="420"><br>改版连杆座/安装件 CAD 设计。 |
| <img src="images/ed630aaf206b2373b458c409e840b7ce.jpg" alt="改版末端平台 CAD" width="420"><br>改版末端平台与轴承/连杆安装几何。 | <img src="images/bc97d03f7ef3bbd601feaae3bde8008b.jpg" alt="可打印或 CNC 的改版板件" width="420"><br>平面板件模型，可 3D 打印，也可导出后做 CNC。 |

## 机械建模和制造文件

改版机械文件放在 `part_model_rev/`。该目录包含 SolidWorks 零件文件（`.SLDPRT`）、`.3mf` 打印布局，以及当前用于固定 IMU 和末端上表面 AprilTag 的 `999.STL`。

- `.3mf` 可以作为 3D 打印的起点。
- `999.STL` 用于当前 IMU + 末端上表面 AprilTag 固定件，是双相机手眼链路里的机械基准。
- `.SLDPRT` 用于继续改尺寸、改孔位、改装配关系。
- 需要打印时导出 STL/3MF；需要 CNC 时可以从 CAD 导出 STEP/DXF，再生成 CAM 加工路径。
- 打印或加工前要按真实硬件复核孔径、轴承配合、舵机避让、碳管连杆尺寸和装配间隙。

## 双相机手眼布局

`Dual_Camera_HandEye/` 记录当前手眼协同方案：

- 底座相机固定在底座/机架上，观察末端执行器上表面的 AprilTag，用来估计或核验 `base_T_tool`。
- 末端执行器下表面连接抓取机构。
- 执行机构侧面的相机观察待抓取物体。这个相机的固定安装外参写成 `tool_T_object_camera`，优先从 CAD/装配测量得到，不再假设它要观察底座 AprilTag。

这个 demo 复用现有输出：

- `AprilTag_Vision/myAprilTag/output/apriltag_latest.json`
- `IMU/wt61c_latest.json`
- `Delta_Gcode_Servo/real_machine_test/gamepad_controller.py` 里的传感器快照读取路径

它只做坐标链路和离线核验，不打开舵机串口，也不发送运动命令。

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

`run_control_bt.sh` 会通过舵机驱动板发送真实运动命令。运行前必须确认机械臂处在安全参考位，周围没有障碍物，舵机供电和串口连接正常。

## 目录说明

- `bt_8bitdo_min/`：当前最小 8BitDo 蓝牙手柄控制包，包含只读测试、映射检查、串口预检和实机控制入口。
- `Delta_Gcode_Servo/`：Delta 机械臂 G-code、逆运动学和总线舵机控制工具。
- `Delta-Robot/`：原始 Delta 机械臂仿真和模型资源。
- `part_model_rev/`：改版 SolidWorks/3MF/STL 机械文件；`999.STL` 是当前 IMU + AprilTag 固定件。
- `Dual_Camera_HandEye/`：底座相机 + 侧面物体相机的手眼坐标链路 demo，复用现有 AprilTag/IMU 快照。
- `lx225_tool_demo/`：LX-225 总线舵机工具和 demo 配置。
- `IMU/`：WT61C IMU 读取、可视化和 JSON 快照输出工具。
- `AprilTag_Vision/`：AprilTag 视觉识别、相机标定和定位输出工具。
- `Bus_Servo/`：总线舵机示例和二维云台相关工具。

## 只读测试流程

进入当前手柄控制包：

```bash
cd ~/Desktop/78arm/bt_8bitdo_min
```

安装依赖和权限规则：

```bash
bash deploy/install_ubuntu18.sh
```

第一次安装后建议退出登录再重新登录，使 `input` 组权限生效。

采集一次手柄日志：

```bash
bash deploy/run_log_test.sh 30
```

检查采集结果是否覆盖实机控制需要的输入：

```bash
bash deploy/run_mapping_check.sh
```

也可以打开只读实时状态显示：

```bash
bash deploy/run_show_state.sh
```

这个脚本只读取手柄，不打开舵机串口，因此不会移动机械臂。

## 串口和舵机预检

实机控制前先读取舵机驱动板和舵机反馈：

```bash
bash deploy/run_serial_check.sh --port /dev/ttyUSB0
```

这个脚本会打开串口，读取 1/2/3 号舵机位置和电压信息，但不会发送运动命令。

需要做小幅写入动作检查时，再运行：

```bash
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0
```

这一步已经属于写命令测试，运行前要确认机械臂处在安全空间内。

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

程序会自动寻找当前的 `/dev/input/eventX`，不要把设备号固定写死成 `event8`。日常使用时，保持 `config/gamepad_8bitdo_bt.json` 里的 `device.device_path` 为空即可。

## 安全注意事项

- `run_log_test.sh`、`run_mapping_check.sh`、`run_show_state.sh` 是只读路径，不会移动舵机。
- `run_serial_check.sh` 会打开串口并读取反馈，但不发送运动命令。
- `run_serial_move_check.sh` 和 `run_control_bt.sh` 会发送实际舵机运动命令。
- 运行实机控制前，必须确认机械臂在安全参考姿态，舵机 1/2/3 在线，周围无障碍物。
- 如果出现逆运动学失败、串口异常、舵机反馈异常或手柄断连，应停止运行并重新检查硬件状态。

## Git 同步注意

仓库中存在厂家资料包、视频、安装包、运行日志和缓存文件，这些不适合直接提交到 GitHub。提交前建议先运行：

```bash
git status --short --ignored
git add -n .
```

确认不会把大文件或本地运行产物加入暂存区后，再正式提交。
