# Delta G-code Servo

## 78arm 双相机手眼对接

实机传感器快照读取逻辑在 `real_machine_test/gamepad_controller.py` 中维护。
`Dual_Camera_HandEye/` 复用这些现有输出做坐标链路核验，不新建控制包，也不替代
本目录的实机控制入口。

当前双相机关系：

- 底座相机观察末端执行器上表面的 AprilTag，用来核验 `base_T_tool`。
- 执行机构侧面相机观察待抓取物体，物体位置通过
  `base_T_tool * tool_T_object_camera * object_camera_T_object` 投影到机械臂基座。
- `tool_T_object_camera` 来自 `part_model_rev/999.STL` 对应装配的 CAD/实测尺寸。

把 Delta Robot 的 `G-code` 轨迹转换成 `LX-225` 总线舵机命令，并支持：

- 导出舵机 JSON
- 通过串口直接执行

这个目录是独立工具，和 `Gcode`、`Bus_Servo`、`Delta-Robot` 同级，目标是打通这条链路：

- `G-code -> XYZ 路径点`
- `XYZ -> Delta 逆运动学关节角`
- `关节角 -> 舵机物理角度`
- `舵机物理角度 -> LX-225 位置值(0..1000)`
- `JSON 导出`
- `串口在线执行`

## 目录结构

```text
Delta_Gcode_Servo/
  delta_gcode_servo/
    __init__.py
    __main__.py
    cli.py
    config.py
    gcode.py
    kinematics.py
    robot.py
    servo.py
  output/
  logs/
  export_servo_json.bat
  run_gcode_servo.bat
  pipeline.ps1
  requirements.txt
```

## 当前约定

- 上层使用角度和路径点，不直接把 `0..1000` 当主接口
- 当前默认：
- 三个舵机 ID 是 `1, 2, 3`
- `0° -> 0`
- `240° -> 1000`
- 每段轨迹使用固定 `time_ms`
- 串口层直接走总线舵机协议，不依赖官方上位机协议

## 安装依赖

```bash
pip install -r requirements.txt
```

## 命令行

导出 JSON：

```bash
python -m delta_gcode_servo export-servo-commands ..\Gcode\delta_path.gcode --time-ms 120
```

在线执行：

```bash
python -m delta_gcode_servo run-gcode ..\Gcode\delta_path.gcode --port COM9 --time-ms 120
```

如果要控制串口打开后的启动等待，可以加 `--connect-delay`：

```bash
python -m delta_gcode_servo run-gcode ..\Gcode\delta_path.gcode --port COM9 --time-ms 120 --connect-delay 0
```

常用参数：

- `--port`：串口号，例如 `COM9`
- `--time-ms`：每段动作的目标执行时间，单位毫秒
- `--connect-delay`：串口打开后开始发送前的等待时间，单位秒
- `--settle-time`：每条命令额外附加等待时间，单位秒

## 一键脚本

导出 JSON：

```bat
export_servo_json.bat ..\Gcode\delta_path.gcode
```

串口执行：

```powershell
.\run_gcode_servo.bat ..\Gcode\delta_path.gcode COM9 120
```

注意：

- 在 PowerShell 里运行当前目录下的批处理，要写 `.\`
- `run_gcode_servo.bat` 当前只暴露 `gcode_path`、`port`、`time_ms`
- 如果要控制 `--connect-delay`，请直接使用 Python 命令行

## 输出文件

`output/` 下会生成：

- `*_servo.json`

`logs/` 下会生成：

- `export_*.log`
- `run_*.log`

JSON 中包含：

- 原始 `XYZ`
- 对应关节角 `joint_angles_deg`
- 对应舵机角 `servo_angles_deg`
- 对应位置值 `servo_positions`
- 每段命令的 `time_ms`

## 运行行为说明

### 1. `time_ms` 的实际含义

`time_ms` 是每条舵机命令的目标动作时间，单位毫秒。

例如：

- `time_ms=120`：每条命令目标执行时间约 `120ms`
- `time_ms=300`：动作更慢
- `time_ms=500`：动作更慢、更稳

当前 `run-gcode` 会在每条命令发送后等待：

```text
time_ms / 1000 + settle_time
```

所以现在命令会按节奏顺序执行，不会像旧实现那样快速连续覆盖，只剩下一阵颤动。

### 2. 启动延迟

旧实现里，串口打开后固定等待 `2.0s`。

现在这个等待已经改成可配置：

- 默认 `connect_delay = 0.2s`
- 如果硬件不需要额外初始化等待，可以直接设成 `0`

推荐：

```bash
python -m delta_gcode_servo run-gcode ..\Gcode\delta_path.gcode --port COM9 --time-ms 120 --connect-delay 0
```

### 3. 动作连续性

原始 `G-code` 点数较少时，舵机会在离散点之间明显跳动，不够丝滑。

现在每段轨迹会先按线性步长自动细分，再转换成舵机命令。
这个步长由 [delta_gcode_servo/config.py](C:/Users/hanjuncheng/Desktop/nodejs/Delta_Gcode_Servo/delta_gcode_servo/config.py) 里的 `step_increment_linear` 控制。

当前默认：

- `step_increment_linear = 0.4`

含义：

- 数值越小，动作越连续
- 但生成的舵机命令会更多，串口负担也会更大

以当前 `..\Gcode\delta_path.gcode` 为例：

- 原始 G-code 运动点数：`49`
- 细分后实际舵机命令数：`843`

如果还想更丝滑，可以继续减小 `step_increment_linear`，例如改成 `0.2`。

## 当前还没做的内容

- 每个舵机的零位偏置标定
- 每个舵机的方向校准
- 安全限位约束
- 回读校验和重试
- 基于 `F` 自动换算每段时间

当前版本先保证链路跑通，再逐步补这些能力。
