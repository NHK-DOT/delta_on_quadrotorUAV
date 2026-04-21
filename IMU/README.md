# IMU 使用说明

## 文件说明

- `wt61c_live_viewer.py`
  WT61C 实时读取脚本。负责串口采集、协议解析、终端打印、实时可视化，以及把最新一帧覆盖写入 JSON。

- `wt61c_latest.json`
  运行脚本后自动生成或自动覆盖的最新数据快照文件。这个文件不是手工编辑文件，每次运行时都会被刷新。

- `build_gamepad.py`
  这个文件不是本次 IMU 读取链路的一部分。如果它是你原来放进来的，就和 IMU 实时查看脚本分开管理。

## 当前默认配置

这份脚本已经按你当前这只 IMU 的实际状态配置好了：

- 串口：`COM4`
- 波特率：`9600`
- 传感器型号：WT61C / WT61 系列兼容协议

注意：

- 你找到的产品资料里写的是“默认 115200 / 默认 100Hz”，但我实际读取到你这只设备时，当前工作参数是 `9600`，而且当前输出频率更接近 `10Hz` 左右。
- 这通常说明设备参数已经被改过，或者当前工作模式不是出厂默认模式。

## 运行前提

建议直接使用你现有的 `base` 环境，因为我已经验证过这个环境里有脚本需要的依赖：

- `pyserial`
- `matplotlib`

## 推荐目录

建议在这个目录下运行：

```powershell
cd C:\Users\hanjuncheng\Desktop\nodejs\IMU
```

## 最常用启动方式

### 1. 图形界面 + 终端打印

```powershell
conda run -n base python wt61c_live_viewer.py
```

作用：

- 打开实时可视化窗口
- 终端持续打印姿态、加速度、角速度、温度
- 自动覆盖写入当前目录下的 `wt61c_latest.json`

### 2. 只看终端，不开图形界面

```powershell
conda run -n base python wt61c_live_viewer.py --no-gui
```

适合：

- 远程终端
- 不想弹窗口
- 只想看实时数值和 JSON 快照

### 3. 跑一小段时间自动退出

```powershell
conda run -n base python wt61c_live_viewer.py --no-gui --duration 10
```

表示：

- 连续采集 10 秒
- 自动退出

这个模式适合测试串口是否正常。

## 运行后你会看到什么

### 终端输出

终端会持续打印类似下面的数据：

```text
RPY=(roll, pitch, yaw)
A=(ax, ay, az)
G=(gx, gy, gz)
T=temperature
rate=sample rate
```

其中：

- `RPY` 是欧拉角，单位是度
- `A` 是加速度，单位是 `g`
- `G` 是角速度，单位是 `deg/s`
- `T` 是温度，单位是摄氏度
- `rate` 是脚本估算到的当前输出频率

### 图形界面

图形界面里有四块内容：

- 左侧姿态仪
  直观看当前滚转、俯仰、偏航变化

- 右上欧拉角曲线
  实时看 `roll / pitch / yaw`

- 左下加速度曲线
  实时看 `ax / ay / az`

- 右下角速度曲线
  实时看 `gx / gy / gz`

## JSON 快照文件说明

默认会覆盖写入当前目录下的：

- [wt61c_latest.json](C:/Users/hanjuncheng/Desktop/nodejs/IMU/wt61c_latest.json)

这个文件适合：

- 给其他程序读取
- 调试时查看最新一帧
- 后续和逆运动学期望姿态做对比

文件里包含：

- 串口信息
- 时间戳
- 温度
- 当前估计输出频率
- 三轴加速度
- 三轴角速度
- 欧拉角
- 原始 WT61 帧载荷

## 常用可选参数

### 改串口

```powershell
conda run -n base python wt61c_live_viewer.py --port COM5
```

### 改波特率

```powershell
conda run -n base python wt61c_live_viewer.py --baud 115200
```

### 改快照输出文件

```powershell
conda run -n base python wt61c_live_viewer.py --snapshot-file C:\Users\hanjuncheng\Desktop\imu_latest.json
```

### 改图表显示历史长度

```powershell
conda run -n base python wt61c_live_viewer.py --history-seconds 30
```

表示图里保留最近 30 秒历史。

### 改刷新速度

```powershell
conda run -n base python wt61c_live_viewer.py --refresh-ms 50
```

## 这套脚本适合做什么

这套脚本最适合做：

- 检查 IMU 是否在线
- 看末端执行机构姿态是否变化
- 验证逆运动学算出的目标姿态是否基本实现
- 给后续控制程序提供一个“最新姿态快照”

## 这套脚本不适合做什么

它不适合单独验证：

- 末端绝对位置
- 空间位移
- 高精度轨迹重建

原因很简单：

- WT61C 给你的是姿态和惯性量
- 单个 IMU 不等于可靠的位置传感器

所以如果你后面要验证完整正逆运动学，建议至少区分两类验证：

- 姿态验证：这只 IMU 可以直接做
- 位置验证：需要编码器、视觉、动捕、或者别的外部基准

## 常见问题

### 1. 打不开串口

通常是这几种情况：

- 串口号不对
- 另一个程序已经占用了 `COM4`
- 设备重新插拔后串口号变化了

先去设备管理器确认当前串口号，再重新运行。

### 2. 打开了但没数据

优先检查：

- 波特率是否仍然是 `9600`
- 接线是否松动
- 模块是否还在持续输出姿态数据

### 3. 姿态数值不对

先确认：

- IMU 的安装方向和你的机构坐标系定义是否一致
- 是否需要做安装偏置标定
- 机械结构附近是否有明显磁干扰

## 当前推荐用法

如果你现在只是想快速把它接到现有工程里，最直接的流程是：

1. 进入 `IMU` 文件夹
2. 运行 `wt61c_live_viewer.py`
3. 看终端输出和图形界面
4. 让其他程序直接读取 `wt61c_latest.json`

这样你现有的多机控制、逆运动学、手柄控制模块都不用先大改，就能先把 IMU 数据挂进来。
