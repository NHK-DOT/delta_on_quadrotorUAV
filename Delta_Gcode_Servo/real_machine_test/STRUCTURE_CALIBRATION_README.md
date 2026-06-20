# Delta 新结构工作空间标定采样说明

本流程用于更换机械结构后的 Delta 机械臂重新建模。脚本会自动启动 IMU 读取和 AprilTag 识别后台进程，然后把舵机 raw、视觉工具点位置、IMU 姿态和结构尺寸写入采样数据集。

## 安全原则

- 主采样脚本只读取舵机 raw，不发送任何舵机运动命令。
- 脚本启动的 IMU 和 AprilTag 子进程只负责更新 JSON 快照。
- 主程序正常退出、输入 `q`、或按 `Ctrl+C` 中断时，会一起关闭 IMU 和 AprilTag 子进程。
- 最下点是安全运动空间下沿，不向三臂伸直、连杆死点方向继续留余量。
- 相机不需要看到执行机构中心圆盘，只要能稳定看到安装在执行机构上的 AprilTag。
- raw 安全范围和 home 参考位已经拆开，不能再假设 `raw_max` 就是初始位。

## 坐标约定

- 相机向下照。
- 相机测量原点记为 `0`，作为本轮采样的 base 原点。
- `X+` 为向右。
- `Y+` 为向前。
- `Z+` 为向上。
- 相机 0 平面在舵机转轴下方，所以舵机转轴中心相对相机 0 平面的 `servo_axis_z_offset_mm` 是正数。

如果后续发现相机测量原点没有正好落在三个舵机转轴中心三角形的外接圆圆心上，拟合时需要额外估计一个相机原点到 Delta 机械中心的 `x/y` 平移偏置。

## 需要量的尺寸

启动脚本后需要输入，单位都是 mm：

- `upper_arm_mm`：大臂/主动臂长度，舵机转轴中心到肘部铰点中心。
- `lower_arm_mm`：小臂/从动臂长度，肘部铰点中心到末端平台铰点中心。
- `platform_radius_mm`：执行机构三小臂末端连接点组成三角形的外接圆半径。
- `servo_axis_radius_mm`：顶部三个舵机转轴中心组成三角形的外接圆半径，可暂时留空。
- `servo_axis_z_offset_mm`：舵机转轴中心相对相机 0 平面的 Z 高度，可暂时留空。

前三项必须填。后两项如果现在不好量，可以先留空，后续拟合时再补。

## 一键运行

默认会自动启动：

- `IMU/wt61c_live_viewer.py`
- `AprilTag_Vision/myAprilTag/src/apriltag_usb_detector.py`

运行：

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm\Delta_Gcode_Servo\real_machine_test
python structure_calibration_sampler.py --port COM15
```

默认端口：

- 舵机总线：`COM15`
- IMU：`COM16`

如果 AprilTag ID 固定，比如 `3`：

```powershell
python structure_calibration_sampler.py --port COM15 --hand-tag-id 3
```

如果相机不是配置文件里的默认索引：

```powershell
python structure_calibration_sampler.py --port COM15 --apriltag-camera-index 1
```

如果你已经手动启动了 IMU 和 AprilTag，不希望脚本自动启动后台进程：

```powershell
python structure_calibration_sampler.py --port COM15 --no-autostart-sensors
```

## 退出和子进程清理

推荐用以下任一方式退出：

- 在 `采样标签 >` 输入 `q`
- 按 `Ctrl+C`

脚本会依次关闭 AprilTag 和 IMU 子进程。如果普通关闭超时，会强制结束。

子进程日志写入：

```text
Delta_Gcode_Servo/real_machine_test/structure_calibration_samples/logs/
```

主要日志：

- `imu.log`
- `apriltag.log`

如果摄像头打不开、IMU 串口被占用、或者 AprilTag 没识别到，先看这里。

## 推荐采样点

先采这些基础点：

- `top_home`：最上，接近初始位。
- `bottom_safe`：安全最下沿。
- `center_mid`：中心中层。
- `left_mid`：最左中。
- `right_mid`：最右中。
- `front_mid`：最前中。
- `back_mid`：最后中。

然后补四个斜向中层点：

- `left_front_mid`
- `right_front_mid`
- `left_back_mid`
- `right_back_mid`

每个点手动摆好后，在脚本提示后输入标签：

```text
采样标签 > bottom_safe
```

脚本会同时记录：

- 舵机 `raw1/raw2/raw3`
- 视觉估计的 `x_mm/y_mm/z_mm`
- AprilTag 检测 ID 和快照年龄
- IMU roll/pitch/yaw
- 几何尺寸

## 输出文件

默认输出目录：

```text
Delta_Gcode_Servo/real_machine_test/structure_calibration_samples/
```

主要文件：

- `samples.csv`：用于查看和拟合的表格。
- `samples.jsonl`：完整原始记录，包括视觉链、IMU 和 raw。
- `geometry.json`：本次输入的结构尺寸。
- `logs/imu.log`：IMU 子进程日志。
- `logs/apriltag.log`：AprilTag 子进程日志。

## 快照新鲜度

脚本启动子进程后会等待 IMU 和 AprilTag 快照变新。默认要求快照年龄小于 `3000 ms`。

如果启动后提示快照不新鲜，说明对应后台程序没有正常更新 JSON。常见原因：

- 摄像头索引不对。
- AprilTag 没出现在画面里。
- IMU 端口不对或被占用。
- 手眼标定文件路径不对。

## 后续拟合思路

采样完成后，用 `samples.csv/jsonl` 做离线拟合：

1. 先修正 raw 到关节角的零位、方向和比例。
2. 再拟合 Delta 几何参数，例如大臂、小臂、执行机构外接圆半径、舵机轴半径和 Z 偏置。
3. 用拟合后的 FK 计算每个采样点的预测 XYZ。
4. 比较预测 XYZ 和视觉 XYZ 的误差。
5. 误差可接受后，再更新控制器的 IK/FK 参数和工作空间边界。

初始目标可以先定为中心区域误差小于 5 到 10 mm，边界区域误差小于 15 到 20 mm。
