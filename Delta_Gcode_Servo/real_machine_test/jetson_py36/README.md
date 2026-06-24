# Jetson Xavier NX Python 3.6 AprilTag 工作空间采样包

这个目录是给 `192.168.1.80` 那台 Jetson Xavier NX 用的现场包。那台机器系统 Python 是 `3.6.9`，不能直接跑主线里 Python 3.11 风格的控制脚本，所以这里保留一套 Python 3.6 兼容入口。

目标是把三件事放在同一个现场流程里：

- 3K 鱼眼降采样 AprilTag 识别，读取 Jetson 本地 JSON。
- 8BitDo 蓝牙手柄低速遥控机械臂小范围移动。
- 按 `B` 采样当前 AprilTag 工具点 XYZ、舵机 raw 反馈、旧 FK 位置和视觉-FK 偏置。

默认 Jetson 路径：

```bash
/home/nvidia/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
```

## 从本机部署到 Jetson

在本机仓库根目录或任意位置运行：

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm
python Delta_Gcode_Servo\real_machine_test\jetson_py36\deploy_to_jetson.py --password nvidia
```

部署脚本会复制最小运行包到：

```text
/home/nvidia/Desktop/78arm
```

并在 Jetson 上执行 Python 编译检查。它不会复制 `Jetson_Vision_Export/saved_runs`、历史 samples、日志等生成物。

## 工作空间标定流程

这是 `192.168.1.80` Jetson Xavier NX 的现场主线流程：先做只读自检，确认
AprilTag、8BitDo、舵机反馈和启动位都正常；再进入低速手动采样；最后用采样
数据拟合模型并扫描工作空间。

最短闭环如下。

登录 Jetson：

```bash
ssh nvidia@192.168.1.80
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
HAND_TAG_ID=3 bash run_sampler_py36_jetson.sh
```

这个脚本会先启动 3K AprilTag 进程，再跑只读自检，全部通过后才进入采样控制。
进入控制后，移动到标定点并按 `B` 记录当前 AprilTag 工具点 XYZ 和舵机 raw。

如果 AprilTag 已经手动启动：

```bash
NO_AUTOSTART_APRILTAG=1 HAND_TAG_ID=3 bash run_sampler_py36_jetson.sh
```

如果串口不是 `/dev/ttyUSB0`：

```bash
PORT=/dev/ttyUSB1 HAND_TAG_ID=3 bash run_sampler_py36_jetson.sh
```

## 只读自检

单独跑自检，不发送任何移动命令：

```bash
python3 jetson_workspace_preflight.py --port /dev/ttyUSB0 --hand-tag-id 3
```

自检项：

- Python 版本和 `pyserial` 是否可用。
- 3K AprilTag JSON 是否存在、是否新鲜、是否包含 `--hand-tag-id`。
- `Dual_Camera_HandEye/output/calibration_result.json` 是否能把 tag 位姿换算成工具点 XYZ。
- 8BitDo 是否能在 `/dev/input/event*` 下打开。
- 舵机板 `/dev/ttyUSB0` 是否能读回 servo 1/2/3 raw 和电压。
- 当前 raw 是否接近 `lx225_tool_demo/config/lx225_tool.demo.toml` 里的 `startup_check_raw`。

默认启动位容差是 `25 ticks`。如果不在启动自检位，采样脚本默认拒绝启动。

舵机驱动板 `0x15` 反馈按有符号 int16 解释。例如 `0xFF43` 应看作 `-189`，不是无符号的 `65347`。`startup_check_raw` 是只读自检位，`home_raw` 是采样控制放行后的运动映射参考位。当前拆装后的 home 反馈已经写入配置：servo 1/2/3 分别是 `813`、`457`、`-189`。

## 手柄控制

进入采样控制后会再次要求输入 `YES`，确认后才允许发送低速舵机命令。

| 控制 | 动作 |
| --- | --- |
| D-pad 左/右 | X 小范围移动 |
| D-pad 上/下 | Y 小范围移动 |
| 右摇杆 Y | Z 小范围移动 |
| B | 采样当前点 |
| X | 切换 safe scan: FREE/X/Y/Z |
| A | 退出 |

默认速度：

```text
XY: 35 mm/s
Z: 25 mm/s
servo raw limit: 180 raw/s
```

## 输出数据

每次运行默认输出到：

```text
jetson_py36/samples/YYYYMMDD_HHMMSS/
```

关键文件：

| 文件 | 用途 |
| --- | --- |
| `preflight_report.json` | 一键脚本启动前的自检结果 |
| `samples.csv` | 快速查看和模型拟合用表格 |
| `samples.jsonl` | 完整采样数据，每行一个点 |
| `session.json` | 本次运行参数、输入和输出路径 |
| `runtime_status.log` | 实时状态覆盖写入 |
| `debug.log` | 串口/IK/反馈错误日志 |
| `logs/jetson_apriltag3k.log` | 3K AprilTag 子进程日志 |

`samples.csv` 关键列：

```text
raw1/raw2/raw3
fk_x_mm/fk_y_mm/fk_z_mm
x_mm/y_mm/z_mm
offset_x_mm/offset_y_mm/offset_z_mm
vision_detection_id
vision_snapshot_age_ms
```

其中 `x_mm/y_mm/z_mm` 是 AprilTag 视觉链路估计的工具点位置，`fk_*` 是当前旧模型根据舵机 raw 反馈计算出来的位置。

## 离线拟合和工作空间扫描

推荐在本机或 Python 3.11 环境跑离线模型工具：

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test
python3 workspace_model_tools.py fit \
  jetson_py36/samples/YYYYMMDD_HHMMSS/samples.jsonl \
  --output-dir jetson_py36/samples/YYYYMMDD_HHMMSS/model \
  --compute-workspace \
  --workspace-step-mm 5
```

输出：

```text
model/fit_report.json
model/fit_residuals.csv
model/workspace_grid.csv
model/workspace_summary.json
```

如果没有 SciPy，可以强制无 SciPy 搜索：

```bash
python3 workspace_model_tools.py fit samples.jsonl --no-scipy --compute-workspace
```

已有 `fit_report.json` 时，单独重新计算工作空间：

```bash
python3 workspace_model_tools.py workspace \
  --model model/fit_report.json \
  --output-dir model \
  --step-mm 5
```

正逆解接口在主线已有：

```python
from delta_gcode_servo.kinematics import inverse_kinematics, forward_kinematics
from delta_gcode_servo.config import robot_params

params = robot_params()
angles_rad, ok = inverse_kinematics(x_mm, y_mm, z_mm, params)
xyz_mm, ok = forward_kinematics(theta1_rad, theta2_rad, theta3_rad, params)
```

`workspace_model_tools.py` 复用这套 FK/IK，并用采样数据拟合 `l1/l2/l3/servo_offset_x/servo_offset_z` 和视觉常量偏置，再网格扫描整个可达空间。

## 现场判断标准

先采这些点，每点可以按 `B` 采 2 到 3 次：

```text
top_home
center_mid
bottom_safe
left_mid
right_mid
front_mid
back_mid
left_front_mid
right_front_mid
left_back_mid
right_back_mid
```

拟合后重点看：

- `evaluation.rms_residual_norm_mm`
- `evaluation.max_residual_norm_mm`
- `fit_residuals.csv` 是否有单方向系统偏差
- `workspace_summary.json` 的 `suggested_controller_bounds`

不要只因为拟合能跑就立刻改控制器参数。先确认 AprilTag 坐标轴、tag 尺寸、3K 标定、`tool_T_hand_tag` 和重复采样抖动都合理。
