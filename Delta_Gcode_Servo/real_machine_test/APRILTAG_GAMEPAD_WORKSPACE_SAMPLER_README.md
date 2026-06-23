# AprilTag + 8BitDo 工作空间采样闭环脚本

这个流程用于当前机械结构改动后的工作空间重新采样。它把三件事放在同一个现场脚本里：

- 8BitDo 蓝牙手柄低速遥控机械臂。
- Jetson 3K 鱼眼全 FOV 降采样 AprilTag 识别。
- 按 `B` 时同时记录 AprilTag 的 `base_T_tool` 位置、当前舵机 raw 反馈、FK 位置和视觉-FK 偏置。

脚本位置：

```bash
Delta_Gcode_Servo/real_machine_test/apriltag_gamepad_workspace_sampler.py
Delta_Gcode_Servo/real_machine_test/workspace_model_tools.py
Delta_Gcode_Servo/real_machine_test/run_apriltag_workspace_sampler_jetson.sh
```

## 运行位置

推荐在 `192.168.1.80` 那台 Jetson 上运行最终采样脚本，因为 3K 鱼眼 AprilTag JSON 是 Jetson 本地实时生成的：

```text
/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json
```

本机 `78arm` 仓库负责保留源码、README 和后处理工具。需要先把本机改动同步到 Jetson 的 `78arm` 目录。

## 复用的主线逻辑

现场脚本继承当前主线：

```text
Delta_Gcode_Servo/real_machine_test/gamepad_controller.py
```

所以它继续使用现有的：

- 启动前串口连接和舵机反馈读取。
- 当前反馈不在准备位时输入 `HOME` 后慢速回到参考位。
- FK/IK、raw 舵机映射、工作空间裁剪、target lead 限制。
- 堵转/反馈不变化保护。
- `X` safe-scan 轴锁定、`Y` sensor frame、`LB/RB` 工具舵机。

新增的是：按 `B` 不再只写旧 `workspace_points.csv`，而是写完整 AprilTag 工作空间数据集。

## 手柄映射

默认使用旧包里的 8BitDo evdev 读取器：

```text
bt_8bitdo_min/config/gamepad_8bitdo_bt.json
```

按键：

| 控制 | 动作 |
| --- | --- |
| D-pad 左/右 | X 小范围移动 |
| D-pad 上/下 | Y 小范围移动 |
| 右摇杆 Y | Z 小范围移动 |
| A | 退出 |
| B | 采样当前点 |
| X | 切换 safe-scan: FREE/X/Y/Z |
| Y | 切换 sensor frame |
| LB/RB | 工具舵机，如果 servo4 已配置 |

`START` 的 A/B 回放在这个采样脚本里被禁用，避免未验证数据时误触发自动路径。

## Jetson 一键启动

在 Jetson：

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test
bash run_apriltag_workspace_sampler_jetson.sh
```

如果 AprilTag ID 固定，比如 `3`：

```bash
HAND_TAG_ID=3 bash run_apriltag_workspace_sampler_jetson.sh
```

如果舵机板不是 `/dev/ttyUSB0`：

```bash
PORT=/dev/ttyUSB1 HAND_TAG_ID=3 bash run_apriltag_workspace_sampler_jetson.sh
```

采样脚本默认比普通主线控制更慢：

```text
XY: 35 mm/s
Z: 25 mm/s
servo: 180 raw/s
```

需要更慢时：

```bash
HAND_TAG_ID=3 bash run_apriltag_workspace_sampler_jetson.sh \
  --speed-xy-mm-s 20 --speed-z-mm-s 15 --max-servo-speed-ticks-s 120
```

这个脚本会尝试启动：

```text
/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench/run_fullfov_1280x960_gui.sh
```

它会输出 Jetson 3K 鱼眼全 FOV 降采样 AprilTag 快照。若你已经手动启动 AprilTag 检测，可以加：

```bash
bash run_apriltag_workspace_sampler_jetson.sh --no-autostart-apriltag
```

## 完整命令

```bash
python3 apriltag_gamepad_workspace_sampler.py \
  --port /dev/ttyUSB0 \
  --base-camera-snapshot /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json \
  --apriltag-launch /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench/run_fullfov_1280x960_gui.sh \
  --calibration ~/Desktop/78arm/Dual_Camera_HandEye/output/calibration_result.json \
  --gamepad-config ~/Desktop/78arm/bt_8bitdo_min/config/gamepad_8bitdo_bt.json \
  --hand-tag-id 3
```

## 现场采样步骤

1. 确认机械臂在最上面安全位置附近，周围没有障碍。
2. 确认 8BitDo 已蓝牙连接；必要时先跑 `bt_8bitdo_min/deploy/install_ubuntu18.sh` 安装蓝牙、pyserial 和 input 权限规则，然后重新登录。
3. 启动脚本。
4. 脚本会先读舵机反馈。如果不在准备位，会要求输入 `HOME` 才慢速回位。
5. 用 D-pad 和右摇杆低速移动机械臂。
6. 每到一个点，确认 AprilTag 稳定识别后按 `B`。
7. 脚本会打印 raw、视觉 XYZ 和 `vision - FK` 偏置。
8. 按 `A` 退出。

推荐先采这些点，每个点可以采 2 到 3 次：

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

## 输出文件

默认每次运行新建目录：

```text
Delta_Gcode_Servo/real_machine_test/apriltag_workspace_samples/YYYYMMDD_HHMMSS/
```

主要文件：

| 文件 | 用途 |
| --- | --- |
| `samples.csv` | 拟合和快速查看用的表格 |
| `samples.jsonl` | 完整原始记录，每行一个采样点 |
| `session.json` | 本次运行参数、输入文件、输出路径 |
| `runtime_status.log` | 实时状态覆盖写入 |
| `vision_tool_preview_latest.json` | 最新视觉链路预览 |
| `logs/jetson_apriltag3k.log` | AprilTag 子进程日志 |

`samples.csv` 中关键列：

```text
raw1/raw2/raw3
fk_x_mm/fk_y_mm/fk_z_mm
x_mm/y_mm/z_mm
offset_x_mm/offset_y_mm/offset_z_mm
vision_detection_id
vision_snapshot_age_ms
```

其中 `x_mm/y_mm/z_mm` 是 AprilTag 视觉链路估计的工具点位置，`fk_*` 是当前旧模型根据舵机反馈算出的 FK 位置。

## 用数据集拟合模型

采样完成后运行：

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test
python3 workspace_model_tools.py fit \
  apriltag_workspace_samples/YYYYMMDD_HHMMSS/samples.jsonl \
  --output-dir apriltag_workspace_samples/YYYYMMDD_HHMMSS/model \
  --compute-workspace \
  --workspace-step-mm 5
```

如果 Jetson 装了 SciPy，会用 `scipy.optimize.least_squares`。如果没装 SciPy，会自动退回无 SciPy 坐标搜索。也可以强制：

```bash
python3 workspace_model_tools.py fit samples.jsonl --no-scipy
```

输出：

```text
model/fit_report.json
model/fit_residuals.csv
model/workspace_grid.csv
model/workspace_summary.json
```

重点看 `fit_report.json`：

- `evaluation.rms_residual_norm_mm`
- `evaluation.max_residual_norm_mm`
- `fitted_params.l1/l2/l3/servo_offset_x/servo_offset_z`
- `vision_offset_model_plus_offset_to_vision_mm`
- `controller_patch_hint.fields`

## 计算整个工作空间

如果已有 `fit_report.json`，单独计算工作空间：

```bash
python3 workspace_model_tools.py workspace \
  --model apriltag_workspace_samples/YYYYMMDD_HHMMSS/model/fit_report.json \
  --output-dir apriltag_workspace_samples/YYYYMMDD_HHMMSS/model \
  --step-mm 5
```

更细的网格更慢，例如：

```bash
python3 workspace_model_tools.py workspace --model model/fit_report.json --step-mm 2
```

`workspace_summary.json` 会给：

- 可达点数量和比例。
- X/Y/Z 极限。
- 每个 Z 层的最大 XY 半径。
- 建议写回控制器的保守边界 `suggested_controller_bounds`。

## 正逆解接口

现有正逆解已经在：

```text
Delta_Gcode_Servo/delta_gcode_servo/kinematics.py
```

接口：

```python
from delta_gcode_servo.kinematics import inverse_kinematics, forward_kinematics
from delta_gcode_servo.config import robot_params

params = robot_params()
angles_rad, ok = inverse_kinematics(x_mm, y_mm, z_mm, params)
xyz_mm, ok = forward_kinematics(theta1_rad, theta2_rad, theta3_rad, params)
```

`workspace_model_tools.py` 没有另写一套运动学，而是复用这两个接口。拟合出来的参数仍然对应 `RobotParams` 里的：

```text
l1
l2
l3
servo_offset_x
servo_offset_z
workspace_z_min
workspace_z_max
workspace_xy_max
```

## 写回控制器前的判断

不要只因为拟合能跑就立刻改 `config.py`。建议门槛：

- 中心区域 RMS 小于 5 到 10 mm。
- 边界区域最大误差小于 15 到 20 mm。
- `fit_residuals.csv` 没有某一个方向系统性偏大。
- `vision_snapshot_age_ms` 多数小于 `1000 ms`。
- 同一点重复采样的 `x/y/z` 抖动不大。

满足后，再把 `fit_report.json` 里的：

```json
"controller_patch_hint": {
  "fields": {
    "l1": ...,
    "l2": ...,
    "l3": ...,
    "servo_offset_x": ...,
    "servo_offset_z": ...
  }
}
```

手动写回：

```text
Delta_Gcode_Servo/delta_gcode_servo/config.py
```

工作空间边界建议从 `workspace_summary.json` 的 `suggested_controller_bounds` 取保守值。

## 自测

不连接硬件时可以检查离线工具：

```bash
python3 workspace_model_tools.py self-test --no-scipy
python3 -m py_compile apriltag_gamepad_workspace_sampler.py workspace_model_tools.py
```

预期 `self-test` 会打印 `self-test ok`。

## 常见问题

AprilTag 快照不新鲜：

```text
Sample refused: AprilTag snapshot is stale
```

处理：确认 3K GUI 正在跑、tag 在画面里、`--base-camera-snapshot` 路径正确。

8BitDo 找不到：

```text
8BitDo Bluetooth gamepad init failed
```

处理：先跑 `bt_8bitdo_min/deploy/install_ubuntu18.sh`，重新登录，让用户进入 `input` 组；不要固定写死 `event8`。

启动时要求 `HOME`：

这是正常安全逻辑。说明舵机反馈不在配置的参考 raw 附近。确认机械结构没有干涉后，输入大写 `HOME` 才会慢速回位。

拟合误差很大：

先不要改控制器参数。检查 AprilTag 坐标轴、`Dual_Camera_HandEye/output/calibration_result.json`、`tool_T_hand_tag`、tag 尺寸、相机是否用了同一个 1280x960 全 FOV 标定。
