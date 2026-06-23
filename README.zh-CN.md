# 78arm 中文说明

English README: [README.md](README.md)

开源协议：GNU GPL v3.0，见 [LICENSE](LICENSE)。上游 MIT 项目的版权声明保留在
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 当前主线

当前现场主线以 `192.168.1.80` 的 Jetson Xavier NX 为准：

```text
Delta_Gcode_Servo/real_machine_test/jetson_py36/run_sampler_py36_jetson.sh
```

这条链路把 3K 鱼眼降采样 AprilTag、8BitDo 蓝牙手柄、舵机反馈和工作空间采样放在同一个 Jetson 现场流程里：

```text
Jetson Xavier NX 192.168.1.80
  -> 3K 鱼眼全 FOV 降采样 AprilTag JSON
  -> jetson_py36 只读自检
  -> 检查 8BitDo 是否连接
  -> 检查舵机 1/2/3 是否能回读
  -> 检查机械臂是否接近 home_raw 初始位
  -> 输入 YES 后允许低速手动移动
  -> 按 B 采样 AprilTag 工具点 XYZ + 舵机 raw
  -> workspace_model_tools.py 拟合模型并扫描工作空间
```

旧的 `bt_8bitdo_min` 独立控制包已经收敛掉。现在它只是兼容依赖库，只保留：

- `src/evdev_gamepad.py`：8BitDo Linux evdev 读取器。
- `src/servo_driver.py`：幻尔 xArm 1.6 舵机驱动板串口协议。
- `config/gamepad_8bitdo_bt.json`：当前 8BitDo 按键/轴映射。
- `deploy/install_ubuntu18.sh`：Jetson Xavier NX 上的蓝牙、pyserial、input 权限安装脚本。

## 部署到 Jetson Xavier NX

从本机部署到八零：

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm
python Delta_Gcode_Servo\real_machine_test\jetson_py36\deploy_to_jetson.py --password nvidia
```

部署目标：

```text
/home/nvidia/Desktop/78arm
```

部署脚本只复制当前运行需要的最小包，不复制 `Jetson_Vision_Export/saved_runs`、历史 samples、日志和缓存。

## Jetson 现场命令

登录 Jetson：

```bash
ssh nvidia@192.168.1.80
```

如果 8BitDo 权限还没配置：

```bash
cd ~/Desktop/78arm/bt_8bitdo_min
bash deploy/install_ubuntu18.sh
```

第一次安装后退出登录再重新登录，使 `input` 组权限生效。

只读自检，不移动舵机：

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
python3 jetson_workspace_preflight.py --port /dev/ttyUSB0 --hand-tag-id 3
```

进入采样：

```bash
HAND_TAG_ID=3 bash run_sampler_py36_jetson.sh
```

如果舵机板不是 `/dev/ttyUSB0`：

```bash
PORT=/dev/ttyUSB1 HAND_TAG_ID=3 bash run_sampler_py36_jetson.sh
```

## 自检内容

`jetson_workspace_preflight.py` 会检查：

- Python 3.6 和 `pyserial`。
- 3K AprilTag JSON 是否存在、是否新鲜、是否包含指定 tag id。
- `Dual_Camera_HandEye/output/calibration_result.json` 是否能换算出工具点 XYZ。
- 8BitDo 是否能在 `/dev/input/event*` 下打开。
- 舵机板是否能读回 servo 1/2/3 raw 和电压。
- 当前 raw 是否接近 `lx225_tool_demo/config/lx225_tool.demo.toml` 的 `home_raw`。

默认 home 容差是 `25 ticks`。如果不在初始位，采样脚本默认拒绝启动。

## 手柄控制

进入采样控制后，脚本会再次要求输入 `YES`，确认后才允许发送低速舵机命令。

| 控制 | 动作 |
| --- | --- |
| D-pad 左/右 | X 小范围移动 |
| D-pad 上/下 | Y 小范围移动 |
| 右摇杆 Y | Z 小范围移动 |
| B | 采样当前 AprilTag XYZ + 舵机 raw |
| X | 切换 safe scan: FREE/X/Y/Z |
| A | 退出 |

默认速度：

```text
XY: 35 mm/s
Z: 25 mm/s
servo raw limit: 180 raw/s
```

## 采样输出和模型计算

每次运行输出到：

```text
Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/YYYYMMDD_HHMMSS/
```

关键文件：

- `preflight_report.json`
- `samples.csv`
- `samples.jsonl`
- `session.json`
- `runtime_status.log`
- `debug.log`
- `logs/jetson_apriltag3k.log`

采样后拟合模型并扫描工作空间：

```bash
cd ~/Desktop/78arm
python3 Delta_Gcode_Servo/real_machine_test/workspace_model_tools.py fit \
  Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/YYYYMMDD_HHMMSS/samples.jsonl \
  --output-dir Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/YYYYMMDD_HHMMSS/model \
  --compute-workspace
```

输出：

```text
model/fit_report.json
model/fit_residuals.csv
model/workspace_grid.csv
model/workspace_summary.json
```

正逆解接口在：

```text
Delta_Gcode_Servo/delta_gcode_servo/kinematics.py
```

模型工具复用这套 FK/IK，并用采样数据拟合结构参数和视觉常量偏置。

## 目录说明

- `Delta_Gcode_Servo/`：Jetson 采样入口、模型拟合、G-code、Delta IK/FK、总线舵机控制工具。
- `bt_8bitdo_min/`：当前 Jetson Xavier NX 采样流程的最小兼容依赖。
- `Jetson_AprilTag3K/`：3K 鱼眼全 FOV 降采样 GPU AprilTag 流程。
- `Dual_Camera_HandEye/`：底座相机、AprilTag、工具坐标链路。
- `lx225_tool_demo/`：LX-225 舵机配置，包含当前 `home_raw`。
- `IMU/`：WT61C IMU 工具和快照。
- `AprilTag_Vision/`：旧本机 AprilTag 检测工具。
- `Jetson_Vision_Export/`：历史 Jetson 视觉部署归档。
- `part_model_rev/`：当前机械结构文件。

## 安全注意

- `jetson_workspace_preflight.py` 是只读路径，不会移动舵机。
- `run_sampler_py36_jetson.sh` 会先跑自检，进入控制前还需要输入 `YES`。
- 运行前必须确认机械臂在安全参考姿态，舵机 1/2/3 在线，周围无障碍物。
- 如果出现 AprilTag 丢失、手柄断连、串口异常、舵机反馈异常或 IK 失败，应停止运行并重新检查硬件。

## Git 同步注意

不要提交运行快照和生成物，例如：

- `IMU/wt61c_latest.json`
- `Jetson_Vision_Export/saved_runs/`
- `jetson_py36/samples/`
- 日志、缓存、`.pyc`

提交前先看：

```bash
git status --short --ignored
git add -n .
```
