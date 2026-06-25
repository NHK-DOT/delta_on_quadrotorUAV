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
  -> 检查机械臂是否接近 startup_check_raw 启动自检位
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

运行自检或采样前，先确认当前两个串口分别是谁。下面这套流程只做枚举和被动接收，
不会向舵机驱动板写入任何字节。

```bash
ls -l /dev/serial/by-id /dev/serial/by-path 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

for p in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$p" ] || continue
  echo "[$p]"
  udevadm info --query=property --name="$p" 2>/dev/null \
    | grep -E '^(DEVNAME|ID_BUS|ID_VENDOR|ID_VENDOR_ID|ID_MODEL|ID_MODEL_ID|ID_SERIAL|ID_PATH|ID_USB_DRIVER)=' || true
  fuser -v "$p" 2>/dev/null || true
done
```

然后用 `9600` 被动监听区分 WT61C IMU。WT61C 会连续吐出 `0x55 0x51`、
`0x55 0x52`、`0x55 0x53` 或 `0x55 0x54` 帧，并且校验和应当正确。舵机
驱动板通常不会主动吐数据，只会在主机查询后响应，所以被动监听时没有字节是正常的。

```bash
python3 - <<'PY'
import binascii
import os
import select
import time

import serial

ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
baudrate = 9600
duration = 5.0


def wt61_frames(buf):
    frames = []
    i = 0
    while i + 11 <= len(buf):
        if buf[i] == 0x55 and buf[i + 1] in (0x51, 0x52, 0x53, 0x54, 0x59):
            frame = buf[i : i + 11]
            frames.append(((sum(frame[:10]) & 0xFF) == frame[10], bytes(frame)))
            i += 11
        else:
            i += 1
    return frames


serials = []
for port in ports:
    if not os.path.exists(port):
        continue
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=0,
        write_timeout=0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    ser.dtr = False
    ser.rts = False
    serials.append(ser)
    print("%s opened for passive read @ %d" % (port, baudrate))

buffers = {ser.port: bytearray() for ser in serials}
counts = {ser.port: 0 for ser in serials}
end = time.time() + duration

while serials and time.time() < end:
    readable, _, _ = select.select(serials, [], [], 0.2)
    for ser in readable:
        data = ser.read(4096)
        counts[ser.port] += len(data)
        if len(buffers[ser.port]) < 256:
            buffers[ser.port].extend(data[: 256 - len(buffers[ser.port])])

for ser in serials:
    ser.close()

for port in ports:
    buf = buffers.get(port, bytearray())
    frames = wt61_frames(buf)
    ok = sum(1 for valid, _ in frames if valid)
    print("%s bytes=%d wt61_frames=%d checksum_ok=%d" % (
        port,
        counts.get(port, 0),
        len(frames),
        ok,
    ))
    if buf:
        print("  sample_hex=%s" % binascii.hexlify(bytes(buf)).decode("ascii"))
PY
```

当前 `192.168.1.80` 这台 Jetson 的实测映射是：

```text
IMU:    /dev/ttyUSB1 @ 9600
舵机板: /dev/ttyUSB0 @ 9600
```

如果担心插拔后 `/dev/ttyUSB*` 编号变化，优先使用稳定的 `by-path`：

```text
IMU:    /dev/serial/by-path/platform-3610000.xhci-usb-0:2.4.1:1.0-port0
舵机板: /dev/serial/by-path/platform-3610000.xhci-usb-0:2.4.2:1.0-port0
```

## 工作空间标定流程

这是 `192.168.1.80` 的当前主线流程。先只读自检，再低速手动采样，最后用采样
数据拟合模型并扫描工作空间。更完整的脚本说明在：

```text
Delta_Gcode_Servo/real_machine_test/jetson_py36/README.md
```

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
- 当前 raw 是否接近 `lx225_tool_demo/config/lx225_tool.demo.toml` 的 `startup_check_raw`。

默认启动位容差是 `30 ticks`。如果 raw 明显超出配置范围，预检和采样脚本会拒绝启动；如果只是不在启动自检位附近，采样脚本会要求输入大写 `HOME`，才会慢速回到配置里的 `home_raw`。

舵机驱动板 `0x15` 反馈按有符号 int16 解释。例如 `0xFF43` 应看作 `-189`，不是无符号的 `65347`。配置里有两个相关字段：

- `startup_check_raw`：只读自检使用的启动位。
- `home_raw`：允许进入采样控制后，FK/IK 运动映射使用的 home 参考位。

当前拆掉机械结构后的舵机本体 home 反馈已经写入配置：servo 1/2/3 分别是 `750`、`762`、`758`。三个主舵机当前都使用 `0..1000` raw 映射范围，`home_raw` 和 `startup_check_raw` 都同步到这些实测 home 值。

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
- `lx225_tool_demo/`：LX-225 舵机配置，包含当前 `home_raw` 和 `startup_check_raw`。
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
