# Jetson 上的 NVIDIA nvAprilTags GPU 识别管线

这个目录是 `192.168.1.80` Jetson Xavier NX 上当前使用的 3K 鱼眼 AprilTag GPU 识别主线。

当前目标不是做单张图片识别率，而是给机械臂小范围采样和闭环控制提供实时、低延迟、当前帧的 AprilTag 位姿。默认路径必须保持 NVIDIA `nvAprilTags` GPU 检测，不允许因为暗光识别问题退回 CPU 检测。

## 当前硬件

- 主机：Jetson Xavier NX Developer Kit
- 系统：JetPack/L4T R32.4.4，CUDA 10.2
- 相机：CSI IMX219 fisheye
- 检测器：NVIDIA `nvAprilTags`
- 标签族：`tag36h11`
- 当前生产画幅：3K 全视野采集 `3264x2464@21`，降采样到 `1280x960`

## 编译

Jetson 工作目录里已经带有 `nvAprilTags.h` 和 `libapril_tagging.a`。

```bash
cd /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench
./build_jetson.sh
```

## 启动 GUI

运动采样时用这个脚本：

```bash
cd /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench
./run_motion_1280x960_gui.sh
```

脚本会做这些事：

- 停掉旧的 `jetson-vision.service` 相机占用者
- 执行 `jetson_clocks`
- 重启 `nvargus-daemon`
- 清理旧的 `nv_gpu_apriltag_bench` 实例，避免多个进程抢相机或同时写 JSON
- 打开本地 Jetson GUI 窗口
- 采集 `3264x2464@21` 全视野
- 降采样到 `1280x960`
- 写出 `/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json`

GUI 显示的是原始彩色 BGR 图像；送入 `nvAprilTagsDetect` 的是另一份预处理后的检测图。这样可以同时保留视频观感和检测质量。

## 当前默认运动模式

默认预处理模式是：

```text
motion
```

处理链路是：

```text
BGR 相机帧 -> 灰度 -> unsharp 锐化 -> gamma 0.70 -> BGRA 上传 -> NVIDIA GPU nvAprilTagsDetect
```

这个模式的目的不是让静止暗光画面最容易识别，而是在机械臂移动时尽量保留 AprilTag 边缘，减少运动模糊导致的漏识别。

`run_motion_1280x960_gui.sh` 默认参数：

```bash
PREPROCESS=motion
GUI_HOLD_MS=0
OUTPUT_HOLD_MS=0
TNR_MODE=0
TNR_STRENGTH=0
EXPOSURE_COMPENSATION=0
EXPOSURETIMERANGE="34000 8000000"
GAINRANGE="1 12"
ISPDIGITALGAINRANGE="1 4"
```

含义：

- `GUI_HOLD_MS=0`：GUI 不显示上一帧旧框
- `OUTPUT_HOLD_MS=0`：JSON 不复用上一帧旧检测
- `TNR_MODE=0`：关闭 Jetson ISP 时域降噪，避免运动拖影
- `EXPOSURETIMERANGE="34000 8000000"`：限制曝光上限到 8 ms，优先减少运动模糊
- `GAINRANGE` 和 `ISPDIGITALGAINRANGE`：允许用增益补偿短曝光带来的变暗

注意：当前 Jetson Xavier NX 上，`nvarguscamerasrc` 日志虽然显示传感器最小曝光是 `13000 ns`，但实际会拒绝 `exposuretimerange="13000 8000000"`，报 `Invalid Exposure Time Range Input`。因此脚本使用 `34000` 作为下限。

## 直接运行命令

一般不需要手打这一长串，优先用 `run_motion_1280x960_gui.sh`。需要排查参数时可以直接运行：

```bash
./nv_gpu_apriltag_bench \
  --mode 0 \
  --sensor 3264x2464 \
  --sensor-fps 21 \
  --out 1280x960 \
  --seconds 0 \
  --warmup 8 \
  --gui \
  --preprocess motion \
  --calib-json /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json \
  --output-json /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json
```

需要看姿态轴时才加 `--draw-axes`。

## 可用预处理模式

```text
raw
equalize
clahe
gamma06
color_gamma06
color_gamma045
gain
y_equalize
y_clahe
gray_blur_gamma045
gray_blur_gamma05
gray_blur_gamma06
gray_blur_gamma07
gray_median_gamma06
gray_sharp_gamma06
motion
motion_clahe
```

当前建议：

- 机械臂移动采样：用 `motion`
- 静止暗光确认：可以试 `PREPROCESS=gray_blur_gamma07 ./run_fullfov_1280x960_gui.sh`
- 如果 `motion` 太暗或边缘仍差：可以试 `PREPROCESS=motion_clahe ./run_motion_1280x960_gui.sh`

## 已测结果

所有 full-FOV 结果都是 IMX219 mode 0，输入采集为 `3264x2464@21`，表里的分辨率是送入检测器的降采样画幅。

| 管线 | 时钟 | FPS | 检测平均耗时 | 结论 |
| --- | --- | ---: | ---: | --- |
| full FOV -> 960x724 | `jetson_clocks` | 约 21 | 约 16 ms | 稳定，能识别 |
| full FOV -> 1280x960 | `jetson_clocks` | 约 21 | 约 26 ms | 当前主线 |
| full FOV -> 1920x1448 | `jetson_clocks` | 约 13-19 | 约 36 ms | 延迟明显，识别弱 |
| 原生 720p crop | dynamic | 高 | 低 | 视野被裁掉，弃用 |
| 原生 3264x2464 | dynamic | 约 7 | 约 90 ms | 太慢 |

传感器 mode 0 的上限就是约 21 fps。`1280x960` 加 `jetson_clocks` 时，主要瓶颈不是 GPU 检测，而是相机帧率上限。

暗光静止测试记录：

| 预处理 | 帧数 | FPS | 有 tag 帧数 | 预处理平均耗时 | 检测平均耗时 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `raw` | 126 | 20.95 | 2 | 1.16 ms | 15.16 ms |
| `equalize` | 126 | 20.94 | 0 | 4.63 ms | 15.64 ms |
| `gray_blur_gamma06` | 126 | 20.91 | 24 | 2.86 ms | 22.54 ms |
| `gray_blur_gamma07` | 252 | 20.96 | 132 | 2.82 ms | 21.13 ms |
| `gray_blur_gamma07` + 重复 ID 过滤 + 短 JSON hold | 252 | 20.95 | 206 | 2.75 ms | 23.19 ms |

`gray_blur_gamma07` 对静止暗光画面有效，但它会先做模糊，不适合作为运动采样默认值。移动场景要优先保留边缘，所以默认改成 `motion`。

## 关于 hold 机制

当前版本不再使用 hold。命令行里的 `--gui-hold-ms` 和 `--output-hold-ms` 还保留，是为了兼容旧启动命令，但程序内部已经把它们当作 no-op。

现在的规则：

- GUI 只画当前帧真实识别到的框
- JSON 只写当前帧真实识别到的检测
- 没识别到时，`detections` 就是空数组
- 运动采样时不允许用旧位置冒充当前 tag 位置

旧版本如果使用 hold，会在 JSON 里出现：

```json
{
  "is_held": true,
  "held_ms": 73.0,
  "source_timestamp_unix": 1782240000.0
}
```

当前运动采样版本应该始终是：

```json
{
  "is_held": false
}
```

如果移动时 GUI 上没有框，应该解决图像采集问题，而不是恢复 hold。

## 移动时识别不到怎么办

如果 AprilTag 停下来能识别，移动时识别不到，优先按这个顺序处理：

1. 确认只运行了一个 GPU bench：

```bash
pgrep -af '^./nv_gpu_apriltag_bench( |$)'
```

正常应该只有一个进程。

2. 确认当前 JSON 仍然新鲜，并且是运动模式：

```bash
python3 - <<'PY'
import json
import os
import time

p = "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json"
data = json.load(open(p))
print("age_ms", int((time.time() - os.stat(p).st_mtime) * 1000))
print("fps", data.get("timing", {}).get("display_fps"))
print("preprocess", data.get("camera", {}).get("detector_preprocess"))
print("pixel_mode", data.get("camera", {}).get("pixel_mode"))
print("ids", [d.get("id") for d in data.get("detections", [])])
print("held", [(d.get("is_held"), d.get("held_ms")) for d in data.get("detections", [])])
PY
```

期望看到：

```text
preprocess motion
pixel_mode BGR_gray_unsharp_gamma0.70_to_BGRA_cuda
held []
```

3. 增加 tag 和底座区域的照明。

短曝光会让画面变暗。移动识别优先要少拖影，所以应先加物理光源，再考虑加长曝光。

4. 按顺序试曝光上限：

```bash
EXPOSURETIMERANGE="34000 6000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 8000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 10000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 12000000" ./run_motion_1280x960_gui.sh
```

曝光上限越短，拖影越少，但画面越暗。曝光上限越长，静止时越容易识别，但移动时更容易糊。

5. 必要时试 `motion_clahe`：

```bash
PREPROCESS=motion_clahe ./run_motion_1280x960_gui.sh
```

它会比 `motion` 多做 CLAHE，可能提升局部对比度，但也可能放大噪声。是否使用要以现场移动识别效果为准。

## 现场状态记录

最近一次在 `192.168.1.80` 上确认：

```text
进程数：1 个 ./nv_gpu_apriltag_bench
采集：3264x2464@21 -> 1280x960
预处理：motion
像素链路：BGR_gray_unsharp_gamma0.70_to_BGRA_cuda
曝光范围：34000 8000000
TNR：关闭
FPS：约 21
hold：无
```

如果 `ids=[]`，表示当前视野、光照、距离或 tag 姿态下没有识别到 AprilTag；这不是 hold 失效，而是当前帧真实未识别。

## 标定注意事项

当前 JSON 标定文件来自 OpenCV fisheye 标定，但 NVIDIA `nvAprilTags` 接口只接受 pinhole 的 `fx/fy/cx/cy`。本程序把标定矩阵传给 GPU 检测器，但没有先对图像做 fisheye undistort。

用于机械臂闭环前，需要用已知距离和位置检查输出的 XYZ。如果画面边缘误差过大，再考虑加入 undistort 或重新建立适合当前鱼眼画幅的工作空间映射。

## 不允许的改动

这些不要再作为默认路径：

- 不要把默认检测器换成 Python/OpenCV CPU AprilTag
- 不要为了静止识别率牺牲 21 fps GPU 实时性
- 不要恢复 GUI 或 JSON hold 来掩盖移动漏识别
- 不要用旧位置冒充机械臂采样时的当前 tag 位置

正确方向是：保持 3K 全视野降采样和 GPU `nvAprilTags`，在视频帧和 GPU 检测之间做轻量图像处理，并从曝光、TNR、照明、tag 位置上解决移动识别质量。
