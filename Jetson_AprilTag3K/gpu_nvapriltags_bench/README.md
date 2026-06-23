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

默认预处理模式重新回到前一个 no-hold GPU 版本已经验证过的路径：

```text
gray_blur_gamma07
```

处理链路是：

```text
BGR 相机帧 -> 灰度 -> 3x3 Gaussian blur -> gamma 0.70 -> BGRA 上传 -> NVIDIA GPU nvAprilTagsDetect
```

这个路径虽然有轻微模糊，但实测比强锐化的 `motion` 更适合当前 NVIDIA `nvAprilTags` 输入。AprilTag 的 quad/边界检测对降采样后的噪声和伪边缘敏感，过强锐化会让当前场景识别率变差。

`run_motion_1280x960_gui.sh` 默认参数：

```bash
PREPROCESS=gray_blur_gamma07
GUI_HOLD_MS=0
OUTPUT_HOLD_MS=0
```

含义：

- `GUI_HOLD_MS=0`：GUI 不显示上一帧旧框
- `OUTPUT_HOLD_MS=0`：JSON 不复用上一帧旧检测
- 不默认限制曝光、不默认关闭 TNR：前一次 A/B 测试证明短曝光/TNR off 会把当前识别率打到 0
- `PREPROCESS=gray_blur_gamma07`：保持前一个无 hold GPU 版本能识别的输入分布

注意：当前 Jetson Xavier NX 上，`nvarguscamerasrc` 日志虽然显示传感器最小曝光是 `13000 ns`，但实际会拒绝 `exposuretimerange="13000 8000000"`，报 `Invalid Exposure Time Range Input`。如果确实要做曝光实验，使用 `34000` 或更高作为下限。

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
  --preprocess gray_blur_gamma07 \
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

- 机械臂移动采样：默认用 `gray_blur_gamma07`
- 当前 1280x960 漏识别多时：用 `./run_robust_1600x1208_gui.sh`
- 静止暗光确认：可以试 `PREPROCESS=gray_blur_gamma07 ./run_fullfov_1280x960_gui.sh`
- `motion` / `motion_clahe` 只作为实验项，不再作为默认路径

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

2026-06-24 追加 A/B 结论：把默认从 `gray_blur_gamma07` 改成 `motion` 是错误方向。实际同场景测试显示，`motion` 强锐化不是提升，而是让检测归零；问题更像是当前 `nvAprilTags` 对降采样后噪声、伪边缘和标签像素尺寸敏感。当前默认已经恢复到 `gray_blur_gamma07`。

| 测试项 | 画幅 | 预处理/ISP | 帧数 | 有 tag 帧数 | 命中率 | FPS |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 前一个 no-hold 默认 | 1280x960 | `gray_blur_gamma07`，默认 ISP | 168 | 31 | 18.5% | 20.92 |
| 错误运动默认 | 1280x960 | `motion` + 8ms 短曝光 + TNR off | 168 | 0 | 0.0% | 20.92 |
| `motion` 默认 ISP | 1280x960 | `motion`，默认 ISP | 168 | 0 | 0.0% | 20.94 |
| `gray_blur_gamma07` + 短曝光 | 1280x960 | 8ms 短曝光 + TNR off | 168 | 0 | 0.0% | 20.93 |
| 降到 960x724 | 960x724 | `gray_blur_gamma07`，默认 ISP | 168 | 0 | 0.0% | 20.96 |
| 提高到 1600x1208 | 1600x1208 | `gray_blur_gamma07`，默认 ISP | 168 | 50 | 29.8% | 20.89 |

所以现在的优先级是：

1. 保持 GPU + no-hold。
2. 默认恢复到 `gray_blur_gamma07`。
3. 对识别率不足的场景，提高检测画幅到 `1600x1208`，而不是强行锐化或短曝光。
4. 需要更高帧率时再考虑 ROI/异步 latest-frame，而不是把 full-FOV 降到过低分辨率。

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

1280x960 默认路径期望看到：

```text
preprocess gray_blur_gamma07
pixel_mode BGR_gray_blur_gamma0.70_to_BGRA_cuda
held []
```

3. 如果 1280x960 命中率低，先提高检测画幅：

```bash
./run_robust_1600x1208_gui.sh
```

同场景 A/B 里，`1600x1208` 从 31/168 提升到 50/168，而且仍接近 21 fps。它比盲目锐化或降到 960x724 更可靠。

`run_robust_1600x1208_gui.sh` 为了避免 GUI 显示拖慢检测循环，默认加了：

```bash
GUI_SCALE=0.75
GUI_EVERY=2
```

这只降低 GUI 显示负载；GPU 检测和 JSON 输出仍然使用 `1600x1208`。

4. 如果确实要做曝光实验，再按顺序试曝光上限：

```bash
EXPOSURETIMERANGE="34000 6000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 8000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 10000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 12000000" ./run_motion_1280x960_gui.sh
```

曝光上限越短，拖影越少，但同场景测试里 8ms 短曝光让识别变成 0/168。不要把短曝光作为默认策略。

5. 必要时试 `motion_clahe`：

```bash
PREPROCESS=motion_clahe ./run_motion_1280x960_gui.sh
```

它会比 `motion` 多做 CLAHE，可能提升局部对比度，但也可能放大噪声。是否使用必须以现场 A/B 命中率为准。

## 现场状态记录

最近一次在 `192.168.1.80` 上确认：

```text
进程数：1 个 ./nv_gpu_apriltag_bench
采集：3264x2464@21 -> 1600x1208
预处理：gray_blur_gamma07
像素链路：BGR_gray_blur_gamma0.70_to_BGRA_cuda
曝光范围：默认 ISP
TNR：默认 ISP
GUI：0.75 缩放，每 2 帧刷新一次
JSON/display_fps：约 19.2
当前识别：id=3
hold：无
```

如果 `ids=[]`，表示当前视野、光照、距离或 tag 姿态下没有识别到 AprilTag；这不是 hold 失效，而是当前帧真实未识别。

## 标定注意事项

当前 JSON 标定文件来自 OpenCV fisheye 标定，但 NVIDIA `nvAprilTags` 接口只接受 pinhole 的 `fx/fy/cx/cy`。本程序把标定矩阵传给 GPU 检测器，但没有先对图像做 fisheye undistort。

用于机械臂闭环前，需要用已知距离和位置检查输出的 XYZ。如果画面边缘误差过大，再考虑加入 undistort 或重新建立适合当前鱼眼画幅的工作空间映射。

## 资料依据

- AprilRobotics 官方 README 说明：增大 `quad_decimate` 会加快检测，但代价是检测距离；如果图像有噪声，`quad_sigma` 这类 Gaussian blur 参数可能有帮助。来源：https://github.com/AprilRobotics/apriltag
- `pupil-apriltags` API 文档说明：低分辨率 quad 检测会带来检测率/位姿精度损失；非常 noisy 的图像可受益于非零 `quad_sigma`；`decode_sharpening` 可能帮助小标签，但在特殊光照或低光照下不一定有帮助。来源：https://pupil-apriltags.readthedocs.io/en/latest/api.html
- NVIDIA Isaac ROS AprilTag 文档给出的图像链路是 camera -> rectify -> resize -> AprilTag，输入分辨率要按检测距离和标签所需像素数选择。来源：https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag

## 不允许的改动

这些不要再作为默认路径：

- 不要把默认检测器换成 Python/OpenCV CPU AprilTag
- 不要为了静止识别率牺牲 21 fps GPU 实时性
- 不要恢复 GUI 或 JSON hold 来掩盖移动漏识别
- 不要用旧位置冒充机械臂采样时的当前 tag 位置

正确方向是：保持 3K 全视野降采样和 GPU `nvAprilTags`，在视频帧和 GPU 检测之间做轻量图像处理，并从曝光、TNR、照明、tag 位置上解决移动识别质量。
