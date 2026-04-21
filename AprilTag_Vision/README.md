# AprilTag Vision

这是一个面向普通 USB 摄像头的 AprilTag 识别项目，覆盖标签生成、摄像头标定、实时识别，以及后续部署到 `STM32MP257` 的思路。

## 项目内容

- `src/generate_apriltags.py`
  生成可打印的 AprilTag 标签图和 A4 拼版图。
- `src/calibrate_camera.py`
  用棋盘格标定摄像头，得到 `camera_matrix` 和 `dist_coeffs`。
- `src/apriltag_usb_detector.py`
  用 USB 摄像头实时识别、框选、估计标签相对相机的位置，并输出 JSON 快照。
- `docs/APRILTAG_原理到应用.md`
  从原理、论文、工程落地到使用步骤的中文说明。
- `docs/PERFORMANCE_AND_STM32MP257.md`
  说明当前性能优化、GPU/NPU 能力边界，以及未来部署到 `STM32MP257` 的路线。
- `papers/pdf/`
  AprilTag 核心论文 PDF。
- `papers/markdown/`
  用 MarkItDown 转出来的论文 Markdown。
- `image/`
  可直接打印的标签图。

## 快速开始

```powershell
cd C:\Users\hanjuncheng\Desktop\nodejs\AprilTag_Vision
python src\generate_apriltags.py
```

打印标签后，量出黑色方框边长。如果是 `80 mm`，直接运行：

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --tag-size-m 0.08
```

如果你已经完成标定，检测脚本会自动读取 `calibration/camera_intrinsics.json`，进入更准确的 `calibrated_pose` 模式。

## 标定

推荐流程：

```powershell
python src\calibrate_camera.py --camera-index 1 --backend dshow --cols 9 --rows 6 --square-size-m 0.025
```

说明：

- `cols` / `rows` 是棋盘格的内角点数，不是黑白格子数。
- `square-size-m` 是单个小方格真实边长，单位米。
- 标定完成后会生成 `calibration/camera_intrinsics.json`。

## 性能模式

`src/apriltag_usb_detector.py` 支持几种预设 profile：

- `accuracy`：精度优先。
- `balanced`：默认模式，兼顾速度和稳定性。
- `speed`：速度优先，关闭较重的精修和绘制。
- `mp257`：按未来 `STM32MP257` 部署思路预设，优先减轻 CPU 负担。

PC 上推荐：

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile balanced --width 1280 --height 720 --fps 15 --tag-size-m 0.08
```

如果你想先按未来 `STM32MP257` 的资源约束来跑：

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile mp257 --width 640 --height 480 --fps 30 --tag-size-m 0.08 --no-draw-axes
```

## 1080p 优化与居中裁剪

现在检测脚本除了直接降分辨率，还支持“完整分辨率采集 + 居中裁剪检测区”。

新增参数：

- `--crop-width`
  指定居中裁剪检测区宽度
- `--crop-height`
  指定居中裁剪检测区高度

这两个参数的意义是：

- 相机仍然按你指定的高分辨率采集
- 检测时只在画面中心区域搜索标签
- 保留中心区域的原始像素密度，而不是把整张图一起缩小

例如，如果你想保留 `1080p` 采集，但只在中心 `960x540` 范围里检测：

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --width 1920 --height 1080 --profile mp257 --crop-width 960 --crop-height 540 --detect-scale 1.0 --tag-size-m 0.08
```

如果你还想再进一步压负载：

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --width 1920 --height 1080 --profile mp257 --crop-width 960 --crop-height 540 --detect-scale 0.75 --tag-size-m 0.08
```

推荐理解：

- `960x540` 全图降分辨率：看得更广，但每个目标像素更少
- `1920x1080` + `960x540` 居中裁剪：视野更窄，但中心区域保留更高细节

## 当前已生成标签

`image/` 目录下已经有：

- `tag36h11_id_00_80mm.png` 到 `tag36h11_id_11_80mm.png`
- `tag36h11_sheet_page_01_80mm_A4.png` 到 `tag36h11_sheet_page_06_80mm_A4.png`

## 文档入口

建议阅读顺序：

1. `docs/APRILTAG_原理到应用.md`
2. `docs/PERFORMANCE_AND_STM32MP257.md`
3. `papers/README.md`

## 论文与来源

- Olson 2011: https://april.eecs.umich.edu/media/pdfs/olson2011tags.pdf
- Wang 2016: https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf
- Krogius 2019: https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf
- AprilTag 官方项目: https://github.com/AprilRobotics/apriltag
