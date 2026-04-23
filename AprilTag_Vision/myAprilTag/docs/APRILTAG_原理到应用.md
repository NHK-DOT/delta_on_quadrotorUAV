# AprilTag 从原理到应用

## 1. 项目目标

这个项目的目标很明确：

1. 生成一组可打印的 AprilTag 标签。
2. 用普通 USB 摄像头稳定识别和框选标签。
3. 输出标签在图像中的二维位置。
4. 利用标签真实尺寸估计 `Z` 轴远近。
5. 为后续部署到 `STM32MP257` 提前整理一条可落地的路线。

## 2. 为什么选 AprilTag

普通二维码更偏向信息存储，AprilTag 更偏向视觉定位。

你当前的需求更关心：

- 远近变化
- 倾斜角度
- 杂乱背景
- 实时识别
- 后续位置估计

这些都是 AprilTag 更擅长的方向。

## 3. 参考论文

本项目整理了三篇核心论文：

- `papers/pdf/olson2011tags.pdf`
  对应 `papers/markdown/olson2011tags.md`
- `papers/pdf/wang2016iros.pdf`
  对应 `papers/markdown/wang2016iros.md`
- `papers/pdf/krogius2019iros.pdf`
  对应 `papers/markdown/krogius2019iros.md`

这三篇论文分别覆盖：

- AprilTag 的基本设计思想
- AprilTag 2 的速度和鲁棒性改进
- 更灵活的标签布局思路

## 4. 当前工程选择

当前实现选择：

- 标签家族：`tag36h11`
- 检测后端：`OpenCV cv2.aruco`
- 输入设备：普通 USB Camera
- 输出内容：标签 ID、中心点、框选、`X/Y/Z`、JSON 快照

选择 `tag36h11` 的原因很直接：

- OpenCV 内建支持
- Python 部署最省事
- 足够满足比赛视觉识别和测距

## 5. 标签生成

标签由 `src/generate_apriltags.py` 生成，输出到 `image/`。

默认会生成：

- `ID 00` 到 `ID 11`
- 单张高分辨率 PNG
- A4 拼版打印图

命令：

```powershell
python src\generate_apriltags.py
```

如果你要更多 ID：

```powershell
python src\generate_apriltags.py --ids 0-23 --marker-size-mm 80
```

## 6. 标定怎么理解

`src/calibrate_camera.py` 的作用是给摄像头求内参和畸变参数。

它输出：

- `camera_matrix`
- `dist_coeffs`
- `rms_reprojection_error`

文件位置：

- `calibration/camera_intrinsics.json`

推荐标定命令：

```powershell
python src\calibrate_camera.py --camera-index 1 --backend dshow --cols 9 --rows 6 --square-size-m 0.025
```

注意：

- `cols` 和 `rows` 是内角点数，不是格子数。
- `square-size-m` 是单个方格真实边长，单位米。
- 标定分辨率最好和后面识别时一致。

## 7. 检测脚本现在做什么

`src/apriltag_usb_detector.py` 负责：

- 打开 USB 摄像头
- 检测 AprilTag
- 框选标签
- 估计 `X/Y/Z`
- 生成 `output/apriltag_latest.json`

如果检测脚本发现标定文件存在，就会进入更准确的 `calibrated_pose` 模式；否则走近似测距模式。

## 8. 二维位置和 Z 轴远近

识别出标签后，程序会拿到四个角点。四点平均就是标签中心，也就是图像中的二维位置。

`Z` 轴远近来自针孔模型：

```text
Z = f * S / s
```

其中：

- `f`：焦距，像素单位
- `S`：标签真实边长，米
- `s`：标签图像边长，像素

如果已有标定参数，程序会直接用 OpenCV 的位姿估计接口输出更稳定的 `X/Y/Z`。

## 9. 性能模式

检测脚本现在支持以下模式：

- `accuracy`
- `balanced`
- `speed`
- `mp257`

同时支持这些关键参数：

- `--detect-scale`
- `--refine`
- `--snapshot-hz`
- `--use-roi-tracking`
- `--roi-padding`
- `--crop-width`
- `--crop-height`
- `--draw-axes / --no-draw-axes`

这些参数的意义是：

- 先缩小检测图，减少 CPU 压力
- 降低角点精修强度
- 避免每帧都写盘
- 利用前一帧 ROI 限定搜索区域
- 在需要时只对居中的检测区做搜索

## 10. 推荐运行方式

### PC 调试

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile balanced --width 1280 --height 720 --fps 15 --tag-size-m 0.08
```

### 速度优先

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile speed --width 960 --height 540 --fps 15 --tag-size-m 0.08
```

### 提前模拟 STM32MP257 风格

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile mp257 --width 640 --height 480 --fps 30 --tag-size-m 0.08 --no-draw-axes
```

### 1080p 采集 + 居中裁剪检测

如果你不想把整个 `1080p` 一起缩小，而是希望“保留中心区域细节”，现在可以这样跑：

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --width 1920 --height 1080 --profile mp257 --crop-width 960 --crop-height 540 --detect-scale 1.0 --tag-size-m 0.08
```

这条命令的意思是：

- 相机仍然按 `1920x1080` 采集
- 检测时只在中心 `960x540` 区域搜索
- 不额外缩小这块裁剪区

如果你想继续压负载，可以把 `--detect-scale` 设成 `0.75` 或 `0.5`。

## 11. 为什么 1080p 会比 540p 慢很多

核心原因很简单：

- `1920x1080` 大约是 `2.07 MP`
- `960x540` 大约是 `0.52 MP`

前者像素数约为后者的 `4 倍`。

AprilTag 检测不是简单的“放大 4 倍就慢 4 倍”，因为全图检测里还包含：

- 候选四边形搜索
- 解码
- 角点精修
- 位姿估计
- 绘制和写盘

所以实际掉速可能比 4 倍还明显，这也是为什么你会看到：
- 540p 能跑得很顺
- 1080p 全图检测会明显掉帧

这里最实用的优化不是盲目缩糊整张图，而是：
- 高分辨率采集
- 居中裁剪检测区
- 必要时再对检测区降采样

## 12. 当前部署思路

对你最终要部署到 `STM32MP257` 这件事，现在最合理的路线不是一上来强上 NPU，而是：

1. 先把 CPU 跑 AprilTag 这条链路压到轻量版本。
2. 让输入分辨率控制在未来板子能稳定承受的水平。
3. 未来在板子上，把裁剪和缩放尽量前移到 `V4L2 / DCMIPP / GStreamer`。
4. 如果后面需要再引入小型神经网络做候选区域筛选，再考虑 NPU。

## 13. 你现在该怎么用

建议顺序：

1. 打印 `image/` 里的标签。
2. 运行 `src/calibrate_camera.py` 完成标定。
3. 用 `src/apriltag_usb_detector.py` 先在 PC 上跑通。
4. 用 `--profile mp257` 提前模拟未来边缘部署策略。
5. 最后再迁移到 `STM32MP257`。
