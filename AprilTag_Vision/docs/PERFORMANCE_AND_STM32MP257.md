# 性能优化与 STM32MP257 部署建议

## 1. 结论先说清楚

这套逻辑最终部署到 `STM32MP257` 时，正确方向不是单纯盯着 `GPU` 或 `NPU`，而是先把任务拆开：

- `AprilTag 检测 / 解码 / 位姿估计`
  这部分本质上是经典计算机视觉，当前主路径更适合跑在 `CPU`。
- `相机采集、裁剪、缩放、格式转换`
  尽量前移到 `V4L2 / DCMIPP / GStreamer`。
- `神经网络前处理或候选区域筛选`
  这部分才适合交给 `NPU`。
- `图形显示`
  这是 `GPU` 更自然的工作。

所以真正合理的目标是：

1. 让 CPU 只处理更小、更干净的 ROI 或低分辨率帧。
2. 不让 CPU 同时扛全图采集、全图检测、全图显示和频繁写盘。
3. 如果后面引入小型神经网络，只让 NPU 做候选区域筛选，不直接解码 AprilTag。

## 2. 官方能力边界

根据 ST 官方资料：

- `STM32MP25` 系列带 `GPU / VPU / NPU`，NPU 峰值可到 `1.35 TOPS`。
- `STM32MP23x/25x` GPU 额外支持 `OpenGL ES 3.2.8`、`Vulkan 1.3`、`OpenVG 1.3`、`OpenCL 3.0`、`OpenVX 1.3`。
- `STM32MP2x NPU` 原生执行的网络格式是 `NBG`。
- `TensorFlow Lite` 和 `ONNX Runtime` 可以通过 ST 的 AI 工具链走 `STM32MP2` 的 NPU。

来源：

- STM32MP25 microprocessor  
  https://wiki.st.com/stm32mpu/wiki/STM32MP25_microprocessor
- GPU internal peripheral  
  https://wiki.st.com/stm32mpu/wiki/GPU_internal_peripheral
- ST Edge AI: Guide for STM32MPU  
  https://wiki.st.com/stm32mpu/wiki/ST_Edge_AI%3A_Guide_for_STM32MPU
- X-LINUX-AI tool suite  
  https://wiki.st.com/stm32mpu/wiki/X-LINUX-AI_tool_suite
- STM32MP25 V4L2 camera overview  
  https://wiki.st.com/stm32mpu/wiki/STM32MP25_V4L2_camera_overview

这几条官方信息对应出的工程结论是：

- `OpenCV aruco.detectMarkers()` 不会自动用上 `MP257` 的 GPU。
- `AprilTag` 经典视觉算法不能直接原样丢到 NPU 上跑。
- `NPU` 更适合跑轻量候选区域网络。

## 3. 当前代码已经做了什么优化

现在的 `src/apriltag_usb_detector.py` 已经加入了几项对 PC 和 `STM32MP257` 都有意义的优化：

- `--profile`
  预设性能模式，支持 `accuracy / balanced / speed / mp257`
- `--detect-scale`
  检测前先降采样
- `--refine`
  控制角点精修强度
- `--snapshot-hz`
  限制 JSON 写盘频率
- `--use-roi-tracking`
  优先在上一帧 ROI 内检测
- `--roi-padding`
  控制 ROI 扩展范围
- `--draw-axes / --no-draw-axes`
  控制是否绘制坐标轴

这些优化的价值是：

- 不依赖某一台 PC
- 迁移到 `STM32MP257` 后仍然成立
- 本质上在减轻 CPU 和内存带宽压力

另外，脚本现在还支持：

- `--crop-width`
- `--crop-height`

它们用于“完整分辨率采集 + 居中裁剪检测区”。

## 4. 推荐运行方式

### PC 调试

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile balanced --width 1280 --height 720 --fps 15 --tag-size-m 0.08
```

### 追求速度

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile speed --width 960 --height 540 --fps 15 --tag-size-m 0.08
```

### 1080p 输入但只检测中心区域

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --width 1920 --height 1080 --profile mp257 --crop-width 960 --crop-height 540 --detect-scale 1.0 --tag-size-m 0.08
```

如果还想更轻一点：

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --width 1920 --height 1080 --profile mp257 --crop-width 960 --crop-height 540 --detect-scale 0.75 --tag-size-m 0.08
```

### 提前模拟 STM32MP257

```powershell
python src\apriltag_usb_detector.py --camera-index 1 --backend dshow --profile mp257 --width 640 --height 480 --fps 30 --tag-size-m 0.08 --no-draw-axes
```

## 5. 为什么 1080p 会掉得很厉害

以你现在常用的两档为例：

- `1920x1080` 大约 `2.07 MP`
- `960x540` 大约 `0.52 MP`

1080p 的像素量大约是 540p 的 `4 倍`。

而 AprilTag 的全图检测不是单纯线性缩放，它还叠加了：

- 候选四边形搜索
- 解码
- 角点精修
- 位姿估计
- 绘制
- JSON 写盘

所以你看到“540p 很顺、1080p 很慢”是正常现象。

这也是为什么现在建议优先做：

- 降采样检测
- ROI 限定
- 居中裁剪
- 采集链路前移裁剪缩放

## 6. 为什么现在不把“GPU 优化”当第一优先级

你当前的瓶颈不是单纯“算得慢”，而是：

- 全图检测
- Python 主循环串行
- 每帧位姿估计
- 每帧绘制和写盘

即使你做了部分 GPU 预处理，如果链路仍然是：

- 摄像头给高分辨率全图
- Python 收全图
- CPU 做全图搜索

收益不会像想象中那么大。

所以优先级应该是：

1. 降输入分辨率
2. 利用 ROI
3. 把裁剪缩放前移到采集链路
4. 再谈 GPU/NPU 进一步卸载

## 7. STM32MP257 上更合适的相机链路

在 `STM32MP257` 上，更合理的做法不是让 Python 自己拿 `1920x1080` 再缩，而是：

- 在 `DCMIPP / V4L2` 层先裁剪
- 在采集链路上先缩放到更小分辨率
- 应用层尽量只接 `640x480` 或 `960x540` 级别的帧

这类优化比“单纯多开线程”更有价值。

## 8. 对 MP257 最实用的三阶段路线

### 阶段 A

- 摄像头
- V4L2 / DCMIPP 预裁剪缩放
- CPU AprilTag

特点：最容易落地，工程复杂度最低。

### 阶段 B

- 摄像头
- V4L2 / DCMIPP 输出较小帧
- CPU AprilTag 检测
- 只对检测到的标签做位姿估计和绘制

特点：最符合你当前项目的现实需求。

### 阶段 C

- 摄像头
- NPU 跑一个轻量候选区域网络
- 输出“可能有标签的区域”
- CPU 只在 ROI 内解码 AprilTag

这才是 NPU 真正有意义的位置。

## 9. 如果以后一定要用 NPU

建议不是把 AprilTag 本体网络化，而是训练一个小模型做：

- `tag board / marker board / square fiducial` 候选框检测
- 或者简单的候选热区定位

要求：

- 模型要小
- 最好量化
- 使用 `TensorFlow Lite` 或 `ONNX`
- 最后转换到 ST 的 `NBG` 路线

这样才能对上 ST 官方的 NPU 工具链。

## 10. 一句话结论

对 `STM32MP257` 来说：

- `AprilTag` 主解码仍应按 `CPU 经典视觉` 来设计。
- `GPU` 更适合显示和部分预处理。
- `NPU` 更适合先做“候选区域筛选”。

真正关键不是“强行让 AprilTag 跑 NPU”，而是：

> 用对硬件分工，让 CPU 不再处理全分辨率全图搜索。
