# 性能优化与 STM32MP257 部署建议

## 1. 结论先说清楚

这套链路最终部署到 `STM32MP257` 时，正确目标不是“把 AprilTag 主检测强行改成 GPU / NPU 跑”，而是先把硬件分工做对。

合理分工是：

- `AprilTag 检测 / 解码 / 位姿估计`
  仍按经典视觉主路径设计，核心负载在 CPU
- `采集、裁剪、缩放、格式转换`
  尽量前移到 `V4L2 / DCMIPP / GStreamer`
- `图形显示`
  更适合 GPU
- `候选区域筛选`
  如果以后要引入轻量网络，这部分才是 NPU 真正有意义的位置

所以工程上的真实目标应当是：

1. 让 CPU 处理更小、更干净的输入
2. 不让 CPU 同时承担高分辨率全图检测、每帧绘制、同步写盘和 UI
3. 只在必要频率上做“昂贵检测”

## 2. 官方能力边界

根据 ST 官方资料：

- `STM32MP25` 系列包含 `GPU / VPU / NPU`
- `GPU` 支持 `OpenGL ES`、`Vulkan`、`OpenCL`、`OpenVX`
- `NPU` 走 ST 的 AI 工具链，更适合执行转换后的神经网络

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

这些信息落到当前项目上的结论是：

- `OpenCV aruco.detectMarkers()` 不会自动把 AprilTag 主检测卸载到 `MP257` 的 GPU
- AprilTag 经典视觉主流程不能原样直接塞给 NPU
- `GPU / OpenCL` 最多帮助部分图像操作
- `NPU` 更适合以后做“候选区域筛选网络”

## 3. 当前脚本已经补齐的有效优化

`src/apriltag_usb_detector.py` 现在有这些对 PC 和 `STM32MP257` 都成立的优化手段。

### 3.1 采集链路优化

- `--pixel-format {auto,mjpg,yuy2}`
  尝试控制相机像素格式。很多 USB 摄像头在 `MJPG` 下更容易拿到更高分辨率和更高帧率
- `--buffer-size`
  控制采集缓冲，减少延迟积压
- `--async-capture`
  后台线程抓最新帧，减轻主线程等待相机的时间

### 3.2 检测链路优化

- `--detect-scale`
  全图降采样检测
- `--roi-detect-scale`
  已有 ROI 时用单独倍率检测
- `--crop-width / --crop-height`
  高分辨率采集，但只在中心裁剪区检测
- `--use-roi-tracking`
  优先在上一帧目标附近搜索
- `--roi-padding`
  控制跟踪 ROI 的扩展范围
- `--max-detect-hz`
  限制昂贵检测的执行频率
- `--full-frame-interval`
  每 N 次检测强制全图重搜一次

### 3.3 非检测负载优化

- `--snapshot-hz`
  控制 JSON 输出频率
- `--snapshot-pretty`
  可切换成更易读但更重的格式化输出
- 后台异步写快照
  避免同步写盘拖慢主循环
- `--no-gui`
  只输出检测结果，不打开窗口
- 无 GUI 模式下不再做无意义绘制
  这一点对嵌入式部署尤其重要

### 3.4 交互和诊断优化

- `--list-cameras`
  扫描摄像头索引
- `--diagnose-accel`
  打印 OpenCV / OpenCL / CUDA 状态
- `--print-config`
  输出启动后解析出的实际配置
- 热键：
  - `r` 重置 ROI 并重新全图搜索
  - `f` 强制下一次检测全图搜索
  - `p` 打印当前状态

## 4. 为什么 1080p 会明显更慢

以常见两档为例：

- `1920x1080` 约 `2.07 MP`
- `960x540` 约 `0.52 MP`

1080p 的像素量大约是 540p 的 `4 倍`。

而 AprilTag 的慢，不只是像素变多，还会叠加：

- 候选四边形搜索
- 解码
- 角点精修
- 位姿估计
- 绘制
- JSON 写盘

所以你看到“540p 很顺、1080p 明显掉下来”是正常现象。

## 5. 为什么现在不把“GPU 加速”当第一优先级

你当前的性能问题通常不是单一算子慢，而是整条链路的系统性负载：

- 相机给高分辨率全图
- Python 收全图
- CPU 做全图搜索
- 每帧都做完整检测
- 每帧都画图
- 每次快照都同步落盘

即使某些前处理勉强用上了 `OpenCL`，只要主检测还是高分辨率全图 CPU 搜索，收益就不会像直觉里那样大。

所以优先级应该是：

1. 调整相机采集格式
2. 降分辨率或限 ROI
3. 控制检测频率
4. 去掉不必要绘制和写盘
5. 再考虑 GPU / NPU 的局部卸载

## 6. 推荐运行方式

### 6.1 PC 默认调试

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --profile balanced --pixel-format mjpg --width 1280 --height 720 --fps 30 --tag-size-m 0.08
```

### 6.2 追求速度

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --profile speed --pixel-format mjpg --width 960 --height 540 --fps 30 --tag-size-m 0.08 --no-draw-axes
```

### 6.3 1080p 采集，但只看中心区域

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --profile balanced --pixel-format mjpg --width 1920 --height 1080 --fps 30 --crop-width 960 --crop-height 540 --tag-size-m 0.08
```

如果你还需要继续减负载：

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --profile balanced --pixel-format mjpg --width 1920 --height 1080 --fps 30 --crop-width 960 --crop-height 540 --detect-scale 0.75 --max-detect-hz 10 --tag-size-m 0.08
```

### 6.4 提前模拟 `STM32MP257`

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --profile mp257 --width 640 --height 480 --fps 30 --tag-size-m 0.08 --no-draw-axes
```

### 6.5 面向后续控制链路的无 GUI 模式

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --profile mp257 --width 640 --height 480 --fps 30 --tag-size-m 0.08 --no-gui --snapshot-hz 5
```

如果要尽量减轻 CPU：

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --profile mp257 --width 640 --height 480 --fps 30 --tag-size-m 0.08 --no-gui --snapshot-hz 2 --max-detect-hz 8 --no-draw-axes
```

## 7. 在 STM32MP257 上更合理的相机链路

更合理的落地方式不是让 Python 自己拿 `1920x1080` 再慢慢缩，而是：

- 在 `DCMIPP / V4L2` 层先裁剪
- 在采集链路先缩放
- 应用层尽量只接 `640x480` 或 `960x540`

这类优化通常比“单纯多开一点线程”更有价值。

## 8. 对 MP257 最实用的三阶段路线

### 阶段 A

- 摄像头
- `V4L2 / DCMIPP` 预裁剪缩放
- CPU AprilTag

特点：最容易落地，工程复杂度最低。

### 阶段 B

- 摄像头
- 应用层接较小分辨率图像
- CPU AprilTag 检测
- 只保留必要绘制和较低频率快照

特点：最符合当前项目实际需求。

### 阶段 C

- 摄像头
- NPU 跑轻量候选区域网络
- 输出“可能有标签的区域”
- CPU 只在 ROI 内做 AprilTag 解码

这才是 NPU 真正有价值的位置。

## 9. 如果以后一定要用 NPU

建议不是把 AprilTag 主流程网络化，而是训练一个小模型做：

- `tag board / fiducial` 候选框检测
- 或简单热区定位

要求：

- 模型足够小
- 最好量化
- 用 `TensorFlow Lite` 或 `ONNX`
- 再转换进 ST 的工具链

## 10. 一句话结论

对 `STM32MP257` 来说：

- AprilTag 主解码仍应按 `CPU 经典视觉` 设计
- GPU 更适合显示和部分预处理
- NPU 更适合以后做候选区域筛选

真正关键不是“强行让 AprilTag 跑 GPU / NPU”，而是：

> 用对硬件分工，让 CPU 不再处理全分辨率全图搜索。
