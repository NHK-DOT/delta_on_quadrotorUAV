# vision_test

STM32MP257F-DK 视觉推理测试 ROS 2 包。支持 ST NPU (stai_mpu) 和 CPU (onnxruntime) 双后端。

## 代码结构

```
vision_test/
├── package.xml                          # ROS 2 包描述
├── setup.py                             # Python 包构建配置
├── setup.cfg                            # 安装路径配置
├── config/
│   └── vision_test.yaml                 # 默认参数配置
├── launch/
│   └── vision_test.launch.py            # ROS 2 launch 文件
├── resource/
│   └── vision_test                      # ament 资源索引标记
└── vision_test/
    ├── __init__.py
    ├── inference_engine.py              # 推理引擎（核心）
    ├── bench_node.py                    # ROS 2 benchmark 节点
    └── system_monitor.py                # 系统资源监控
```

### inference_engine.py — 推理引擎

根据模型文件扩展名自动选择后端：

| 扩展名 | 后端 | 说明 |
|--------|------|------|
| `.nb` | stai_mpu (NPU) | ST X-LINUX-AI Neural-ART Binary，INT8 量化，硬件加速 |
| `.onnx` | onnxruntime (CPU) | ONNX Runtime，FP32，纯 CPU 推理 |

后处理自动检测输出格式：
- `(1, 5, N)` — 单类别模型（如 person-only），4 bbox + 1 confidence
- `(1, 84, N)` — 80 类 COCO YOLOv8，4 bbox + 80 class scores

### bench_node.py — ROS 2 节点

三种运行模式：
- `synthetic`（默认）— 生成随机图像，纯测推理性能，不需要摄像头
- `v4l2` — 直接读取 V4L2 摄像头，不需要 ROS 2 相机驱动和 GUI
- `subscribe` — 订阅 `/camera/image_raw` 话题，需要 ROS 2 相机驱动

### system_monitor.py — 系统监控

零依赖，直接读取 `/proc/meminfo` 和 `/proc/stat`，后台线程每秒采样。

## 使用命令

### 方式 1：直接在板子上运行（推荐，不依赖 Docker）

```bash
# 进入工作目录
cd /usr/local/Uav_Delta_capture/uav_delta_capture

# 默认配置运行（NPU + 合成图像 + 100 轮）
python3 -m vision_test.bench_node --ros-args -p use_npu:=true

# 自定义参数
python3 -m vision_test.bench_node --ros-args \
  -p model_path:=/usr/local/x-linux-ai/object-detection/models/coco_ssd_mobilenet/ssd_mobilenet_v2_fpnlite_10_256_int8_per_tensor.nb \
  -p input_size:=256 \
  -p use_npu:=true \
  -p num_iterations:=50 \
  -p mode:=synthetic

# 使用 CPU 后端（onnxruntime）
python3 -m vision_test.bench_node --ros-args \
  -p model_path:=/usr/local/Uav_Delta_capture/models/yolov8n.onnx \
  -p use_npu:=false

# V4L2 摄像头模式（直接读取，不需要 ROS 2 相机驱动）
python3 -m vision_test.bench_node --ros-args \
  -p mode:=v4l2 \
  -p camera_device:=/dev/video7 \
  -p camera_width:=640 \
  -p camera_height:=480

# V4L2 模式 + 保存检测帧（事后查看）
python3 -m vision_test.bench_node --ros-args \
  -p mode:=v4l2 \
  -p save_frames:=true

# 订阅 ROS 2 话题（需要先有相机驱动发布 image_raw）
python3 -m vision_test.bench_node --ros-args \
  -p mode:=subscribe \
  -p image_topic:=/camera/image_raw
```

### 方式 2：通过 ROS 2 launch（需要 colcon 构建）

```bash
# 构建
colcon build --packages-select vision_test --symlink-install --parallel-workers 1
source install/setup.bash

# 默认运行
ros2 launch vision_test vision_test.launch.py

# 带参数
ros2 launch vision_test vision_test.launch.py use_npu:=true num_iterations:=50
```

### 方式 3：归档独立脚本测试（不依赖 ROS 2）

```bash
# NPU benchmark（stai_mpu）
python3 /usr/local/Uav_Delta_capture/archive_legacy/bench_npu.py

# CPU benchmark（onnxruntime）
python3 /usr/local/Uav_Delta_capture/archive_legacy/bench_standalone.py
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model_path` | YOLOv8n 320x320 .nb | 模型文件路径 |
| `input_size` | 320 | 推理输入分辨率 |
| `use_npu` | true | 是否使用 NPU（.nb 模型自动启用） |
| `num_iterations` | 100 | 推理轮数 |
| `warmup_iterations` | 10 | 预热轮数（不计入统计） |
| `mode` | synthetic | synthetic / v4l2 / subscribe |
| `image_topic` | /camera/image_raw | 订阅模式下的图像话题 |
| `camera_device` | /dev/video7 | V4L2 设备路径（v4l2 模式） |
| `camera_width` | 640 | 采集宽度 |
| `camera_height` | 480 | 采集高度 |
| `save_frames` | false | 保存检测帧到 /tmp/vision_test_frames/ |
| `report_path` | /tmp/vision_test_report.txt | 报告输出路径 |
| `conf_thresh` | 0.25 | 检测置信度阈值 |

## 板子上可用的 NPU 模型

| 模型 | 路径 | 输入 | 类别 |
|------|------|------|------|
| YOLOv8n person | `.../yolov8n_people/yolov8n_320_quant_pt_uf_od_coco-person-st.nb` | 320x320 | person |
| SSD MobileNetV2 | `.../coco_ssd_mobilenet/ssd_mobilenet_v2_fpnlite_10_256_int8_per_tensor.nb` | 256x256 | COCO 80 类 |

## 实测性能

| 后端 | 模型 | 输入 | FPS |
|------|------|------|-----|
| NPU (stai_mpu) | YOLOv8n INT8 .nb | 320x320 | **49.2** |
| CPU (onnxruntime) | YOLOv8n FP32 .onnx | 640x640 | 0.3 |

结论：NPU 推理 49.2 FPS，远超实时需求 (>=10 FPS)，板子可以本地运行视觉 + 运动学 + ROS。
