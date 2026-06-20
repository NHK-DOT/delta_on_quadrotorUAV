# AprilTag Camera Vision

## 78arm 双相机手眼对接

当前 78arm 的手眼 demo 位于 `../../Dual_Camera_HandEye/`。本目录继续只负责
USB 相机打开、像素格式协商、AprilTag 检测和 JSON 快照输出，不承担机械臂控制。

- 底座相机运行本检测器后，`output/apriltag_latest.json` 可作为
  `base_camera_T_hand_tag` 的来源：它观察末端执行器上表面的 AprilTag。
- 执行机构侧面相机运行本检测器或同格式物体识别输出后，快照可作为
  `object_camera_T_object` 的来源：它观察待抓取物体。
- `Dual_Camera_HandEye/demo.py snapshot-transform` 可以把本目录的
  `output/apriltag_latest.json` 转换成坐标变换 JSON。
- `Dual_Camera_HandEye/demo.py project-object` 可以结合当前 `base_T_tool` 和
  `tool_T_object_camera`，输出物体在机械臂基座坐标系下的位置。

本目录不打开舵机串口，也不发送运动命令。

一个面向 USB 摄像头和机载摄像头的独立 AprilTag 检测项目。

这个项目的目标很明确：

- 用普通摄像头稳定识别 `AprilTag`
- 输出标签 ID、像素位置、近似距离或标定位姿
- 用配置文件管理参数，不靠超长命令行
- 优先把相机链路协商到高帧率视频模式
- 在主机侧先把整条链路跑顺

当前项目已经具备：

- 标签生成
- 相机标定
- 实时检测与位姿估计
- JSON 快照输出
- 摄像头模式测速
- `MJPG` 路径优先策略
- 运行时性能分段计时

## 项目定位

这不是一个“只研究 AprilTag 算法论文”的仓库，而是一个能直接拿来调相机、调模式、跑识别的工程项目。

这套项目当前最重要的经验不是“怎么上 GPU”，而是：

> 对普通 UVC 摄像头来说，视频传输格式往往比检测算法本身更先决定整条链路的帧率上限。

尤其是同一颗摄像头通常会同时支持：

- `MJPG`
  压缩传输，通常高帧率
- `YUYV / YUY2`
  未压缩传输，高分辨率下往往低帧率

所以本项目当前优先做的是：

1. 让摄像头尽量走 `MJPG` 传输路径
2. 到主机后允许驱动/DirectShow解码成 `RGB24`
3. 不接受掉回 `YUY2/YUYV` 的低帧率路径

## 目录结构

- `src/apriltag_usb_detector.py`
  主检测脚本
- `src/calibrate_camera.py`
  棋盘格标定脚本
- `src/calibrate_fisheye_camera.py`
  YOLO/object 160 度鱼眼相机标定脚本，使用说明见 `docs/YOLO_FISHEYE_CAMERA_CALIBRATION.md`
- `src/generate_apriltags.py`
  标签生成脚本
- `config/apriltag_detector.toml`
  默认配置文件，日常主要改这里
- `calibration/`
  标定文件和采集图
- `output/`
  JSON 输出和抓拍图
- `image/`
  已生成的标签图
- `docs/`
  项目附加说明
- `papers/`
  参考论文

## 依赖安装

```powershell
cd C:\Users\hanjuncheng\Desktop\nodejs\AprilTag_Vision\myAprilTag
python -m pip install -r requirements.txt
```

当前依赖：

- `numpy`
- `opencv-contrib-python`
- `Pillow`

## 最常用的启动方式

这个项目默认走配置文件，不需要每次手敲很长的命令。

直接运行：

```powershell
cd C:\Users\hanjuncheng\Desktop\nodejs\AprilTag_Vision\myAprilTag
python src\apriltag_usb_detector.py
```

脚本会自动读取：

[apriltag_detector.toml](C:/Users/hanjuncheng/Desktop/nodejs/AprilTag_Vision/myAprilTag/config/apriltag_detector.toml)

也就是说，日常调参优先改配置文件，而不是命令行。

## 当前默认策略

当前默认配置已经偏向 USB 摄像头高帧率路径：

- `backend = "dshow"`
- `pixel_format = "mjpg"`
- `strict_pixel_format = true`
- `disable_rgb_convert = false`
- `width = 1280`
- `height = 720`
- `fps = 60`

这套配置的含义是：

1. 用标准方式请求摄像头走 `MJPG`
2. 允许主机侧把 `MJPG` 解码成 `RGB24`
3. 如果最终掉回 `YUY2/YUYV`，脚本直接退出

换句话说：

> 当前项目允许 `MJPG` 和 `RGB24`，拒绝 `YUY2/YUYV`。

这样做是因为：

- `MJPG`
  代表压缩传输路径
- `RGB24`
  往往代表主机侧已经把 `MJPG` 解码后再交给 OpenCV
- `YUY2/YUYV`
  通常意味着回到了未压缩低帧率路径

## 配置文件说明

主要配置都在：

[apriltag_detector.toml](C:/Users/hanjuncheng/Desktop/nodejs/AprilTag_Vision/myAprilTag/config/apriltag_detector.toml)

里面每一项前面都已经写了注释。最关键的参数有这些：

### 相机相关

- `camera_index`
  选择哪个摄像头
- `backend`
  Windows 常用 `dshow`
- `pixel_format`
  当前建议 `mjpg`
- `strict_pixel_format`
  开启后，如果最终不是 `MJPG` 或 `RGB24` 就报错退出
- `disable_rgb_convert`
  当前默认 `false`，也就是允许主机侧解码成 RGB 再交给 OpenCV
- `width`
  请求采集宽度
- `height`
  请求采集高度
- `fps`
  请求帧率

### 检测相关

- `process_width`
- `process_height`

这两个参数很重要。

它们的作用是：

- 相机仍按较高分辨率取流
- 进入检测前，先把整张图缩小
- 保留完整视角
- 降低检测像素密度

例如：

- 相机采集 `1280x720`
- 检测处理 `960x540`

这是“保留视角，减轻检测负担”的做法，不是中心裁剪。

### 识别性能相关

- `profile`
  当前默认 `speed`
- `detect_scale`
  已经做了 `process_width/process_height` 时，通常建议保持 `1.0`
- `max_detect_hz`
  控制识别频率上限
- `refine`
  `none / subpix / apriltag`
- `draw_axes`
  是否画位姿坐标轴

### 输出相关

- `snapshot_file`
- `capture_dir`
- `snapshot_hz`
- `snapshot_pretty`

## 相机模式测速

这个项目专门加了“纯取流测速模式”，目的不是跑识别，而是确认：

- 哪个分辨率真正能打开
- 哪个像素格式实际协商成功
- 哪个模式的真实新帧率最高

不跑识别，只跑测速：

```powershell
python src\apriltag_usb_detector.py --benchmark-capture
```

批量扫描：

```powershell
python src\apriltag_usb_detector.py --camera-index 0 --backend dshow --fps 60 --benchmark-capture --benchmark-seconds 5 --benchmark-grid "1920x1080;1280x720;960x540;640x480" --benchmark-pixel-formats all
```

它会输出：

- 请求分辨率
- 实际打开分辨率
- 实际格式
- `delivered_fps`
- `new_fps`
- `median_interval`
- `repeats`

其中最重要的是：

- `actual_pixel_format`
- `actual_width/actual_height`
- `new_fps`

## 当前项目的核心经验

通过这套项目对摄像头模式的实测，可以总结出一个非常实用的结论：

### 对普通 UVC 摄像头

高帧率通常依赖：

- `MJPG`

而不是：

- `YUYV / YUY2`

原因很简单：

- `MJPG` 是压缩传输
- `YUYV/YUY2` 是未压缩传输
- 高分辨率下未压缩会把带宽吃满

### 对识别链路

只要相机成功走到 `MJPG` 路径，整条管线的主要压力通常就不再是“视频输出带宽”，而会转移到：

- 解码
- 显示
- 检测本体
- 位姿估计

本项目已经内建了分段耗时统计，会在终端打印：

- `capture_wait`
- `resize`
- `gray`
- `detect`
- `pose`
- `gui`

这样你能直接看到当前瓶颈在哪一段。

## 检测管线

当前主检测脚本的实际流程是：

1. 打开摄像头
2. 尽量协商到 `MJPG`
3. 读取帧
4. 如果配置了 `process_width/process_height`，先整帧缩小
5. 再对缩小后的帧做灰度化
6. 用 OpenCV ArUco/AprilTag 路线检测
7. 如果有标定文件，做位姿估计
8. 输出 JSON

也就是说，当前项目优先保证：

- 视角完整
- 管线稳定
- 相机模式正确

## 标定

如果要做更准确的位姿估计，先标定相机：

```powershell
python src\calibrate_camera.py --camera-index 0 --backend dshow --cols 9 --rows 6 --square-size-m 0.025
```

输出文件默认是：

[camera_intrinsics.json](C:/Users/hanjuncheng/Desktop/nodejs/AprilTag_Vision/myAprilTag/calibration/camera_intrinsics.json)

存在这个文件时，主检测脚本会进入：

- `calibrated_pose`

否则会退回：

- `size_based_approximation`

## 标签生成

生成 AprilTag 标签：

```powershell
python src\generate_apriltags.py
```

当前已经附带一批可直接打印的标签图，位于：

[image](C:/Users/hanjuncheng/Desktop/nodejs/AprilTag_Vision/myAprilTag/image)

## 输出 JSON

默认输出文件：

[apriltag_latest.json](C:/Users/hanjuncheng/Desktop/nodejs/AprilTag_Vision/myAprilTag/output/apriltag_latest.json)

主要字段包括：

- `camera`
- `tag_family`
- `tag_size_m`
- `estimation_mode`
- `performance_profile`
- `timing`
- `roi`
- `detections`

每个 detection 包含：

- `id`
- `center_px`
- `size_px`
- `position_m`
- `normalized_xy`

## 常用运行方式

### 1. 直接按配置跑

```powershell
python src\apriltag_usb_detector.py
```

### 2. 只测速，不识别

```powershell
python src\apriltag_usb_detector.py --benchmark-capture
```

### 3. 打印运行配置

```powershell
python src\apriltag_usb_detector.py --print-config
```

### 4. 查看可用摄像头

```powershell
python src\apriltag_usb_detector.py --list-cameras
```

## 运行时热键

- `q`
  退出
- `s`
  保存当前标注画面
- `r`
  重置 ROI 跟踪
- `f`
  强制下一次做全图搜索
- `p`
  打印当前状态
- `h`
  打印帮助

## 当前项目边界

这个项目当前的主目标不是“强行让 AprilTag 本体跑 GPU/NPU”，而是：

- 把摄像头模式协商对
- 把 CPU 路径跑顺
- 把配置管理和测速工具补齐

也就是说，当前重点是：

> 先让普通主机 + 普通摄像头的 AprilTag 链路稳定、清晰、可重复配置。

在这个目标下，当前项目已经足够独立成一个单独工程。

## 参考资料

- AprilTag 官方项目  
  https://github.com/AprilRobotics/apriltag
- Olson 2011  
  https://april.eecs.umich.edu/media/pdfs/olson2011tags.pdf
- Wang 2016  
  https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf
- Krogius 2019  
  https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf
