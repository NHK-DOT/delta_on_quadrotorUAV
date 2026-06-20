# YOLO 鱼眼相机标定说明

这个文档对应末端/侧向的 YOLO 扳手识别相机，不是底座 AprilTag 相机。

你现在的分工是：

- 底座相机：AprilTag 相机，用 `calibration/camera_intrinsics.json`。
- 末端/侧向相机：YOLO 扳手识别相机，160 度鱼眼镜头，用 `calibration/yolo_fisheye_camera_intrinsics.json`。

原来的 `src/calibrate_camera.py` 使用普通针孔相机模型 `cv2.calibrateCamera`。160 度鱼眼镜头畸变很大，不建议直接用那个脚本。鱼眼相机应使用 `cv2.fisheye.calibrate`，所以这里另建了脚本：

```text
src/calibrate_fisheye_camera.py
```

## 1. 准备棋盘格

推荐用普通黑白棋盘格。

参数含义：

- `--cols`：每一行的内角点数量，不是格子数量。
- `--rows`：每一列的内角点数量，不是格子数量。
- `--square-size-m`：单个格子的真实边长，单位是米。

如果你的棋盘格是 10 x 7 个方格，那么内角点通常是：

```text
cols = 9
rows = 6
```

如果单格边长是 25 mm：

```text
square-size-m = 0.025
```

## 2. 查相机编号

在 `AprilTag_Vision/myAprilTag` 目录下执行：

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm\AprilTag_Vision\myAprilTag
python src\apriltag_usb_detector.py --list-cameras
```

找到 YOLO 扳手相机对应的 `camera-index`。不要把它和底座 AprilTag 相机编号混用。

## 3. 开始标定

示例命令：

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm\AprilTag_Vision\myAprilTag
python src\calibrate_fisheye_camera.py --camera-index 0 --backend dshow --width 1920 --height 1080 --cols 9 --rows 6 --square-size-m 0.025
```

如果 YOLO 相机不是 `0`，把 `--camera-index 0` 改成实际编号。

## 3.1 Jetson 软排 CSI 相机

如果 YOLO 鱼眼相机接在 Jetson 的软排 CSI 接口上，不走 `/dev/video0` 的普通 USB 打开方式，而是走 `nvarguscamerasrc`：

```bash
python3 src/calibrate_fisheye_camera.py --source csi --sensor-id 0 --flip-method 0 --width 1920 --height 1080 --fps 30 --cols 9 --rows 6 --square-size-m 0.025
```

常用参数：

- `--source csi`：使用 Jetson CSI 软排相机。
- `--sensor-id 0`：第 0 路 CSI 相机。多相机时可能要改成 `1`。
- `--flip-method 0`：画面旋转/翻转方式。安装方向不对时可以改 `2`、`4`、`6` 试。

如果你的相机驱动不是 `nvarguscamerasrc`，可以自己给 GStreamer 管线：

```bash
python3 src/calibrate_fisheye_camera.py --source gstreamer --gst-pipeline "你的 GStreamer pipeline" --cols 9 --rows 6 --square-size-m 0.025
```

软排线只是连接方式，不改变鱼眼标定模型。只要后续使用时还是同一颗镜头、同一分辨率、同一对焦和同一安装位置，这份标定就有效。

运行后会打开预览窗口：

- `space`：当前画面识别到完整棋盘格时，保存一张有效样本。
- `c`：样本数量足够后执行标定。
- `q`：退出，不写标定结果。

默认至少需要 20 张有效样本。

## 4. 样本怎么采

鱼眼镜头必须覆盖画面不同区域，否则边缘畸变会拟合不好。

采样时让棋盘格覆盖这些位置：

- 画面中心。
- 左上、右上、左下、右下四个角。
- 左边缘、右边缘、上边缘、下边缘。
- 近距离和远距离。
- 正对镜头、向左倾斜、向右倾斜、向上倾斜、向下倾斜。

不要只在画面中心采样。160 度鱼眼的主要误差通常在画面边缘。

## 5. 输出文件

脚本默认写入：

```text
calibration/yolo_fisheye_camera_intrinsics.json
```

采集图默认保存到：

```text
calibration/captures_yolo_fisheye/
```

这个输出文件是另创的，不会覆盖底座 AprilTag 相机的：

```text
calibration/camera_intrinsics.json
```

## 6. 结果怎么看

输出 JSON 里关键字段：

- `model: "opencv_fisheye"`：表示这是 OpenCV 鱼眼模型。
- `camera_role: "yolo_object_camera"`：表示这是 YOLO/object 相机。
- `camera_matrix`：相机内参矩阵。
- `dist_coeffs`：鱼眼畸变参数，通常是 4 个值。
- `rms_reprojection_error`：重投影误差，越小越好。
- `image_size`：标定时使用的分辨率。

后续 YOLO 识别如果要做去畸变或像素到空间的换算，应读取这个鱼眼文件，而不是读取 `camera_intrinsics.json`。

## 7. 常见问题

如果棋盘格经常识别不到：

- 增加光照。
- 避免反光。
- 让棋盘格完整出现在画面里。
- 不要让棋盘格太靠近镜头导致严重模糊。
- 先用 1280 x 720 标定确认流程，再换回正式分辨率。

如果按 `c` 后标定失败或误差很大：

- 多采几张边缘和角落位置。
- 删除质量差的采集图后重新采。
- 确认 `cols/rows` 是内角点数。
- 确认 `square-size-m` 单位是米。

如果提示 `CHECK_COND` 失败，可以临时加：

```powershell
python src\calibrate_fisheye_camera.py --camera-index 0 --backend dshow --cols 9 --rows 6 --square-size-m 0.025 --no-check-cond
```

但这只是放宽求解条件。更推荐补采更多角度和边缘样本。
