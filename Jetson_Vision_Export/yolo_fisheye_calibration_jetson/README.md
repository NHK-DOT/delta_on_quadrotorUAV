# YOLO Fisheye Calibration Package For Jetson

这个小包用于 `192.168.1.64` 那台 Jetson 上的末端/侧向 YOLO 扳手识别鱼眼相机标定。

它不控制机械臂，不打开舵机串口，只打开相机并采集棋盘格标定图。

## 文件

- `calibrate_fisheye_camera.py`：OpenCV fisheye 标定脚本，兼容 Ubuntu 18.04 / Python 3.6。
- `run_csi_fisheye_calibration.sh`：软排 CSI 相机启动脚本。
- `run_usb_fisheye_calibration.sh`：USB 相机启动脚本，备用。
- `YOLO_FISHEYE_CAMERA_CALIBRATION.md`：完整中文说明。

## 软排 CSI 相机

在 Jetson 桌面解压后执行：

```bash
cd ~/Desktop/yolo_fisheye_calibration_jetson
bash run_csi_fisheye_calibration.sh
```

默认参数：

- `sensor-id=0`
- `flip-method=0`
- `1920x1080`
- `30 fps`
- 棋盘格内角点 `9 x 6`
- 单格边长 `0.025 m`

如果安装方向不对：

```bash
FLIP_METHOD=2 bash run_csi_fisheye_calibration.sh
```

如果是第二路 CSI：

```bash
SENSOR_ID=1 bash run_csi_fisheye_calibration.sh
```

## USB 相机备用命令

```bash
cd ~/Desktop/yolo_fisheye_calibration_jetson
bash run_usb_fisheye_calibration.sh
```

## 热键

- `space`：保存当前有效棋盘格样本。
- `c`：样本数量足够后执行标定。
- `q`：退出。

## 输出

默认输出：

```text
calibration/yolo_fisheye_camera_intrinsics.json
```

采集图：

```text
calibration/captures_yolo_fisheye/
```

这份文件是末端 YOLO 鱼眼相机的内参，不要覆盖底座 AprilTag 相机的 `camera_intrinsics.json`。
