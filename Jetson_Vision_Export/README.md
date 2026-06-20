# Jetson Vision Export

This folder stores the Jetson vision deployment package recovered from the SSD and migrated during the NX recovery work.

## Files

- `jetson_vision_export_20260612_0215.tar.gz`: application-level deployment archive, about 250 MB.
- `jetson_vision_export_20260612_0215.tar.gz.sha256`: portable SHA-256 checksum for the archive.
- `install_jetson_vision_export.sh`: installer script copied with the package. It downloads the archive from an HTTP server and installs it on a Jetson target.
- `yolo_fisheye_calibration_jetson/`: small desktop package for the Jetson-side YOLO/object 160-degree fisheye camera calibration.
- `yolo_fisheye_calibration_jetson.tar.gz`: archive of that desktop calibration package.
- `yolo_fisheye_calibration_jetson.tar.gz.sha256`: checksum for the fisheye calibration archive.

The archive contains:

- `/home/nvidia/vision_starter`: TensorRT YOLO service, HTTP preview server, UDP output script, ONNX and engine model files, datasets, and helper scripts.
- `/home/nvidia/orbbec_sdk`: Orbbec SDK files plus `depth_center_json.cpp` and `depth_grid_daemon.cpp`.
- `/etc/systemd/system`: service files for `jetson-vision.service`, `orbbec-depth-grid.service`, and `vision-coco-depth.service`.
- `/etc/udev/rules.d`: Orbbec USB udev rule.
- `docker_info`: metadata showing the original Docker base as `nvcr.io/nvidia/l4t-jetpack:r35.4.1`.

## Verify

```bash
sha256sum -c jetson_vision_export_20260612_0215.tar.gz.sha256
```

Expected digest:

```text
317c7c632ae5e45a3292ddf13856c63d93770750ce76bd1f311c4fc5b728d346
```

## Deploy By HTTP

Serve this directory from the machine that holds the package:

```bash
cd Jetson_Vision_Export
python3 -m http.server 8765 --bind 0.0.0.0
```

On the Jetson target:

```bash
wget http://<server-ip>:8765/install_jetson_vision_export.sh
chmod +x install_jetson_vision_export.sh
sudo SERVER=<server-ip>:8765 ./install_jetson_vision_export.sh
```

The installer writes only application/service-layer files. It does not install a kernel, DTB, DTBO, UEFI image, or camera device tree.

## JetPack 4 Notes

Test target: `192.168.1.64`, JetPack R32.4.4, Ubuntu 18.04, Python 3.6.9, TensorRT 7.1.3.

After install on that board:

- `depth_center_json` and `depth_grid_daemon` were rebuilt locally with `g++`.
- TensorRT engines were rebuilt locally from ONNX:
  - `models/yolov8n_320_trt7_fp16.engine`
  - `models/wrench_320_trt7_fp16.engine`
  - `models/wrench_public_neg_320_trt7_fp16.engine`
  - `models/snow_king_320_trt7_fp16.engine`
- `models/yolov8n_320_fp16.engine` was replaced with the TensorRT 7 engine so the default service can deserialize it on JetPack 4.
- `scripts/trt_yolo_server.py` needed a Python 3.6 fallback for `ThreadingHTTPServer`, because Python 3.6 does not provide that class.

The final smoke test on `192.168.1.64` reached:

```text
http://192.168.1.64:8090/latest.json
```

and returned JSON. At the time of the test, no `/dev/video*` device and no Orbbec USB device were connected, so the service correctly reported `camera_open_failed`.

## Useful Commands

Run default COCO service:

```bash
sudo systemctl restart jetson-vision.service
curl http://127.0.0.1:8090/latest.json
```

Run wrench model manually:

```bash
cd /home/nvidia/vision_starter
ENGINE=models/wrench_320_trt7_fp16.engine LABEL=wrench ./run_coco_depth_service.sh
```

Enable Orbbec depth service only after an Orbbec device appears in `lsusb`:

```bash
lsusb | grep -Ei '2bc5|orbbec'
sudo systemctl enable --now orbbec-depth-grid.service
```

## YOLO Fisheye Calibration On Jetson Desktop

For the 160-degree YOLO/object camera connected by Jetson CSI ribbon cable, copy
or extract the small calibration package on the target desktop:

```bash
cd ~/Desktop
tar -xzf yolo_fisheye_calibration_jetson.tar.gz
cd yolo_fisheye_calibration_jetson
bash run_csi_fisheye_calibration.sh
```

This package only opens the camera and runs OpenCV fisheye calibration. It does
not open the servo serial port and does not move the arm.
