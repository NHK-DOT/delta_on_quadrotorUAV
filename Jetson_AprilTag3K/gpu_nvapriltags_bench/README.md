# NVIDIA nvAprilTags GPU Detector on Jetson

Tested target:

- Jetson Xavier NX Developer Kit, JetPack/L4T R32.4.4, CUDA 10.2
- Camera: CSI IMX219 fisheye
- GPU detector: NVIDIA `nvAprilTags`, tag family `tag36h11`

## Build

The Jetson working directory already contains `nvAprilTags.h` and `libapril_tagging.a`.

```bash
cd /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench
./build_jetson.sh
```

## Run GUI

```bash
cd /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench
./run_fullfov_1280x960_gui.sh
```

The script:

- stops the old `jetson-vision.service` camera consumer
- enables `jetson_clocks`
- restarts `nvargus-daemon`
- captures full sensor `3264x2464@21`
- downsamples to `1280x960`
- opens a local Jetson GUI
- writes `/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json`

The GUI is color because OpenCV receives BGR frames, then uploads BGRA to CUDA. The detector itself runs through NVIDIA's GPU library.

## Direct Command

```bash
./nv_gpu_apriltag_bench \
  --mode 0 \
  --sensor 3264x2464 \
  --sensor-fps 21 \
  --out 1280x960 \
  --seconds 0 \
  --warmup 8 \
  --gui \
  --calib-json /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json \
  --output-json /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json
```

Add `--draw-axes` only when orientation arrows are needed.

## Useful Results

All full-FOV rows capture IMX219 mode 0 at `3264x2464@21`; the listed size is the downsampled frame sent to the detector.

| Pipeline | Clocks | FPS | Detect avg | Result |
| --- | --- | ---: | ---: | --- |
| full FOV -> 960x724 | `jetson_clocks` | about 21 | about 16 ms | stable and detects tags |
| full FOV -> 1280x960 | `jetson_clocks` | about 21 | about 26 ms | current chosen path |
| full FOV -> 1920x1448 | `jetson_clocks` | about 13-19 | about 36 ms | visible lag and weak detection |
| native 720p crop | dynamic | high | low | rejected because FOV is cropped |
| native full 3264x2464 | dynamic | about 7 | about 90 ms | too slow |

The sensor mode is the 21 fps ceiling. With `jetson_clocks`, the GPU detector is not the primary frame-rate limit at 1280x960.

## Calibration Caveat

The JSON calibration is OpenCV fisheye, but NVIDIA `nvAprilTags` accepts pinhole `fx/fy/cx/cy` only. This program passes the calibrated matrix and does not undistort the image. For robot closed-loop use, verify measured XYZ against known tag distances and consider adding an undistort step if edge-of-frame pose error is too large.
