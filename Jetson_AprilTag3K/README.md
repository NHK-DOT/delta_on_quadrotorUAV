# Jetson Full-FOV GPU AprilTag

This folder keeps the Jetson path that worked on 2026-06-23 for the CSI fisheye camera.

## Working Pipeline

```text
IMX219 CSI camera mode 0
3264x2464 @ 21 fps full sensor FOV
-> nvvidconv downsample
-> 1280x960 BGR frame
-> NVIDIA nvAprilTags GPU detector
-> Jetson local OpenCV GUI + latest JSON snapshot
```

This is not native 1080p or 720p camera mode. Those modes crop the sensor and lose the fisheye FOV advantage.

## Main Jetson Command

On the Jetson desktop:

```bash
cd /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench
./run_fullfov_1280x960_gui.sh
```

The GUI shows live FPS, tag count, detector latency, and XYZ in meters. The process writes:

```text
/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json
```

That JSON uses the same `detections[].position_m` shape consumed by the arm-side Python tools.

## Calibration

The usable calibration is stored in:

```text
calibration/fullfov_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json
```

It is calibrated for the final 1280x960 image used by detection, while preserving full 3K FOV through downsampling.

Latest result:

```text
raw images: 100
valid checkerboards: 32
RMS reprojection error: 0.735554 px
image size: 1280x960
checkerboard: 10x7 internal corners
square size: 0.020 m
```

OpenCV `CALIB_CHECK_COND` failed on this sample set, then calibration succeeded without that flag. The RMS and sample count are good enough for current robot testing, but validate AprilTag pose against known distances before closing the servo loop.

## What Was Rejected

- Native 720p/1080p camera modes: fast, but crop the sensor FOV.
- Full native 3264x2464 AprilTag detection: too slow on this Jetson.
- Old CPU C++ AprilTag path: only a few FPS and not worth keeping as the primary path.
- Browser MJPEG stream: extra latency and not needed for Jetson-local testing.

## Contents

- `gpu_nvapriltags_bench/`: buildable NVIDIA GPU AprilTag detector and launch scripts.
- `calibration/fullfov_3k_downsample_1280x960/`: current usable intrinsics and calibration notes.

The NVIDIA `nvAprilTags.h` and `libapril_tagging.a` files are present on the Jetson working directory. They are NVIDIA-provided JetPack 4.4 artifacts; check licensing before redistributing them publicly.
