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

The live detector now keeps the original BGR frame for display, but sends a
separate preprocessed frame into `nvAprilTagsDetect`. The default preprocessing
is:

```text
gray_blur_gamma07
```

That means: BGR camera frame -> grayscale -> small 3x3 Gaussian blur -> gamma
0.70 -> BGRA upload -> NVIDIA GPU detector. This preserves the original video
view while improving the detector input under the current dark lighting.

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
  --preprocess gray_blur_gamma07 \
  --calib-json /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json \
  --output-json /home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json
```

Add `--draw-axes` only when orientation arrows are needed.

Useful preprocess modes:

```text
raw
equalize
clahe
gamma06
color_gamma06
color_gamma045
gain
y_equalize
y_clahe
gray_blur_gamma045
gray_blur_gamma05
gray_blur_gamma06
gray_blur_gamma07
gray_median_gamma06
gray_sharp_gamma06
```

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

Current dark-scene verification on `192.168.1.80`:

| Preprocess | Frames | FPS | Frames With Tags | Preprocess Avg | Detect Avg |
| --- | ---: | ---: | ---: | ---: | ---: |
| `raw` | 126 | 20.95 | 2 | 1.16 ms | 15.16 ms |
| `equalize` | 126 | 20.94 | 0 | 4.63 ms | 15.64 ms |
| `gray_blur_gamma06` | 126 | 20.91 | 24 | 2.86 ms | 22.54 ms |
| `gray_blur_gamma07` | 252 | 20.96 | 132 | 2.82 ms | 21.13 ms |
| `gray_blur_gamma07` + duplicate-ID filter + 180 ms JSON hold | 252 | 20.95 | 206 | 2.75 ms | 23.19 ms |

`gray_blur_gamma07` is the current default because it keeps the pipeline at the
sensor frame-rate ceiling while substantially improving tag recognition. It is
still the NVIDIA GPU detector, not a CPU fallback.

The GUI also keeps a short last-good overlay (`--gui-hold-ms`, default 350 ms)
so the box does not disappear on isolated missed frames. JSON output can reuse
the last detection for a short bounded window (`--output-hold-ms`, default
180 ms). Held detections are marked with:

```json
{
  "is_held": true,
  "held_ms": 73.0,
  "source_timestamp_unix": 1782240000.0
}
```

This is a bounded continuity aid for the sampler, not a replacement for real
detection. Set `OUTPUT_HOLD_MS=0` to disable it.

The bench also exposes Jetson ISP controls for experiments:

```bash
EXPOSURE_COMPENSATION=0.7 TNR_MODE=2 TNR_STRENGTH=0.7 bash run_fullfov_1280x960_gui.sh
GAINRANGE="1 8" ISPDIGITALGAINRANGE="1 4" bash run_fullfov_1280x960_gui.sh
```

In the current scene these ISP changes did not beat the base camera settings;
the best measured result remained the base camera path plus `gray_blur_gamma07`.

## Calibration Caveat

The JSON calibration is OpenCV fisheye, but NVIDIA `nvAprilTags` accepts pinhole `fx/fy/cx/cy` only. This program passes the calibrated matrix and does not undistort the image. For robot closed-loop use, verify measured XYZ against known tag distances and consider adding an undistort step if edge-of-frame pose error is too large.
