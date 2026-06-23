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
./run_motion_1280x960_gui.sh
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

The live detector keeps the original BGR frame for display, but sends a
separate preprocessed frame into `nvAprilTagsDetect`. The moving-arm default
preprocessing is:

```text
motion
```

That means: BGR camera frame -> grayscale -> unsharp mask -> gamma 0.70 -> BGRA
upload -> NVIDIA GPU detector. This preserves the original video view while
improving the detector input without blurring tag edges during motion.

For stationary dark-scene checks, `PREPROCESS=gray_blur_gamma07
./run_fullfov_1280x960_gui.sh` is still available, but it is no longer the
moving-arm default.

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
  --preprocess motion \
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
motion
motion_clahe
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
| `gray_blur_gamma07` + duplicate-ID filter + short JSON hold | 252 | 20.95 | 206 | 2.75 ms | 23.19 ms |

`gray_blur_gamma07` was the best measured stationary dark-scene preprocessing,
but it intentionally blurs before gamma. The moving-arm default is now `motion`
because a moving tag needs sharper edges more than stationary low-light
forgiveness. Both paths still use the NVIDIA GPU detector, not a CPU fallback.

The GUI and JSON output do not hold stale detections. The command-line
`GUI_HOLD_MS` and `OUTPUT_HOLD_MS` options are kept only for compatibility with
older launch commands; the current binary ignores hold and writes/draws only
real detections from the current frame. This is deliberate because stale boxes
visibly trail the tag during motion.

Older builds marked held detections with:

```json
{
  "is_held": true,
  "held_ms": 73.0,
  "source_timestamp_unix": 1782240000.0
}
```

Current moving-arm builds should always show `"is_held": false`.

The bench also exposes Jetson ISP controls for experiments:

```bash
EXPOSURE_COMPENSATION=0.7 TNR_MODE=2 TNR_STRENGTH=0.7 bash run_fullfov_1280x960_gui.sh
GAINRANGE="1 8" ISPDIGITALGAINRANGE="1 4" bash run_fullfov_1280x960_gui.sh
```

In the current scene these ISP changes did not beat the base camera settings;
the best measured result remained the base camera path plus `gray_blur_gamma07`.

## Moving Tag Optimization

If the tag is visible only when the arm stops, do not increase hold time. That
usually means the detector input is suffering from motion blur or low edge
contrast while the tag is moving. Keep hold disabled for moving-arm operation:

```bash
./run_motion_1280x960_gui.sh
```

`run_motion_1280x960_gui.sh` applies these defaults:

```bash
PREPROCESS=motion
GUI_HOLD_MS=0
OUTPUT_HOLD_MS=0
TNR_MODE=0
TNR_STRENGTH=0
EXPOSURETIMERANGE="34000 8000000"
GAINRANGE="1 12"
ISPDIGITALGAINRANGE="1 4"
```

If the image becomes too dark, add physical lighting before increasing exposure
again. Longer exposure makes the tag easier to see when stationary but worse
during motion.

Try these exposure ceilings in order while moving the tag at the real arm speed:

```bash
EXPOSURETIMERANGE="34000 6000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 8000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 10000000" ./run_motion_1280x960_gui.sh
EXPOSURETIMERANGE="34000 12000000" ./run_motion_1280x960_gui.sh
```

If all short-exposure modes are too dark, the correct fix is stronger, more
even lighting on the tag and base area. Do not restore GUI/JSON hold to hide the
misses; the sampler needs current-frame tag position.

On the current Jetson Xavier NX image, `exposuretimerange` values starting at
`13000` are reported in the sensor range but rejected by `nvarguscamerasrc`.
Use `34000` or higher as the lower bound.

## Calibration Caveat

The JSON calibration is OpenCV fisheye, but NVIDIA `nvAprilTags` accepts pinhole `fx/fy/cx/cy` only. This program passes the calibrated matrix and does not undistort the image. For robot closed-loop use, verify measured XYZ against known tag distances and consider adding an undistort step if edge-of-frame pose error is too large.
