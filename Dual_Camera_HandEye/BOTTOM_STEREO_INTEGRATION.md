# Bottom stereo integration plan

The next hardware step is to add a bottom-facing stereo recognition camera and
merge it with the existing upper AprilTag hand-eye chain.

## Frame chain

Keep the Delta controller frame as the motion base. The current upper-camera
field data gives a coarse relationship between Delta FK positions and the
AprilTag vision frame:

```text
apriltag_vision_T_delta_fk
```

The bottom stereo path should produce detections in its own camera frame:

```text
bottom_stereo_T_object
```

After mounting the stereo camera on the end effector, calibrate:

```text
tool_T_bottom_stereo
tool_T_pickup
```

Then object position in the Delta base is:

```text
delta_base_T_object =
    delta_base_T_tool
  * tool_T_bottom_stereo
  * bottom_stereo_T_object
```

The pickup target is:

```text
delta_base_T_pickup_target =
    delta_base_T_object
  * inverse(tool_T_pickup)
```

## Practical order

1. Keep using the upper AprilTag to estimate and validate `delta_base_T_tool`.
2. Mount the bottom stereo rigidly and record its mechanical offset from the
   tool/pickup center.
3. Make the bottom stereo detector write a small JSON snapshot with timestamp,
   object ID, confidence, and either metric XYZ or a ray plus depth.
4. Run offline hand-eye checks before connecting the output to motion.
5. Start with XY-only visual following at fixed Z.
6. Add descent and pickup only after the XY sign and scale are verified.

## Current field result

The 2026-06-26 AprilTag workspace run is stored at:

```text
Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/20260626_151933/
```

Use:

```text
model_z_ge_155/rigid_alignment_report.json
```

as a coarse prior only. Its RMS residual is about 40 mm, so it should not be
used as the final precision hand-eye transform.

## Orbbec DaBai DCW2 bring-up

On the Jetson, the bottom stereo/depth camera is detected as:

```text
RGB UVC: /dev/video1
Depth SDK device: DaBai DCW2, serial CH8J945001W
```

RGB can be opened through OpenCV/V4L2. Depth uses OrbbecSDK, not the RealSense
stack. The tested SDK package is:

```text
OrbbecSDK_v1.10.5_arm64.deb
```

After installation, run the headless depth probe:

```bash
cd ~/Desktop/78arm/Dual_Camera_HandEye
bash tools/run_orbbec_depth_probe_jetson.sh
```

Expected successful output includes a device count, the DaBai DCW2 serial
number, available depth profiles, and one depth frame summary. The first tested
depth mode was `640x400@15fps`.

For an on-screen depth view on the Jetson display:

```bash
cd ~/Desktop/78arm/Dual_Camera_HandEye
export DISPLAY=:0
export XAUTHORITY=/home/nvidia/.Xauthority
bash tools/run_orbbec_depth_fast_preview_jetson.sh
```

## Current FPS split

The DaBai DCW2 depth stream is currently limited by the OrbbecSDK profiles to
`640x400@15fps`. That is enough for calibration and 3D tool localization as
long as motion is slow and each RGB detection uses the nearest timestamped
depth frame.

The previous 30 fps path is the RGB recognition path, not the depth path:

```text
/dev/video1 -> V4L2 MJPG 640x480@30fps -> TensorRT .engine
```

On 2026-06-26, `vision_starter` benchmarked the wrench TensorRT engine with:

```bash
cd ~/vision_starter
python3 scripts/benchmark_trt_camera.py \
  --engine models/wrench_320_trt7_fp16.engine \
  --source /dev/video1 \
  --width 640 --height 480 --fps 30 --fourcc MJPG --frames 120
```

Measured result:

```text
frames=120 elapsed=4.033s end_to_end_fps=29.8
capture_ms=20.157
preprocess_ms=3.264
infer_ms=8.894
postprocess_ms=1.275
```

Practical plan: run wrench detection on RGB at about 30 fps, then attach the
latest valid 15 fps depth sample around the detected wrench center for metric
XYZ. If `/dev/video1` disappears after SDK tests, reload the UVC driver:

```bash
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo
```
