# OpenCV Equalized AprilTag Producer

This is the current preferred AprilTag JSON producer on the Jetson Xavier NX at
`192.168.1.80`.

## Why This Exists

A 1280x960 frame captured from the 3K fisheye camera contained the base tag, but
raw detection returned no tags. Offline testing on the saved image showed:

- Raw OpenCV AprilTag detection: `0` detections.
- Histogram-equalized OpenCV AprilTag detection: detected `tag36h11 id=3`.
- GPU `nvAprilTags` with an equalized BGRA input still returned `0` detections.

So the workspace sampler now defaults to an OpenCV `cv2.aruco` producer with
grayscale histogram equalization.

## Start Producer

On the Jetson:

```bash
cd /home/nvidia/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
bash run_opencv_equalized_apriltag_jetson.sh
```

It writes the same JSON path used by preflight and sampler:

```text
/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json
```

The JSON includes:

- `detections[].id`
- `detections[].position_m`
- `detections[].rotation_matrix`
- `detections[].center_px`
- `detections[].corners_px`
- `detections[].size_px`

Those fields are enough for `jetson_workspace_common.py` to convert the detected
hand tag pose into tool XYZ through the existing hand-eye calibration.

## Current Verified Result

On the Jetson Xavier NX current scene:

```text
frames=160
frames_with_tags=134
last id=3
last position_m ~= (-0.0173, 0.0116, 0.2949)
```

The image is dark and noisy. If detection becomes unstable, first improve light
on the tag or try:

```bash
PREPROCESS=clahe bash run_opencv_equalized_apriltag_jetson.sh
PREPROCESS=gamma:0.60 bash run_opencv_equalized_apriltag_jetson.sh
```

## Sampler Default

`jetson_workspace_common.py` now points `DEFAULT_APRILTAG_LAUNCH` to:

```text
/home/nvidia/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36/run_opencv_equalized_apriltag_jetson.sh
```

So `run_sampler_py36_jetson.sh` will start this producer unless
`NO_AUTOSTART_APRILTAG=1` is set.
