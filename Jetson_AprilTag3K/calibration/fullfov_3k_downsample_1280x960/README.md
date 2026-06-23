# Full-FOV 3K Downsample Calibration, 1280x960

This calibration matches the current Jetson AprilTag pipeline:

```text
3264x2464 sensor mode 0 full FOV
-> downsample to 1280x960
-> AprilTag detection and pose in 1280x960 pixel coordinates
```

Use this file for the GPU detector:

```text
apriltag_fullfov_1280x960_intrinsics.json
```

Do not use these intrinsics directly for native 3264x2464 detection. If a future detector runs on the original 3K frame, recalibrate at 3K or scale this matrix carefully.

Quality from the latest offline run:

```text
raw images scanned: 100
valid samples: 32
timeouts: 68
RMS reprojection error: 0.735554 px
checkerboard internal corners: 10 cols x 7 rows
square size: 0.020 m
```

The many timeouts came from OpenCV chessboard detection on fisheye frames, not from the final AprilTag runtime. The valid samples cover center, edges, and corners well enough for the current 1280x960 full-FOV setup.
