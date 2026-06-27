# Wrench Jetson Runtime Notes

This folder contains the Jetson Xavier NX Python 3.6 TensorRT runtime used by the wrench detector in `Dual_Camera_HandEye`.

Current migrated-board service:

```text
preview: http://192.168.1.80:8090/
latest:  http://192.168.1.80:8090/latest.json
engine:  /home/nvidia/vision_starter/models/wrench_combined_20260626_320_trt7_fp16.engine
source:  /dev/video1 Orbbec DaBai RGB, 640x480 MJPG
depth:   /tmp/orbbec_depth_grid.json
```

The runtime publishes both raw and filtered target fields:

```text
target           raw YOLO detection
target_smoothed  EMA-filtered center, offset, depth, and camera-frame position
```

Use `target_smoothed` for arm follow and close-range approach planning. Keep raw `target` for debugging and model-quality checks.

Safety boundary: this runtime does not open a servo port, arm the aircraft, or close a gripper. Real motion should stay gated behind the bench safety checks, dry-run planner, and explicit manual enable.

## 320 vs 640 on this board

The 640 FP16 TensorRT engine builds successfully, but live service testing on 2026-06-27 showed about `8.7 FPS` and lower current-scene confidence than the 320 engine. Keep `wrench_combined_20260626_320_trt7_fp16.engine` as the default for moving follow/grasp work on this JetPack 4 board.
