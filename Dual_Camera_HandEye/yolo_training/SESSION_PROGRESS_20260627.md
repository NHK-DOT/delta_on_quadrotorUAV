# Session Progress - 2026-06-27

Target Jetson: `192.168.1.80`, user `nvidia`, password `nvidia`.

Shared GitHub repo: `NHK-DOT/delta_on_quadrotorUAV`.

## Stable Runtime State

- Jetson service is running on `http://192.168.1.80:8090/`.
- Health endpoint is OK: `/healthz`.
- Default engine restored to stable model:
  `/home/nvidia/vision_starter/models/wrench_combined_gpu_resume_e43_best_320_trt7_fp16.engine`
- Do not use the motion-affine e25 engine as default. It failed live detection at normal threshold.

## Completed Today

- Confirmed Windows `yolo8` env was CPU-only, upgraded to CUDA PyTorch:
  `torch 2.12.1+cu126`, RTX 4070 Laptop GPU works.
- Pulled Jetson YOLO data into:
  `C:\Users\allen\wrench_training_jetson_20260627`
- GPU-resumed training from yesterday's checkpoint:
  `wrench_combined_gpu_resume_e43_best`
  - best mAP50-95: `0.93136`
  - 320 ONNX exported and TensorRT 7 FP16 engine built.
  - This is the current stable default engine.
- Added `/raw.jpg` to the Jetson YOLO service so raw frames can be captured without overlay text/boxes.
- Later discovered `/raw.jpg` + `/latest.json` as separate requests can desync during motion.
- Added `/snapshot.json` to return same-frame raw JPEG base64 plus latest detection JSON.
- Updated `collect_wrench_autolabel_from_server.py` to prefer `/snapshot.json` and fall back to old endpoints.
- Snapshot smoke test succeeded:
  - `/snapshot.json` returned valid detection and JPEG bytes.
  - collector saved 3/3 synchronized samples.

## Mechanical Arm / Capture Notes

- A larger bounded motion trajectory was executed once successfully:
  - offsets around 25-30 mm in XY
  - raw movements reached roughly servo raw ranges near:
    `1=700..760`, `2=764..900`, `3=764..904`
  - arm returned near start with only a few ticks of error.
- During the later synced capture attempt, servo feedback began timing out on `/dev/ttyUSB0`.
- No process was holding `/dev/ttyUSB0`.
- `dmesg` showed USB suspend messages.
- Stop moving the arm until feedback reads recover.

## Bad / Experimental Model

Do not deploy:

`wrench_motion_affine_gpu_e25`

Details:

- Built from an augmented dataset that included the earlier motion capture.
- The earlier motion capture likely had frame/label mismatch because raw image and latest detection were fetched as separate HTTP requests.
- Training metrics on the hard motion validation set reached best mAP50-95 about `0.88083`, but live TensorRT test failed:
  - `--conf 0.20`: no valid detection in current scene.
  - `--conf 0.05`: low-confidence large false boxes around `0.07-0.13`.
- The stable engine was restored after this failed live test.

## Partially Collected Data

- Unsynced/risky dataset:
  `/home/nvidia/Desktop/78arm/Dual_Camera_HandEye/yolo_training/datasets/wrench_motion_live_20260627_182311`
  - 180 train / 40 val
  - likely frame-label desync risk during motion; do not use directly.
- Synced dataset attempt:
  `/home/nvidia/Desktop/78arm/Dual_Camera_HandEye/yolo_training/datasets/wrench_motion_synced_20260627_225534`
  - 180 train saved with `/snapshot.json`
  - val did not run because servo feedback timed out before motion function completed.
  - check quality before use; may be mostly static if arm did not move.

## Next Steps

1. Recover servo feedback on `/dev/ttyUSB0`.
   - Check power/control board.
   - Replug or power-cycle the USB servo controller if needed.
   - Confirm with repeated raw reads before moving.
2. Keep service on the stable `wrench_combined_gpu_resume_e43_best_320_trt7_fp16.engine`.
3. Use `/snapshot.json` for all future motion data collection.
4. Redo motion capture only after servo feedback is reliable.
5. Rebuild dataset from synchronized samples only.
6. Train a short CUDA run first, then deploy only if live 320 TensorRT detection beats or matches the stable engine.

## Useful Paths

Windows:

- `C:\Users\allen\wrench_training_jetson_20260627`
- `C:\Users\allen\trt_yolo_server_jetson_192_168_1_80.py`
- `C:\Users\allen\collect_wrench_autolabel_from_server.py`

Jetson:

- `/home/nvidia/vision_starter/scripts/trt_yolo_server.py`
- `/home/nvidia/vision_starter/models/`
- `/home/nvidia/Desktop/78arm/Dual_Camera_HandEye/yolo_training/`
- `/home/nvidia/Desktop/78arm/Dual_Camera_HandEye/yolo_training/datasets/`
