# Session Progress - 2026-06-28

Target Jetson: `192.168.1.80`, user `nvidia`.

Shared GitHub repo: `NHK-DOT/delta_on_quadrotorUAV`.

## Current Runtime State

- Jetson service is running on `http://192.168.1.80:8090/`.
- Active engine:
  `/home/nvidia/vision_starter/models/wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine`
- Runtime threshold: `conf=0.20`, `iou=0.45`, `max_detections=1`.
- Endpoints:
  - `/healthz`
  - `/latest.json`
  - `/raw.jpg`
  - `/snapshot.json`

## Completed Today

- Confirmed the service was alive, but it had been left on the old stable engine
  with temporary low threshold `conf=0.05`.
- Captured 100 synchronized current-view samples through `/snapshot.json`.
- Created a manual static dataset:
  `wrench_current_static_manual_20260628`
  - 80 train / 20 val
  - manual YOLO label: `0 0.719531 0.416667 0.560937 0.833333`
  - manual pixel box: `(x=281, y=0, w=359, h=400)`
- Built combined augmented dataset:
  `wrench_combined_current_manual_affine_20260628`
  - 1720 train / 56 val
- Trained CUDA run:
  `runs_gpu/wrench_current_manual_affine_gpu_e15`
  - started from `wrench_combined_gpu_resume_e180/weights/best.pt`
  - 15 epochs
  - `mAP50=0.995`
  - `mAP50-95=0.971`
- Exported ONNX:
  - `wrench_current_manual_affine_gpu_e15_best_320.onnx`
  - `wrench_current_manual_affine_gpu_e15_best_640.onnx`
- Built TensorRT 7 FP16 engine on Jetson:
  `wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine`

## Live Test

New engine, `conf=0.20`:

- 30/30 sampled frames valid
- confidence min/mean/max: `0.882 / 0.883 / 0.884`
- FPS min/mean/max: `22.91 / 23.56 / 23.92`
- representative box: `x=277, y=0, w=362, h=406`

Old stable engine, same `conf=0.20`:

- 25/30 sampled frames valid
- confidence min/mean/max: `0.202 / 0.239 / 0.274`
- representative box: `x=127, y=0, w=512, h=422`

Decision: keep the new `wrench_current_manual_affine_gpu_e15` TensorRT engine
as the active runtime model.

## Mechanical Arm State

Servo feedback recovered with a Python 3.6-compatible raw serial probe:

- voltage: `12335 mV`
- repeated positions: `{1:542, 2:564, 3:694}`

Important difference from old preflight:

- old configured home/startup raw values: `{1:750, 2:762, 3:758}`
- old safe feedback sample: `{1:736, 2:837, 3:878}`
- current readback is much lower on servo 1 and 2

Decision: do not command motion remotely from this state. First confirm the
physical pose, controller power state, and whether the current raw values are
expected after the latest hardware move. Large jumps toward the old home range
are not appropriate without local visual confirmation.

## Version / Board Notes

- Jetson is on JetPack 4.4.1 / L4T R32 with Python 3.6 and TensorRT 7.1.3.
- The newer Delta control package uses syntax that Python 3.6 cannot import
  directly (`from __future__ import annotations` in this environment fails).
- For this board, keep training on Windows CUDA and deploy TensorRT engines to
  Jetson.
- If migrating to a newer JetPack/Ubuntu stack later, plan it as a separate
  bring-up task because camera boot behavior, TensorRT versions, Python ABI, and
  STM32MP257 integration can all shift at once.

## Next Steps

1. Confirm the physical arm pose and whether raw `{1:542,2:564,3:694}` is safe.
2. Make the Delta control code Python 3.6-compatible or run it under a newer
   Python on Jetson before using the high-level arm scripts.
3. After safe arm feedback and home agreement, redo moving capture with
   `/snapshot.json` only.
4. Add motion samples, retrain another short CUDA run, and redeploy only if the
   live TensorRT comparison beats this e15 engine.
