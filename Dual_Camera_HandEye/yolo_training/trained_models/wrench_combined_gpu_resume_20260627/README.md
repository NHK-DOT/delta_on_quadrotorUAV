# Wrench Combined YOLOv8n GPU Resume - 2026-06-27

CUDA fine-tune of the 2026-06-26 combined wrench detector using the migrated
Jetson dataset. Training ran on the Windows RTX 4070 Laptop GPU, then the 320
ONNX was rebuilt into TensorRT 7 FP16 on the Jetson at `192.168.1.80`.

## Training

- Starting checkpoint: `wrench_combined_yolov8n_640_e60_best.pt`
- Dataset: `wrench_combined_autolabel_20260626_flight_aug`
- Train images: `310`
- Validation images: `36`
- Image size: `640`
- Requested epochs: `180`
- Actual epochs: `43`, stopped by early stopping
- Device: CUDA, `NVIDIA GeForce RTX 4070 Laptop GPU`
- PyTorch: `2.12.1+cu126`
- Ultralytics: `8.4.56`

Best validation metrics from `results.csv`:

- Best epoch: `3`
- Precision: `0.99838`
- Recall: `1.00000`
- mAP50: `0.99500`
- mAP50-95: `0.93136`

The previous CPU-trained checkpoint reached best mAP50-95 `0.92538` at epoch
58, so this GPU resume is a small validation improvement rather than a major
data-quality jump. The validation split is still small and pseudo-labeled, so
use real moving-arm camera checks as the stronger deployment signal.

## Files

- `wrench_combined_gpu_resume_e43_best.pt`: best CUDA-resume PyTorch checkpoint.
- `wrench_combined_gpu_resume_e43_best_320.onnx`: static ONNX export for the
  high-frame-rate Jetson TensorRT control path.
- `wrench_combined_gpu_resume_e43_best_640.onnx`: static ONNX export at the
  training resolution for offline comparison.
- `results.csv`, `results.png`, `BoxPR_curve.png`, `args.yaml`: training records.

## Jetson TensorRT Check

The 320 ONNX was copied to:

```text
/home/nvidia/vision_starter/models/wrench_combined_gpu_resume_e43_best_320.onnx
```

TensorRT 7 FP16 engine was built as:

```text
/home/nvidia/vision_starter/models/wrench_combined_gpu_resume_e43_best_320_trt7_fp16.engine
```

`trtexec` passed on Jetson with approximate 320-engine timing:

- Mean GPU compute: `4.01 ms`
- Mean host latency: `4.08 ms`
- Mean end-to-end latency: `4.20 ms`

Live service was switched to the new 320 engine on port `8090` with the same
camera/depth settings as the prior runtime.

Sample live output after switch:

- Valid detections: yes
- Confidence: about `0.87` in the current close-range scene
- Reported live FPS: about `22-27 FPS`
- Depth: fresh and OK, around `0.29-0.30 m`
- Smoothed normalized target: about `x=0.065`, `y=-0.098`

Deployment note: keep the 320 TensorRT engine as the default for this JetPack
4.4.1 / TensorRT 7 board. The earlier 640 TensorRT experiment was too slow in
the real camera pipeline despite passing TensorRT-only benchmarking.
