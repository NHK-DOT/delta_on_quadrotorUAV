# Wrench Detector - current manual affine GPU e15

Date: 2026-06-28

This model is the current Jetson runtime candidate for wrench detection in the
Delta-on-UAV grasping stack. It was trained on Windows with CUDA and deployed on
the Jetson as a TensorRT 7 FP16 engine.

## Training

- Base checkpoint: `wrench_combined_gpu_resume_e180/weights/best.pt`
- Added data: 100 synchronized current-view samples from Jetson `/snapshot.json`
- Manual label for the current view: pixel box `(x=281, y=0, w=359, h=400)`
- Combined dataset: `wrench_combined_current_manual_affine_20260628`
- Dataset size: `1720 train / 56 val`
- Augmentation focus: affine transforms, motion/blur-like robustness, and the
  current difficult camera/background view
- Training run: `runs_gpu/wrench_current_manual_affine_gpu_e15`
- Epochs: 15
- Windows CUDA environment: `torch 2.12.1+cu126`, RTX 4070 Laptop GPU

Final validation:

- `mAP50 = 0.995`
- `mAP50-95 = 0.971`

## Jetson Deployment

Target board:

- Jetson at `192.168.1.80`
- JetPack 4.4.1 / L4T R32
- Python 3.6
- TensorRT 7.1.3

Runtime service:

- URL: `http://192.168.1.80:8090/`
- JSON: `http://192.168.1.80:8090/latest.json`
- Health: `http://192.168.1.80:8090/healthz`
- Snapshot capture: `http://192.168.1.80:8090/snapshot.json`

TensorRT build command used on Jetson:

```bash
cd /home/nvidia/vision_starter
/usr/src/tensorrt/bin/trtexec \
  --onnx=models/wrench_current_manual_affine_gpu_e15_best_320.onnx \
  --explicitBatch --fp16 --workspace=1024 \
  --saveEngine=models/wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine
```

TensorRT benchmark:

- Mean GPU compute latency: about `4.0 ms`
- Mean host latency: about `4.05 ms`

Current service command:

```bash
cd /home/nvidia/vision_starter
if [ -f outputs/trt_yolo_server.pid ]; then kill $(cat outputs/trt_yolo_server.pid) 2>/dev/null || true; fi
nohup python3 scripts/trt_yolo_server.py \
  --engine models/wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine \
  --source /dev/video1 --width 640 --height 480 --camera-fps 30 --fourcc MJPG \
  --infer-fps 30 --display-fps 15 --capture-thread \
  --label wrench --conf 0.20 --iou 0.45 --max-detections 1 \
  --depth-json /tmp/orbbec_depth_grid.json --depth-max-age 1.0 \
  --camera-hfov-deg 67 --camera-vfov-deg 52 --target-smooth-alpha 0.35 \
  --host 0.0.0.0 --port 8090 \
  > outputs/trt_yolo_server.log 2>&1 < /dev/null &
echo $! > outputs/trt_yolo_server.pid
```

## Live Comparison

Live test on the current Jetson camera scene at `conf=0.20`:

| Engine | Valid frames | Mean confidence | Mean FPS | Box behavior |
| --- | ---: | ---: | ---: | --- |
| `wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine` | 30/30 | 0.883 | 23.56 | tight current-view wrench box |
| `wrench_combined_gpu_resume_e43_best_320_trt7_fp16.engine` | 25/30 | 0.239 | 22.55 | larger, less stable box |

The new engine is the current preferred runtime engine.

## Checksums

```text
b5c0ee4ba78b5d5f532b7ddb4288071b4bec7cb58e6805cdd8a0347978c6e0e6  wrench_current_manual_affine_gpu_e15_best_320.onnx
fd7f3d8272f10dec135466cb92a582f101b690a7e1ef6adf9543c98b4099b287  wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine
```

## Notes

- Do not train on this Jetson generation. It has Python 3.6, TensorRT 7, and no
  suitable PyTorch/Ultralytics install. Train on Windows CUDA, export ONNX, then
  build TensorRT on Jetson.
- Use `/snapshot.json` for future capture. Separate `/raw.jpg` and
  `/latest.json` requests can desynchronize frames and labels during motion.
- Mechanical-arm feedback recovered on 2026-06-28, but the current raw pose
  `{1:542, 2:564, 3:694}` is far from the old home/startup raw values
  `{1:750, 2:762, 3:758}`. Do not command large motion until the physical arm
  pose and configured home/ranges are checked on site.
