# Wrench Combined YOLOv8n - 2026-06-26

Trained from the static autolabel set plus the small robot-motion autolabel set,
using the flight-augmented copies of both datasets.

## Training

- Base model: `yolov8n.pt`
- Image size: `640`
- Epochs: `60`
- Device: CPU
- Train images: `310`
- Validation images: `36`

Final validation metrics from `results.csv`:

- Precision: `0.9985`
- Recall: `1.0000`
- mAP50: `0.9950`

The validation set is still pseudo-labeled and small, so treat these metrics as
a regression check, not field proof.

## Files

- `wrench_combined_yolov8n_640_e60_best.pt`: best PyTorch checkpoint.
- `wrench_combined_yolov8n_640_e60_best_320.onnx`: static ONNX export for the current 320-style Jetson TensorRT path.
- `wrench_combined_yolov8n_640_e60_best_640.onnx`: static ONNX export at the training resolution.
- `results.csv`, `results.png`, `BoxPR_curve.png`: training records.

Local smoke test on the current camera frame:

- old live TensorRT engine: about `0.85` confidence
- this new checkpoint at 640: `0.9366` confidence

Jetson deployment status:

- ONNX copied to `/home/nvidia/vision_starter/models/wrench_combined_20260626_320.onnx`.
- TensorRT 7 FP16 engine built as `/home/nvidia/vision_starter/models/wrench_combined_20260626_320_trt7_fp16.engine`.
- Live detector on port `8090` was switched to the new engine.
- 50-frame live check after switch: `50/50` valid, average confidence `0.8998`, average FPS `29.2`.

## Jetson live check on 2026-06-27

Current migrated-board runtime check at `192.168.1.80:8090`:

- Jetson stack: Ubuntu 18.04 / JetPack 4.4.1 / TensorRT 7 / Python 3.6.
- Service script must stay Python 3.6 compatible; do not require `http.server.ThreadingHTTPServer`.
- Live 80-frame sample after enabling `target_smoothed`: `80/80` valid.
- Average confidence: `0.7698` in the current close-range scene.
- Average reported inference FPS: `22.36` with HTTP sampling load; previous 50-frame service check was about `29.2 FPS`.
- Depth JSON remained fresh: average depth age about `0.072 s`.
- Raw center jitter improved with EMA smoothing: dx stdev `2.83 px -> 1.29 px`, dy stdev `0.55 px -> 0.27 px`.

Deployment note: keep the 320 ONNX as the portable handoff and rebuild the TensorRT engine on the actual Jetson. TensorRT engines from JetPack 5 / TensorRT 8 are not portable to this JetPack 4 / TensorRT 7 board.
