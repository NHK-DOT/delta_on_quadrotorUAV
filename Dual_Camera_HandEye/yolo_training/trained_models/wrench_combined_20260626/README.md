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

Next deployment step is to copy the 320 ONNX to the Jetson and rebuild the
TensorRT 7 FP16 engine there before switching the live detector.
