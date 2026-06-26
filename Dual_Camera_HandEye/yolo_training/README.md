# Wrench Flight Retraining

This folder is for retraining the wrench detector for in-flight use.

## Why retrain

The current model is usable for lab preview, but flight adds:

- motion blur from vehicle movement and vibration
- rolling exposure changes
- JPEG/MJPG compression artifacts
- small or partially cropped wrench boxes
- background clutter and false positives
- different lighting and shadows

## Data to collect

Use the same camera path as deployment. Save videos or frames from:

- static wrench, different backgrounds
- hand-held wrench moving quickly through the frame
- wrench far away and close to camera
- strong/weak light, backlight, shadows
- drone/arm vibration if possible
- negative scenes with no wrench

Label only the visible wrench area. Keep hard negatives in the dataset with
empty label files.

Expected dataset layout:

```text
dataset/
  data.yaml
  images/train/*.jpg
  labels/train/*.txt
  images/val/*.jpg
  labels/val/*.txt
```

## Build flight-augmented dataset

```bash
python Dual_Camera_HandEye/yolo_training/augment_wrench_flight_dataset.py \
  --src /path/to/wrench_dataset \
  --dst /path/to/wrench_dataset_flight_aug \
  --train-variants 4 \
  --val-variants 1
```

The offline augmentation preserves labels and adds:

- directional motion blur
- Gaussian blur
- gamma/exposure shifts
- contrast shifts
- sensor noise
- JPEG artifacts
- low-resolution resampling

## Train

```bash
python Dual_Camera_HandEye/yolo_training/train_wrench_flight_aug.py \
  --data /path/to/wrench_dataset_flight_aug/data.yaml \
  --model yolov8n.pt \
  --imgsz 640 \
  --epochs 180 \
  --batch 16
```

For Jetson TensorRT 7 deployment, export ONNX first on the training machine,
then rebuild the TensorRT engine on the Jetson that will run it.

## Evaluation target

Do not judge only by mAP. Also run a field validation clip and check:

- confidence stability over time
- false positives in no-wrench frames
- detection under motion blur
- box center jitter
- FPS after engine rebuild

For grasping/following, center jitter matters as much as raw confidence.
