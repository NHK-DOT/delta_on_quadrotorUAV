#!/usr/bin/env python3
"""Train a wrench detector with flight-oriented augmentation settings."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True, help="YOLO data.yaml.")
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--project", default="runs/wrench_flight")
    parser.add_argument("--name", default="wrench_flight_aug")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--epochs", type=int, default=180)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default=0)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()

    model = YOLO(args.model)
    model.train(
        data=str(args.data),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=args.project,
        name=args.name,
        patience=40,
        cos_lr=True,
        close_mosaic=20,
        cache=False,
        single_cls=True,
        hsv_h=0.025,
        hsv_s=0.75,
        hsv_v=0.55,
        degrees=12.0,
        translate=0.18,
        scale=0.75,
        shear=4.0,
        perspective=0.0008,
        flipud=0.0,
        fliplr=0.35,
        mosaic=0.75,
        mixup=0.12,
        copy_paste=0.0,
        erasing=0.25,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
