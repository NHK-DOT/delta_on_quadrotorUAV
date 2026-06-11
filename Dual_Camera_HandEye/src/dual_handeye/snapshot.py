from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .geometry import Transform


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        payload = json.load(fh)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return payload


def select_detection(payload: dict[str, Any], tag_id: int | None = None) -> dict[str, Any]:
    detections = payload.get("detections")
    if not isinstance(detections, list) or not detections:
        raise ValueError("snapshot contains no detections")

    for detection in detections:
        if not isinstance(detection, dict):
            continue
        if tag_id is None or detection.get("id") == tag_id:
            return detection
    raise ValueError(f"snapshot contains no detection with id={tag_id}")


def detection_to_transform(detection: dict[str, Any]) -> Transform:
    position = detection.get("position_m")
    if not isinstance(position, dict):
        raise ValueError("detection has no position_m object")

    orientation = detection.get("orientation_deg")
    if not isinstance(orientation, dict):
        orientation = {}

    return Transform.from_rpy_deg(
        translation=[
            float(position.get("x", 0.0)),
            float(position.get("y", 0.0)),
            float(position.get("z", 0.0)),
        ],
        rpy_deg=[
            float(orientation.get("roll", 0.0)),
            float(orientation.get("pitch", 0.0)),
            float(orientation.get("yaw", 0.0)),
        ],
    )


def detection_transform_from_snapshot(path: Path, tag_id: int | None = None) -> tuple[Transform, dict[str, Any]]:
    payload = read_json(path)
    detection = select_detection(payload, tag_id=tag_id)
    return detection_to_transform(detection), detection


def image_error_from_snapshot(
    path: Path,
    tag_id: int | None = None,
    target_xy: tuple[float, float] = (0.0, 0.0),
) -> tuple[np.ndarray, dict[str, Any], dict[str, Any]]:
    payload = read_json(path)
    detection = select_detection(payload, tag_id=tag_id)
    normalized_xy = detection.get("normalized_xy")
    if isinstance(normalized_xy, dict):
        x_norm = float(normalized_xy.get("x", 0.0))
        y_norm = float(normalized_xy.get("y", 0.0))
    else:
        center = detection.get("center_px")
        frame = payload.get("processing_frame") or payload.get("camera")
        if not isinstance(center, dict) or not isinstance(frame, dict):
            raise ValueError("detection needs normalized_xy or center_px plus frame width/height")
        width = float(frame.get("width", 0.0))
        height = float(frame.get("height", 0.0))
        if width <= 0.0 or height <= 0.0:
            raise ValueError("frame width/height must be positive")
        x_norm = (float(center.get("x", 0.0)) - width * 0.5) / (width * 0.5)
        y_norm = (float(center.get("y", 0.0)) - height * 0.5) / (height * 0.5)
    target = np.asarray(target_xy, dtype=float)
    error = np.asarray([x_norm, y_norm], dtype=float) - target
    return error, detection, payload
