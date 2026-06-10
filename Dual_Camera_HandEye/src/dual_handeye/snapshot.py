from __future__ import annotations

import json
from pathlib import Path
from typing import Any

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
