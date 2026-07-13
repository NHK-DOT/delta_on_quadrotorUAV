#!/usr/bin/env python3
"""One-way NX vision/arm observer sender for STM32MP257."""

from __future__ import annotations

import argparse
import json
import os
import socket
import time
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.error import URLError
from urllib.request import urlopen


PROTOCOL = "78arm.nx-arm-bridge/v1"
ARM_STATES = {"IDLE", "GRASPING", "GRASPED", "FAILED", "UNKNOWN"}


def read_file(path: Optional[Path]) -> Dict[str, Any]:
    if path is None or not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def read_url(url: str) -> Dict[str, Any]:
    try:
        with urlopen(url, timeout=1.0) as response:
            value = json.loads(response.read().decode("utf-8"))
    except (OSError, URLError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def target_from(payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    detection = payload.get("detection", payload)
    if isinstance(payload.get("detections"), list) and payload["detections"]:
        detection = payload["detections"][0]
    if not isinstance(detection, dict):
        return None
    center = detection.get("center", detection.get("center_px", {}))
    width = detection.get("image_width", payload.get("width"))
    height = detection.get("image_height", payload.get("height"))
    confidence = detection.get("conf", detection.get("confidence", 0.0))
    if not isinstance(center, dict) or not isinstance(width, (int, float)) or not isinstance(height, (int, float)):
        return None
    try:
        center_x, center_y, confidence = float(center["x"]), float(center["y"]), float(confidence)
    except (KeyError, TypeError, ValueError):
        return None
    if width <= 0 or height <= 0 or confidence <= 0.0:
        return None
    try:
        distance = max(0.0, float(detection.get("distance_m", payload.get("distance_m", 0.0)) or 0.0))
    except (TypeError, ValueError):
        distance = 0.0
    return {
        "target_id": int(detection.get("target_id", 0)),
        "offset": {"dx": center_x - width * 0.5, "dy": center_y - height * 0.5},
        "conf": confidence,
        "distance_m": distance,
        "frame_id": str(detection.get("frame_id", "camera_optical_frame")),
    }


def packet(sequence: int, vision: Dict[str, Any], arm: Dict[str, Any]) -> Dict[str, Any]:
    state = str(arm.get("state", "UNKNOWN")).upper()
    if state not in ARM_STATES:
        state = "UNKNOWN"
    target = target_from(vision)
    return {
        "protocol": PROTOCOL,
        "source": {"node": "jetson-xavier-nx", "role": "vision_arm_observer"},
        "sequence": sequence,
        "timestamp_unix": time.time(),
        "target": target,
        "arm_status": {"state": state, "detail": str(arm.get("detail", ""))[:256]},
        "health": {"vision_ok": target is not None, "arm_status_ok": bool(arm)},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mp257-host", default=os.environ.get("MP257_HOST", ""))
    parser.add_argument("--udp-port", type=int, default=int(os.environ.get("MP257_UDP_PORT", "5005")))
    parser.add_argument("--latest-url", default="http://127.0.0.1:8090/latest.json")
    parser.add_argument("--vision-json-file", type=Path)
    parser.add_argument("--arm-status-file", type=Path, default=Path("/tmp/78arm_arm_status.json"))
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not args.dry_run and not args.mp257_host:
        parser.error("--mp257-host or MP257_HOST is required unless --dry-run")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    try:
        while True:
            vision = read_file(args.vision_json_file) if args.vision_json_file else read_url(args.latest_url)
            encoded = json.dumps(packet(sequence, vision, read_file(args.arm_status_file)), separators=(",", ":"), sort_keys=True).encode("utf-8")
            if args.dry_run:
                print(encoded.decode("utf-8"))
            else:
                sock.sendto(encoded, (args.mp257_host, args.udp_port))
            sequence += 1
            if args.once:
                return 0
            time.sleep(1.0 / max(args.rate_hz, 0.1))
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
