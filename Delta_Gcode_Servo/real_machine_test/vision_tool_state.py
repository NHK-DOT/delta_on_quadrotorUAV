#!/usr/bin/env python3
"""Read-only base-camera-to-tool preview helpers for the real-machine demo.

This module converts the base camera AprilTag snapshot into a Delta-arm tool
position, then runs the same IK and raw servo mapping used by the controller.
It does not open the servo serial port and does not send motion commands.
"""

from __future__ import annotations

import json
import sys
import time
import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DELTA_SERVO_ROOT = PROJECT_ROOT / "Delta_Gcode_Servo"
HANDEYE_SRC = PROJECT_ROOT / "Dual_Camera_HandEye" / "src"

for import_path in (DELTA_SERVO_ROOT, HANDEYE_SRC):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from delta_gcode_servo.config import robot_params
from delta_gcode_servo.kinematics import inverse_kinematics
from delta_gcode_servo.servo_mapping import default_mapping_config_path, load_servo_mappings_for_ids
from dual_handeye.anchor import estimate_base_T_tool_from_hand_tag, known_transform, transform_from_result
from dual_handeye.geometry import Transform, transform_from_json
from dual_handeye.snapshot import detection_transform_from_snapshot


@dataclass(frozen=True)
class VisionToolPreviewConfig:
    calibration_path: Path = PROJECT_ROOT / "Dual_Camera_HandEye" / "output" / "calibration_result.json"
    apriltag_snapshot_path: Path = PROJECT_ROOT / "AprilTag_Vision" / "myAprilTag" / "output" / "apriltag_latest.json"
    imu_snapshot_path: Path = PROJECT_ROOT / "IMU" / "wt61c_latest.json"
    output_path: Path = PROJECT_ROOT / "Delta_Gcode_Servo" / "real_machine_test" / "vision_tool_preview_latest.json"
    hand_tag_id: int | None = None
    tool_hand_tag_path: Path | None = None
    min_snapshot_fresh_ms: float | None = 750.0


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        payload = json.load(fh)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return payload


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")


def snapshot_age_ms(payload: dict[str, Any]) -> float | None:
    timestamp = payload.get("timestamp_unix")
    if not isinstance(timestamp, (int, float)):
        return None
    return max(0.0, (time.time() - float(timestamp)) * 1000.0)


def load_transform_file(path: Path) -> Transform:
    payload = read_json(path)
    if "transform" in payload and isinstance(payload["transform"], dict):
        payload = payload["transform"]
    return transform_from_json(payload)


def load_tool_hand_tag(calibration: dict[str, Any], override_path: Path | None) -> Transform:
    if override_path is not None:
        return load_transform_file(override_path)
    return known_transform(calibration, "tool_T_hand_tag")


def load_imu_summary(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        payload = read_json(path)
    except (OSError, json.JSONDecodeError, ValueError):
        return None

    angles = payload.get("angles_deg")
    gyro = payload.get("gyro_dps")
    accel = payload.get("accel_g")
    return {
        "source": str(path),
        "age_ms": snapshot_age_ms(payload),
        "angles_deg": angles if isinstance(angles, dict) else None,
        "gyro_dps": gyro if isinstance(gyro, dict) else None,
        "accel_g": accel if isinstance(accel, dict) else None,
    }


def estimate_tool_from_base_camera(config: VisionToolPreviewConfig) -> tuple[Transform, dict[str, Any]]:
    calibration = read_json(config.calibration_path)
    snapshot_payload = read_json(config.apriltag_snapshot_path)
    base_T_base_camera = transform_from_result(calibration, "base_camera")
    tool_T_hand_tag = load_tool_hand_tag(calibration, config.tool_hand_tag_path)
    base_camera_T_hand_tag, detection = detection_transform_from_snapshot(
        config.apriltag_snapshot_path,
        tag_id=config.hand_tag_id,
    )
    base_T_tool = estimate_base_T_tool_from_hand_tag(
        base_T_base_camera,
        base_camera_T_hand_tag,
        tool_T_hand_tag,
    )
    metadata = {
        "source_calibration": str(config.calibration_path),
        "source_base_camera_snapshot": str(config.apriltag_snapshot_path),
        "snapshot_age_ms": snapshot_age_ms(snapshot_payload),
        "detection_id": detection.get("id"),
        "detection": detection,
        "base_T_base_camera": base_T_base_camera.to_json(),
        "base_camera_T_hand_tag": base_camera_T_hand_tag.to_json(),
        "tool_T_hand_tag": tool_T_hand_tag.to_json(),
    }
    return base_T_tool, metadata


def default_reference_angles_rad() -> np.ndarray:
    params = robot_params()
    reference = np.asarray(params.home_position, dtype=float)
    angles_rad, ok = inverse_kinematics(
        float(reference[0]),
        float(reference[1]),
        float(reference[2]),
        params,
    )
    if not ok:
        raise RuntimeError("default home_position is not reachable")
    return angles_rad


def servo_raw_preview_for_angles(angles_rad: np.ndarray) -> dict[str, Any]:
    params = robot_params()
    servo_ids = [int(v) for v in params.servo_ids]
    servo_mappings = load_servo_mappings_for_ids(servo_ids)
    reference_angles = default_reference_angles_rad()
    servo_raw_directions = {servo_id: -1 for servo_id in servo_ids}
    servo_logical_directions = {
        servo_id: servo_raw_directions[servo_id] * (1 if servo_mappings[servo_id].logical_span >= 0.0 else -1)
        for servo_id in servo_ids
    }
    servo_units_per_degree = {
        servo_id: servo_mappings[servo_id].logical_units_per_degree(
            physical_min_deg=float(params.servo_physical_angle_min_deg),
            physical_max_deg=float(params.servo_physical_angle_max_deg),
        )
        for servo_id in servo_ids
    }
    reference_raw = {
        servo_id: servo_mappings[servo_id].reference_raw
        for servo_id in servo_ids
    }
    reference_coord = {
        servo_id: servo_mappings[servo_id].raw_to_logical(reference_raw[servo_id])
        for servo_id in servo_ids
    }

    target_raw: dict[int, int] = {}
    limit_errors: list[str] = []
    for index, servo_id in enumerate(servo_ids):
        mapping = servo_mappings[servo_id]
        delta_deg = float(np.degrees(angles_rad[index] - reference_angles[index]))
        target_coord = (
            reference_coord[servo_id]
            + servo_logical_directions[servo_id] * delta_deg * servo_units_per_degree[servo_id]
        )
        logical_low = min(mapping.logical_min, mapping.logical_max)
        logical_high = max(mapping.logical_min, mapping.logical_max)
        if target_coord < logical_low or target_coord > logical_high:
            limit_errors.append(
                f"servo{servo_id}: logical {target_coord:.3f} outside {logical_low:.3f}..{logical_high:.3f}"
            )
        target_raw[servo_id] = mapping.logical_to_raw(target_coord)

    return {
        "reference_home_position_mm": [float(v) for v in params.home_position],
        "reference_angles_deg": [float(v) for v in np.degrees(reference_angles)],
        "target_raw": {str(k): int(v) for k, v in target_raw.items()},
        "limit_errors": limit_errors,
        "raw_mapping_config": str(default_mapping_config_path()),
    }


def build_vision_tool_preview(config: VisionToolPreviewConfig) -> dict[str, Any]:
    base_T_tool, metadata = estimate_tool_from_base_camera(config)
    point_mm = base_T_tool.t * 1000.0
    params = robot_params()
    angles_rad, reachable = inverse_kinematics(
        float(point_mm[0]),
        float(point_mm[1]),
        float(point_mm[2]),
        params,
    )
    servo_preview = servo_raw_preview_for_angles(angles_rad) if reachable else None
    imu_summary = load_imu_summary(config.imu_snapshot_path)
    warnings: list[str] = []
    age_ms = metadata.get("snapshot_age_ms")
    if (
        config.min_snapshot_fresh_ms is not None
        and isinstance(age_ms, (int, float))
        and age_ms > config.min_snapshot_fresh_ms
    ):
        warnings.append(
            f"base camera snapshot is stale: {age_ms:.0f} ms > {config.min_snapshot_fresh_ms:.0f} ms"
        )
    if servo_preview is not None and servo_preview["limit_errors"]:
        warnings.extend(servo_preview["limit_errors"])

    return {
        "mode": "base_camera_tool_preview",
        "units": "m/mm/deg/raw",
        "created_unix": time.time(),
        "inputs": {
            "calibration": str(config.calibration_path),
            "base_camera_snapshot": str(config.apriltag_snapshot_path),
            "imu_snapshot": str(config.imu_snapshot_path),
            "hand_tag_id": config.hand_tag_id,
            "tool_hand_tag_override": str(config.tool_hand_tag_path) if config.tool_hand_tag_path else None,
        },
        "detection_id": metadata["detection_id"],
        "snapshot_age_ms": metadata["snapshot_age_ms"],
        "base_T_tool": base_T_tool.to_json(),
        "tool_position_mm": [float(v) for v in point_mm],
        "delta_ik": {
            "reachable": bool(reachable),
            "angles_deg": [float(v) for v in np.degrees(angles_rad)],
            "workspace_xyz_mm": [float(v) for v in point_mm],
        },
        "servo_raw_preview": servo_preview,
        "imu": imu_summary,
        "coordinate_chain": {
            "base_T_base_camera": metadata["base_T_base_camera"],
            "base_camera_T_hand_tag": metadata["base_camera_T_hand_tag"],
            "tool_T_hand_tag": metadata["tool_T_hand_tag"],
            "formula": "base_T_tool = base_T_base_camera * base_camera_T_hand_tag * inverse(tool_T_hand_tag)",
        },
        "warnings": warnings,
        "motion_command_state": "disabled_preview_only",
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview base-camera AprilTag -> base_T_tool -> Delta IK/raw mapping without servo motion."
    )
    parser.add_argument("--calibration", type=Path, default=VisionToolPreviewConfig.calibration_path)
    parser.add_argument("--base-camera-snapshot", type=Path, default=VisionToolPreviewConfig.apriltag_snapshot_path)
    parser.add_argument("--imu-snapshot", type=Path, default=VisionToolPreviewConfig.imu_snapshot_path)
    parser.add_argument("--output", type=Path, default=VisionToolPreviewConfig.output_path)
    parser.add_argument("--hand-tag-id", type=int, default=None)
    parser.add_argument("--tool-hand-tag", type=Path, default=None)
    parser.add_argument("--fresh-ms", type=float, default=VisionToolPreviewConfig.min_snapshot_fresh_ms)
    return parser.parse_args(argv)


def print_preview_summary(payload: dict[str, Any]) -> None:
    xyz = payload["tool_position_mm"]
    ik = payload["delta_ik"]
    raw_preview = payload.get("servo_raw_preview") or {}
    print(
        "base_T_tool xyz: "
        f"x={xyz[0]:+.2f} mm, y={xyz[1]:+.2f} mm, z={xyz[2]:+.2f} mm"
    )
    print(f"IK reachable: {ik['reachable']} angles_deg={['%.2f' % v for v in ik['angles_deg']]}")
    if raw_preview:
        print(f"servo raw preview: {raw_preview.get('target_raw')}")
    for warning in payload.get("warnings", []):
        print(f"warning: {warning}")
    print("motion command: disabled_preview_only")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config = VisionToolPreviewConfig(
        calibration_path=args.calibration,
        apriltag_snapshot_path=args.base_camera_snapshot,
        imu_snapshot_path=args.imu_snapshot,
        output_path=args.output,
        hand_tag_id=args.hand_tag_id,
        tool_hand_tag_path=args.tool_hand_tag,
        min_snapshot_fresh_ms=args.fresh_ms,
    )
    payload = build_vision_tool_preview(config)
    write_json(config.output_path, payload)
    print(f"Wrote preview: {config.output_path}")
    print_preview_summary(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
