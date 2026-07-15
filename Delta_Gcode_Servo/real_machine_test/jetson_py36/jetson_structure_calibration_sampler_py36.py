#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only Jetson sampler for rebuilt Delta-arm structure calibration.

This script never sends any servo motion command. It records configurable
arm-servo raw feedback and optional vision metadata.

Recommended use: manually move the arm to one pose, then run one sampling
command for that label.
"""

from __future__ import print_function

import argparse
import csv
import os
import sys
import time

from jetson_workspace_common import (
    DEFAULT_APRILTAG_JSON,
    DEFAULT_APRILTAG_INTRINSICS,
    DEFAULT_CALIBRATION,
    DEFAULT_SERVO_CONFIG,
    ServoMapper,
    capture_tool_pose_from_live_opencv,
    load_tool_pose_from_apriltag,
    open_servo_driver,
    snapshot_age_ms,
    write_json,
    append_jsonl,
)


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600
DEFAULT_SERVO_TIMEOUT_SEC = 0.35
DEFAULT_FRESH_MS = 2000.0
DEFAULT_ARM_SERVO_IDS = (1, 3, 4)


def now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def finite_xyz(values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return None
    try:
        parsed = [float(values[0]), float(values[1]), float(values[2])]
    except (TypeError, ValueError):
        return None
    for value in parsed:
        if not (-1e9 < value < 1e9):
            return None
    return parsed


def append_csv(path, row):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    exists = os.path.exists(path)
    with open(path, "a", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def parse_servo_ids(text):
    parts = [part.strip() for part in str(text).split(",") if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("servo IDs must contain exactly three values")
    try:
        servo_ids = tuple(int(part) for part in parts)
    except ValueError:
        raise argparse.ArgumentTypeError("servo IDs must be integers")
    if len(set(servo_ids)) != 3:
        raise argparse.ArgumentTypeError("servo IDs must be unique")
    return servo_ids


def read_arm_raw(port, baudrate, timeout, servo_ids):
    driver = open_servo_driver(port, baudrate)
    try:
        raw = driver.read_servo_positions(servo_ids, timeout=timeout)
        return {servo_id: int(raw[servo_id]) for servo_id in servo_ids}
    finally:
        driver.close()


def parse_args(argv):
    default_output = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "structure_calibration_samples_jetson",
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True, help="Sample label, for example top_home or left_mid")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--servo-timeout", type=float, default=DEFAULT_SERVO_TIMEOUT_SEC)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--servo-ids", type=parse_servo_ids, default=DEFAULT_ARM_SERVO_IDS)
    parser.add_argument("--vision-mode", choices=["none", "snapshot", "live_cpu"], default="snapshot")
    parser.add_argument("--base-camera-snapshot", default=DEFAULT_APRILTAG_JSON)
    parser.add_argument("--apriltag-intrinsics", default=DEFAULT_APRILTAG_INTRINSICS)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION)
    parser.add_argument("--hand-tag-id", type=int, default=3)
    parser.add_argument("--fresh-ms", type=float, default=DEFAULT_FRESH_MS)
    parser.add_argument("--allow-stale-vision", action="store_true")
    parser.add_argument("--cpu-capture-width", type=int, default=1600)
    parser.add_argument("--cpu-capture-height", type=int, default=1208)
    parser.add_argument("--cpu-capture-fps", type=int, default=21)
    parser.add_argument("--cpu-capture-warmup", type=int, default=35)
    parser.add_argument("--cpu-sensor-id", type=int, default=0)
    parser.add_argument("--cpu-sensor-mode", type=int, default=0)
    parser.add_argument("--cpu-interpolation-method", type=int, default=3)
    parser.add_argument("--cpu-flip-method", type=int, default=0)
    parser.add_argument("--output-dir", default=default_output)
    parser.add_argument("--geometry-json", default="", help="Optional JSON file for geometry metadata")
    parser.add_argument("--upper-arm-mm", type=float, default=None)
    parser.add_argument("--lower-arm-mm", type=float, default=None)
    parser.add_argument("--platform-radius-mm", type=float, default=None)
    parser.add_argument("--servo-axis-radius-mm", type=float, default=None)
    parser.add_argument("--servo-axis-z-offset-mm", type=float, default=None)
    parser.add_argument("--note", default="")
    return parser.parse_args(argv)


def load_geometry_payload(args):
    payload = {}
    if args.geometry_json:
        try:
            import json

            with open(args.geometry_json, "r", encoding="utf-8") as fh:
                item = json.load(fh)
            if isinstance(item, dict):
                payload.update(item)
        except Exception:
            payload["geometry_json_error"] = "failed_to_read"
    for key, value in (
        ("upper_arm_mm", args.upper_arm_mm),
        ("lower_arm_mm", args.lower_arm_mm),
        ("platform_radius_mm", args.platform_radius_mm),
        ("servo_axis_radius_mm", args.servo_axis_radius_mm),
        ("servo_axis_z_offset_mm", args.servo_axis_z_offset_mm),
    ):
        if value is not None:
            payload[key] = float(value)
    if args.note:
        payload["note"] = args.note
    return payload


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    output_dir = os.path.abspath(args.output_dir)
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    csv_path = os.path.join(output_dir, "samples.csv")
    jsonl_path = os.path.join(output_dir, "samples.jsonl")
    latest_path = os.path.join(output_dir, "latest_sample.json")
    safe_label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in str(args.label))
    servo_ids = tuple(args.servo_ids)

    raw = read_arm_raw(args.port, args.baudrate, args.servo_timeout, servo_ids)
    mapper = ServoMapper(args.servo_config)
    raw_physical_deg = {}
    for servo_id in servo_ids:
        item = mapper.mappings[servo_id]
        logical_low = min(float(item["logical_min"]), float(item["logical_max"]))
        logical_high = max(float(item["logical_min"]), float(item["logical_max"]))
        raw_low = min(int(item["raw_min"]), int(item["raw_max"]))
        raw_high = max(int(item["raw_min"]), int(item["raw_max"]))
        value = float(raw[servo_id])
        logical = logical_low + ((value - raw_low) * (logical_high - logical_low) / max(1e-9, (raw_high - raw_low)))
        raw_physical_deg[servo_id] = logical * (240.0 / 1000.0)
    if args.vision_mode == "none":
        pose = {}
        xyz = None
    elif args.vision_mode == "snapshot":
        age_ms = snapshot_age_ms(args.base_camera_snapshot)
        if age_ms is None:
            print("AprilTag snapshot missing or has no timestamp: %s" % args.base_camera_snapshot, file=sys.stderr)
            return 2
        if age_ms > float(args.fresh_ms) and not args.allow_stale_vision:
            print(
                "AprilTag snapshot stale: %.0f ms > %.0f ms (%s)"
                % (age_ms, float(args.fresh_ms), args.base_camera_snapshot),
                file=sys.stderr,
            )
            return 2
        pose = load_tool_pose_from_apriltag(args.base_camera_snapshot, args.calibration, args.hand_tag_id)
    else:
        debug_dir = os.path.join(output_dir, "live_cpu_debug")
        debug_image_path = os.path.join(debug_dir, safe_label + "_capture.jpg")
        debug_overlay_path = os.path.join(debug_dir, safe_label + "_overlay.jpg")
        pose = capture_tool_pose_from_live_opencv(
            calibration_path=args.calibration,
            intrinsics_path=args.apriltag_intrinsics,
            hand_tag_id=args.hand_tag_id,
            stop_gpu_detector=True,
            sensor_id=args.cpu_sensor_id,
            sensor_mode=args.cpu_sensor_mode,
            sensor_width=3264,
            sensor_height=2464,
            sensor_fps=args.cpu_capture_fps,
            output_width=args.cpu_capture_width,
            output_height=args.cpu_capture_height,
            interpolation_method=args.cpu_interpolation_method,
            flip_method=args.cpu_flip_method,
            warmup_frames=args.cpu_capture_warmup,
            tag_size_m=0.0305,
            debug_image_path=debug_image_path,
            debug_overlay_path=debug_overlay_path,
        )
    if args.vision_mode != "none":
        xyz = finite_xyz(pose.get("tool_position_mm"))
    if args.vision_mode != "none" and xyz is None:
        print("tool_position_mm missing from AprilTag pose", file=sys.stderr)
        return 2

    geometry = load_geometry_payload(args)
    sample = {
        "label": str(args.label),
        "timestamp_iso": now_iso(),
        "timestamp_unix": time.time(),
        "mode": "jetson_py36_read_only_structure_calibration_sample",
        "motion_command_state": "read_only_no_motion",
        "vision_mode": args.vision_mode,
        "port": args.port,
        "hand_tag_id": args.hand_tag_id,
        "servo_ids": list(servo_ids),
        "servo_raw": {str(servo_id): int(raw[servo_id]) for servo_id in servo_ids},
        "servo_mapped_physical_deg": {
            str(servo_id): float(raw_physical_deg[servo_id]) for servo_id in servo_ids
        },
        "vision": pose,
        "geometry": geometry,
        "sources": {
            "servo_config": args.servo_config,
            "base_camera_snapshot": args.base_camera_snapshot,
            "apriltag_intrinsics": args.apriltag_intrinsics,
            "calibration": args.calibration,
        },
    }
    append_jsonl(jsonl_path, sample)

    row = {
        "label": sample["label"],
        "timestamp_iso": sample["timestamp_iso"],
        "servo_ids": "/".join(str(servo_id) for servo_id in servo_ids),
        "raw_servo_%d" % servo_ids[0]: raw[servo_ids[0]],
        "raw_servo_%d" % servo_ids[1]: raw[servo_ids[1]],
        "raw_servo_%d" % servo_ids[2]: raw[servo_ids[2]],
        "x_mm": "" if xyz is None else xyz[0],
        "y_mm": "" if xyz is None else xyz[1],
        "z_mm": "" if xyz is None else xyz[2],
        "vision_detection_id": pose.get("detection_id", ""),
        "vision_snapshot_age_ms": pose.get("snapshot_age_ms", ""),
        "upper_arm_mm": geometry.get("upper_arm_mm", ""),
        "lower_arm_mm": geometry.get("lower_arm_mm", ""),
        "platform_radius_mm": geometry.get("platform_radius_mm", ""),
        "servo_axis_radius_mm": geometry.get("servo_axis_radius_mm", ""),
        "servo_axis_z_offset_mm": geometry.get("servo_axis_z_offset_mm", ""),
        "note": geometry.get("note", ""),
    }
    append_csv(csv_path, row)
    write_json(latest_path, sample)

    print("sampled %s" % sample["label"])
    print("  raw: %s" % " ".join("%d=%d" % (servo_id, raw[servo_id]) for servo_id in servo_ids))
    if xyz is not None:
        print("  xyz mm: (%.3f, %.3f, %.3f)" % (xyz[0], xyz[1], xyz[2]))
    print("  vision mode: %s" % args.vision_mode)
    print("  snapshot age ms: %s" % pose.get("snapshot_age_ms"))
    if args.vision_mode == "live_cpu":
        print("  live capture: %s" % pose.get("debug_image_path"))
        print("  live overlay: %s" % pose.get("debug_overlay_path"))
    print("  csv: %s" % csv_path)
    print("  jsonl: %s" % jsonl_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
