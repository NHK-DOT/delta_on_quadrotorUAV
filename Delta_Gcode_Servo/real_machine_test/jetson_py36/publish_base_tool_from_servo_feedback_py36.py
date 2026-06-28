#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish real base_T_tool from LX-225 feedback for Python 3.6 Jetson.

This is a read-only bridge: it reads servo feedback, converts raw positions to
Delta FK with the existing Jetson workspace mapping, and writes a JSON transform
that the hand-eye fusion process can consume as BASE_TOOL_JSON.
"""

from __future__ import print_function

import argparse
import json
import math
import os
import time

from jetson_workspace_common import DEFAULT_SERVO_CONFIG, ServoMapper, forward_kinematics, open_servo_driver


def write_json_atomic(path, payload):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    os.replace(tmp, path)


def make_transform(x_m, y_m, z_m):
    return [
        [1.0, 0.0, 0.0, x_m],
        [0.0, 1.0, 0.0, y_m],
        [0.0, 0.0, 1.0, z_m],
        [0.0, 0.0, 0.0, 1.0],
    ]


def read_pose(driver, mapper, servo_ids, timeout):
    raw = driver.read_servo_positions(servo_ids, timeout=timeout)
    raw = {servo_id: int(raw[servo_id]) for servo_id in servo_ids}
    angles = mapper.raw_to_angles(raw)
    xyz_mm, ok = forward_kinematics(angles[0], angles[1], angles[2])
    if not ok:
        raise RuntimeError("forward_kinematics failed for raw=%r" % (raw,))
    xyz_m = [float(v) / 1000.0 for v in xyz_mm]
    return raw, angles, xyz_mm, xyz_m


def build_payload(raw, angles, xyz_mm, xyz_m, mapper):
    now = time.time()
    z_min_m = 0.155
    z_max_m = 0.263
    warnings = []
    if xyz_m[2] < z_min_m or xyz_m[2] > z_max_m:
        warnings.append("tool z is outside nominal planner range %.3f..%.3f m" % (z_min_m, z_max_m))
    return {
        "valid": True,
        "mode": "servo_feedback_fk",
        "timestamp": now,
        "timestamp_unix": now,
        "units": "m",
        "raw": raw,
        "home_raw": mapper.reference_raw,
        "angles_deg": [round(math.degrees(v), 4) for v in angles],
        "tool_position_base_mm": {
            "x": round(xyz_mm[0], 3),
            "y": round(xyz_mm[1], 3),
            "z": round(xyz_mm[2], 3),
        },
        "tool_position_base_m": {
            "x": round(xyz_m[0], 6),
            "y": round(xyz_m[1], 6),
            "z": round(xyz_m[2], 6),
        },
        "matrix": make_transform(xyz_m[0], xyz_m[1], xyz_m[2]),
        "warnings": warnings,
        "note": "Translation comes from real servo feedback FK. Rotation is identity because this Delta tool is treated as non-tilting for grasp planning.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--servo-timeout", type=float, default=0.8)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--output", default="/home/nvidia/Desktop/78arm/Dual_Camera_HandEye/output/base_tool_from_servo_latest.json")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    mapper = ServoMapper(args.servo_config)
    servo_ids = mapper.servo_ids
    interval = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.1
    driver = open_servo_driver(args.port, args.baudrate)
    try:
        while True:
            try:
                raw, angles, xyz_mm, xyz_m = read_pose(driver, mapper, servo_ids, args.servo_timeout)
                payload = build_payload(raw, angles, xyz_mm, xyz_m, mapper)
            except Exception as exc:
                payload = {
                    "valid": False,
                    "mode": "servo_feedback_fk",
                    "timestamp": time.time(),
                    "timestamp_unix": time.time(),
                    "status": "error",
                    "error": repr(exc),
                }
            write_json_atomic(args.output, payload)
            print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)
            if args.once:
                return 0 if payload.get("valid") else 1
            time.sleep(interval)
    finally:
        driver.close()


if __name__ == "__main__":
    raise SystemExit(main())
