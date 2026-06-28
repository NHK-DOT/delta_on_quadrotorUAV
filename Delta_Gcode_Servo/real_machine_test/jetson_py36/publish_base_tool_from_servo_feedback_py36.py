#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish real base_T_tool from LX-225 feedback for Python 3.6 Jetson.

By default this is a read-only bridge: it reads servo feedback, converts raw
positions to Delta FK with the existing Jetson workspace mapping, and writes a
JSON transform that the hand-eye fusion process can consume as BASE_TOOL_JSON.

An optional bounded hold mode can periodically send a small feedback-corrected
raw command while publishing. Keep it disabled unless the current target raw is
known and near the actual pose.
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


def parse_target_raw(text, servo_ids):
    if not text:
        return None
    parts = [part.strip() for part in str(text).split(",") if part.strip()]
    if len(parts) != len(servo_ids):
        raise ValueError("hold target raw must contain %d comma-separated values" % len(servo_ids))
    return {servo_id: int(parts[index]) for index, servo_id in enumerate(servo_ids)}


def clamp(value, low, high):
    return max(low, min(high, value))


def maybe_send_hold(driver, raw, hold_target, tolerance_ticks, max_lead_ticks, move_ms):
    if not hold_target:
        return None
    error = {sid: int(hold_target[sid]) - int(raw[sid]) for sid in hold_target}
    max_error = max(abs(v) for v in error.values())
    if max_error > int(tolerance_ticks):
        command = {
            sid: int(hold_target[sid]) + clamp(int(error[sid]), -int(max_lead_ticks), int(max_lead_ticks))
            for sid in hold_target
        }
        mode = "lead_correction"
    else:
        command = {sid: int(hold_target[sid]) for sid in hold_target}
        mode = "target_refresh"
    driver.set_servo_positions([(sid, command[sid]) for sid in sorted(command)], int(move_ms))
    return {
        "enabled": True,
        "mode": mode,
        "target_raw": hold_target,
        "command_raw": command,
        "error_ticks": error,
        "max_abs_error_ticks": max_error,
        "tolerance_ticks": int(tolerance_ticks),
        "max_lead_ticks": int(max_lead_ticks),
    }


def build_payload(raw, angles, xyz_mm, xyz_m, mapper, hold_status=None):
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
        "hold": hold_status or {"enabled": False},
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
    parser.add_argument("--hold-target-raw", default="", help="Optional desired feedback raw as raw1,raw2,raw3.")
    parser.add_argument("--hold-refresh-sec", type=float, default=1.0)
    parser.add_argument("--hold-tolerance-ticks", type=int, default=3)
    parser.add_argument("--hold-max-lead-ticks", type=int, default=10)
    parser.add_argument("--hold-move-ms", type=int, default=900)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    mapper = ServoMapper(args.servo_config)
    servo_ids = mapper.servo_ids
    hold_target = parse_target_raw(args.hold_target_raw, servo_ids)
    interval = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.1
    last_hold = 0.0
    driver = open_servo_driver(args.port, args.baudrate)
    try:
        while True:
            try:
                raw, angles, xyz_mm, xyz_m = read_pose(driver, mapper, servo_ids, args.servo_timeout)
                hold_status = {"enabled": False}
                now = time.time()
                if hold_target and now - last_hold >= max(0.1, float(args.hold_refresh_sec)):
                    hold_status = maybe_send_hold(
                        driver,
                        raw,
                        hold_target,
                        args.hold_tolerance_ticks,
                        args.hold_max_lead_ticks,
                        args.hold_move_ms,
                    )
                    last_hold = now
                elif hold_target:
                    hold_status = {"enabled": True, "mode": "waiting_for_refresh", "target_raw": hold_target}
                payload = build_payload(raw, angles, xyz_mm, xyz_m, mapper, hold_status=hold_status)
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
