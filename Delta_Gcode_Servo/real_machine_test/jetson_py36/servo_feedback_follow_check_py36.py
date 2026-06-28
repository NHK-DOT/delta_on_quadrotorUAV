#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Servo feedback/hold/follow diagnostic for the Jetson Python 3.6 stack.

Default mode is read-only. Use --step-all-ticks to command one small symmetric
step and verify that feedback follows before any workspace sampling or visual
following run.
"""

from __future__ import print_function

import argparse
import json
import time

from jetson_workspace_common import DEFAULT_SERVO_CONFIG, ServoMapper, forward_kinematics, open_servo_driver


SERVO_IDS = [1, 2, 3]


def pose_from_raw(mapper, raw):
    angles = mapper.raw_to_angles(raw)
    xyz, ok = forward_kinematics(angles[0], angles[1], angles[2])
    return xyz, ok


def read_raw(driver, mapper):
    raw = driver.read_servo_positions(SERVO_IDS, timeout=0.8)
    raw = {sid: int(raw[sid]) for sid in SERVO_IDS}
    xyz, ok = pose_from_raw(mapper, raw)
    return raw, xyz, ok


def print_sample(index, raw, xyz, ok, voltage):
    print(json.dumps({
        "index": index,
        "raw": raw,
        "tool_xyz_mm": {"x": round(xyz[0], 3), "y": round(xyz[1], 3), "z": round(xyz[2], 3)},
        "fk_ok": bool(ok),
        "battery_mv": voltage,
    }, ensure_ascii=False, separators=(",", ":")))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--interval-sec", type=float, default=0.25)
    parser.add_argument("--step-all-ticks", type=int, default=0, help="Optional one-shot symmetric raw step for follow testing.")
    parser.add_argument("--move-ms", type=int, default=900)
    parser.add_argument("--settle-sec", type=float, default=1.25)
    parser.add_argument("--max-follow-error-ticks", type=int, default=8)
    args = parser.parse_args()

    mapper = ServoMapper(args.servo_config)
    driver = open_servo_driver(args.port, args.baudrate)
    try:
        start_raw, start_xyz, start_ok = read_raw(driver, mapper)
        voltage = driver.get_battery_voltage_mv(timeout=0.6)
        print_sample("start", start_raw, start_xyz, start_ok, voltage)

        for i in range(max(0, int(args.samples))):
            raw, xyz, ok = read_raw(driver, mapper)
            voltage = driver.get_battery_voltage_mv(timeout=0.6)
            print_sample(i, raw, xyz, ok, voltage)
            time.sleep(max(0.0, float(args.interval_sec)))

        if int(args.step_all_ticks) != 0:
            target = {sid: int(start_raw[sid]) + int(args.step_all_ticks) for sid in SERVO_IDS}
            target_xyz, target_ok = pose_from_raw(mapper, target)
            print(json.dumps({
                "command": "step_all",
                "target_raw": target,
                "predicted_tool_xyz_mm": {
                    "x": round(target_xyz[0], 3),
                    "y": round(target_xyz[1], 3),
                    "z": round(target_xyz[2], 3),
                },
                "fk_ok": bool(target_ok),
            }, ensure_ascii=False, separators=(",", ":")))
            driver.set_servo_positions([(sid, target[sid]) for sid in SERVO_IDS], int(args.move_ms))
            time.sleep(max(0.0, float(args.settle_sec)))
            raw, xyz, ok = read_raw(driver, mapper)
            error = {sid: int(target[sid]) - int(raw[sid]) for sid in SERVO_IDS}
            passed = max(abs(v) for v in error.values()) <= int(args.max_follow_error_ticks)
            print(json.dumps({
                "result": "follow_check",
                "passed": bool(passed),
                "feedback_raw": raw,
                "follow_error_ticks": error,
                "tool_xyz_mm": {"x": round(xyz[0], 3), "y": round(xyz[1], 3), "z": round(xyz[2], 3)},
            }, ensure_ascii=False, separators=(",", ":")))
            return 0 if passed else 2

        return 0
    finally:
        driver.close()


if __name__ == "__main__":
    raise SystemExit(main())
