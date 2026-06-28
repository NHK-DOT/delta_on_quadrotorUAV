#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Small feedback-closed raw-position nudge for the Jetson Python 3.6 stack.

This is a diagnostic/recovery tool, not a grasp executor. It reads real servo
feedback, commands a bounded lead over the desired raw target, and stops once
feedback is close enough. Use it only with small targets near the current pose.
"""

from __future__ import print_function

import argparse
import json
import time

from jetson_workspace_common import DEFAULT_SERVO_CONFIG, ServoMapper, forward_kinematics, open_servo_driver


SERVO_IDS = [1, 2, 3]


def clamp(value, low, high):
    return max(low, min(high, value))


def pose_from_raw(mapper, raw):
    angles = mapper.raw_to_angles(raw)
    xyz, ok = forward_kinematics(angles[0], angles[1], angles[2])
    return xyz, ok


def read_raw_retry(driver, mapper, tries, delay_sec):
    last_error = None
    for _ in range(max(1, int(tries))):
        try:
            raw = driver.read_servo_positions(SERVO_IDS, timeout=1.0)
            raw = {sid: int(raw[sid]) for sid in SERVO_IDS}
            xyz, ok = pose_from_raw(mapper, raw)
            return raw, xyz, ok
        except Exception as exc:
            last_error = repr(exc)
            time.sleep(max(0.0, float(delay_sec)))
    raise RuntimeError(last_error)


def print_row(row):
    print(json.dumps(row, ensure_ascii=False, separators=(",", ":")))


def parse_target(text):
    parts = [part.strip() for part in str(text).split(",") if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("target must be raw1,raw2,raw3")
    return {sid: int(parts[index]) for index, sid in enumerate(SERVO_IDS)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--target-raw", type=parse_target, required=True, help="Desired feedback raw as raw1,raw2,raw3.")
    parser.add_argument("--tolerance-ticks", type=int, default=3)
    parser.add_argument("--max-lead-ticks", type=int, default=10)
    parser.add_argument("--iterations", type=int, default=6)
    parser.add_argument("--move-ms", type=int, default=1200)
    parser.add_argument("--settle-sec", type=float, default=1.5)
    parser.add_argument("--read-retries", type=int, default=5)
    parser.add_argument("--read-retry-delay-sec", type=float, default=0.3)
    args = parser.parse_args()

    mapper = ServoMapper(args.servo_config)
    driver = open_servo_driver(args.port, args.baudrate)
    desired = dict(args.target_raw)
    exit_code = 1

    try:
        for iteration in range(max(1, int(args.iterations))):
            raw, xyz, ok = read_raw_retry(driver, mapper, args.read_retries, args.read_retry_delay_sec)
            error = {sid: int(desired[sid]) - int(raw[sid]) for sid in SERVO_IDS}
            max_error = max(abs(v) for v in error.values())
            print_row({
                "phase": "read",
                "iteration": iteration,
                "desired_raw": desired,
                "feedback_raw": raw,
                "error_ticks": error,
                "max_abs_error_ticks": max_error,
                "tool_xyz_mm": {"x": round(xyz[0], 3), "y": round(xyz[1], 3), "z": round(xyz[2], 3)},
                "fk_ok": bool(ok),
            })
            if max_error <= int(args.tolerance_ticks):
                exit_code = 0
                break

            command = {
                sid: int(desired[sid]) + clamp(int(error[sid]), -int(args.max_lead_ticks), int(args.max_lead_ticks))
                for sid in SERVO_IDS
            }
            print_row({
                "phase": "command",
                "iteration": iteration,
                "command_raw": command,
                "lead_over_desired_ticks": {sid: int(command[sid]) - int(desired[sid]) for sid in SERVO_IDS},
            })
            driver.set_servo_positions([(sid, command[sid]) for sid in SERVO_IDS], int(args.move_ms))
            time.sleep(max(0.0, float(args.settle_sec)))

        raw, xyz, ok = read_raw_retry(driver, mapper, args.read_retries, args.read_retry_delay_sec)
        final_error = {sid: int(desired[sid]) - int(raw[sid]) for sid in SERVO_IDS}
        final_max_error = max(abs(v) for v in final_error.values())
        passed = final_max_error <= int(args.tolerance_ticks)
        print_row({
            "phase": "final",
            "passed": bool(passed),
            "desired_raw": desired,
            "feedback_raw": raw,
            "error_ticks": final_error,
            "max_abs_error_ticks": final_max_error,
            "tool_xyz_mm": {"x": round(xyz[0], 3), "y": round(xyz[1], 3), "z": round(xyz[2], 3)},
            "fk_ok": bool(ok),
        })
        return 0 if passed else exit_code
    finally:
        driver.close()


if __name__ == "__main__":
    raise SystemExit(main())
