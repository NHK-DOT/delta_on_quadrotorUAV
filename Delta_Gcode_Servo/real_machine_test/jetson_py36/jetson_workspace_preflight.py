#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only Jetson preflight for AprilTag workspace sampling.

This script intentionally sends no servo move commands. It checks the pieces
that must be true before the low-speed sampling controller is allowed to run.
"""

from __future__ import print_function

import argparse
import json
import os
import sys
import time
import traceback

from jetson_workspace_common import (
    DEFAULT_APRILTAG_JSON,
    DEFAULT_CALIBRATION,
    DEFAULT_GAMEPAD_CONFIG,
    DEFAULT_SERVO_CONFIG,
    ServoMapper,
    load_tool_pose_from_apriltag,
    open_gamepad,
    open_servo_driver,
    read_json,
    select_detection,
    snapshot_age_ms,
    write_json,
)


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600
DEFAULT_FRESH_MS = 2000.0
DEFAULT_HOME_TOLERANCE_TICKS = 150
DEFAULT_RAW_RANGE_MARGIN_TICKS = 30


def ok_line(name, detail):
    print("[OK]   %-22s %s" % (name, detail))


def fail_line(name, detail):
    print("[FAIL] %-22s %s" % (name, detail))


def warn_line(name, detail):
    print("[WARN] %-22s %s" % (name, detail))


def add_result(results, name, ok, detail, required=True, data=None):
    item = {
        "name": name,
        "ok": bool(ok),
        "required": bool(required),
        "detail": str(detail),
        "data": data or {},
    }
    results.append(item)
    if ok:
        ok_line(name, detail)
    elif required:
        fail_line(name, detail)
    else:
        warn_line(name, detail)
    return bool(ok)


def check_python(results):
    version = "%d.%d.%d" % (sys.version_info[0], sys.version_info[1], sys.version_info[2])
    return add_result(
        results,
        "python",
        sys.version_info[:2] >= (3, 6),
        "Python %s" % version,
        data={"version": version, "executable": sys.executable},
    )


def check_pyserial(results):
    try:
        import serial  # noqa: F401

        version = getattr(serial, "__version__", "unknown")
        return add_result(results, "pyserial", True, "serial import ok, version=%s" % version)
    except Exception as exc:
        return add_result(
            results,
            "pyserial",
            False,
            "import failed: %s; install with: python3 -m pip install pyserial" % exc,
        )


def check_file(results, name, path, required=True):
    exists = os.path.exists(path)
    detail = path if exists else "missing: %s" % path
    return add_result(results, name, exists, detail, required=required, data={"path": path})


def detection_ids(payload):
    detections = payload.get("detections")
    if not isinstance(detections, list):
        return []
    ids = []
    for detection in detections:
        if isinstance(detection, dict) and "id" in detection:
            ids.append(detection.get("id"))
    return ids


def check_apriltag(results, args):
    if args.skip_vision:
        return add_result(results, "apriltag", True, "skipped by --skip-vision", required=False)

    path = args.base_camera_snapshot
    if not check_file(results, "apriltag_json", path):
        return False

    try:
        payload = read_json(path)
    except Exception as exc:
        return add_result(results, "apriltag_json_parse", False, "invalid JSON: %s" % exc)

    ids = detection_ids(payload)
    age = snapshot_age_ms(path)
    age_detail = "unknown" if age is None else "%.0f ms" % age
    if age is None:
        fresh_ok = False
    else:
        fresh_ok = age <= float(args.fresh_ms)
    add_result(
        results,
        "apriltag_fresh",
        fresh_ok or args.allow_stale_vision,
        "age=%s, max=%.0f ms, ids=%s" % (age_detail, args.fresh_ms, ids),
        required=not args.allow_stale_vision,
        data={"age_ms": age, "ids": ids, "max_age_ms": args.fresh_ms},
    )

    try:
        detection = select_detection(payload, args.hand_tag_id)
        detail = "tag id=%s detected" % detection.get("id")
        add_result(results, "hand_tag", True, detail, data={"id": detection.get("id")})
    except Exception as exc:
        add_result(results, "hand_tag", False, "%s; detected ids=%s" % (exc, ids))
        return False

    try:
        pose = load_tool_pose_from_apriltag(path, args.calibration, args.hand_tag_id)
        xyz = pose.get("tool_position_mm")
        detail = "tool xyz mm=(%.2f, %.2f, %.2f)" % (xyz[0], xyz[1], xyz[2])
        return add_result(
            results,
            "vision_chain",
            True,
            detail,
            data={"tool_position_mm": xyz, "snapshot_age_ms": pose.get("snapshot_age_ms")},
        )
    except Exception as exc:
        if args.verbose:
            traceback.print_exc()
        return add_result(results, "vision_chain", False, "failed: %s" % exc)


def check_gamepad(results, args):
    if args.skip_gamepad:
        return add_result(results, "8bitdo", True, "skipped by --skip-gamepad", required=False)
    try:
        reader = open_gamepad(args.gamepad_config, args.gamepad_device)
        try:
            reader.pump(timeout=0.05)
            axes = {}
            buttons = {}
            if reader.state is not None:
                axes = dict(reader.state.axes)
                buttons = dict(reader.state.buttons)
            detail = "%s" % (reader.device_path,)
            return add_result(
                results,
                "8bitdo",
                True,
                detail,
                data={"device_path": reader.device_path, "axes": axes, "buttons": buttons},
            )
        finally:
            reader.close()
    except Exception as exc:
        return add_result(
            results,
            "8bitdo",
            False,
            "%s; pair/connect the 8BitDo pad and check /dev/input permissions" % exc,
        )


def check_servo(results, args):
    if args.skip_servo:
        return add_result(results, "servo_board", True, "skipped by --skip-servo", required=False)

    mapper = None
    try:
        mapper = ServoMapper(args.servo_config)
        home_raw = dict(mapper.reference_raw)
        startup_check_raw = dict(mapper.startup_check_raw)
        add_result(
            results,
            "servo_config",
            True,
            args.servo_config,
            data={"home_raw": home_raw, "startup_check_raw": startup_check_raw},
        )
    except Exception as exc:
        return add_result(results, "servo_config", False, "failed: %s" % exc)

    driver = None
    try:
        driver = open_servo_driver(args.port, args.baudrate)
        raw = driver.read_servo_positions(mapper.servo_ids, timeout=args.servo_timeout)
        raw = {servo_id: int(raw[servo_id]) for servo_id in mapper.servo_ids}
        detail = "raw 1=%d 2=%d 3=%d" % (raw[1], raw[2], raw[3])
        add_result(results, "servo_feedback", True, detail, data={"raw": raw})

        range_violations = mapper.raw_range_violations(raw, margin_ticks=args.raw_range_margin)
        if range_violations:
            detail_items = []
            for servo_id in mapper.servo_ids:
                if servo_id in range_violations:
                    item = range_violations[servo_id]
                    detail_items.append(
                        "%d=%d outside [%d,%d]"
                        % (servo_id, item["raw"], item["low"], item["high"])
                    )
            add_result(
                results,
                "servo_raw_range",
                False,
                "; ".join(detail_items),
                required=not args.skip_home_check,
                data={"violations": range_violations, "margin": args.raw_range_margin},
            )
        else:
            add_result(
                results,
                "servo_raw_range",
                True,
                "all feedback raw values are inside configured ranges +/- %d ticks" % args.raw_range_margin,
                data={"margin": args.raw_range_margin},
            )

        try:
            voltage_mv = driver.get_battery_voltage_mv(timeout=args.servo_timeout)
            add_result(results, "servo_battery", True, "%d mV" % voltage_mv, data={"battery_mv": voltage_mv})
        except Exception as exc:
            add_result(results, "servo_battery", False, "read failed: %s" % exc, required=False)

        errors = mapper.startup_check_errors(raw)
        max_abs = max(abs(int(value)) for value in errors.values())
        home_ok = max_abs <= int(args.home_tolerance)
        return add_result(
            results,
            "home_pose",
            home_ok or args.skip_home_check,
            "startup errors ticks: 1=%+d 2=%+d 3=%+d, tolerance=%d"
            % (errors[1], errors[2], errors[3], args.home_tolerance),
            required=not args.skip_home_check,
            data={"errors": errors, "tolerance": args.home_tolerance, "home_ok": home_ok},
        )
    except Exception as exc:
        if args.verbose:
            traceback.print_exc()
        return add_result(results, "servo_feedback", False, "%s on %s" % (exc, args.port))
    finally:
        if driver is not None:
            try:
                driver.close()
            except Exception:
                pass


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--servo-timeout", type=float, default=0.25)
    parser.add_argument("--base-camera-snapshot", default=DEFAULT_APRILTAG_JSON)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--gamepad-config", default=DEFAULT_GAMEPAD_CONFIG)
    parser.add_argument("--gamepad-device", default="")
    parser.add_argument("--hand-tag-id", type=int, default=None)
    parser.add_argument("--fresh-ms", type=float, default=DEFAULT_FRESH_MS)
    parser.add_argument("--home-tolerance", type=int, default=DEFAULT_HOME_TOLERANCE_TICKS)
    parser.add_argument("--raw-range-margin", type=int, default=DEFAULT_RAW_RANGE_MARGIN_TICKS)
    parser.add_argument("--allow-stale-vision", action="store_true")
    parser.add_argument("--skip-vision", action="store_true")
    parser.add_argument("--skip-gamepad", action="store_true")
    parser.add_argument("--skip-servo", action="store_true")
    parser.add_argument("--skip-home-check", action="store_true")
    parser.add_argument("--report", default="preflight_report.json")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    results = []

    print("Jetson 78arm workspace sampler preflight")
    print("time_unix: %.3f" % time.time())
    print("port: %s" % args.port)
    print("hand_tag_id: %s" % args.hand_tag_id)
    print("")

    check_python(results)
    check_pyserial(results)
    check_file(results, "calibration", args.calibration)
    check_file(results, "servo_config", args.servo_config)
    check_file(results, "gamepad_config", args.gamepad_config)
    check_apriltag(results, args)
    check_gamepad(results, args)
    check_servo(results, args)

    required_failures = [item for item in results if item["required"] and not item["ok"]]
    payload = {
        "created_unix": time.time(),
        "ok": not required_failures,
        "args": vars(args),
        "results": results,
        "required_failures": required_failures,
    }
    if args.report:
        write_json(args.report, payload)
        print("")
        print("report: %s" % os.path.abspath(args.report))

    print("")
    if required_failures:
        print("PRECHECK FAILED: %d required check(s) failed." % len(required_failures))
        return 2
    print("PRECHECK OK: all required checks passed. No move command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
