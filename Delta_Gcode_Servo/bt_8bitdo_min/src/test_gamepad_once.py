#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One-shot overwriting log test for the 8BitDo Bluetooth gamepad."""

from __future__ import print_function

import argparse
import json
import select
import sys
import time
from datetime import datetime
from pathlib import Path

from evdev_gamepad import (
    EV_ABS,
    EV_KEY,
    GamepadState,
    event_code_name,
    find_device,
    load_config,
    open_event_device,
    package_root,
    parse_proc_input_devices,
    read_event,
)


def default_log_path():
    return package_root() / "logs" / "gamepad_once.log"


def default_json_path():
    return package_root() / "logs" / "gamepad_once.json"


def build_axis_stats(state):
    stats = {}
    for code, item in state.axis_by_code.items():
        name, spec = item
        value = int(state.axis_value(name))
        stats[name] = {
            "code": code,
            "linux": spec.get("linux", event_code_name(EV_ABS, code)),
            "configured_min": int(spec.get("min", 0)),
            "configured_center": int(spec.get("center", 0)),
            "configured_max": int(spec.get("max", 255)),
            "observed_min": value,
            "observed_max": value,
            "last": value,
            "events": 0,
        }
    return stats


def build_button_stats(state):
    stats = {}
    for code, item in state.button_by_code.items():
        name, spec = item
        stats[name] = {
            "code": code,
            "linux": spec.get("linux", event_code_name(EV_KEY, code)),
            "press_count": 0,
            "release_count": 0,
            "last": bool(state.button_value(name)),
            "events": 0,
        }
    return stats


def write_header(log_file, config, device_path, device_meta, duration):
    log_file.write("# 8BitDo Bluetooth gamepad one-shot log\n")
    log_file.write("# overwritten_at=%s\n" % datetime.now().isoformat(timespec="seconds"))
    log_file.write("# duration_sec=%.1f\n" % duration)
    log_file.write("# profile=%s\n" % config.get("profile", "unknown"))
    log_file.write("# device_path=%s\n" % device_path)
    log_file.write("# device_name=%s\n" % device_meta.get("name", "unknown"))
    log_file.write("# bus=%s vendor=%s product=%s\n" % (
        device_meta.get("bus", ""),
        device_meta.get("vendor", ""),
        device_meta.get("product", ""),
    ))
    log_file.write("# Move both sticks to every edge, press LT/RT fully, press every button once.\n")
    log_file.write("# RAW_EVENTS columns: elapsed_sec event_type code linux_name value mapped_name normalized\n")
    log_file.write("\n")


def print_candidates():
    print("Input devices:")
    for device in parse_proc_input_devices():
        print("  {path}: {name} bus={bus} vendor={vendor} product={product}".format(
            path=device.get("path", ""),
            name=device.get("name", ""),
            bus=device.get("bus", ""),
            vendor=device.get("vendor", ""),
            product=device.get("product", ""),
        ))


def run_test(args):
    config = load_config(args.config)
    if args.device:
        config.setdefault("device", {})["device_path"] = args.device

    try:
        device_path, device_meta = find_device(config)
    except Exception as exc:
        print("Device not found: %s" % exc)
        print_candidates()
        return 2

    log_path = Path(args.log)
    json_path = Path(args.json)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.parent.mkdir(parents=True, exist_ok=True)

    print("Using device: %s" % device_path)
    print("Log will be overwritten: %s" % log_path)
    print("JSON will be overwritten: %s" % json_path)
    print("Test for %.1f seconds. Move every axis to max travel and press every key." % args.duration)

    event_file = open_event_device(device_path)
    try:
        state = GamepadState(config, fd=event_file.fileno())
        axis_stats = build_axis_stats(state)
        button_stats = build_button_stats(state)
        start = time.monotonic()
        deadline = start + args.duration
        raw_event_count = 0

        with log_path.open("w", encoding="utf-8") as log_file:
            write_header(log_file, config, device_path, device_meta, args.duration)
            log_file.write("RAW_EVENTS\n")

            while time.monotonic() < deadline:
                timeout = max(0.0, min(0.20, deadline - time.monotonic()))
                readable, _writable, _failed = select.select([event_file], [], [], timeout)
                if not readable:
                    continue

                event = read_event(event_file)
                if event is None:
                    continue

                event_type = event["type"]
                code = event["code"]
                value = event["value"]
                mapped_name = state.update(event_type, code, value)
                normalized = ""

                if event_type == EV_ABS and mapped_name in axis_stats:
                    stat = axis_stats[mapped_name]
                    stat["observed_min"] = min(stat["observed_min"], int(value))
                    stat["observed_max"] = max(stat["observed_max"], int(value))
                    stat["last"] = int(value)
                    stat["events"] += 1
                    normalized = "%.3f" % state.normalized_axis(mapped_name)
                elif event_type == EV_KEY and mapped_name in button_stats:
                    stat = button_stats[mapped_name]
                    if int(value) == 1:
                        stat["press_count"] += 1
                    elif int(value) == 0:
                        stat["release_count"] += 1
                    stat["last"] = bool(value)
                    stat["events"] += 1

                if event_type in (EV_ABS, EV_KEY):
                    raw_event_count += 1
                    elapsed = time.monotonic() - start
                    log_file.write(
                        "%.6f type=%d code=%d name=%s value=%s mapped=%s normalized=%s\n"
                        % (
                            elapsed,
                            event_type,
                            code,
                            event_code_name(event_type, code),
                            value,
                            mapped_name or "",
                            normalized,
                        )
                    )

            summary = {
                "created_at": datetime.now().isoformat(timespec="seconds"),
                "duration_sec": args.duration,
                "device_path": device_path,
                "device_meta": device_meta,
                "raw_event_count": raw_event_count,
                "axes": axis_stats,
                "buttons": button_stats,
                "final_snapshot": state.snapshot(),
            }

            log_file.write("\nSUMMARY axes\n")
            for name in sorted(axis_stats.keys()):
                stat = axis_stats[name]
                log_file.write(
                    "%s code=%s linux=%s observed_min=%s observed_max=%s last=%s events=%s\n"
                    % (
                        name,
                        stat["code"],
                        stat["linux"],
                        stat["observed_min"],
                        stat["observed_max"],
                        stat["last"],
                        stat["events"],
                    )
                )

            log_file.write("\nSUMMARY buttons\n")
            for name in sorted(button_stats.keys()):
                stat = button_stats[name]
                log_file.write(
                    "%s code=%s linux=%s press=%s release=%s last=%s events=%s\n"
                    % (
                        name,
                        stat["code"],
                        stat["linux"],
                        stat["press_count"],
                        stat["release_count"],
                        stat["last"],
                        stat["events"],
                    )
                )

        with json_path.open("w", encoding="utf-8") as json_file:
            json.dump(summary, json_file, indent=2, sort_keys=True)
            json_file.write("\n")

        print("Done. Raw events: %d" % raw_event_count)
        print("Wrote: %s" % log_path)
        print("Wrote: %s" % json_path)
        return 0
    finally:
        event_file.close()


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(package_root() / "config" / "gamepad_8bitdo_bt.json"))
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--device", default="", help="Override /dev/input/eventX")
    parser.add_argument("--log", default=str(default_log_path()))
    parser.add_argument("--json", default=str(default_json_path()))
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    return run_test(args)


if __name__ == "__main__":
    raise SystemExit(main())
