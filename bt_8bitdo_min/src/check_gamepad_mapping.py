#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check whether a one-shot gamepad log covers the controls used by the arm."""

from __future__ import print_function

import argparse
import json
from pathlib import Path

from evdev_gamepad import load_config, package_root


LEGACY_BUTTON_LABELS = {
    "a": "quit",
    "b": "record",
    "x": "safe_scan",
    "y": "sensor_frame",
    "lb": "tooling_close",
    "rb": "tooling_open",
}


def _as_list(value):
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    return [value]


def button_events(summary, button_name):
    item = summary.get("buttons", {}).get(str(button_name), {})
    return int(item.get("events", 0) or 0), int(item.get("press_count", 0) or 0)


def axis_events(summary, axis_name):
    item = summary.get("axes", {}).get(str(axis_name), {})
    return int(item.get("events", 0) or 0), item


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(package_root() / "config" / "gamepad_8bitdo_bt.json"),
    )
    parser.add_argument(
        "--json",
        default=str(package_root() / "logs" / "gamepad_once.json"),
    )
    args = parser.parse_args(argv)

    config = load_config(args.config)
    with Path(args.json).open("r", encoding="utf-8") as json_file:
        summary = json.load(json_file)

    legacy = config.get("legacy_control", {})
    legacy_axes = legacy.get("axes", {})
    legacy_buttons = legacy.get("buttons", {})

    print("Device: %s (%s)" % (
        summary.get("device_meta", {}).get("name", "unknown"),
        summary.get("device_path", "unknown"),
    ))
    print("")
    print("Motion axes:")
    axis_ok = True
    for logical_name in ("x", "y", "z"):
        axis_item = legacy_axes.get(logical_name, {})
        source = axis_item.get("source", logical_name) if isinstance(axis_item, dict) else axis_item
        events, stat = axis_events(summary, source)
        observed_min = stat.get("observed_min", "")
        observed_max = stat.get("observed_max", "")
        ok = events > 0
        axis_ok = axis_ok and ok
        print("  %-2s <- %-8s events=%-4d range=%s..%s %s" % (
            logical_name,
            source,
            events,
            observed_min,
            observed_max,
            "OK" if ok else "MISSING",
        ))

    print("")
    print("Action buttons:")
    button_ok = True
    for logical_name in ("a", "b", "x", "y", "lb", "rb"):
        sources = _as_list(legacy_buttons.get(logical_name, logical_name))
        counts = [button_events(summary, source) for source in sources]
        events = sum(item[0] for item in counts)
        presses = sum(item[1] for item in counts)
        ok = presses > 0
        button_ok = button_ok and ok
        print("  %-2s %-14s <- %-24s press=%-3d events=%-3d %s" % (
            logical_name,
            LEGACY_BUTTON_LABELS.get(logical_name, ""),
            ",".join(str(source) for source in sources),
            presses,
            events,
            "OK" if ok else "MISSING",
        ))

    unmapped_buttons = summary.get("unmapped_buttons", {})
    if unmapped_buttons:
        print("")
        print("Unmapped button events:")
        for code in sorted(unmapped_buttons.keys(), key=lambda item: int(item)):
            stat = unmapped_buttons[code]
            print("  code=%s %-22s press=%s events=%s" % (
                stat.get("code", code),
                stat.get("linux", ""),
                stat.get("press_count", 0),
                stat.get("events", 0),
            ))

    print("")
    if axis_ok and button_ok:
        print("Result: mapping capture is complete for real-machine control.")
        return 0
    if axis_ok:
        print("Result: motion axes are covered, but some action buttons were not captured.")
        return 1
    print("Result: motion mapping is incomplete.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
