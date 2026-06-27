#!/usr/bin/env python3
"""Preview XY image-follow commands from the live wrench detector.

This tool does not open the servo serial port and does not command motion. It
only converts the wrench detector's normalized image error into a small proposed
XY step so the sign and gain can be checked before enabling real motion.
"""

from __future__ import print_function

import argparse
import json
import math
import time
import urllib.request


def clamp(value, low, high):
    return max(low, min(high, value))


def fetch_json(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def compute_step(target, latest, args):
    norm = target.get("normalized_xy") or {}
    if norm:
        ex = float(norm.get("x", 0.0))
        ey = float(norm.get("y", 0.0))
    else:
        offset = target.get("offset") or {}
        image = latest.get("image") or latest.get("processing_frame") or {}
        half_w = float(image.get("w", 0.0) or image.get("width", 0.0) or 0.0) / 2.0
        half_h = float(image.get("h", 0.0) or image.get("height", 0.0) or 0.0) / 2.0
        ex = float(offset.get("dx", 0.0) or 0.0) / half_w if half_w > 0 else 0.0
        ey = float(offset.get("dy", 0.0) or 0.0) / half_h if half_h > 0 else 0.0
    if abs(ex) < args.deadband:
        ex = 0.0
    if abs(ey) < args.deadband:
        ey = 0.0

    sx = -1.0 if args.invert_x else 1.0
    sy = -1.0 if args.invert_y else 1.0
    dx = sx * args.gain_mm_per_norm * ex
    dy = sy * args.gain_mm_per_norm * ey
    mag = math.sqrt(dx * dx + dy * dy)
    if mag > args.max_step_mm > 0:
        scale = args.max_step_mm / mag
        dx *= scale
        dy *= scale
    return ex, ey, dx, dy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--latest-url", default="http://127.0.0.1:8090/latest.json")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--samples", type=int, default=0, help="0 means run until Ctrl+C.")
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--gain-mm-per-norm", type=float, default=8.0)
    parser.add_argument("--max-step-mm", type=float, default=2.0)
    parser.add_argument("--min-conf", type=float, default=0.25)
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument("--invert-x", action="store_true")
    parser.add_argument("--invert-y", action="store_true")
    args = parser.parse_args()

    interval = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.2
    count = 0
    while True:
        count += 1
        now = time.time()
        try:
            latest = fetch_json(args.latest_url, timeout=2.0)
            target = latest.get("target_smoothed") or latest.get("target") or {}
            age = now - float(latest.get("timestamp_unix", latest.get("timestamp", now)))
            conf = float(target.get("conf", 0.0) or 0.0)
            target_source = "smoothed" if latest.get("target_smoothed") else "raw"
            if not latest.get("valid") or not target:
                print("NO_TARGET status=%s" % latest.get("status"))
            elif age > args.max_age_sec:
                print("STALE age=%.3fs" % age)
            elif conf < args.min_conf:
                print("LOW_CONF conf=%.3f" % conf)
            else:
                ex, ey, dx, dy = compute_step(target, latest, args)
                distance = target.get("distance_m")
                distance_text = "" if distance is None else " z=%.3fm" % float(distance)
                print(
                    "FOLLOW_PREVIEW source=%s conf=%.3f err=(%+.3f,%+.3f)%s step_xy_mm=(%+.2f,%+.2f)"
                    % (target_source, conf, ex, ey, distance_text, dx, dy)
                )
        except Exception as exc:
            print("ERROR %r" % (exc,))
        if args.samples and count >= args.samples:
            return 0
        time.sleep(interval)


if __name__ == "__main__":
    raise SystemExit(main())
