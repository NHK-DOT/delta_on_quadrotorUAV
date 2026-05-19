#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Small write-motion serial smoke test for the servo driver board."""

from __future__ import print_function

import argparse
import sys
import time

from config import BAUDRATE, DEFAULT_PORT, SERVO_MAPPINGS
from servo_driver import BusServoDriver, serial_permission_hint


def parse_ids(text):
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def parse_targets(text):
    targets = {}
    if not text:
        return targets
    for item in str(text).split(","):
        if not item.strip():
            continue
        if ":" not in item:
            raise ValueError("target must use servo_id:position, got %r" % item)
        servo_id, position = item.split(":", 1)
        targets[int(servo_id.strip())] = int(position.strip())
    return targets


def servo_limits(servo_id):
    config = SERVO_MAPPINGS.get(int(servo_id))
    if config is None:
        return 0, 1000
    low = min(int(config["raw_min"]), int(config["raw_max"]))
    high = max(int(config["raw_min"]), int(config["raw_max"]))
    return low, high


def clamp_target(servo_id, position):
    low, high = servo_limits(servo_id)
    return max(low, min(high, int(position)))


def choose_nudge_target(servo_id, current, delta):
    low, high = servo_limits(servo_id)
    if int(current) < low:
        return min(high, low + abs(int(delta)))
    if int(current) > high:
        return max(low, high - abs(int(delta)))

    preferred = int(current) + int(delta)
    if low <= preferred <= high:
        return preferred
    fallback = int(current) - int(delta)
    if low <= fallback <= high:
        return fallback
    raise RuntimeError(
        "servo%d cannot move by %d ticks inside limits [%d, %d] from %d"
        % (servo_id, delta, low, high, current)
    )


def read_positions_with_retries(driver, servo_ids, timeout, attempts):
    last_error = None
    for attempt in range(1, max(1, int(attempts)) + 1):
        try:
            return driver.read_servo_positions(servo_ids, timeout=timeout)
        except Exception as exc:
            last_error = exc
            if attempt < attempts:
                print("Position read attempt %d failed: %s; retrying..." % (attempt, exc))
                time.sleep(0.08)
    raise last_error


def packet_trace(direction, packet, note=""):
    hex_bytes = " ".join("%02X" % item for item in bytearray(packet))
    if note:
        print("%s %-18s %s" % (direction, note, hex_bytes))
    else:
        print("%s %s" % (direction, hex_bytes))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--ids", default="1,2,3")
    parser.add_argument("--delta", type=int, default=24, help="relative nudge in raw ticks")
    parser.add_argument("--time-ms", type=int, default=500, help="servo move duration")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--read-retries", type=int, default=3)
    parser.add_argument(
        "--targets",
        default="",
        help="absolute targets as servo_id:position,servo_id:position; skips relative nudge",
    )
    parser.add_argument(
        "--no-return",
        action="store_false",
        dest="return_to_start",
        help="leave servos at the target instead of returning to the start positions",
    )
    parser.add_argument("--trace", action="store_true", help="print serial TX/RX packets")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    servo_ids = parse_ids(args.ids)
    explicit_targets = parse_targets(args.targets)
    if explicit_targets:
        servo_ids = list(explicit_targets.keys())

    driver = BusServoDriver(
        port=args.port,
        baudrate=args.baudrate,
        timeout=max(args.timeout, 0.05),
        connect_delay=0.2,
        trace_hook=packet_trace if args.trace else None,
    )

    try:
        print("Opening serial %s @ %d..." % (args.port, args.baudrate))
        driver.connect()
        print("Serial opened.")

        if explicit_targets:
            start_positions = {}
            targets = {
                servo_id: clamp_target(servo_id, position)
                for servo_id, position in explicit_targets.items()
            }
            print("Writing absolute servo targets:")
        else:
            print("Reading current positions for IDs: %s" % ",".join(str(item) for item in servo_ids))
            start_positions = read_positions_with_retries(
                driver,
                servo_ids,
                timeout=args.timeout,
                attempts=args.read_retries,
            )
            targets = {
                servo_id: choose_nudge_target(servo_id, start_positions[servo_id], args.delta)
                for servo_id in servo_ids
            }
            print("Writing relative nudge targets:")

        for servo_id in servo_ids:
            start_text = "unknown"
            if servo_id in start_positions:
                start_text = str(start_positions[servo_id])
            print("  servo%d: %s -> %d" % (servo_id, start_text, targets[servo_id]))

        driver.set_servo_positions(
            [(servo_id, targets[servo_id]) for servo_id in servo_ids],
            args.time_ms,
        )
        print("Move command sent.")
        time.sleep(max(args.time_ms, 0) / 1000.0 + 0.15)

        if args.return_to_start and start_positions:
            print("Returning to start positions:")
            for servo_id in servo_ids:
                print("  servo%d: %d -> %d" % (servo_id, targets[servo_id], start_positions[servo_id]))
            driver.set_servo_positions(
                [(servo_id, start_positions[servo_id]) for servo_id in servo_ids],
                args.time_ms,
            )
            print("Return command sent.")
            time.sleep(max(args.time_ms, 0) / 1000.0 + 0.15)

        try:
            feedback = read_positions_with_retries(
                driver,
                servo_ids,
                timeout=args.timeout,
                attempts=args.read_retries,
            )
            print("Feedback after motion:")
            for servo_id in servo_ids:
                print("  servo%d = %d" % (servo_id, feedback[servo_id]))
        except Exception as exc:
            print("Feedback read skipped/failed after motion: %s" % exc)

        print("Result: serial write-motion check command completed.")
        return 0
    except Exception as exc:
        print("Result: serial write-motion check FAILED: %s" % exc)
        print(serial_permission_hint(args.port))
        return 2
    finally:
        try:
            driver.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
