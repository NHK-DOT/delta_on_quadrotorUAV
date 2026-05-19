#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only serial preflight for the servo driver board."""

from __future__ import print_function

import argparse
import sys

from config import BAUDRATE, DEFAULT_PORT
from servo_driver import BusServoDriver, serial_permission_hint


def parse_ids(text):
    return [int(item.strip()) for item in str(text).split(",") if item.strip()]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--ids", default="1,2,3")
    parser.add_argument("--timeout", type=float, default=0.25)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    servo_ids = parse_ids(args.ids)
    driver = BusServoDriver(
        port=args.port,
        baudrate=args.baudrate,
        timeout=max(args.timeout, 0.05),
        connect_delay=0.2,
    )
    try:
        print("Opening serial %s @ %d..." % (args.port, args.baudrate))
        driver.connect()
        print("Serial opened.")

        print("Reading servo positions for IDs: %s" % ",".join(str(item) for item in servo_ids))
        positions = driver.read_servo_positions(servo_ids, timeout=args.timeout)
        for servo_id in servo_ids:
            print("  servo%d = %d" % (servo_id, positions[servo_id]))

        try:
            voltage_mv = driver.get_battery_voltage_mv(timeout=args.timeout)
            print("Battery voltage: %d mV" % voltage_mv)
        except Exception as exc:
            print("Battery voltage read skipped/failed: %s" % exc)

        print("Result: serial read-only check OK. No move command was sent.")
        return 0
    except Exception as exc:
        print("Result: serial read-only check FAILED: %s" % exc)
        print(serial_permission_hint(args.port))
        return 2
    finally:
        try:
            driver.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
