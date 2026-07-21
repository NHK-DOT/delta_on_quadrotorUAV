#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Toggle one Jetson GPIO output from an 8BitDo Bluetooth gamepad button.

This is a standalone hardware test. It only reads the gamepad event device and
drives one GPIO pin; it does not import or command the delta-arm controller.

Default wiring for a Xavier NX developer-kit 40-pin header:
  - BOARD 29: GPIO output signal, measured against GND
  - BOARD 6:  GND reference

Do not power an electromagnet directly from BOARD 29. Use BOARD 29 as the logic
input to a MOSFET/relay driver, with a shared GND.
"""

from __future__ import print_function

import argparse
import os
import sys
import time
from pathlib import Path


THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parents[2]
GAMEPAD_SRC = PROJECT_ROOT / "bt_8bitdo_min" / "src"
DEFAULT_CONFIG = PROJECT_ROOT / "bt_8bitdo_min" / "config" / "gamepad_8bitdo_bt.json"

if str(GAMEPAD_SRC) not in sys.path:
    sys.path.insert(0, str(GAMEPAD_SRC))

from evdev_gamepad import BluetoothGamepadReader, DEFAULT_LEGACY_BUTTONS  # noqa: E402


def load_gpio(dry_run):
    if dry_run:
        return None
    try:
        import Jetson.GPIO as GPIO
    except ImportError:
        print("ERROR: Jetson.GPIO is not installed. Use --dry-run to test only the gamepad.")
        raise
    return GPIO


def set_output(GPIO, pin, logical_high, active_high=True, dry_run=False):
    electrical_high = bool(logical_high) if active_high else not bool(logical_high)
    if dry_run:
        print("DRY-RUN GPIO BOARD %s -> %s" % (pin, "HIGH" if electrical_high else "LOW"))
        return electrical_high

    GPIO.output(pin, GPIO.HIGH if electrical_high else GPIO.LOW)
    return electrical_high


def resolve_button_names(state, requested_name):
    if requested_name in state.buttons:
        return [requested_name]

    button_config = state.legacy_config().get("buttons", {})
    mapped = button_config.get(requested_name, DEFAULT_LEGACY_BUTTONS.get(requested_name))
    if isinstance(mapped, (list, tuple)):
        names = [str(name) for name in mapped if str(name) in state.buttons]
        return names

    if mapped is not None and str(mapped) in state.buttons:
        return [str(mapped)]

    return []


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Press one 8BitDo button to toggle a Jetson GPIO output high/low."
    )
    parser.add_argument(
        "--pin",
        type=int,
        default=29,
        help="Jetson 40-pin header BOARD pin to drive. Default: 29.",
    )
    parser.add_argument(
        "--button",
        default="x",
        help="Button name or legacy alias from gamepad_8bitdo_bt.json. Default: x.",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Path to gamepad_8bitdo_bt.json.",
    )
    parser.add_argument(
        "--device",
        default="",
        help="Optional /dev/input/eventX path. Empty means auto-detect from config.",
    )
    parser.add_argument(
        "--active-low",
        action="store_true",
        help="Invert the electrical output, useful for active-low relay modules.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not touch GPIO; print toggles only. Use this first.",
    )
    parser.add_argument(
        "--poll-timeout",
        type=float,
        default=0.05,
        help="select() timeout while waiting for gamepad events. Default: 0.05s.",
    )
    parser.add_argument(
        "--leave-high-on-exit",
        action="store_true",
        help="Leave the pin in its current state on Ctrl+C. Default is force LOW and cleanup.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    active_high = not args.active_low

    config_path = Path(args.config).expanduser()
    if not config_path.exists():
        print("ERROR: gamepad config not found: %s" % config_path)
        return 2

    GPIO = load_gpio(args.dry_run)
    if GPIO is not None:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(args.pin, GPIO.OUT, initial=GPIO.LOW if active_high else GPIO.HIGH)

    reader = BluetoothGamepadReader(
        config_path=str(config_path),
        device_path=args.device or None,
        announce=True,
    )
    if not reader.is_available():
        print("ERROR: 8BitDo gamepad not available: %s" % reader.last_error)
        if GPIO is not None:
            GPIO.cleanup(args.pin)
        return 3

    resolved_button_names = resolve_button_names(reader.state, args.button)
    if not resolved_button_names:
        names = ", ".join(sorted(reader.state.buttons.keys()))
        print("ERROR: unknown button '%s'. Available buttons: %s" % (args.button, names))
        reader.close()
        if GPIO is not None:
            GPIO.cleanup(args.pin)
        return 4

    logical_on = False
    last_pressed = any(reader.state.button_value(name) for name in resolved_button_names)
    set_output(GPIO, args.pin, logical_on, active_high=active_high, dry_run=args.dry_run)

    print("")
    print("GPIO toggle test is running.")
    print("  pin: BOARD %s" % args.pin)
    print("  button: %s" % args.button)
    if resolved_button_names != [args.button]:
        print("  resolved button: %s" % ", ".join(resolved_button_names))
    print("  mode: %s" % ("dry-run" if args.dry_run else "Jetson.GPIO BOARD"))
    print("  output polarity: %s" % ("active-high" if active_high else "active-low"))
    print("Press the button once -> logical ON, press again -> logical OFF.")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            reader.pump(timeout=args.poll_timeout)
            pressed = any(reader.state.button_value(name) for name in resolved_button_names)
            if pressed and not last_pressed:
                logical_on = not logical_on
                electrical_high = set_output(
                    GPIO,
                    args.pin,
                    logical_on,
                    active_high=active_high,
                    dry_run=args.dry_run,
                )
                print(
                    "[%s] %s pressed -> logical %s, electrical %s"
                    % (
                        time.strftime("%H:%M:%S"),
                        args.button,
                        "ON" if logical_on else "OFF",
                        "HIGH" if electrical_high else "LOW",
                    )
                )
            last_pressed = pressed
    except KeyboardInterrupt:
        print("\nStopping GPIO toggle test.")
    finally:
        reader.close()
        if GPIO is not None:
            if not args.leave_high_on_exit:
                set_output(GPIO, args.pin, False, active_high=active_high, dry_run=False)
            GPIO.cleanup(args.pin)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
