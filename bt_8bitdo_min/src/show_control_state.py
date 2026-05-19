#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Print normalized 8BitDo Bluetooth control state."""

from __future__ import print_function

import argparse
import sys
import time

from evdev_gamepad import BluetoothGamepadReader, package_root


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(package_root() / "config" / "gamepad_8bitdo_bt.json"))
    parser.add_argument("--device", default="", help="Override /dev/input/eventX")
    parser.add_argument("--rate", type=float, default=20.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    reader = BluetoothGamepadReader(
        config_path=args.config,
        device_path=args.device or None,
        announce=True,
    )
    if not reader.is_available():
        return 2

    interval = 1.0 / max(args.rate, 1.0)
    print("Press Ctrl+C to stop.")
    try:
        while True:
            reader.pump(timeout=interval)
            snapshot = reader.state.snapshot()
            axes = snapshot["axes_normalized"]
            ctrl_x, ctrl_y, ctrl_z, legacy_buttons = reader.state.legacy_tuple()
            actions = snapshot["actions"]
            active = [name for name, pressed in sorted(actions.items()) if pressed]
            pressed = [name for name, value in sorted(legacy_buttons.items()) if value]
            line = (
                "CTRL_X={ctrl_x:+.0f} CTRL_Y={ctrl_y:+.0f} CTRL_Z={ctrl_z:+.0f} "
                "raw LX={left_x:+.3f} LY={left_y:+.3f} "
                "RX={right_x:+.3f} RY={right_y:+.3f} "
                "LT={lt:+.3f} RT={rt:+.3f} buttons={buttons} active={active}"
            ).format(
                ctrl_x=ctrl_x,
                ctrl_y=ctrl_y,
                ctrl_z=ctrl_z,
                left_x=axes.get("left_x", 0.0),
                left_y=axes.get("left_y", 0.0),
                right_x=axes.get("right_x", 0.0),
                right_y=axes.get("right_y", 0.0),
                lt=axes.get("lt", 0.0),
                rt=axes.get("rt", 0.0),
                buttons=",".join(pressed) if pressed else "-",
                active=",".join(active) if active else "-",
            )
            print("\r" + line[:160].ljust(160), end="")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
