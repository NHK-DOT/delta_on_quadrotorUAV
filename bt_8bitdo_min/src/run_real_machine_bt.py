#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run the existing real-machine controller with the Bluetooth 8BitDo reader."""

from __future__ import print_function

import sys
from pathlib import Path

from evdev_gamepad import BluetoothGamepadReader


BT_ROOT = Path(__file__).resolve().parents[1]
DELTA_GCODE_SERVO_ROOT = Path(__file__).resolve().parents[2]
REAL_MACHINE_DIR = DELTA_GCODE_SERVO_ROOT / "real_machine_test"
CONFIG_PATH = BT_ROOT / "config" / "gamepad_8bitdo_bt.json"


class PatchedGamepadReader(BluetoothGamepadReader):
    def __init__(self, deadzone=0.0):
        # The raw deadzone is controlled by config/gamepad_8bitdo_bt.json.
        super(PatchedGamepadReader, self).__init__(
            config_path=str(CONFIG_PATH),
            announce=True,
        )


def main():
    sys.path.insert(0, str(REAL_MACHINE_DIR))
    try:
        import gamepad_controller as legacy_controller
    except Exception as exc:
        print("Failed to import real_machine_test/gamepad_controller.py: %s" % exc)
        print("Use the same Python environment that already runs the old controller.")
        return 2

    legacy_controller.GamepadReader = PatchedGamepadReader
    legacy_controller.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
