import os

from evdev_gamepad import BluetoothGamepadReader, package_root


class LinuxJoystickReader(object):
    """Compatibility wrapper used by the Python 3.6 arm controller.

    The standalone controller was originally written for /dev/input/js0.
    In this package it reads the 8BitDo Bluetooth pad through evdev instead,
    while keeping the same read() return shape.
    """

    def __init__(self, device="/dev/input/js0", threshold=0.55):
        self.device = device
        self.threshold = float(threshold)
        root = package_root()
        config_path = os.environ.get(
            "BT_GAMEPAD_CONFIG",
            str(root / "config" / "gamepad_8bitdo_bt.json"),
        )
        device_path = os.environ.get("BT_GAMEPAD_DEVICE", "")
        if not device_path and str(device).startswith("/dev/input/event"):
            device_path = str(device)

        self.reader = BluetoothGamepadReader(
            config_path=config_path,
            device_path=device_path or None,
            announce=True,
        )
        if not self.reader.is_available():
            raise RuntimeError(self.reader.last_error or "Bluetooth gamepad not available")

    def close(self):
        self.reader.close()

    def is_available(self):
        return self.reader.is_available()

    def read(self):
        return self.reader.read()
