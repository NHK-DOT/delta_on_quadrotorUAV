import os
import struct


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


class LinuxJoystickReader(object):
    def __init__(self, device="/dev/input/js0", threshold=0.55):
        self.device = device
        self.threshold = float(threshold)
        self.fd = None
        self.axes = {}
        self.buttons = {}
        self.known_axes = set()
        self.known_buttons = set()
        self.open()

    def open(self):
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        self.poll()

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def is_available(self):
        return self.fd is not None

    def _normalize_axis(self, value):
        value = float(value) / 32767.0
        if value > 1.0:
            return 1.0
        if value < -1.0:
            return -1.0
        return value

    def _axis_to_dpad(self, value):
        if abs(value) < self.threshold:
            return 0.0
        return 1.0 if value > 0.0 else -1.0

    def poll(self):
        if self.fd is None:
            return
        while True:
            try:
                data = os.read(self.fd, 8)
            except BlockingIOError:
                break
            except OSError:
                self.close()
                break
            if not data or len(data) < 8:
                break
            _time_ms, value, event_type, number = struct.unpack("IhBB", data)
            clean_type = event_type & ~JS_EVENT_INIT
            if clean_type == JS_EVENT_AXIS:
                self.axes[number] = self._normalize_axis(value)
                self.known_axes.add(number)
            elif clean_type == JS_EVENT_BUTTON:
                self.buttons[number] = bool(value)
                self.known_buttons.add(number)

    def _button(self, number):
        return bool(self.buttons.get(number, False))

    def read(self):
        self.poll()

        if 6 in self.known_axes or 7 in self.known_axes:
            dpad_x = self._axis_to_dpad(self.axes.get(6, 0.0))
            # Linux joystick Y axes are usually negative when pushed up.
            dpad_y = -self._axis_to_dpad(self.axes.get(7, 0.0))
        else:
            dpad_x = self._axis_to_dpad(self.axes.get(0, 0.0))
            dpad_y = -self._axis_to_dpad(self.axes.get(1, 0.0))

        # Some controllers expose D-pad as buttons instead of axes.
        if self._button(13):
            dpad_x = -1.0
        elif self._button(14):
            dpad_x = 1.0
        if self._button(11):
            dpad_y = 1.0
        elif self._button(12):
            dpad_y = -1.0

        if dpad_x != 0.0 and dpad_y != 0.0:
            dpad_x, dpad_y = 0.0, 0.0

        if 4 in self.known_axes:
            right_y = self.axes.get(4, 0.0)
        elif 3 in self.known_axes:
            right_y = self.axes.get(3, 0.0)
        else:
            right_y = 0.0
        if abs(right_y) < self.threshold:
            right_y = 0.0

        buttons = {
            "a": self._button(0),
            "b": self._button(1),
            "x": self._button(2),
            "y": self._button(3),
            "lb": self._button(4),
            "rb": self._button(5),
        }
        return float(dpad_x), float(dpad_y), float(right_y), buttons
