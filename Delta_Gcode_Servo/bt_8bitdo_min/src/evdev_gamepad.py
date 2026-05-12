#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal Linux evdev reader for the 8BitDo Bluetooth gamepad.

This module uses only the Python standard library. It is intended for old
Ubuntu 18.04 systems where pygame/SDL mappings can be unreliable.
"""

from __future__ import print_function

import json
import os
import re
import select
import struct
from pathlib import Path

try:
    import fcntl
except ImportError:  # pragma: no cover - Windows compile check only.
    fcntl = None


EV_SYN = 0
EV_KEY = 1
EV_ABS = 3
EV_MSC = 4

EVENT_STRUCT = struct.Struct("llHHi")
ABS_INFO_STRUCT = struct.Struct("iiiiii")

ABS_CODE_NAMES = {
    0: "ABS_X",
    1: "ABS_Y",
    2: "ABS_Z",
    5: "ABS_RZ",
    9: "ABS_GAS",
    10: "ABS_BRAKE",
    16: "ABS_HAT0X",
    17: "ABS_HAT0Y",
}

KEY_CODE_NAMES = {
    304: "BTN_SOUTH",
    305: "BTN_EAST",
    306: "BTN_C",
    307: "BTN_NORTH",
    308: "BTN_WEST",
    309: "BTN_Z",
    310: "BTN_TL",
    311: "BTN_TR",
    312: "BTN_TL2",
    313: "BTN_TR2",
    314: "BTN_SELECT",
    315: "BTN_START",
    316: "BTN_MODE",
    317: "BTN_THUMBL",
    318: "BTN_THUMBR",
    319: "BTN_UNKNOWN_319",
    704: "BTN_TRIGGER_HAPPY1",
    705: "BTN_TRIGGER_HAPPY2",
    706: "BTN_TRIGGER_HAPPY3",
    707: "BTN_TRIGGER_HAPPY4",
    708: "BTN_TRIGGER_HAPPY5",
    709: "BTN_TRIGGER_HAPPY6",
    710: "BTN_TRIGGER_HAPPY7",
    711: "BTN_TRIGGER_HAPPY8",
}


def package_root():
    return Path(__file__).resolve().parents[1]


def default_config_path():
    return package_root() / "config" / "gamepad_8bitdo_bt.json"


def load_config(config_path=None):
    path = Path(config_path) if config_path else default_config_path()
    with path.open("r", encoding="utf-8") as config_file:
        return json.load(config_file)


def _norm_hex(value):
    text = str(value or "").strip().lower().replace("0x", "")
    text = text.lstrip("0")
    return text or "0"


def parse_proc_input_devices(proc_path="/proc/bus/input/devices"):
    path = Path(proc_path)
    if not path.exists():
        return []

    text = path.read_text(encoding="utf-8", errors="replace")
    devices = []
    for block in text.split("\n\n"):
        if not block.strip():
            continue

        device = {"handlers": []}
        for line in block.splitlines():
            line = line.strip()
            if line.startswith("I:"):
                for item in line[2:].strip().split():
                    if "=" in item:
                        key, value = item.split("=", 1)
                        device[key.lower()] = value.lower()
            elif line.startswith("N:"):
                match = re.search(r'Name="(.*)"', line)
                if match:
                    device["name"] = match.group(1)
            elif line.startswith("H:"):
                handlers_text = line.split("Handlers=", 1)[-1]
                handlers = handlers_text.split()
                device["handlers"] = handlers
                for handler in handlers:
                    if handler.startswith("event"):
                        device["event"] = handler
                        device["path"] = "/dev/input/" + handler

        if device.get("event"):
            devices.append(device)
    return devices


def candidate_matches(config, device):
    expected = config.get("device", {})
    name_contains = str(expected.get("name_contains", "")).lower()
    if name_contains and name_contains not in str(device.get("name", "")).lower():
        return False

    for key in ("bus", "vendor", "product"):
        expected_value = expected.get(key)
        if expected_value and _norm_hex(expected_value) != _norm_hex(device.get(key)):
            return False
    return True


def list_matching_devices(config):
    return [device for device in parse_proc_input_devices() if candidate_matches(config, device)]


def find_device(config):
    configured_path = config.get("device", {}).get("device_path", "")
    if configured_path:
        configured_path = os.path.expanduser(configured_path)
        if os.path.exists(configured_path):
            return configured_path, {"path": configured_path, "source": "config"}
        raise FileNotFoundError("configured device_path does not exist: %s" % configured_path)

    candidates = list_matching_devices(config)
    existing = [device for device in candidates if os.path.exists(device.get("path", ""))]
    if existing:
        return existing[0]["path"], existing[0]

    if candidates:
        return candidates[0]["path"], candidates[0]

    raise FileNotFoundError("8BitDo Bluetooth input event device was not found")


def open_event_device(path):
    return open(path, "rb", buffering=0)


def read_event(event_file):
    data = event_file.read(EVENT_STRUCT.size)
    if not data or len(data) < EVENT_STRUCT.size:
        return None
    sec, usec, event_type, code, value = EVENT_STRUCT.unpack(data)
    return {
        "time": sec + (usec / 1000000.0),
        "type": event_type,
        "code": code,
        "value": value,
    }


_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_DIRBITS = 2
_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_READ = 2


def _ioc(direction, type_value, number, size):
    return (
        (direction << _IOC_DIRSHIFT)
        | (type_value << _IOC_TYPESHIFT)
        | (number << _IOC_NRSHIFT)
        | (size << _IOC_SIZESHIFT)
    )


def eviocgabs(abs_code):
    return _ioc(_IOC_READ, ord("E"), 0x40 + int(abs_code), ABS_INFO_STRUCT.size)


def get_abs_info(fd, abs_code):
    if fcntl is None:
        return None
    buffer = bytearray(ABS_INFO_STRUCT.size)
    try:
        fcntl.ioctl(fd, eviocgabs(abs_code), buffer, True)
    except OSError:
        return None

    value, min_value, max_value, fuzz, flat, resolution = ABS_INFO_STRUCT.unpack(bytes(buffer))
    return {
        "value": value,
        "min": min_value,
        "max": max_value,
        "fuzz": fuzz,
        "flat": flat,
        "resolution": resolution,
    }


def clamp(value, low, high):
    return max(low, min(high, value))


def event_code_name(event_type, code):
    if event_type == EV_ABS:
        return ABS_CODE_NAMES.get(code, "ABS_%s" % code)
    if event_type == EV_KEY:
        return KEY_CODE_NAMES.get(code, "KEY_%s" % code)
    if event_type == EV_MSC:
        return "MSC_%s" % code
    if event_type == EV_SYN:
        return "SYN_%s" % code
    return "TYPE_%s_CODE_%s" % (event_type, code)


class GamepadState(object):
    def __init__(self, config, fd=None):
        self.config = config
        self.axis_by_code = {}
        self.button_by_code = {}
        self.axes = {}
        self.buttons = {}

        for name, spec in config.get("axes", {}).items():
            spec_copy = dict(spec)
            code = int(spec_copy["code"])
            self.axis_by_code[code] = (name, spec_copy)
            self.axes[name] = int(spec_copy.get("center", spec_copy.get("min", 0)))

        for name, spec in config.get("buttons", {}).items():
            spec_copy = dict(spec)
            code = int(spec_copy["code"])
            self.button_by_code[code] = (name, spec_copy)
            self.buttons[name] = False

        if fd is not None:
            self.load_abs_current_values(fd)

    def load_abs_current_values(self, fd):
        for code, item in self.axis_by_code.items():
            name, spec = item
            info = get_abs_info(fd, code)
            if not info:
                continue
            self.axes[name] = int(info["value"])
            spec["min"] = int(info["min"])
            spec["max"] = int(info["max"])
            if spec.get("kind") != "trigger":
                spec["center"] = int(spec.get("center", (info["min"] + info["max"]) / 2))
            spec["deadzone"] = int(spec.get("deadzone", info.get("flat", 0)))
            spec["flat"] = int(info.get("flat", 0))
            spec["resolution"] = int(info.get("resolution", 0))

    def update(self, event_type, code, value):
        if event_type == EV_ABS and code in self.axis_by_code:
            name, _spec = self.axis_by_code[code]
            self.axes[name] = int(value)
            return name

        if event_type == EV_KEY and code in self.button_by_code:
            name, _spec = self.button_by_code[code]
            self.buttons[name] = bool(value)
            return name

        return None

    def axis_value(self, name):
        return self.axes.get(name, 0)

    def button_value(self, name):
        return bool(self.buttons.get(name, False))

    def normalized_axis(self, name):
        spec = None
        for _code, item in self.axis_by_code.items():
            axis_name, axis_spec = item
            if axis_name == name:
                spec = axis_spec
                break
        if spec is None:
            return 0.0

        value = float(self.axis_value(name))
        min_value = float(spec.get("min", 0))
        max_value = float(spec.get("max", 255))
        span = max_value - min_value
        if span == 0:
            return 0.0

        if spec.get("kind") == "trigger":
            normalized = (value - min_value) / span
        else:
            center = float(spec.get("center", (min_value + max_value) / 2.0))
            deadzone = float(spec.get("deadzone", 0))
            delta = value - center
            if abs(delta) <= deadzone:
                return 0.0
            scale = max(center - min_value, max_value - center)
            normalized = 0.0 if scale == 0 else delta / scale

        if bool(spec.get("invert", False)):
            normalized = -normalized
        return clamp(normalized, -1.0, 1.0)

    def action_buttons(self):
        actions = {}
        for action_name, button_name in self.config.get("actions", {}).items():
            actions[action_name] = self.button_value(button_name)
        return actions

    def legacy_buttons(self):
        return {
            "a": self.button_value("south"),
            "b": self.button_value("east"),
            "x": self.button_value("west"),
            "y": self.button_value("north"),
            "lb": self.button_value("lb"),
            "rb": self.button_value("rb"),
        }

    def legacy_tuple(self):
        return (
            self.normalized_axis("left_x"),
            self.normalized_axis("left_y"),
            self.normalized_axis("right_y"),
            self.legacy_buttons(),
        )

    def snapshot(self):
        return {
            "axes_raw": dict(self.axes),
            "axes_normalized": {
                name: self.normalized_axis(name)
                for name in sorted(self.axes.keys())
            },
            "buttons": dict(self.buttons),
            "actions": self.action_buttons(),
        }


class BluetoothGamepadReader(object):
    """Small reader compatible with the old controller's GamepadReader.read()."""

    def __init__(self, config_path=None, device_path=None, announce=True):
        self.config = load_config(config_path)
        if device_path:
            self.config.setdefault("device", {})["device_path"] = device_path
        self.device_path = None
        self.device_meta = {}
        self.event_file = None
        self.state = None
        self.last_error = None
        self.last_detected_count = 0
        self.right_y_axis = int(self.config.get("axes", {}).get("right_y", {}).get("code", 5))
        self.refresh(announce=announce)

    def refresh(self, announce=False):
        self.close()
        try:
            self.device_path, self.device_meta = find_device(self.config)
            self.event_file = open_event_device(self.device_path)
            self.state = GamepadState(self.config, fd=self.event_file.fileno())
            self.last_error = None
            self.last_detected_count = 1
            if announce:
                name = self.device_meta.get("name", "unknown")
                print("8BitDo Bluetooth gamepad: %s (%s)" % (name, self.device_path))
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self.event_file = None
            self.state = None
            self.last_detected_count = 0
            if announce:
                print("8BitDo Bluetooth gamepad init failed: %s" % exc)
            return False

    def is_available(self):
        return self.event_file is not None and self.state is not None

    def pump(self, timeout=0.0):
        if not self.is_available():
            return 0

        count = 0
        while True:
            readable, _writable, _failed = select.select([self.event_file], [], [], timeout)
            if not readable:
                return count
            event = read_event(self.event_file)
            if event is None:
                return count
            self.state.update(event["type"], event["code"], event["value"])
            count += 1
            timeout = 0.0

    def read(self):
        self.pump(timeout=0.0)
        if self.state is None:
            return 0.0, 0.0, 0.0, {
                "a": False,
                "b": False,
                "x": False,
                "y": False,
                "lb": False,
                "rb": False,
            }
        return self.state.legacy_tuple()

    def close(self):
        if self.event_file is not None:
            try:
                self.event_file.close()
            except Exception:
                pass
        self.event_file = None
