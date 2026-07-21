#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Direct raw-position 8BitDo jog controller for the Jetson Python 3.6 stack.

This controller intentionally bypasses Delta IK/FK motion planning. Use it when
the arm geometry or link lengths have changed and the Cartesian model is not yet
ready for motion control.

Hardware role split:
- servos 1/3/4 are the three arm actuators
- servos 5/6 are the retractable landing gear
"""

from __future__ import print_function

import argparse
import os
import sys
import time

from jetson_workspace_common import (
    DEFAULT_GAMEPAD_CONFIG,
    DEFAULT_SERVO_CONFIG,
    load_servo_mapping_config,
    load_simple_toml,
    open_gamepad,
    open_servo_driver,
)


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600
DEFAULT_UPDATE_RATE_HZ = 40.0
DEFAULT_ARM_RAW_PER_SEC = 140.0
DEFAULT_ARM_DEADZONE = 0.22
DEFAULT_FEEDBACK_INTERVAL_SEC = 0.20
DEFAULT_STATUS_INTERVAL_SEC = 0.50
DEFAULT_SERVO_TIMEOUT_SEC = 0.35
DEFAULT_FEEDBACK_READ_RETRIES = 3
DEFAULT_FEEDBACK_RETRY_DELAY_SEC = 0.03

DEFAULT_ARM_SERVO_IDS = (1, 3, 4)


def clamp(value, low, high):
    return max(low, min(high, value))


def now_text():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def shaped_axis(value, deadzone):
    value = float(value)
    deadzone = max(0.0, min(0.95, float(deadzone)))
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / max(1e-6, 1.0 - deadzone)
    return scaled if value >= 0.0 else -scaled


def choose_axis(primary, backup):
    primary = float(primary)
    backup = float(backup)
    return primary if abs(primary) >= abs(backup) else backup


def quantize_raw(raw_value, step, raw_min, raw_max):
    step = max(1, int(step))
    raw_value = int(round(float(raw_value) / step) * step)
    return int(clamp(raw_value, min(raw_min, raw_max), max(raw_min, raw_max)))


def parse_arm_servo_ids(text):
    parts = [part.strip() for part in str(text).split(",") if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("arm servo IDs must contain exactly three IDs")
    try:
        servo_ids = tuple(int(part) for part in parts)
    except ValueError:
        raise argparse.ArgumentTypeError("arm servo IDs must be integers")
    if len(set(servo_ids)) != len(servo_ids):
        raise argparse.ArgumentTypeError("arm servo IDs must be unique")
    return servo_ids


def parse_arm_directions(text):
    parts = [part.strip() for part in str(text).split(",") if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("arm directions must be dir1,dir2,dir3")
    try:
        return tuple(1 if int(part) >= 0 else -1 for part in parts)
    except ValueError:
        raise argparse.ArgumentTypeError("arm directions must be integers")


def parse_arm_raw_max(text):
    parts = [part.strip() for part in str(text).split(",") if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("arm raw maximums must contain exactly three values")
    try:
        return tuple(int(part) for part in parts)
    except ValueError:
        raise argparse.ArgumentTypeError("arm raw maximums must be integers")


def load_landing_gear_profile(config_path):
    data = load_simple_toml(config_path)
    servos = data.get("servos", {})
    landing = data.get("landing_gear", {})
    if not isinstance(servos, dict) or not isinstance(landing, dict):
        return None

    servo_ids = landing.get("servo_ids", [5, 6])
    if isinstance(servo_ids, str):
        text = servo_ids.strip().strip("[]")
        try:
            servo_ids = [int(value.strip()) for value in text.split(",") if value.strip()]
        except ValueError:
            servo_ids = [5, 6]
    elif not isinstance(servo_ids, list):
        servo_ids = [5, 6]

    try:
        servo_ids = [int(value) for value in servo_ids]
    except (TypeError, ValueError):
        servo_ids = [5, 6]

    profile_steps = []
    profile_raw_mins = []
    profile_raw_maxs = []
    for servo_id in servo_ids:
        for item in servos.values():
            if not isinstance(item, dict):
                continue
            try:
                if int(item.get("id", -1)) != servo_id:
                    continue
            except (TypeError, ValueError):
                continue
            profile_steps.append(int(item.get("position_step", 5)))
            profile_raw_mins.append(int(item.get("raw_min", 0)))
            profile_raw_maxs.append(int(item.get("raw_max", 1000)))
            break

    if not servo_ids:
        return None

    return {
        "servo_ids": tuple(servo_ids),
        "raw_min": min(profile_raw_mins) if profile_raw_mins else 0,
        "raw_max": max(profile_raw_maxs) if profile_raw_maxs else 1000,
        "position_step": max(1, min(profile_steps) if profile_steps else 5),
        "down_raw": int(landing.get("down_raw", 500)),
        "up_raw": int(landing.get("up_raw", 1000)),
        "move_time_ms": max(20, int(landing.get("move_time_ms", 180))),
    }


class RawJogController(object):
    def __init__(self, args):
        self.args = args
        self.mappings = load_servo_mapping_config(args.servo_config)
        self.arm_servo_ids = tuple(args.arm_servo_ids)
        missing_ids = [servo_id for servo_id in self.arm_servo_ids if servo_id not in self.mappings]
        if missing_ids:
            raise ValueError("arm servo IDs missing from config: %s" % missing_ids)
        self.arm_directions = dict(zip(self.arm_servo_ids, args.arm_directions))
        self.arm_raw_limits = {}
        for index, servo_id in enumerate(self.arm_servo_ids):
            item = self.mappings[servo_id]
            low = min(int(item["raw_min"]), int(item["raw_max"]))
            high = max(int(item["raw_min"]), int(item["raw_max"]))
            if args.arm_raw_max is not None:
                high = min(high, int(args.arm_raw_max[index]))
            if low > high:
                raise ValueError("invalid raw limits for arm servo %d" % servo_id)
            self.arm_raw_limits[servo_id] = (low, high)
        self.landing_gear = load_landing_gear_profile(args.servo_config) if args.enable_landing_gear else None
        if self.landing_gear is not None:
            overlap = sorted(set(self.arm_servo_ids).intersection(self.landing_gear["servo_ids"]))
            if overlap:
                raise ValueError("arm servo IDs overlap landing gear IDs: %s" % overlap)
        self.driver = None
        self.gamepad = None
        self.running = True
        self.fault = None

        self.output_dir = os.path.abspath(args.output_dir)
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.status_path = os.path.join(self.output_dir, "runtime_status.log")
        self.debug_path = os.path.join(self.output_dir, "debug.log")
        self.debug_file = open(self.debug_path, "a", encoding="utf-8", buffering=1)

        self.arm_command_float = {}
        self.arm_command_raw = {}
        self.arm_feedback_raw = {}
        self.landing_feedback_raw = {}
        self.landing_command_raw = {}
        self.landing_state = "UNKNOWN"
        self.last_buttons = {}
        self.last_feedback_time = 0.0
        self.last_status_print = 0.0
        self.last_status_write = 0.0
        self.last_axes = {
            "left_x": 0.0,
            "left_y": 0.0,
            "right_y": 0.0,
            "dpad_x": 0.0,
            "dpad_y": 0.0,
            "servo1": 0.0,
            "servo2": 0.0,
            "servo3": 0.0,
        }
        self.last_landing_feedback_error = None
        self.motion_armed = False
        self.disabled_arm_servos = {}

    def log(self, message):
        try:
            self.debug_file.write("%0.6f %s %s\n" % (time.time(), now_text(), message))
        except Exception:
            pass

    def connect(self):
        print("Opening serial %s @ %d..." % (self.args.port, self.args.baudrate))
        self.driver = open_servo_driver(self.args.port, self.args.baudrate)
        print("Serial opened.")
        print("Opening 8BitDo controller...")
        self.gamepad = open_gamepad(self.args.gamepad_config, self.args.gamepad_device)
        print("8BitDo opened.")
        self.read_feedback(force=True, strict=True)
        self.disable_out_of_range_arm_servos()
        self.arm_command_raw = dict(self.arm_feedback_raw)
        self.arm_command_float = {
            servo_id: float(self.arm_feedback_raw[servo_id]) for servo_id in self.arm_servo_ids
        }
        if self.landing_gear is not None:
            servo_ids = self.landing_gear["servo_ids"]
            if not self.landing_feedback_raw:
                feedback = self.driver.read_servo_positions(servo_ids, timeout=self.args.servo_timeout)
                for servo_id in servo_ids:
                    self.landing_feedback_raw[servo_id] = int(feedback[servo_id])
            self.landing_command_raw = dict(self.landing_feedback_raw)
        self.write_status()

    def disable_out_of_range_arm_servos(self):
        for servo_id in self.arm_servo_ids:
            low, high = self.arm_raw_limits[servo_id]
            raw_value = int(self.arm_feedback_raw[servo_id])
            if raw_value < low or raw_value > high:
                self.disabled_arm_servos[servo_id] = (raw_value, low, high)

        if self.disabled_arm_servos:
            detail = ", ".join(
                "%d=%d outside [%d,%d]" % (servo_id, raw, low, high)
                for servo_id, (raw, low, high) in sorted(self.disabled_arm_servos.items())
            )
            self.log("DISABLED_OUT_OF_RANGE %s" % detail)
            print("Disabled arm axes with out-of-range feedback: %s" % detail)

    def close(self):
        if self.gamepad is not None:
            try:
                self.gamepad.close()
            except Exception:
                pass
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception:
                pass
        try:
            self.write_status()
        except Exception:
            pass
        try:
            self.debug_file.close()
        except Exception:
            pass

    def read_feedback(self, force=False, strict=False):
        now = time.time()
        if not force and now - self.last_feedback_time < float(self.args.feedback_interval_sec):
            return True
        self.last_feedback_time = now
        last_error = None
        for _attempt in range(max(1, int(self.args.feedback_read_retries))):
            try:
                feedback = self.driver.read_servo_positions(self.arm_servo_ids, timeout=self.args.servo_timeout)
                self.arm_feedback_raw = {
                    servo_id: int(feedback[servo_id]) for servo_id in self.arm_servo_ids
                }
                if self.fault and self.fault.startswith("feedback read failed:"):
                    self.fault = None
                break
            except Exception as exc:
                last_error = exc
                self.log("FEEDBACK_RETRY %s" % exc)
                time.sleep(max(0.0, float(self.args.feedback_retry_delay_sec)))
        else:
            self.fault = "feedback read failed: %s" % last_error
            self.log("FEEDBACK_FAILED %s" % last_error)
            return not strict

        if self.landing_gear is not None:
            try:
                landing_ids = self.landing_gear["servo_ids"]
                landing_feedback = self.driver.read_servo_positions(landing_ids, timeout=self.args.servo_timeout)
                self.landing_feedback_raw = {
                    servo_id: int(landing_feedback[servo_id]) for servo_id in landing_ids
                }
                self.last_landing_feedback_error = None
            except Exception as exc:
                self.last_landing_feedback_error = str(exc)
                self.log("LANDING_FEEDBACK_WARN %s" % exc)
        return True

    def current_buttons(self):
        state = self.gamepad.state
        if state is None:
            return {
                "a": False,
                "b": False,
                "x": False,
                "y": False,
                "lb": False,
                "rb": False,
                "start": False,
                "select": False,
            }
        buttons = state.legacy_buttons()
        buttons["start"] = state.button_value("start")
        buttons["select"] = state.button_value("select")
        return buttons

    def poll_axes(self):
        self.gamepad.pump(timeout=0.0)
        state = self.gamepad.state
        if state is None:
            return self.last_axes

        left_x = shaped_axis(state.normalized_axis("left_x"), self.args.arm_deadzone)
        left_y = shaped_axis(state.normalized_axis("left_y"), self.args.arm_deadzone)
        right_y = shaped_axis(state.normalized_axis("right_y"), self.args.arm_deadzone)
        dpad_x = shaped_axis(state.normalized_axis("dpad_x"), 0.02)
        dpad_y = shaped_axis(state.normalized_axis("dpad_y"), 0.02)
        if self.args.enable_dpad_backup:
            servo1_axis = choose_axis(left_x, dpad_x)
            servo2_axis = choose_axis(left_y, dpad_y)
        else:
            servo1_axis = left_x
            servo2_axis = left_y
        servo3_axis = right_y
        self.last_axes = {
            "left_x": left_x,
            "left_y": left_y,
            "right_y": right_y,
            "dpad_x": dpad_x,
            "dpad_y": dpad_y,
            "servo1": servo1_axis,
            "servo2": servo2_axis,
            "servo3": servo3_axis,
        }
        return self.last_axes

    def apply_landing_gear_state(self, target_state):
        if self.landing_gear is None:
            return False
        raw_value = self.landing_gear["up_raw"] if target_state == "UP" else self.landing_gear["down_raw"]
        quantized = quantize_raw(
            raw_value,
            self.landing_gear["position_step"],
            self.landing_gear["raw_min"],
            self.landing_gear["raw_max"],
        )
        updated = False
        for servo_id in self.landing_gear["servo_ids"]:
            if self.landing_command_raw.get(servo_id) != quantized:
                self.landing_command_raw[servo_id] = quantized
                updated = True
        self.landing_state = target_state
        if updated:
            self.log("LANDING_GEAR %s raw=%d" % (target_state, quantized))
        return updated

    def update_from_gamepad(self, dt):
        axes = self.poll_axes()
        buttons = self.current_buttons()
        previous = dict(self.last_buttons)
        self.last_buttons = dict(buttons)

        if buttons.get("start", False) or buttons.get("a", False):
            self.running = False
            self.log("QUIT_BUTTON")
            return False, False

        if buttons.get("y", False):
            self.fault = "emergency stop requested by Y"
            self.log("EMERGENCY_STOP_Y")
            self.running = False
            return False, False

        landing_changed = False
        if buttons.get("lb", False) and not previous.get("lb", False):
            landing_changed = self.apply_landing_gear_state("DOWN")
        if buttons.get("rb", False) and not previous.get("rb", False):
            landing_changed = self.apply_landing_gear_state("UP") or landing_changed

        max_motion_axis = max(abs(axes["servo1"]), abs(axes["servo2"]), abs(axes["servo3"]))
        if not self.motion_armed:
            if max_motion_axis < 0.01:
                self.motion_armed = True
                self.log("MOTION_ARMED")
            else:
                self.log(
                    "WAIT_NEUTRAL s1=%+.2f s2=%+.2f s3=%+.2f"
                    % (axes["servo1"], axes["servo2"], axes["servo3"])
                )
                return False, landing_changed

        if self.args.coupled_z:
            arm_axes = {servo_id: axes["right_y"] for servo_id in self.arm_servo_ids}
        else:
            arm_axes = dict(zip(self.arm_servo_ids, (axes["servo1"], axes["servo2"], axes["servo3"])))

        arm_changed = False
        for servo_id in self.arm_servo_ids:
            if servo_id in self.disabled_arm_servos:
                continue
            axis_value = arm_axes[servo_id]
            if abs(axis_value) < 0.001:
                continue
            low, high = self.arm_raw_limits[servo_id]
            next_float = self.arm_command_float[servo_id] + (
                axis_value * float(self.args.arm_raw_per_sec) * float(dt) * float(self.arm_directions[servo_id])
            )
            next_raw = quantize_raw(next_float, self.mappings[servo_id]["position_step"], low, high)
            if int(self.args.max_feedback_lead_ticks) > 0:
                feedback = int(self.arm_feedback_raw[servo_id])
                next_raw = int(clamp(
                    next_raw,
                    max(low, feedback - int(self.args.max_feedback_lead_ticks)),
                    min(high, feedback + int(self.args.max_feedback_lead_ticks)),
                ))
            self.arm_command_float[servo_id] = float(next_raw)
            if next_raw != self.arm_command_raw[servo_id]:
                self.arm_command_raw[servo_id] = int(next_raw)
                arm_changed = True

        return arm_changed, landing_changed

    def send_targets(self, arm_changed, landing_changed):
        if not arm_changed and not landing_changed:
            return True
        targets = []
        max_delta = 0

        for servo_id in self.arm_servo_ids:
            if not arm_changed:
                continue
            target = int(self.arm_command_raw[servo_id])
            current = int(self.arm_feedback_raw.get(servo_id, target))
            targets.append((servo_id, target))
            max_delta = max(max_delta, abs(target - current))

        landing_move_ms = 0
        if landing_changed and self.landing_gear is not None:
            landing_move_ms = int(self.landing_gear["move_time_ms"])
            for servo_id in self.landing_gear["servo_ids"]:
                target = int(self.landing_command_raw[servo_id])
                current = int(self.landing_feedback_raw.get(servo_id, target))
                targets.append((servo_id, target))
                max_delta = max(max_delta, abs(target - current))

        if not targets:
            return True

        arm_move_ms = max(35, int((float(max_delta) / max(1.0, float(self.args.arm_raw_per_sec))) * 1000.0))
        time_ms = max(arm_move_ms, landing_move_ms)
        try:
            self.driver.set_servo_positions(targets, time_ms)
            self.log(
                "SEND time_ms=%d targets=%s"
                % (time_ms, ",".join("%d:%d" % (servo_id, raw) for servo_id, raw in targets))
            )
            return True
        except Exception as exc:
            self.fault = "servo send failed: %s" % exc
            self.log("SEND_FAILED %s" % exc)
            return False

    def status_text(self):
        arm_feedback = " ".join(
            "%d=%s" % (servo_id, self.arm_feedback_raw.get(servo_id, ""))
            for servo_id in self.arm_servo_ids
        )
        arm_command = " ".join(
            "%d=%s" % (servo_id, self.arm_command_raw.get(servo_id, ""))
            for servo_id in self.arm_servo_ids
        )
        lines = [
            "jetson_py36 raw jog controller",
            "time: %s" % now_text(),
            "output_dir: %s" % self.output_dir,
            "port: %s" % self.args.port,
            "arm_servos: %s" % "/".join(str(servo_id) for servo_id in self.arm_servo_ids),
            "landing_gear_servos: %s" % (
                ",".join(str(item) for item in self.landing_gear["servo_ids"])
                if self.landing_gear is not None
                else "disabled"
            ),
            "arm_feedback_raw: %s" % arm_feedback,
            "arm_command_raw: %s" % arm_command,
            "axes: left_x=%+.2f left_y=%+.2f right_y=%+.2f dpad_x=%+.2f dpad_y=%+.2f"
            % (
                self.last_axes["left_x"],
                self.last_axes["left_y"],
                self.last_axes["right_y"],
                self.last_axes["dpad_x"],
                self.last_axes["dpad_y"],
            ),
            "servo_axes: s1=%+.2f s2=%+.2f s3=%+.2f"
            % (
                self.last_axes["servo1"],
                self.last_axes["servo2"],
                self.last_axes["servo3"],
            ),
            "motion_armed: %s" % self.motion_armed,
            "landing_gear_state: %s" % self.landing_state,
        ]
        if self.disabled_arm_servos:
            lines.append(
                "disabled_arm_servos: %s"
                % ", ".join(
                    "%d=%d outside [%d,%d]" % (servo_id, raw, low, high)
                    for servo_id, (raw, low, high) in sorted(self.disabled_arm_servos.items())
                )
            )
        if self.landing_gear is not None:
            lines.append(
                "landing_gear_raw: %s"
                % ", ".join(
                    "%d=%s/%s"
                    % (
                        servo_id,
                        self.landing_feedback_raw.get(servo_id, ""),
                        self.landing_command_raw.get(servo_id, ""),
                    )
                    for servo_id in self.landing_gear["servo_ids"]
                )
            )
            if self.last_landing_feedback_error:
                lines.append("landing_gear_feedback_warn: %s" % self.last_landing_feedback_error)
        if self.fault:
            lines.append("fault: %s" % self.fault)
        return "\n".join(lines) + "\n"

    def write_status(self):
        with open(self.status_path, "w", encoding="utf-8") as fh:
            fh.write(self.status_text())

    def run(self):
        self.connect()
        print("")
        print("Raw jog control enabled.")
        if self.args.coupled_z:
            print("  right stick Y -> coupled raw jog for servos %s" % "/".join(str(item) for item in self.arm_servo_ids))
        else:
            print("  left stick X  -> servo %d raw" % self.arm_servo_ids[0])
            print("  left stick Y  -> servo %d raw" % self.arm_servo_ids[1])
            print("  right stick Y -> servo %d raw" % self.arm_servo_ids[2])
        if self.args.enable_dpad_backup:
            print("  dpad X/Y      -> backup input for servo 1/2")
        print("  LB            -> landing gear DOWN")
        print("  RB            -> landing gear UP")
        print("  A or START    -> quit")
        print("  Y             -> emergency stop")
        print("  motion will arm only after all axes return to neutral once")
        print("status: %s" % self.status_path)
        last_loop_time = time.time()
        while self.running:
            now = time.time()
            dt = max(0.001, min(0.20, now - last_loop_time))
            last_loop_time = now

            if not self.read_feedback():
                print(self.fault or "feedback failed")
                return 2

            arm_changed, landing_changed = self.update_from_gamepad(dt)
            if not self.running:
                break
            if not self.send_targets(arm_changed, landing_changed):
                print(self.fault or "send failed")
                return 2

            if now - self.last_status_print >= float(self.args.status_interval_sec):
                self.last_status_print = now
                print(
                    "status arm=(%s) cmd=(%s) axes=(%+.2f,%+.2f,%+.2f) gear=%s"
                    % (
                        ",".join(str(self.arm_feedback_raw.get(servo_id, 0)) for servo_id in self.arm_servo_ids),
                        ",".join(str(self.arm_command_raw.get(servo_id, 0)) for servo_id in self.arm_servo_ids),
                        self.last_axes["servo1"],
                        self.last_axes["servo2"],
                        self.last_axes["servo3"],
                        self.landing_state,
                    )
                )
            if now - self.last_status_write >= 0.10:
                self.last_status_write = now
                self.write_status()
            time.sleep(max(0.0, (1.0 / float(self.args.update_rate_hz)) - 0.002))
        print("Quit.")
        return 0


def parse_args(argv):
    default_output = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "samples",
        "raw_jog_%s" % time.strftime("%Y%m%d_%H%M%S", time.localtime()),
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--servo-timeout", type=float, default=DEFAULT_SERVO_TIMEOUT_SEC)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--gamepad-config", default=DEFAULT_GAMEPAD_CONFIG)
    parser.add_argument("--gamepad-device", default="")
    parser.add_argument("--output-dir", default=default_output)
    parser.add_argument("--update-rate-hz", type=float, default=DEFAULT_UPDATE_RATE_HZ)
    parser.add_argument("--arm-raw-per-sec", type=float, default=DEFAULT_ARM_RAW_PER_SEC)
    parser.add_argument("--arm-deadzone", type=float, default=DEFAULT_ARM_DEADZONE)
    parser.add_argument("--feedback-interval-sec", type=float, default=DEFAULT_FEEDBACK_INTERVAL_SEC)
    parser.add_argument("--status-interval-sec", type=float, default=DEFAULT_STATUS_INTERVAL_SEC)
    parser.add_argument("--feedback-read-retries", type=int, default=DEFAULT_FEEDBACK_READ_RETRIES)
    parser.add_argument("--feedback-retry-delay-sec", type=float, default=DEFAULT_FEEDBACK_RETRY_DELAY_SEC)
    parser.add_argument(
        "--arm-servo-ids",
        type=parse_arm_servo_ids,
        default=DEFAULT_ARM_SERVO_IDS,
        help="three configured arm-servo IDs in left-X,left-Y,right-Y order (default: 1,3,4)",
    )
    parser.add_argument(
        "--arm-raw-max",
        type=parse_arm_raw_max,
        default=None,
        help="optional per-axis upper raw guard in the same order as --arm-servo-ids",
    )
    parser.add_argument("--coupled-z", action="store_true", help="drive all arm axes together from right-stick Y")
    parser.add_argument("--max-feedback-lead-ticks", type=int, default=10)
    parser.add_argument(
        "--enable-landing-gear",
        action="store_true",
        help="enable optional 5/6 landing-gear commands on LB/RB",
    )
    parser.add_argument("--enable-dpad-backup", action="store_true")
    parser.add_argument("--arm-directions", type=parse_arm_directions, default=(1, 1, 1))
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    controller = RawJogController(args)
    try:
        return controller.run()
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
