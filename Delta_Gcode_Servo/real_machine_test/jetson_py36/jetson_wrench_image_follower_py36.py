#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Low-speed XY image follower for the live wrench detector.

Python 3.6 compatible. This script opens the servo bus and commands very small
XY steps from the wrench detector's normalized image error. Z is locked at the
startup feedback height.
"""

from __future__ import print_function

import argparse
import json
import math
import signal
import time
import urllib.request

from jetson_workspace_common import (
    DEFAULT_GAMEPAD_CONFIG,
    DEFAULT_SERVO_CONFIG,
    ServoMapper,
    forward_kinematics,
    inverse_kinematics,
    open_gamepad,
    open_servo_driver,
)


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600


def clamp(value, low, high):
    return max(low, min(high, value))


def fetch_json(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


class WrenchImageFollower(object):
    def __init__(self, args):
        self.args = args
        self.mapper = ServoMapper(args.servo_config)
        self.servo_ids = self.mapper.servo_ids
        self.driver = None
        self.gamepad = None
        self.running = True
        self.current_raw = dict(self.mapper.reference_raw)
        self.command_raw = dict(self.mapper.reference_raw)
        self.target_raw = dict(self.mapper.reference_raw)
        self.current_position = [0.0, 0.0, 240.0]
        self.target_position = [0.0, 0.0, 240.0]
        self.locked_z = None
        self.last_feedback = 0.0
        self.last_seen = 0.0
        self.ik_failures = 0
        self.filtered_error = None

    def connect(self):
        print("Opening servo %s @ %d" % (self.args.port, self.args.baudrate))
        self.driver = open_servo_driver(self.args.port, self.args.baudrate)
        if not self.args.no_gamepad:
            try:
                self.gamepad = open_gamepad(self.args.gamepad_config, self.args.gamepad_device)
                print("Gamepad opened: Y/A stop enabled.")
            except Exception as exc:
                print("Gamepad unavailable, Ctrl+C/timeout stop only: %s" % exc)

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

    def read_feedback(self, force=False):
        now = time.time()
        if not force and now - self.last_feedback < self.args.feedback_interval_sec:
            return True
        self.last_feedback = now
        last_error = None
        for attempt in range(max(1, int(self.args.feedback_read_retries))):
            try:
                raw = self.driver.read_servo_positions(self.servo_ids, timeout=self.args.servo_timeout)
                self.current_raw = {servo_id: int(raw[servo_id]) for servo_id in self.servo_ids}
                angles = self.mapper.raw_to_angles(self.current_raw)
                xyz, ok = forward_kinematics(angles[0], angles[1], angles[2])
                if not ok:
                    print("feedback FK failed; holding")
                    return False
                self.current_position = list(xyz)
                return True
            except Exception as exc:
                last_error = exc
                print("FEEDBACK_RETRY %d/%d %s" % (
                    attempt + 1,
                    max(1, int(self.args.feedback_read_retries)),
                    exc,
                ))
                time.sleep(0.03)
        print("FEEDBACK_FAILED %s" % last_error)
        return False

    def clamp_raw_to_limits(self, raw_values):
        limited = {}
        for servo_id in self.servo_ids:
            item = self.mapper.mappings[servo_id]
            value = int(raw_values[servo_id])
            value = int(clamp(value, min(item["raw_min"], item["raw_max"]), max(item["raw_min"], item["raw_max"])))
            value = min(value, int(self.mapper.reference_raw[servo_id]))
            if self.args.max_feedback_lead_ticks > 0:
                feedback = int(self.current_raw[servo_id])
                value = int(clamp(
                    value,
                    feedback - int(self.args.max_feedback_lead_ticks),
                    feedback + int(self.args.max_feedback_lead_ticks),
                ))
            limited[servo_id] = value
        return limited

    def set_target_position(self, xyz):
        target_x = float(xyz[0])
        target_y = float(xyz[1])
        radius = math.sqrt(target_x * target_x + target_y * target_y)
        if self.args.soft_xy_radius_mm > 0 and radius > float(self.args.soft_xy_radius_mm):
            scale = float(self.args.soft_xy_radius_mm) / max(1e-6, radius)
            target_x *= scale
            target_y *= scale
            print("SOFT_XY_LIMIT radius=%.1f limit=%.1f" % (radius, float(self.args.soft_xy_radius_mm)))
        target = [
            clamp(target_x, -150.0, 150.0),
            clamp(target_y, -150.0, 150.0),
            clamp(float(xyz[2]), float(self.args.z_min_mm), float(self.args.z_max_mm)),
        ]
        angles, ok = inverse_kinematics(target[0], target[1], target[2])
        if not ok:
            self.ik_failures += 1
            print("IK_FAIL target=(%.2f,%.2f,%.2f)" % (target[0], target[1], target[2]))
            if self.ik_failures >= int(self.args.max_ik_failures):
                print("STOP repeated IK failures")
                self.running = False
            return False
        self.ik_failures = 0
        self.target_position = target
        self.target_raw = self.clamp_raw_to_limits(self.mapper.angles_to_raw(angles))
        return True

    def command_respects_z_floor(self, raw_values):
        angles = self.mapper.raw_to_angles(raw_values)
        xyz, ok = forward_kinematics(angles[0], angles[1], angles[2])
        return ok and xyz[2] >= float(self.args.z_min_mm)

    def send_limited_motion(self):
        max_delta = max(1.0, float(self.args.max_servo_raw_s) / float(self.args.rate_hz))
        next_raw = {}
        changed = False
        for servo_id in self.servo_ids:
            current = int(self.command_raw[servo_id])
            target = int(self.target_raw[servo_id])
            delta = target - current
            if abs(delta) <= max_delta:
                value = target
            else:
                value = current + (int(max_delta) if delta > 0 else -int(max_delta))
            next_raw[servo_id] = value
            changed = changed or value != current
        next_raw = self.clamp_raw_to_limits(next_raw)
        if not changed or not self.command_respects_z_floor(next_raw):
            return True
        max_move = max(abs(next_raw[sid] - self.command_raw[sid]) for sid in self.servo_ids)
        time_ms = max(30, int((max_move / max(1.0, float(self.args.max_servo_raw_s))) * 1000.0))
        self.driver.set_servo_positions([(sid, next_raw[sid]) for sid in self.servo_ids], time_ms)
        self.command_raw = dict(next_raw)
        return True

    def read_stop_buttons(self):
        if self.gamepad is None:
            return False
        _x, _y, _z, buttons = self.gamepad.read()
        return bool(buttons.get("y") or buttons.get("a"))

    def compute_image_step(self, target):
        norm = target.get("normalized_xy") or {}
        ex = float(norm.get("x", 0.0))
        ey = float(norm.get("y", 0.0))
        if 0.0 < float(self.args.error_alpha) < 1.0:
            if self.filtered_error is None:
                self.filtered_error = [ex, ey]
            else:
                alpha = float(self.args.error_alpha)
                self.filtered_error[0] = alpha * ex + (1.0 - alpha) * self.filtered_error[0]
                self.filtered_error[1] = alpha * ey + (1.0 - alpha) * self.filtered_error[1]
            ex, ey = self.filtered_error
        if abs(ex) < float(self.args.deadband):
            ex = 0.0
        if abs(ey) < float(self.args.deadband):
            ey = 0.0
        sx = -1.0 if self.args.invert_x else 1.0
        sy = -1.0 if self.args.invert_y else 1.0
        dx = sx * float(self.args.gain_mm_per_norm) * ex
        dy = sy * float(self.args.gain_mm_per_norm) * ey
        mag = math.sqrt(dx * dx + dy * dy)
        if mag > float(self.args.max_step_mm) > 0:
            scale = float(self.args.max_step_mm) / mag
            dx *= scale
            dy *= scale
        return ex, ey, dx, dy

    def update_target_from_wrench(self):
        try:
            latest = fetch_json(self.args.latest_url, timeout=0.35)
        except Exception as exc:
            print("VISION_READ_FAIL %s" % exc)
            return False
        target = latest.get("target") or {}
        age = time.time() - float(latest.get("timestamp_unix", latest.get("timestamp", time.time())))
        conf = float(target.get("conf", 0.0) or 0.0)
        if not latest.get("valid") or not target or age > float(self.args.max_age_sec) or conf < float(self.args.min_conf):
            print("HOLD target_valid=%s age=%.3f conf=%.3f" % (bool(target), age, conf))
            return False
        ex, ey, dx, dy = self.compute_image_step(target)
        next_xyz = [self.target_position[0] + dx, self.target_position[1] + dy, self.locked_z]
        if self.set_target_position(next_xyz):
            self.last_seen = time.time()
            print(
                "FOLLOW conf=%.3f err=(%+.3f,%+.3f) step=(%+.2f,%+.2f) target=(%.1f,%.1f,%.1f)"
                % (conf, ex, ey, dx, dy, self.target_position[0], self.target_position[1], self.target_position[2])
            )
            return True
        return False

    def run(self):
        self.connect()
        if not self.read_feedback(force=True):
            return 2
        self.command_raw = self.clamp_raw_to_limits(self.current_raw)
        self.locked_z = clamp(self.current_position[2], float(self.args.z_min_mm), float(self.args.z_max_mm))
        self.target_position = [self.current_position[0], self.current_position[1], self.locked_z]
        if not self.set_target_position(self.target_position):
            return 2
        print("START fk=(%.1f,%.1f,%.1f) locked_z=%.1f duration=%.1fs" % (
            self.current_position[0], self.current_position[1], self.current_position[2], self.locked_z, self.args.duration_sec
        ))
        if not self.args.no_confirm:
            answer = input("Type FOLLOW to enable low-speed XY wrench follow: ").strip()
            if answer != "FOLLOW":
                print("Cancelled.")
                return 1
        deadline = time.time() + float(self.args.duration_sec)
        interval = 1.0 / float(self.args.rate_hz)
        while self.running and time.time() < deadline:
            if self.read_stop_buttons():
                print("STOP button requested")
                break
            if not self.read_feedback():
                break
            self.update_target_from_wrench()
            if not self.send_limited_motion():
                return 2
            time.sleep(interval)
        print("STOP follower")
        return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--servo-timeout", type=float, default=0.20)
    parser.add_argument("--feedback-read-retries", type=int, default=3)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--latest-url", default="http://127.0.0.1:8090/latest.json")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--feedback-interval-sec", type=float, default=0.20)
    parser.add_argument("--gain-mm-per-norm", type=float, default=5.0)
    parser.add_argument("--max-step-mm", type=float, default=0.8)
    parser.add_argument("--error-alpha", type=float, default=0.45, help="Low-pass factor for detector image error; 1 disables smoothing.")
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--min-conf", type=float, default=0.25)
    parser.add_argument("--max-age-sec", type=float, default=0.50)
    parser.add_argument("--invert-x", action="store_true")
    parser.add_argument("--invert-y", action="store_true")
    parser.add_argument("--z-min-mm", type=float, default=155.0)
    parser.add_argument("--z-max-mm", type=float, default=280.0)
    parser.add_argument("--max-servo-raw-s", type=float, default=80.0)
    parser.add_argument("--max-feedback-lead-ticks", type=int, default=25)
    parser.add_argument("--soft-xy-radius-mm", type=float, default=85.0)
    parser.add_argument("--max-ik-failures", type=int, default=5)
    parser.add_argument("--gamepad-config", default=DEFAULT_GAMEPAD_CONFIG)
    parser.add_argument("--gamepad-device", default="")
    parser.add_argument("--no-gamepad", action="store_true")
    parser.add_argument("--no-confirm", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    follower = WrenchImageFollower(args)

    def stop(_signum, _frame):
        follower.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        return follower.run()
    finally:
        follower.close()


if __name__ == "__main__":
    raise SystemExit(main())
