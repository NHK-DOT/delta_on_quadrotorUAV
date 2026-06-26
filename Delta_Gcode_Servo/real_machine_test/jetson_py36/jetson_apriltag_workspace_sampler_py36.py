#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python 3.6 Jetson AprilTag + 8BitDo workspace sampler.

This is the field entrypoint for the Jetson Xavier NX at 192.168.1.80. It combines
low-speed manual 8BitDo control, servo feedback, and the 3K fisheye AprilTag
JSON stream. Press B to record one sample for workspace model fitting.
"""

from __future__ import print_function

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
import traceback

from jetson_workspace_common import (
    DEFAULT_APRILTAG_JSON,
    DEFAULT_APRILTAG_LAUNCH,
    DEFAULT_CALIBRATION,
    DEFAULT_GAMEPAD_CONFIG,
    DEFAULT_SERVO_CONFIG,
    ServoMapper,
    append_jsonl,
    forward_kinematics,
    inverse_kinematics,
    load_tool_pose_from_apriltag,
    now_iso,
    open_gamepad,
    open_servo_driver,
    snapshot_age_ms,
    write_json,
)


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600
DEFAULT_UPDATE_RATE_HZ = 40.0
DEFAULT_SPEED_XY_MM_S = 34.0
DEFAULT_SPEED_Z_MM_S = 20.0
DEFAULT_MAX_SERVO_RAW_S = 160.0
DEFAULT_STARTUP_HOME_RAW_S = 120.0
DEFAULT_HOME_TOLERANCE_TICKS = 150
DEFAULT_RAW_RANGE_MARGIN_TICKS = 30
DEFAULT_FRESH_MS = 1000.0
DEFAULT_FEEDBACK_INTERVAL_SEC = 0.25
DEFAULT_SERVO_TIMEOUT_SEC = 0.20
DEFAULT_FEEDBACK_READ_RETRIES = 3
DEFAULT_Z_MIN_MM = 155.0
DEFAULT_Z_MAX_MM = 280.0
DEFAULT_MAX_FEEDBACK_LEAD_TICKS = 35


def clamp(value, low, high):
    return max(low, min(high, value))


def ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def timestamp_run_id():
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


def append_csv(path, row):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    exists = os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def finite_xyz(values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        return None
    out = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        out.append(number)
    return out


class ManagedAprilTagProcess(object):
    def __init__(self, launch_script, output_json, log_path):
        self.launch_script = launch_script
        self.output_json = output_json
        self.log_path = log_path
        self.process = None
        self.log_file = None

    def start(self):
        if not self.launch_script or not os.path.exists(self.launch_script):
            print("AprilTag launch script missing: %s" % self.launch_script)
            return False
        ensure_dir(os.path.dirname(os.path.abspath(self.log_path)))
        self.log_file = open(self.log_path, "a", encoding="utf-8", buffering=1)
        self.log_file.write("\n# %s start %s\n" % (now_iso(), self.launch_script))
        env = os.environ.copy()
        env["OUT_JSON"] = self.output_json
        self.process = subprocess.Popen(
            ["bash", self.launch_script],
            cwd=os.path.dirname(os.path.abspath(self.launch_script)),
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            env=env,
        )
        print("Started AprilTag process pid=%s" % self.process.pid)
        return True

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            print("Stopping AprilTag process...")
            try:
                self.process.terminate()
                deadline = time.time() + 4.0
                while self.process.poll() is None and time.time() < deadline:
                    time.sleep(0.1)
                if self.process.poll() is None:
                    self.process.kill()
            except Exception:
                pass
        if self.log_file is not None:
            try:
                self.log_file.write("# %s stop rc=%s\n" % (now_iso(), self.process.poll() if self.process else None))
                self.log_file.close()
            except Exception:
                pass


class JetsonWorkspaceSampler(object):
    def __init__(self, args):
        self.args = args
        self.mapper = ServoMapper(args.servo_config)
        self.servo_ids = self.mapper.servo_ids
        self.driver = None
        self.gamepad = None
        self.running = True
        self.fault = None

        self.output_dir = os.path.abspath(args.output_dir)
        ensure_dir(self.output_dir)
        self.samples_csv = os.path.join(self.output_dir, "samples.csv")
        self.samples_jsonl = os.path.join(self.output_dir, "samples.jsonl")
        self.session_json = os.path.join(self.output_dir, "session.json")
        self.status_path = os.path.join(self.output_dir, "runtime_status.log")
        self.debug_log_path = os.path.join(self.output_dir, "debug.log")
        self.debug_log = open(self.debug_log_path, "a", encoding="utf-8", buffering=1)

        self.update_interval = 1.0 / float(args.update_rate_hz)
        self.speed_xy = float(args.speed_xy_mm_s)
        self.speed_z = float(args.speed_z_mm_s)
        self.max_servo_raw_s = float(args.max_servo_raw_s)
        self.max_feedback_lead_ticks = int(args.max_feedback_lead_ticks)
        self.feedback_interval = float(args.feedback_interval_sec)

        self.safe_scan_mode = "FREE"
        self.sample_count = self.count_existing_samples()
        self.home_raw = dict(self.mapper.reference_raw)
        self.current_raw = dict(self.home_raw)
        self.command_raw = dict(self.home_raw)
        self.target_raw = dict(self.home_raw)
        self.current_angles = self.mapper.raw_to_angles(self.current_raw)
        self.target_angles = list(self.current_angles)
        self.current_position, ok = forward_kinematics(
            self.current_angles[0], self.current_angles[1], self.current_angles[2]
        )
        if not ok:
            self.current_position = [0.0, 0.0, 240.0]
        self.target_position = list(self.current_position)
        self.last_feedback_time = 0.0
        self.last_status_time = 0.0
        self.last_voltage_time = 0.0
        self.battery_mv = None
        self.last_axes = (0.0, 0.0, 0.0)
        self.last_motion_axes = (0.0, 0.0, 0.0)
        self.last_buttons = {"a": False, "b": False, "x": False, "y": False, "lb": False, "rb": False}

        self.session_payload = {
            "created_iso": now_iso(),
            "created_unix": time.time(),
            "mode": "jetson_py36_apriltag_8bitdo_workspace_sampler",
            "units": "mm/raw/radian",
            "files": {
                "samples_csv": self.samples_csv,
                "samples_jsonl": self.samples_jsonl,
                "runtime_status": self.status_path,
                "debug_log": self.debug_log_path,
            },
            "inputs": {
                "base_camera_snapshot": args.base_camera_snapshot,
                "calibration": args.calibration,
                "servo_config": args.servo_config,
                "gamepad_config": args.gamepad_config,
                "hand_tag_id": args.hand_tag_id,
            },
            "control": {
                "port": args.port,
                "baudrate": args.baudrate,
                "speed_xy_mm_s": self.speed_xy,
                "speed_z_mm_s": self.speed_z,
                "max_servo_raw_s": self.max_servo_raw_s,
                "max_feedback_lead_ticks": self.max_feedback_lead_ticks,
                "feedback_interval_sec": self.feedback_interval,
                "update_rate_hz": args.update_rate_hz,
                "z_min_mm": float(args.z_min_mm),
                "z_max_mm": float(args.z_max_mm),
            },
            "home_raw": self.home_raw,
            "startup_check_raw": dict(self.mapper.startup_check_raw),
            "note": args.note,
        }
        write_json(self.session_json, self.session_payload)

    def count_existing_samples(self):
        if not os.path.exists(self.samples_csv):
            return 0
        try:
            with open(self.samples_csv, "r", newline="", encoding="utf-8-sig") as fh:
                return sum(1 for _ in csv.DictReader(fh))
        except Exception:
            return 0

    def log(self, message):
        try:
            self.debug_log.write("%0.6f %s %s\n" % (time.time(), now_iso(), message))
        except Exception:
            pass

    def connect(self):
        print("Opening serial %s @ %d..." % (self.args.port, self.args.baudrate))
        self.driver = open_servo_driver(self.args.port, self.args.baudrate)
        print("Serial opened.")
        print("Opening 8BitDo controller...")
        self.gamepad = open_gamepad(self.args.gamepad_config, self.args.gamepad_device)
        print("8BitDo opened.")

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
            self.debug_log.close()
        except Exception:
            pass

    def read_feedback(self, force=False, strict=False):
        now = time.time()
        if not force and now - self.last_feedback_time < self.feedback_interval:
            return True
        self.last_feedback_time = now
        last_error = None
        attempts = max(1, int(self.args.feedback_read_retries))
        for attempt in range(attempts):
            try:
                raw = self.driver.read_servo_positions(self.servo_ids, timeout=self.args.servo_timeout)
                self.current_raw = {servo_id: int(raw[servo_id]) for servo_id in self.servo_ids}
                self.current_angles = self.mapper.raw_to_angles(self.current_raw)
                xyz, ok = forward_kinematics(self.current_angles[0], self.current_angles[1], self.current_angles[2])
                if ok:
                    self.current_position = list(xyz)
                elif strict:
                    self.fault = "feedback raw cannot be converted by FK"
                    return False
                if self.fault and self.fault.startswith("servo feedback read failed"):
                    self.fault = None
                if now - self.last_voltage_time >= 1.5:
                    self.last_voltage_time = now
                    try:
                        self.battery_mv = self.driver.get_battery_voltage_mv(timeout=self.args.servo_timeout)
                    except Exception as exc:
                        self.log("BATTERY_READ_FAILED %s" % exc)
                return True
            except Exception as exc:
                last_error = exc
                self.log("FEEDBACK_READ_RETRY %d/%d %s" % (attempt + 1, attempts, exc))
                time.sleep(0.03)
        self.fault = "servo feedback read failed: %s" % last_error
        self.log("FEEDBACK_FAILED %s" % last_error)
        return not strict

    def home_errors(self):
        return self.mapper.startup_check_errors(self.current_raw)

    def at_home(self):
        errors = self.home_errors()
        return max(abs(int(value)) for value in errors.values()) <= int(self.args.home_tolerance)

    def raw_range_violations(self):
        return self.mapper.raw_range_violations(self.current_raw, margin_ticks=self.args.raw_range_margin)

    def print_raw_range_violations(self, violations):
        if not violations:
            return
        print("  Raw range warning:")
        for servo_id in self.servo_ids:
            if servo_id in violations:
                item = violations[servo_id]
                print(
                    "    servo %d raw=%d outside configured range [%d,%d] +/- %d ticks"
                    % (
                        servo_id,
                        item["raw"],
                        item["configured_raw_min"],
                        item["configured_raw_max"],
                        self.args.raw_range_margin,
                    )
                )

    def clamp_target_position(self, position):
        return [
            clamp(float(position[0]), -150.0, 150.0),
            clamp(float(position[1]), -150.0, 150.0),
            clamp(float(position[2]), float(self.args.z_min_mm), float(self.args.z_max_mm)),
        ]

    def set_target_position(self, position):
        target_position = self.clamp_target_position(position)
        angles, ok = inverse_kinematics(target_position[0], target_position[1], target_position[2])
        if not ok:
            self.log("IK_FAIL xyz=%.3f %.3f %.3f" % (target_position[0], target_position[1], target_position[2]))
            return False
        self.target_position = target_position
        self.target_angles = list(angles)
        self.target_raw = self.clamp_raw_to_motion_limits(self.mapper.angles_to_raw(angles))
        return True

    def sync_control_state_to_feedback(self):
        self.command_raw = self.clamp_raw_to_motion_limits(self.current_raw)
        if not self.set_target_position(self.current_position):
            self.target_raw = dict(self.command_raw)
            self.target_angles = list(self.current_angles)
            self.target_position = list(self.current_position)
        self.write_status()

    def clamp_raw_to_motion_limits(self, raw_values):
        limited = {}
        for servo_id in self.servo_ids:
            item = self.mapper.mappings[servo_id]
            low = min(int(item["raw_min"]), int(item["raw_max"]))
            high = max(int(item["raw_min"]), int(item["raw_max"]))
            home_limit = int(self.home_raw[servo_id])
            value = int(raw_values[servo_id])
            value = int(clamp(value, low, high))
            limited[servo_id] = min(value, home_limit)
        return limited

    def move_to_motion_home(self):
        target_raw = dict(self.home_raw)
        current_raw = dict(self.current_raw)
        max_error = max(abs(int(target_raw[servo_id]) - int(current_raw[servo_id])) for servo_id in self.servo_ids)
        if max_error <= int(self.args.home_tolerance):
            print("Already near motion mapping home raw.")
            return True
        time_ms = max(500, int((max_error / max(1.0, float(self.args.startup_home_raw_s))) * 1000.0))
        targets = [(servo_id, int(target_raw[servo_id])) for servo_id in self.servo_ids]
        print(
            "  HOME move command: target raw 1=%d 2=%d 3=%d, time=%d ms"
            % (target_raw[1], target_raw[2], target_raw[3], time_ms)
        )
        try:
            self.driver.set_servo_positions(targets, time_ms)
        except Exception as exc:
            self.fault = "startup HOME send failed: %s" % exc
            print(self.fault)
            self.log("STARTUP_HOME_SEND_FAILED %s" % exc)
            return False
        time.sleep(min(8.0, time_ms / 1000.0 + 0.5))
        if not self.read_feedback(force=True, strict=True):
            print(self.fault or "startup HOME feedback failed")
            return False
        violations = self.raw_range_violations()
        if violations:
            self.print_raw_range_violations(violations)
            print("Startup HOME refused: feedback is outside configured raw range.")
            return False
        errors = self.mapper.home_errors(self.current_raw)
        max_after = max(abs(int(errors[servo_id])) for servo_id in self.servo_ids)
        print(
            "  HOME result raw:  1=%d 2=%d 3=%d"
            % (self.current_raw[1], self.current_raw[2], self.current_raw[3])
        )
        print("  HOME result errors to motion home: 1=%+d 2=%+d 3=%+d" % (errors[1], errors[2], errors[3]))
        if max_after > int(self.args.home_tolerance):
            print("Startup HOME did not reach configured motion home within tolerance.")
            return False
        return True

    def startup_check(self):
        print("")
        print("Startup safety check")
        startup_raw = self.mapper.startup_check_raw
        print(
            "  Expected startup raw: 1=%d 2=%d 3=%d"
            % (startup_raw[1], startup_raw[2], startup_raw[3])
        )
        print("  Motion mapping home raw: 1=%d 2=%d 3=%d" % (self.home_raw[1], self.home_raw[2], self.home_raw[3]))
        print("  This sampler will read feedback before any move command.")
        if not self.read_feedback(force=True, strict=True):
            print(self.fault or "feedback failed")
            return False
        errors = self.home_errors()
        print(
            "  Current raw:       1=%d 2=%d 3=%d"
            % (self.current_raw[1], self.current_raw[2], self.current_raw[3])
        )
        print("  Home error ticks:  1=%+d 2=%+d 3=%+d" % (errors[1], errors[2], errors[3]))
        violations = self.raw_range_violations()
        if violations:
            self.print_raw_range_violations(violations)
            print("Startup refused: servo feedback is outside configured raw range.")
            return False
        if not self.at_home():
            print("Startup feedback is not near configured startup_check_raw.")
            print("Type HOME to slowly move to motion mapping home_raw, or rerun with --allow-not-home for expert manual sampling.")
            if self.args.allow_not_home:
                print("Continuing because --allow-not-home was set.")
            else:
                answer = input("Type HOME to slowly move to configured home_raw: ").strip()
                if answer != "HOME":
                    print("Startup HOME cancelled.")
                    return False
                if not self.move_to_motion_home():
                    return False
                errors = self.home_errors()
                if not self.at_home():
                    print("Startup refused: arm is still not near configured startup_check_raw.")
                    print("Startup errors ticks: 1=%+d 2=%+d 3=%+d" % (errors[1], errors[2], errors[3]))
                    return False
        age = snapshot_age_ms(self.args.base_camera_snapshot)
        if age is None:
            print("AprilTag snapshot is missing or has no timestamp: %s" % self.args.base_camera_snapshot)
            if not self.args.allow_stale_vision:
                return False
        elif age > float(self.args.fresh_ms):
            print("AprilTag snapshot is stale: %.0f ms > %.0f ms" % (age, self.args.fresh_ms))
            if not self.args.allow_stale_vision:
                return False
        try:
            pose = load_tool_pose_from_apriltag(
                self.args.base_camera_snapshot,
                self.args.calibration,
                self.args.hand_tag_id,
            )
            xyz = pose.get("tool_position_mm")
            print("  AprilTag tool xyz: %.2f %.2f %.2f mm" % (xyz[0], xyz[1], xyz[2]))
        except Exception as exc:
            print("AprilTag/tool transform check failed: %s" % exc)
            if not self.args.allow_stale_vision:
                return False

        if not self.args.no_confirm:
            answer = input("Type YES to enable low-speed manual servo control: ").strip()
            if answer != "YES":
                print("Cancelled.")
                return False

        self.sync_control_state_to_feedback()
        return True

    def axis_to_step(self, value, threshold=0.55):
        if abs(float(value)) < threshold:
            return 0.0
        return 1.0 if value > 0 else -1.0

    def cycle_safe_scan(self):
        modes = ["FREE", "X", "Y", "Z"]
        index = modes.index(self.safe_scan_mode)
        self.safe_scan_mode = modes[(index + 1) % len(modes)]
        print("safe scan -> %s" % self.safe_scan_mode)
        self.write_status()

    def update_from_gamepad(self):
        dpad_x, dpad_y, right_y, buttons = self.gamepad.read()
        self.last_axes = (dpad_x, dpad_y, right_y)
        previous = dict(self.last_buttons)
        self.last_buttons = dict(buttons)

        if buttons.get("a", False):
            self.running = False
            return False

        if buttons.get("y", False):
            self.fault = "emergency stop requested by Y"
            self.log("EMERGENCY_STOP_Y")
            self.running = False
            return False

        if buttons.get("x", False) and not previous.get("x", False):
            self.cycle_safe_scan()

        if buttons.get("b", False) and not previous.get("b", False):
            self.record_sample("sample_%04d" % (self.sample_count + 1))

        motion_x = self.axis_to_step(dpad_x)
        motion_y = self.axis_to_step(dpad_y)
        motion_z = self.axis_to_step(right_y)
        self.last_motion_axes = (motion_x, motion_y, motion_z)
        if max(abs(motion_x), abs(motion_y), abs(motion_z)) < 0.01:
            return True

        next_position = list(self.target_position)
        next_position[0] += motion_x * self.speed_xy * self.update_interval
        next_position[1] += -motion_y * self.speed_xy * self.update_interval
        next_position[2] += -motion_z * self.speed_z * self.update_interval
        if self.safe_scan_mode == "X":
            next_position[1] = self.target_position[1]
            next_position[2] = self.target_position[2]
        elif self.safe_scan_mode == "Y":
            next_position[0] = self.target_position[0]
            next_position[2] = self.target_position[2]
        elif self.safe_scan_mode == "Z":
            next_position[0] = self.target_position[0]
            next_position[1] = self.target_position[1]

        self.set_target_position(next_position)
        return True

    def compute_limited_command_raw(self):
        max_delta = max(1.0, self.max_servo_raw_s * self.update_interval)
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
            item = self.mapper.mappings[servo_id]
            value = int(clamp(value, min(item["raw_min"], item["raw_max"]), max(item["raw_min"], item["raw_max"])))
            value = min(value, int(self.home_raw[servo_id]))
            if self.max_feedback_lead_ticks > 0:
                feedback = int(self.current_raw[servo_id])
                value = int(clamp(
                    value,
                    feedback - self.max_feedback_lead_ticks,
                    feedback + self.max_feedback_lead_ticks,
                ))
                value = int(clamp(value, min(item["raw_min"], item["raw_max"]), max(item["raw_min"], item["raw_max"])))
                value = min(value, int(self.home_raw[servo_id]))
            next_raw[servo_id] = value
            if value != current:
                changed = True
        if changed and not self.command_respects_z_floor(next_raw):
            return dict(self.command_raw), False
        return next_raw, changed

    def command_respects_z_floor(self, proposed_raw):
        z_min = float(self.args.z_min_mm)
        proposed_angles = self.mapper.raw_to_angles(proposed_raw)
        proposed_position, proposed_ok = forward_kinematics(
            proposed_angles[0], proposed_angles[1], proposed_angles[2]
        )
        if not proposed_ok:
            self.log("COMMAND_FK_FAIL raw=1:%d 2:%d 3:%d" % (
                proposed_raw[1],
                proposed_raw[2],
                proposed_raw[3],
            ))
            return False
        if proposed_position[2] >= z_min:
            return True

        current_angles = self.mapper.raw_to_angles(self.command_raw)
        current_position, current_ok = forward_kinematics(
            current_angles[0], current_angles[1], current_angles[2]
        )
        if current_ok and proposed_position[2] > current_position[2]:
            return True

        self.log(
            "COMMAND_Z_FLOOR_HOLD proposed_z=%.3f z_min=%.3f raw=1:%d 2:%d 3:%d"
            % (proposed_position[2], z_min, proposed_raw[1], proposed_raw[2], proposed_raw[3])
        )
        return False

    def send_motion(self):
        next_raw, changed = self.compute_limited_command_raw()
        if not changed:
            return True
        try:
            max_move = max(abs(next_raw[servo_id] - self.command_raw[servo_id]) for servo_id in self.servo_ids)
            time_ms = max(20, int((max_move / max(1.0, self.max_servo_raw_s)) * 1000.0))
            targets = [(servo_id, next_raw[servo_id]) for servo_id in self.servo_ids]
            self.driver.set_servo_positions(targets, time_ms)
            self.command_raw = dict(next_raw)
            return True
        except Exception as exc:
            self.fault = "servo send failed: %s" % exc
            self.log("SEND_FAILED %s" % exc)
            return False

    def record_sample(self, label):
        try:
            if not self.read_feedback(force=True, strict=True):
                print("sample refused: %s" % (self.fault or "feedback failed"))
                return False
            vision = load_tool_pose_from_apriltag(
                self.args.base_camera_snapshot,
                self.args.calibration,
                self.args.hand_tag_id,
            )
        except Exception as exc:
            print("sample refused: vision/feedback failed: %s" % exc)
            if self.args.verbose:
                traceback.print_exc()
            return False

        vision_xyz = finite_xyz(vision.get("tool_position_mm"))
        fk_xyz = finite_xyz(self.current_position)
        if vision_xyz is None or fk_xyz is None:
            print("sample refused: invalid XYZ")
            return False
        if fk_xyz[2] < float(self.args.z_min_mm):
            print("sample refused: feedback z %.1f below z_min %.1f" % (fk_xyz[2], float(self.args.z_min_mm)))
            return False
        age = vision.get("snapshot_age_ms")
        if isinstance(age, (int, float)) and age > float(self.args.fresh_ms) and not self.args.allow_stale_vision:
            print("sample refused: AprilTag snapshot stale %.0f ms" % age)
            return False

        offset = [vision_xyz[i] - fk_xyz[i] for i in range(3)]
        self.sample_count += 1
        sample = {
            "index": self.sample_count,
            "label": label,
            "timestamp_iso": now_iso(),
            "timestamp_unix": time.time(),
            "mode": "jetson_py36_manual_8bitdo_apriltag_sample",
            "safe_scan_mode": self.safe_scan_mode,
            "servo_raw": {str(servo_id): int(self.current_raw[servo_id]) for servo_id in self.servo_ids},
            "servo_target_raw": {str(servo_id): int(self.target_raw[servo_id]) for servo_id in self.servo_ids},
            "servo_angles_deg": [math.degrees(value) for value in self.current_angles],
            "fk_feedback_xyz_mm": fk_xyz,
            "target_xyz_mm": finite_xyz(self.target_position),
            "vision": vision,
            "vision_tool_preview": vision,
            "tool_position_mm": vision_xyz,
            "vision_xyz_mm": vision_xyz,
            "vision_minus_fk_offset_mm": offset,
            "battery_mv": self.battery_mv,
            "input_axes": list(self.last_axes),
            "motion_axes": list(self.last_motion_axes),
            "operator_note": self.args.note,
        }
        append_jsonl(self.samples_jsonl, sample)
        row = {
            "index": self.sample_count,
            "label": label,
            "timestamp_iso": sample["timestamp_iso"],
            "raw1": self.current_raw[1],
            "raw2": self.current_raw[2],
            "raw3": self.current_raw[3],
            "fk_x_mm": fk_xyz[0],
            "fk_y_mm": fk_xyz[1],
            "fk_z_mm": fk_xyz[2],
            "x_mm": vision_xyz[0],
            "y_mm": vision_xyz[1],
            "z_mm": vision_xyz[2],
            "offset_x_mm": offset[0],
            "offset_y_mm": offset[1],
            "offset_z_mm": offset[2],
            "vision_detection_id": vision.get("detection_id"),
            "vision_snapshot_age_ms": age if age is not None else "",
            "safe_scan_mode": self.safe_scan_mode,
            "battery_mv": self.battery_mv if self.battery_mv is not None else "",
        }
        append_csv(self.samples_csv, row)
        print(
            "sample #%d raw=(%d,%d,%d) vision=(%+.2f,%+.2f,%+.2f) offset=(%+.2f,%+.2f,%+.2f) mm"
            % (
                self.sample_count,
                self.current_raw[1],
                self.current_raw[2],
                self.current_raw[3],
                vision_xyz[0],
                vision_xyz[1],
                vision_xyz[2],
                offset[0],
                offset[1],
                offset[2],
            )
        )
        self.write_status()
        return True

    def status_text(self):
        age = snapshot_age_ms(self.args.base_camera_snapshot)
        age_text = "" if age is None else "%.0f" % age
        lines = [
            "jetson_py36 AprilTag workspace sampler",
            "time: %s" % now_iso(),
            "output_dir: %s" % self.output_dir,
            "samples: %d" % self.sample_count,
            "port: %s" % self.args.port,
            "hand_tag_id: %s" % self.args.hand_tag_id,
            "safe_scan: %s" % self.safe_scan_mode,
            "feedback_raw: 1=%d 2=%d 3=%d" % (self.current_raw[1], self.current_raw[2], self.current_raw[3]),
            "target_raw: 1=%d 2=%d 3=%d" % (self.target_raw[1], self.target_raw[2], self.target_raw[3]),
            "command_raw: 1=%d 2=%d 3=%d" % (self.command_raw[1], self.command_raw[2], self.command_raw[3]),
            "fk_xyz_mm: %.3f %.3f %.3f" % (self.current_position[0], self.current_position[1], self.current_position[2]),
            "target_xyz_mm: %.3f %.3f %.3f" % (self.target_position[0], self.target_position[1], self.target_position[2]),
            "axes: dpad_x=%+.2f dpad_y=%+.2f right_y=%+.2f" % self.last_axes,
            "motion_axes: x=%+.2f y=%+.2f z=%+.2f" % self.last_motion_axes,
            "apriltag_age_ms: %s" % age_text,
            "battery_mv: %s" % (self.battery_mv if self.battery_mv is not None else ""),
        ]
        if self.fault:
            lines.append("fault: %s" % self.fault)
        return "\n".join(lines) + "\n"

    def write_status(self):
        with open(self.status_path, "w", encoding="utf-8") as fh:
            fh.write(self.status_text())

    def run(self):
        self.connect()
        if not self.startup_check():
            return 2
        print("")
        print("Controls:")
        print("  D-pad: X/Y low-speed motion")
        print("  Right stick Y: Z low-speed motion")
        print("  B: sample current AprilTag XYZ + servo raw")
        print("  X: cycle safe-scan FREE/X/Y/Z")
        print("  Y: emergency stop sampler")
        print("  A: quit")
        print("Samples: %s" % self.samples_csv)
        print("Full JSONL: %s" % self.samples_jsonl)
        last_status_print = 0.0
        while self.running:
            if not self.read_feedback():
                print(self.fault or "feedback failed")
                return 2
            self.update_from_gamepad()
            if not self.running:
                break
            if not self.send_motion():
                print(self.fault or "send failed")
                return 2
            now = time.time()
            if now - self.last_status_time >= 0.25:
                self.write_status()
                self.last_status_time = now
            if now - last_status_print >= 2.0:
                last_status_print = now
                age = snapshot_age_ms(self.args.base_camera_snapshot)
                age_text = "unknown" if age is None else "%.0fms" % age
                print(
                    "status raw=(%d,%d,%d) fk=(%.1f,%.1f,%.1f) tag_age=%s samples=%d"
                    % (
                        self.current_raw[1],
                        self.current_raw[2],
                        self.current_raw[3],
                        self.current_position[0],
                        self.current_position[1],
                        self.current_position[2],
                        age_text,
                        self.sample_count,
                    )
                )
            time.sleep(self.update_interval)
        print("Quit.")
        return 0


def wait_for_fresh_snapshot(path, max_age_ms, timeout_sec):
    deadline = time.time() + float(timeout_sec)
    while time.time() < deadline:
        age = snapshot_age_ms(path)
        if age is not None and age <= float(max_age_ms):
            print("AprilTag snapshot fresh: age=%.0f ms" % age)
            return True
        time.sleep(0.25)
    age = snapshot_age_ms(path)
    if age is None:
        print("AprilTag snapshot not ready: %s" % path)
    else:
        print("AprilTag snapshot not fresh: %.0f ms" % age)
    return False


def parse_args(argv):
    default_output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "samples", timestamp_run_id())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--servo-timeout", type=float, default=DEFAULT_SERVO_TIMEOUT_SEC)
    parser.add_argument("--base-camera-snapshot", default=DEFAULT_APRILTAG_JSON)
    parser.add_argument("--apriltag-launch", default=DEFAULT_APRILTAG_LAUNCH)
    parser.add_argument("--no-autostart-apriltag", action="store_true")
    parser.add_argument("--apriltag-startup-timeout", type=float, default=15.0)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--gamepad-config", default=DEFAULT_GAMEPAD_CONFIG)
    parser.add_argument("--gamepad-device", default="")
    parser.add_argument("--hand-tag-id", type=int, default=None)
    parser.add_argument("--fresh-ms", type=float, default=DEFAULT_FRESH_MS)
    parser.add_argument("--allow-stale-vision", action="store_true")
    parser.add_argument("--home-tolerance", type=int, default=DEFAULT_HOME_TOLERANCE_TICKS)
    parser.add_argument("--raw-range-margin", type=int, default=DEFAULT_RAW_RANGE_MARGIN_TICKS)
    parser.add_argument("--allow-not-home", action="store_true")
    parser.add_argument("--no-confirm", action="store_true")
    parser.add_argument("--output-dir", default=default_output)
    parser.add_argument("--update-rate-hz", type=float, default=DEFAULT_UPDATE_RATE_HZ)
    parser.add_argument("--feedback-interval-sec", type=float, default=DEFAULT_FEEDBACK_INTERVAL_SEC)
    parser.add_argument("--feedback-read-retries", type=int, default=DEFAULT_FEEDBACK_READ_RETRIES)
    parser.add_argument("--speed-xy-mm-s", type=float, default=DEFAULT_SPEED_XY_MM_S)
    parser.add_argument("--speed-z-mm-s", type=float, default=DEFAULT_SPEED_Z_MM_S)
    parser.add_argument("--max-servo-raw-s", type=float, default=DEFAULT_MAX_SERVO_RAW_S)
    parser.add_argument("--max-feedback-lead-ticks", type=int, default=DEFAULT_MAX_FEEDBACK_LEAD_TICKS)
    parser.add_argument("--startup-home-raw-s", type=float, default=DEFAULT_STARTUP_HOME_RAW_S)
    parser.add_argument("--z-min-mm", type=float, default=DEFAULT_Z_MIN_MM)
    parser.add_argument("--z-max-mm", type=float, default=DEFAULT_Z_MAX_MM)
    parser.add_argument("--note", default="")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    apriltag_proc = None
    sampler = None

    def handle_signal(_signum, _frame):
        if sampler is not None:
            sampler.running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        if not args.no_autostart_apriltag:
            apriltag_proc = ManagedAprilTagProcess(
                args.apriltag_launch,
                args.base_camera_snapshot,
                os.path.join(args.output_dir, "logs", "jetson_apriltag3k.log"),
            )
            apriltag_proc.start()
            wait_for_fresh_snapshot(args.base_camera_snapshot, args.fresh_ms, args.apriltag_startup_timeout)
        sampler = JetsonWorkspaceSampler(args)
        return sampler.run()
    except Exception as exc:
        print("fatal: %s" % exc)
        if args.verbose:
            traceback.print_exc()
        return 2
    finally:
        if sampler is not None:
            sampler.close()
        if apriltag_proc is not None:
            apriltag_proc.stop()


if __name__ == "__main__":
    raise SystemExit(main())
