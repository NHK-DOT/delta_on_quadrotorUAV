#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

from config import (
    BAUDRATE,
    DEFAULT_JOYSTICK,
    DEFAULT_PORT,
    DPAD_SLEW_RATE,
    DPAD_THRESHOLD,
    FEEDBACK_INTERVAL_SEC,
    FEEDBACK_TIMEOUT_SEC,
    MAX_SERVO_SPEED_TICKS_PER_SEC,
    MIN_EFFECTIVE_MOVE_TICKS,
    SERVO_RAW_DIRECTIONS,
    SPEED_XY_MM_PER_SEC,
    SPEED_Z_MM_PER_SEC,
    STARTUP_FEEDBACK_TIMEOUT_SEC,
    STARTUP_TOLERANCE_TICKS,
    TOOLING_SERVO,
    TOOLING_SERVO_ENABLED,
    UPDATE_RATE_HZ,
    robot_params,
)
from joystick_linux import LinuxJoystickReader
from kinematics import DeltaRobot, forward_kinematics, inverse_kinematics
from servo_driver import BusServoDriver, serial_permission_hint
from servo_mapping import load_servo_mappings_for_ids


def prompt_input(message):
    return input(message)


def clamp(value, low, high):
    return max(low, min(high, value))


def format_packet_hex(packet):
    if not packet:
        return "<none>"
    return " ".join("%02X" % byte for byte in bytearray(packet))


class ToolingServo(object):
    def __init__(self, config):
        self.servo_id = int(config["servo_id"])
        self.raw_min = int(config["raw_min"])
        self.raw_max = int(config["raw_max"])
        self.position_step = int(config["position_step"])
        self.speed_ticks_per_sec = float(config["speed_ticks_per_sec"])

    @property
    def center_raw(self):
        value = (self.raw_min + self.raw_max) / 2.0
        return self.clamp(value)

    def clamp(self, raw_value):
        value = int(round(float(raw_value) / self.position_step) * self.position_step)
        low = min(self.raw_min, self.raw_max)
        high = max(self.raw_min, self.raw_max)
        return max(low, min(high, value))


class RealTimeArmController(object):
    def __init__(self, port, joystick_device, dry_run=False, tooling_enabled=True):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.port = port
        self.joystick_device = joystick_device
        self.dry_run = bool(dry_run)

        self.debug_log_path = os.path.join(self.base_dir, "gamepad_diagnostic.log")
        self.status_path = os.path.join(self.base_dir, "runtime_status.log")
        self.record_path = os.path.join(self.base_dir, "workspace_points.csv")
        self.debug_log_file = open(self.debug_log_path, "w", encoding="utf-8")
        self.debug_log_buffer = []
        self.last_debug_flush = time.time()
        self.debug_flush_interval = 0.15

        self.params = robot_params()
        self.robot = DeltaRobot(self.params)
        self.servo_ids = [1, 2, 3]
        self.servo_mappings = load_servo_mappings_for_ids(self.servo_ids)
        self.physical_angle_min_deg = self.params.servo_physical_angle_min_deg
        self.physical_angle_max_deg = self.params.servo_physical_angle_max_deg

        self.servo_logical_directions = {}
        self.servo_units_per_degree = {}
        self.reference_servo_positions = {}
        self.reference_servo_coords = {}
        self.servo_limits = {}
        for servo_id in self.servo_ids:
            mapping = self.servo_mappings[servo_id]
            raw_direction = SERVO_RAW_DIRECTIONS[servo_id]
            logical_sign = 1 if mapping.logical_span >= 0.0 else -1
            self.servo_logical_directions[servo_id] = raw_direction * logical_sign
            self.servo_units_per_degree[servo_id] = mapping.logical_units_per_degree(
                physical_min_deg=self.physical_angle_min_deg,
                physical_max_deg=self.physical_angle_max_deg,
            )
            self.reference_servo_positions[servo_id] = mapping.quantize_raw(mapping.raw_max)
            self.reference_servo_coords[servo_id] = mapping.raw_to_logical(
                self.reference_servo_positions[servo_id]
            )
            self.servo_limits[servo_id] = (mapping.raw_low, mapping.raw_high)

        self.driver = None
        if not self.dry_run:
            self.driver = BusServoDriver(
                port=port,
                baudrate=BAUDRATE,
                timeout=1.0,
                connect_delay=0.2,
                trace_hook=self.trace_packet,
            )
        self.gamepad = None

        self.reference_position = [float(value) for value in self.params.home_position]
        self.reference_angles = None
        self.current_position = self.reference_position[:]
        self.current_angles = None
        self.target_position = self.reference_position[:]
        self.target_angles = None

        self.current_servo_positions = self.reference_servo_positions.copy()
        self.command_servo_positions = self.reference_servo_positions.copy()
        self.target_servo_positions = self.reference_servo_positions.copy()
        self.last_sent_positions = self.reference_servo_positions.copy()

        self.update_rate = float(UPDATE_RATE_HZ)
        self.update_interval = 1.0 / self.update_rate
        self.speed_xy = float(SPEED_XY_MM_PER_SEC)
        self.speed_z = float(SPEED_Z_MM_PER_SEC)
        self.max_servo_speed_ticks_per_sec = float(MAX_SERVO_SPEED_TICKS_PER_SEC)
        self.min_effective_move_ticks = int(MIN_EFFECTIVE_MOVE_TICKS)
        self.min_command_time_ms = 20
        self.dpad_threshold = float(DPAD_THRESHOLD)
        self.dpad_slew_rate = float(DPAD_SLEW_RATE)
        self.motion_axes = [0.0, 0.0, 0.0]
        self.last_axes = (0.0, 0.0, 0.0)
        self.last_motion_axes = (0.0, 0.0, 0.0)
        self.last_buttons = {"a": False, "b": False, "x": False, "y": False, "lb": False, "rb": False}
        self.safe_scan_mode = "FREE"
        self.enforce_workspace_bounds = True

        self.last_feedback_poll_time = 0.0
        self.feedback_failure_count = 0
        self.feedback_failure_limit = 10
        self.battery_voltage_mv = None
        self.last_voltage_poll_time = 0.0
        self.last_send_time = None
        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        self.record_count = 0
        self.is_ready = False
        self.safety_fault_message = None

        self.tooling = ToolingServo(TOOLING_SERVO) if tooling_enabled and TOOLING_SERVO_ENABLED else None
        self.tooling_target_position = None
        self.last_sent_tooling_position = None

    def debug_log(self, message):
        line = "%0.6f %s %s\n" % (time.time(), datetime.now().isoformat(timespec="milliseconds"), message)
        self.debug_log_buffer.append(line)
        if len(self.debug_log_buffer) >= 64 or time.time() - self.last_debug_flush >= self.debug_flush_interval:
            self.flush_debug_log()

    def flush_debug_log(self):
        try:
            if self.debug_log_buffer:
                self.debug_log_file.writelines(self.debug_log_buffer)
                self.debug_log_buffer = []
            self.debug_log_file.flush()
            self.last_debug_flush = time.time()
        except Exception:
            pass

    def trace_packet(self, direction, packet, note):
        text = format_packet_hex(packet)
        self.debug_log("SERIAL %s %s %s" % (direction, note, text))

    def connect(self):
        if not self.dry_run:
            try:
                print("Opening serial %s @ %d..." % (self.port, BAUDRATE))
                self.driver.connect()
                print("Serial opened.")
            except Exception as exc:
                print("Serial open failed: %s" % exc)
                print(serial_permission_hint(self.port))
                return False
        else:
            print("DRY RUN: serial port is not opened.")

        try:
            self.gamepad = LinuxJoystickReader(self.joystick_device, threshold=self.dpad_threshold)
            print("Joystick opened: %s" % self.joystick_device)
        except Exception as exc:
            print("Joystick open failed: %s" % exc)
            print("If this is a permission problem, run:")
            print("  sudo chmod a+r %s" % self.joystick_device)
            print("or add the user to the input group and log in again.")
            return False
        return True

    def confirm_startup_pose(self):
        print("")
        print("Startup check:")
        print("  1. The arm must be in the mechanical safe reference pose.")
        print("  2. Servo 1/2/3 should be close to these raw positions:")
        for servo_id in self.servo_ids:
            print("     servo%d -> %d" % (servo_id, self.reference_servo_positions[servo_id]))
        print("  3. This program will read current raw positions before sending motion.")
        answer = prompt_input("Type YES to continue: ").strip()
        return answer == "YES"

    def init_reference_pose(self):
        angles, ok = inverse_kinematics(
            self.reference_position[0],
            self.reference_position[1],
            self.reference_position[2],
            self.params,
        )
        if not ok:
            print("Reference pose IK failed.")
            return False
        self.reference_angles = angles[:]
        self.current_angles = angles[:]
        self.target_angles = angles[:]
        self.current_position = self.reference_position[:]
        self.target_position = self.reference_position[:]
        print(
            "Reference XYZ=(%.1f, %.1f, %.1f), IK deg=(%.2f, %.2f, %.2f)"
            % (
                self.reference_position[0],
                self.reference_position[1],
                self.reference_position[2],
                math.degrees(angles[0]),
                math.degrees(angles[1]),
                math.degrees(angles[2]),
            )
        )
        return True

    def quantize_servo_position(self, servo_id, value):
        return self.servo_mappings[servo_id].quantize_raw(value)

    def servo_position_to_coord(self, servo_id, position):
        return self.servo_mappings[servo_id].raw_to_logical(position)

    def servo_positions_to_angles(self, servo_positions):
        if self.reference_angles is None:
            raise RuntimeError("reference angles are not initialized")
        angles = [0.0, 0.0, 0.0]
        for index, servo_id in enumerate(self.servo_ids):
            current_coord = self.servo_position_to_coord(servo_id, servo_positions[servo_id])
            delta_coord = current_coord - self.reference_servo_coords[servo_id]
            delta_deg = delta_coord / (
                self.servo_logical_directions[servo_id] * self.servo_units_per_degree[servo_id]
            )
            angles[index] = self.reference_angles[index] + math.radians(delta_deg)
        return angles

    def angles_to_servo_positions(self, angles):
        if self.reference_angles is None:
            raise RuntimeError("reference angles are not initialized")
        positions = {}
        for index, servo_id in enumerate(self.servo_ids):
            mapping = self.servo_mappings[servo_id]
            delta_deg = math.degrees(float(angles[index] - self.reference_angles[index]))
            target_coord = (
                self.reference_servo_coords[servo_id]
                + self.servo_logical_directions[servo_id]
                * delta_deg
                * self.servo_units_per_degree[servo_id]
            )
            raw = mapping.logical_to_raw(target_coord)
            low, high = self.servo_limits[servo_id]
            positions[servo_id] = max(low, min(high, raw))
        return positions

    def sync_servo_feedback(self, force=False, strict=False):
        now = time.time()
        if not force and now - self.last_feedback_poll_time < FEEDBACK_INTERVAL_SEC:
            return True
        self.last_feedback_poll_time = now

        if self.dry_run:
            feedback_positions = self.command_servo_positions.copy()
        else:
            try:
                timeout = STARTUP_FEEDBACK_TIMEOUT_SEC if force or strict else FEEDBACK_TIMEOUT_SEC
                feedback_positions = self.driver.read_servo_positions(self.servo_ids, timeout=timeout)
                feedback_positions = {
                    servo_id: self.quantize_servo_position(servo_id, int(feedback_positions[servo_id]))
                    for servo_id in self.servo_ids
                }
                if now - self.last_voltage_poll_time >= 1.0:
                    self.last_voltage_poll_time = now
                    try:
                        self.battery_voltage_mv = self.driver.get_battery_voltage_mv(timeout=FEEDBACK_TIMEOUT_SEC)
                    except Exception:
                        pass
            except Exception as exc:
                self.feedback_failure_count += 1
                self.debug_log("FEEDBACK_ERROR count=%d error=%s" % (self.feedback_failure_count, exc))
                if strict or self.feedback_failure_count >= self.feedback_failure_limit:
                    self.safety_fault_message = "feedback read failed: %s" % exc
                    return False
                return True

        self.feedback_failure_count = 0
        self.current_servo_positions = feedback_positions
        self.current_angles = self.servo_positions_to_angles(feedback_positions)
        pose, ok = forward_kinematics(
            self.current_angles[0],
            self.current_angles[1],
            self.current_angles[2],
            self.params,
        )
        if ok:
            self.current_position = pose
        elif strict:
            self.safety_fault_message = "feedback angles cannot be converted to a valid pose"
            return False
        return True

    def confirm_and_init(self):
        if not self.confirm_startup_pose():
            print("Cancelled.")
            return False
        if not self.init_reference_pose():
            return False
        if not self.sync_servo_feedback(force=True, strict=True):
            print(self.safety_fault_message or "feedback read failed")
            return False

        mismatches = []
        for servo_id in self.servo_ids:
            actual = self.current_servo_positions[servo_id]
            expected = self.reference_servo_positions[servo_id]
            if abs(actual - expected) > STARTUP_TOLERANCE_TICKS:
                mismatches.append((servo_id, actual, expected))
        if mismatches:
            print("Startup refused: servo feedback is not near the reference pose.")
            for servo_id, actual, expected in mismatches:
                print("  servo%d: actual=%d expected=%d" % (servo_id, actual, expected))
            print("Use servo_calibration.py first, then run this controller again.")
            return False

        self.target_position = self.current_position[:]
        self.target_angles = self.current_angles[:]
        self.target_servo_positions = self.current_servo_positions.copy()
        self.command_servo_positions = self.current_servo_positions.copy()
        self.last_sent_positions = self.current_servo_positions.copy()
        self.last_send_time = time.time()
        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        if self.tooling is not None:
            self.tooling_target_position = self.tooling.center_raw
            self.last_sent_tooling_position = self.tooling_target_position
        self.is_ready = True
        self.write_runtime_status()
        print("Ready.")
        return True

    def _axis_to_dpad(self, value):
        if abs(value) < self.dpad_threshold:
            return 0.0
        return 1.0 if value > 0.0 else -1.0

    def smooth_axes(self, target_axes):
        target = [float(value) for value in target_axes]
        max_step = self.dpad_slew_rate * self.update_interval
        for index in range(3):
            delta = clamp(target[index] - self.motion_axes[index], -max_step, max_step)
            self.motion_axes[index] += delta
            if abs(self.motion_axes[index]) < 0.01:
                self.motion_axes[index] = 0.0
        return tuple(float(value) for value in self.motion_axes)

    def cycle_safe_scan_mode(self):
        modes = ["FREE", "X", "Y", "Z"]
        index = modes.index(self.safe_scan_mode)
        self.safe_scan_mode = modes[(index + 1) % len(modes)]
        print("safe scan -> %s" % self.safe_scan_mode)
        self.write_runtime_status()

    def update_tooling_from_buttons(self, buttons):
        if self.tooling is None:
            return False
        if self.tooling_target_position is None:
            self.tooling_target_position = self.tooling.center_raw
        direction = 0
        if buttons.get("rb", False):
            direction += 1
        if buttons.get("lb", False):
            direction -= 1
        if direction == 0:
            return False
        delta = direction * self.tooling.speed_ticks_per_sec * self.update_interval
        next_position = self.tooling.clamp(self.tooling_target_position + delta)
        if next_position == self.tooling_target_position:
            return False
        self.tooling_target_position = next_position
        return True

    def record_current_point(self):
        self.record_count += 1
        file_exists = os.path.exists(self.record_path)
        row = {
            "index": self.record_count,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "safe_scan_mode": self.safe_scan_mode,
            "x_mm": round(float(self.current_position[0]), 3),
            "y_mm": round(float(self.current_position[1]), 3),
            "z_mm": round(float(self.current_position[2]), 3),
            "target_x_mm": round(float(self.target_position[0]), 3),
            "target_y_mm": round(float(self.target_position[1]), 3),
            "target_z_mm": round(float(self.target_position[2]), 3),
            "servo1": self.current_servo_positions[1],
            "servo2": self.current_servo_positions[2],
            "servo3": self.current_servo_positions[3],
        }
        with open(self.record_path, "a", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=list(row.keys()))
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)
        print("recorded point #%d" % self.record_count)

    def update_from_gamepad(self):
        dpad_x, dpad_y, right_y, buttons = self.gamepad.read()
        self.last_axes = (dpad_x, dpad_y, right_y)
        previous_buttons = self.last_buttons.copy()
        self.last_buttons = buttons.copy()

        if buttons.get("a", False):
            return False, False
        if buttons.get("b", False) and not previous_buttons.get("b", False):
            self.record_current_point()
        if buttons.get("x", False) and not previous_buttons.get("x", False):
            self.cycle_safe_scan_mode()

        tooling_changed = self.update_tooling_from_buttons(buttons)
        target_axes = (dpad_x, dpad_y, self._axis_to_dpad(right_y))
        motion_x, motion_y, motion_z = self.smooth_axes(target_axes)
        self.last_motion_axes = (motion_x, motion_y, motion_z)

        if max(abs(motion_x), abs(motion_y), abs(motion_z)) < 0.01:
            return True, tooling_changed

        new_position = self.target_position[:]
        delta_x = motion_x * self.speed_xy * self.update_interval
        delta_y = -motion_y * self.speed_xy * self.update_interval
        delta_z = -motion_z * self.speed_z * self.update_interval

        if self.safe_scan_mode == "X":
            delta_y = 0.0
            delta_z = 0.0
        elif self.safe_scan_mode == "Y":
            delta_x = 0.0
            delta_z = 0.0
        elif self.safe_scan_mode == "Z":
            delta_x = 0.0
            delta_y = 0.0

        new_position[0] += delta_x
        new_position[1] += delta_y
        new_position[2] += delta_z

        if self.enforce_workspace_bounds:
            bounds = self.robot.get_workspace_bounds()
            new_position[0] = clamp(new_position[0], bounds["x_min"], bounds["x_max"])
            new_position[1] = clamp(new_position[1], bounds["y_min"], bounds["y_max"])
            new_position[2] = clamp(new_position[2], bounds["z_min"], bounds["z_max"])

        angles, ok = inverse_kinematics(new_position[0], new_position[1], new_position[2], self.params)
        if not ok:
            self.debug_log("IK_FAIL xyz=(%.3f,%.3f,%.3f)" % (new_position[0], new_position[1], new_position[2]))
            return True, tooling_changed

        self.target_position = new_position
        self.target_angles = angles
        self.target_servo_positions = self.angles_to_servo_positions(angles)
        return True, True

    def compute_next_servo_command(self):
        if self.target_angles is None:
            return self.command_servo_positions.copy(), self.min_command_time_ms

        desired_positions = self.angles_to_servo_positions(self.target_angles)
        self.target_servo_positions = desired_positions
        now = time.time()
        if self.last_send_time is None:
            self.last_send_time = now
        elapsed = max(now - self.last_send_time, self.update_interval)
        self.last_send_time = now

        next_positions = {}
        max_move_ticks = 0
        for servo_id in self.servo_ids:
            self.servo_tick_budget[servo_id] += self.max_servo_speed_ticks_per_sec * elapsed
            current = self.command_servo_positions[servo_id]
            desired = desired_positions[servo_id]
            error = desired - current
            available = int(self.servo_tick_budget[servo_id])

            if error == 0:
                next_position = current
                self.servo_tick_budget[servo_id] = 0.0
            elif available < self.min_effective_move_ticks:
                next_position = current
            else:
                move_ticks = min(abs(error), available)
                direction = 1 if error > 0 else -1
                next_position = current + direction * move_ticks
                self.servo_tick_budget[servo_id] -= move_ticks
                max_move_ticks = max(max_move_ticks, move_ticks)

            low, high = self.servo_limits[servo_id]
            next_positions[servo_id] = max(low, min(high, int(next_position)))

        if max_move_ticks:
            time_ms = max(self.min_command_time_ms, int((max_move_ticks / self.max_servo_speed_ticks_per_sec) * 1000))
        else:
            time_ms = self.min_command_time_ms
        return next_positions, time_ms

    def send_servo_positions(self):
        if not self.is_ready:
            return False
        next_positions, time_ms = self.compute_next_servo_command()
        tooling_changed = (
            self.tooling is not None
            and self.tooling_target_position is not None
            and self.tooling_target_position != self.last_sent_tooling_position
        )
        if next_positions == self.last_sent_positions and not tooling_changed:
            return True

        targets = [(servo_id, next_positions[servo_id]) for servo_id in self.servo_ids]
        if self.tooling is not None and self.tooling_target_position is not None:
            targets.append((self.tooling.servo_id, self.tooling_target_position))

        try:
            if self.dry_run:
                print("DRY targets=%s time_ms=%d" % (targets, time_ms))
            else:
                self.driver.set_servo_positions(targets, time_ms)
            self.command_servo_positions = next_positions.copy()
            self.last_sent_positions = next_positions.copy()
            if self.tooling is not None:
                self.last_sent_tooling_position = self.tooling_target_position
            return True
        except Exception as exc:
            self.safety_fault_message = "send failed: %s" % exc
            return False

    def build_status_snapshot(self):
        lines = [
            "arm_min_py36 runtime status",
            "time: %s" % datetime.now().isoformat(timespec="seconds"),
            "port: %s" % self.port,
            "joystick: %s" % self.joystick_device,
            "dry_run: %s" % self.dry_run,
            "safe_scan: %s" % self.safe_scan_mode,
            "target_xyz_mm: %.3f %.3f %.3f"
            % (self.target_position[0], self.target_position[1], self.target_position[2]),
            "feedback_xyz_mm: %.3f %.3f %.3f"
            % (self.current_position[0], self.current_position[1], self.current_position[2]),
            "feedback_raw: 1=%d 2=%d 3=%d"
            % (
                self.current_servo_positions[1],
                self.current_servo_positions[2],
                self.current_servo_positions[3],
            ),
            "target_raw: 1=%d 2=%d 3=%d"
            % (
                self.target_servo_positions[1],
                self.target_servo_positions[2],
                self.target_servo_positions[3],
            ),
            "axes: dx=%+.2f dy=%+.2f rz=%+.2f" % self.last_axes,
            "motion_axes: dx=%+.2f dy=%+.2f rz=%+.2f" % self.last_motion_axes,
            "battery_mv: %s" % (self.battery_voltage_mv if self.battery_voltage_mv is not None else ""),
        ]
        if self.tooling is not None:
            lines.append(
                "tooling: id=%d target=%s"
                % (self.tooling.servo_id, self.tooling_target_position)
            )
        if self.safety_fault_message:
            lines.append("fault: %s" % self.safety_fault_message)
        return "\n".join(lines) + "\n"

    def write_runtime_status(self):
        try:
            with open(self.status_path, "w", encoding="utf-8") as status_file:
                status_file.write(self.build_status_snapshot())
        except Exception:
            pass

    def run(self):
        if not self.connect():
            self.cleanup()
            return
        if not self.confirm_and_init():
            self.cleanup()
            return

        print("")
        print("Controls:")
        print("  D-pad/left stick: X/Y")
        print("  right stick Y: Z")
        print("  A: quit")
        print("  B: record current point")
        print("  X: switch safe scan FREE/X/Y/Z")
        print("  LB/RB: optional servo4 tooling")
        print("Status file: %s" % self.status_path)

        try:
            last_status = 0.0
            while True:
                if not self.sync_servo_feedback():
                    print(self.safety_fault_message or "feedback sync failed")
                    break
                keep_running, _changed = self.update_from_gamepad()
                if not keep_running:
                    print("Quit.")
                    break
                if not self.send_servo_positions():
                    print(self.safety_fault_message or "send failed")
                    break
                now = time.time()
                if now - last_status >= 0.2:
                    self.write_runtime_status()
                    last_status = now
                time.sleep(self.update_interval)
        except KeyboardInterrupt:
            print("Interrupted.")
        finally:
            self.cleanup()

    def cleanup(self):
        try:
            if self.gamepad is not None:
                self.gamepad.close()
        except Exception:
            pass
        try:
            if self.driver is not None:
                self.driver.close()
        except Exception:
            pass
        try:
            self.flush_debug_log()
            self.debug_log_file.close()
        except Exception:
            pass


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Minimal Python 3.6 realtime 78arm controller.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device, default: %(default)s")
    parser.add_argument("--joystick", default=DEFAULT_JOYSTICK, help="Linux joystick device, default: %(default)s")
    parser.add_argument("--dry-run", action="store_true", help="do not open serial, only compute commands")
    parser.add_argument("--no-tooling", action="store_true", help="disable optional servo4 LB/RB control")
    parser.add_argument("--speed-xy", type=float, default=SPEED_XY_MM_PER_SEC)
    parser.add_argument("--speed-z", type=float, default=SPEED_Z_MM_PER_SEC)
    parser.add_argument("--servo-speed", type=float, default=MAX_SERVO_SPEED_TICKS_PER_SEC)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    controller = RealTimeArmController(
        port=args.port,
        joystick_device=args.joystick,
        dry_run=args.dry_run,
        tooling_enabled=not args.no_tooling,
    )
    controller.speed_xy = float(args.speed_xy)
    controller.speed_z = float(args.speed_z)
    controller.max_servo_speed_ticks_per_sec = float(args.servo_speed)
    controller.run()


if __name__ == "__main__":
    main()
