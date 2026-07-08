#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dry-run or execute a conservative wrench grasp sequence on the Jetson mainline.

Python 3.6 compatible. By default this script does not open the servo serial
port. It reads the grasp sequence JSON produced by
Dual_Camera_HandEye/tools/plan_wrench_grasp_sequence.py, checks each waypoint
with the same Jetson IK/raw mapping used by the sampler, and prints the command
plan. Real motion requires --execute and a typed GRASP confirmation.
"""

from __future__ import print_function

import argparse
import math
import os
import signal
import time

from jetson_workspace_common import (
    DEFAULT_SERVO_CONFIG,
    PROJECT_ROOT,
    ServoMapper,
    forward_kinematics,
    inverse_kinematics,
    open_servo_driver,
    read_json,
    write_json,
)


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600
DEFAULT_SEQUENCE_JSON = os.path.join(
    PROJECT_ROOT,
    "Dual_Camera_HandEye",
    "output",
    "wrench_grasp_sequence_latest.json",
)
DEFAULT_STATUS_JSON = os.path.join(
    PROJECT_ROOT,
    "Delta_Gcode_Servo",
    "real_machine_test",
    "jetson_py36",
    "wrench_grasp_demo_status.json",
)


def clamp(value, low, high):
    return max(low, min(high, value))


def resolve_project_path(path):
    if os.path.isabs(path):
        return path
    return os.path.join(PROJECT_ROOT, path)


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def waypoint(name, x, y, z, gripper, speed):
    return {
        "name": name,
        "position_mm": {
            "x": float(x),
            "y": float(y),
            "z": float(z),
        },
        "gripper": gripper,
        "speed_mm_s": float(speed),
    }


def clamp_xy_radius(x, y, limit):
    radius = math.sqrt(x * x + y * y)
    if limit > 0.0 and radius > limit and radius > 1e-9:
        scale = limit / radius
        return x * scale, y * scale
    return x, y


def build_manual_target_sequence(args):
    target = args.target_xyz_mm
    x = float(target[0]) + float(args.grasp_offset_x_mm)
    y = float(target[1]) + float(args.grasp_offset_y_mm)
    z = float(target[2]) + float(args.grasp_offset_z_mm)
    x, y = clamp_xy_radius(x, y, float(args.xy_limit_mm))
    z = clamp(z, float(args.z_min_mm), float(args.z_max_mm))
    approach_z = clamp(z + float(args.approach_height_mm), float(args.z_min_mm), float(args.z_max_mm))
    lift_z = clamp(z + float(args.lift_height_mm), float(args.z_min_mm), float(args.z_max_mm))
    return {
        "valid": True,
        "status": "manual_target_planned",
        "timestamp": time.time(),
        "source": "manual_target_xyz_mm",
        "object": {
            "class": "manual",
            "position_base_mm": {"x": x, "y": y, "z": z},
        },
        "sequence": [
            waypoint("home", args.home_x_mm, args.home_y_mm, args.home_z_mm, "open", args.travel_speed_mm_s),
            waypoint("pregrasp", x, y, approach_z, "open", args.travel_speed_mm_s),
            waypoint("approach", x, y, z, "open", args.approach_speed_mm_s),
            waypoint("grasp", x, y, z, "close", args.grasp_speed_mm_s),
            waypoint("lift", x, y, lift_z, "close", args.travel_speed_mm_s),
            waypoint("return_home", args.home_x_mm, args.home_y_mm, args.home_z_mm, "hold", args.travel_speed_mm_s),
        ],
    }


def load_sequence_payload(args):
    if args.target_xyz_mm:
        return build_manual_target_sequence(args), "manual_target_xyz_mm"
    path = resolve_project_path(args.sequence_json)
    if not os.path.exists(path):
        raise RuntimeError("sequence JSON missing: %s" % path)
    payload = read_json(path)
    if not payload.get("valid"):
        raise RuntimeError("sequence JSON is not valid: status=%s" % payload.get("status"))
    timestamp = payload.get("timestamp")
    if isinstance(timestamp, (int, float)) and float(args.max_sequence_age_sec) > 0:
        age = time.time() - float(timestamp)
        if age > float(args.max_sequence_age_sec) and not args.allow_stale_sequence:
            raise RuntimeError(
                "sequence JSON is stale: %.2fs > %.2fs; rerun planner or use --allow-stale-sequence"
                % (age, float(args.max_sequence_age_sec))
            )
    sequence = payload.get("sequence")
    if not isinstance(sequence, list) or not sequence:
        raise RuntimeError("sequence JSON has no waypoints")
    return payload, path


def read_position_mm(item):
    position = item.get("position_mm")
    if isinstance(position, dict):
        x = finite_float(position.get("x"))
        y = finite_float(position.get("y"))
        z = finite_float(position.get("z"))
    elif isinstance(position, (list, tuple)) and len(position) == 3:
        x = finite_float(position[0])
        y = finite_float(position[1])
        z = finite_float(position[2])
    else:
        x = y = z = None
    if x is None or y is None or z is None:
        raise RuntimeError("bad waypoint position: %r" % (position,))
    return [x, y, z]


class WrenchGraspDemo(object):
    def __init__(self, args):
        self.args = args
        self.mapper = ServoMapper(args.servo_config)
        self.servo_ids = list(self.mapper.servo_ids)
        self.driver = None
        self.running = True
        self.home_raw = dict(self.mapper.reference_raw)
        self.current_raw = dict(self.home_raw)
        self.command_raw = dict(self.home_raw)
        self.current_angles = self.mapper.raw_to_angles(self.current_raw)
        self.current_position = [0.0, 0.0, 240.0]
        self.target_raw = dict(self.home_raw)
        self.gripper_raw = None
        self.status = {}

    def close(self):
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception:
                pass

    def connect(self):
        print("Opening servo %s @ %d" % (self.args.port, self.args.baudrate))
        self.driver = open_servo_driver(self.args.port, self.args.baudrate)

    def clamp_raw_to_motion_limits(self, raw_values):
        limited = {}
        for servo_id in self.servo_ids:
            item = self.mapper.mappings[servo_id]
            low = min(int(item["raw_min"]), int(item["raw_max"]))
            high = max(int(item["raw_min"]), int(item["raw_max"]))
            value = int(raw_values[servo_id])
            value = int(clamp(value, low, high))
            limited[servo_id] = min(value, int(self.home_raw[servo_id]))
        return limited

    def raw_range_violations(self):
        return self.mapper.raw_range_violations(self.current_raw, margin_ticks=self.args.raw_range_margin)

    def startup_errors(self):
        return self.mapper.startup_check_errors(self.current_raw)

    def at_startup(self):
        errors = self.startup_errors()
        return max(abs(int(value)) for value in errors.values()) <= int(self.args.home_tolerance)

    def read_feedback(self, strict=True):
        last_error = None
        attempts = max(1, int(self.args.feedback_read_retries))
        for attempt in range(attempts):
            try:
                raw = self.driver.read_servo_positions(self.servo_ids, timeout=self.args.servo_timeout)
                self.current_raw = {servo_id: int(raw[servo_id]) for servo_id in self.servo_ids}
                self.current_angles = self.mapper.raw_to_angles(self.current_raw)
                xyz, ok = forward_kinematics(
                    self.current_angles[0],
                    self.current_angles[1],
                    self.current_angles[2],
                )
                if ok:
                    self.current_position = list(xyz)
                elif strict:
                    raise RuntimeError("feedback raw cannot be converted by FK")
                return True
            except Exception as exc:
                last_error = exc
                print("FEEDBACK_RETRY %d/%d %s" % (attempt + 1, attempts, exc))
                time.sleep(0.03)
        if strict:
            print("FEEDBACK_FAILED %s" % last_error)
            return False
        return True

    def move_to_motion_home(self):
        target_raw = dict(self.home_raw)
        current_raw = dict(self.current_raw)
        max_error = max(abs(int(target_raw[servo_id]) - int(current_raw[servo_id])) for servo_id in self.servo_ids)
        if max_error <= int(self.args.home_tolerance):
            print("Already near motion mapping home raw.")
            self.command_raw = dict(target_raw)
            return True
        time_ms = max(500, int((max_error / max(1.0, float(self.args.startup_home_raw_s))) * 1000.0))
        print(
            "HOME move: raw 1=%d 2=%d 3=%d time=%d ms"
            % (target_raw[1], target_raw[2], target_raw[3], time_ms)
        )
        self.driver.set_servo_positions([(servo_id, target_raw[servo_id]) for servo_id in self.servo_ids], time_ms)
        time.sleep(min(8.0, time_ms / 1000.0 + 0.5))
        if not self.read_feedback(strict=True):
            return False
        errors = self.mapper.home_errors(self.current_raw)
        max_after = max(abs(int(errors[servo_id])) for servo_id in self.servo_ids)
        print("HOME result errors: 1=%+d 2=%+d 3=%+d" % (errors[1], errors[2], errors[3]))
        if max_after > int(self.args.home_tolerance):
            print("Startup HOME did not reach configured motion home within tolerance.")
            return False
        self.command_raw = dict(target_raw)
        return True

    def startup_check(self):
        startup_raw = self.mapper.startup_check_raw
        print("")
        print("Startup safety check")
        print(
            "  Expected startup raw: 1=%d 2=%d 3=%d"
            % (startup_raw[1], startup_raw[2], startup_raw[3])
        )
        print("  Motion mapping home: 1=%d 2=%d 3=%d" % (self.home_raw[1], self.home_raw[2], self.home_raw[3]))
        if not self.read_feedback(strict=True):
            return False
        errors = self.startup_errors()
        print(
            "  Current raw:          1=%d 2=%d 3=%d"
            % (self.current_raw[1], self.current_raw[2], self.current_raw[3])
        )
        print("  Startup error ticks:  1=%+d 2=%+d 3=%+d" % (errors[1], errors[2], errors[3]))
        violations = self.raw_range_violations()
        if violations:
            print("Startup refused: servo feedback is outside configured raw range.")
            for servo_id in self.servo_ids:
                if servo_id in violations:
                    item = violations[servo_id]
                    print(
                        "  servo %d raw=%d outside [%d,%d] +/- %d"
                        % (
                            servo_id,
                            item["raw"],
                            item["configured_raw_min"],
                            item["configured_raw_max"],
                            self.args.raw_range_margin,
                        )
                    )
            return False
        if not self.at_startup():
            if self.args.allow_not_home:
                print("Continuing because --allow-not-home was set.")
            else:
                print("Arm is not near startup_check_raw.")
                answer = input("Type HOME to slowly move to configured home_raw: ").strip()
                if answer != "HOME":
                    print("Startup HOME cancelled.")
                    return False
                if not self.move_to_motion_home():
                    return False
        self.command_raw = self.clamp_raw_to_motion_limits(self.current_raw)
        return True

    def normalize_waypoint(self, item, index):
        name = str(item.get("name", "wp_%02d" % index))
        position = read_position_mm(item)
        speed = finite_float(item.get("speed_mm_s", self.args.travel_speed_mm_s))
        if speed is None or speed <= 0.0:
            speed = float(self.args.travel_speed_mm_s)
        gripper = str(item.get("gripper", "hold")).strip().lower()
        if gripper not in ("open", "close", "hold", "none"):
            raise RuntimeError("bad gripper state %r at %s" % (gripper, name))
        radius = math.sqrt(position[0] * position[0] + position[1] * position[1])
        if float(self.args.xy_limit_mm) > 0 and radius > float(self.args.xy_limit_mm) + 1e-6:
            raise RuntimeError("%s XY radius %.2f exceeds %.2f" % (name, radius, float(self.args.xy_limit_mm)))
        if position[2] < float(self.args.z_min_mm) or position[2] > float(self.args.z_max_mm):
            raise RuntimeError(
                "%s z %.2f outside [%.2f, %.2f]"
                % (name, position[2], float(self.args.z_min_mm), float(self.args.z_max_mm))
            )
        angles, ok = inverse_kinematics(position[0], position[1], position[2])
        if not ok:
            raise RuntimeError("%s IK failed at %.2f %.2f %.2f" % (name, position[0], position[1], position[2]))
        is_home_position = (
            abs(position[0] - float(self.args.home_x_mm)) < 1e-6
            and abs(position[1] - float(self.args.home_y_mm)) < 1e-6
            and abs(position[2] - float(self.args.home_z_mm)) < 1e-6
        )
        if name in ("home", "return_home") and is_home_position:
            raw = dict(self.home_raw)
        else:
            raw = self.clamp_raw_to_motion_limits(self.mapper.angles_to_raw(angles))
        raw_angles = self.mapper.raw_to_angles(raw)
        raw_xyz, fk_ok = forward_kinematics(raw_angles[0], raw_angles[1], raw_angles[2])
        if not fk_ok or raw_xyz[2] < float(self.args.z_min_mm):
            raise RuntimeError("%s raw command violates z floor" % name)
        return {
            "name": name,
            "position_mm": position,
            "speed_mm_s": speed,
            "gripper": gripper,
            "arm_raw": raw,
            "fk_after_raw_mm": raw_xyz,
        }

    def gripper_target_raw(self, state):
        if state in ("hold", "none"):
            return None
        if self.args.gripper_mode == "none":
            return None
        servo_id = int(self.args.gripper_servo_id)
        if servo_id not in self.mapper.mappings:
            raise RuntimeError("gripper servo id %d missing from servo config" % servo_id)
        if state == "open":
            raw = self.args.gripper_open_raw
        elif state == "close":
            raw = self.args.gripper_close_raw
        else:
            raw = None
        if raw is None:
            raise RuntimeError(
                "gripper state %s needs --gripper-%s-raw when --gripper-mode=servo4"
                % (state, state)
            )
        item = self.mapper.mappings[servo_id]
        low = min(int(item["raw_min"]), int(item["raw_max"]))
        high = max(int(item["raw_min"]), int(item["raw_max"]))
        return int(clamp(int(raw), low, high))

    def build_plan(self, payload):
        plan = []
        previous_position = [float(self.args.home_x_mm), float(self.args.home_y_mm), float(self.args.home_z_mm)]
        previous_raw = dict(self.home_raw)
        for index, item in enumerate(payload.get("sequence") or []):
            command = self.normalize_waypoint(item, index)
            grip_raw = self.gripper_target_raw(command["gripper"])
            distance = math.sqrt(sum((command["position_mm"][i] - previous_position[i]) ** 2 for i in range(3)))
            xyz_time = distance / max(1e-6, float(command["speed_mm_s"]))
            max_raw_delta = max(abs(int(command["arm_raw"][servo_id]) - int(previous_raw[servo_id])) for servo_id in self.servo_ids)
            raw_time = max_raw_delta / max(1.0, float(self.args.max_servo_raw_s))
            move_time_ms = max(int(self.args.min_move_ms), int(max(xyz_time, raw_time) * 1000.0))
            if grip_raw is not None and grip_raw != self.gripper_raw:
                move_time_ms = max(move_time_ms, int(self.args.gripper_move_ms))
            command["gripper_raw"] = grip_raw
            command["time_ms"] = move_time_ms
            plan.append(command)
            previous_position = list(command["position_mm"])
            previous_raw = dict(command["arm_raw"])
        if not plan:
            raise RuntimeError("empty command plan")
        return plan

    def print_plan(self, payload, plan):
        print("")
        print("Wrench grasp plan")
        print("  source: %s" % payload.get("source"))
        if payload.get("object"):
            print("  object: %s" % payload.get("object"))
        for index, command in enumerate(plan):
            grip = command["gripper"]
            if command["gripper_raw"] is not None:
                grip = "%s/%d" % (grip, command["gripper_raw"])
            print(
                "  %02d %-12s xyz=(%7.2f,%7.2f,%7.2f) raw=(%4d,%4d,%4d) gripper=%s time=%dms"
                % (
                    index + 1,
                    command["name"],
                    command["position_mm"][0],
                    command["position_mm"][1],
                    command["position_mm"][2],
                    command["arm_raw"][1],
                    command["arm_raw"][2],
                    command["arm_raw"][3],
                    grip,
                    command["time_ms"],
                )
            )

    def write_status(self, payload, plan, state, message):
        status_path = resolve_project_path(self.args.status_json)
        out = {
            "timestamp_unix": time.time(),
            "state": state,
            "message": message,
            "execute": bool(self.args.execute),
            "sequence_source": payload.get("source"),
            "plan": plan,
        }
        write_json(status_path, out)
        print("status_json=%s" % status_path)

    def send_command(self, command):
        targets = [(servo_id, int(command["arm_raw"][servo_id])) for servo_id in self.servo_ids]
        if command["gripper_raw"] is not None:
            targets.append((int(self.args.gripper_servo_id), int(command["gripper_raw"])))
        self.driver.set_servo_positions(targets, int(command["time_ms"]))
        self.command_raw = dict(command["arm_raw"])
        if command["gripper_raw"] is not None:
            self.gripper_raw = int(command["gripper_raw"])

    def execute_plan(self, payload, plan):
        self.connect()
        if not self.startup_check():
            return 2
        if not self.args.no_confirm:
            print("")
            print("This will send real servo commands for the wrench grasp demo.")
            answer = input("Type GRASP to execute the planned sequence: ").strip()
            if answer != "GRASP":
                print("Cancelled.")
                return 1
        for command in plan:
            if not self.running:
                print("Stopped before %s" % command["name"])
                return 130
            print(
                "SEND %-12s raw=(%d,%d,%d) gripper=%s time=%dms"
                % (
                    command["name"],
                    command["arm_raw"][1],
                    command["arm_raw"][2],
                    command["arm_raw"][3],
                    command["gripper"],
                    command["time_ms"],
                )
            )
            self.send_command(command)
            time.sleep(float(command["time_ms"]) / 1000.0 + float(self.args.settle_sec))
            if not self.read_feedback(strict=True):
                return 2
        print("GRASP_SEQUENCE_DONE")
        return 0

    def run(self):
        payload, source = load_sequence_payload(self.args)
        payload["source"] = source
        plan = self.build_plan(payload)
        self.print_plan(payload, plan)
        if not self.args.execute:
            self.write_status(payload, plan, "dry_run_ok", "validated plan without opening serial")
            print("")
            print("Dry-run only. Add --execute and type GRASP on the Jetson to move the real arm.")
            return 0
        rc = self.execute_plan(payload, plan)
        self.write_status(payload, plan, "done" if rc == 0 else "failed", "rc=%d" % rc)
        return rc


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-json", default=DEFAULT_SEQUENCE_JSON)
    parser.add_argument("--status-json", default=DEFAULT_STATUS_JSON)
    parser.add_argument("--target-xyz-mm", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--servo-timeout", type=float, default=0.20)
    parser.add_argument("--feedback-read-retries", type=int, default=3)
    parser.add_argument("--servo-config", default=DEFAULT_SERVO_CONFIG)
    parser.add_argument("--execute", action="store_true", help="Open serial and execute the sequence after typed confirmation.")
    parser.add_argument("--no-confirm", action="store_true")
    parser.add_argument("--allow-not-home", action="store_true")
    parser.add_argument("--allow-stale-sequence", action="store_true")
    parser.add_argument("--max-sequence-age-sec", type=float, default=2.0)
    parser.add_argument("--home-tolerance", type=int, default=30)
    parser.add_argument("--raw-range-margin", type=int, default=30)
    parser.add_argument("--startup-home-raw-s", type=float, default=120.0)
    parser.add_argument("--max-servo-raw-s", type=float, default=80.0)
    parser.add_argument("--min-move-ms", type=int, default=350)
    parser.add_argument("--settle-sec", type=float, default=0.20)
    parser.add_argument("--xy-limit-mm", type=float, default=115.0)
    parser.add_argument("--z-min-mm", type=float, default=155.0)
    parser.add_argument("--z-max-mm", type=float, default=263.0)
    parser.add_argument("--home-x-mm", type=float, default=0.0)
    parser.add_argument("--home-y-mm", type=float, default=0.0)
    parser.add_argument("--home-z-mm", type=float, default=240.0)
    parser.add_argument("--grasp-offset-x-mm", type=float, default=0.0)
    parser.add_argument("--grasp-offset-y-mm", type=float, default=0.0)
    parser.add_argument("--grasp-offset-z-mm", type=float, default=0.0)
    parser.add_argument("--approach-height-mm", type=float, default=25.0)
    parser.add_argument("--lift-height-mm", type=float, default=35.0)
    parser.add_argument("--travel-speed-mm-s", type=float, default=35.0)
    parser.add_argument("--approach-speed-mm-s", type=float, default=18.0)
    parser.add_argument("--grasp-speed-mm-s", type=float, default=8.0)
    parser.add_argument("--gripper-mode", choices=("none", "servo4"), default="none")
    parser.add_argument("--gripper-servo-id", type=int, default=4)
    parser.add_argument("--gripper-open-raw", type=int)
    parser.add_argument("--gripper-close-raw", type=int)
    parser.add_argument("--gripper-move-ms", type=int, default=500)
    return parser.parse_args()


def main():
    args = parse_args()
    demo = WrenchGraspDemo(args)

    def stop(_signum, _frame):
        demo.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        return demo.run()
    finally:
        demo.close()


if __name__ == "__main__":
    raise SystemExit(main())
