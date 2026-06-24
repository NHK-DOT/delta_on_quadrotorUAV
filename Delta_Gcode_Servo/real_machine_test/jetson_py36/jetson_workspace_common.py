#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python 3.6 helpers for Jetson AprilTag workspace sampling."""

from __future__ import print_function

import json
import math
import os
import sys
import time
from datetime import datetime


HERE = os.path.dirname(os.path.abspath(__file__))
REAL_MACHINE_DIR = os.path.dirname(HERE)
DELTA_GCODE_SERVO_DIR = os.path.dirname(REAL_MACHINE_DIR)
PROJECT_ROOT = os.path.dirname(DELTA_GCODE_SERVO_DIR)
BT_8BITDO_SRC = os.path.join(PROJECT_ROOT, "bt_8bitdo_min", "src")
if BT_8BITDO_SRC not in sys.path:
    sys.path.insert(0, BT_8BITDO_SRC)

DEFAULT_APRILTAG_JSON = "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json"
DEFAULT_APRILTAG_LAUNCH = "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench/run_fullfov_1280x960_gui.sh"
DEFAULT_SERVO_CONFIG = os.path.join(PROJECT_ROOT, "lx225_tool_demo", "config", "lx225_tool.demo.toml")
DEFAULT_GAMEPAD_CONFIG = os.path.join(PROJECT_ROOT, "bt_8bitdo_min", "config", "gamepad_8bitdo_bt.json")
DEFAULT_CALIBRATION = os.path.join(PROJECT_ROOT, "Dual_Camera_HandEye", "output", "calibration_result.json")


def now_iso():
    return datetime.now().isoformat(timespec="seconds")


def read_json(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def write_json(path, payload):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")


def append_jsonl(path, payload):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        fh.write("\n")


def snapshot_age_ms(path):
    try:
        payload = read_json(path)
    except Exception:
        return None
    timestamp = payload.get("timestamp_unix")
    if not isinstance(timestamp, (int, float)):
        return None
    return max(0.0, (time.time() - float(timestamp)) * 1000.0)


def parse_scalar(text):
    text = str(text).strip()
    if not text:
        return ""
    if text[0] in ("'", '"') and text[-1:] == text[0]:
        return text[1:-1]
    lower = text.lower()
    if lower in ("true", "false"):
        return lower == "true"
    try:
        if any(ch in text for ch in (".", "e", "E")):
            return float(text)
        return int(text)
    except ValueError:
        return text


def load_simple_toml(path):
    data = {}
    current = data
    with open(path, "r", encoding="utf-8") as fh:
        for raw_line in fh:
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            if line.startswith("[") and line.endswith("]"):
                current = data
                for part in line[1:-1].split("."):
                    current = current.setdefault(part, {})
                continue
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            current[key.strip()] = parse_scalar(value.strip())
    return data


def load_servo_mapping_config(path):
    data = load_simple_toml(path)
    servos = data.get("servos", {})
    defaults = data.get("defaults", {})
    default_step = int(defaults.get("position_step", 1))
    mappings = {}
    for _name, item in servos.items():
        servo_id = int(item.get("id"))
        startup_check_raw = item.get("startup_check_raw")
        mappings[servo_id] = {
            "id": servo_id,
            "raw_min": int(item.get("raw_min", 0)),
            "raw_max": int(item.get("raw_max", 1000)),
            "home_raw": int(item.get("home_raw", item.get("raw_max", 1000))),
            "startup_check_raw": None if startup_check_raw is None else int(startup_check_raw),
            "logical_min": float(item.get("mapped_angle_at_raw_min", 0.0)),
            "logical_max": float(item.get("mapped_angle_at_raw_max", 1000.0)),
            "position_step": int(item.get("position_step", default_step)),
        }
    return mappings


def clamp(value, low, high):
    return max(low, min(high, value))


def linear_map(x, in_min, in_max, out_min, out_max):
    if in_min == in_max:
        raise ValueError("zero input range")
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)


class ServoMapper(object):
    def __init__(self, config_path):
        self.mappings = load_servo_mapping_config(config_path)
        self.servo_ids = [1, 2, 3]
        self.raw_directions = {1: -1, 2: -1, 3: -1}
        self.physical_min_deg = 0.0
        self.physical_max_deg = 240.0
        self.reference_angles = inverse_kinematics(0.0, 0.0, 240.0)[0]
        self.reference_raw = {}
        self.startup_check_raw = {}
        self.reference_coord = {}
        self.logical_directions = {}
        self.units_per_degree = {}
        for servo_id in self.servo_ids:
            item = self.mappings[servo_id]
            self.reference_raw[servo_id] = clamp_raw(item, item.get("home_raw", item["raw_max"]))
            if item.get("startup_check_raw") is None:
                self.startup_check_raw[servo_id] = self.reference_raw[servo_id]
            else:
                self.startup_check_raw[servo_id] = int(item["startup_check_raw"])
            self.reference_coord[servo_id] = raw_to_logical(item, self.reference_raw[servo_id])
            logical_span = item["logical_max"] - item["logical_min"]
            self.logical_directions[servo_id] = self.raw_directions[servo_id] * (1 if logical_span >= 0 else -1)
            self.units_per_degree[servo_id] = abs(logical_span) / abs(self.physical_max_deg - self.physical_min_deg)

    def raw_to_angles(self, raw_positions):
        angles = [0.0, 0.0, 0.0]
        for index, servo_id in enumerate(self.servo_ids):
            item = self.mappings[servo_id]
            current_coord = raw_to_logical(item, raw_positions[servo_id])
            delta_coord = current_coord - self.reference_coord[servo_id]
            delta_deg = delta_coord / (self.logical_directions[servo_id] * self.units_per_degree[servo_id])
            angles[index] = self.reference_angles[index] + math.radians(delta_deg)
        return angles

    def angles_to_raw(self, angles):
        raw = {}
        for index, servo_id in enumerate(self.servo_ids):
            item = self.mappings[servo_id]
            delta_deg = math.degrees(angles[index] - self.reference_angles[index])
            target_coord = self.reference_coord[servo_id] + (
                self.logical_directions[servo_id] * delta_deg * self.units_per_degree[servo_id]
            )
            raw[servo_id] = logical_to_raw(item, target_coord)
        return raw

    def home_errors(self, raw_positions):
        return {
            servo_id: int(raw_positions[servo_id]) - int(self.reference_raw[servo_id])
            for servo_id in self.servo_ids
        }

    def startup_check_errors(self, raw_positions):
        return {
            servo_id: int(raw_positions[servo_id]) - int(self.startup_check_raw[servo_id])
            for servo_id in self.servo_ids
        }


def quantize_raw(item, value):
    step = max(1, int(item["position_step"]))
    raw = int(round(float(value) / step) * step)
    return clamp_raw(item, raw)


def clamp_raw(item, value):
    raw = int(round(float(value)))
    low = min(int(item["raw_min"]), int(item["raw_max"]))
    high = max(int(item["raw_min"]), int(item["raw_max"]))
    return max(low, min(high, raw))


def raw_to_logical(item, raw_value):
    return linear_map(
        float(raw_value),
        float(item["raw_min"]),
        float(item["raw_max"]),
        float(item["logical_min"]),
        float(item["logical_max"]),
    )


def logical_to_raw(item, logical_value):
    raw = linear_map(
        float(logical_value),
        float(item["logical_min"]),
        float(item["logical_max"]),
        float(item["raw_min"]),
        float(item["raw_max"]),
    )
    return quantize_raw(item, raw)


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_mul(a, scalar):
    return [a[0] * scalar, a[1] * scalar, a[2] * scalar]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm(a):
    return math.sqrt(dot(a, a))


def mat_mul(a, b):
    out = [[0.0] * len(b[0]) for _ in range(len(a))]
    for i in range(len(a)):
        for j in range(len(b[0])):
            value = 0.0
            for k in range(len(b)):
                value += a[i][k] * b[k][j]
            out[i][j] = value
    return out


def mat_vec_mul(a, v):
    return [sum(a[i][k] * v[k] for k in range(len(v))) for i in range(len(a))]


def transform_mul(a, b):
    return mat_mul(a, b)


def transform_inv(t):
    r = [row[:3] for row in t[:3]]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    trans = [t[0][3], t[1][3], t[2][3]]
    inv_trans = mat_vec_mul(rt, [-trans[0], -trans[1], -trans[2]])
    out = [
        [rt[0][0], rt[0][1], rt[0][2], inv_trans[0]],
        [rt[1][0], rt[1][1], rt[1][2], inv_trans[1]],
        [rt[2][0], rt[2][1], rt[2][2], inv_trans[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return out


def transform_translation(t):
    return [t[0][3], t[1][3], t[2][3]]


def transform_from_matrix(matrix):
    return [[float(value) for value in row] for row in matrix]


def transform_from_translation_rotation(translation, rotation):
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], translation[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], translation[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def identity_transform():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return mat_mul(mat_mul(rz, ry), rx)


def transform_from_json(payload):
    if not isinstance(payload, dict):
        return identity_transform()
    if isinstance(payload.get("matrix"), list):
        return transform_from_matrix(payload["matrix"])
    translation = [float(v) for v in payload.get("translation", [0.0, 0.0, 0.0])]
    if isinstance(payload.get("rotation_matrix"), list):
        return transform_from_translation_rotation(translation, payload["rotation_matrix"])
    rpy = [float(v) for v in payload.get("rotation_rpy_deg", [0.0, 0.0, 0.0])]
    return transform_from_translation_rotation(
        translation,
        rpy_matrix(math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2])),
    )


def detection_to_transform(detection):
    position = detection.get("position_m")
    if not isinstance(position, dict):
        raise ValueError("detection has no position_m")
    translation = [float(position.get("x", 0.0)), float(position.get("y", 0.0)), float(position.get("z", 0.0))]
    if isinstance(detection.get("orientation_matrix_column_major"), list):
        flat = detection["orientation_matrix_column_major"]
        if len(flat) == 9:
            rotation = [
                [float(flat[0]), float(flat[3]), float(flat[6])],
                [float(flat[1]), float(flat[4]), float(flat[7])],
                [float(flat[2]), float(flat[5]), float(flat[8])],
            ]
            return transform_from_translation_rotation(translation, rotation)
    if isinstance(detection.get("rotation_matrix"), list):
        return transform_from_translation_rotation(translation, detection["rotation_matrix"])
    orientation = detection.get("orientation_deg")
    if not isinstance(orientation, dict):
        orientation = {}
    rotation = rpy_matrix(
        math.radians(float(orientation.get("roll", 0.0))),
        math.radians(float(orientation.get("pitch", 0.0))),
        math.radians(float(orientation.get("yaw", 0.0))),
    )
    return transform_from_translation_rotation(translation, rotation)


def select_detection(payload, tag_id):
    detections = payload.get("detections")
    if not isinstance(detections, list) or not detections:
        raise ValueError("no detections")
    for detection in detections:
        if not isinstance(detection, dict):
            continue
        if tag_id is None or detection.get("id") == tag_id:
            return detection
    raise ValueError("tag id %s not found" % tag_id)


def load_tool_pose_from_apriltag(snapshot_path, calibration_path, hand_tag_id):
    calibration = read_json(calibration_path)
    snapshot = read_json(snapshot_path)
    base_camera_tf = transform_from_json(calibration["results"]["base_camera"]["transform"])
    tool_hand_tag = transform_from_json(calibration["known_transforms"]["tool_T_hand_tag"])
    detection = select_detection(snapshot, hand_tag_id)
    base_camera_hand_tag = detection_to_transform(detection)
    base_tool = transform_mul(transform_mul(base_camera_tf, base_camera_hand_tag), transform_inv(tool_hand_tag))
    xyz_m = transform_translation(base_tool)
    timestamp = snapshot.get("timestamp_unix")
    age_ms = None
    if isinstance(timestamp, (int, float)):
        age_ms = max(0.0, (time.time() - float(timestamp)) * 1000.0)
    return {
        "detection_id": detection.get("id"),
        "snapshot_age_ms": age_ms,
        "tool_position_mm": [xyz_m[0] * 1000.0, xyz_m[1] * 1000.0, xyz_m[2] * 1000.0],
        "raw_detection": detection,
    }


class RobotParams(object):
    def __init__(self):
        self.l1 = 100.0
        self.l2 = 150.0
        self.l3 = 48.0
        self.servo_offset_x = 75.0
        self.servo_offset_z = 41.231
        self.servo_angle_min = math.radians(45.0)
        self.servo_angle_max = math.radians(225.0)
        self.workspace_z_min = 110.0
        self.workspace_z_max = 280.0
        self.workspace_xy_max = 150.0
        self.ball_joint_angle_limit = math.radians(34.1)
        self.servo_distribution = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]


def local_to_global(x_local, y_local, z_local, servo_angle):
    return [
        x_local * math.cos(servo_angle) + y_local * math.sin(servo_angle),
        -x_local * math.sin(servo_angle) + y_local * math.cos(servo_angle),
        z_local,
    ]


def platform_offset(radius, servo_angle):
    return [radius * math.cos(servo_angle), -radius * math.sin(servo_angle), 0.0]


def inverse_kinematics(x, y, z, params=None):
    params = params or RobotParams()
    results = [inverse_kinematics_single(x, y, z, params.servo_distribution[i], params) for i in range(3)]
    if not all(item[1] for item in results):
        return [0.0, 0.0, 0.0], False
    return [item[0] for item in results], True


def inverse_kinematics_single(xt, yt, zt, servo_angle, params):
    zt = zt - params.servo_offset_z
    x_rot = xt * math.cos(servo_angle) - yt * math.sin(servo_angle)
    y_rot = xt * math.sin(servo_angle) + yt * math.cos(servo_angle)
    arm_end_x = x_rot + params.l3
    under = params.l2 ** 2 - y_rot ** 2
    if under < 0:
        return 0.0, False
    l2p = math.sqrt(under)
    l2p_angle = math.asin(max(-1.0, min(1.0, y_rot / params.l2)))
    if abs(l2p_angle) >= params.ball_joint_angle_limit:
        return 0.0, False
    ext = math.sqrt(zt ** 2 + (params.servo_offset_x - arm_end_x) ** 2)
    if ext <= l2p - params.l1 or ext >= params.l1 + l2p:
        return 0.0, False
    cos_phi = (params.l1 ** 2 + ext ** 2 - l2p ** 2) / (2.0 * params.l1 * ext)
    phi = math.acos(max(-1.0, min(1.0, cos_phi)))
    omega = math.atan2(zt, params.servo_offset_x - arm_end_x)
    theta = phi + omega
    if params.servo_angle_min <= theta <= params.servo_angle_max:
        return theta, True
    return 0.0, False


def forward_kinematics(theta1, theta2, theta3, params=None):
    params = params or RobotParams()
    theta = [theta1, theta2, theta3]
    centers = []
    for i, servo_angle in enumerate(params.servo_distribution):
        elbow = local_to_global(
            params.servo_offset_x - params.l1 * math.cos(theta[i]),
            0.0,
            params.servo_offset_z + params.l1 * math.sin(theta[i]),
            servo_angle,
        )
        centers.append(vec_sub(elbow, platform_offset(params.l3, servo_angle)))
    return intersect_three_spheres(centers, params.l2)


def intersect_three_spheres(centers, radius):
    p1, p2, p3 = centers
    ex = vec_sub(p2, p1)
    d = norm(ex)
    if d < 1e-9:
        return [0.0, 0.0, 0.0], False
    ex = vec_mul(ex, 1.0 / d)
    p3p1 = vec_sub(p3, p1)
    i_val = dot(ex, p3p1)
    temp = vec_sub(p3p1, vec_mul(ex, i_val))
    temp_norm = norm(temp)
    if temp_norm < 1e-9:
        return [0.0, 0.0, 0.0], False
    ey = vec_mul(temp, 1.0 / temp_norm)
    ez = cross(ex, ey)
    j_val = dot(ey, p3p1)
    if abs(j_val) < 1e-9:
        return [0.0, 0.0, 0.0], False
    x_coord = d / 2.0
    y_coord = (i_val ** 2 + j_val ** 2 - 2.0 * i_val * x_coord) / (2.0 * j_val)
    z_sq = radius ** 2 - x_coord ** 2 - y_coord ** 2
    if z_sq < -1e-6:
        return [0.0, 0.0, 0.0], False
    z_coord = math.sqrt(max(z_sq, 0.0))
    base = vec_add(p1, vec_add(vec_mul(ex, x_coord), vec_mul(ey, y_coord)))
    candidate1 = vec_add(base, vec_mul(ez, z_coord))
    candidate2 = vec_sub(base, vec_mul(ez, z_coord))
    return (candidate1 if candidate1[2] >= candidate2[2] else candidate2), True


def open_gamepad(config_path, device_path):
    from evdev_gamepad import BluetoothGamepadReader

    reader = BluetoothGamepadReader(config_path=config_path, device_path=device_path or None, announce=True)
    if not reader.is_available():
        raise RuntimeError(reader.last_error or "8BitDo gamepad not available")
    return reader


def open_servo_driver(port, baudrate):
    from servo_driver import BusServoDriver

    driver = BusServoDriver(port=port, baudrate=baudrate, timeout=1.0, connect_delay=0.2)
    driver.connect()
    return driver
