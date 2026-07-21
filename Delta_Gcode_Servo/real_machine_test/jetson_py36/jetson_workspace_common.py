#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Python 3.6 helpers for Jetson AprilTag workspace sampling."""

from __future__ import print_function

import json
import math
import os
import subprocess
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
DEFAULT_APRILTAG_INTRINSICS = (
    "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/calibration/"
    "usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json"
)
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
        self.servo_ids = [1, 3, 4]
        self.raw_directions = {1: -1, 3: -1, 4: -1}
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

    def raw_range_violations(self, raw_positions, margin_ticks=0):
        violations = {}
        margin = max(0, int(margin_ticks))
        for servo_id in self.servo_ids:
            item = self.mappings[servo_id]
            low = min(int(item["raw_min"]), int(item["raw_max"])) - margin
            high = max(int(item["raw_min"]), int(item["raw_max"])) + margin
            value = int(raw_positions[servo_id])
            if value < low or value > high:
                violations[servo_id] = {
                    "raw": value,
                    "low": low,
                    "high": high,
                    "configured_raw_min": int(item["raw_min"]),
                    "configured_raw_max": int(item["raw_max"]),
                }
        return violations


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


def stop_running_gpu_apriltag_bench():
    script = (
        "PIDS=$(pgrep -f '^./nv_gpu_apriltag_bench( |$)' || true); "
        "if [[ -n \"$PIDS\" ]]; then "
        "  kill $PIDS >/dev/null 2>&1 || true; "
        "  sleep 2; "
        "  for pid in $PIDS; do "
        "    if kill -0 \"$pid\" >/dev/null 2>&1; then "
        "      kill -KILL \"$pid\" >/dev/null 2>&1 || true; "
        "    fi; "
        "  done; "
        "fi"
    )
    return subprocess.call(["bash", "-lc", script])


def capture_fullfov_frame_bgr(
    sensor_id=0,
    sensor_mode=0,
    sensor_width=3264,
    sensor_height=2464,
    sensor_fps=21,
    output_width=1600,
    output_height=1208,
    interpolation_method=3,
    flip_method=0,
    warmup_frames=35,
):
    import cv2

    conv = "nvvidconv"
    conv_props = []
    if interpolation_method is not None:
        conv_props.append("interpolation-method=%d" % int(interpolation_method))
    conv_props.append("flip-method=%d" % int(flip_method))
    conv_stage = conv + " " + " ".join(conv_props)
    pipeline = (
        "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1 ! "
        "%s ! "
        "video/x-raw,width=%d,height=%d,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    ) % (
        int(sensor_id),
        int(sensor_mode),
        int(sensor_width),
        int(sensor_height),
        int(sensor_fps),
        conv_stage,
        int(output_width),
        int(output_height),
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("failed to open CSI pipeline: %s" % pipeline)
    frame = None
    try:
        for _index in range(max(1, int(warmup_frames))):
            ok, image = cap.read()
            if ok and image is not None:
                frame = image
    finally:
        cap.release()
    if frame is None:
        raise RuntimeError("no frame read from CSI pipeline")
    return frame, pipeline


def _opencv_apriltag_dictionary():
    import cv2

    tag36h11 = getattr(cv2.aruco, "DICT_APRILTAG_36h11", getattr(cv2.aruco, "DICT_APRILTAG_36H11", None))
    if tag36h11 is None:
        raise RuntimeError("OpenCV AprilTag 36h11 dictionary is unavailable")
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(tag36h11)
    return cv2.aruco.Dictionary_get(tag36h11)


def _opencv_apriltag_detector(dictionary):
    import cv2

    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()
    if hasattr(params, "cornerRefinementMethod"):
        params.cornerRefinementMethod = getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1)
    if hasattr(cv2.aruco, "ArucoDetector"):
        return params, cv2.aruco.ArucoDetector(dictionary, params)
    return params, None


def _opencv_detect_markers(image, dictionary, params, detector):
    import cv2

    if detector is not None:
        return detector.detectMarkers(image)
    return cv2.aruco.detectMarkers(image, dictionary, parameters=params)


def detect_live_apriltag_opencv(frame_bgr, hand_tag_id):
    import cv2

    frame_h, frame_w = frame_bgr.shape[:2]
    dictionary = _opencv_apriltag_dictionary()
    params, detector = _opencv_apriltag_detector(dictionary)
    center_positions = [
        (0.50, 0.50),
        (0.45, 0.50),
        (0.55, 0.50),
        (0.50, 0.45),
        (0.50, 0.55),
        (0.45, 0.45),
        (0.55, 0.45),
        (0.45, 0.55),
        (0.55, 0.55),
    ]
    roi_fractions = [
        (1.00, 1.00),
        (0.90, 0.90),
        (0.80, 0.80),
        (0.75, 0.75),
        (0.70, 0.70),
        (0.65, 0.65),
        (0.60, 0.60),
    ]
    best = None
    for frac_w, frac_h in roi_fractions:
        crop_w = max(32, int(round(frame_w * frac_w)))
        crop_h = max(32, int(round(frame_h * frac_h)))
        for center_x, center_y in center_positions:
            left = int(round(frame_w * center_x - crop_w / 2.0))
            top = int(round(frame_h * center_y - crop_h / 2.0))
            left = max(0, min(frame_w - crop_w, left))
            top = max(0, min(frame_h - crop_h, top))
            roi = frame_bgr[top : top + crop_h, left : left + crop_w]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            variants = [
                ("gray", gray),
                ("equalize", cv2.equalizeHist(gray)),
                ("gauss_equalize", cv2.equalizeHist(cv2.GaussianBlur(gray, (3, 3), 0))),
                ("otsu", cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]),
            ]
            for variant_name, variant in variants:
                corners, ids, _rejected = _opencv_detect_markers(variant, dictionary, params, detector)
                if ids is None or len(ids) == 0:
                    continue
                ids_flat = [int(value) for value in ids.reshape(-1).tolist()]
                for index, detection_id in enumerate(ids_flat):
                    if hand_tag_id is not None and int(detection_id) != int(hand_tag_id):
                        continue
                    pts = corners[index].reshape(4, 2).astype(float)
                    pts[:, 0] += float(left)
                    pts[:, 1] += float(top)
                    side = 0.0
                    for edge_index in range(4):
                        p0 = pts[edge_index]
                        p1 = pts[(edge_index + 1) % 4]
                        side += math.hypot(float(p1[0] - p0[0]), float(p1[1] - p0[1]))
                    side /= 4.0
                    area = abs(cv2.contourArea(pts.astype("float32")))
                    score = side + (area / 1000.0)
                    if best is None or score > best["score"]:
                        center = pts.mean(axis=0)
                        best = {
                            "id": int(detection_id),
                            "corners_px": [
                                {"x": float(point[0]), "y": float(point[1])}
                                for point in pts
                            ],
                            "center_px": {"x": float(center[0]), "y": float(center[1])},
                            "normalized_xy": {
                                "x": float((center[0] - frame_w / 2.0) / max(1.0, frame_w / 2.0)),
                                "y": float((center[1] - frame_h / 2.0) / max(1.0, frame_h / 2.0)),
                            },
                            "roi": {
                                "x": int(left),
                                "y": int(top),
                                "width": int(crop_w),
                                "height": int(crop_h),
                            },
                            "variant": variant_name,
                            "avg_side_px": float(side),
                            "score": float(score),
                        }
    if best is None:
        raise ValueError("live CPU detection failed for tag id %s" % hand_tag_id)
    return best


def capture_tool_pose_from_live_opencv(
    calibration_path,
    intrinsics_path,
    hand_tag_id,
    stop_gpu_detector=True,
    sensor_id=0,
    sensor_mode=0,
    sensor_width=3264,
    sensor_height=2464,
    sensor_fps=21,
    output_width=1600,
    output_height=1208,
    interpolation_method=3,
    flip_method=0,
    warmup_frames=35,
    tag_size_m=0.0305,
    debug_image_path="",
    debug_overlay_path="",
):
    import cv2
    import numpy as np

    if stop_gpu_detector:
        stop_running_gpu_apriltag_bench()
    frame, pipeline = capture_fullfov_frame_bgr(
        sensor_id=sensor_id,
        sensor_mode=sensor_mode,
        sensor_width=sensor_width,
        sensor_height=sensor_height,
        sensor_fps=sensor_fps,
        output_width=output_width,
        output_height=output_height,
        interpolation_method=interpolation_method,
        flip_method=flip_method,
        warmup_frames=warmup_frames,
    )
    detection = detect_live_apriltag_opencv(frame, hand_tag_id)
    intrinsics = read_json(intrinsics_path)
    camera_matrix = np.array(intrinsics["camera_matrix"], dtype=np.float64)
    calib_size = intrinsics.get("image_size", {})
    calib_w = int(calib_size.get("width", frame.shape[1]))
    calib_h = int(calib_size.get("height", frame.shape[0]))
    scale_x = float(calib_w) / float(frame.shape[1])
    scale_y = float(calib_h) / float(frame.shape[0])
    image_points = []
    for point in detection["corners_px"]:
        image_points.append([float(point["x"]) * scale_x, float(point["y"]) * scale_y])
    image_points = np.array(image_points, dtype=np.float64)
    half = float(tag_size_m) / 2.0
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    solve_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", getattr(cv2, "SOLVEPNP_ITERATIVE", 0))
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, None, flags=solve_flag)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, None)
    if not ok:
        raise RuntimeError("solvePnP failed for live CPU AprilTag pose")
    rotation, _jacobian = cv2.Rodrigues(rvec)
    orientation_column_major = [
        float(rotation[0][0]),
        float(rotation[1][0]),
        float(rotation[2][0]),
        float(rotation[0][1]),
        float(rotation[1][1]),
        float(rotation[2][1]),
        float(rotation[0][2]),
        float(rotation[1][2]),
        float(rotation[2][2]),
    ]
    detection["position_m"] = {
        "x": float(tvec[0][0]),
        "y": float(tvec[1][0]),
        "z": float(tvec[2][0]),
    }
    detection["orientation_matrix_column_major"] = orientation_column_major
    detection["source_timestamp_unix"] = time.time()
    detection["capture_variant"] = detection.pop("variant")
    detection["capture_roi"] = detection.pop("roi")
    if debug_image_path:
        parent = os.path.dirname(os.path.abspath(debug_image_path))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        cv2.imwrite(debug_image_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    if debug_overlay_path:
        parent = os.path.dirname(os.path.abspath(debug_overlay_path))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        overlay = frame.copy()
        pts = []
        for point in detection["corners_px"]:
            pts.append([int(round(float(point["x"]))), int(round(float(point["y"])))])
        cv2.polylines(overlay, [np.array(pts, dtype=np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 3)
        cv2.putText(
            overlay,
            "id=%s live_cpu" % detection["id"],
            (30, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(debug_overlay_path, overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    calibration = read_json(calibration_path)
    base_camera_tf = transform_from_json(calibration["results"]["base_camera"]["transform"])
    tool_hand_tag = transform_from_json(calibration["known_transforms"]["tool_T_hand_tag"])
    base_camera_hand_tag = transform_from_translation_rotation(
        [float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])],
        [
            [float(rotation[0][0]), float(rotation[0][1]), float(rotation[0][2])],
            [float(rotation[1][0]), float(rotation[1][1]), float(rotation[1][2])],
            [float(rotation[2][0]), float(rotation[2][1]), float(rotation[2][2])],
        ],
    )
    base_tool = transform_mul(transform_mul(base_camera_tf, base_camera_hand_tag), transform_inv(tool_hand_tag))
    xyz_m = transform_translation(base_tool)
    return {
        "detection_id": detection["id"],
        "snapshot_age_ms": 0.0,
        "tool_position_mm": [xyz_m[0] * 1000.0, xyz_m[1] * 1000.0, xyz_m[2] * 1000.0],
        "raw_detection": detection,
        "vision_mode": "live_cpu_fullfov_capture",
        "capture_frame": {"width": int(frame.shape[1]), "height": int(frame.shape[0])},
        "capture_pipeline": pipeline,
        "intrinsics_source": intrinsics_path,
        "debug_image_path": debug_image_path,
        "debug_overlay_path": debug_overlay_path,
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
        self.workspace_z_min = 90.0
        self.workspace_z_max = 280.0
        self.workspace_xy_max = 150.0
        self.ball_joint_angle_limit = math.radians(34.1)
        self.servo_distribution = [-math.pi / 2.0, math.pi / 6.0, 5.0 * math.pi / 6.0]


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
