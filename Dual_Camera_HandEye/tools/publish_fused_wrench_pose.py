#!/usr/bin/env python3

import argparse
import json
import math
import socket
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List


Matrix = List[List[float]]


def matmul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def transform_point(matrix: Matrix, point: List[float]) -> List[float]:
    x, y, z = point
    return [
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
    ]


def rpy_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Matrix:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def transform_from_rpy(values: List[float]) -> Matrix:
    if len(values) != 6:
        raise ValueError("--base-tool-rpy needs 6 values: x y z roll pitch yaw")
    x, y, z, roll, pitch, yaw = [float(v) for v in values]
    rot = rpy_matrix(roll, pitch, yaw)
    return [
        [rot[0][0], rot[0][1], rot[0][2], x],
        [rot[1][0], rot[1][1], rot[1][2], y],
        [rot[2][0], rot[2][1], rot[2][2], z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def read_json(path: Any) -> Dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def matrix_from_transform(payload: Dict[str, Any]) -> Matrix:
    if "matrix" in payload:
        return [[float(v) for v in row] for row in payload["matrix"]]
    if "transform" in payload:
        return matrix_from_transform(payload["transform"])
    for key in ("base_T_tool", "tool_T_bottom_stereo", "tool_T_object_camera"):
        if key in payload:
            return matrix_from_transform(payload[key])
    raise ValueError("transform JSON must contain matrix, transform, base_T_tool, tool_T_bottom_stereo, or tool_T_object_camera")


def load_tool_camera_transform(args: argparse.Namespace) -> Matrix:
    if args.tool_camera_json:
        return matrix_from_transform(read_json(args.tool_camera_json))
    calibration = read_json(args.calibration)
    key = args.calibration_tool_key
    if key in calibration.get("results", {}):
        return matrix_from_transform(calibration["results"][key]["transform"])
    if key in calibration.get("known_transforms", {}):
        return matrix_from_transform(calibration["known_transforms"][key])
    raise ValueError("could not find tool camera transform key %r in %s" % (key, args.calibration))


def load_base_tool_payload(args: argparse.Namespace) -> Dict[str, Any]:
    if args.base_tool_json:
        payload = read_json(args.base_tool_json)
        return {
            "matrix": matrix_from_transform(payload),
            "source": args.base_tool_json,
            "mode": payload.get("mode", "servo_feedback"),
            "timestamp": payload.get("timestamp"),
            "raw": payload.get("raw"),
            "tool_position_base_m": payload.get("tool_position_base_m"),
        }
    if args.base_tool_rpy:
        return {
            "matrix": transform_from_rpy(args.base_tool_rpy),
            "source": "--base-tool-rpy",
            "mode": "static_rpy_simulated",
            "timestamp": None,
            "raw": None,
            "tool_position_base_m": {
                "x": float(args.base_tool_rpy[0]),
                "y": float(args.base_tool_rpy[1]),
                "z": float(args.base_tool_rpy[2]),
            },
        }
    raise ValueError("provide --base-tool-json or --base-tool-rpy")


def fetch_latest(url: str) -> Dict[str, Any]:
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def source_timestamp(payload: Dict[str, Any]) -> float:
    for key in ("timestamp_unix", "timestamp"):
        value = payload.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return 0.0


def target_center(target: Dict[str, Any]) -> Any:
    return target.get("center_px") or target.get("center")


def build_payload(args: argparse.Namespace, seq: int) -> Dict[str, Any]:
    latest = fetch_latest(args.latest_url)
    now = time.time()
    src_ts = source_timestamp(latest)
    source_age = now - src_ts if src_ts > 0.0 else None
    target = latest.get("target") or {}
    position = target.get("position_camera_m") or target.get("position_m")
    if source_age is not None and source_age > float(args.max_source_age_sec):
        return {
            "valid": False,
            "status": "stale_source",
            "timestamp": now,
            "seq": seq,
            "source": args.latest_url,
            "source_timestamp": src_ts,
            "source_age_sec": round(source_age, 4),
        }
    if not latest.get("valid") or not position:
        return {
            "valid": False,
            "status": latest.get("status", "no_target_position"),
            "timestamp": now,
            "seq": seq,
            "source": args.latest_url,
            "source_timestamp": src_ts or None,
            "source_age_sec": None if source_age is None else round(source_age, 4),
        }

    base_tool = load_base_tool_payload(args)
    base_t_tool = base_tool["matrix"]
    tool_t_camera = load_tool_camera_transform(args)
    base_t_camera = matmul(base_t_tool, tool_t_camera)
    camera_point = [float(position["x"]), float(position["y"]), float(position["z"])]
    base_point = transform_point(base_t_camera, camera_point)

    status = "ok"
    warnings = []
    if base_tool["mode"] == "static_rpy_simulated":
        status = "ok_simulated_base_tool"
        warnings.append("base_T_tool is static/simulated; use BASE_TOOL_JSON from servo feedback for real grasp planning")

    return {
        "valid": True,
        "status": status,
        "warnings": warnings,
        "seq": seq,
        "timestamp": now,
        "source": args.latest_url,
        "source_timestamp": src_ts or None,
        "source_age_sec": None if source_age is None else round(source_age, 4),
        "frames": {
            "base": "delta_base",
            "tool": "tool",
            "camera": "bottom_stereo",
            "object": "wrench",
        },
        "transforms": {
            "base_T_tool": {
                "source": base_tool["source"],
                "mode": base_tool["mode"],
                "timestamp": base_tool["timestamp"],
                "raw": base_tool["raw"],
                "tool_position_base_m": base_tool["tool_position_base_m"],
            },
            "tool_T_camera": {
                "source": args.tool_camera_json or args.calibration,
                "key": args.calibration_tool_key,
            },
        },
        "target": {
            "class": target.get("class", "wrench"),
            "confidence": target.get("conf"),
            "box": target.get("box"),
            "center_px": target_center(target),
            "offset_px": target.get("offset"),
            "image": latest.get("image"),
        },
        "wrench_position_camera_m": {
            "x": round(camera_point[0], 5),
            "y": round(camera_point[1], 5),
            "z": round(camera_point[2], 5),
        },
        "wrench_position_base_m": {
            "x": round(base_point[0], 5),
            "y": round(base_point[1], 5),
            "z": round(base_point[2], 5),
        },
        "distance_m": target.get("distance_m"),
        "distance_method": target.get("distance_method"),
    }


def publish_udp(sock: socket.socket, host: str, port: int, payload: Dict[str, Any]) -> None:
    body = (json.dumps(payload, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
    sock.sendto(body, (host, port))


def write_output(path: str, payload: Dict[str, Any]) -> None:
    if not path:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    tmp = output.with_suffix(output.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(output)


def main() -> int:
    parser = argparse.ArgumentParser(description="Fuse wrench RGB+depth with hand-eye transforms and optionally publish UDP.")
    parser.add_argument("--latest-url", default="http://127.0.0.1:8090/latest.json")
    parser.add_argument("--calibration", default="Dual_Camera_HandEye/output/calibration_result.json")
    parser.add_argument("--calibration-tool-key", default="object_camera")
    parser.add_argument("--tool-camera-json", default="", help="Override tool_T_bottom_stereo/tool_T_object_camera JSON.")
    parser.add_argument("--base-tool-json", default="", help="JSON containing current base_T_tool.")
    parser.add_argument("--base-tool-rpy", type=float, nargs=6, metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"))
    parser.add_argument("--output", default="Dual_Camera_HandEye/output/fused_wrench_pose_latest.json")
    parser.add_argument("--udp-host", default="")
    parser.add_argument("--udp-port", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--max-source-age-sec", type=float, default=0.75)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    interval = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.1
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.udp_host and args.udp_port else None
    seq = 0
    while True:
        seq += 1
        try:
            payload = build_payload(args, seq)
        except Exception as exc:
            payload = {"valid": False, "status": "error", "error": repr(exc), "seq": seq, "timestamp": time.time()}
        write_output(args.output, payload)
        if sock and payload.get("valid"):
            publish_udp(sock, args.udp_host, args.udp_port, payload)
        print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")), flush=True)
        if args.once:
            return 0 if payload.get("valid") else 1
        time.sleep(interval)


if __name__ == "__main__":
    raise SystemExit(main())
