#!/usr/bin/env python3
"""Build a conservative wrench grasp sequence from the fused wrench pose.

This tool is intentionally a planner/publisher, not a direct actuator. It reads
the fused wrench pose JSON, generates pregrasp/approach/grasp/lift/home
waypoints in the delta base frame, writes a command JSON, and can optionally
publish the same command over UDP to a main controller.
"""

import argparse
import json
import math
import socket
import time
from pathlib import Path


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_xy_radius(x, y, limit):
    radius = math.sqrt(x * x + y * y)
    if limit > 0.0 and radius > limit and radius > 1e-9:
        scale = limit / radius
        return x * scale, y * scale
    return x, y


def waypoint(name, x, y, z, gripper, speed):
    return {
        "name": name,
        "position_mm": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
        "gripper": gripper,
        "speed_mm_s": float(speed),
    }


def build_sequence(args):
    fused = read_json(args.fused_pose)
    if not fused.get("valid"):
        return {
            "valid": False,
            "status": "no_valid_wrench_pose",
            "source_status": fused.get("status"),
            "timestamp": time.time(),
        }

    wrench = fused.get("wrench_position_base_m") or {}
    x = float(wrench["x"]) * 1000.0 + float(args.grasp_offset_x_mm)
    y = float(wrench["y"]) * 1000.0 + float(args.grasp_offset_y_mm)
    z = float(wrench["z"]) * 1000.0 + float(args.grasp_offset_z_mm)

    x, y = clamp_xy_radius(x, y, float(args.xy_limit_mm))
    z = clamp(z, float(args.z_min_mm), float(args.z_max_mm))

    approach_z = clamp(z + float(args.approach_height_mm), float(args.z_min_mm), float(args.z_max_mm))
    lift_z = clamp(z + float(args.lift_height_mm), float(args.z_min_mm), float(args.z_max_mm))

    points = [
        waypoint("home", float(args.home_x_mm), float(args.home_y_mm), float(args.home_z_mm), "open", args.travel_speed_mm_s),
        waypoint("pregrasp", x, y, approach_z, "open", args.travel_speed_mm_s),
        waypoint("approach", x, y, z, "open", args.approach_speed_mm_s),
        waypoint("grasp", x, y, z, "close", args.grasp_speed_mm_s),
        waypoint("lift", x, y, lift_z, "close", args.travel_speed_mm_s),
        waypoint("return_home", float(args.home_x_mm), float(args.home_y_mm), float(args.home_z_mm), "hold", args.travel_speed_mm_s),
    ]

    attitude = fused.get("target", {}).get("center_px") or {}
    image = fused.get("target", {}).get("image") or {}
    attitude_trim = {
        "mode": "disabled_no_flight_controller_protocol",
        "roll_norm": 0.0,
        "pitch_norm": 0.0,
        "yaw_norm": 0.0,
        "throttle_norm": 0.0,
        "note": "Fill this from the detector image error once the flight-control receiver contract is confirmed.",
    }
    if isinstance(attitude, dict) and isinstance(image, dict):
        attitude_trim["source_center_px"] = attitude
        attitude_trim["source_image"] = image

    return {
        "valid": True,
        "status": "planned",
        "timestamp": time.time(),
        "source": str(args.fused_pose),
        "source_seq": fused.get("seq"),
        "safety": {
            "dry_run": bool(args.dry_run),
            "xy_limit_mm": float(args.xy_limit_mm),
            "z_min_mm": float(args.z_min_mm),
            "z_max_mm": float(args.z_max_mm),
            "requires_executor_confirmation": True,
        },
        "object": {
            "class": "wrench",
            "confidence": fused.get("target", {}).get("confidence"),
            "position_base_mm": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
        },
        "sequence": points,
        "uav_attitude_trim": attitude_trim,
    }


def publish_udp(host, port, payload):
    body = (json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(body, (host, port))
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fused-pose", type=Path, default=Path("Dual_Camera_HandEye/output/fused_wrench_pose_latest.json"))
    parser.add_argument("--output", type=Path, default=Path("Dual_Camera_HandEye/output/wrench_grasp_sequence_latest.json"))
    parser.add_argument("--udp-host", default="")
    parser.add_argument("--udp-port", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true", default=True)
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
    args = parser.parse_args()

    payload = build_sequence(args)
    write_json(args.output, payload)
    if args.udp_host and args.udp_port and payload.get("valid"):
        publish_udp(args.udp_host, args.udp_port, payload)
    print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    return 0 if payload.get("valid") else 1


if __name__ == "__main__":
    raise SystemExit(main())
