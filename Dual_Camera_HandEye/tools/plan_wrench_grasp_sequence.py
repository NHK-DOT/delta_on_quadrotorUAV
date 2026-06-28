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


def workspace_violation(x, y, z, args):
    radius = math.sqrt(x * x + y * y)
    violations = []
    if float(args.xy_limit_mm) > 0.0 and radius > float(args.xy_limit_mm):
        violations.append("xy_radius %.3f > %.3f" % (radius, float(args.xy_limit_mm)))
    if z < float(args.z_min_mm):
        violations.append("z %.3f < %.3f" % (z, float(args.z_min_mm)))
    if z > float(args.z_max_mm):
        violations.append("z %.3f > %.3f" % (z, float(args.z_max_mm)))
    return violations


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

    base_tool = fused.get("transforms", {}).get("base_T_tool", {})
    tool_position = base_tool.get("tool_position_base_m") or {}
    tool_z_m = tool_position.get("z")
    if base_tool and not base_tool.get("valid", True):
        return {
            "valid": False,
            "status": "invalid_base_tool_feedback",
            "timestamp": time.time(),
            "source": str(args.fused_pose),
            "source_seq": fused.get("seq"),
            "base_tool": base_tool,
        }
    if isinstance(tool_z_m, (int, float)):
        tool_z_mm = float(tool_z_m) * 1000.0
        if not args.allow_tool_out_of_range and (
            tool_z_mm < float(args.tool_z_min_mm) or tool_z_mm > float(args.tool_z_max_mm)
        ):
            return {
                "valid": False,
                "status": "tool_out_of_range",
                "timestamp": time.time(),
                "source": str(args.fused_pose),
                "source_seq": fused.get("seq"),
                "base_tool": {
                    "mode": base_tool.get("mode"),
                    "raw": base_tool.get("raw"),
                    "tool_position_base_mm": {
                        "x": round(float(tool_position.get("x", 0.0)) * 1000.0, 3),
                        "y": round(float(tool_position.get("y", 0.0)) * 1000.0, 3),
                        "z": round(tool_z_mm, 3),
                    },
                    "warnings": base_tool.get("warnings") or [],
                },
                "safety": {
                    "tool_z_min_mm": float(args.tool_z_min_mm),
                    "tool_z_max_mm": float(args.tool_z_max_mm),
                    "requires_executor_confirmation": True,
                },
            }

    wrench = fused.get("wrench_position_base_m") or {}
    x = float(wrench["x"]) * 1000.0 + float(args.grasp_offset_x_mm)
    y = float(wrench["y"]) * 1000.0 + float(args.grasp_offset_y_mm)
    z = float(wrench["z"]) * 1000.0 + float(args.grasp_offset_z_mm)

    source_age = fused.get("source_age_sec")
    if isinstance(source_age, (int, float)) and source_age > float(args.max_source_age_sec):
        return {
            "valid": False,
            "status": "stale_fused_pose",
            "timestamp": time.time(),
            "source": str(args.fused_pose),
            "source_seq": fused.get("seq"),
            "source_age_sec": source_age,
        }

    transform_mode = fused.get("transforms", {}).get("base_T_tool", {}).get("mode")
    if transform_mode == "static_rpy_simulated" and not args.allow_simulated_base_tool:
        return {
            "valid": False,
            "status": "simulated_base_tool_rejected",
            "timestamp": time.time(),
            "source": str(args.fused_pose),
            "source_seq": fused.get("seq"),
            "reason": "planner requires BASE_TOOL_JSON from servo feedback unless --allow-simulated-base-tool is set",
        }

    original = {"x": x, "y": y, "z": z}
    violations = workspace_violation(x, y, z, args)
    if violations and not args.allow_workspace_clamp:
        return {
            "valid": False,
            "status": "out_of_workspace",
            "timestamp": time.time(),
            "source": str(args.fused_pose),
            "source_seq": fused.get("seq"),
            "violations": violations,
            "object": {
                "class": "wrench",
                "confidence": fused.get("target", {}).get("confidence"),
                "position_base_mm": {
                    "x": round(original["x"], 3),
                    "y": round(original["y"], 3),
                    "z": round(original["z"], 3),
                },
            },
            "safety": {
                "dry_run": bool(args.dry_run),
                "xy_limit_mm": float(args.xy_limit_mm),
                "z_min_mm": float(args.z_min_mm),
                "z_max_mm": float(args.z_max_mm),
                "requires_executor_confirmation": True,
            },
        }

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
            "tool_z_min_mm": float(args.tool_z_min_mm),
            "tool_z_max_mm": float(args.tool_z_max_mm),
            "requires_executor_confirmation": True,
            "allow_workspace_clamp": bool(args.allow_workspace_clamp),
        },
        "object": {
            "class": "wrench",
            "confidence": fused.get("target", {}).get("confidence"),
            "unclamped_position_base_mm": {
                "x": round(original["x"], 3),
                "y": round(original["y"], 3),
                "z": round(original["z"], 3),
            },
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
    parser.add_argument("--max-source-age-sec", type=float, default=1.0)
    parser.add_argument("--allow-workspace-clamp", action="store_true")
    parser.add_argument("--allow-simulated-base-tool", action="store_true")
    parser.add_argument("--tool-z-min-mm", type=float, default=155.0)
    parser.add_argument("--tool-z-max-mm", type=float, default=280.0)
    parser.add_argument("--allow-tool-out-of-range", action="store_true")
    args = parser.parse_args()

    payload = build_sequence(args)
    write_json(args.output, payload)
    if args.udp_host and args.udp_port and payload.get("valid"):
        publish_udp(args.udp_host, args.udp_port, payload)
    print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    return 0 if payload.get("valid") else 1


if __name__ == "__main__":
    raise SystemExit(main())
