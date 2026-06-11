from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

from .anchor import (
    estimate_base_T_tool_from_hand_tag,
    known_transform,
    plan_image_follow_step,
    plan_pickup_follow_step,
    transform_from_result,
)
from .calibration import calibrate_dataset, load_dataset
from .geometry import Transform, transform_error, transform_from_json
from .snapshot import detection_transform_from_snapshot, image_error_from_snapshot
from .synthetic import build_synthetic_dataset


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="78arm dual-camera hand-eye calibration demo")
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="generate synthetic sample data")
    generate.add_argument("--output", type=Path, default=Path("output/synthetic_samples.json"))
    generate.add_argument("--samples", type=int, default=24)
    generate.add_argument("--seed", type=int, default=78)
    generate.add_argument("--translation-noise-m", type=float, default=0.0015)
    generate.add_argument("--rotation-noise-deg", type=float, default=0.25)

    calibrate = subparsers.add_parser("calibrate", help="estimate camera extrinsics from samples")
    calibrate.add_argument("--samples", type=Path, required=True)
    calibrate.add_argument("--output", type=Path, default=Path("output/calibration_result.json"))
    calibrate.add_argument("--wrist-method", choices=["direct", "handeye"], default="direct")

    snapshot = subparsers.add_parser("snapshot-transform", help="convert an existing AprilTag snapshot detection to a transform")
    snapshot.add_argument("--snapshot", type=Path, required=True)
    snapshot.add_argument("--tag-id", type=int)
    snapshot.add_argument("--transform-name", default="camera_T_target")
    snapshot.add_argument("--output", type=Path, default=Path("output/snapshot_transform.json"))

    project = subparsers.add_parser("project-object", help="project side-camera object pose into the arm base frame")
    project.add_argument("--calibration", type=Path, required=True)
    project.add_argument("--base-tool", type=Path, help="JSON transform for current base_T_tool")
    project.add_argument(
        "--base-tool-rpy",
        type=float,
        nargs=6,
        metavar=("X_M", "Y_M", "Z_M", "ROLL_DEG", "PITCH_DEG", "YAW_DEG"),
        help="Inline current base_T_tool as xyz meters plus rpy degrees",
    )
    project.add_argument("--object-snapshot", type=Path, required=True)
    project.add_argument("--object-id", type=int)
    project.add_argument("--output", type=Path, default=Path("output/object_in_base.json"))

    estimate_tool = subparsers.add_parser("estimate-tool", help="estimate current base_T_tool from the base camera hand-tag snapshot")
    estimate_tool.add_argument("--calibration", type=Path, required=True)
    estimate_tool.add_argument("--base-camera-snapshot", type=Path, required=True)
    estimate_tool.add_argument("--hand-tag-id", type=int)
    estimate_tool.add_argument("--tool-hand-tag", type=Path, help="optional JSON transform overriding known_transforms.tool_T_hand_tag")
    estimate_tool.add_argument("--output", type=Path, default=Path("output/base_tool_from_camera.json"))

    follow = subparsers.add_parser("plan-follow-step", help="plan one small visual-servo step for the magnet pickup point")
    follow.add_argument("--calibration", type=Path, required=True)
    follow.add_argument("--base-tool", type=Path, help="JSON transform for current base_T_tool")
    follow.add_argument(
        "--base-tool-rpy",
        type=float,
        nargs=6,
        metavar=("X_M", "Y_M", "Z_M", "ROLL_DEG", "PITCH_DEG", "YAW_DEG"),
        help="Inline current base_T_tool as xyz meters plus rpy degrees",
    )
    follow.add_argument("--object-snapshot", type=Path, required=True)
    follow.add_argument("--object-id", type=int)
    follow.add_argument("--pickup-offset-mm", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    follow.add_argument("--object-offset-mm", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    follow.add_argument("--track-axes", choices=["xy", "xyz"], default="xy")
    follow.add_argument("--max-step-mm", type=float, default=5.0)
    follow.add_argument("--tolerance-mm", type=float, default=2.0)
    follow.add_argument("--check-delta-ik", action="store_true")
    follow.add_argument("--output", type=Path, default=Path("output/follow_step.json"))

    image_follow = subparsers.add_parser("plan-image-follow-step", help="plan one gimbal-like image-centering visual-servo step")
    image_follow.add_argument("--calibration", type=Path, required=True)
    image_follow.add_argument("--base-tool", type=Path, help="JSON transform for current base_T_tool")
    image_follow.add_argument(
        "--base-tool-rpy",
        type=float,
        nargs=6,
        metavar=("X_M", "Y_M", "Z_M", "ROLL_DEG", "PITCH_DEG", "YAW_DEG"),
        help="Inline current base_T_tool as xyz meters plus rpy degrees",
    )
    image_follow.add_argument("--object-snapshot", type=Path, required=True)
    image_follow.add_argument("--object-id", type=int)
    image_follow.add_argument("--target-normalized-xy", type=float, nargs=2, default=[0.0, 0.0])
    image_follow.add_argument("--gain-mm-per-norm", type=float, default=15.0)
    image_follow.add_argument("--max-step-mm", type=float, default=3.0)
    image_follow.add_argument("--tolerance-norm", type=float, default=0.04)
    image_follow.add_argument("--lock-z", action="store_true", default=True)
    image_follow.add_argument("--allow-z", action="store_true", help="allow the camera-frame step to affect base Z")
    image_follow.add_argument("--check-delta-ik", action="store_true")
    image_follow.add_argument("--output", type=Path, default=Path("output/image_follow_step.json"))

    args = parser.parse_args(argv)
    if args.command == "generate":
        return command_generate(args)
    if args.command == "calibrate":
        return command_calibrate(args)
    if args.command == "snapshot-transform":
        return command_snapshot_transform(args)
    if args.command == "project-object":
        return command_project_object(args)
    if args.command == "estimate-tool":
        return command_estimate_tool(args)
    if args.command == "plan-follow-step":
        return command_plan_follow_step(args)
    if args.command == "plan-image-follow-step":
        return command_plan_image_follow_step(args)
    raise RuntimeError(f"unhandled command: {args.command}")


def command_generate(args: argparse.Namespace) -> int:
    dataset = build_synthetic_dataset(
        sample_count=args.samples,
        seed=args.seed,
        translation_noise_m=args.translation_noise_m,
        rotation_noise_deg=args.rotation_noise_deg,
    )
    write_json(args.output, dataset)
    print(f"Wrote synthetic samples: {args.output}")
    print(f"Sample count: {len(dataset['samples'])}")
    return 0


def command_calibrate(args: argparse.Namespace) -> int:
    payload = read_json(args.samples)
    dataset = load_dataset(payload)
    result = calibrate_dataset(dataset, wrist_method=args.wrist_method)

    if "ground_truth" in payload:
        result["ground_truth_error"] = compare_with_ground_truth(payload, result)

    write_json(args.output, result)
    print(f"Wrote calibration result: {args.output}")
    print_summary(result)
    return 0


def command_snapshot_transform(args: argparse.Namespace) -> int:
    transform, detection = detection_transform_from_snapshot(args.snapshot, tag_id=args.tag_id)
    payload = {
        "transform_name": args.transform_name,
        "source_snapshot": str(args.snapshot),
        "detection_id": detection.get("id"),
        "transform": transform.to_json(),
    }
    write_json(args.output, payload)
    print(f"Wrote snapshot transform: {args.output}")
    print_transform_summary(args.transform_name, transform)
    return 0


def command_project_object(args: argparse.Namespace) -> int:
    if args.base_tool is None and args.base_tool_rpy is None:
        raise ValueError("provide --base-tool or --base-tool-rpy")
    if args.base_tool is not None and args.base_tool_rpy is not None:
        raise ValueError("provide only one of --base-tool or --base-tool-rpy")

    calibration = read_json(args.calibration)
    base_T_tool = load_transform_arg(args.base_tool, args.base_tool_rpy)
    tool_T_object_camera = transform_from_result(calibration, "object_camera")
    object_camera_T_object, detection = detection_transform_from_snapshot(
        args.object_snapshot,
        tag_id=args.object_id,
    )
    base_T_object = base_T_tool @ tool_T_object_camera @ object_camera_T_object

    payload = {
        "transform_name": "base_T_object",
        "source_calibration": str(args.calibration),
        "source_object_snapshot": str(args.object_snapshot),
        "detection_id": detection.get("id"),
        "base_T_tool": base_T_tool.to_json(),
        "tool_T_object_camera": tool_T_object_camera.to_json(),
        "object_camera_T_object": object_camera_T_object.to_json(),
        "base_T_object": base_T_object.to_json(),
    }
    write_json(args.output, payload)
    print(f"Wrote object projection: {args.output}")
    print_transform_summary("base_T_object", base_T_object)
    return 0


def command_estimate_tool(args: argparse.Namespace) -> int:
    calibration = read_json(args.calibration)
    base_T_base_camera = transform_from_result(calibration, "base_camera")
    if args.tool_hand_tag is not None:
        tool_T_hand_tag = load_transform_arg(args.tool_hand_tag, None)
    else:
        tool_T_hand_tag = known_transform(calibration, "tool_T_hand_tag")

    base_camera_T_hand_tag, detection = detection_transform_from_snapshot(
        args.base_camera_snapshot,
        tag_id=args.hand_tag_id,
    )
    base_T_tool = estimate_base_T_tool_from_hand_tag(
        base_T_base_camera,
        base_camera_T_hand_tag,
        tool_T_hand_tag,
    )
    payload = {
        "transform_name": "base_T_tool",
        "source_calibration": str(args.calibration),
        "source_base_camera_snapshot": str(args.base_camera_snapshot),
        "detection_id": detection.get("id"),
        "base_T_base_camera": base_T_base_camera.to_json(),
        "base_camera_T_hand_tag": base_camera_T_hand_tag.to_json(),
        "tool_T_hand_tag": tool_T_hand_tag.to_json(),
        "base_T_tool": base_T_tool.to_json(),
    }
    write_json(args.output, payload)
    print(f"Wrote base tool estimate: {args.output}")
    print_transform_summary("base_T_tool", base_T_tool)
    return 0


def command_plan_follow_step(args: argparse.Namespace) -> int:
    if args.base_tool is None and args.base_tool_rpy is None:
        raise ValueError("provide --base-tool or --base-tool-rpy")
    if args.base_tool is not None and args.base_tool_rpy is not None:
        raise ValueError("provide only one of --base-tool or --base-tool-rpy")

    calibration = read_json(args.calibration)
    base_T_tool = load_transform_arg(args.base_tool, args.base_tool_rpy)
    tool_T_object_camera = transform_from_result(calibration, "object_camera")
    object_camera_T_object, detection = detection_transform_from_snapshot(
        args.object_snapshot,
        tag_id=args.object_id,
    )
    tool_T_pickup = Transform.from_rpy_deg(
        translation=np.asarray(args.pickup_offset_mm, dtype=float) / 1000.0,
        rpy_deg=[0.0, 0.0, 0.0],
    )
    object_offset_base_m = np.asarray(args.object_offset_mm, dtype=float) / 1000.0
    plan = plan_pickup_follow_step(
        base_T_tool=base_T_tool,
        tool_T_object_camera=tool_T_object_camera,
        object_camera_T_object=object_camera_T_object,
        tool_T_pickup=tool_T_pickup,
        object_offset_base_m=object_offset_base_m,
        track_axes=args.track_axes,
        max_step_m=args.max_step_mm / 1000.0,
        tolerance_m=args.tolerance_mm / 1000.0,
    )

    payload = {
        "mode": "visual_follow_step",
        "detection_id": detection.get("id"),
        "track_axes": args.track_axes,
        "pickup_offset_mm": [float(v) for v in args.pickup_offset_mm],
        "object_offset_mm": [float(v) for v in args.object_offset_mm],
        "max_step_mm": float(args.max_step_mm),
        "tolerance_mm": float(args.tolerance_mm),
        "base_T_tool": base_T_tool.to_json(),
        "tool_T_object_camera": tool_T_object_camera.to_json(),
        "object_camera_T_object": object_camera_T_object.to_json(),
        "base_T_object": plan.base_T_object.to_json(),
        "current_base_T_pickup": plan.current_base_T_pickup.to_json(),
        "desired_base_T_pickup": plan.desired_base_T_pickup.to_json(),
        "next_base_T_tool": plan.next_base_T_tool.to_json(),
        "error_mm": [float(v * 1000.0) for v in plan.error_m],
        "command_step_mm": [float(v * 1000.0) for v in plan.command_step_m],
        "within_tolerance": plan.within_tolerance,
    }
    if args.check_delta_ik:
        payload["delta_ik_check"] = check_delta_ik(plan.next_base_T_tool)

    write_json(args.output, payload)
    print(f"Wrote follow step plan: {args.output}")
    print_vector_summary("pickup_error", payload["error_mm"])
    print_vector_summary("command_step", payload["command_step_mm"])
    print_transform_summary("next_base_T_tool", plan.next_base_T_tool)
    if plan.within_tolerance:
        print("pickup target is within tolerance")
    return 0


def command_plan_image_follow_step(args: argparse.Namespace) -> int:
    if args.base_tool is None and args.base_tool_rpy is None:
        raise ValueError("provide --base-tool or --base-tool-rpy")
    if args.base_tool is not None and args.base_tool_rpy is not None:
        raise ValueError("provide only one of --base-tool or --base-tool-rpy")

    calibration = read_json(args.calibration)
    base_T_tool = load_transform_arg(args.base_tool, args.base_tool_rpy)
    tool_T_object_camera = transform_from_result(calibration, "object_camera")
    image_error, detection, snapshot_payload = image_error_from_snapshot(
        args.object_snapshot,
        tag_id=args.object_id,
        target_xy=tuple(args.target_normalized_xy),
    )
    plan = plan_image_follow_step(
        base_T_tool=base_T_tool,
        tool_T_object_camera=tool_T_object_camera,
        image_error=image_error,
        gain_m_per_norm=args.gain_mm_per_norm / 1000.0,
        max_step_m=args.max_step_mm / 1000.0,
        tolerance_norm=args.tolerance_norm,
        lock_z=not args.allow_z,
    )
    payload = {
        "mode": "image_follow_step",
        "source_object_snapshot": str(args.object_snapshot),
        "detection_id": detection.get("id"),
        "target_normalized_xy": [float(v) for v in args.target_normalized_xy],
        "image_error": [float(v) for v in plan.image_error],
        "gain_mm_per_norm": float(args.gain_mm_per_norm),
        "max_step_mm": float(args.max_step_mm),
        "tolerance_norm": float(args.tolerance_norm),
        "lock_z": not args.allow_z,
        "base_T_tool": base_T_tool.to_json(),
        "tool_T_object_camera": tool_T_object_camera.to_json(),
        "command_step_camera_mm": [float(v * 1000.0) for v in plan.command_step_camera_m],
        "command_step_base_mm": [float(v * 1000.0) for v in plan.command_step_base_m],
        "next_base_T_tool": plan.next_base_T_tool.to_json(),
        "within_tolerance": plan.within_tolerance,
        "snapshot_timing": snapshot_payload.get("timing", {}),
    }
    if args.check_delta_ik:
        payload["delta_ik_check"] = check_delta_ik(plan.next_base_T_tool)

    write_json(args.output, payload)
    print(f"Wrote image follow step plan: {args.output}")
    print_vector_summary("command_step_base", payload["command_step_base_mm"])
    print_transform_summary("next_base_T_tool", plan.next_base_T_tool)
    print(
        f"image_error: x={plan.image_error[0]:+.3f}, "
        f"y={plan.image_error[1]:+.3f}"
    )
    if plan.within_tolerance:
        print("image target is within tolerance")
    return 0


def compare_with_ground_truth(payload: dict[str, Any], result: dict[str, Any]) -> dict[str, Any]:
    ground_truth = payload.get("ground_truth", {})
    output: dict[str, Any] = {}
    mappings = {
        "base_camera": "base_T_base_camera",
        "object_camera": "tool_T_object_camera",
    }
    for result_key, transform_name in mappings.items():
        estimated_payload = (
            result.get("results", {})
            .get(result_key, {})
            .get("transform")
        )
        truth_payload = ground_truth.get(transform_name)
        if estimated_payload is None or truth_payload is None:
            continue
        estimated = Transform(estimated_payload["matrix"])
        truth = Transform(truth_payload["matrix"])
        output[transform_name] = transform_error(truth, estimated)
    return output


def print_summary(result: dict[str, Any]) -> None:
    for name, estimation in result.get("results", {}).items():
        transform_name = estimation.get("transform_name", name)
        residuals = estimation.get("residuals", {})
        print(
            f"{transform_name}: samples={estimation.get('sample_count')} "
            f"mean={residuals.get('translation_mean_mm', 0.0):.2f} mm, "
            f"{residuals.get('rotation_mean_deg', 0.0):.3f} deg"
        )
    for name, error in result.get("ground_truth_error", {}).items():
        print(
            f"{name} truth error: {error['translation_mm']:.2f} mm, "
            f"{error['rotation_deg']:.3f} deg"
        )
    for warning in result.get("warnings", []):
        print(f"warning: {warning}")


def print_transform_summary(name: str, transform: Transform) -> None:
    print(
        f"{name}: x={transform.t[0]:+.4f} m, "
        f"y={transform.t[1]:+.4f} m, z={transform.t[2]:+.4f} m"
    )


def load_transform_arg(path: Path | None, rpy_values: list[float] | None) -> Transform:
    if path is not None:
        payload = read_json(path)
        if "transform" in payload and isinstance(payload["transform"], dict):
            payload = payload["transform"]
        return transform_from_json(payload)
    assert rpy_values is not None
    return Transform.from_rpy_deg(
        translation=rpy_values[:3],
        rpy_deg=rpy_values[3:],
    )


def print_vector_summary(name: str, values_mm: list[float]) -> None:
    print(
        f"{name}: dx={values_mm[0]:+.2f} mm, "
        f"dy={values_mm[1]:+.2f} mm, dz={values_mm[2]:+.2f} mm"
    )


def check_delta_ik(base_T_tool: Transform) -> dict[str, Any]:
    repo_root = Path(__file__).resolve().parents[3]
    delta_path = repo_root / "Delta_Gcode_Servo"
    if str(delta_path) not in sys.path:
        sys.path.insert(0, str(delta_path))

    from delta_gcode_servo.config import robot_params
    from delta_gcode_servo.kinematics import inverse_kinematics

    point_mm = base_T_tool.t * 1000.0
    angles_rad, ok = inverse_kinematics(
        float(point_mm[0]),
        float(point_mm[1]),
        float(point_mm[2]),
        robot_params(),
    )
    return {
        "uses": "Delta_Gcode_Servo.delta_gcode_servo.kinematics.inverse_kinematics",
        "point_mm": [float(v) for v in point_mm],
        "reachable": bool(ok),
        "angles_deg": [float(v) for v in np.rad2deg(angles_rad)],
    }


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
