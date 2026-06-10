from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from .calibration import calibrate_dataset, load_dataset
from .geometry import Transform, transform_error, transform_from_json
from .snapshot import detection_transform_from_snapshot
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

    args = parser.parse_args(argv)
    if args.command == "generate":
        return command_generate(args)
    if args.command == "calibrate":
        return command_calibrate(args)
    if args.command == "snapshot-transform":
        return command_snapshot_transform(args)
    if args.command == "project-object":
        return command_project_object(args)
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
    tool_T_object_camera = transform_from_calibration(calibration, "object_camera")
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


def transform_from_calibration(payload: dict[str, Any], result_key: str) -> Transform:
    result_payload = (
        payload.get("results", {})
        .get(result_key, {})
        .get("transform")
    )
    if not isinstance(result_payload, dict):
        raise ValueError(f"calibration has no results.{result_key}.transform")
    return transform_from_json(result_payload)


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
