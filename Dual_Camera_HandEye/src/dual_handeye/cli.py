from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from .calibration import calibrate_dataset, load_dataset
from .geometry import Transform, transform_error
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

    args = parser.parse_args(argv)
    if args.command == "generate":
        return command_generate(args)
    if args.command == "calibrate":
        return command_calibrate(args)
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


def compare_with_ground_truth(payload: dict[str, Any], result: dict[str, Any]) -> dict[str, Any]:
    ground_truth = payload.get("ground_truth", {})
    output: dict[str, Any] = {}
    mappings = {
        "base_camera": "base_T_base_camera",
        "wrist_camera": "tool_T_wrist_camera",
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


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
