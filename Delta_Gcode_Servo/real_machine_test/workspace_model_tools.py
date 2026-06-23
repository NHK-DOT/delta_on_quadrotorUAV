#!/usr/bin/env python3
"""Fit Delta-arm geometry from AprilTag/servo samples and scan workspace.

This tool is intentionally offline. The realtime sampler records raw servo
feedback plus vision XYZ. This file consumes that dataset and estimates:

- Delta geometry parameters used by FK/IK.
- A constant XYZ offset between model FK and AprilTag vision coordinates.
- Optional per-servo joint angle offsets.
- A conservative grid-sampled reachable workspace.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import tempfile
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Any, Iterable

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DELTA_SERVO_ROOT = PROJECT_ROOT / "Delta_Gcode_Servo"
if str(DELTA_SERVO_ROOT) not in sys.path:
    sys.path.insert(0, str(DELTA_SERVO_ROOT))

from delta_gcode_servo.config import RobotParams, robot_params
from delta_gcode_servo.kinematics import forward_kinematics, inverse_kinematics
from delta_gcode_servo.servo_mapping import default_mapping_config_path, load_servo_mappings_for_ids


SERVO_IDS = [1, 2, 3]
DEFAULT_FIT_NAMES = [
    "l1",
    "l2",
    "l3",
    "servo_offset_x",
    "servo_offset_z",
    "vision_dx",
    "vision_dy",
    "vision_dz",
]
SERVO_OFFSET_NAMES = ["theta1_offset_deg", "theta2_offset_deg", "theta3_offset_deg"]


@dataclass(frozen=True)
class ModelSample:
    label: str
    timestamp: str
    servo_raw: dict[int, int]
    vision_xyz_mm: np.ndarray


@dataclass(frozen=True)
class RawAngleMapper:
    params: RobotParams
    servo_ids: list[int]
    reference_angles_rad: np.ndarray
    reference_servo_coords: dict[int, float]
    servo_logical_directions: dict[int, float]
    servo_units_per_degree: dict[int, float]
    mappings: dict[int, Any]

    @classmethod
    def from_params(cls, params: RobotParams) -> "RawAngleMapper":
        servo_ids = [int(value) for value in params.servo_ids[:3]]
        mappings = load_servo_mappings_for_ids(servo_ids)
        reference_position = np.asarray(params.home_position, dtype=float)
        reference_angles_rad, ok = inverse_kinematics(
            float(reference_position[0]),
            float(reference_position[1]),
            float(reference_position[2]),
            params,
        )
        if not ok:
            raise RuntimeError("home_position is not reachable with the current parameters")

        servo_raw_directions = {servo_id: -1 for servo_id in servo_ids}
        servo_logical_directions = {
            servo_id: servo_raw_directions[servo_id]
            * (1.0 if mappings[servo_id].logical_span >= 0.0 else -1.0)
            for servo_id in servo_ids
        }
        reference_raw = {
            servo_id: mappings[servo_id].reference_raw
            for servo_id in servo_ids
        }
        reference_servo_coords = {
            servo_id: mappings[servo_id].raw_to_logical(reference_raw[servo_id])
            for servo_id in servo_ids
        }
        servo_units_per_degree = {
            servo_id: mappings[servo_id].logical_units_per_degree(
                physical_min_deg=float(params.servo_physical_angle_min_deg),
                physical_max_deg=float(params.servo_physical_angle_max_deg),
            )
            for servo_id in servo_ids
        }
        return cls(
            params=params,
            servo_ids=servo_ids,
            reference_angles_rad=reference_angles_rad,
            reference_servo_coords=reference_servo_coords,
            servo_logical_directions=servo_logical_directions,
            servo_units_per_degree=servo_units_per_degree,
            mappings=mappings,
        )

    def raw_to_angles(self, servo_raw: dict[int, int], angle_offsets_deg: np.ndarray | None = None) -> np.ndarray:
        angles_rad = np.zeros(3, dtype=float)
        for index, servo_id in enumerate(self.servo_ids):
            mapping = self.mappings[servo_id]
            current_coord = mapping.raw_to_logical(servo_raw[servo_id])
            delta_coord = current_coord - self.reference_servo_coords[servo_id]
            delta_deg = delta_coord / (
                self.servo_logical_directions[servo_id] * self.servo_units_per_degree[servo_id]
            )
            angles_rad[index] = self.reference_angles_rad[index] + math.radians(delta_deg)
        if angle_offsets_deg is not None:
            angles_rad = angles_rad + np.radians(angle_offsets_deg)
        return angles_rad

    def angles_to_raw(self, angles_rad: np.ndarray) -> tuple[bool, dict[int, int], list[str]]:
        raw: dict[int, int] = {}
        errors: list[str] = []
        for index, servo_id in enumerate(self.servo_ids):
            mapping = self.mappings[servo_id]
            delta_deg = float(np.degrees(angles_rad[index] - self.reference_angles_rad[index]))
            target_coord = (
                self.reference_servo_coords[servo_id]
                + self.servo_logical_directions[servo_id] * delta_deg * self.servo_units_per_degree[servo_id]
            )
            logical_low = min(mapping.logical_min, mapping.logical_max)
            logical_high = max(mapping.logical_min, mapping.logical_max)
            if target_coord < logical_low or target_coord > logical_high:
                errors.append(
                    f"servo{servo_id} logical {target_coord:.3f} outside {logical_low:.3f}..{logical_high:.3f}"
                )
            raw_value = mapping.logical_to_raw(target_coord)
            if raw_value <= mapping.raw_low or raw_value >= mapping.raw_high:
                errors.append(f"servo{servo_id} raw {raw_value} outside {mapping.raw_low}..{mapping.raw_high}")
            raw[servo_id] = raw_value
        return not errors, raw, errors


def _finite_xyz(values: Iterable[Any]) -> np.ndarray | None:
    try:
        array = np.asarray([float(value) for value in values], dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        return None
    return array


def _parse_raw_dict(payload: Any) -> dict[int, int] | None:
    if not isinstance(payload, dict):
        return None
    raw: dict[int, int] = {}
    for servo_id in SERVO_IDS:
        value = payload.get(str(servo_id), payload.get(servo_id))
        if value is None:
            return None
        try:
            raw[servo_id] = int(round(float(value)))
        except (TypeError, ValueError):
            return None
    return raw


def _extract_vision_xyz(payload: dict[str, Any]) -> np.ndarray | None:
    for key in ("vision_tool_preview", "vision"):
        item = payload.get(key)
        if isinstance(item, dict):
            xyz = item.get("tool_position_mm")
            if isinstance(xyz, list):
                parsed = _finite_xyz(xyz)
                if parsed is not None:
                    return parsed
    xyz = payload.get("tool_position_mm")
    if isinstance(xyz, list):
        return _finite_xyz(xyz)
    return None


def load_samples(path: Path) -> list[ModelSample]:
    if path.suffix.lower() == ".csv":
        return load_samples_csv(path)
    return load_samples_jsonl(path)


def load_samples_jsonl(path: Path) -> list[ModelSample]:
    samples: list[ModelSample] = []
    with path.open("r", encoding="utf-8") as fh:
        for line_no, line in enumerate(fh, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                payload = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSONL record: {exc}") from exc
            if not isinstance(payload, dict):
                continue
            raw = _parse_raw_dict(payload.get("servo_raw"))
            xyz = _extract_vision_xyz(payload)
            if raw is None or xyz is None:
                continue
            samples.append(
                ModelSample(
                    label=str(payload.get("label", payload.get("index", f"row{line_no}"))),
                    timestamp=str(payload.get("timestamp_iso", payload.get("timestamp", ""))),
                    servo_raw=raw,
                    vision_xyz_mm=xyz,
                )
            )
    return samples


def load_samples_csv(path: Path) -> list[ModelSample]:
    samples: list[ModelSample] = []
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        for row_no, row in enumerate(reader, start=2):
            try:
                raw = {
                    1: int(round(float(row["raw1"]))),
                    2: int(round(float(row["raw2"]))),
                    3: int(round(float(row["raw3"]))),
                }
            except (KeyError, TypeError, ValueError):
                continue
            xyz = _finite_xyz([row.get("x_mm"), row.get("y_mm"), row.get("z_mm")])
            if xyz is None:
                continue
            samples.append(
                ModelSample(
                    label=str(row.get("label", f"row{row_no}")),
                    timestamp=str(row.get("timestamp_iso", row.get("timestamp", ""))),
                    servo_raw=raw,
                    vision_xyz_mm=xyz,
                )
            )
    return samples


def params_to_json(params: RobotParams) -> dict[str, Any]:
    payload = asdict(params)
    payload["servo_angle_min_deg"] = math.degrees(params.servo_angle_min)
    payload["servo_angle_max_deg"] = math.degrees(params.servo_angle_max)
    payload["ball_joint_angle_limit_deg"] = math.degrees(params.ball_joint_angle_limit)
    return payload


def vector_names(fit_servo_offsets: bool) -> list[str]:
    names = list(DEFAULT_FIT_NAMES)
    if fit_servo_offsets:
        names.extend(SERVO_OFFSET_NAMES)
    return names


def initial_vector(params: RobotParams, fit_servo_offsets: bool) -> np.ndarray:
    values = [
        params.l1,
        params.l2,
        params.l3,
        params.servo_offset_x,
        params.servo_offset_z,
        0.0,
        0.0,
        0.0,
    ]
    if fit_servo_offsets:
        values.extend([0.0, 0.0, 0.0])
    return np.asarray(values, dtype=float)


def vector_bounds(fit_servo_offsets: bool) -> tuple[np.ndarray, np.ndarray]:
    low = [40.0, 60.0, 0.0, 20.0, -120.0, -300.0, -300.0, -300.0]
    high = [260.0, 340.0, 140.0, 180.0, 180.0, 300.0, 300.0, 300.0]
    if fit_servo_offsets:
        low.extend([-30.0, -30.0, -30.0])
        high.extend([30.0, 30.0, 30.0])
    return np.asarray(low, dtype=float), np.asarray(high, dtype=float)


def vector_to_model(base_params: RobotParams, vector: np.ndarray, fit_servo_offsets: bool) -> tuple[RobotParams, np.ndarray, np.ndarray]:
    params = replace(
        base_params,
        l1=float(vector[0]),
        l2=float(vector[1]),
        l3=float(vector[2]),
        servo_offset_x=float(vector[3]),
        servo_offset_z=float(vector[4]),
    )
    vision_offset = np.asarray(vector[5:8], dtype=float)
    if fit_servo_offsets:
        angle_offsets_deg = np.asarray(vector[8:11], dtype=float)
    else:
        angle_offsets_deg = np.zeros(3, dtype=float)
    return params, vision_offset, angle_offsets_deg


def model_residuals(
    vector: np.ndarray,
    *,
    samples: list[ModelSample],
    base_params: RobotParams,
    fit_servo_offsets: bool,
    invalid_penalty_mm: float = 1000.0,
) -> np.ndarray:
    params, vision_offset, angle_offsets_deg = vector_to_model(base_params, vector, fit_servo_offsets)
    residuals: list[float] = []
    try:
        mapper = RawAngleMapper.from_params(params)
    except Exception:
        return np.full(len(samples) * 3, invalid_penalty_mm, dtype=float)

    for sample in samples:
        try:
            angles_rad = mapper.raw_to_angles(sample.servo_raw, angle_offsets_deg)
            fk_xyz, ok = forward_kinematics(
                float(angles_rad[0]),
                float(angles_rad[1]),
                float(angles_rad[2]),
                params,
            )
        except Exception:
            ok = False
            fk_xyz = np.zeros(3, dtype=float)
        if not ok:
            residuals.extend([invalid_penalty_mm, invalid_penalty_mm, invalid_penalty_mm])
            continue
        residuals.extend((fk_xyz + vision_offset - sample.vision_xyz_mm).tolist())
    return np.asarray(residuals, dtype=float)


def rms(values: np.ndarray) -> float:
    if values.size == 0:
        return float("nan")
    return float(math.sqrt(float(np.mean(np.square(values)))))


def pattern_search_fit(
    *,
    samples: list[ModelSample],
    base_params: RobotParams,
    fit_servo_offsets: bool,
    max_iter: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    low, high = vector_bounds(fit_servo_offsets)
    x = np.clip(initial_vector(base_params, fit_servo_offsets), low, high)
    step_values = [8.0, 8.0, 4.0, 4.0, 4.0, 8.0, 8.0, 8.0]
    if fit_servo_offsets:
        step_values.extend([1.5, 1.5, 1.5])
    step = np.asarray(step_values, dtype=float)

    def cost(v: np.ndarray) -> float:
        return rms(model_residuals(v, samples=samples, base_params=base_params, fit_servo_offsets=fit_servo_offsets))

    best = cost(x)
    iterations = 0
    for iterations in range(1, max_iter + 1):
        improved = False
        for index in range(len(x)):
            for direction in (1.0, -1.0):
                candidate = x.copy()
                candidate[index] = np.clip(candidate[index] + direction * step[index], low[index], high[index])
                candidate_cost = cost(candidate)
                if candidate_cost + 1e-9 < best:
                    x = candidate
                    best = candidate_cost
                    improved = True
                    break
            if improved:
                break
        if not improved:
            step *= 0.55
            if float(np.max(step)) < 0.05:
                break

    return x, {
        "optimizer": "pattern_search_no_scipy",
        "iterations": iterations,
        "final_step": step.tolist(),
        "final_rms_mm": best,
    }


def scipy_fit(
    *,
    samples: list[ModelSample],
    base_params: RobotParams,
    fit_servo_offsets: bool,
    max_iter: int,
) -> tuple[np.ndarray, dict[str, Any]]:
    from scipy.optimize import least_squares  # type: ignore

    low, high = vector_bounds(fit_servo_offsets)
    x0 = np.clip(initial_vector(base_params, fit_servo_offsets), low, high)
    result = least_squares(
        lambda vector: model_residuals(
            vector,
            samples=samples,
            base_params=base_params,
            fit_servo_offsets=fit_servo_offsets,
        ),
        x0=x0,
        bounds=(low, high),
        max_nfev=max_iter,
        x_scale="jac",
        loss="soft_l1",
        f_scale=10.0,
    )
    return result.x, {
        "optimizer": "scipy.optimize.least_squares",
        "success": bool(result.success),
        "status": int(result.status),
        "message": str(result.message),
        "nfev": int(result.nfev),
        "cost": float(result.cost),
        "final_rms_mm": rms(result.fun),
    }


def fit_model(
    *,
    samples: list[ModelSample],
    fit_servo_offsets: bool,
    max_iter: int,
    no_scipy: bool,
) -> tuple[np.ndarray, dict[str, Any]]:
    base_params = robot_params()
    if len(samples) < 4:
        raise ValueError("At least 4 valid samples are required for a useful fit")
    if not no_scipy:
        try:
            return scipy_fit(
                samples=samples,
                base_params=base_params,
                fit_servo_offsets=fit_servo_offsets,
                max_iter=max_iter,
            )
        except ImportError:
            pass
    return pattern_search_fit(
        samples=samples,
        base_params=base_params,
        fit_servo_offsets=fit_servo_offsets,
        max_iter=max_iter,
    )


def evaluate_model(
    *,
    samples: list[ModelSample],
    base_params: RobotParams,
    vector: np.ndarray,
    fit_servo_offsets: bool,
) -> dict[str, Any]:
    params, vision_offset, angle_offsets_deg = vector_to_model(base_params, vector, fit_servo_offsets)
    mapper = RawAngleMapper.from_params(params)
    rows: list[dict[str, Any]] = []
    errors: list[float] = []
    invalid_count = 0
    for sample in samples:
        angles_rad = mapper.raw_to_angles(sample.servo_raw, angle_offsets_deg)
        fk_xyz, ok = forward_kinematics(float(angles_rad[0]), float(angles_rad[1]), float(angles_rad[2]), params)
        if not ok:
            invalid_count += 1
            continue
        predicted_vision_xyz = fk_xyz + vision_offset
        residual = predicted_vision_xyz - sample.vision_xyz_mm
        error_norm = float(np.linalg.norm(residual))
        errors.append(error_norm)
        rows.append(
            {
                "label": sample.label,
                "timestamp": sample.timestamp,
                "raw1": sample.servo_raw[1],
                "raw2": sample.servo_raw[2],
                "raw3": sample.servo_raw[3],
                "vision_x_mm": float(sample.vision_xyz_mm[0]),
                "vision_y_mm": float(sample.vision_xyz_mm[1]),
                "vision_z_mm": float(sample.vision_xyz_mm[2]),
                "fk_x_mm": float(fk_xyz[0]),
                "fk_y_mm": float(fk_xyz[1]),
                "fk_z_mm": float(fk_xyz[2]),
                "predicted_vision_x_mm": float(predicted_vision_xyz[0]),
                "predicted_vision_y_mm": float(predicted_vision_xyz[1]),
                "predicted_vision_z_mm": float(predicted_vision_xyz[2]),
                "residual_x_mm": float(residual[0]),
                "residual_y_mm": float(residual[1]),
                "residual_z_mm": float(residual[2]),
                "residual_norm_mm": error_norm,
            }
        )
    error_array = np.asarray(errors, dtype=float)
    return {
        "sample_count": len(samples),
        "valid_fk_count": len(rows),
        "invalid_fk_count": invalid_count,
        "rms_residual_norm_mm": rms(error_array),
        "mean_residual_norm_mm": float(np.mean(error_array)) if error_array.size else None,
        "max_residual_norm_mm": float(np.max(error_array)) if error_array.size else None,
        "rows": rows,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
        fh.write("\n")


def model_payload(
    *,
    vector: np.ndarray,
    fit_servo_offsets: bool,
    optimizer: dict[str, Any],
    evaluation: dict[str, Any],
) -> dict[str, Any]:
    base_params = robot_params()
    params, vision_offset, angle_offsets_deg = vector_to_model(base_params, vector, fit_servo_offsets)
    names = vector_names(fit_servo_offsets)
    return {
        "created_unix": time.time(),
        "units": "mm/deg/raw",
        "fit_names": names,
        "fit_vector": {name: float(value) for name, value in zip(names, vector, strict=True)},
        "base_params_before_fit": params_to_json(base_params),
        "fitted_params": params_to_json(params),
        "vision_offset_model_plus_offset_to_vision_mm": [float(value) for value in vision_offset],
        "servo_angle_offsets_deg": [float(value) for value in angle_offsets_deg],
        "optimizer": optimizer,
        "evaluation": {key: value for key, value in evaluation.items() if key != "rows"},
        "raw_mapping_config": str(default_mapping_config_path()),
        "controller_patch_hint": {
            "file": "Delta_Gcode_Servo/delta_gcode_servo/config.py",
            "fields": {
                "l1": float(params.l1),
                "l2": float(params.l2),
                "l3": float(params.l3),
                "servo_offset_x": float(params.servo_offset_x),
                "servo_offset_z": float(params.servo_offset_z),
            },
            "note": "Apply only after checking residuals and physical clearance. Keep workspace bounds conservative.",
        },
    }


def load_model_report(path: Path) -> tuple[RobotParams, np.ndarray, np.ndarray]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    params_data = payload.get("fitted_params")
    if not isinstance(params_data, dict):
        raise ValueError(f"{path} does not contain fitted_params")
    base = robot_params()
    params = replace(
        base,
        l1=float(params_data.get("l1", base.l1)),
        l2=float(params_data.get("l2", base.l2)),
        l3=float(params_data.get("l3", base.l3)),
        servo_offset_x=float(params_data.get("servo_offset_x", base.servo_offset_x)),
        servo_offset_z=float(params_data.get("servo_offset_z", base.servo_offset_z)),
    )
    vision_offset = np.asarray(
        payload.get("vision_offset_model_plus_offset_to_vision_mm", [0.0, 0.0, 0.0]),
        dtype=float,
    )
    angle_offsets = np.asarray(payload.get("servo_angle_offsets_deg", [0.0, 0.0, 0.0]), dtype=float)
    return params, vision_offset, angle_offsets


def scan_workspace(
    *,
    params: RobotParams,
    step_mm: float,
    x_limit_mm: float | None = None,
    y_limit_mm: float | None = None,
    z_min_mm: float | None = None,
    z_max_mm: float | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    mapper = RawAngleMapper.from_params(params)
    x_limit = float(params.workspace_xy_max if x_limit_mm is None else x_limit_mm)
    y_limit = float(params.workspace_xy_max if y_limit_mm is None else y_limit_mm)
    z_min = float(params.workspace_z_min if z_min_mm is None else z_min_mm)
    z_max = float(params.workspace_z_max if z_max_mm is None else z_max_mm)
    step = max(1.0, float(step_mm))
    rows: list[dict[str, Any]] = []
    total = 0
    x_values = np.arange(-x_limit, x_limit + 0.5 * step, step)
    y_values = np.arange(-y_limit, y_limit + 0.5 * step, step)
    z_values = np.arange(z_min, z_max + 0.5 * step, step)
    for z in z_values:
        for y in y_values:
            for x in x_values:
                total += 1
                if math.hypot(float(x), float(y)) > max(x_limit, y_limit):
                    continue
                angles_rad, ok = inverse_kinematics(float(x), float(y), float(z), params)
                if not ok:
                    continue
                raw_ok, raw, errors = mapper.angles_to_raw(angles_rad)
                if not raw_ok:
                    continue
                rows.append(
                    {
                        "x_mm": round(float(x), 6),
                        "y_mm": round(float(y), 6),
                        "z_mm": round(float(z), 6),
                        "xy_radius_mm": round(float(math.hypot(float(x), float(y))), 6),
                        "theta1_deg": round(float(math.degrees(angles_rad[0])), 6),
                        "theta2_deg": round(float(math.degrees(angles_rad[1])), 6),
                        "theta3_deg": round(float(math.degrees(angles_rad[2])), 6),
                        "raw1": raw[1],
                        "raw2": raw[2],
                        "raw3": raw[3],
                        "raw_limit_errors": ";".join(errors),
                    }
                )

    if rows:
        xs = np.asarray([row["x_mm"] for row in rows], dtype=float)
        ys = np.asarray([row["y_mm"] for row in rows], dtype=float)
        zs = np.asarray([row["z_mm"] for row in rows], dtype=float)
        radii = np.asarray([row["xy_radius_mm"] for row in rows], dtype=float)
        per_z: dict[str, float] = {}
        for z in sorted(set(float(row["z_mm"]) for row in rows)):
            per_z[f"{z:.3f}"] = float(
                max(float(row["xy_radius_mm"]) for row in rows if abs(float(row["z_mm"]) - z) < 1e-9)
            )
        margin = step
        suggested_xy = max(0.0, float(np.max(radii)) - margin)
        suggested_z_min = float(np.min(zs)) + margin
        suggested_z_max = float(np.max(zs)) - margin
        bounds = {
            "x_min_mm": float(np.min(xs)),
            "x_max_mm": float(np.max(xs)),
            "y_min_mm": float(np.min(ys)),
            "y_max_mm": float(np.max(ys)),
            "z_min_mm": float(np.min(zs)),
            "z_max_mm": float(np.max(zs)),
            "xy_radius_max_mm": float(np.max(radii)),
            "suggested_controller_bounds": {
                "workspace_xy_max": suggested_xy,
                "workspace_z_min": min(suggested_z_min, suggested_z_max),
                "workspace_z_max": max(suggested_z_min, suggested_z_max),
                "margin_mm": margin,
            },
            "xy_radius_max_by_z_mm": per_z,
        }
    else:
        bounds = {
            "x_min_mm": None,
            "x_max_mm": None,
            "y_min_mm": None,
            "y_max_mm": None,
            "z_min_mm": None,
            "z_max_mm": None,
            "xy_radius_max_mm": None,
            "suggested_controller_bounds": None,
            "xy_radius_max_by_z_mm": {},
        }
    summary = {
        "created_unix": time.time(),
        "units": "mm/deg/raw",
        "grid_step_mm": step,
        "search_bounds": {
            "x": [-x_limit, x_limit],
            "y": [-y_limit, y_limit],
            "z": [z_min, z_max],
        },
        "total_grid_points": total,
        "reachable_points": len(rows),
        "reachable_fraction": (len(rows) / total) if total else 0.0,
        "bounds": bounds,
        "params": params_to_json(params),
        "raw_mapping_config": str(default_mapping_config_path()),
    }
    return rows, summary


def command_fit(args: argparse.Namespace) -> int:
    samples = load_samples(args.samples)
    if len(samples) < 4:
        print(f"Need at least 4 valid samples, got {len(samples)} from {args.samples}", file=sys.stderr)
        return 2
    vector, optimizer = fit_model(
        samples=samples,
        fit_servo_offsets=args.fit_servo_offsets,
        max_iter=args.max_iter,
        no_scipy=args.no_scipy,
    )
    base_params = robot_params()
    evaluation = evaluate_model(
        samples=samples,
        base_params=base_params,
        vector=vector,
        fit_servo_offsets=args.fit_servo_offsets,
    )
    payload = model_payload(
        vector=vector,
        fit_servo_offsets=args.fit_servo_offsets,
        optimizer=optimizer,
        evaluation=evaluation,
    )
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    fit_report_path = output_dir / "fit_report.json"
    residuals_path = output_dir / "fit_residuals.csv"
    write_json(fit_report_path, payload)
    write_csv(residuals_path, evaluation["rows"])
    print(f"Wrote fit report: {fit_report_path}")
    print(f"Wrote residuals:  {residuals_path}")
    print(
        "Fit summary: "
        f"samples={evaluation['sample_count']}, "
        f"valid_fk={evaluation['valid_fk_count']}, "
        f"rms_norm={evaluation['rms_residual_norm_mm']:.3f} mm, "
        f"max_norm={evaluation['max_residual_norm_mm']:.3f} mm"
    )
    if args.compute_workspace:
        params, _vision_offset, _angle_offsets = load_model_report(fit_report_path)
        rows, summary = scan_workspace(
            params=params,
            step_mm=args.workspace_step_mm,
            x_limit_mm=args.workspace_xy_limit_mm,
            y_limit_mm=args.workspace_xy_limit_mm,
            z_min_mm=args.workspace_z_min_mm,
            z_max_mm=args.workspace_z_max_mm,
        )
        write_csv(output_dir / "workspace_grid.csv", rows)
        write_json(output_dir / "workspace_summary.json", summary)
        print(f"Wrote workspace grid: {output_dir / 'workspace_grid.csv'}")
        print(f"Wrote workspace summary: {output_dir / 'workspace_summary.json'}")
    return 0


def command_workspace(args: argparse.Namespace) -> int:
    params, _vision_offset, _angle_offsets = load_model_report(args.model)
    rows, summary = scan_workspace(
        params=params,
        step_mm=args.step_mm,
        x_limit_mm=args.workspace_xy_limit_mm,
        y_limit_mm=args.workspace_xy_limit_mm,
        z_min_mm=args.workspace_z_min_mm,
        z_max_mm=args.workspace_z_max_mm,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "workspace_grid.csv", rows)
    write_json(args.output_dir / "workspace_summary.json", summary)
    print(f"Reachable points: {summary['reachable_points']} / {summary['total_grid_points']}")
    print(f"Wrote: {args.output_dir / 'workspace_grid.csv'}")
    print(f"Wrote: {args.output_dir / 'workspace_summary.json'}")
    return 0


def build_synthetic_samples() -> list[ModelSample]:
    params = robot_params()
    mapper = RawAngleMapper.from_params(params)
    points = [
        np.array([0.0, 0.0, 240.0]),
        np.array([25.0, 0.0, 230.0]),
        np.array([-25.0, 0.0, 230.0]),
        np.array([0.0, 25.0, 225.0]),
        np.array([0.0, -25.0, 225.0]),
        np.array([35.0, 20.0, 215.0]),
        np.array([-35.0, -20.0, 215.0]),
        np.array([15.0, -35.0, 205.0]),
    ]
    offset = np.array([5.0, -3.0, 2.0], dtype=float)
    samples: list[ModelSample] = []
    for index, point in enumerate(points, start=1):
        angles_rad, ok = inverse_kinematics(float(point[0]), float(point[1]), float(point[2]), params)
        if not ok:
            continue
        raw_ok, raw, _errors = mapper.angles_to_raw(angles_rad)
        if not raw_ok:
            continue
        samples.append(
            ModelSample(
                label=f"synthetic_{index}",
                timestamp="",
                servo_raw=raw,
                vision_xyz_mm=point + offset,
            )
        )
    return samples


def command_self_test(args: argparse.Namespace) -> int:
    samples = build_synthetic_samples()
    with tempfile.TemporaryDirectory() as tmp:
        output_dir = Path(tmp)
        vector, optimizer = fit_model(
            samples=samples,
            fit_servo_offsets=False,
            max_iter=80,
            no_scipy=args.no_scipy,
        )
        evaluation = evaluate_model(
            samples=samples,
            base_params=robot_params(),
            vector=vector,
            fit_servo_offsets=False,
        )
        params, _vision_offset, _angle_offsets = vector_to_model(robot_params(), vector, False)
        rows, summary = scan_workspace(params=params, step_mm=25.0)
        write_json(output_dir / "fit_report.json", model_payload(
            vector=vector,
            fit_servo_offsets=False,
            optimizer=optimizer,
            evaluation=evaluation,
        ))
        write_csv(output_dir / "workspace_grid.csv", rows)
        print(
            "self-test ok: "
            f"samples={len(samples)}, "
            f"rms_norm={evaluation['rms_residual_norm_mm']:.3f} mm, "
            f"reachable={summary['reachable_points']}"
        )
    return 0


def add_workspace_bounds_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--workspace-xy-limit-mm", type=float, default=None)
    parser.add_argument("--workspace-z-min-mm", type=float, default=None)
    parser.add_argument("--workspace-z-max-mm", type=float, default=None)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    fit_parser = subparsers.add_parser("fit", help="Fit model from samples.csv or samples.jsonl")
    fit_parser.add_argument("samples", type=Path)
    fit_parser.add_argument("--output-dir", type=Path, default=Path(__file__).with_name("workspace_model_output"))
    fit_parser.add_argument("--max-iter", type=int, default=400)
    fit_parser.add_argument("--fit-servo-offsets", action="store_true")
    fit_parser.add_argument("--no-scipy", action="store_true")
    fit_parser.add_argument("--compute-workspace", action="store_true")
    fit_parser.add_argument("--workspace-step-mm", type=float, default=5.0)
    add_workspace_bounds_args(fit_parser)
    fit_parser.set_defaults(func=command_fit)

    workspace_parser = subparsers.add_parser("workspace", help="Scan reachable workspace from fit_report.json")
    workspace_parser.add_argument("--model", type=Path, required=True)
    workspace_parser.add_argument("--output-dir", type=Path, default=Path(__file__).with_name("workspace_model_output"))
    workspace_parser.add_argument("--step-mm", type=float, default=5.0)
    add_workspace_bounds_args(workspace_parser)
    workspace_parser.set_defaults(func=command_workspace)

    self_test_parser = subparsers.add_parser("self-test", help="Run a synthetic dataset smoke test")
    self_test_parser.add_argument("--no-scipy", action="store_true")
    self_test_parser.set_defaults(func=command_self_test)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
