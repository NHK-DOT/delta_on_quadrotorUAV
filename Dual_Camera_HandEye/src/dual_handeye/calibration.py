from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .geometry import Transform, average_transforms, transform_error, transform_from_json


@dataclass
class HandEyeDataset:
    units: str
    known_transforms: dict[str, Transform]
    samples: list[dict[str, Any]]


def load_dataset(payload: dict[str, Any]) -> HandEyeDataset:
    units = str(payload.get("units", "m"))
    if units != "m":
        raise ValueError("Only meter units are supported in this demo. Convert input data to meters first.")
    known = {
        name: transform_from_json(value)
        for name, value in payload.get("known_transforms", {}).items()
    }
    samples = list(payload.get("samples", []))
    if not samples:
        raise ValueError("dataset contains no samples")
    return HandEyeDataset(units=units, known_transforms=known, samples=samples)


def sample_transform(sample: dict[str, Any], path: list[str]) -> Transform | None:
    cursor: Any = sample
    for key in path:
        if not isinstance(cursor, dict) or key not in cursor:
            return None
        cursor = cursor[key]
    if not isinstance(cursor, dict):
        return None
    return transform_from_json(cursor)


def estimate_base_camera(dataset: HandEyeDataset) -> dict[str, Any] | None:
    tool_T_hand_tag = dataset.known_transforms.get("tool_T_hand_tag")
    if tool_T_hand_tag is None:
        return None

    estimates: list[Transform] = []
    used_samples: list[str] = []
    for index, sample in enumerate(dataset.samples):
        base_T_tool = sample_transform(sample, ["base_T_tool"])
        camera_T_hand_tag = sample_transform(sample, ["base_camera", "camera_T_hand_tag"])
        if base_T_tool is None or camera_T_hand_tag is None:
            continue
        estimates.append(base_T_tool @ tool_T_hand_tag @ camera_T_hand_tag.inverse())
        used_samples.append(str(sample.get("name", f"sample_{index:03d}")))

    if not estimates:
        return None

    base_T_base_camera = average_transforms(estimates)
    errors = [transform_error(base_T_base_camera, estimate) for estimate in estimates]
    return {
        "transform_name": "base_T_base_camera",
        "sample_count": len(estimates),
        "used_samples": used_samples,
        "transform": base_T_base_camera,
        "residuals": summarize_errors(errors),
    }


def estimate_wrist_camera_direct(dataset: HandEyeDataset) -> dict[str, Any] | None:
    base_T_base_tag = dataset.known_transforms.get("base_T_base_tag")
    if base_T_base_tag is None:
        return None

    estimates: list[Transform] = []
    used_samples: list[str] = []
    for index, sample in enumerate(dataset.samples):
        base_T_tool = sample_transform(sample, ["base_T_tool"])
        camera_T_base_tag = sample_transform(sample, ["wrist_camera", "camera_T_base_tag"])
        if base_T_tool is None or camera_T_base_tag is None:
            continue
        estimates.append(base_T_tool.inverse() @ base_T_base_tag @ camera_T_base_tag.inverse())
        used_samples.append(str(sample.get("name", f"sample_{index:03d}")))

    if not estimates:
        return None

    tool_T_wrist_camera = average_transforms(estimates)
    errors = [transform_error(tool_T_wrist_camera, estimate) for estimate in estimates]
    return {
        "transform_name": "tool_T_wrist_camera",
        "method": "direct_known_base_tag",
        "sample_count": len(estimates),
        "used_samples": used_samples,
        "transform": tool_T_wrist_camera,
        "residuals": summarize_errors(errors),
    }


def estimate_wrist_camera_handeye(dataset: HandEyeDataset) -> dict[str, Any] | None:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("opencv-contrib-python is required for --wrist-method handeye") from exc

    base_T_tools: list[Transform] = []
    camera_T_targets: list[Transform] = []
    used_samples: list[str] = []
    for index, sample in enumerate(dataset.samples):
        base_T_tool = sample_transform(sample, ["base_T_tool"])
        camera_T_base_tag = sample_transform(sample, ["wrist_camera", "camera_T_base_tag"])
        if base_T_tool is None or camera_T_base_tag is None:
            continue
        base_T_tools.append(base_T_tool)
        camera_T_targets.append(camera_T_base_tag)
        used_samples.append(str(sample.get("name", f"sample_{index:03d}")))

    if len(base_T_tools) < 6:
        return None

    rotation_span_deg = estimate_rotation_span_deg(base_T_tools)
    if rotation_span_deg < 10.0:
        raise RuntimeError(
            "Wrist hand-eye is poorly observable: tool rotation span is "
            f"{rotation_span_deg:.2f} deg. For a Delta arm with fixed tool orientation, "
            "use the direct known-base-tag method instead."
        )

    r_gripper2base = [tf.R for tf in base_T_tools]
    t_gripper2base = [tf.t.reshape(3, 1) for tf in base_T_tools]
    r_target2cam = [tf.R for tf in camera_T_targets]
    t_target2cam = [tf.t.reshape(3, 1) for tf in camera_T_targets]

    r_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    tool_T_wrist_camera = Transform.from_rt(r_cam2gripper, np.asarray(t_cam2gripper).reshape(3))

    if "base_T_base_tag" in dataset.known_transforms:
        base_T_base_tag = dataset.known_transforms["base_T_base_tag"]
        errors = []
        for base_T_tool, camera_T_target in zip(base_T_tools, camera_T_targets):
            predicted = (base_T_tool @ tool_T_wrist_camera).inverse() @ base_T_base_tag
            errors.append(transform_error(predicted, camera_T_target))
    else:
        base_T_targets = [
            base_T_tool @ tool_T_wrist_camera @ camera_T_target
            for base_T_tool, camera_T_target in zip(base_T_tools, camera_T_targets)
        ]
        avg_target = average_transforms(base_T_targets)
        errors = [transform_error(avg_target, estimate) for estimate in base_T_targets]

    return {
        "transform_name": "tool_T_wrist_camera",
        "method": "opencv_calibrateHandEye",
        "sample_count": len(base_T_tools),
        "used_samples": used_samples,
        "tool_rotation_span_deg": rotation_span_deg,
        "transform": tool_T_wrist_camera,
        "residuals": summarize_errors(errors),
    }


def estimate_rotation_span_deg(transforms: list[Transform]) -> float:
    if len(transforms) < 2:
        return 0.0
    from .geometry import rotation_angle

    max_angle = 0.0
    for i, a in enumerate(transforms):
        for b in transforms[i + 1 :]:
            angle = rotation_angle(a.R.T @ b.R)
            max_angle = max(max_angle, angle)
    return float(np.rad2deg(max_angle))


def summarize_errors(errors: list[dict[str, float]]) -> dict[str, float]:
    if not errors:
        return {}
    translation_mm = np.asarray([err["translation_mm"] for err in errors], dtype=float)
    rotation_deg = np.asarray([err["rotation_deg"] for err in errors], dtype=float)
    return {
        "translation_mean_mm": float(translation_mm.mean()),
        "translation_max_mm": float(translation_mm.max()),
        "rotation_mean_deg": float(rotation_deg.mean()),
        "rotation_max_deg": float(rotation_deg.max()),
    }


def calibrate_dataset(dataset: HandEyeDataset, wrist_method: str = "direct") -> dict[str, Any]:
    result: dict[str, Any] = {
        "units": dataset.units,
        "sample_count": len(dataset.samples),
        "results": {},
        "warnings": [],
    }

    base_camera = estimate_base_camera(dataset)
    if base_camera is None:
        result["warnings"].append(
            "Skipped base camera calibration: need known_transforms.tool_T_hand_tag "
            "and base_camera.camera_T_hand_tag samples."
        )
    else:
        result["results"]["base_camera"] = serialize_estimation(base_camera)

    if wrist_method == "direct":
        wrist_camera = estimate_wrist_camera_direct(dataset)
    elif wrist_method == "handeye":
        wrist_camera = estimate_wrist_camera_handeye(dataset)
    else:
        raise ValueError(f"unknown wrist_method: {wrist_method}")

    if wrist_camera is None:
        result["warnings"].append(
            "Skipped wrist camera calibration: need known_transforms.base_T_base_tag "
            "and wrist_camera.camera_T_base_tag samples, or use --wrist-method handeye."
        )
    else:
        result["results"]["wrist_camera"] = serialize_estimation(wrist_camera)

    return result


def serialize_estimation(estimation: dict[str, Any]) -> dict[str, Any]:
    serialized: dict[str, Any] = {}
    for key, value in estimation.items():
        if isinstance(value, Transform):
            serialized[key] = value.to_json()
        else:
            serialized[key] = value
    return serialized
