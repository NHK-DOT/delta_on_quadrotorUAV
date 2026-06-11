from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from .geometry import Transform, transform_from_json


TrackAxes = Literal["xy", "xyz"]


@dataclass(frozen=True)
class FollowPlan:
    base_T_object: Transform
    current_base_T_pickup: Transform
    desired_base_T_pickup: Transform
    next_base_T_tool: Transform
    error_m: np.ndarray
    command_step_m: np.ndarray
    within_tolerance: bool


@dataclass(frozen=True)
class ImageFollowPlan:
    next_base_T_tool: Transform
    image_error: np.ndarray
    command_step_camera_m: np.ndarray
    command_step_base_m: np.ndarray
    within_tolerance: bool


def estimate_base_T_tool_from_hand_tag(
    base_T_base_camera: Transform,
    base_camera_T_hand_tag: Transform,
    tool_T_hand_tag: Transform,
) -> Transform:
    return base_T_base_camera @ base_camera_T_hand_tag @ tool_T_hand_tag.inverse()


def project_object_to_base(
    base_T_tool: Transform,
    tool_T_object_camera: Transform,
    object_camera_T_object: Transform,
) -> Transform:
    return base_T_tool @ tool_T_object_camera @ object_camera_T_object


def plan_pickup_follow_step(
    *,
    base_T_tool: Transform,
    tool_T_object_camera: Transform,
    object_camera_T_object: Transform,
    tool_T_pickup: Transform,
    object_offset_base_m: np.ndarray,
    track_axes: TrackAxes,
    max_step_m: float,
    tolerance_m: float,
) -> FollowPlan:
    base_T_object = project_object_to_base(
        base_T_tool,
        tool_T_object_camera,
        object_camera_T_object,
    )
    current_base_T_pickup = base_T_tool @ tool_T_pickup
    desired_base_T_pickup = Transform.from_rt(
        current_base_T_pickup.R,
        base_T_object.t + object_offset_base_m,
    )

    error_m = desired_base_T_pickup.t - current_base_T_pickup.t
    if track_axes == "xy":
        error_m = np.array([error_m[0], error_m[1], 0.0], dtype=float)
    elif track_axes != "xyz":
        raise ValueError(f"unsupported track_axes: {track_axes}")

    command_step_m = clamp_vector_norm(error_m, max_step_m)
    next_matrix = np.array(base_T_tool.matrix, dtype=float)
    next_matrix[:3, 3] = base_T_tool.t + command_step_m
    next_base_T_tool = Transform(next_matrix)

    return FollowPlan(
        base_T_object=base_T_object,
        current_base_T_pickup=current_base_T_pickup,
        desired_base_T_pickup=desired_base_T_pickup,
        next_base_T_tool=next_base_T_tool,
        error_m=error_m,
        command_step_m=command_step_m,
        within_tolerance=float(np.linalg.norm(error_m)) <= tolerance_m,
    )


def plan_image_follow_step(
    *,
    base_T_tool: Transform,
    tool_T_object_camera: Transform,
    image_error: np.ndarray,
    gain_m_per_norm: float,
    max_step_m: float,
    tolerance_norm: float,
    lock_z: bool,
) -> ImageFollowPlan:
    camera_R_base = (base_T_tool @ tool_T_object_camera).R
    command_step_camera_m = np.array(
        [
            gain_m_per_norm * image_error[0],
            gain_m_per_norm * image_error[1],
            0.0,
        ],
        dtype=float,
    )
    command_step_base_m = camera_R_base @ command_step_camera_m
    if lock_z:
        command_step_base_m[2] = 0.0
    command_step_base_m = clamp_vector_norm(command_step_base_m, max_step_m)

    next_matrix = np.array(base_T_tool.matrix, dtype=float)
    next_matrix[:3, 3] = base_T_tool.t + command_step_base_m
    return ImageFollowPlan(
        next_base_T_tool=Transform(next_matrix),
        image_error=image_error,
        command_step_camera_m=command_step_camera_m,
        command_step_base_m=command_step_base_m,
        within_tolerance=float(np.linalg.norm(image_error)) <= tolerance_norm,
    )


def clamp_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    if max_norm <= 0:
        return np.zeros(3, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= max_norm or norm < 1e-12:
        return vector
    return vector * (max_norm / norm)


def transform_from_result(payload: dict, result_key: str) -> Transform:
    result_payload = (
        payload.get("results", {})
        .get(result_key, {})
        .get("transform")
    )
    if not isinstance(result_payload, dict):
        raise ValueError(f"calibration has no results.{result_key}.transform")
    return transform_from_json(result_payload)


def known_transform(payload: dict, name: str) -> Transform:
    known_payload = payload.get("known_transforms", {}).get(name)
    if not isinstance(known_payload, dict):
        raise ValueError(f"calibration has no known_transforms.{name}")
    return transform_from_json(known_payload)
