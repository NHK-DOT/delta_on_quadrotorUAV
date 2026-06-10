from __future__ import annotations

import math
from typing import Any

import numpy as np

from .geometry import Transform, add_noise


def build_synthetic_dataset(
    sample_count: int = 24,
    seed: int = 78,
    translation_noise_m: float = 0.0015,
    rotation_noise_deg: float = 0.25,
) -> dict[str, Any]:
    rng = np.random.default_rng(seed)

    base_T_base_camera = Transform.from_rpy_deg(
        translation=[0.28, -0.36, 0.42],
        rpy_deg=[118.0, 0.0, 38.0],
    )
    tool_T_hand_tag = Transform.from_rpy_deg(
        translation=[0.0, 0.0, -0.035],
        rpy_deg=[0.0, 0.0, 0.0],
    )
    tool_T_object_camera = Transform.from_rpy_deg(
        translation=[0.045, -0.012, 0.032],
        rpy_deg=[0.0, -58.0, 3.0],
    )
    base_T_object = Transform.from_rpy_deg(
        translation=[0.06, -0.04, -0.315],
        rpy_deg=[0.0, 0.0, 22.0],
    )

    samples = []
    for i in range(sample_count):
        base_T_tool = make_delta_like_tool_pose(i, sample_count)
        base_camera_T_hand_tag = (
            base_T_base_camera.inverse() @ base_T_tool @ tool_T_hand_tag
        )
        object_camera_T_object = (
            (base_T_tool @ tool_T_object_camera).inverse() @ base_T_object
        )

        base_camera_T_hand_tag = add_noise(
            base_camera_T_hand_tag, translation_noise_m, rotation_noise_deg, rng
        )
        object_camera_T_object = add_noise(
            object_camera_T_object, translation_noise_m, rotation_noise_deg, rng
        )

        samples.append(
            {
                "name": f"p{i + 1:03d}",
                "base_T_tool": base_T_tool.to_json(),
                "base_camera": {
                    "camera_T_hand_tag": base_camera_T_hand_tag.to_json(),
                },
                "object_camera": {
                    "camera_T_object": object_camera_T_object.to_json(),
                },
            }
        )

    return {
        "units": "m",
        "description": "Synthetic dual-camera hand-eye dataset for 78arm.",
        "ground_truth": {
            "base_T_base_camera": base_T_base_camera.to_json(),
            "tool_T_object_camera": tool_T_object_camera.to_json(),
            "base_T_object": base_T_object.to_json(),
        },
        "known_transforms": {
            "tool_T_hand_tag": tool_T_hand_tag.to_json(),
            "tool_T_object_camera": tool_T_object_camera.to_json(),
        },
        "samples": samples,
    }


def make_delta_like_tool_pose(index: int, count: int) -> Transform:
    angle = 2.0 * math.pi * index / max(1, count)
    ring = 0.075 + 0.025 * math.sin(3.0 * angle)
    x = ring * math.cos(angle)
    y = ring * math.sin(angle)
    z = -0.25 - 0.055 * (0.5 + 0.5 * math.sin(2.0 * angle + 0.4))

    # Keep rotation constant to mimic a Delta arm end effector.
    return Transform.from_rpy_deg(
        translation=[x, y, z],
        rpy_deg=[0.0, 0.0, 0.0],
    )
