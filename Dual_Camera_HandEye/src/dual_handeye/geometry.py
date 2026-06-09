from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np


EPS = 1e-12


@dataclass(frozen=True)
class Transform:
    matrix: np.ndarray

    def __post_init__(self) -> None:
        matrix = np.asarray(self.matrix, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(f"Transform matrix must be 4x4, got {matrix.shape}")
        object.__setattr__(self, "matrix", matrix)

    @staticmethod
    def identity() -> "Transform":
        return Transform(np.eye(4, dtype=float))

    @staticmethod
    def from_rt(rotation: np.ndarray, translation: Iterable[float]) -> "Transform":
        t = np.asarray(list(translation), dtype=float)
        if t.shape != (3,):
            raise ValueError("translation must contain exactly 3 values")
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = np.asarray(rotation, dtype=float)
        matrix[:3, 3] = t
        return Transform(matrix)

    @staticmethod
    def from_rpy_deg(translation: Iterable[float], rpy_deg: Iterable[float]) -> "Transform":
        rpy = np.deg2rad(np.asarray(list(rpy_deg), dtype=float))
        if rpy.shape != (3,):
            raise ValueError("rotation_rpy_deg must contain exactly 3 values")
        return Transform.from_rt(rpy_to_matrix(rpy[0], rpy[1], rpy[2]), translation)

    @staticmethod
    def from_quaternion_xyzw(
        translation: Iterable[float], quaternion_xyzw: Iterable[float]
    ) -> "Transform":
        return Transform.from_rt(
            quaternion_xyzw_to_matrix(np.asarray(list(quaternion_xyzw), dtype=float)),
            translation,
        )

    @property
    def R(self) -> np.ndarray:
        return self.matrix[:3, :3]

    @property
    def t(self) -> np.ndarray:
        return self.matrix[:3, 3]

    def inverse(self) -> "Transform":
        inv = np.eye(4, dtype=float)
        inv[:3, :3] = self.R.T
        inv[:3, 3] = -self.R.T @ self.t
        return Transform(inv)

    def __matmul__(self, other: "Transform") -> "Transform":
        return Transform(self.matrix @ other.matrix)

    def to_json(self) -> dict[str, Any]:
        return {
            "translation": [float(x) for x in self.t],
            "rotation_rpy_deg": [float(x) for x in matrix_to_rpy_deg(self.R)],
            "quaternion_xyzw": [float(x) for x in matrix_to_quaternion_xyzw(self.R)],
            "matrix": [[float(v) for v in row] for row in self.matrix],
        }


def transform_from_json(payload: dict[str, Any]) -> Transform:
    if "matrix" in payload:
        return Transform(np.asarray(payload["matrix"], dtype=float))
    translation = payload.get("translation", [0.0, 0.0, 0.0])
    if "quaternion_xyzw" in payload:
        return Transform.from_quaternion_xyzw(translation, payload["quaternion_xyzw"])
    if "rotation_rpy_deg" in payload:
        return Transform.from_rpy_deg(translation, payload["rotation_rpy_deg"])
    if "rotation_matrix" in payload:
        return Transform.from_rt(np.asarray(payload["rotation_matrix"], dtype=float), translation)
    raise ValueError("transform must contain matrix, quaternion_xyzw, rotation_rpy_deg, or rotation_matrix")


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def matrix_to_rpy_deg(rotation: np.ndarray) -> np.ndarray:
    r = np.asarray(rotation, dtype=float)
    sy = -r[2, 0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        pitch = math.atan2(sy, cy)
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        pitch = math.atan2(sy, cy)
        yaw = 0.0
    return np.rad2deg(np.array([roll, pitch, yaw], dtype=float))


def quaternion_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    if q.shape != (4,):
        raise ValueError("quaternion_xyzw must contain exactly 4 values")
    norm = float(np.linalg.norm(q))
    if norm < EPS:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    r = np.asarray(rotation, dtype=float)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def average_transforms(transforms: list[Transform]) -> Transform:
    if not transforms:
        raise ValueError("cannot average an empty transform list")
    translations = np.asarray([tf.t for tf in transforms], dtype=float)
    quaternions = [matrix_to_quaternion_xyzw(tf.R) for tf in transforms]
    reference = quaternions[0]
    aligned = []
    for q in quaternions:
        aligned.append(-q if float(np.dot(q, reference)) < 0.0 else q)
    q_avg = average_quaternions(np.asarray(aligned, dtype=float))
    return Transform.from_rt(quaternion_xyzw_to_matrix(q_avg), translations.mean(axis=0))


def average_quaternions(quaternions_xyzw: np.ndarray) -> np.ndarray:
    accumulator = np.zeros((4, 4), dtype=float)
    for q in quaternions_xyzw:
        q = q / np.linalg.norm(q)
        accumulator += np.outer(q, q)
    _, eigenvectors = np.linalg.eigh(accumulator)
    q_avg = eigenvectors[:, -1]
    if q_avg[3] < 0:
        q_avg = -q_avg
    return q_avg / np.linalg.norm(q_avg)


def transform_error(a: Transform, b: Transform) -> dict[str, float]:
    delta = a.inverse() @ b
    translation_m = float(np.linalg.norm(delta.t))
    rotation_rad = rotation_angle(delta.R)
    return {
        "translation_m": translation_m,
        "translation_mm": translation_m * 1000.0,
        "rotation_deg": math.degrees(rotation_rad),
    }


def rotation_angle(rotation: np.ndarray) -> float:
    value = (float(np.trace(rotation)) - 1.0) * 0.5
    value = min(1.0, max(-1.0, value))
    return math.acos(value)


def add_noise(transform: Transform, translation_sigma_m: float, rotation_sigma_deg: float, rng: np.random.Generator) -> Transform:
    dt = rng.normal(0.0, translation_sigma_m, size=3)
    dr = np.deg2rad(rng.normal(0.0, rotation_sigma_deg, size=3))
    noise = Transform.from_rpy_deg(dt, np.rad2deg(dr))
    return transform @ noise
