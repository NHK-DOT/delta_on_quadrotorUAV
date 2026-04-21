from __future__ import annotations

from math import acos, asin, atan2, cos, pi, sin, sqrt
from typing import Tuple

import numpy as np

from .config import RobotParams, robot_params


def local_to_global(x_local: float, y_local: float, z_local: float, servo_angle: float) -> np.ndarray:
    return np.array(
        [
            x_local * cos(servo_angle) + y_local * sin(servo_angle),
            -x_local * sin(servo_angle) + y_local * cos(servo_angle),
            z_local,
        ],
        dtype=float,
    )


def platform_offset(radius: float, servo_angle: float) -> np.ndarray:
    return np.array([radius * cos(servo_angle), -radius * sin(servo_angle), 0.0], dtype=float)


def wrap_to_pi(values: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(values), np.cos(values))


def inverse_kinematics(
    x: float, y: float, z: float, params: RobotParams | None = None
) -> Tuple[np.ndarray, bool]:
    params = params or robot_params()
    results = [
        inverse_kinematics_single(x, y, z, params.servo_distribution[i], params)
        for i in range(3)
    ]
    valid = all(item[1] for item in results)
    if not valid:
        return np.zeros(3, dtype=float), False
    return np.array([item[0] for item in results], dtype=float), True


def inverse_kinematics_single(
    xt: float, yt: float, zt: float, servo_angle: float, params: RobotParams
) -> Tuple[float, bool]:
    zt = zt - params.servo_offset_z
    x_rot = xt * cos(servo_angle) - yt * sin(servo_angle)
    y_rot = xt * sin(servo_angle) + yt * cos(servo_angle)
    arm_end_x = x_rot + params.l3
    under = params.l2**2 - y_rot**2
    if under < 0:
        return 0.0, False
    l2p = sqrt(under)
    l2p_angle = asin(max(-1.0, min(1.0, y_rot / params.l2)))
    if abs(l2p_angle) >= params.ball_joint_angle_limit:
        return 0.0, False
    ext = sqrt(zt**2 + (params.servo_offset_x - arm_end_x) ** 2)
    if ext <= l2p - params.l1 or ext >= params.l1 + l2p:
        return 0.0, False
    cos_phi = (params.l1**2 + ext**2 - l2p**2) / (2 * params.l1 * ext)
    cos_phi = max(-1.0, min(1.0, cos_phi))
    phi = acos(cos_phi)
    omega = atan2(zt, params.servo_offset_x - arm_end_x)
    theta = phi + omega
    if params.servo_angle_min <= theta <= params.servo_angle_max:
        return theta, True
    return 0.0, False


def forward_kinematics(
    theta1: float, theta2: float, theta3: float, params: RobotParams | None = None
) -> Tuple[np.ndarray, bool]:
    params = params or robot_params()
    theta = np.array([theta1, theta2, theta3], dtype=float)
    elbows = np.zeros((3, 3), dtype=float)
    centers = np.zeros((3, 3), dtype=float)
    for i, servo_angle in enumerate(params.servo_distribution):
        elbows[i] = local_to_global(
            params.servo_offset_x - params.l1 * cos(theta[i]),
            0.0,
            params.servo_offset_z + params.l1 * sin(theta[i]),
            servo_angle,
        )
        centers[i] = elbows[i] - platform_offset(params.l3, servo_angle)
    point, ok = intersect_three_spheres(centers, params.l2)
    if not ok:
        return np.zeros(3, dtype=float), False
    theta_check, ik_ok = inverse_kinematics(point[0], point[1], point[2], params)
    if not ik_ok:
        return np.zeros(3, dtype=float), False
    if np.max(np.abs(wrap_to_pi(theta_check - theta))) > 1e-4:
        return np.zeros(3, dtype=float), False
    return point, True


def intersect_three_spheres(centers: np.ndarray, radius: float) -> Tuple[np.ndarray, bool]:
    p1, p2, p3 = centers
    ex = p2 - p1
    d = np.linalg.norm(ex)
    if d < 1e-9:
        return np.zeros(3, dtype=float), False
    ex = ex / d
    i_val = float(np.dot(ex, p3 - p1))
    temp = p3 - p1 - i_val * ex
    temp_norm = np.linalg.norm(temp)
    if temp_norm < 1e-9:
        return np.zeros(3, dtype=float), False
    ey = temp / temp_norm
    ez = np.cross(ex, ey)
    j_val = float(np.dot(ey, p3 - p1))
    if abs(j_val) < 1e-9:
        return np.zeros(3, dtype=float), False
    x_coord = d / 2.0
    y_coord = (i_val**2 + j_val**2 - 2.0 * i_val * x_coord) / (2.0 * j_val)
    z_sq = radius**2 - x_coord**2 - y_coord**2
    if z_sq < -1e-6:
        return np.zeros(3, dtype=float), False
    z_coord = sqrt(max(z_sq, 0.0))
    candidate1 = p1 + x_coord * ex + y_coord * ey + z_coord * ez
    candidate2 = p1 + x_coord * ex + y_coord * ey - z_coord * ez
    point = candidate1 if candidate1[2] >= candidate2[2] else candidate2
    return point, True
