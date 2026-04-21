from __future__ import annotations

from typing import Dict

import numpy as np

from .config import RobotParams, robot_params
from .kinematics import forward_kinematics, inverse_kinematics


class DeltaRobot:
    def __init__(self, params: RobotParams | None = None) -> None:
        self.params = params or robot_params()
        self.current_position = np.array(self.params.home_position, dtype=float)
        self.current_theta, _ = inverse_kinematics(*self.current_position, self.params)
        self.trajectory_history = np.zeros((0, 3), dtype=float)

    def compute_ik(self, x: float, y: float, z: float):
        return inverse_kinematics(x, y, z, self.params)

    def compute_fk(self, theta1: float, theta2: float, theta3: float):
        return forward_kinematics(theta1, theta2, theta3, self.params)

    def get_workspace_bounds(self) -> Dict[str, float]:
        return {
            "x_min": -self.params.workspace_xy_max,
            "x_max": self.params.workspace_xy_max,
            "y_min": -self.params.workspace_xy_max,
            "y_max": self.params.workspace_xy_max,
            "z_min": self.params.workspace_z_min,
            "z_max": self.params.workspace_z_max,
        }
