from __future__ import annotations

from dataclasses import dataclass, field
from math import pi, radians
from typing import Dict, List


@dataclass
class RobotParams:
    l1: float = 100.0
    l2: float = 150.0
    l3: float = 48.0
    servo_offset_x: float = 75.0
    servo_offset_y: float = 0.0
    servo_offset_z: float = 41.231
    servo_offset_z_inverted: float = -293.0
    servo_angle_min: float = field(default_factory=lambda: radians(45.0))
    servo_angle_max: float = field(default_factory=lambda: radians(225.0))
    workspace_z_min: float = 110.0
    workspace_z_max: float = 280.0
    workspace_xy_max: float = 150.0
    ball_joint_angle_limit: float = field(default_factory=lambda: radians(34.1))
    num_servos: int = 3
    servo_distribution: List[float] = field(default_factory=lambda: [0.0, 2 * pi / 3, 4 * pi / 3])
    step_increment_linear: float = 1.8
    step_delay_linear: float = 0.0
    home_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 240.0])
    ik_tolerance: float = 1e-6
    collision_margin: float = 5.0
    servo_ids: List[int] = field(default_factory=lambda: [1, 2, 3])
    servo_physical_angle_min_deg: float = 0.0
    servo_physical_angle_max_deg: float = 240.0
    servo_position_min: int = 0
    servo_position_max: int = 1000
    servo_position_step: int = 1

    @property
    def end_effector_types(self) -> Dict[str, int]:
        return {
            "NONE": 0,
            "CONTINUOUS_ROTATION": 1,
            "CLAW_GRIPPER": 2,
            "VACUUM_GRIPPER": 3,
            "ELECTROMAGNET": 4,
        }


def robot_params() -> RobotParams:
    return RobotParams()
