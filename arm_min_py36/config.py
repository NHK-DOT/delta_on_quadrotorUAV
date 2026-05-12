from math import pi, radians
import os


DEFAULT_PORT = os.environ.get("ARM_SERIAL_PORT", "/dev/ttyUSB0")
DEFAULT_JOYSTICK = os.environ.get("ARM_JOYSTICK", "/dev/input/js0")
BAUDRATE = 9600

UPDATE_RATE_HZ = 50.0
SERVO_MOVE_TIME_MS = 20
MAX_SERVO_SPEED_TICKS_PER_SEC = 400.0
MIN_EFFECTIVE_MOVE_TICKS = 4

SPEED_XY_MM_PER_SEC = 100.0
SPEED_Z_MM_PER_SEC = 80.0
DPAD_THRESHOLD = 0.55
DPAD_SLEW_RATE = 16.0

STARTUP_TOLERANCE_TICKS = 25
FEEDBACK_INTERVAL_SEC = 0.35
FEEDBACK_TIMEOUT_SEC = 0.08
STARTUP_FEEDBACK_TIMEOUT_SEC = 0.25


class RobotParams(object):
    def __init__(self):
        self.l1 = 100.0
        self.l2 = 150.0
        self.l3 = 48.0
        self.servo_offset_x = 75.0
        self.servo_offset_y = 0.0
        self.servo_offset_z = 41.231
        self.servo_angle_min = radians(45.0)
        self.servo_angle_max = radians(225.0)
        self.workspace_z_min = 110.0
        self.workspace_z_max = 280.0
        self.workspace_xy_max = 150.0
        self.ball_joint_angle_limit = radians(34.1)
        self.num_servos = 3
        self.servo_distribution = [0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0]
        self.home_position = [0.0, 0.0, 240.0]
        self.servo_ids = [1, 2, 3]
        self.servo_physical_angle_min_deg = 0.0
        self.servo_physical_angle_max_deg = 240.0


def robot_params():
    return RobotParams()


SERVO_MAPPINGS = {
    1: {
        "name": "servo1",
        "raw_min": 0,
        "raw_max": 834,
        "logical_min": 0.0,
        "logical_max": 1000.0,
        "position_step": 4,
    },
    2: {
        "name": "servo2",
        "raw_min": 0,
        "raw_max": 770,
        "logical_min": 0.0,
        "logical_max": 1000.0,
        "position_step": 4,
    },
    3: {
        "name": "servo3",
        "raw_min": 0,
        "raw_max": 816,
        "logical_min": 0.0,
        "logical_max": 1000.0,
        "position_step": 4,
    },
}

SERVO_RAW_DIRECTIONS = {
    1: -1,
    2: -1,
    3: -1,
}

TOOLING_SERVO_ENABLED = True
TOOLING_SERVO = {
    "servo_id": 4,
    "raw_min": 0,
    "raw_max": 1000,
    "position_step": 5,
    "speed_ticks_per_sec": 120.0,
}
