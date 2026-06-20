#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实机测试版 Xbox 手柄控制器。

这版改成“驱动板反馈闭环”：
1. 后台透明读取 xArm 1.6 驱动板返回的舵机当前位置。
2. 通过反馈位置反算关节角和执行机构位姿。
3. 指令发送基于实际反馈位置限速，不再假设舵机已经走到目标。
4. 检测到持续误差但反馈位置不变化时，触发堵转保护并停机。
"""

from __future__ import annotations

import csv
import json
import sys
import time
import tomllib
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from delta_gcode_servo.config import robot_params
from delta_gcode_servo.kinematics import forward_kinematics, inverse_kinematics
from delta_gcode_servo.robot import DeltaRobot
from delta_gcode_servo.servo import BusServoDriver
from delta_gcode_servo.servo_mapping import load_servo_mappings_for_ids
from vision_tool_state import VisionToolPreviewConfig, build_vision_tool_preview, write_json


@dataclass
class ToolingServoConfig:
    profile_name: str
    servo_id: int
    raw_min: int
    raw_max: int
    position_step: int

    @property
    def center_raw(self) -> int:
        midpoint = (self.raw_min + self.raw_max) / 2.0
        return int(round(midpoint / self.position_step) * self.position_step)

    def clamp(self, raw_value: int | float) -> int:
        quantized = int(round(float(raw_value) / self.position_step) * self.position_step)
        low = min(self.raw_min, self.raw_max)
        high = max(self.raw_min, self.raw_max)
        return max(low, min(high, quantized))


@dataclass
class SensorSnapshot:
    imu_payload: dict[str, Any] | None = None
    imu_age_ms: float | None = None
    apriltag_payload: dict[str, Any] | None = None
    apriltag_age_ms: float | None = None

    @property
    def imu_angles_deg(self) -> dict[str, float] | None:
        payload = self.imu_payload or {}
        angles = payload.get("angles_deg")
        return angles if isinstance(angles, dict) else None

    @property
    def primary_detection(self) -> dict[str, Any] | None:
        payload = self.apriltag_payload or {}
        detections = payload.get("detections")
        if isinstance(detections, list) and detections:
            first = detections[0]
            if isinstance(first, dict):
                return first
        return None

    def yaw_deg(self, mode: str) -> float | None:
        normalized = mode.upper()
        if normalized == "IMU":
            angles = self.imu_angles_deg
            if angles is None:
                return None
            yaw = angles.get("yaw")
            return float(yaw) if isinstance(yaw, (int, float)) else None
        if normalized == "TAG":
            detection = self.primary_detection
            if detection is None:
                return None
            orientation = detection.get("orientation_deg")
            if not isinstance(orientation, dict):
                return None
            yaw = orientation.get("yaw")
            return float(yaw) if isinstance(yaw, (int, float)) else None
        return None


def read_json_snapshot(path: Path) -> tuple[dict[str, Any] | None, float | None]:
    if not path.exists():
        return None, None

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None, None

    if not isinstance(payload, dict):
        return None, None

    timestamp = payload.get("timestamp_unix")
    if isinstance(timestamp, (int, float)):
        age_ms = max(0.0, (time.time() - float(timestamp)) * 1000.0)
    else:
        age_ms = None
    return payload, age_ms


def load_tooling_servo_config(config_path: Path) -> ToolingServoConfig | None:
    if not config_path.exists():
        return None

    try:
        config_data = tomllib.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, tomllib.TOMLDecodeError):
        return None

    servos = config_data.get("servos")
    if not isinstance(servos, dict):
        return None

    servo4 = servos.get("servo4")
    if not isinstance(servo4, dict):
        return None

    try:
        return ToolingServoConfig(
            profile_name="servo4",
            servo_id=int(servo4.get("id", 4)),
            raw_min=int(servo4.get("raw_min", 0)),
            raw_max=int(servo4.get("raw_max", 1000)),
            position_step=max(1, int(servo4.get("position_step", 5))),
        )
    except (TypeError, ValueError):
        return None


class GamepadReader:
    """读取 Xbox 手柄输入。"""

    @staticmethod
    def empty_buttons() -> dict[str, bool]:
        return {
            "a": False,
            "b": False,
            "x": False,
            "y": False,
            "lb": False,
            "rb": False,
            "back": False,
            "start": False,
        }

    def __init__(self, deadzone: float = 0.0):
        self.deadzone = deadzone
        self.pygame = None
        self.joystick = None
        self.axis_baselines: list[float] = []
        self.right_y_axis = 4
        self.last_error: str | None = None
        self.last_detected_count = 0
        self.refresh(announce=True)

    def is_available(self) -> bool:
        return self.joystick is not None

    def refresh(self, *, announce: bool = False) -> bool:
        try:
            if self.pygame is None:
                import pygame

                pygame.init()
                self.pygame = pygame

            assert self.pygame is not None
            self.pygame.joystick.quit()
            time.sleep(0.05)
            self.pygame.joystick.init()
            self.pygame.event.pump()

            self.last_detected_count = int(self.pygame.joystick.get_count())
            if self.last_detected_count <= 0:
                self.joystick = None
                self.last_error = "no_joystick_detected"
                if announce:
                    print("未检测到手柄。")
                return False

            self.joystick = self.pygame.joystick.Joystick(0)
            self.joystick.init()
            self._capture_axis_baselines()
            self.right_y_axis = self._select_right_y_axis()
            self.last_error = None

            if announce:
                print(f"检测到手柄: {self.joystick.get_name()}")
                print(f"轴数量: {self.joystick.get_numaxes()}")
                print(f"十字键数量: {self.joystick.get_numhats()}")
                print(f"按钮数量: {self.joystick.get_numbuttons()}")
                print(f"当前死区: {self.deadzone:.2f}")
                print(f"右摇杆Y轴映射: axis {self.right_y_axis}")
            return True
        except Exception as exc:
            self.joystick = None
            self.last_error = str(exc)
            if announce:
                print(f"手柄初始化失败: {exc}")
                print(f"当前 Python: {sys.executable}")
                print("如果这里是没有安装 pygame 的解释器，请改用装了 pygame 的那个 python 来启动。")
            return False

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) <= self.deadzone:
            return 0.0

        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled

    def _capture_axis_baselines(self) -> None:
        if self.joystick is None or self.pygame is None:
            self.axis_baselines = []
            return

        num_axes = self.joystick.get_numaxes()
        samples = np.zeros(num_axes, dtype=float)
        sample_count = 20
        for _ in range(sample_count):
            self.pygame.event.pump()
            for axis_index in range(num_axes):
                samples[axis_index] += float(self.joystick.get_axis(axis_index))
            time.sleep(0.005)

        self.axis_baselines = (samples / sample_count).tolist()

    def _select_right_y_axis(self) -> int:
        if self.joystick is None:
            return 4

        num_axes = self.joystick.get_numaxes()
        candidates = [axis for axis in [4, 3] if axis < num_axes]
        if not candidates:
            return max(0, num_axes - 1)

        return min(
            candidates,
            key=lambda axis: abs(self.axis_baselines[axis]) if axis < len(self.axis_baselines) else 999.0,
        )

    def _read_axis(self, index: int) -> float:
        if self.joystick is None or self.joystick.get_numaxes() <= index:
            return 0.0
        baseline = self.axis_baselines[index] if index < len(self.axis_baselines) else 0.0
        value = float(self.joystick.get_axis(index)) - baseline
        value = max(-1.0, min(1.0, value))
        return self._apply_deadzone(value)

    def _read_button(self, index: int) -> bool:
        if self.joystick is None or self.joystick.get_numbuttons() <= index:
            return False
        return bool(self.joystick.get_button(index))

    def _read_hat(self, index: int = 0) -> tuple[float, float]:
        if self.joystick is None or self.joystick.get_numhats() <= index:
            return 0.0, 0.0

        hat_x, hat_y = self.joystick.get_hat(index)
        # Only accept the four cardinal D-pad directions. If two directions are
        # pressed together, do not emit a diagonal XY command.
        if hat_x != 0 and hat_y != 0:
            return 0.0, 0.0
        return float(hat_x), float(hat_y)

    def read(self) -> Tuple[float, float, float, dict[str, bool]]:
        """返回 (dpad_x, dpad_y, right_y, buttons)。"""
        if not self.joystick or self.pygame is None:
            return 0.0, 0.0, 0.0, self.empty_buttons()

        try:
            self.pygame.event.pump()

            dpad_x, dpad_y = self._read_hat()
            right_y = self._read_axis(self.right_y_axis)

            buttons = {
                "a": self._read_button(0),
                "b": self._read_button(1),
                "x": self._read_button(2),
                "y": self._read_button(3),
                "lb": self._read_button(4),
                "rb": self._read_button(5),
                "back": self._read_button(6),
                "start": self._read_button(7),
            }
            return dpad_x, dpad_y, right_y, buttons
        except Exception as exc:
            print(f"读取手柄失败: {exc}")
            return 0.0, 0.0, 0.0, self.empty_buttons()


class RealTimeArmController:
    """实机测试控制器。"""

    def __init__(self, port: str = "COM9", baudrate: int = 9600):
        self.project_root = Path(__file__).resolve().parents[2]
        self.debug_log_path = Path(__file__).with_name("gamepad_diagnostic.log")
        self.debug_log_file = self.debug_log_path.open("w", encoding="utf-8")
        self.debug_log_file.write(f"# gamepad diagnostic {datetime.now().isoformat(timespec='seconds')}\n")
        self.debug_log_file.flush()
        self.debug_log_buffer: list[str] = []
        self.debug_log_flush_interval = 0.12
        self.debug_log_flush_limit = 96
        self.last_debug_log_flush_time = time.perf_counter()
        self.driver = BusServoDriver(
            port=port,
            baudrate=baudrate,
            connect_delay=0.2,
            packet_trace_hook=self.trace_packet,
        )
        self.robot = DeltaRobot()
        self.params = robot_params()
        self.port = port
        self.servo_ids = [1, 2, 3]
        self.physical_angle_min_deg = float(self.params.servo_physical_angle_min_deg)
        self.physical_angle_max_deg = float(self.params.servo_physical_angle_max_deg)
        self.servo_mappings = load_servo_mappings_for_ids(self.servo_ids)
        self.servo_raw_directions = {1: -1, 2: -1, 3: -1}
        self.servo_logical_directions = {
            servo_id: self.servo_raw_directions[servo_id]
            * (1 if self.servo_mappings[servo_id].logical_span >= 0.0 else -1)
            for servo_id in self.servo_ids
        }
        self.servo_units_per_degree = {
            servo_id: self.servo_mappings[servo_id].logical_units_per_degree(
                physical_min_deg=self.physical_angle_min_deg,
                physical_max_deg=self.physical_angle_max_deg,
            )
            for servo_id in self.servo_ids
        }
        self.reference_servo_positions = {
            servo_id: self.servo_mappings[servo_id].reference_raw
            for servo_id in self.servo_ids
        }
        self.reference_servo_coords = {
            servo_id: self.servo_mappings[servo_id].raw_to_logical(self.reference_servo_positions[servo_id])
            for servo_id in self.servo_ids
        }
        self.servo_limits = {
            servo_id: (
                self.servo_mappings[servo_id].raw_low,
                self.servo_mappings[servo_id].raw_high,
            )
            for servo_id in self.servo_ids
        }
        self.startup_tolerance_ticks = 25
        self.reference_position = np.array(self.params.home_position, dtype=float)

        # 空载调试模式: 把参考位放在模型中间层，避免一上来就贴着 Z 上边界。
        self.reference_position = np.array(self.params.home_position, dtype=float)
        self.reference_angles_rad: np.ndarray | None = None

        self.current_servo_positions = self.reference_servo_positions.copy()
        self.current_angles_rad: np.ndarray | None = None
        self.current_position = self.reference_position.copy()
        self.current_pose_valid = True

        self.command_servo_positions = self.reference_servo_positions.copy()
        self.target_servo_positions = self.reference_servo_positions.copy()
        self.target_angles_rad: np.ndarray | None = None
        self.target_position = self.reference_position.copy()

        self.max_servo_speed_ticks_per_sec = 400.0
        self.min_effective_move_ticks = 4
        self.min_command_time_ms = 20
        self.position_tolerance_ticks = 0
        self.speed_xy = 100.0
        self.speed_z = 80.0
        self.update_rate = 50
        self.update_interval = 1.0 / self.update_rate
        self.axis_filter_alpha = 1.0
        self.dpad_threshold = 0.55
        self.filtered_axes = np.zeros(3, dtype=float)
        self.last_dpad_axes = (0.0, 0.0, 0.0)
        self.motion_dpad_axes = np.zeros(3, dtype=float)
        self.last_motion_dpad_axes = (0.0, 0.0, 0.0)
        self.dpad_slew_rate = 16.0
        self.xy_rotation_rad = 0.0
        self.enforce_workspace_bounds = True
        self.enable_stall_guard = True
        self.max_target_lead_mm = 18.0
        self.target_reanchor_error_mm = 45.0
        self.playback_step_mm = 2.0
        self.playback_speed_mm_per_sec = 70.0
        self.playback_endpoint_tolerance_mm = 18.0
        self.playback_lead_timeout_sec = 2.0
        self.playback_settle_error_ticks = 10
        self.playback_lift_clearance_mm = 30.0
        self.playback_mode = "LINE"
        self.playback_active = False
        self.startup_home_servo_speed_ticks_per_sec = 120.0
        self.startup_home_timeout_sec = 20.0

        self.feedback_interval = 0.35
        self.feedback_timeout = 0.05
        self.startup_feedback_timeout = 0.20
        self.feedback_failure_limit = 10
        self.stall_error_ticks = 20
        self.stall_timeout = 1.2

        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        self.last_sent_positions = self.current_servo_positions.copy()
        self.last_send_time: float | None = None
        self.last_feedback_poll_time = 0.0
        self.last_tooling_feedback_poll_time = 0.0
        self.tooling_feedback_interval = 0.80
        self.last_feedback_change_time: float | None = None
        self.last_motion_command_time: float | None = None
        self.last_voltage_poll_time = 0.0
        self.last_sensor_poll_time = 0.0
        self.feedback_failure_count = 0
        self.battery_voltage_mv: int | None = None
        self.last_axes = (0.0, 0.0, 0.0)
        self.last_axes_raw = (0.0, 0.0, 0.0)
        self.last_buttons = GamepadReader.empty_buttons()
        self.safety_fault_message: str | None = None
        self.safe_scan_mode = "FREE"
        self.record_file_path = Path(__file__).with_name("workspace_points.csv")
        self.runtime_status_path = Path(__file__).with_name("runtime_status.log")
        self.record_count = 0
        self.sampled_points: list[np.ndarray] = []
        self.status_update_interval = 0.2
        self.sensor_poll_interval = 0.10
        self.sensor_frame_mode = "OFF"
        self.sensor_heading_zero_deg: float | None = None
        self.sensor_snapshot = SensorSnapshot()
        self.imu_snapshot_path = self.project_root / "IMU" / "wt61c_latest.json"
        self.apriltag_snapshot_path = self.project_root / "AprilTag_Vision" / "myAprilTag" / "output" / "apriltag_latest.json"
        self.handeye_calibration_path = self.project_root / "Dual_Camera_HandEye" / "output" / "calibration_result.json"
        self.vision_tool_preview_path = Path(__file__).with_name("vision_tool_preview_latest.json")
        self.vision_tool_preview_interval = 0.25
        self.last_vision_tool_preview_time = 0.0
        self.vision_tool_preview: dict[str, Any] | None = None
        self.vision_tool_preview_error: str | None = None
        self.vision_hand_tag_id: int | None = None
        self.tooling_config = load_tooling_servo_config(
            self.project_root / "lx225_tool_demo" / "config" / "lx225_tool.demo.toml"
        )
        self.tooling_speed_ticks_per_sec = 120.0
        self.tooling_current_position: int | None = None
        self.tooling_target_position: int | None = None
        self.last_sent_tooling_position: int | None = None
        self.is_ready = False

        self.gamepad = GamepadReader(deadzone=0.05)

    def debug_log(self, message: str) -> None:
        try:
            now = time.perf_counter()
            self.debug_log_buffer.append(
                f"{now:.6f} {datetime.now().isoformat(timespec='milliseconds')} {message}\n"
            )
            if (
                len(self.debug_log_buffer) >= self.debug_log_flush_limit
                or now - self.last_debug_log_flush_time >= self.debug_log_flush_interval
            ):
                self.flush_debug_log(now=now)
        except Exception:
            pass

    def flush_debug_log(self, *, now: float | None = None) -> None:
        try:
            if self.debug_log_buffer:
                self.debug_log_file.writelines(self.debug_log_buffer)
                self.debug_log_buffer.clear()
            self.debug_log_file.flush()
            self.last_debug_log_flush_time = time.perf_counter() if now is None else now
        except Exception:
            pass

    def trace_packet(self, direction: str, packet: bytes, note: str) -> None:
        hex_text = packet.hex(" ").upper() if packet else "<none>"
        self.debug_log(f"SERIAL {direction} {note} {hex_text}")

    def connect(self) -> bool:
        try:
            print(f"\n正在连接串口 {self.port}...")
            self.driver.connect()
            print("串口连接成功。")

            if not self.gamepad.is_available():
                print("手柄第一次扫描未就绪，正在重扫...")
                if not self.gamepad.refresh(announce=True):
                    print(
                        "手柄仍未初始化，无法进入控制。"
                        f" count={self.gamepad.last_detected_count}, error={self.gamepad.last_error}"
                    )
                    return False

            print("手柄连接成功。")
            return True
        except Exception as exc:
            print(f"连接失败: {exc}")
            return False

    def quantize_servo_position(self, servo_id: int, value: int | float) -> int:
        return self.servo_mappings[servo_id].quantize_raw(value)

    def servo_position_to_coord(self, servo_id: int, position: int | float) -> float:
        return self.servo_mappings[servo_id].raw_to_logical(position)

    def edge_pressed(self, name: str, buttons: dict[str, bool]) -> bool:
        return buttons.get(name, False) and not self.last_buttons.get(name, False)

    @staticmethod
    def format_xyz(point: np.ndarray) -> str:
        return f"[{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}]"

    def clamp_position_to_workspace(self, position: np.ndarray) -> np.ndarray:
        candidate = np.asarray(position, dtype=float).copy()
        if not self.enforce_workspace_bounds:
            return candidate

        bounds = self.robot.get_workspace_bounds()
        before = candidate.copy()
        candidate[0] = np.clip(candidate[0], bounds["x_min"], bounds["x_max"])
        candidate[1] = np.clip(candidate[1], bounds["y_min"], bounds["y_max"])
        candidate[2] = np.clip(candidate[2], bounds["z_min"], bounds["z_max"])
        xy_radius = float(np.linalg.norm(candidate[:2]))
        xy_limit = float(self.params.workspace_xy_max)
        if xy_radius > xy_limit and xy_radius > 1e-9:
            candidate[:2] *= xy_limit / xy_radius
        if not np.allclose(before, candidate, atol=1e-6):
            self.debug_log(
                "WORKSPACE_CLAMP "
                f"from=({before[0]:.3f},{before[1]:.3f},{before[2]:.3f}) "
                f"to=({candidate[0]:.3f},{candidate[1]:.3f},{candidate[2]:.3f})"
            )
        return candidate

    def target_feedback_error_mm(self) -> float:
        return float(np.linalg.norm(self.target_position - self.current_position))

    def target_servo_error_ticks(self) -> int:
        return max(
            abs(self.target_servo_positions[servo_id] - self.current_servo_positions[servo_id])
            for servo_id in self.servo_ids
        )

    def reanchor_target_to_feedback(self, reason: str) -> None:
        if self.current_angles_rad is None:
            return

        self.target_position = self.current_position.copy()
        self.target_angles_rad = self.current_angles_rad.copy()
        self.target_servo_positions = self.current_servo_positions.copy()
        self.command_servo_positions = self.current_servo_positions.copy()
        self.last_sent_positions = self.current_servo_positions.copy()
        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        self.last_send_time = time.perf_counter()
        self.debug_log(
            f"TARGET_REANCHOR reason={reason} "
            f"xyz=({self.target_position[0]:.3f},{self.target_position[1]:.3f},{self.target_position[2]:.3f}) "
            f"raw={self.current_servo_positions}"
        )

    def user_motion_origin(self) -> np.ndarray:
        error_mm = self.target_feedback_error_mm()
        if error_mm > self.target_reanchor_error_mm:
            self.reanchor_target_to_feedback(f"error_mm={error_mm:.2f}")
            return self.current_position.copy()

        if error_mm <= self.max_target_lead_mm:
            return self.target_position.copy()

        direction = self.target_position - self.current_position
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm < 1e-9:
            return self.current_position.copy()

        limited = self.current_position + direction * (self.max_target_lead_mm / direction_norm)
        self.debug_log(
            f"TARGET_LEAD_LIMIT error_mm={error_mm:.2f} "
            f"origin=({limited[0]:.3f},{limited[1]:.3f},{limited[2]:.3f})"
        )
        return limited

    def resolve_motion_target(
        self,
        position: np.ndarray,
        source: str,
    ) -> tuple[bool, np.ndarray, np.ndarray, dict[int, int]]:
        candidate = self.clamp_position_to_workspace(position)
        angles_rad, success = inverse_kinematics(
            candidate[0],
            candidate[1],
            candidate[2],
            self.params,
        )
        if not success:
            self.debug_log(
                f"IK_FAIL source={source} "
                f"xyz=({candidate[0]:.3f},{candidate[1]:.3f},{candidate[2]:.3f})"
            )
            return False, candidate, angles_rad, {}

        try:
            positions = self.angles_to_servo_positions(angles_rad, strict=True)
        except ValueError as exc:
            self.debug_log(
                f"RAW_LIMIT_FAIL source={source} "
                f"xyz=({candidate[0]:.3f},{candidate[1]:.3f},{candidate[2]:.3f}) error={exc}"
            )
            return False, candidate, angles_rad, {}

        return True, candidate, angles_rad, positions

    def set_motion_target(self, position: np.ndarray, source: str) -> bool:
        ok, candidate, angles_rad, positions = self.resolve_motion_target(position, source)
        if not ok:
            return False

        self.target_position = candidate
        self.target_angles_rad = angles_rad
        self.target_servo_positions = positions
        self.debug_log(
            f"TARGET_OK source={source} "
            f"xyz=({candidate[0]:.3f},{candidate[1]:.3f},{candidate[2]:.3f}) "
            f"raw={positions}"
        )
        return True

    def interpolate_line_points(self, start: np.ndarray, end: np.ndarray, step_mm: float) -> np.ndarray:
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        distance = float(np.linalg.norm(end - start))
        if distance < 1e-6:
            return np.zeros((0, 3), dtype=float)

        num_steps = max(1, int(np.ceil(distance / max(0.1, step_mm))))
        t_values = np.linspace(1.0 / num_steps, 1.0, num_steps)[:, None]
        return start + (end - start) * t_values

    def interpolate_waypoint_path(self, waypoints: list[np.ndarray], step_mm: float) -> np.ndarray:
        segments: list[np.ndarray] = []
        for start, end in zip(waypoints, waypoints[1:]):
            segment = self.interpolate_line_points(start, end, step_mm)
            if len(segment):
                segments.append(segment)
        return np.vstack(segments) if segments else np.zeros((0, 3), dtype=float)

    def cycle_playback_mode(self) -> None:
        modes = ["LINE", "PICK_PLACE"]
        current_index = modes.index(self.playback_mode)
        self.playback_mode = modes[(current_index + 1) % len(modes)]
        print(f"AB playback mode -> {self.playback_mode}")
        self.write_runtime_status()

    def validate_path_points(
        self,
        points: np.ndarray,
        source: str,
    ) -> list[tuple[np.ndarray, np.ndarray, dict[int, int]]]:
        validated: list[tuple[np.ndarray, np.ndarray, dict[int, int]]] = []
        for index, point in enumerate(points, start=1):
            ok, candidate, angles_rad, positions = self.resolve_motion_target(point, f"{source}:{index}")
            if not ok:
                print(f"Path validation failed at point {index}: {self.format_xyz(candidate)}")
                return []
            validated.append((candidate, angles_rad, positions))
        return validated

    def wait_until_target_lead_ok(self) -> bool:
        deadline = time.perf_counter() + self.playback_lead_timeout_sec
        while self.target_feedback_error_mm() > self.max_target_lead_mm:
            if time.perf_counter() >= deadline:
                self.safety_fault_message = (
                    f"playback target lead too large: {self.target_feedback_error_mm():.1f} mm"
                )
                return False
            if not self.send_servo_positions():
                return False
            if not self.sync_servo_feedback():
                return False
            if not self.check_safety_guard():
                return False
            time.sleep(self.update_interval)
        return True

    def wait_until_target_settled(self, timeout_sec: float = 2.0) -> bool:
        deadline = time.perf_counter() + timeout_sec
        while self.target_servo_error_ticks() > self.playback_settle_error_ticks:
            if time.perf_counter() >= deadline:
                self.debug_log(
                    f"PLAYBACK_SETTLE_TIMEOUT error_ticks={self.target_servo_error_ticks()}"
                )
                return False
            if not self.send_servo_positions():
                return False
            if not self.sync_servo_feedback():
                return False
            if not self.check_safety_guard():
                return False
            time.sleep(self.update_interval)
        return True

    def execute_validated_path(
        self,
        validated: list[tuple[np.ndarray, np.ndarray, dict[int, int]]],
        *,
        label: str,
        speed_mm_per_sec: float,
        settle_timeout_sec: float = 2.0,
    ) -> bool:
        if not validated:
            return False

        self.playback_active = True
        self.motion_dpad_axes[:] = 0.0
        self.last_motion_dpad_axes = (0.0, 0.0, 0.0)
        sleep_time = max(self.update_interval, self.playback_step_mm / max(1.0, speed_mm_per_sec))
        try:
            for index, (point, angles_rad, positions) in enumerate(validated, start=1):
                if not self.wait_until_target_lead_ok():
                    print(self.safety_fault_message or f"{label} stopped by target lead guard.")
                    return False

                self.target_position = point
                self.target_angles_rad = angles_rad
                self.target_servo_positions = positions
                if not self.send_servo_positions():
                    print(self.safety_fault_message or f"{label} servo command send failed.")
                    return False

                if index % 5 == 0:
                    if not self.sync_servo_feedback():
                        print(self.safety_fault_message or f"{label} feedback sync failed.")
                        return False
                    if not self.check_safety_guard():
                        print(self.safety_fault_message)
                        return False
                    self.write_runtime_status()

                time.sleep(sleep_time)

            if not self.wait_until_target_settled(settle_timeout_sec):
                if self.safety_fault_message:
                    print(self.safety_fault_message)
                    return False
                print(f"{label} ended, but feedback did not settle within the timeout.")
            self.sync_servo_feedback(force=True)
            self.write_runtime_status()
            return True
        finally:
            self.playback_active = False

    def play_last_sample_segment(self) -> bool:
        if len(self.sampled_points) < 2:
            print("Need at least two sampled points. Press B at two positions first.")
            return False

        if self.current_angles_rad is None:
            print("Controller is not initialized.")
            return False

        start_a = self.sampled_points[-2].copy()
        start_b = self.sampled_points[-1].copy()
        self.reanchor_target_to_feedback("playback_start")
        current = self.current_position.copy()

        if self.playback_mode == "LINE":
            dist_a = float(np.linalg.norm(current - start_a))
            dist_b = float(np.linalg.norm(current - start_b))

            if dist_a <= dist_b:
                start_point = start_a
                end_point = start_b
                direction_text = "sample[-2] -> sample[-1]"
                start_error = dist_a
            else:
                start_point = start_b
                end_point = start_a
                direction_text = "sample[-1] -> sample[-2]"
                start_error = dist_b

            if start_error > self.playback_endpoint_tolerance_mm:
                print(
                    "Current position is not close enough to either sampled endpoint. "
                    f"nearest error={start_error:.1f} mm, limit={self.playback_endpoint_tolerance_mm:.1f} mm."
                )
                return False

            waypoints = [current, end_point]
            points = self.interpolate_waypoint_path(waypoints, self.playback_step_mm)
        else:
            home = self.reference_position.copy()
            a_point = start_a
            b_point = start_b
            travel_z = min(
                float(self.params.workspace_z_max),
                max(float(home[2]), float(a_point[2]), float(b_point[2])) + self.playback_lift_clearance_mm,
            )
            a_lift = np.array([a_point[0], a_point[1], travel_z], dtype=float)
            b_lift = np.array([b_point[0], b_point[1], travel_z], dtype=float)
            waypoints = [current, home, a_point, a_lift, b_lift, b_point, b_lift, home]
            start_point = a_point
            end_point = b_point
            direction_text = "home -> A -> lift -> B -> lift -> home"
            points = self.interpolate_waypoint_path(waypoints, self.playback_step_mm)

        if len(points) == 0:
            print("Sampled path is too short to play.")
            return False

        validated = self.validate_path_points(points, "sample-playback")
        if not validated:
            return False

        distance = float(np.sum(np.linalg.norm(np.diff(np.vstack([current, points]), axis=0), axis=1)))
        est_seconds = distance / max(1.0, self.playback_speed_mm_per_sec)
        print("")
        print("Sample segment playback request")
        print(f"  mode     : {self.playback_mode}")
        print(f"  direction: {direction_text}")
        print(f"  endpoint: {self.format_xyz(start_point)}")
        print(f"  current : {self.format_xyz(current)}")
        print(f"  target  : {self.format_xyz(end_point)}")
        print(f"  distance: {distance:.1f} mm")
        print(f"  points  : {len(validated)} at step {self.playback_step_mm:.1f} mm")
        print(f"  estimate: {est_seconds:.2f} s, servo limit {self.max_servo_speed_ticks_per_sec:.0f} raw/s")
        response = input("Type PLAY to execute this motion: ").strip()
        if response != "PLAY":
            print("Playback cancelled.")
            return False

        if self.execute_validated_path(
            validated,
            label=f"{self.playback_mode} playback",
            speed_mm_per_sec=self.playback_speed_mm_per_sec,
        ):
            print("Playback finished.")
            return True
        return False

    def filter_axes(self, left_x: float, left_y: float, right_y: float) -> tuple[float, float, float]:
        raw_axes = np.array([left_x, left_y, right_y], dtype=float)
        self.filtered_axes = (
            self.axis_filter_alpha * raw_axes
            + (1.0 - self.axis_filter_alpha) * self.filtered_axes
        )
        self.filtered_axes[np.abs(raw_axes) < 0.01] = 0.0
        return tuple(float(value) for value in self.filtered_axes)

    def _axis_to_dpad(self, x_value: float, y_value: float) -> tuple[float, float]:
        abs_x = abs(x_value)
        abs_y = abs(y_value)
        if max(abs_x, abs_y) < self.dpad_threshold:
            return 0.0, 0.0
        if abs_y >= abs_x:
            return 0.0, 1.0 if y_value > 0.0 else -1.0
        return 1.0 if x_value > 0.0 else -1.0, 0.0

    def gamepad_to_dpad_axes(
        self,
        left_x: float,
        left_y: float,
        right_y: float,
    ) -> tuple[float, float, float]:
        dpad_left_x, dpad_left_y = self._axis_to_dpad(left_x, left_y)
        if abs(right_y) >= self.dpad_threshold:
            dpad_right_y = 1.0 if right_y > 0.0 else -1.0
        else:
            dpad_right_y = 0.0
        return dpad_left_x, dpad_left_y, dpad_right_y

    def smooth_dpad_axes(self, target_axes: tuple[float, float, float]) -> tuple[float, float, float]:
        target = np.array(target_axes, dtype=float)
        max_step = self.dpad_slew_rate * self.update_interval
        delta = np.clip(target - self.motion_dpad_axes, -max_step, max_step)
        self.motion_dpad_axes += delta
        self.motion_dpad_axes[np.abs(self.motion_dpad_axes) < 0.01] = 0.0
        return tuple(float(value) for value in self.motion_dpad_axes)

    def sync_sensor_feedback(self, *, force: bool = False) -> None:
        now = time.perf_counter()
        if not force and now - self.last_sensor_poll_time < self.sensor_poll_interval:
            return

        self.last_sensor_poll_time = now
        imu_payload, imu_age_ms = read_json_snapshot(self.imu_snapshot_path)
        apriltag_payload, apriltag_age_ms = read_json_snapshot(self.apriltag_snapshot_path)
        self.sensor_snapshot = SensorSnapshot(
            imu_payload=imu_payload,
            imu_age_ms=imu_age_ms,
            apriltag_payload=apriltag_payload,
            apriltag_age_ms=apriltag_age_ms,
        )
        self.update_control_frame_from_sensors()

    def update_vision_tool_preview(self, *, force: bool = False) -> dict[str, Any] | None:
        now = time.perf_counter()
        if not force and now - self.last_vision_tool_preview_time < self.vision_tool_preview_interval:
            return self.vision_tool_preview

        self.last_vision_tool_preview_time = now
        config = VisionToolPreviewConfig(
            calibration_path=self.handeye_calibration_path,
            apriltag_snapshot_path=self.apriltag_snapshot_path,
            imu_snapshot_path=self.imu_snapshot_path,
            output_path=self.vision_tool_preview_path,
            hand_tag_id=self.vision_hand_tag_id,
        )
        try:
            payload = build_vision_tool_preview(config)
            write_json(self.vision_tool_preview_path, payload)
            self.vision_tool_preview = payload
            self.vision_tool_preview_error = None
            self.debug_log(
                "VISION_TOOL_PREVIEW "
                f"xyz_mm={payload.get('tool_position_mm')} "
                f"ik={payload.get('delta_ik', {}).get('reachable')}"
            )
            return payload
        except Exception as exc:
            self.vision_tool_preview_error = str(exc)
            self.debug_log(f"VISION_TOOL_PREVIEW_ERROR error={exc}")
            return None

    def preview_vision_motion_target(self) -> bool:
        payload = self.update_vision_tool_preview(force=True)
        if payload is None:
            return False

        delta_ik = payload.get("delta_ik", {})
        if not delta_ik.get("reachable", False):
            return False

        target_position = np.asarray(payload["tool_position_mm"], dtype=float)
        ok, candidate, _angles_rad, positions = self.resolve_motion_target(
            target_position,
            "vision-base-camera-preview",
        )
        self.debug_log(
            "VISION_TARGET_PREVIEW "
            f"ok={ok} xyz=({candidate[0]:.3f},{candidate[1]:.3f},{candidate[2]:.3f}) raw={positions}"
        )

        # Motion is intentionally disabled for the first base-camera chain test.
        # Enable only after confirming the visual XYZ axes and scale on hardware:
        #
        # if self.set_motion_target(candidate, "vision-base-camera"):
        #     return self.send_servo_positions()
        #
        return ok

    def cycle_sensor_frame_mode(self) -> None:
        modes = ["OFF", "IMU", "TAG"]
        current_index = modes.index(self.sensor_frame_mode)
        self.sensor_frame_mode = modes[(current_index + 1) % len(modes)]
        self.sensor_heading_zero_deg = self.sensor_snapshot.yaw_deg(self.sensor_frame_mode)
        self.update_control_frame_from_sensors()
        print(f"sensor frame mode -> {self.sensor_frame_mode}")
        self.write_runtime_status()

    def update_control_frame_from_sensors(self) -> None:
        current_yaw_deg = self.sensor_snapshot.yaw_deg(self.sensor_frame_mode)
        if self.sensor_frame_mode == "OFF" or current_yaw_deg is None:
            self.xy_rotation_rad = 0.0
            return

        if self.sensor_heading_zero_deg is None:
            self.sensor_heading_zero_deg = current_yaw_deg

        self.xy_rotation_rad = np.radians(current_yaw_deg - self.sensor_heading_zero_deg)

    def sync_tooling_feedback(self, *, force: bool = False) -> None:
        if self.tooling_config is None:
            return

        now = time.perf_counter()
        if not force and now - self.last_tooling_feedback_poll_time < self.tooling_feedback_interval:
            return
        self.last_tooling_feedback_poll_time = now

        try:
            feedback = self.driver.read_servo_positions(
                [self.tooling_config.servo_id],
                timeout=self.feedback_timeout,
            )
        except Exception:
            return

        if self.tooling_config.servo_id not in feedback:
            return

        position = self.tooling_config.clamp(int(feedback[self.tooling_config.servo_id]))
        self.tooling_current_position = position
        if self.tooling_target_position is None:
            self.tooling_target_position = position
        if self.last_sent_tooling_position is None:
            self.last_sent_tooling_position = position

    def update_tooling_from_buttons(self, buttons: dict[str, bool]) -> bool:
        if self.tooling_config is None:
            return False

        if self.tooling_target_position is None:
            self.tooling_target_position = self.tooling_config.center_raw

        direction = 0
        if buttons.get("rb", False):
            direction += 1
        if buttons.get("lb", False):
            direction -= 1
        if direction == 0:
            return False

        delta_ticks = direction * self.tooling_speed_ticks_per_sec * self.update_interval
        next_target = self.tooling_config.clamp(self.tooling_target_position + delta_ticks)
        if next_target == self.tooling_target_position:
            return False

        self.tooling_target_position = next_target
        return True

    def cycle_safe_scan_mode(self) -> None:
        modes = ["FREE", "X", "Y", "Z"]
        current_index = modes.index(self.safe_scan_mode)
        self.safe_scan_mode = modes[(current_index + 1) % len(modes)]
        print(f"safe scan 模式切换为: {self.safe_scan_mode}")
        self.write_runtime_status()

    def record_current_point(self) -> None:
        self.record_count += 1
        file_exists = self.record_file_path.exists()
        row = {
            "index": self.record_count,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "safe_scan_mode": self.safe_scan_mode,
            "feedback_x_mm": round(float(self.current_position[0]), 3),
            "feedback_y_mm": round(float(self.current_position[1]), 3),
            "feedback_z_mm": round(float(self.current_position[2]), 3),
            "target_x_mm": round(float(self.target_position[0]), 3),
            "target_y_mm": round(float(self.target_position[1]), 3),
            "target_z_mm": round(float(self.target_position[2]), 3),
            "servo1_feedback": self.current_servo_positions[1],
            "servo2_feedback": self.current_servo_positions[2],
            "servo3_feedback": self.current_servo_positions[3],
            "servo1_target": self.target_servo_positions[1],
            "servo2_target": self.target_servo_positions[2],
            "servo3_target": self.target_servo_positions[3],
            "battery_mv": self.battery_voltage_mv if self.battery_voltage_mv is not None else "",
        }

        try:
            with self.record_file_path.open("a", encoding="utf-8", newline="") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=list(row.keys()))
                if not file_exists:
                    writer.writeheader()
                writer.writerow(row)

            self.sampled_points.append(self.current_position.copy())
            if len(self.sampled_points) > 20:
                self.sampled_points = self.sampled_points[-20:]

            print(
                "已记录工作空间点: "
                f"#{self.record_count} -> "
                f"X={self.current_position[0]:.2f}, "
                f"Y={self.current_position[1]:.2f}, "
                f"Z={self.current_position[2]:.2f}"
            )
            print(
                f"Sample buffer: {len(self.sampled_points)} point(s). "
                f"Mode={self.playback_mode}. BACK toggles mode, START replays last segment."
            )
            self.write_runtime_status()
        except OSError as exc:
            print(f"记录工作空间点失败: {exc}")

    def build_status_snapshot(self) -> str:
        if self.current_angles_rad is None:
            return "状态尚未初始化。"

        lines = [
            "Delta 机械臂实时状态",
            f"时间: {datetime.now().isoformat(timespec='seconds')}",
            f"safe scan: {self.safe_scan_mode}",
            f"测试模式: 空载调试 | 工作空间裁剪={'开启' if self.enforce_workspace_bounds else '关闭'} | 堵转保护={'开启' if self.enable_stall_guard else '关闭'}",
            f"速度参数: servo={self.max_servo_speed_ticks_per_sec:.0f} tick/s, xy={self.speed_xy:.0f} mm/s, z={self.speed_z:.0f} mm/s",
            (
                "motion guard: "
                f"target_lead={self.target_feedback_error_mm():.1f}/{self.max_target_lead_mm:.1f} mm, "
                f"sample_points={len(self.sampled_points)}, "
                f"playback_mode={self.playback_mode}, "
                f"path_step={self.playback_step_mm:.1f} mm, "
                f"path_speed={self.playback_speed_mm_per_sec:.1f} mm/s"
            ),
            f"手柄轴映射: right_y_axis={self.gamepad.right_y_axis}",
            f"diagnostic log: {self.debug_log_path.name}",
            (
                "反馈末端: "
                f"X={self.current_position[0]:.2f} mm, "
                f"Y={self.current_position[1]:.2f} mm, "
                f"Z={self.current_position[2]:.2f} mm"
            ),
            (
                "目标末端: "
                f"X={self.target_position[0]:.2f} mm, "
                f"Y={self.target_position[1]:.2f} mm, "
                f"Z={self.target_position[2]:.2f} mm"
            ),
            (
                "反馈舵机: "
                + ", ".join(f"{servo_id}={self.current_servo_positions[servo_id]}" for servo_id in self.servo_ids)
            ),
            (
                "目标舵机: "
                + ", ".join(f"{servo_id}={self.target_servo_positions[servo_id]}" for servo_id in self.servo_ids)
            ),
            (
                "反馈关节角: "
                + ", ".join(f"{value:.2f}°" for value in np.degrees(self.current_angles_rad))
            ),
            f"输入原始: DX={self.last_axes[0]:+.3f}, DY={self.last_axes[1]:+.3f}, RY={self.last_axes[2]:+.3f}",
            f"十字输入: DX={self.last_dpad_axes[0]:+.0f}, DY={self.last_dpad_axes[1]:+.0f}, RY={self.last_dpad_axes[2]:+.0f}",
            f"平滑输入: DX={self.last_motion_dpad_axes[0]:+.2f}, DY={self.last_motion_dpad_axes[1]:+.2f}, RY={self.last_motion_dpad_axes[2]:+.2f}",
            f"记录点数量: {self.record_count}",
        ]

        lines.append(
            f"sensor frame: {self.sensor_frame_mode} | xy_rotation_deg={np.degrees(self.xy_rotation_rad):+.2f}"
        )
        lines.append(
            f"axes raw: LX={self.last_axes_raw[0]:+.3f}, LY={self.last_axes_raw[1]:+.3f}, RY={self.last_axes_raw[2]:+.3f}"
        )
        if self.tooling_config is not None:
            lines.append(
                "tooling servo: "
                f"id={self.tooling_config.servo_id}, "
                f"feedback={self.tooling_current_position}, "
                f"target={self.tooling_target_position}, "
                f"range={self.tooling_config.raw_min}-{self.tooling_config.raw_max}"
            )

        imu_angles = self.sensor_snapshot.imu_angles_deg
        if imu_angles is not None:
            imu_text = (
                "imu: "
                f"roll={float(imu_angles.get('roll', 0.0)):+.2f} deg, "
                f"pitch={float(imu_angles.get('pitch', 0.0)):+.2f} deg, "
                f"yaw={float(imu_angles.get('yaw', 0.0)):+.2f} deg"
            )
            if self.sensor_snapshot.imu_age_ms is not None:
                imu_text += f", age_ms={self.sensor_snapshot.imu_age_ms:.0f}"
            lines.append(imu_text)

        detection = self.sensor_snapshot.primary_detection
        if detection is not None:
            position_m = detection.get("position_m") if isinstance(detection.get("position_m"), dict) else {}
            orientation = detection.get("orientation_deg") if isinstance(detection.get("orientation_deg"), dict) else {}
            tag_text = (
                "apriltag: "
                f"id={detection.get('id')}, "
                f"x={float(position_m.get('x', 0.0)):+.3f} m, "
                f"y={float(position_m.get('y', 0.0)):+.3f} m, "
                f"z={float(position_m.get('z', 0.0)):+.3f} m"
            )
            if orientation:
                tag_text += f", yaw={float(orientation.get('yaw', 0.0)):+.2f} deg"
            if self.sensor_snapshot.apriltag_age_ms is not None:
                tag_text += f", age_ms={self.sensor_snapshot.apriltag_age_ms:.0f}"
            lines.append(tag_text)

        if self.vision_tool_preview is not None:
            vision_xyz = self.vision_tool_preview.get("tool_position_mm", [0.0, 0.0, 0.0])
            delta_ik = self.vision_tool_preview.get("delta_ik", {})
            if isinstance(vision_xyz, list) and len(vision_xyz) >= 3:
                lines.append(
                    "vision base_T_tool: "
                    f"X={float(vision_xyz[0]):+.2f} mm, "
                    f"Y={float(vision_xyz[1]):+.2f} mm, "
                    f"Z={float(vision_xyz[2]):+.2f} mm, "
                    f"ik={bool(delta_ik.get('reachable', False))}"
                )
        elif self.vision_tool_preview_error:
            lines.append(f"vision base_T_tool: unavailable ({self.vision_tool_preview_error})")

        if self.battery_voltage_mv is not None:
            lines.append(f"驱动板电压: {self.battery_voltage_mv} mV")
        if self.safety_fault_message:
            lines.append(f"安全状态: {self.safety_fault_message}")

        return "\n".join(lines) + "\n"

    def write_runtime_status(self) -> None:
        try:
            self.runtime_status_path.write_text(
                self.build_status_snapshot(),
                encoding="utf-8",
            )
        except OSError:
            pass

    def confirm_startup_pose(self) -> bool:
        print("\n启动前请确认机械臂周围无遮挡，舵机供电稳定。")
        print("程序会先读取当前舵机角度；如果不在准备位，会要求输入 HOME 后慢速回到预设初始位姿。")
        for servo_id in self.servo_ids:
            print(f"  舵机 {servo_id}: 准备位 {self.reference_servo_positions[servo_id]}")

        response = input("\n确认可以开始初始化吗？(y/n): ").strip().lower()
        return response == "y"

    def init_reference_pose(self) -> bool:
        try:
            angles_rad, success = inverse_kinematics(
                self.reference_position[0],
                self.reference_position[1],
                self.reference_position[2],
                self.params,
            )
            if not success:
                print("参考位姿逆运动学计算失败。")
                return False

            self.reference_angles_rad = angles_rad.copy()
            self.current_angles_rad = angles_rad.copy()
            self.target_angles_rad = angles_rad.copy()
            self.current_position = self.reference_position.copy()
            self.target_position = self.reference_position.copy()

            print("\n参考位姿初始化完成。")
            print(
                f"空载调试参数: 舵机限速={self.max_servo_speed_ticks_per_sec:.0f} 刻度/秒, "
                f"XY速度={self.speed_xy:.0f} mm/s, Z速度={self.speed_z:.0f} mm/s, "
                f"工作空间裁剪={'开启' if self.enforce_workspace_bounds else '关闭'}"
            )
            print(
                f"参考末端位置: X={self.reference_position[0]:.1f}, "
                f"Y={self.reference_position[1]:.1f}, "
                f"Z={self.reference_position[2]:.1f}"
            )
            print(
                "参考关节角(度): "
                + ", ".join(f"{value:.2f}" for value in np.degrees(self.reference_angles_rad))
            )
            return True
        except Exception as exc:
            print(f"参考位姿初始化失败: {exc}")
            return False

    def servo_positions_to_angles(self, servo_positions: dict[int, int]) -> np.ndarray:
        if self.reference_angles_rad is None:
            raise RuntimeError("参考角度尚未初始化")

        angles_rad = np.zeros(3, dtype=float)
        for index, servo_id in enumerate(self.servo_ids):
            current_coord = self.servo_position_to_coord(servo_id, servo_positions[servo_id])
            delta_coord = current_coord - self.reference_servo_coords[servo_id]
            delta_deg = delta_coord / (
                self.servo_logical_directions[servo_id] * self.servo_units_per_degree[servo_id]
            )
            angles_rad[index] = self.reference_angles_rad[index] + np.radians(delta_deg)
        return angles_rad

    def sync_servo_feedback(
        self,
        *,
        force: bool = False,
        strict: bool = False,
        require_pose: bool | None = None,
    ) -> bool:
        if require_pose is None:
            require_pose = strict

        now = time.perf_counter()
        if not force and now - self.last_feedback_poll_time < self.feedback_interval:
            return True

        self.last_feedback_poll_time = now

        try:
            read_timeout = self.startup_feedback_timeout if force or strict else self.feedback_timeout
            feedback_positions = self.driver.read_servo_positions(self.servo_ids, timeout=read_timeout)
            feedback_positions = {
                servo_id: self.quantize_servo_position(servo_id, int(feedback_positions[servo_id]))
                for servo_id in self.servo_ids
            }
            self.debug_log(f"FEEDBACK positions={feedback_positions}")

            if feedback_positions != self.current_servo_positions:
                self.last_feedback_change_time = now

            self.current_servo_positions = feedback_positions
            self.current_angles_rad = self.servo_positions_to_angles(feedback_positions)

            pose, success = forward_kinematics(
                self.current_angles_rad[0],
                self.current_angles_rad[1],
                self.current_angles_rad[2],
                self.params,
            )
            if success:
                self.current_position = pose
                self.current_pose_valid = True
            else:
                self.current_pose_valid = False
                self.debug_log(
                    "FEEDBACK_FK_FAIL "
                    f"angles_deg={[round(float(value), 3) for value in np.degrees(self.current_angles_rad)]}"
                )
            if not success and require_pose:
                raise RuntimeError("反馈角度无法通过正运动学恢复当前位姿")

            if now - self.last_voltage_poll_time >= 1.0:
                self.last_voltage_poll_time = now
                try:
                    self.battery_voltage_mv = self.driver.get_battery_voltage_mv(timeout=self.feedback_timeout)
                except Exception:
                    pass

            self.sync_tooling_feedback(force=force)

            self.feedback_failure_count = 0
            return True
        except Exception as exc:
            self.feedback_failure_count += 1
            self.debug_log(f"FEEDBACK_ERROR count={self.feedback_failure_count} error={exc}")
            if strict or self.feedback_failure_count >= self.feedback_failure_limit:
                self.safety_fault_message = f"驱动板反馈读取失败: {exc}"
                return False
            return True

    def sync_command_state_to_feedback(self, reason: str) -> None:
        if self.current_angles_rad is None:
            return

        self.target_position = self.current_position.copy()
        self.target_angles_rad = self.current_angles_rad.copy()
        self.target_servo_positions = self.current_servo_positions.copy()
        self.command_servo_positions = self.current_servo_positions.copy()
        self.last_sent_positions = self.current_servo_positions.copy()
        self.last_send_time = time.perf_counter()
        self.last_feedback_change_time = self.last_send_time
        self.last_motion_command_time = None
        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        self.debug_log(
            f"COMMAND_STATE_SYNC reason={reason} "
            f"xyz=({self.current_position[0]:.3f},{self.current_position[1]:.3f},{self.current_position[2]:.3f}) "
            f"raw={self.current_servo_positions}"
        )

    def startup_home_to_reference(self) -> bool:
        if self.reference_angles_rad is None or self.current_angles_rad is None:
            return False

        ok, target_position, target_angles, target_raw = self.resolve_motion_target(
            self.reference_position,
            "startup-home",
        )
        if not ok:
            print("Startup home pose is not reachable with current limits.")
            return False

        raw_errors = {
            servo_id: abs(target_raw[servo_id] - self.current_servo_positions[servo_id])
            for servo_id in self.servo_ids
        }
        max_error = max(raw_errors.values()) if raw_errors else 0
        if max_error <= self.startup_tolerance_ticks:
            print("Startup feedback is already near the reference pose.")
            return True

        print("")
        print("Startup homing required.")
        current_xyz_text = self.format_xyz(self.current_position) if self.current_pose_valid else "unavailable"
        print(f"  current xyz: {current_xyz_text}")
        print(f"  target  xyz: {self.format_xyz(target_position)}")
        print(f"  current raw: {self.current_servo_positions}")
        print(f"  target  raw: {target_raw}")
        print(f"  speed       : {self.startup_home_servo_speed_ticks_per_sec:.0f} raw/s")
        response = input("Type HOME to slowly move to the reference pose: ").strip()
        if response != "HOME":
            print("Startup homing cancelled.")
            return False

        previous_ready = self.is_ready
        homing_complete = False

        self.sync_command_state_to_feedback("startup_home_begin")
        self.target_position = target_position
        self.target_angles_rad = target_angles
        self.target_servo_positions = target_raw
        self.is_ready = True

        saved_servo_speed = self.max_servo_speed_ticks_per_sec
        self.max_servo_speed_ticks_per_sec = self.startup_home_servo_speed_ticks_per_sec
        deadline = time.perf_counter() + max(
            self.startup_home_timeout_sec,
            (max_error / max(1.0, self.startup_home_servo_speed_ticks_per_sec)) * 3.0 + 3.0,
        )
        try:
            while self.target_servo_error_ticks() > self.startup_tolerance_ticks:
                if time.perf_counter() >= deadline:
                    self.safety_fault_message = (
                        f"startup homing timeout: error_ticks={self.target_servo_error_ticks()}"
                    )
                    print(self.safety_fault_message)
                    return False
                if not self.send_servo_positions():
                    print(self.safety_fault_message or "Startup homing send failed.")
                    return False
                if not self.sync_servo_feedback():
                    print(self.safety_fault_message or "Startup homing feedback sync failed.")
                    return False
                if not self.check_safety_guard():
                    print(self.safety_fault_message)
                    return False
                time.sleep(self.update_interval)

            if not self.sync_servo_feedback(force=True, strict=True, require_pose=True):
                print(self.safety_fault_message or "Startup final feedback read failed.")
                return False

            final_error = self.target_servo_error_ticks()
            if final_error > self.startup_tolerance_ticks:
                self.safety_fault_message = f"startup homing did not reach tolerance: {final_error} ticks"
                print(self.safety_fault_message)
                return False

            print("Startup homing complete.")
            homing_complete = True
            return True
        finally:
            self.max_servo_speed_ticks_per_sec = saved_servo_speed
            if not homing_complete:
                self.is_ready = previous_ready

    def confirm_and_init(self) -> bool:
        if not self.confirm_startup_pose():
            print("用户取消了初始化。")
            return False

        if not self.init_reference_pose():
            return False

        if not self.sync_servo_feedback(force=True, strict=True, require_pose=False):
            print(self.safety_fault_message or "无法读取驱动板反馈。")
            return False

        self.sync_command_state_to_feedback("startup_feedback")

        mismatches = []
        for servo_id in self.servo_ids:
            actual = self.quantize_servo_position(servo_id, self.current_servo_positions[servo_id])
            expected = self.quantize_servo_position(servo_id, self.reference_servo_positions[servo_id])
            if abs(actual - expected) > self.startup_tolerance_ticks:
                mismatches.append((servo_id, actual, expected))

        if mismatches:
            print("\n驱动板反馈显示当前舵机不在准备位。")
            for servo_id, actual, expected in mismatches:
                print(f"  舵机 {servo_id}: 当前 {actual}, 期望 {expected}")
            print("将使用当前反馈作为起点，二次确认后慢速回到准备位。")
            if not self.startup_home_to_reference():
                return False
            if not self.sync_servo_feedback(force=True, strict=True, require_pose=True):
                print(self.safety_fault_message or "启动回位后无法读取驱动板反馈。")
                return False
            self.sync_command_state_to_feedback("startup_home_done")
        elif not self.current_pose_valid:
            print("舵机反馈已接近准备位，但当前几何参数无法通过正运动学恢复末端 XYZ，已拒绝进入控制。")
            return False

        self.target_position = self.current_position.copy()
        self.target_angles_rad = self.current_angles_rad.copy()
        self.target_servo_positions = self.current_servo_positions.copy()
        self.command_servo_positions = self.current_servo_positions.copy()
        self.last_sent_positions = self.command_servo_positions.copy()
        self.last_send_time = time.perf_counter()
        self.last_feedback_change_time = self.last_send_time
        self.last_motion_command_time = None
        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        self.debug_log(
            f"READY current_raw={self.current_servo_positions} target_xyz={self.target_position.tolist()} "
            f"reference_raw={self.reference_servo_positions} reference_coord={self.reference_servo_coords}"
        )
        self.write_runtime_status()
        print("系统就绪，可以开始手柄控制。")
        self.is_ready = True
        return True

    def angles_to_servo_positions(self, angles_rad: np.ndarray, *, strict: bool = False) -> dict[int, int]:
        if self.reference_angles_rad is None:
            raise RuntimeError("参考角度尚未初始化")

        target_positions: dict[int, int] = {}
        for index, servo_id in enumerate(self.servo_ids):
            delta_deg = float(np.degrees(angles_rad[index] - self.reference_angles_rad[index]))
            target_coord = (
                self.reference_servo_coords[servo_id]
                + self.servo_logical_directions[servo_id] * delta_deg * self.servo_units_per_degree[servo_id]
            )
            mapping = self.servo_mappings[servo_id]
            logical_low = min(mapping.logical_min, mapping.logical_max)
            logical_high = max(mapping.logical_min, mapping.logical_max)
            if target_coord < logical_low or target_coord > logical_high:
                message = (
                    f"servo={servo_id} coord={target_coord:.3f} "
                    f"logical_limits=({logical_low:.3f},{logical_high:.3f})"
                )
                self.debug_log(f"RAW_RANGE_FAIL {message}")
                if strict:
                    raise ValueError(message)

            min_pos, max_pos = self.servo_limits[servo_id]
            quantized_position = mapping.logical_to_raw(target_coord)
            if quantized_position <= min_pos or quantized_position >= max_pos:
                self.debug_log(
                    f"RAW_CLAMP servo={servo_id} coord={target_coord:.3f} raw={quantized_position} "
                    f"limits=({min_pos},{max_pos})"
                )
            target_positions[servo_id] = max(min_pos, min(max_pos, quantized_position))

        return target_positions

    def compute_next_servo_command(self) -> tuple[dict[int, int], int]:
        if self.target_angles_rad is None:
            return self.command_servo_positions.copy(), self.min_command_time_ms

        desired_positions = self.angles_to_servo_positions(self.target_angles_rad, strict=True)
        self.target_servo_positions = desired_positions
        self.debug_log(
            f"COMMAND_DESIRED feedback={self.current_servo_positions} command={self.command_servo_positions} "
            f"desired={desired_positions} "
            f"target_xyz=({self.target_position[0]:.3f},{self.target_position[1]:.3f},{self.target_position[2]:.3f})"
        )

        now = time.perf_counter()
        if self.last_send_time is None:
            self.last_send_time = now
        elapsed = max(now - self.last_send_time, self.update_interval)
        self.last_send_time = now

        next_positions: dict[int, int] = {}
        max_move_ticks = 0

        for servo_id in self.servo_ids:
            self.servo_tick_budget[servo_id] += self.max_servo_speed_ticks_per_sec * elapsed

            current_position = self.command_servo_positions[servo_id]
            desired_position = desired_positions[servo_id]
            position_error = desired_position - current_position
            available_ticks = int(self.servo_tick_budget[servo_id])

            if abs(position_error) <= self.position_tolerance_ticks:
                next_position = current_position
                self.servo_tick_budget[servo_id] = 0.0
            elif available_ticks < self.min_effective_move_ticks:
                next_position = current_position
            else:
                move_ticks = min(abs(position_error), available_ticks)
                if move_ticks < self.min_effective_move_ticks and abs(position_error) > self.position_tolerance_ticks:
                    next_position = current_position
                else:
                    direction = 1 if position_error > 0 else -1
                    next_position = current_position + direction * move_ticks
                    self.servo_tick_budget[servo_id] -= move_ticks
                    max_move_ticks = max(max_move_ticks, move_ticks)

            min_pos, max_pos = self.servo_limits[servo_id]
            next_positions[servo_id] = max(min_pos, min(max_pos, next_position))

        time_ms = (
            max(self.min_command_time_ms, int((max_move_ticks / self.max_servo_speed_ticks_per_sec) * 1000))
            if max_move_ticks
            else self.min_command_time_ms
        )
        self.debug_log(f"COMMAND_NEXT next={next_positions} time_ms={time_ms} max_move_ticks={max_move_ticks}")
        return next_positions, time_ms

    def send_servo_positions(self) -> bool:
        if not self.is_ready or not self.driver.ser or not self.driver.ser.is_open:
            return False

        try:
            next_positions, time_ms = self.compute_next_servo_command()
            tooling_changed = (
                self.tooling_config is not None
                and self.tooling_target_position is not None
                and self.tooling_target_position != self.last_sent_tooling_position
            )
            if next_positions == self.last_sent_positions and not tooling_changed:
                self.debug_log(f"SEND_SKIP unchanged next={next_positions}")
                return True

            targets = [(servo_id, next_positions[servo_id]) for servo_id in self.servo_ids]
            if self.tooling_config is not None and self.tooling_target_position is not None:
                targets.append((self.tooling_config.servo_id, self.tooling_target_position))
            self.debug_log(f"SEND targets={targets} time_ms={time_ms}")
            self.driver.set_servo_positions(targets, time_ms)
            self.command_servo_positions = next_positions.copy()
            self.last_sent_positions = next_positions.copy()
            if self.tooling_config is not None:
                self.last_sent_tooling_position = self.tooling_target_position
            self.last_motion_command_time = time.perf_counter()
            return True
        except Exception as exc:
            self.safety_fault_message = f"发送舵机指令失败: {exc}"
            self.debug_log(f"SEND_ERROR error={exc}")
            return False

    def check_safety_guard(self) -> bool:
        if not self.enable_stall_guard:
            return True

        if self.last_feedback_change_time is None or self.last_motion_command_time is None:
            return True

        max_error = max(
            abs(self.target_servo_positions[servo_id] - self.current_servo_positions[servo_id])
            for servo_id in self.servo_ids
        )
        if max_error < self.stall_error_ticks:
            return True

        now = time.perf_counter()
        if now - self.last_motion_command_time < self.stall_timeout:
            return True

        if self.last_feedback_change_time >= self.last_motion_command_time:
            return True

        self.safety_fault_message = "检测到舵机目标与反馈长期偏差过大，疑似触碰机械边界或发生堵转，已自动停机。"
        return False

    def update_from_gamepad(self) -> Tuple[bool, bool]:
        if not self.gamepad.is_available():
            return True, False

        previous_buttons = self.last_buttons.copy()
        dpad_x, dpad_y, right_y, buttons = self.gamepad.read()
        self.last_axes_raw = (dpad_x, dpad_y, right_y)
        dpad_x, dpad_y, right_y = self.filter_axes(dpad_x, dpad_y, right_y)
        self.last_axes = (dpad_x, dpad_y, right_y)
        dpad_left_x, dpad_left_y, dpad_right_y = self.gamepad_to_dpad_axes(dpad_x, dpad_y, right_y)
        self.last_dpad_axes = (dpad_left_x, dpad_left_y, dpad_right_y)
        motion_left_x, motion_left_y, motion_right_y = self.smooth_dpad_axes(self.last_dpad_axes)
        self.last_motion_dpad_axes = (motion_left_x, motion_left_y, motion_right_y)
        self.last_buttons = buttons.copy()
        self.debug_log(
            "GAMEPAD "
            f"raw=({self.last_axes_raw[0]:+.3f},{self.last_axes_raw[1]:+.3f},{self.last_axes_raw[2]:+.3f}) "
            f"filtered=({dpad_x:+.3f},{dpad_y:+.3f},{right_y:+.3f}) "
            f"dpad=({dpad_left_x:+.0f},{dpad_left_y:+.0f},{dpad_right_y:+.0f}) "
            f"motion=({motion_left_x:+.2f},{motion_left_y:+.2f},{motion_right_y:+.2f}) "
            f"buttons={buttons}"
        )

        if buttons["a"]:
            return False, False

        if buttons.get("b", False) and not previous_buttons.get("b", False):
            self.record_current_point()

        if buttons.get("x", False) and not previous_buttons.get("x", False):
            self.cycle_safe_scan_mode()

        if buttons.get("y", False) and not previous_buttons.get("y", False):
            self.cycle_sensor_frame_mode()

        if buttons.get("back", False) and not previous_buttons.get("back", False):
            self.cycle_playback_mode()

        if buttons.get("start", False) and not previous_buttons.get("start", False):
            self.play_last_sample_segment()
            return True, False

        tooling_changed = self.update_tooling_from_buttons(buttons)
        if max(abs(motion_left_x), abs(motion_left_y), abs(motion_right_y)) < 0.01:
            self.debug_log(f"GAMEPAD_IDLE tooling_changed={tooling_changed}")
            return True, tooling_changed

        new_position = self.user_motion_origin()
        delta_x_user = motion_left_x * self.speed_xy * self.update_interval
        delta_y_user = motion_left_y * self.speed_xy * self.update_interval
        cos_angle = float(np.cos(self.xy_rotation_rad))
        sin_angle = float(np.sin(self.xy_rotation_rad))
        delta_x_model = cos_angle * delta_x_user - sin_angle * delta_y_user
        delta_y_model = sin_angle * delta_x_user + cos_angle * delta_y_user
        delta_z_model = -motion_right_y * self.speed_z * self.update_interval

        if self.safe_scan_mode == "X":
            delta_y_model = 0.0
            delta_z_model = 0.0
        elif self.safe_scan_mode == "Y":
            delta_x_model = 0.0
            delta_z_model = 0.0
        elif self.safe_scan_mode == "Z":
            delta_x_model = 0.0
            delta_y_model = 0.0

        new_position[0] += delta_x_model
        new_position[1] += delta_y_model
        new_position[2] += delta_z_model
        self.debug_log(
            f"POSE_DELTA dxyz=({delta_x_model:+.3f},{delta_y_model:+.3f},{delta_z_model:+.3f}) "
            f"candidate=({new_position[0]:.3f},{new_position[1]:.3f},{new_position[2]:.3f})"
        )

        new_position = self.clamp_position_to_workspace(new_position)

        if np.allclose(new_position, self.target_position, atol=1e-6):
            self.debug_log("POSE_SKIP unchanged_after_limits")
            return True, tooling_changed

        if not self.set_motion_target(new_position, "gamepad"):
            return True, tooling_changed

        return True, True

    def print_status(self) -> None:
        self.write_runtime_status()

    def run(self) -> None:
        if not self.connect():
            self.cleanup()
            return

        if not self.confirm_and_init():
            self.cleanup()
            return

        print("\n开始实时控制。")
        print("十字键控制 X/Y，右摇杆控制 Z，A 退出，B 记录点，X 切换 safe scan。")
        print("START -> 二次确认后执行最近两个采样点；BACK -> 切换 LINE / PICK_PLACE 路径模式。")
        print(f"实时状态覆盖写入: {self.runtime_status_path.name}")

        try:
            last_status_write = 0.0
            while True:
                if not self.sync_servo_feedback():
                    print(self.safety_fault_message or "反馈同步失败。")
                    break

                continue_run, _ = self.update_from_gamepad()
                if not continue_run:
                    print("\n收到退出指令。")
                    break

                if not self.send_servo_positions():
                    print(self.safety_fault_message or "发送指令失败。")
                    break

                if not self.check_safety_guard():
                    print(self.safety_fault_message)
                    break

                now = time.time()
                if now - last_status_write >= self.status_update_interval:
                    self.write_runtime_status()
                    last_status_write = now

                time.sleep(self.update_interval)
        except KeyboardInterrupt:
            print("\n程序已中断。")
        finally:
            self.cleanup()

    def run_controller(self) -> None:
        if not self.connect():
            self.cleanup()
            return

        if not self.confirm_and_init():
            self.cleanup()
            return

        self.sync_sensor_feedback(force=True)
        self.update_vision_tool_preview(force=True)
        print("\nStart realtime control.")
        print("D-pad -> X/Y, right stick -> Z, A -> quit, B -> record, X -> safe scan.")
        print("Y -> sensor frame mode, LB/RB -> tooling servo if servo4 is configured.")
        print("START -> confirmed replay between last two sampled points; BACK -> toggle LINE / PICK_PLACE.")
        print(f"Runtime status file: {self.runtime_status_path.name}")

        try:
            last_status_write = 0.0
            while True:
                if not self.sync_servo_feedback():
                    print(self.safety_fault_message or "Feedback sync failed.")
                    break

                self.sync_sensor_feedback()
                self.update_vision_tool_preview()
                continue_run, _ = self.update_from_gamepad()
                if not continue_run:
                    print("\nQuit command received.")
                    break

                if not self.send_servo_positions():
                    print(self.safety_fault_message or "Servo command send failed.")
                    break

                if not self.check_safety_guard():
                    print(self.safety_fault_message)
                    break

                now = time.time()
                if now - last_status_write >= self.status_update_interval:
                    self.write_runtime_status()
                    last_status_write = now

                time.sleep(self.update_interval)
        except KeyboardInterrupt:
            print("\nController interrupted.")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        try:
            self.flush_debug_log()
            self.driver.close()
            print("串口已关闭。")
        except Exception:
            pass
        try:
            self.flush_debug_log()
            self.debug_log_file.close()
        except Exception:
            pass


def main() -> None:
    try:
        port = input("输入串口 (默认 COM9): ").strip() or "COM9"
        controller = RealTimeArmController(port=port)
        controller.run_controller()
    except Exception as exc:
        print(f"程序出错: {exc}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
