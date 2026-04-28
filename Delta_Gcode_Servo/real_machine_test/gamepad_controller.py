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

    def __init__(self, deadzone: float = 0.0):
        self.deadzone = deadzone
        self.pygame = None
        self.joystick = None
        self.axis_baselines: list[float] = []
        self.right_y_axis = 4

        try:
            import pygame

            pygame.init()
            pygame.joystick.init()
            self.pygame = pygame

            if pygame.joystick.get_count() == 0:
                print("未检测到手柄。")
                return

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self._capture_axis_baselines()
            self.right_y_axis = self._select_right_y_axis()

            print(f"检测到手柄: {self.joystick.get_name()}")
            print(f"轴数量: {self.joystick.get_numaxes()}")
            print(f"按钮数量: {self.joystick.get_numbuttons()}")
            print(f"当前死区: {self.deadzone:.2f}")
            print(f"右摇杆Y轴映射: axis {self.right_y_axis}")
        except Exception as exc:
            print(f"手柄初始化失败: {exc}")
            self.joystick = None

    def is_available(self) -> bool:
        return self.joystick is not None

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

    def read(self) -> Tuple[float, float, float, dict[str, bool]]:
        """返回 (left_x, left_y, right_y, buttons)。"""
        if not self.joystick or self.pygame is None:
            return 0.0, 0.0, 0.0, {"a": False, "b": False, "x": False, "y": False, "lb": False, "rb": False}

        try:
            self.pygame.event.pump()

            left_x = self._read_axis(0)
            left_y = self._read_axis(1)

            right_y = self._read_axis(self.right_y_axis)

            buttons = {
                "a": self._read_button(0),
                "b": self._read_button(1),
                "x": self._read_button(2),
                "y": self._read_button(3),
                "lb": self._read_button(4),
                "rb": self._read_button(5),
            }
            return left_x, left_y, right_y, buttons
        except Exception as exc:
            print(f"读取手柄失败: {exc}")
            return 0.0, 0.0, 0.0, {"a": False, "b": False, "x": False, "y": False, "lb": False, "rb": False}


class RealTimeArmController:
    """实机测试控制器。"""

    def __init__(self, port: str = "COM9", baudrate: int = 9600):
        self.driver = BusServoDriver(port=port, baudrate=baudrate, connect_delay=0.2)
        self.robot = DeltaRobot()
        self.params = robot_params()
        self.port = port
        self.project_root = Path(__file__).resolve().parents[2]
        self.servo_ids = [1, 2, 3]
        self.servo_step_ticks = 10

        raw_reference_servo_positions = {1: 988, 2: 920, 3: 1000}
        raw_servo_limits = {1: (500, 988), 2: (500, 920), 3: (500, 1000)}
        self.reference_servo_positions = {
            servo_id: self.quantize_servo_position(position)
            for servo_id, position in raw_reference_servo_positions.items()
        }
        self.servo_limits = {
            servo_id: (
                self.quantize_servo_position(raw_min),
                self.quantize_servo_position(raw_max),
            )
            for servo_id, (raw_min, raw_max) in raw_servo_limits.items()
        }
        self.startup_tolerance_ticks = 25
        self.reference_position = np.array(self.params.home_position, dtype=float)

        # 空载调试模式: 把参考位放在模型中间层，避免一上来就贴着 Z 上边界。
        self.reference_position = np.array(self.params.home_position, dtype=float)
        self.reference_angles_rad: np.ndarray | None = None

        self.current_servo_positions = self.reference_servo_positions.copy()
        self.current_angles_rad: np.ndarray | None = None
        self.current_position = self.reference_position.copy()

        self.target_servo_positions = self.reference_servo_positions.copy()
        self.target_angles_rad: np.ndarray | None = None
        self.target_position = self.reference_position.copy()

        self.servo_ticks_per_degree = 1000.0 / 240.0
        self.servo_directions = {1: -1, 2: -1, 3: -1}

        self.max_servo_speed_ticks_per_sec = 300.0
        self.min_effective_move_ticks = 10
        self.position_tolerance_ticks = 0
        self.speed_xy = 120.0
        self.speed_z = 100.0
        self.update_rate = 50
        self.update_interval = 1.0 / self.update_rate
        self.axis_filter_alpha = 0.35
        self.filtered_axes = np.zeros(3, dtype=float)
        self.xy_rotation_rad = 0.0
        self.enforce_workspace_bounds = False
        self.enable_stall_guard = False

        self.feedback_interval = 0.05
        self.feedback_timeout = 0.2
        self.feedback_failure_limit = 5
        self.stall_error_ticks = 20
        self.stall_timeout = 1.2

        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        self.last_sent_positions = self.current_servo_positions.copy()
        self.last_send_time: float | None = None
        self.last_feedback_poll_time = 0.0
        self.last_feedback_change_time: float | None = None
        self.last_motion_command_time: float | None = None
        self.last_voltage_poll_time = 0.0
        self.last_sensor_poll_time = 0.0
        self.feedback_failure_count = 0
        self.battery_voltage_mv: int | None = None
        self.last_axes = (0.0, 0.0, 0.0)
        self.last_axes_raw = (0.0, 0.0, 0.0)
        self.last_buttons = {"a": False, "b": False, "x": False, "y": False, "lb": False, "rb": False}
        self.safety_fault_message: str | None = None
        self.safe_scan_mode = "FREE"
        self.record_file_path = Path(__file__).with_name("workspace_points.csv")
        self.runtime_status_path = Path(__file__).with_name("runtime_status.log")
        self.record_count = 0
        self.status_update_interval = 0.2
        self.sensor_poll_interval = 0.10
        self.sensor_frame_mode = "OFF"
        self.sensor_heading_zero_deg: float | None = None
        self.sensor_snapshot = SensorSnapshot()
        self.imu_snapshot_path = self.project_root / "IMU" / "wt61c_latest.json"
        self.apriltag_snapshot_path = self.project_root / "AprilTag_Vision" / "myAprilTag" / "output" / "apriltag_latest.json"
        self.tooling_config = load_tooling_servo_config(
            self.project_root / "lx225_tool_demo" / "config" / "lx225_tool.demo.toml"
        )
        self.tooling_speed_ticks_per_sec = 180.0
        self.tooling_current_position: int | None = None
        self.tooling_target_position: int | None = None
        self.last_sent_tooling_position: int | None = None
        self.is_ready = False

        self.gamepad = GamepadReader(deadzone=0.05)

    def connect(self) -> bool:
        try:
            print(f"\n正在连接串口 {self.port}...")
            self.driver.connect()
            print("串口连接成功。")

            if not self.gamepad.is_available():
                print("手柄未初始化，无法进入控制。")
                return False

            print("手柄连接成功。")
            return True
        except Exception as exc:
            print(f"连接失败: {exc}")
            return False

    def quantize_servo_position(self, value: int | float) -> int:
        step = self.servo_step_ticks
        return int(np.floor((float(value) / step) + 0.5) * step)

    def edge_pressed(self, name: str, buttons: dict[str, bool]) -> bool:
        return buttons.get(name, False) and not self.last_buttons.get(name, False)

    def filter_axes(self, left_x: float, left_y: float, right_y: float) -> tuple[float, float, float]:
        raw_axes = np.array([left_x, left_y, right_y], dtype=float)
        self.filtered_axes = (
            self.axis_filter_alpha * raw_axes
            + (1.0 - self.axis_filter_alpha) * self.filtered_axes
        )
        self.filtered_axes[np.abs(raw_axes) < 0.01] = 0.0
        return tuple(float(value) for value in self.filtered_axes)

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

    def sync_tooling_feedback(self) -> None:
        if self.tooling_config is None:
            return

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

            print(
                "已记录工作空间点: "
                f"#{self.record_count} -> "
                f"X={self.current_position[0]:.2f}, "
                f"Y={self.current_position[1]:.2f}, "
                f"Z={self.current_position[2]:.2f}"
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
            f"手柄轴映射: right_y_axis={self.gamepad.right_y_axis}",
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
            f"摇杆输入: LX={self.last_axes[0]:+.3f}, LY={self.last_axes[1]:+.3f}, RY={self.last_axes[2]:+.3f}",
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
        print("\n启动前请确认机械结构已经在安全准备位。")
        print("程序会通过驱动板透明读取当前位置，并与准备位进行比对。")
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
            delta_ticks = servo_positions[servo_id] - self.reference_servo_positions[servo_id]
            delta_deg = delta_ticks / (self.servo_directions[servo_id] * self.servo_ticks_per_degree)
            angles_rad[index] = self.reference_angles_rad[index] + np.radians(delta_deg)
        return angles_rad

    def sync_servo_feedback(self, *, force: bool = False, strict: bool = False) -> bool:
        now = time.perf_counter()
        if not force and now - self.last_feedback_poll_time < self.feedback_interval:
            return True

        self.last_feedback_poll_time = now

        try:
            feedback_positions = self.driver.read_servo_positions(self.servo_ids, timeout=self.feedback_timeout)
            feedback_positions = {
                servo_id: self.quantize_servo_position(int(feedback_positions[servo_id]))
                for servo_id in self.servo_ids
            }

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
            elif strict:
                raise RuntimeError("反馈角度无法通过正运动学恢复当前位姿")

            if now - self.last_voltage_poll_time >= 1.0:
                self.last_voltage_poll_time = now
                try:
                    self.battery_voltage_mv = self.driver.get_battery_voltage_mv(timeout=self.feedback_timeout)
                except Exception:
                    pass

            self.sync_tooling_feedback()

            self.feedback_failure_count = 0
            return True
        except Exception as exc:
            self.feedback_failure_count += 1
            if strict or self.feedback_failure_count >= self.feedback_failure_limit:
                self.safety_fault_message = f"驱动板反馈读取失败: {exc}"
                return False
            return True

    def confirm_and_init(self) -> bool:
        if not self.confirm_startup_pose():
            print("用户取消了初始化。")
            return False

        if not self.init_reference_pose():
            return False

        if not self.sync_servo_feedback(force=True, strict=True):
            print(self.safety_fault_message or "无法读取驱动板反馈。")
            return False

        mismatches = []
        for servo_id in self.servo_ids:
            actual = self.quantize_servo_position(self.current_servo_positions[servo_id])
            expected = self.quantize_servo_position(self.reference_servo_positions[servo_id])
            if abs(actual - expected) > self.startup_tolerance_ticks:
                mismatches.append((servo_id, actual, expected))

        if mismatches:
            print("\n驱动板反馈显示当前舵机不在准备位，已拒绝启动。")
            for servo_id, actual, expected in mismatches:
                print(f"  舵机 {servo_id}: 当前 {actual}, 期望 {expected}")
            print("请先在上位机或校准脚本中回到准备位。")
            return False

        self.target_position = self.current_position.copy()
        self.target_angles_rad = self.current_angles_rad.copy()
        self.target_servo_positions = self.current_servo_positions.copy()
        self.last_sent_positions = self.current_servo_positions.copy()
        self.last_send_time = time.perf_counter()
        self.last_feedback_change_time = self.last_send_time
        self.last_motion_command_time = None
        self.servo_tick_budget = {servo_id: 0.0 for servo_id in self.servo_ids}
        self.write_runtime_status()
        print("系统就绪，可以开始手柄控制。")
        self.is_ready = True
        return True

    def angles_to_servo_positions(self, angles_rad: np.ndarray) -> dict[int, int]:
        if self.reference_angles_rad is None:
            raise RuntimeError("参考角度尚未初始化")

        target_positions: dict[int, int] = {}
        for index, servo_id in enumerate(self.servo_ids):
            delta_deg = float(np.degrees(angles_rad[index] - self.reference_angles_rad[index]))
            raw_position = int(
                round(
                    self.reference_servo_positions[servo_id]
                    + self.servo_directions[servo_id] * delta_deg * self.servo_ticks_per_degree
                )
            )
            min_pos, max_pos = self.servo_limits[servo_id]
            quantized_position = self.quantize_servo_position(raw_position)
            target_positions[servo_id] = max(min_pos, min(max_pos, quantized_position))

        return target_positions

    def compute_next_servo_command(self) -> tuple[dict[int, int], int]:
        if self.target_angles_rad is None:
            return self.current_servo_positions.copy(), max(50, int(self.update_interval * 1000))

        desired_positions = self.angles_to_servo_positions(self.target_angles_rad)
        self.target_servo_positions = desired_positions

        now = time.perf_counter()
        if self.last_send_time is None:
            self.last_send_time = now
        elapsed = max(now - self.last_send_time, self.update_interval)
        self.last_send_time = now

        next_positions: dict[int, int] = {}
        max_move_ticks = 0

        for servo_id in self.servo_ids:
            self.servo_tick_budget[servo_id] += self.max_servo_speed_ticks_per_sec * elapsed

            current_position = self.current_servo_positions[servo_id]
            desired_position = desired_positions[servo_id]
            position_error = desired_position - current_position
            available_ticks = int(self.servo_tick_budget[servo_id])

            if abs(position_error) <= self.position_tolerance_ticks:
                next_position = current_position
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

        time_ms = max(50, int((max_move_ticks / self.max_servo_speed_ticks_per_sec) * 1000)) if max_move_ticks else 50
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
                return True

            targets = [(servo_id, next_positions[servo_id]) for servo_id in self.servo_ids]
            if self.tooling_config is not None and self.tooling_target_position is not None:
                targets.append((self.tooling_config.servo_id, self.tooling_target_position))
            self.driver.set_servo_positions(targets, time_ms)
            self.last_sent_positions = next_positions.copy()
            if self.tooling_config is not None:
                self.last_sent_tooling_position = self.tooling_target_position
            self.last_motion_command_time = time.perf_counter()
            return True
        except Exception as exc:
            self.safety_fault_message = f"发送舵机指令失败: {exc}"
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
        left_x, left_y, right_y, buttons = self.gamepad.read()
        self.last_axes_raw = (left_x, left_y, right_y)
        left_x, left_y, right_y = self.filter_axes(left_x, left_y, right_y)
        self.last_axes = (left_x, left_y, right_y)
        self.last_buttons = buttons.copy()

        if buttons["a"]:
            return False, False

        if buttons.get("b", False) and not previous_buttons.get("b", False):
            self.record_current_point()

        if buttons.get("x", False) and not previous_buttons.get("x", False):
            self.cycle_safe_scan_mode()

        if buttons.get("y", False) and not previous_buttons.get("y", False):
            self.cycle_sensor_frame_mode()

        tooling_changed = self.update_tooling_from_buttons(buttons)
        if max(abs(left_x), abs(left_y), abs(right_y)) < 0.01:
            return True, tooling_changed

        new_position = self.target_position.copy()
        delta_x_user = left_x * self.speed_xy * self.update_interval
        delta_y_user = -left_y * self.speed_xy * self.update_interval
        cos_angle = float(np.cos(self.xy_rotation_rad))
        sin_angle = float(np.sin(self.xy_rotation_rad))
        delta_x_model = cos_angle * delta_x_user - sin_angle * delta_y_user
        delta_y_model = sin_angle * delta_x_user + cos_angle * delta_y_user
        delta_z_model = -right_y * self.speed_z * self.update_interval

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

        if self.enforce_workspace_bounds:
            bounds = self.robot.get_workspace_bounds()
            new_position[0] = np.clip(new_position[0], bounds["x_min"], bounds["x_max"])
            new_position[1] = np.clip(new_position[1], bounds["y_min"], bounds["y_max"])
            new_position[2] = np.clip(new_position[2], bounds["z_min"], bounds["z_max"])

        if np.allclose(new_position, self.target_position, atol=1e-6):
            return True, tooling_changed

        angles_rad, success = inverse_kinematics(
            new_position[0],
            new_position[1],
            new_position[2],
            self.params,
        )
        if not success:
            return True, tooling_changed

        self.target_position = new_position
        self.target_angles_rad = angles_rad
        self.target_servo_positions = self.angles_to_servo_positions(angles_rad)
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
        print("左摇杆控制 X/Y，右摇杆控制 Z，A 退出，B 记录点，X 切换 safe scan。")
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
        print("\nStart realtime control.")
        print("Left stick -> X/Y, right stick -> Z, A -> quit, B -> record, X -> safe scan.")
        print("Y -> sensor frame mode, LB/RB -> tooling servo if servo4 is configured.")
        print(f"Runtime status file: {self.runtime_status_path.name}")

        try:
            last_status_write = 0.0
            while True:
                if not self.sync_servo_feedback():
                    print(self.safety_fault_message or "Feedback sync failed.")
                    break

                self.sync_sensor_feedback()
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
            self.driver.close()
            print("串口已关闭。")
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
