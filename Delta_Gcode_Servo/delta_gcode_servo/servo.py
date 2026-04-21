from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import serial

from .config import RobotParams, robot_params
from .gcode import parse_gcode_file
from .robot import DeltaRobot


class Packet:
    HEADER = b"\x55\x55"

    @staticmethod
    def pack(cmd: int, params: list[int]) -> bytes:
        length = 2 + len(params)
        packet = bytearray()
        packet.extend(Packet.HEADER)
        packet.append(length)
        packet.append(cmd)
        packet.extend(params)
        return bytes(packet)


class BusServoDriver:
    CMD_SERVO_MOVE = 0x03
    CMD_GET_BATTERY_VOLTAGE = 0x0F
    CMD_MULT_SERVO_UNLOAD = 0x14
    CMD_MULT_SERVO_POS_READ = 0x15

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0, connect_delay: float = 0.2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect_delay = connect_delay
        self.ser: serial.Serial | None = None

    def connect(self) -> None:
        if self.ser and self.ser.is_open:
            return
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )
        if self.connect_delay > 0:
            time.sleep(self.connect_delay)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _write_packet(self, cmd: int, params: list[int] | None = None) -> None:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")
        packet = Packet.pack(cmd, params or [])
        self.ser.write(packet)
        self.ser.flush()

    def _read_packet(
        self,
        *,
        expected_cmd: int | None = None,
        timeout: float | None = None,
    ) -> tuple[int, list[int]]:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")

        deadline = time.perf_counter() + (self.timeout if timeout is None else timeout)
        while time.perf_counter() < deadline:
            first = self.ser.read(1)
            if first != b"\x55":
                continue

            second = self.ser.read(1)
            if second != b"\x55":
                continue

            length_raw = self.ser.read(1)
            cmd_raw = self.ser.read(1)
            if len(length_raw) != 1 or len(cmd_raw) != 1:
                continue

            length = length_raw[0]
            cmd = cmd_raw[0]
            param_len = max(0, length - 2)
            payload = self.ser.read(param_len)
            if len(payload) != param_len:
                continue

            if expected_cmd is not None and cmd != expected_cmd:
                continue

            return cmd, list(payload)

        raise TimeoutError(f"Timed out waiting for controller response, expected_cmd={expected_cmd!r}")

    def set_servo_positions(self, targets: list[tuple[int, int]], time_ms: int) -> None:
        params = [len(targets), time_ms & 0xFF, (time_ms >> 8) & 0xFF]
        for servo_id, position in targets:
            params.extend([servo_id, position & 0xFF, (position >> 8) & 0xFF])
        self._write_packet(self.CMD_SERVO_MOVE, params)

    def get_battery_voltage_mv(self, *, timeout: float | None = None) -> int:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")
        self.ser.reset_input_buffer()
        self._write_packet(self.CMD_GET_BATTERY_VOLTAGE, [])
        _, payload = self._read_packet(expected_cmd=self.CMD_GET_BATTERY_VOLTAGE, timeout=timeout)
        if len(payload) != 2:
            raise RuntimeError(f"Unexpected battery payload length: {len(payload)}")
        return payload[0] | (payload[1] << 8)

    def read_servo_positions(
        self,
        servo_ids: list[int],
        *,
        timeout: float | None = None,
    ) -> dict[int, int]:
        if not servo_ids:
            return {}
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")

        request_ids = list(dict.fromkeys(int(servo_id) for servo_id in servo_ids))
        self.ser.reset_input_buffer()
        self._write_packet(self.CMD_MULT_SERVO_POS_READ, [len(request_ids), *request_ids])
        _, payload = self._read_packet(expected_cmd=self.CMD_MULT_SERVO_POS_READ, timeout=timeout)

        if not payload:
            raise RuntimeError("Empty servo position payload")

        count = payload[0]
        expected_len = 1 + count * 3
        if len(payload) != expected_len:
            raise RuntimeError(f"Unexpected servo position payload length: {len(payload)} != {expected_len}")

        positions: dict[int, int] = {}
        offset = 1
        for _ in range(count):
            servo_id = payload[offset]
            position = payload[offset + 1] | (payload[offset + 2] << 8)
            positions[servo_id] = position
            offset += 3

        missing_ids = [servo_id for servo_id in request_ids if servo_id not in positions]
        if missing_ids:
            raise RuntimeError(f"Controller response missing servo IDs: {missing_ids}")

        return positions

    def unload_servos(self, servo_ids: list[int]) -> None:
        if not servo_ids:
            return
        request_ids = list(dict.fromkeys(int(servo_id) for servo_id in servo_ids))
        self._write_packet(self.CMD_MULT_SERVO_UNLOAD, [len(request_ids), *request_ids])


@dataclass(frozen=True)
class ServoCommand:
    index: int
    source_line: int
    command: str
    x: float
    y: float
    z: float
    joint_angles_deg: list[float]
    servo_angles_deg: list[float]
    servo_positions: list[int]
    time_ms: int
    feed_rate: float | None


def _map_linear(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)


def joint_angles_to_servo_angles_deg(joint_angles_deg: np.ndarray, params: RobotParams) -> np.ndarray:
    servo_angles_deg = joint_angles_deg.astype(float).copy()
    return np.clip(
        servo_angles_deg,
        params.servo_physical_angle_min_deg,
        params.servo_physical_angle_max_deg,
    )


def servo_angles_to_positions(servo_angles_deg: np.ndarray, params: RobotParams) -> np.ndarray:
    positions = np.zeros(len(servo_angles_deg), dtype=int)
    for i, angle_deg in enumerate(servo_angles_deg):
        pos_float = _map_linear(
            float(angle_deg),
            params.servo_physical_angle_min_deg,
            params.servo_physical_angle_max_deg,
            params.servo_position_min,
            params.servo_position_max,
        )
        position = int(round(pos_float / params.servo_position_step) * params.servo_position_step)
        positions[i] = int(min(max(position, params.servo_position_min), params.servo_position_max))
    return positions


def _iter_interpolated_moves(
    moves: list[Any],
    *,
    params: RobotParams,
    default_time_ms: int,
) -> list[tuple[Any, int]]:
    if not moves:
        return []

    interpolated: list[tuple[Any, int]] = [(moves[0], default_time_ms)]
    max_step_mm = params.step_increment_linear
    if max_step_mm <= 0:
        return interpolated + [(move, default_time_ms) for move in moves[1:]]

    for previous, current in zip(moves, moves[1:]):
        start = np.array([previous.x, previous.y, previous.z], dtype=float)
        end = np.array([current.x, current.y, current.z], dtype=float)
        distance = float(np.linalg.norm(end - start))
        steps = max(1, int(math.ceil(distance / max_step_mm)))
        base_time_ms = max(1, default_time_ms // steps)
        remainder_ms = max(0, default_time_ms - (base_time_ms * steps))

        for step in range(1, steps + 1):
            ratio = step / steps
            point = start + (end - start) * ratio
            move_time_ms = base_time_ms + (1 if step <= remainder_ms else 0)
            interpolated.append(
                (
                    current.__class__(
                        x=float(point[0]),
                        y=float(point[1]),
                        z=float(point[2]),
                        feed_rate=current.feed_rate,
                        command=current.command,
                    ),
                    move_time_ms,
                )
            )

    return interpolated


def build_servo_commands_from_gcode(
    gcode_path: str | Path,
    *,
    params: RobotParams | None = None,
    time_ms: int = 100,
) -> list[ServoCommand]:
    params = params or robot_params()
    robot = DeltaRobot(params)
    moves = parse_gcode_file(gcode_path)
    commands: list[ServoCommand] = []
    interpolated_moves = _iter_interpolated_moves(moves, params=params, default_time_ms=time_ms)

    for index, (move, move_time_ms) in enumerate(interpolated_moves, start=1):
        theta_rad, valid = robot.compute_ik(move.x, move.y, move.z)
        if not valid:
            raise ValueError(
                f"Unreachable G-code point at step {index}: X{move.x:.3f} Y{move.y:.3f} Z{move.z:.3f}"
            )
        joint_angles_deg = np.rad2deg(theta_rad)
        servo_angles_deg = joint_angles_to_servo_angles_deg(joint_angles_deg, params)
        servo_positions = servo_angles_to_positions(servo_angles_deg, params)
        commands.append(
            ServoCommand(
                index=index,
                source_line=index + 5,
                command=move.command,
                x=move.x,
                y=move.y,
                z=move.z,
                joint_angles_deg=joint_angles_deg.round(6).tolist(),
                servo_angles_deg=servo_angles_deg.round(6).tolist(),
                servo_positions=servo_positions.tolist(),
                time_ms=move_time_ms,
                feed_rate=move.feed_rate,
            )
        )

    return commands


def export_servo_commands_json(
    gcode_path: str | Path,
    output_path: str | Path,
    *,
    params: RobotParams | None = None,
    time_ms: int = 100,
) -> Path:
    params = params or robot_params()
    commands = build_servo_commands_from_gcode(gcode_path, params=params, time_ms=time_ms)
    payload: dict[str, Any] = {
        "source_gcode": str(Path(gcode_path)),
        "num_commands": len(commands),
        "servo_ids": params.servo_ids,
        "time_ms": time_ms,
        "angle_units": "deg",
        "position_units": "lx225_0_to_1000",
        "commands": [asdict(command) for command in commands],
    }
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return output_path


def run_servo_commands(
    commands: list[ServoCommand],
    *,
    port: str,
    baudrate: int = 9600,
    timeout: float = 1.0,
    connect_delay: float = 0.2,
    settle_time: float = 0.0,
    params: RobotParams | None = None,
) -> None:
    params = params or robot_params()
    driver = BusServoDriver(port=port, baudrate=baudrate, timeout=timeout, connect_delay=connect_delay)
    try:
        driver.connect()
        for command in commands:
            targets = list(zip(params.servo_ids, command.servo_positions, strict=True))
            driver.set_servo_positions([(servo_id, position) for servo_id, position in targets], command.time_ms)
            # Keep the send cadence aligned with the commanded move duration so
            # later packets do not immediately override the current motion.
            wait_time = (command.time_ms / 1000.0) + settle_time
            if wait_time > 0:
                time.sleep(wait_time)
    finally:
        driver.close()


def run_gcode_file(
    gcode_path: str | Path,
    *,
    port: str,
    baudrate: int = 9600,
    timeout: float = 1.0,
    connect_delay: float = 0.2,
    time_ms: int = 100,
    settle_time: float = 0.0,
    params: RobotParams | None = None,
) -> list[ServoCommand]:
    params = params or robot_params()
    commands = build_servo_commands_from_gcode(gcode_path, params=params, time_ms=time_ms)
    run_servo_commands(
        commands,
        port=port,
        baudrate=baudrate,
        timeout=timeout,
        connect_delay=connect_delay,
        settle_time=settle_time,
        params=params,
    )
    return commands
