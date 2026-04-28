from __future__ import annotations

import ctypes
import time
from typing import Callable

import serial

from .protocol import (
    ANGLE_LIMIT_READ,
    ANGLE_LIMIT_WRITE,
    ANGLE_OFFSET_ADJUST,
    ANGLE_OFFSET_READ,
    ANGLE_OFFSET_WRITE,
    BROADCAST_ID,
    ID_READ,
    ID_WRITE,
    POS_READ,
    ResponsePacket,
    build_read_packet,
    build_write_packet,
    checksum,
)


def _to_signed_int16(low: int, high: int) -> int:
    value = 0xFFFF & (low | (high << 8))
    return ctypes.c_int16(value).value


def _build_simple_read_packet(servo_ids: list[int]) -> bytes:
    unique_ids = list(dict.fromkeys(int(servo_id) for servo_id in servo_ids))
    payload = bytes([len(unique_ids), *unique_ids])
    return bytes([0x55, 0x55, len(payload) + 2, 0x15]) + payload


def _parse_simple_positions_payload(payload: bytes) -> dict[int, int]:
    if not payload:
        return {}
    count = payload[0]
    if count == 0:
        return {}
    expected_len = 1 + count * 3
    if len(payload) != expected_len:
        raise RuntimeError(f"Unexpected simple position payload length: {len(payload)} != {expected_len}")

    found: dict[int, int] = {}
    offset = 1
    for _ in range(count):
        returned_id = payload[offset]
        position = payload[offset + 1] | (payload[offset + 2] << 8)
        found[int(returned_id)] = int(position)
        offset += 3
    return found


class LX225Driver:
    def __init__(
        self,
        *,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.2,
        connect_delay: float = 0.1,
    ) -> None:
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

    def __enter__(self) -> "LX225Driver":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _require_open(self) -> serial.Serial:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")
        return self.ser

    def _send(self, packet: bytes) -> None:
        ser = self._require_open()
        ser.write(packet)
        ser.flush()

    def _clear_input(self) -> None:
        ser = self._require_open()
        ser.reset_input_buffer()

    def _read_response(self, *, expected_command: int, deadline_seconds: float | None = None) -> ResponsePacket:
        ser = self._require_open()
        timeout = self.timeout if deadline_seconds is None else deadline_seconds
        deadline = time.perf_counter() + timeout

        while time.perf_counter() < deadline:
            first = ser.read(1)
            if first != b"\x55":
                continue
            second = ser.read(1)
            if second != b"\x55":
                continue

            head = ser.read(3)
            if len(head) != 3:
                continue

            servo_id = head[0]
            length = head[1]
            command = head[2]
            data_len = max(0, length - 3)
            remaining = ser.read(data_len + 1)
            if len(remaining) != data_len + 1:
                continue

            data = remaining[:-1]
            packet_checksum = remaining[-1]
            packet_bytes = bytearray(b"\x55\x55")
            packet_bytes.extend([servo_id, length, command])
            packet_bytes.extend(data)
            expected_checksum = checksum(packet_bytes)
            if packet_checksum != expected_checksum:
                continue
            if command != expected_command:
                continue

            return ResponsePacket(
                servo_id=servo_id,
                length=length,
                command=command,
                data=data,
                checksum=packet_checksum,
            )

        raise TimeoutError(f"Timed out waiting for response to command {expected_command}")

    def _read_value(
        self,
        servo_id: int,
        command: int,
        *,
        parser: Callable[[bytes], object],
        timeout: float | None = None,
    ):
        self._clear_input()
        self._send(build_read_packet(servo_id, command))
        response = self._read_response(expected_command=command, deadline_seconds=timeout)
        return parser(response.data)

    def _read_simple_packet(self, *, expected_command: int = 0x15, timeout: float | None = None) -> tuple[int, bytes]:
        ser = self._require_open()
        deadline = time.perf_counter() + (self.timeout if timeout is None else timeout)

        while time.perf_counter() < deadline:
            first = ser.read(1)
            if first != b"\x55":
                continue
            second = ser.read(1)
            if second != b"\x55":
                continue

            length_raw = ser.read(1)
            command_raw = ser.read(1)
            if len(length_raw) != 1 or len(command_raw) != 1:
                continue

            length = length_raw[0]
            command = command_raw[0]
            payload_len = max(0, length - 2)
            payload = ser.read(payload_len)
            if len(payload) != payload_len:
                continue
            if command != expected_command:
                continue
            return command, payload

        raise TimeoutError(f"Timed out waiting for simple response to command {expected_command}")

    def discover_single_id(self, *, timeout: float | None = None) -> int:
        return int(
            self._read_value(
                BROADCAST_ID,
                ID_READ,
                parser=lambda data: int(data[0]),
                timeout=timeout,
            )
        )

    def read_id(self, servo_id: int, *, timeout: float | None = None) -> int:
        return int(
            self._read_value(
                servo_id,
                ID_READ,
                parser=lambda data: int(data[0]),
                timeout=timeout,
            )
        )

    def write_id(self, old_id: int, new_id: int) -> None:
        self._send(build_write_packet(old_id, ID_WRITE, new_id))

    def read_position(self, servo_id: int, *, timeout: float | None = None) -> int:
        return int(
            self._read_value(
                servo_id,
                POS_READ,
                parser=lambda data: _to_signed_int16(data[0], data[1]),
                timeout=timeout,
            )
        )

    def read_position_simple(self, servo_id: int, *, timeout: float | None = None) -> int | None:
        found = self.read_positions_simple([servo_id], timeout=timeout)
        return found.get(int(servo_id))

    def read_positions_simple(self, servo_ids: list[int], *, timeout: float | None = None) -> dict[int, int]:
        self._clear_input()
        self._send(_build_simple_read_packet(servo_ids))
        _, payload = self._read_simple_packet(timeout=timeout)
        return _parse_simple_positions_payload(payload)

    def scan_simple_positions(self, *, servo_id_min: int = 1, servo_id_max: int = 16, timeout: float | None = None) -> dict[int, int]:
        found: dict[int, int] = {}
        for servo_id in range(int(servo_id_min), int(servo_id_max) + 1):
            try:
                position = self.read_position_simple(servo_id, timeout=timeout)
            except TimeoutError:
                continue
            if position is not None:
                found[servo_id] = position
        return found

    def read_offset(self, servo_id: int, *, timeout: float | None = None) -> int:
        return int(
            self._read_value(
                servo_id,
                ANGLE_OFFSET_READ,
                parser=lambda data: int(ctypes.c_int8(data[0]).value),
                timeout=timeout,
            )
        )

    def adjust_offset(self, servo_id: int, offset: int) -> None:
        self._send(build_write_packet(servo_id, ANGLE_OFFSET_ADJUST, offset & 0xFF))

    def save_offset(self, servo_id: int) -> None:
        self._send(build_write_packet(servo_id, ANGLE_OFFSET_WRITE))

    def read_angle_limit(self, servo_id: int, *, timeout: float | None = None) -> tuple[int, int]:
        return tuple(
            self._read_value(
                servo_id,
                ANGLE_LIMIT_READ,
                parser=lambda data: (
                    _to_signed_int16(data[0], data[1]),
                    _to_signed_int16(data[2], data[3]),
                ),
                timeout=timeout,
            )
        )

    def write_angle_limit(self, servo_id: int, raw_min: int, raw_max: int) -> None:
        self._send(build_write_packet(servo_id, ANGLE_LIMIT_WRITE, raw_min, raw_max))
