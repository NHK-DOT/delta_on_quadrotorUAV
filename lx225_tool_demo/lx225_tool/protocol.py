from __future__ import annotations

from dataclasses import dataclass


FRAME_HEADER = 0x55
HEADER_BYTES = b"\x55\x55"
BROADCAST_ID = 0xFE

MOVE_TIME_WRITE = 1
ID_WRITE = 13
ID_READ = 14
ANGLE_OFFSET_ADJUST = 17
ANGLE_OFFSET_WRITE = 18
ANGLE_OFFSET_READ = 19
ANGLE_LIMIT_WRITE = 20
ANGLE_LIMIT_READ = 21
VIN_LIMIT_WRITE = 22
VIN_LIMIT_READ = 23
TEMP_MAX_LIMIT_WRITE = 24
TEMP_MAX_LIMIT_READ = 25
TEMP_READ = 26
VIN_READ = 27
POS_READ = 28
OR_MOTOR_MODE_WRITE = 29
OR_MOTOR_MODE_READ = 30
LOAD_OR_UNLOAD_WRITE = 31
LOAD_OR_UNLOAD_READ = 32


def checksum(packet: bytes | bytearray) -> int:
    total = sum(packet) - FRAME_HEADER - FRAME_HEADER
    return (~total) & 0xFF


def build_read_packet(servo_id: int, command: int) -> bytes:
    packet = bytearray(HEADER_BYTES)
    packet.append(servo_id & 0xFF)
    packet.append(3)
    packet.append(command & 0xFF)
    packet.append(checksum(packet))
    return bytes(packet)


def build_write_packet(
    servo_id: int,
    command: int,
    data1: int | None = None,
    data2: int | None = None,
) -> bytes:
    packet = bytearray(HEADER_BYTES)
    packet.append(servo_id & 0xFF)
    if data1 is None and data2 is None:
        packet.append(3)
    elif data1 is not None and data2 is None:
        packet.append(4)
    elif data1 is not None and data2 is not None:
        packet.append(7)
    else:
        raise ValueError("data2 requires data1")

    packet.append(command & 0xFF)

    if data1 is not None and data2 is None:
        packet.append(data1 & 0xFF)
    elif data1 is not None and data2 is not None:
        packet.extend([(data1 & 0xFF), ((data1 >> 8) & 0xFF)])
        packet.extend([(data2 & 0xFF), ((data2 >> 8) & 0xFF)])

    packet.append(checksum(packet))
    return bytes(packet)


@dataclass(frozen=True)
class ResponsePacket:
    servo_id: int
    length: int
    command: int
    data: bytes
    checksum: int

