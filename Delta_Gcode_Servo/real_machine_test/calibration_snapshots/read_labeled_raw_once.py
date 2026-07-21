from __future__ import annotations

import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import serial


CMD_MOVE = 0x03
CMD_READ = 0x15
SERVO_IDS = [1, 2, 3]


def to_signed_int16(low: int, high: int) -> int:
    value = (int(low) | (int(high) << 8)) & 0xFFFF
    return value - 0x10000 if value >= 0x8000 else value


def make_read_packet(ids: list[int]) -> bytes:
    params = [len(ids), *ids]
    packet = bytes([0x55, 0x55, 2 + len(params), CMD_READ, *params])
    if packet[3] == CMD_MOVE:
        raise RuntimeError("blocked: command byte is 0x03 motion")
    if packet[3] != CMD_READ:
        raise RuntimeError(f"blocked: command byte is 0x{packet[3]:02X}, expected 0x15 read")
    return packet


def read_packet(ser: serial.Serial, timeout: float = 2.0) -> bytes:
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if ser.read(1) != b"\x55":
            continue
        if ser.read(1) != b"\x55":
            continue
        length_raw = ser.read(1)
        cmd_raw = ser.read(1)
        if len(length_raw) != 1 or len(cmd_raw) != 1:
            continue
        length = length_raw[0]
        cmd = cmd_raw[0]
        payload_len = max(0, length - 2)
        payload = ser.read(payload_len)
        if len(payload) != payload_len or cmd != CMD_READ:
            continue
        return payload
    raise TimeoutError("timeout waiting for 0x15 response")


def parse_positions(payload: bytes) -> dict[int, int]:
    count = payload[0]
    expected = 1 + count * 3
    if len(payload) != expected:
        raise RuntimeError(f"bad payload length {len(payload)} != {expected}: {payload.hex(' ').upper()}")
    out: dict[int, int] = {}
    offset = 1
    for _ in range(count):
        sid = payload[offset]
        out[sid] = to_signed_int16(payload[offset + 1], payload[offset + 2])
        offset += 3
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Append one labeled read-only raw sample group.")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--port", default="COM19")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--settle-sec", type=float, default=12.0)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval-sec", type=float, default=1.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent,
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / f"{args.run_id}_labeled_workspace_raw.csv"
    jsonl_path = args.output_dir / f"{args.run_id}_labeled_workspace_raw.jsonl"
    tx = make_read_packet(SERVO_IDS)
    fieldnames = [
        "run_id",
        "label",
        "sample_in_label",
        "timestamp_local",
        "success",
        "raw1",
        "raw2",
        "raw3",
        "error",
        "tx_hex",
        "command_byte",
    ]
    write_header = not csv_path.exists()

    print(f"label={args.label} settle_sec={args.settle_sec} samples={args.samples}")
    print(f"tx={tx.hex(' ').upper()} cmd_byte=0x{tx[3]:02X}")
    time.sleep(args.settle_sec)

    with csv_path.open("a", encoding="utf-8", newline="") as csv_file, jsonl_path.open(
        "a", encoding="utf-8"
    ) as jsonl_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        with serial.Serial(
            args.port,
            args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.08,
            write_timeout=1.0,
        ) as ser:
            time.sleep(0.5)
            for index in range(1, args.samples + 1):
                row = {
                    "run_id": args.run_id,
                    "label": args.label,
                    "sample_in_label": index,
                    "timestamp_local": datetime.now().isoformat(timespec="seconds"),
                    "success": False,
                    "raw1": "",
                    "raw2": "",
                    "raw3": "",
                    "error": "",
                    "tx_hex": tx.hex(" ").upper(),
                    "command_byte": f"0x{tx[3]:02X}",
                }
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    ser.write(tx)
                    ser.flush()
                    positions = parse_positions(read_packet(ser, timeout=2.0))
                    row["success"] = True
                    row["raw1"] = positions.get(1, "")
                    row["raw2"] = positions.get(2, "")
                    row["raw3"] = positions.get(3, "")
                    print(
                        f"{args.label} sample={index} "
                        f"raw1={row['raw1']} raw2={row['raw2']} raw3={row['raw3']}"
                    )
                except Exception as exc:
                    row["error"] = repr(exc)
                    print(f"{args.label} sample={index} ERROR {exc!r}")
                writer.writerow(row)
                csv_file.flush()
                jsonl_file.write(json.dumps(row, ensure_ascii=False) + "\n")
                jsonl_file.flush()
                if index < args.samples:
                    time.sleep(args.interval_sec)

    print(f"csv={csv_path}")
    print(f"jsonl={jsonl_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
