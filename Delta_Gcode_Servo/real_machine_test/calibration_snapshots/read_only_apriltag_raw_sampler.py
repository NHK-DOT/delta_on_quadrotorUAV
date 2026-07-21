from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import paramiko
import serial


THIS_DIR = Path(__file__).resolve().parent
REAL_MACHINE_TEST_DIR = THIS_DIR.parent
PROJECT_ROOT = REAL_MACHINE_TEST_DIR.parents[1]

if str(REAL_MACHINE_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(REAL_MACHINE_TEST_DIR))

from vision_tool_state import VisionToolPreviewConfig, build_vision_tool_preview


CMD_MOVE = 0x03
CMD_READ = 0x15
ARM_SERVO_IDS = [1, 2, 3]
DEFAULT_JETSON_JSON = "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json"


def now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def to_signed_int16(low: int, high: int) -> int:
    value = (int(low) | (int(high) << 8)) & 0xFFFF
    return value - 0x10000 if value >= 0x8000 else value


def make_read_packet(ids: list[int]) -> bytes:
    # Packet payload contains the count byte and servo id 3, so byte value 0x03
    # can appear safely outside the command byte. Only packet[3] is the command.
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
    if not payload:
        raise RuntimeError("empty 0x15 payload")
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


def read_servo_raw(ser: serial.Serial, tx: bytes) -> tuple[dict[int, int] | None, str]:
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(tx)
        ser.flush()
        return parse_positions(read_packet(ser, timeout=2.0)), ""
    except Exception as exc:
        return None, repr(exc)


def connect_jetson(args: argparse.Namespace) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=args.jetson_host,
        username=args.jetson_user,
        password=args.jetson_password,
        timeout=8,
        look_for_keys=False,
        allow_agent=False,
    )
    return client


def fetch_remote_json(sftp: paramiko.SFTPClient, remote_path: str) -> tuple[dict[str, Any] | None, str]:
    try:
        with sftp.open(remote_path, "r") as fh:
            raw = fh.read()
        if isinstance(raw, bytes):
            text = raw.decode("utf-8")
        else:
            text = raw
        payload = json.loads(text)
        if not isinstance(payload, dict):
            raise ValueError("remote JSON is not an object")
        return payload, ""
    except Exception as exc:
        return None, repr(exc)


def snapshot_age_ms(payload: dict[str, Any] | None) -> float | None:
    if not payload:
        return None
    timestamp = payload.get("timestamp_unix")
    if not isinstance(timestamp, (int, float)):
        return None
    return max(0.0, (time.time() - float(timestamp)) * 1000.0)


def select_detection(payload: dict[str, Any] | None, tag_id: int | None) -> dict[str, Any] | None:
    if not payload:
        return None
    detections = payload.get("detections")
    if not isinstance(detections, list):
        return None
    for item in detections:
        if not isinstance(item, dict):
            continue
        if tag_id is None or item.get("id") == tag_id:
            return item
    return None


def build_preview_or_error(
    snapshot_payload: dict[str, Any] | None,
    snapshot_path: Path,
    args: argparse.Namespace,
) -> tuple[dict[str, Any] | None, str]:
    if snapshot_payload is None:
        return None, "no AprilTag snapshot payload"
    try:
        snapshot_path.write_text(json.dumps(snapshot_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        config = VisionToolPreviewConfig(
            calibration_path=args.calibration,
            apriltag_snapshot_path=snapshot_path,
            imu_snapshot_path=args.imu_snapshot,
            output_path=args.output_dir / "vision_tool_preview_latest.json",
            hand_tag_id=args.hand_tag_id,
            tool_hand_tag_path=args.tool_hand_tag,
            min_snapshot_fresh_ms=args.fresh_ms,
        )
        return build_vision_tool_preview(config), ""
    except Exception as exc:
        return None, repr(exc)


def flatten_detection(detection: dict[str, Any] | None) -> dict[str, Any]:
    if detection is None:
        return {
            "tag_seen": False,
            "tag_id": "",
            "tag_hamming": "",
            "tag_center_x_px": "",
            "tag_center_y_px": "",
            "tag_x_m": "",
            "tag_y_m": "",
            "tag_z_m": "",
        }
    center = detection.get("center_px") if isinstance(detection.get("center_px"), dict) else {}
    position = detection.get("position_m") if isinstance(detection.get("position_m"), dict) else {}
    return {
        "tag_seen": True,
        "tag_id": detection.get("id", ""),
        "tag_hamming": detection.get("hamming", ""),
        "tag_center_x_px": center.get("x", ""),
        "tag_center_y_px": center.get("y", ""),
        "tag_x_m": position.get("x", ""),
        "tag_y_m": position.get("y", ""),
        "tag_z_m": position.get("z", ""),
    }


def flatten_preview(preview: dict[str, Any] | None) -> dict[str, Any]:
    if preview is None:
        return {
            "tool_x_mm": "",
            "tool_y_mm": "",
            "tool_z_mm": "",
            "ik_reachable": "",
            "preview_raw1": "",
            "preview_raw2": "",
            "preview_raw3": "",
        }
    xyz = preview.get("tool_position_mm")
    if not isinstance(xyz, list) or len(xyz) != 3:
        xyz = ["", "", ""]
    ik = preview.get("delta_ik") if isinstance(preview.get("delta_ik"), dict) else {}
    raw_preview = preview.get("servo_raw_preview") if isinstance(preview.get("servo_raw_preview"), dict) else {}
    target_raw = raw_preview.get("target_raw") if isinstance(raw_preview.get("target_raw"), dict) else {}
    return {
        "tool_x_mm": xyz[0],
        "tool_y_mm": xyz[1],
        "tool_z_mm": xyz[2],
        "ik_reachable": ik.get("reachable", ""),
        "preview_raw1": target_raw.get("1", ""),
        "preview_raw2": target_raw.get("2", ""),
        "preview_raw3": target_raw.get("3", ""),
    }


def write_session(args: argparse.Namespace, run_id: str, tx: bytes) -> None:
    payload = {
        "created_iso": now_iso(),
        "run_id": run_id,
        "mode": "read_only_windows_com19_plus_jetson_apriltag",
        "safety": {
            "servo_command": "0x15 read position only",
            "motion_command_0x03": "blocked by make_read_packet",
            "arm_servos": [1, 2, 3],
            "landing_gear_servos": [4, 5, 6],
        },
        "serial": {
            "port": args.port,
            "baudrate": args.baudrate,
            "tx_hex": tx.hex(" ").upper(),
            "command_byte": f"0x{tx[3]:02X}",
        },
        "jetson": {
            "host": args.jetson_host,
            "user": args.jetson_user,
            "remote_apriltag_json": args.remote_json,
        },
        "vision": {
            "hand_tag_id": args.hand_tag_id,
            "calibration": str(args.calibration),
            "imu_snapshot": str(args.imu_snapshot),
            "fresh_ms": args.fresh_ms,
        },
    }
    (args.output_dir / "session.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read-only COM19 servo raw + Jetson AprilTag sampler.")
    parser.add_argument("--port", default="COM19")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--jetson-host", default="192.168.1.80")
    parser.add_argument("--jetson-user", default="nvidia")
    parser.add_argument("--jetson-password", default=os.environ.get("JETSON_PASSWORD", "nvidia"))
    parser.add_argument("--remote-json", default=DEFAULT_JETSON_JSON)
    parser.add_argument("--hand-tag-id", type=int, default=3)
    parser.add_argument("--samples", type=int, default=60, help="0 means run until Ctrl+C")
    parser.add_argument("--interval-sec", type=float, default=2.0)
    parser.add_argument("--settle-sec", type=float, default=3.0)
    parser.add_argument("--label-prefix", default="manual")
    parser.add_argument("--fresh-ms", type=float, default=750.0)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=PROJECT_ROOT / "Dual_Camera_HandEye" / "output" / "calibration_result.json",
    )
    parser.add_argument(
        "--imu-snapshot",
        type=Path,
        default=PROJECT_ROOT / "IMU" / "wt61c_latest.json",
    )
    parser.add_argument("--tool-hand-tag", type=Path, default=None)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=THIS_DIR / datetime.now().strftime("%Y%m%d_%H%M%S_readonly_apriltag_raw"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_id = args.output_dir.name
    csv_path = args.output_dir / "samples.csv"
    jsonl_path = args.output_dir / "samples.jsonl"
    latest_snapshot_path = args.output_dir / "latest_jetson_apriltag_snapshot.json"
    tx = make_read_packet(ARM_SERVO_IDS)
    write_session(args, run_id, tx)

    fieldnames = [
        "run_id",
        "index",
        "label",
        "timestamp_local",
        "serial_success",
        "raw1",
        "raw2",
        "raw3",
        "serial_error",
        "tx_hex",
        "command_byte",
        "apriltag_success",
        "apriltag_age_ms",
        "apriltag_error",
        "tag_seen",
        "tag_id",
        "tag_hamming",
        "tag_center_x_px",
        "tag_center_y_px",
        "tag_x_m",
        "tag_y_m",
        "tag_z_m",
        "preview_success",
        "preview_error",
        "tool_x_mm",
        "tool_y_mm",
        "tool_z_mm",
        "ik_reachable",
        "preview_raw1",
        "preview_raw2",
        "preview_raw3",
    ]

    print(f"output_dir={args.output_dir}")
    print(f"tx={tx.hex(' ').upper()} cmd_byte=0x{tx[3]:02X}")
    print("motion_command=disabled; only 0x15 read position is sent")
    print(f"settle_sec={args.settle_sec} samples={args.samples} interval_sec={args.interval_sec}")
    time.sleep(max(0.0, args.settle_sec))

    jetson = connect_jetson(args)
    sftp = jetson.open_sftp()
    try:
        with csv_path.open("w", encoding="utf-8", newline="") as csv_file, jsonl_path.open(
            "w", encoding="utf-8"
        ) as jsonl_file, serial.Serial(
            args.port,
            args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.08,
            write_timeout=1.0,
        ) as ser:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            time.sleep(0.5)
            index = 0
            while args.samples <= 0 or index < args.samples:
                index += 1
                loop_start = time.perf_counter()
                timestamp = now_iso()
                positions, serial_error = read_servo_raw(ser, tx)
                snapshot_payload, apriltag_error = fetch_remote_json(sftp, args.remote_json)
                detection = select_detection(snapshot_payload, args.hand_tag_id)
                preview, preview_error = build_preview_or_error(snapshot_payload, latest_snapshot_path, args)

                row: dict[str, Any] = {
                    "run_id": run_id,
                    "index": index,
                    "label": f"{args.label_prefix}_{index:04d}",
                    "timestamp_local": timestamp,
                    "serial_success": positions is not None,
                    "raw1": positions.get(1, "") if positions else "",
                    "raw2": positions.get(2, "") if positions else "",
                    "raw3": positions.get(3, "") if positions else "",
                    "serial_error": serial_error,
                    "tx_hex": tx.hex(" ").upper(),
                    "command_byte": f"0x{tx[3]:02X}",
                    "apriltag_success": snapshot_payload is not None,
                    "apriltag_age_ms": snapshot_age_ms(snapshot_payload),
                    "apriltag_error": apriltag_error,
                    "preview_success": preview is not None,
                    "preview_error": preview_error,
                }
                row.update(flatten_detection(detection))
                row.update(flatten_preview(preview))

                writer.writerow(row)
                csv_file.flush()
                sample = {
                    **row,
                    "mode": "read_only_apriltag_raw_sample",
                    "remote_snapshot": snapshot_payload,
                    "selected_detection": detection,
                    "vision_tool_preview": preview,
                }
                jsonl_file.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")
                jsonl_file.flush()

                print(
                    f"#{index:04d} raw=({row['raw1']},{row['raw2']},{row['raw3']}) "
                    f"tag={row['tag_id'] if row['tag_seen'] else '-'} "
                    f"age={row['apriltag_age_ms'] if row['apriltag_age_ms'] is not None else '-'}ms "
                    f"tool=({row['tool_x_mm']},{row['tool_y_mm']},{row['tool_z_mm']}) "
                    f"errors={serial_error or apriltag_error or preview_error or '-'}"
                )

                sleep_for = args.interval_sec - (time.perf_counter() - loop_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("stopped by Ctrl+C")
    finally:
        sftp.close()
        jetson.close()

    print(f"csv={csv_path}")
    print(f"jsonl={jsonl_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
