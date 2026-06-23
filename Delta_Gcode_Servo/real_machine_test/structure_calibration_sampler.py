#!/usr/bin/env python3
"""Read-only sampler for fitting the rebuilt Delta arm geometry.

The script records servo raw feedback, the base-camera AprilTag tool pose,
IMU summary, and manually entered geometry dimensions. It never sends servo
motion commands.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DELTA_SERVO_ROOT = PROJECT_ROOT / "Delta_Gcode_Servo"
IMU_SCRIPT = PROJECT_ROOT / "IMU" / "wt61c_live_viewer.py"
APRILTAG_ROOT = PROJECT_ROOT / "AprilTag_Vision" / "myAprilTag"
APRILTAG_SCRIPT = APRILTAG_ROOT / "src" / "apriltag_usb_detector.py"
for import_path in (DELTA_SERVO_ROOT,):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from delta_gcode_servo.servo import BusServoDriver
from delta_gcode_servo.servo_mapping import load_servo_mappings_for_ids
from vision_tool_state import VisionToolPreviewConfig, build_vision_tool_preview, read_json, write_json


@dataclass(frozen=True)
class StructureGeometry:
    upper_arm_mm: float
    lower_arm_mm: float
    platform_radius_mm: float
    servo_axis_radius_mm: float | None = None
    servo_axis_z_offset_mm: float | None = None
    note: str = ""


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen
    log_file: TextIO


def now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def prompt_float(prompt: str, *, optional: bool = False) -> float | None:
    while True:
        text = input(prompt).strip()
        if optional and text == "":
            return None
        try:
            return float(text)
        except ValueError:
            print("请输入数字，或者留空跳过可选项。")


def prompt_geometry() -> StructureGeometry:
    print("\n输入新结构几何尺寸，单位 mm。")
    print("这些值只写入数据集，不会触发运动。")
    print("坐标约定: 相机向下照；相机测量原点为 0；X+ 向右，Y+ 向前，Z+ 向上。")
    upper_arm = prompt_float("大臂/主动臂长度 upper_arm_mm: ")
    lower_arm = prompt_float("小臂/从动臂长度 lower_arm_mm: ")
    platform_radius = prompt_float("执行机构三小臂末端三角形外接圆半径 platform_radius_mm: ")
    servo_axis_radius = prompt_float("舵机轴到中心的水平半径 servo_axis_radius_mm，可留空: ", optional=True)
    servo_axis_z_offset = prompt_float("舵机转轴中心相对相机 0 平面的 Z 高度 servo_axis_z_offset_mm，可留空: ", optional=True)
    note = input("几何备注，可留空: ").strip()
    assert upper_arm is not None and lower_arm is not None and platform_radius is not None
    return StructureGeometry(
        upper_arm_mm=upper_arm,
        lower_arm_mm=lower_arm,
        platform_radius_mm=platform_radius,
        servo_axis_radius_mm=servo_axis_radius,
        servo_axis_z_offset_mm=servo_axis_z_offset,
        note=note,
    )


def read_servo_raw(port: str, servo_ids: list[int], timeout: float) -> dict[int, int]:
    driver = BusServoDriver(port=port, baudrate=9600, timeout=timeout, connect_delay=0.2)
    try:
        driver.connect()
        return driver.read_servo_positions(servo_ids, timeout=timeout)
    finally:
        driver.close()


def load_imu_snapshot(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return read_json(path)
    except Exception:
        return None


def snapshot_age_ms(path: Path) -> float | None:
    if not path.exists():
        return None
    try:
        payload = read_json(path)
    except Exception:
        return None
    timestamp = payload.get("timestamp_unix")
    if not isinstance(timestamp, (int, float)):
        return None
    return max(0.0, (time.time() - float(timestamp)) * 1000.0)


def start_managed_process(
    *,
    name: str,
    command: list[str],
    cwd: Path,
    log_dir: Path,
) -> ManagedProcess:
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = (log_dir / f"{name}.log").open("a", encoding="utf-8", buffering=1)
    log_file.write(f"\n# {now_iso()} start: {' '.join(command)}\n")
    process = subprocess.Popen(
        command,
        cwd=str(cwd),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        text=True,
    )
    return ManagedProcess(name=name, process=process, log_file=log_file)


def stop_managed_processes(processes: list[ManagedProcess]) -> None:
    for item in reversed(processes):
        proc = item.process
        if proc.poll() is None:
            print(f"Stopping {item.name}...")
            proc.terminate()
    deadline = time.monotonic() + 5.0
    for item in reversed(processes):
        proc = item.process
        while proc.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if proc.poll() is None:
            print(f"Killing {item.name}...")
            proc.kill()
        try:
            item.log_file.write(f"# {now_iso()} stopped rc={proc.poll()}\n")
            item.log_file.close()
        except Exception:
            pass


def wait_for_fresh_snapshot(
    *,
    path: Path,
    name: str,
    max_age_ms: float,
    timeout_sec: float,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        age = snapshot_age_ms(path)
        if age is not None and age <= max_age_ms:
            print(f"{name} snapshot ready: age={age:.0f} ms")
            return True
        time.sleep(0.25)
    age = snapshot_age_ms(path)
    age_text = "missing" if age is None else f"{age:.0f} ms"
    print(f"warning: {name} snapshot not fresh after {timeout_sec:.1f}s, age={age_text}")
    return False


def start_sensor_processes(args: argparse.Namespace, log_dir: Path) -> list[ManagedProcess]:
    processes: list[ManagedProcess] = []
    imu_cmd = [
        sys.executable,
        str(IMU_SCRIPT),
        "--port",
        args.imu_port,
        "--baud",
        str(args.imu_baud),
        "--snapshot-file",
        str(args.imu_snapshot),
        "--snapshot-interval",
        str(args.imu_snapshot_interval),
        "--print-every",
        str(args.sensor_print_every),
        "--no-gui",
        "--duration",
        "0",
    ]
    apriltag_cmd = [
        sys.executable,
        str(APRILTAG_SCRIPT),
        "--config",
        str(args.apriltag_config),
        "--snapshot-file",
        str(args.base_camera_snapshot),
        "--snapshot-hz",
        str(args.apriltag_snapshot_hz),
        "--status-hz",
        str(args.sensor_status_hz),
        "--no-gui",
        "--duration",
        "0",
    ]
    if args.apriltag_camera_index is not None:
        apriltag_cmd.extend(["--camera-index", str(args.apriltag_camera_index)])
    if args.apriltag_tag_size_m is not None:
        apriltag_cmd.extend(["--tag-size-m", str(args.apriltag_tag_size_m)])

    print("Starting IMU reader...")
    processes.append(
        start_managed_process(
            name="imu",
            command=imu_cmd,
            cwd=IMU_SCRIPT.parent,
            log_dir=log_dir,
        )
    )
    print("Starting AprilTag detector...")
    processes.append(
        start_managed_process(
            name="apriltag",
            command=apriltag_cmd,
            cwd=APRILTAG_ROOT,
            log_dir=log_dir,
        )
    )
    return processes


def append_csv(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        fh.write("\n")


def sample_once(
    *,
    label: str,
    geometry: StructureGeometry,
    port: str,
    servo_ids: list[int],
    timeout: float,
    vision_config: VisionToolPreviewConfig,
    csv_path: Path,
    jsonl_path: Path,
) -> dict[str, Any]:
    raw = read_servo_raw(port, servo_ids, timeout)
    mappings = load_servo_mappings_for_ids(servo_ids)
    raw_physical_deg = {
        servo_id: mappings[servo_id].raw_to_physical_deg(
            raw[servo_id],
            physical_min_deg=0.0,
            physical_max_deg=240.0,
        )
        for servo_id in servo_ids
    }
    vision_payload = build_vision_tool_preview(vision_config)
    imu_payload = load_imu_snapshot(vision_config.imu_snapshot_path)
    xyz = vision_payload.get("tool_position_mm")
    if not isinstance(xyz, list) or len(xyz) != 3:
        xyz = [None, None, None]

    sample = {
        "label": label,
        "timestamp_iso": now_iso(),
        "timestamp_unix": time.time(),
        "port": port,
        "motion_command_state": "read_only_no_motion",
        "geometry": asdict(geometry),
        "servo_raw": {str(k): int(v) for k, v in raw.items()},
        "servo_mapped_physical_deg": {str(k): float(v) for k, v in raw_physical_deg.items()},
        "vision": vision_payload,
        "imu": imu_payload,
    }
    append_jsonl(jsonl_path, sample)

    row = {
        "label": label,
        "timestamp_iso": sample["timestamp_iso"],
        "raw1": raw.get(1, ""),
        "raw2": raw.get(2, ""),
        "raw3": raw.get(3, ""),
        "x_mm": xyz[0],
        "y_mm": xyz[1],
        "z_mm": xyz[2],
        "vision_detection_id": vision_payload.get("detection_id"),
        "vision_snapshot_age_ms": vision_payload.get("snapshot_age_ms"),
        "imu_roll_deg": ((imu_payload or {}).get("angles_deg") or {}).get("roll", ""),
        "imu_pitch_deg": ((imu_payload or {}).get("angles_deg") or {}).get("pitch", ""),
        "imu_yaw_deg": ((imu_payload or {}).get("angles_deg") or {}).get("yaw", ""),
        "upper_arm_mm": geometry.upper_arm_mm,
        "lower_arm_mm": geometry.lower_arm_mm,
        "platform_radius_mm": geometry.platform_radius_mm,
        "servo_axis_radius_mm": geometry.servo_axis_radius_mm if geometry.servo_axis_radius_mm is not None else "",
        "servo_axis_z_offset_mm": (
            geometry.servo_axis_z_offset_mm if geometry.servo_axis_z_offset_mm is not None else ""
        ),
    }
    append_csv(csv_path, row)
    write_json(vision_config.output_path, vision_payload)
    return sample


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    default_output_dir = Path(__file__).with_name("structure_calibration_samples")
    parser = argparse.ArgumentParser(description="Read-only Delta structure calibration sampler")
    parser.add_argument("--port", default="COM15", help="Servo bus port, default: COM15")
    parser.add_argument("--timeout", type=float, default=0.6)
    parser.add_argument("--output-dir", type=Path, default=default_output_dir)
    parser.add_argument("--calibration", type=Path, default=VisionToolPreviewConfig.calibration_path)
    parser.add_argument("--base-camera-snapshot", type=Path, default=VisionToolPreviewConfig.apriltag_snapshot_path)
    parser.add_argument("--imu-snapshot", type=Path, default=VisionToolPreviewConfig.imu_snapshot_path)
    parser.add_argument("--hand-tag-id", type=int, default=None)
    parser.add_argument("--tool-hand-tag", type=Path, default=None)
    parser.add_argument("--fresh-ms", type=float, default=3000.0)
    parser.add_argument("--no-autostart-sensors", action="store_true", help="Do not launch IMU/AprilTag processes")
    parser.add_argument("--imu-port", default="COM16", help="WT61C IMU port, default: COM16")
    parser.add_argument("--imu-baud", type=int, default=9600)
    parser.add_argument("--imu-snapshot-interval", type=float, default=0.1)
    parser.add_argument(
        "--apriltag-config",
        type=Path,
        default=APRILTAG_ROOT / "config" / "apriltag_detector.toml",
    )
    parser.add_argument("--apriltag-camera-index", type=int, default=None)
    parser.add_argument("--apriltag-tag-size-m", type=float, default=None)
    parser.add_argument("--apriltag-snapshot-hz", type=float, default=10.0)
    parser.add_argument("--sensor-startup-timeout", type=float, default=20.0)
    parser.add_argument("--sensor-print-every", type=float, default=2.0)
    parser.add_argument("--sensor-status-hz", type=float, default=1.0)
    parser.add_argument(
        "--label-sequence",
        default="",
        help="Comma-separated labels to sample in order; still read-only and waits for Enter before each label.",
    )
    parser.add_argument("--samples-per-label", type=int, default=1)
    parser.add_argument("--settle-sec", type=float, default=0.0, help="Delay after Enter before sampling each point")
    return parser.parse_args(argv)


def parse_label_sequence(text: str) -> list[str]:
    return [item.strip() for item in text.replace("\n", ",").split(",") if item.strip()]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    servo_ids = [1, 2, 3]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    log_dir = args.output_dir / "logs"
    managed_processes: list[ManagedProcess] = []

    try:
        if not args.no_autostart_sensors:
            managed_processes = start_sensor_processes(args, log_dir)
            wait_for_fresh_snapshot(
                path=args.imu_snapshot,
                name="IMU",
                max_age_ms=args.fresh_ms,
                timeout_sec=args.sensor_startup_timeout,
            )
            wait_for_fresh_snapshot(
                path=args.base_camera_snapshot,
                name="AprilTag",
                max_age_ms=args.fresh_ms,
                timeout_sec=args.sensor_startup_timeout,
            )

        geometry = prompt_geometry()
        csv_path = args.output_dir / "samples.csv"
        jsonl_path = args.output_dir / "samples.jsonl"
        geometry_path = args.output_dir / "geometry.json"
        write_json(geometry_path, asdict(geometry))

        vision_config = VisionToolPreviewConfig(
            calibration_path=args.calibration,
            apriltag_snapshot_path=args.base_camera_snapshot,
            imu_snapshot_path=args.imu_snapshot,
            output_path=Path(__file__).with_name("vision_tool_preview_latest.json"),
            hand_tag_id=args.hand_tag_id,
            tool_hand_tag_path=args.tool_hand_tag,
            min_snapshot_fresh_ms=args.fresh_ms,
        )

        print("\n采样模式: 只读舵机 raw + 视觉/IMU 快照，不会发送运动命令。")
        if managed_processes:
            print("IMU 和 AprilTag 子进程已由本程序启动；退出本程序时会一起关闭。")
            print(f"子进程日志目录: {log_dir}")
        print(f"CSV 输出: {csv_path}")
        print(f"JSONL 输出: {jsonl_path}")
        print("每次手动摆到一个点后输入标签，例如 top_home、bottom_safe、left_mid。")
        print("输入 q 退出。\n")

        label_sequence = parse_label_sequence(args.label_sequence)
        samples_per_label = max(1, int(args.samples_per_label))
        if label_sequence:
            print("Queued label mode: move the arm manually to each named point, then press Enter.")
            print("No motion commands will be sent by this script.")
            for label in label_sequence:
                text = input(f"{label}: press Enter to sample, or q to stop > ").strip()
                if text.lower() in {"q", "quit", "exit"}:
                    break
                if args.settle_sec > 0:
                    time.sleep(args.settle_sec)
                for index in range(samples_per_label):
                    sample_label = label if samples_per_label == 1 else f"{label}_{index + 1:02d}"
                    try:
                        sample = sample_once(
                            label=sample_label,
                            geometry=geometry,
                            port=args.port,
                            servo_ids=servo_ids,
                            timeout=args.timeout,
                            vision_config=vision_config,
                            csv_path=csv_path,
                            jsonl_path=jsonl_path,
                        )
                    except Exception as exc:
                        print(f"sample failed for {sample_label}: {exc}")
                        continue

                    raw = sample["servo_raw"]
                    xyz = sample["vision"].get("tool_position_mm")
                    warnings = sample["vision"].get("warnings") or []
                    print(f"sampled {sample_label}: raw={raw}, xyz_mm={xyz}")
                    for warning in warnings:
                        print(f"warning: {warning}")
            return 0

        while True:
            label = input("采样标签 > ").strip()
            if label.lower() in {"q", "quit", "exit"}:
                break
            if not label:
                print("标签不能为空。")
                continue
            try:
                sample = sample_once(
                    label=label,
                    geometry=geometry,
                    port=args.port,
                    servo_ids=servo_ids,
                    timeout=args.timeout,
                    vision_config=vision_config,
                    csv_path=csv_path,
                    jsonl_path=jsonl_path,
                )
            except Exception as exc:
                print(f"采样失败: {exc}")
                continue

            raw = sample["servo_raw"]
            xyz = sample["vision"].get("tool_position_mm")
            warnings = sample["vision"].get("warnings") or []
            print(f"已采样 {label}: raw={raw}, xyz_mm={xyz}")
            for warning in warnings:
                print(f"warning: {warning}")
        return 0
    except KeyboardInterrupt:
        print("\n收到中断，正在关闭子进程...")
        return 130
    finally:
        stop_managed_processes(managed_processes)


if __name__ == "__main__":
    raise SystemExit(main())
