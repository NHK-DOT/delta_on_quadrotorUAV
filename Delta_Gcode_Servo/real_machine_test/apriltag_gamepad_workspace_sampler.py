#!/usr/bin/env python3
"""Realtime 8BitDo gamepad control plus AprilTag workspace sampling.

Run this on the Jetson that owns the 3K fisheye AprilTag process and the servo
bus. The control path reuses RealTimeArmController safety checks, feedback
anchoring, IK/FK, raw servo mapping, startup HOME flow, and landing-gear handling.

Sampling is added on top: press the sample button to record current servo
feedback, FK pose, AprilTag base_T_tool pose, and the vision-minus-FK offset.
"""

import argparse
import csv
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, TextIO, Union

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[2]
BT_8BITDO_SRC = PROJECT_ROOT / "bt_8bitdo_min" / "src"
DELTA_SERVO_ROOT = PROJECT_ROOT / "Delta_Gcode_Servo"
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))
if str(DELTA_SERVO_ROOT) not in sys.path:
    sys.path.insert(0, str(DELTA_SERVO_ROOT))

from gamepad_controller import RealTimeArmController
from vision_tool_state import VisionToolPreviewConfig, build_vision_tool_preview, write_json


DEFAULT_JETSON_ROOT = Path("/home/nvidia/Desktop/yolo_fisheye_calibration_jetson")
DEFAULT_JETSON_APRILTAG_JSON = DEFAULT_JETSON_ROOT / "output" / "apriltag_latest_jetson.json"
DEFAULT_JETSON_APRILTAG_SCRIPT = DEFAULT_JETSON_ROOT / "nv_gpu_apriltags_bench" / "run_fullfov_1280x960_gui.sh"
DEFAULT_8BITDO_CONFIG = PROJECT_ROOT / "bt_8bitdo_min" / "config" / "gamepad_8bitdo_bt.json"


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen
    log_file: TextIO


def now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        fh.write("\n")


def append_csv(path: Path, row: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def snapshot_age_ms(path: Path) -> Optional[float]:
    payload = read_json(path)
    if payload is None:
        return None
    timestamp = payload.get("timestamp_unix")
    if not isinstance(timestamp, (int, float)):
        return None
    return max(0.0, (time.time() - float(timestamp)) * 1000.0)


def start_process(name: str, command: List[str], cwd: Path, log_dir: Path, env: Optional[Dict[str, str]] = None) -> ManagedProcess:
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
        env=env,
    )
    return ManagedProcess(name=name, process=process, log_file=log_file)


def stop_processes(processes: List[ManagedProcess]) -> None:
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


def wait_for_fresh_snapshot(path: Path, *, max_age_ms: float, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        age = snapshot_age_ms(path)
        if age is not None and age <= max_age_ms:
            print(f"AprilTag snapshot ready: {path} age={age:.0f} ms")
            return True
        time.sleep(0.25)
    age = snapshot_age_ms(path)
    text = "missing" if age is None else f"{age:.0f} ms"
    print(f"warning: AprilTag snapshot is not fresh after {timeout_sec:.1f}s, age={text}")
    return False


def install_8bitdo_reader(controller: RealTimeArmController, *, config_path: Path, device_path: Optional[str]) -> None:
    if str(BT_8BITDO_SRC) not in sys.path:
        sys.path.insert(0, str(BT_8BITDO_SRC))
    from evdev_gamepad import BluetoothGamepadReader

    if getattr(controller, "gamepad", None) is not None:
        try:
            controller.gamepad.close()
        except Exception:
            pass
    controller.gamepad = BluetoothGamepadReader(
        config_path=str(config_path),
        device_path=device_path or None,
        announce=True,
    )
    if not controller.gamepad.is_available():
        raise RuntimeError(f"8BitDo gamepad unavailable: {controller.gamepad.last_error}")


def serializable_xyz(value: Optional[Union[np.ndarray, List[float]]]) -> Optional[List[float]]:
    if value is None:
        return None
    arr = np.asarray(value, dtype=float)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        return None
    return [float(v) for v in arr]


class AprilTagWorkspaceSamplerController(RealTimeArmController):
    def __init__(self, *args: Any, sampler_args: argparse.Namespace, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.sampler_args = sampler_args
        self.output_dir = sampler_args.output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.sample_csv_path = self.output_dir / "samples.csv"
        self.sample_jsonl_path = self.output_dir / "samples.jsonl"
        self.session_path = self.output_dir / "session.json"
        self.runtime_status_path = self.output_dir / "runtime_status.log"
        self.vision_tool_preview_path = self.output_dir / "vision_tool_preview_latest.json"
        self.record_file_path = self.sample_csv_path
        self.apriltag_snapshot_path = sampler_args.base_camera_snapshot
        self.handeye_calibration_path = sampler_args.calibration
        self.imu_snapshot_path = sampler_args.imu_snapshot
        self.vision_hand_tag_id = sampler_args.hand_tag_id
        self.vision_tool_preview_interval = sampler_args.vision_interval_sec
        self.max_sample_age_ms = sampler_args.fresh_ms
        self.speed_xy = float(sampler_args.speed_xy_mm_s)
        self.speed_z = float(sampler_args.speed_z_mm_s)
        self.max_servo_speed_ticks_per_sec = float(sampler_args.max_servo_speed_ticks_s)
        self.playback_speed_mm_per_sec = float(sampler_args.playback_speed_mm_s)
        self.playback_step_mm = float(sampler_args.playback_step_mm)
        self.playback_endpoint_tolerance_mm = float(sampler_args.playback_endpoint_tolerance_mm)
        self.sample_on_b = True
        self.operator_note = sampler_args.note
        self.write_session_file()

    def write_session_file(self) -> None:
        payload = {
            "created_iso": now_iso(),
            "created_unix": time.time(),
            "mode": "apriltag_gamepad_workspace_sampler",
            "units": "mm/deg/raw",
            "files": {
                "samples_csv": str(self.sample_csv_path),
                "samples_jsonl": str(self.sample_jsonl_path),
                "runtime_status": str(self.runtime_status_path),
                "vision_preview_latest": str(self.vision_tool_preview_path),
            },
            "inputs": {
                "base_camera_snapshot": str(self.apriltag_snapshot_path),
                "calibration": str(self.handeye_calibration_path),
                "imu_snapshot": str(self.imu_snapshot_path),
                "hand_tag_id": self.vision_hand_tag_id,
                "note": self.operator_note,
            },
            "control": {
                "servo_port": self.port,
                "baudrate": self.driver.baudrate,
                "speed_xy_mm_s": self.speed_xy,
                "speed_z_mm_s": self.speed_z,
                "max_servo_speed_ticks_per_sec": self.max_servo_speed_ticks_per_sec,
                "workspace_bounds_source": "delta_gcode_servo.config.RobotParams",
            },
        }
        write_json(self.session_path, payload)

    def update_vision_tool_preview(self, *, force: bool = False) -> Optional[Dict[str, Any]]:
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
            min_snapshot_fresh_ms=self.max_sample_age_ms,
        )
        try:
            payload = build_vision_tool_preview(config)
            write_json(self.vision_tool_preview_path, payload)
            self.vision_tool_preview = payload
            self.vision_tool_preview_error = None
            return payload
        except Exception as exc:
            self.vision_tool_preview_error = str(exc)
            self.debug_log(f"VISION_TOOL_PREVIEW_ERROR error={exc}")
            return None

    def record_current_point(self) -> None:
        self.record_apriltag_workspace_sample(label=f"sample_{self.record_count + 1:04d}")

    def play_last_sample_segment(self) -> bool:
        return super().play_last_sample_segment()

    def record_apriltag_workspace_sample(self, *, label: str) -> bool:
        payload = self.update_vision_tool_preview(force=True)
        if payload is None:
            print(f"Sample refused: vision preview unavailable ({self.vision_tool_preview_error})")
            return False

        vision_xyz = serializable_xyz(payload.get("tool_position_mm"))
        if vision_xyz is None:
            print("Sample refused: vision payload does not contain tool_position_mm")
            return False

        snapshot_age = payload.get("snapshot_age_ms")
        if isinstance(snapshot_age, (int, float)) and snapshot_age > self.max_sample_age_ms:
            print(
                "Sample refused: AprilTag snapshot is stale "
                f"({snapshot_age:.0f} ms > {self.max_sample_age_ms:.0f} ms)."
            )
            return False

        feedback_xyz = serializable_xyz(self.current_position)
        target_xyz = serializable_xyz(self.target_position)
        if feedback_xyz is None or target_xyz is None:
            print("Sample refused: controller feedback position is invalid")
            return False

        offset = np.asarray(vision_xyz, dtype=float) - np.asarray(feedback_xyz, dtype=float)
        self.record_count += 1
        sample = {
            "index": self.record_count,
            "label": label,
            "timestamp_iso": now_iso(),
            "timestamp_unix": time.time(),
            "mode": "manual_gamepad_apriltag_workspace_sample",
            "safe_scan_mode": self.safe_scan_mode,
            "sensor_frame_mode": self.sensor_frame_mode,
            "operator_note": self.operator_note,
            "servo_raw": {str(servo_id): int(self.current_servo_positions[servo_id]) for servo_id in self.servo_ids},
            "servo_target_raw": {str(servo_id): int(self.target_servo_positions[servo_id]) for servo_id in self.servo_ids},
            "servo_feedback_joint_angles_deg": (
                [float(v) for v in np.degrees(self.current_angles_rad)]
                if self.current_angles_rad is not None
                else None
            ),
            "fk_feedback_xyz_mm": feedback_xyz,
            "target_xyz_mm": target_xyz,
            "vision_tool_preview": payload,
            "vision_xyz_mm": vision_xyz,
            "vision_minus_fk_offset_mm": [float(v) for v in offset],
            "battery_mv": self.battery_voltage_mv,
            "input_axes_raw": list(self.last_axes_raw),
            "input_axes_motion": list(self.last_motion_dpad_axes),
            "warnings": payload.get("warnings", []),
        }
        append_jsonl(self.sample_jsonl_path, sample)

        row = {
            "index": self.record_count,
            "label": label,
            "timestamp_iso": sample["timestamp_iso"],
            "raw1": self.current_servo_positions[1],
            "raw2": self.current_servo_positions[2],
            "raw3": self.current_servo_positions[3],
            "fk_x_mm": feedback_xyz[0],
            "fk_y_mm": feedback_xyz[1],
            "fk_z_mm": feedback_xyz[2],
            "x_mm": vision_xyz[0],
            "y_mm": vision_xyz[1],
            "z_mm": vision_xyz[2],
            "offset_x_mm": float(offset[0]),
            "offset_y_mm": float(offset[1]),
            "offset_z_mm": float(offset[2]),
            "vision_detection_id": payload.get("detection_id"),
            "vision_snapshot_age_ms": payload.get("snapshot_age_ms"),
            "safe_scan_mode": self.safe_scan_mode,
            "sensor_frame_mode": self.sensor_frame_mode,
            "battery_mv": self.battery_voltage_mv if self.battery_voltage_mv is not None else "",
        }
        append_csv(self.sample_csv_path, row)

        self.sampled_points.append(self.current_position.copy())
        if len(self.sampled_points) > 20:
            self.sampled_points = self.sampled_points[-20:]
        print(
            f"sample #{self.record_count}: "
            f"raw=({row['raw1']},{row['raw2']},{row['raw3']}) "
            f"vision=({vision_xyz[0]:+.2f},{vision_xyz[1]:+.2f},{vision_xyz[2]:+.2f}) mm "
            f"offset=({offset[0]:+.2f},{offset[1]:+.2f},{offset[2]:+.2f}) mm"
        )
        for warning in payload.get("warnings", []):
            print(f"warning: {warning}")
        self.write_runtime_status()
        return True

    def build_status_snapshot(self) -> str:
        base = super().build_status_snapshot()
        lines = [
            "",
            "AprilTag workspace sampler",
            f"samples_csv: {self.sample_csv_path}",
            f"samples_jsonl: {self.sample_jsonl_path}",
            f"base_camera_snapshot: {self.apriltag_snapshot_path}",
            f"max_sample_age_ms: {self.max_sample_age_ms:.0f}",
        ]
        if self.vision_tool_preview is not None:
            xyz = self.vision_tool_preview.get("tool_position_mm")
            if isinstance(xyz, list) and len(xyz) >= 3:
                fk = self.current_position
                offset = np.asarray(xyz, dtype=float) - fk
                lines.append(
                    "vision-minus-fk offset: "
                    f"dx={offset[0]:+.2f} dy={offset[1]:+.2f} dz={offset[2]:+.2f} mm"
                )
        return base + "\n".join(lines) + "\n"

    def run_sampler(self) -> None:
        if self.sampler_args.gamepad_backend == "8bitdo":
            install_8bitdo_reader(
                self,
                config_path=self.sampler_args.gamepad_config,
                device_path=self.sampler_args.gamepad_device or None,
            )

        if not self.connect():
            self.cleanup()
            return

        if not self.confirm_and_init():
            self.cleanup()
            return

        self.sync_sensor_feedback(force=True)
        self.update_vision_tool_preview(force=True)
        print("")
        print("Start AprilTag workspace sampling control.")
        print("8BitDo: D-pad -> X/Y, right stick Y -> Z, A -> quit, B -> sample, START -> play last sample segment.")
        print("X -> safe scan axis, Y -> sensor frame mode, LB/RB -> landing gear DOWN/UP on servos 4/5/6.")
        print(f"Samples: {self.sample_csv_path}")
        print(f"Full JSONL: {self.sample_jsonl_path}")

        try:
            last_status_write = 0.0
            while True:
                if not self.sync_servo_feedback():
                    print(self.safety_fault_message or "Feedback sync failed.")
                    break

                self.sync_sensor_feedback()
                self.update_vision_tool_preview()
                continue_run, _changed = self.update_from_gamepad()
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


def maybe_start_jetson_apriltag(args: argparse.Namespace) -> List[ManagedProcess]:
    if args.no_autostart_apriltag:
        return []
    if not args.apriltag_launch.exists():
        print(f"warning: AprilTag launch script not found: {args.apriltag_launch}")
        return []
    process_env = None
    if args.base_camera_snapshot:
        import os

        process_env = os.environ.copy()
        process_env["OUT_JSON"] = str(args.base_camera_snapshot)
    print(f"Starting Jetson AprilTag process: {args.apriltag_launch}")
    return [
        start_process(
            "jetson_apriltag3k",
            ["bash", str(args.apriltag_launch)],
            cwd=args.apriltag_launch.parent,
            log_dir=args.output_dir / "logs",
            env=process_env,
        )
    ]


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    default_output_dir = THIS_DIR / "apriltag_workspace_samples"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--output-dir", type=Path, default=default_output_dir)
    parser.add_argument("--base-camera-snapshot", type=Path, default=DEFAULT_JETSON_APRILTAG_JSON)
    parser.add_argument("--calibration", type=Path, default=VisionToolPreviewConfig.calibration_path)
    parser.add_argument("--imu-snapshot", type=Path, default=VisionToolPreviewConfig.imu_snapshot_path)
    parser.add_argument("--hand-tag-id", type=int, default=None)
    parser.add_argument("--fresh-ms", type=float, default=1000.0)
    parser.add_argument("--vision-interval-sec", type=float, default=0.12)
    parser.add_argument("--speed-xy-mm-s", type=float, default=35.0)
    parser.add_argument("--speed-z-mm-s", type=float, default=25.0)
    parser.add_argument("--max-servo-speed-ticks-s", type=float, default=180.0)
    parser.add_argument("--playback-speed-mm-s", type=float, default=35.0)
    parser.add_argument("--playback-step-mm", type=float, default=1.5)
    parser.add_argument("--playback-endpoint-tolerance-mm", type=float, default=22.0)
    parser.add_argument("--gamepad-backend", choices=["8bitdo", "pygame"], default="8bitdo")
    parser.add_argument("--gamepad-config", type=Path, default=DEFAULT_8BITDO_CONFIG)
    parser.add_argument("--gamepad-device", default="")
    parser.add_argument("--apriltag-launch", type=Path, default=DEFAULT_JETSON_APRILTAG_SCRIPT)
    parser.add_argument("--no-autostart-apriltag", action="store_true")
    parser.add_argument("--apriltag-startup-timeout", type=float, default=20.0)
    parser.add_argument("--note", default="")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    managed: List[ManagedProcess] = []
    try:
        managed = maybe_start_jetson_apriltag(args)
        wait_for_fresh_snapshot(
            args.base_camera_snapshot,
            max_age_ms=args.fresh_ms,
            timeout_sec=args.apriltag_startup_timeout,
        )
        controller = AprilTagWorkspaceSamplerController(
            port=args.port,
            baudrate=args.baudrate,
            sampler_args=args,
        )
        controller.run_sampler()
        return 0
    finally:
        stop_processes(managed)


if __name__ == "__main__":
    raise SystemExit(main())
