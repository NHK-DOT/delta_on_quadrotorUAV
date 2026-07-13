#!/usr/bin/env python3
"""Interactive UWB calibration recorder.

This node is intentionally read-only. It subscribes to UWB AOA and rangefinder
data, applies the same UWB body-frame conversion used by test_mission_node.py,
and records raw plus summary CSV files for later calibration analysis.
"""

import csv
import math
import os
import statistics
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range

from uav_delta_msgs.msg import UwbAoa


SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

HEIGHT_LEVELS = [
    ("ground", "tag on ground"),
    ("low", "tag raised low, roughly below 0.5m"),
    ("high", "tag raised high, roughly around 1.0m"),
]
ANGLES_DEG = [0, 45, 90, 135, 180, 225, 270, 315]
DIRECTION_LABELS = {
    0: "front",
    45: "front-right",
    90: "right",
    135: "rear-right",
    180: "rear",
    225: "rear-left",
    270: "left",
    315: "front-left",
}


@dataclass(frozen=True)
class PoseSpec:
    index: int
    total: int
    pose_id: str
    height_level: str
    angle_deg: int
    is_center: bool
    description: str


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def generate_pose_specs():
    specs = []
    total = len(HEIGHT_LEVELS) * (len(ANGLES_DEG) + 1)
    for height_level, height_desc in HEIGHT_LEVELS:
        for angle in ANGLES_DEG:
            direction = DIRECTION_LABELS[angle]
            specs.append(
                PoseSpec(
                    index=len(specs) + 1,
                    total=total,
                    pose_id=f"{height_level}_{angle:03d}",
                    height_level=height_level,
                    angle_deg=angle,
                    is_center=False,
                    description=f"{height_desc}; {angle:03d}deg {direction}",
                )
            )
        specs.append(
            PoseSpec(
                index=len(specs) + 1,
                total=total,
                pose_id=f"{height_level}_center",
                height_level=height_level,
                angle_deg=-1,
                is_center=True,
                description=f"{height_desc}; center under UAV projection",
            )
        )
    return specs


def circular_mean_deg(values):
    if not values:
        return None
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(sin_sum, cos_sum))


def circular_std_deg(values):
    if len(values) < 2:
        return 0.0
    sin_mean = sum(math.sin(math.radians(v)) for v in values) / len(values)
    cos_mean = sum(math.cos(math.radians(v)) for v in values) / len(values)
    radius = clamp(math.sqrt(sin_mean * sin_mean + cos_mean * cos_mean), 1e-9, 1.0)
    return math.degrees(math.sqrt(-2.0 * math.log(radius)))


def linear_stats(values):
    if not values:
        return None
    return {
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def angle_stats(values):
    if not values:
        return None
    return {
        "mean": circular_mean_deg(values),
        "std": circular_std_deg(values),
        "min": min(values),
        "max": max(values),
    }


def fmt_stats(stats, unit="", precision=2):
    if stats is None:
        return "n/a"
    return (
        f"mean={stats['mean']:.{precision}f}{unit} "
        f"std={stats['std']:.{precision}f}{unit} "
        f"min={stats['min']:.{precision}f}{unit} "
        f"max={stats['max']:.{precision}f}{unit}"
    )


class UwbCalibrationRecorder(Node):
    def __init__(self):
        super().__init__("uwb_calibration_recorder")

        self.uwb_aoa_topic = self.declare_parameter("uwb_aoa_topic", "uwb_aoa/data").value
        self.rangefinder_topic = self.declare_parameter(
            "rangefinder_topic", "/mavros/rangefinder_pub"
        ).value
        self.rangefinder_timeout_sec = max(
            0.1,
            float(self.declare_parameter("rangefinder_timeout_sec", 1.0).value),
        )
        self.tag_height_m = float(self.declare_parameter("tag_height_m", 0.0).value)
        self.uwb_azimuth_offset_deg = float(
            self.declare_parameter("uwb_azimuth_offset_deg", 0.0).value
        )
        self.uwb_mount_pitch_down_deg = clamp(
            float(self.declare_parameter("uwb_mount_pitch_down_deg", -45.0).value),
            -89.0,
            89.0,
        )
        self.uwb_forward_sign = float(self.declare_parameter("uwb_forward_sign", 1.0).value)
        self.uwb_lateral_sign = float(self.declare_parameter("uwb_lateral_sign", -1.0).value)
        self.uwb_min_body_elevation_deg = clamp(
            float(self.declare_parameter("uwb_min_body_elevation_deg", 8.0).value),
            -89.0,
            89.0,
        )
        self.uwb_approach_front_sector_deg = clamp(
            abs(float(self.declare_parameter("uwb_approach_front_sector_deg", 65.0).value)),
            5.0,
            179.0,
        )
        self.sample_window_sec = max(
            0.5,
            float(self.declare_parameter("sample_window_sec", 3.0).value),
        )
        self.live_print_period_sec = max(
            0.2,
            float(self.declare_parameter("live_print_period_sec", 1.0).value),
        )
        self.output_dir = Path(str(self.declare_parameter("output_dir", "/tmp").value))

        self._pose_specs = generate_pose_specs()
        self._lock = threading.Lock()
        self._latest = None
        self._latest_recv_time = None
        self._last_range = None
        self._last_range_recv_time = None
        self._samples = []
        self._sampling = False
        self._last_live_print = 0.0
        self._stop_live_print = False
        self._results = []
        self._raw_rows = []

        self.create_subscription(UwbAoa, self.uwb_aoa_topic, self._uwb_callback, 10)
        self.create_subscription(Range, self.rangefinder_topic, self._range_callback, SENSOR_QOS)
        self.create_timer(0.1, self._live_timer)

        self.get_logger().info(
            "uwb_calibration_recorder started: "
            f"uwb_topic={self.uwb_aoa_topic} range_topic={self.rangefinder_topic} "
            f"sample_window={self.sample_window_sec:.1f}s tag_height={self.tag_height_m:.2f}m "
            f"mount_pitch={self.uwb_mount_pitch_down_deg:.1f}deg "
            f"az_offset={self.uwb_azimuth_offset_deg:.1f}deg "
            f"poses={len(self._pose_specs)}"
        )

    def _uwb_callback(self, msg: UwbAoa):
        frame = self._frame_from_msg(msg)
        with self._lock:
            self._latest = frame
            self._latest_recv_time = time.monotonic()
            if self._sampling:
                self._samples.append(frame)

    def _range_callback(self, msg: Range):
        value = float(msg.range)
        valid = math.isfinite(value) and msg.min_range <= value <= msg.max_range
        with self._lock:
            self._last_range = {
                "range_m": value,
                "range_min_m": float(msg.min_range),
                "range_max_m": float(msg.max_range),
                "range_valid": bool(valid),
            }
            self._last_range_recv_time = time.monotonic()

    def _range_snapshot(self, now_monotonic):
        if self._last_range is None or self._last_range_recv_time is None:
            return {
                "range_m": None,
                "range_min_m": None,
                "range_max_m": None,
                "range_age_sec": None,
                "range_valid": False,
                "range_fresh": False,
                "range_minus_tag_height_m": None,
            }
        age = now_monotonic - self._last_range_recv_time
        fresh = age <= self.rangefinder_timeout_sec
        valid = bool(self._last_range["range_valid"] and fresh)
        range_m = self._last_range["range_m"]
        return {
            **self._last_range,
            "range_age_sec": age,
            "range_valid": valid,
            "range_fresh": fresh,
            "range_minus_tag_height_m": range_m - self.tag_height_m,
        }

    def _frame_from_msg(self, msg: UwbAoa):
        geom = self._uwb_body_geometry(msg.azimuth_deg, msg.distance_m, msg.elevation_deg)
        now_ros = self.get_clock().now().nanoseconds / 1e9
        now_monotonic = time.monotonic()
        with self._lock:
            range_data = self._range_snapshot(now_monotonic)
        return {
            "stamp_sec": now_ros,
            "tag_height_m": self.tag_height_m,
            "distance_m": float(msg.distance_m),
            "raw_azimuth_deg": float(msg.azimuth_deg),
            "raw_elevation_deg": float(msg.elevation_deg),
            "quality": float(msg.quality),
            "signal_valid": bool(msg.signal_valid),
            **range_data,
            **geom,
        }

    def _uwb_body_geometry(self, raw_azimuth, distance, elevation):
        azimuth_base = raw_azimuth - self.uwb_azimuth_offset_deg
        az_rad = math.radians(azimuth_base)
        el_rad = math.radians(elevation)
        pitch_rad = math.radians(self.uwb_mount_pitch_down_deg)

        x_base = distance * math.cos(el_rad) * math.cos(az_rad)
        y_base = distance * math.cos(el_rad) * math.sin(az_rad)
        z_base = distance * math.sin(el_rad)

        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        x_body = cos_p * x_base + sin_p * z_base
        y_body = y_base
        z_body = -sin_p * x_base + cos_p * z_base

        x_body *= self.uwb_forward_sign
        y_body *= self.uwb_lateral_sign

        horizontal_dist = math.sqrt(x_body * x_body + y_body * y_body)
        body_azimuth = math.degrees(math.atan2(y_body, x_body))
        body_elevation = math.degrees(math.atan2(z_body, horizontal_dist))

        return {
            "body_azimuth_deg": body_azimuth,
            "body_elevation_deg": body_elevation,
            "horizontal_dist_m": horizontal_dist,
            "forward_dist_m": x_body,
            "lateral_dist_m": y_body,
            "vertical_dist_m": z_body,
        }

    def _live_timer(self):
        if self._stop_live_print:
            return
        now = time.monotonic()
        if now - self._last_live_print < self.live_print_period_sec:
            return
        self._last_live_print = now

        with self._lock:
            latest = self._latest
            recv_time = self._latest_recv_time
            sampling = self._sampling

        if latest is None:
            self.get_logger().info("waiting for UWB data...")
            return

        age = now - recv_time if recv_time is not None else float("inf")
        range_text = "missing"
        if latest["range_m"] is not None:
            range_text = (
                f"{latest['range_m']:.2f}m valid={str(latest['range_valid']).lower()} "
                f"age={latest['range_age_sec']:.2f}s"
            )
        prefix = "sampling" if sampling else "live"
        self.get_logger().info(
            f"{prefix}: uwb_age={age:.2f}s valid={str(latest['signal_valid']).lower()} "
            f"q={latest['quality']:.2f} raw=("
            f"d={latest['distance_m']:.2f}m az={latest['raw_azimuth_deg']:.1f}deg "
            f"el={latest['raw_elevation_deg']:.1f}deg) body=("
            f"az={latest['body_azimuth_deg']:.1f}deg "
            f"el={latest['body_elevation_deg']:.1f}deg "
            f"fwd={latest['forward_dist_m']:.2f}m "
            f"lat={latest['lateral_dist_m']:.2f}m "
            f"h={latest['horizontal_dist_m']:.2f}m) range={range_text}"
        )

    def run_interactive(self):
        print("")
        print("UWB calibration recorder")
        print("Read-only: subscribes to UWB/rangefinder data and sends no flight commands.")
        print("Angle convention: front=0deg, right=90deg, rear=180deg, left=270deg.")
        print(
            f"Sequence: {len(self._pose_specs)} poses = "
            "3 height levels x (8 directions + center)."
        )
        print(f"tag_height_m={self.tag_height_m:.2f}m, sample_window={self.sample_window_sec:.1f}s")
        print("Place the tag for each prompt, wait for stable live data, then press Enter.")
        print("Press Ctrl-C to stop early.")
        print("")

        self._wait_for_first_frame()

        try:
            for spec in self._pose_specs:
                self._print_pose_prompt(spec)
                self._prompt_enter("Press Enter to record this pose...")
                summary = self._record_pose(spec)
                self._results.append(summary)
                self._print_summary(summary)
        finally:
            self._stop_live_print = True

        self._print_final_report()
        summary_path, raw_path = self._write_csv_files()
        print(f"\nSummary CSV saved: {summary_path}")
        print(f"Raw CSV saved:     {raw_path}")

    def _print_pose_prompt(self, spec: PoseSpec):
        angle_text = "CENTER" if spec.is_center else f"{spec.angle_deg:03d}deg"
        print("")
        print("=" * 72)
        print(
            f"[{spec.index:02d}/{spec.total:02d}] {spec.pose_id}: "
            f"height={spec.height_level} angle={angle_text}"
        )
        print(f"Place tag: {spec.description}")
        next_spec = self._pose_specs[spec.index] if spec.index < spec.total else None
        if next_spec is not None:
            next_angle = "CENTER" if next_spec.is_center else f"{next_spec.angle_deg:03d}deg"
            print(
                f"Next after this: {next_spec.pose_id} "
                f"(height={next_spec.height_level}, angle={next_angle})"
            )
        else:
            print("Next after this: finish and write CSV files")

    def _prompt_enter(self, prompt):
        if os.name == "posix":
            try:
                with open("/dev/tty", "r", encoding="utf-8") as tty:
                    print(prompt, end="", flush=True)
                    tty.readline()
                    return
            except OSError:
                pass
        input(prompt)

    def _wait_for_first_frame(self):
        start = time.monotonic()
        while rclpy.ok():
            with self._lock:
                if self._latest is not None:
                    return
            if time.monotonic() - start > 30.0:
                print("Still waiting for UWB data. Check driver, serial port, and tag power.")
                start = time.monotonic()
            time.sleep(0.2)

    def _record_pose(self, spec: PoseSpec):
        print(f"Recording {spec.pose_id} for {self.sample_window_sec:.1f}s...")
        with self._lock:
            self._samples = []
            self._sampling = True

        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < self.sample_window_sec:
            time.sleep(0.05)

        with self._lock:
            samples = list(self._samples)
            self._sampling = False

        summary = self._summarize_samples(spec, samples)
        self._append_raw_rows(spec, samples)
        print(
            f"[OK] {spec.pose_id} collected: "
            f"uwb_valid={summary['valid_frames']}/{summary['total_frames']} "
            f"range_valid={summary['range_valid_frames']}/{summary['total_frames']} "
            f"verdict={summary['verdict']}"
        )
        return summary

    def _append_raw_rows(self, spec: PoseSpec, samples):
        for sample_index, sample in enumerate(samples):
            self._raw_rows.append(
                {
                    **self._pose_metadata(spec),
                    "sample_index": sample_index,
                    **sample,
                }
            )

    def _pose_metadata(self, spec: PoseSpec):
        return {
            "pose_id": spec.pose_id,
            "index": spec.index,
            "total": spec.total,
            "height_level": spec.height_level,
            "angle_deg": spec.angle_deg,
            "is_center": spec.is_center,
            "description": spec.description,
        }

    def _summarize_samples(self, spec: PoseSpec, samples):
        valid_samples = [s for s in samples if s["signal_valid"] and s["quality"] > 0.0]
        range_valid_samples = [s for s in samples if s["range_valid"]]
        elapsed = 0.0
        if samples:
            elapsed = max(0.0, samples[-1]["stamp_sec"] - samples[0]["stamp_sec"])
        hz = len(samples) / elapsed if elapsed > 0.0 else 0.0

        fields = {
            "distance_m": linear_stats([s["distance_m"] for s in valid_samples]),
            "raw_azimuth_deg": angle_stats([s["raw_azimuth_deg"] for s in valid_samples]),
            "raw_elevation_deg": angle_stats([s["raw_elevation_deg"] for s in valid_samples]),
            "quality": linear_stats([s["quality"] for s in valid_samples]),
            "body_azimuth_deg": angle_stats([s["body_azimuth_deg"] for s in valid_samples]),
            "body_elevation_deg": angle_stats([s["body_elevation_deg"] for s in valid_samples]),
            "horizontal_dist_m": linear_stats([s["horizontal_dist_m"] for s in valid_samples]),
            "forward_dist_m": linear_stats([s["forward_dist_m"] for s in valid_samples]),
            "lateral_dist_m": linear_stats([s["lateral_dist_m"] for s in valid_samples]),
            "vertical_dist_m": linear_stats([s["vertical_dist_m"] for s in valid_samples]),
            "range_m": linear_stats([s["range_m"] for s in range_valid_samples]),
            "range_age_sec": linear_stats([s["range_age_sec"] for s in range_valid_samples]),
            "range_minus_tag_height_m": linear_stats(
                [s["range_minus_tag_height_m"] for s in range_valid_samples]
            ),
        }
        verdict, reasons = self._judge_pose(spec, fields, len(valid_samples), len(samples))

        return {
            **self._pose_metadata(spec),
            "total_frames": len(samples),
            "valid_frames": len(valid_samples),
            "valid_ratio": (len(valid_samples) / len(samples)) if samples else 0.0,
            "range_valid_frames": len(range_valid_samples),
            "range_valid_ratio": (len(range_valid_samples) / len(samples)) if samples else 0.0,
            "hz": hz,
            "fields": fields,
            "verdict": verdict,
            "reasons": reasons,
        }

    def _judge_pose(self, spec: PoseSpec, fields, valid_frames, total_frames):
        reasons = []
        if total_frames == 0:
            return "SUSPECT", ["no UWB frames received during sample window"]
        if valid_frames == 0:
            return "SUSPECT", ["no valid UWB frames during sample window"]
        if valid_frames / total_frames < 0.8:
            reasons.append("UWB valid frame ratio below 80%")

        if fields["range_m"] is None:
            reasons.append("no fresh valid rangefinder samples")

        body_el = fields["body_elevation_deg"]["mean"]
        horizontal = max(fields["horizontal_dist_m"]["mean"], 1e-6)
        forward = fields["forward_dist_m"]["mean"]
        lateral = fields["lateral_dist_m"]["mean"]
        distance = max(fields["distance_m"]["mean"], 1e-6)

        if body_el < self.uwb_min_body_elevation_deg:
            reasons.append(
                f"body elevation {body_el:.1f}deg below mission filter "
                f"{self.uwb_min_body_elevation_deg:.1f}deg"
            )

        if spec.is_center:
            if horizontal / distance > 0.35:
                reasons.append(f"center horizontal component is large: hdist/dist={horizontal / distance:.2f}")
            return ("PASS" if not reasons else "SUSPECT"), reasons

        expected = math.radians(spec.angle_deg)
        expected_forward = math.cos(expected)
        expected_lateral = math.sin(expected)
        measured_norm = math.sqrt(forward * forward + lateral * lateral)
        if measured_norm < 0.05:
            reasons.append("horizontal body vector too small for directional judgment")
        else:
            measured_forward = forward / measured_norm
            measured_lateral = lateral / measured_norm
            dot = expected_forward * measured_forward + expected_lateral * measured_lateral
            dot = clamp(dot, -1.0, 1.0)
            angle_error = math.degrees(math.acos(dot))
            if angle_error > 45.0:
                reasons.append(f"body horizontal direction error is large: {angle_error:.1f}deg")

        return ("PASS" if not reasons else "SUSPECT"), reasons

    def _print_summary(self, summary):
        fields = summary["fields"]
        print("")
        print(f"Result [{summary['pose_id']}] {summary['verdict']}")
        print(
            f"frames uwb_valid/total={summary['valid_frames']}/{summary['total_frames']} "
            f"range_valid/total={summary['range_valid_frames']}/{summary['total_frames']} "
            f"hz={summary['hz']:.1f}"
        )
        print(f"raw distance:      {fmt_stats(fields['distance_m'], 'm')}")
        print(f"raw azimuth:       {fmt_stats(fields['raw_azimuth_deg'], 'deg')}")
        print(f"raw elevation:     {fmt_stats(fields['raw_elevation_deg'], 'deg')}")
        print(f"body azimuth:      {fmt_stats(fields['body_azimuth_deg'], 'deg')}")
        print(f"body elevation:    {fmt_stats(fields['body_elevation_deg'], 'deg')}")
        print(f"body horizontal:   {fmt_stats(fields['horizontal_dist_m'], 'm')}")
        print(f"body forward:      {fmt_stats(fields['forward_dist_m'], 'm')}")
        print(f"body lateral:      {fmt_stats(fields['lateral_dist_m'], 'm')}")
        print(f"body vertical:     {fmt_stats(fields['vertical_dist_m'], 'm')}")
        print(f"rangefinder:       {fmt_stats(fields['range_m'], 'm')}")
        print(f"range-tag_height:  {fmt_stats(fields['range_minus_tag_height_m'], 'm')}")
        if summary["reasons"]:
            print("notes:")
            for reason in summary["reasons"]:
                print(f"  - {reason}")

    def _print_final_report(self):
        print("")
        print("Final UWB calibration summary")
        print(
            "pose             verdict  uwb     range   body_az  body_el  fwd_m   lat_m   range_m"
        )
        for result in self._results:
            fields = result["fields"]

            def mean(name):
                stats = fields.get(name)
                return stats["mean"] if stats else float("nan")

            print(
                f"{result['pose_id']:<16} {result['verdict']:<7} "
                f"{result['valid_frames']:>3}/{result['total_frames']:<3} "
                f"{result['range_valid_frames']:>3}/{result['total_frames']:<3} "
                f"{mean('body_azimuth_deg'):>7.1f} "
                f"{mean('body_elevation_deg'):>7.1f} "
                f"{mean('forward_dist_m'):>6.2f} "
                f"{mean('lateral_dist_m'):>6.2f} "
                f"{mean('range_m'):>7.2f}"
            )

        if any(result["verdict"] != "PASS" for result in self._results):
            print("")
            print("At least one pose is SUSPECT. Use raw CSV before tuning flight code.")

    def _write_csv_files(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        summary_path = self.output_dir / f"uwb_calibration_{stamp}_summary.csv"
        raw_path = self.output_dir / f"uwb_calibration_{stamp}_raw.csv"
        self._write_summary_csv(summary_path)
        self._write_raw_csv(raw_path)
        return summary_path, raw_path

    def _write_summary_csv(self, path):
        stat_fields = [
            "distance_m",
            "raw_azimuth_deg",
            "raw_elevation_deg",
            "quality",
            "body_azimuth_deg",
            "body_elevation_deg",
            "horizontal_dist_m",
            "forward_dist_m",
            "lateral_dist_m",
            "vertical_dist_m",
            "range_m",
            "range_age_sec",
            "range_minus_tag_height_m",
        ]
        columns = [
            "pose_id",
            "index",
            "total",
            "height_level",
            "angle_deg",
            "is_center",
            "description",
            "tag_height_m",
            "verdict",
            "total_frames",
            "valid_frames",
            "valid_ratio",
            "range_valid_frames",
            "range_valid_ratio",
            "hz",
            "notes",
        ]
        for field in stat_fields:
            for suffix in ("mean", "std", "min", "max"):
                columns.append(f"{field}_{suffix}")

        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            for result in self._results:
                row = {
                    "pose_id": result["pose_id"],
                    "index": result["index"],
                    "total": result["total"],
                    "height_level": result["height_level"],
                    "angle_deg": result["angle_deg"],
                    "is_center": result["is_center"],
                    "description": result["description"],
                    "tag_height_m": f"{self.tag_height_m:.6f}",
                    "verdict": result["verdict"],
                    "total_frames": result["total_frames"],
                    "valid_frames": result["valid_frames"],
                    "valid_ratio": f"{result['valid_ratio']:.4f}",
                    "range_valid_frames": result["range_valid_frames"],
                    "range_valid_ratio": f"{result['range_valid_ratio']:.4f}",
                    "hz": f"{result['hz']:.4f}",
                    "notes": "; ".join(result["reasons"]),
                }
                for field in stat_fields:
                    stats = result["fields"].get(field)
                    for suffix in ("mean", "std", "min", "max"):
                        row[f"{field}_{suffix}"] = "" if stats is None else f"{stats[suffix]:.6f}"
                writer.writerow(row)

    def _write_raw_csv(self, path):
        columns = [
            "pose_id",
            "index",
            "total",
            "sample_index",
            "height_level",
            "angle_deg",
            "is_center",
            "description",
            "stamp_sec",
            "tag_height_m",
            "distance_m",
            "raw_azimuth_deg",
            "raw_elevation_deg",
            "quality",
            "signal_valid",
            "range_m",
            "range_min_m",
            "range_max_m",
            "range_age_sec",
            "range_valid",
            "range_fresh",
            "range_minus_tag_height_m",
            "body_azimuth_deg",
            "body_elevation_deg",
            "horizontal_dist_m",
            "forward_dist_m",
            "lateral_dist_m",
            "vertical_dist_m",
        ]
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            for row in self._raw_rows:
                writer.writerow(row)


def main():
    rclpy.init()
    node = UwbCalibrationRecorder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_interactive()
    except KeyboardInterrupt:
        print("\nStopped by operator.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
