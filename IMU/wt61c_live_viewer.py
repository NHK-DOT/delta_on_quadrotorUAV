"""WT61C live viewer for COM serial output.

Features:
- Real-time WT61/WitMotion frame parsing
- Console status printing
- Overwrite-latest JSON snapshot file
- Matplotlib live dashboard with attitude view and trend plots

Default settings match the currently detected sensor on this machine:
- Port: COM4
- Baudrate: 9600
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque

import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle, Polygon
import serial
from serial import SerialException


ACC_SCALE_G = 16.0
GYRO_SCALE_DPS = 2000.0
ANGLE_SCALE_DEG = 180.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def twos_complement_16(lo: int, hi: int) -> int:
    value = (lo & 0xFF) | ((hi & 0xFF) << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def iso_now() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp_path.replace(path)


@dataclass
class Sample:
    timestamp: float
    iso_time: str
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    angles_deg: tuple[float, float, float]
    temperature_c: float | None
    raw_acc: str
    raw_gyro: str
    raw_angle: str


class WT61CReader:
    def __init__(
        self,
        port: str,
        baudrate: int,
        history_seconds: float,
        snapshot_path: Path,
        snapshot_interval: float,
        print_interval: float,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.snapshot_path = snapshot_path
        self.snapshot_interval = snapshot_interval
        self.print_interval = print_interval
        self.history: Deque[Sample] = deque(maxlen=max(int(history_seconds * 120), 600))
        self._history_seconds = history_seconds
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial: serial.Serial | None = None
        self._buffer = bytearray()
        self._latest_sample: Sample | None = None
        self._sample_count = 0
        self._sample_rate_hz = 0.0
        self._rate_times: Deque[float] = deque()
        self._last_snapshot_time = 0.0
        self._last_print_time = 0.0
        self._partial: dict[str, object] = {}

    def start(self) -> None:
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.10,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
        if self._serial is not None and self._serial.is_open:
            self._serial.close()

    def snapshot(self) -> tuple[list[Sample], Sample | None, int, float]:
        with self._lock:
            return list(self.history), self._latest_sample, self._sample_count, self._sample_rate_hz

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop_event.is_set():
            try:
                waiting = self._serial.in_waiting
                chunk = self._serial.read(waiting or 64)
            except SerialException as exc:
                print(f"\nSerial read failed: {exc}", file=sys.stderr)
                self._stop_event.set()
                break

            if not chunk:
                continue
            self._buffer.extend(chunk)
            self._drain_frames()

    def _drain_frames(self) -> None:
        while len(self._buffer) >= 11:
            if self._buffer[0] != 0x55:
                next_header = self._buffer.find(0x55, 1)
                if next_header == -1:
                    self._buffer.clear()
                    return
                del self._buffer[:next_header]
                continue

            if len(self._buffer) < 11:
                return

            frame = bytes(self._buffer[:11])
            checksum = sum(frame[:10]) & 0xFF
            if checksum != frame[10]:
                del self._buffer[0]
                continue

            del self._buffer[:11]
            self._handle_frame(frame[1], frame[2:10])

    def _handle_frame(self, frame_type: int, payload: bytes) -> None:
        if frame_type == 0x51:
            ax = twos_complement_16(payload[0], payload[1]) / 32768.0 * ACC_SCALE_G
            ay = twos_complement_16(payload[2], payload[3]) / 32768.0 * ACC_SCALE_G
            az = twos_complement_16(payload[4], payload[5]) / 32768.0 * ACC_SCALE_G
            temp_c = twos_complement_16(payload[6], payload[7]) / 340.0 + 36.25
            self._partial["accel_g"] = (ax, ay, az)
            self._partial["temperature_c"] = temp_c
            self._partial["raw_acc"] = payload.hex(" ").upper()
            return

        if frame_type == 0x52:
            gx = twos_complement_16(payload[0], payload[1]) / 32768.0 * GYRO_SCALE_DPS
            gy = twos_complement_16(payload[2], payload[3]) / 32768.0 * GYRO_SCALE_DPS
            gz = twos_complement_16(payload[4], payload[5]) / 32768.0 * GYRO_SCALE_DPS
            self._partial["gyro_dps"] = (gx, gy, gz)
            self._partial["raw_gyro"] = payload.hex(" ").upper()
            return

        if frame_type != 0x53:
            return

        roll = twos_complement_16(payload[0], payload[1]) / 32768.0 * ANGLE_SCALE_DEG
        pitch = twos_complement_16(payload[2], payload[3]) / 32768.0 * ANGLE_SCALE_DEG
        yaw = twos_complement_16(payload[4], payload[5]) / 32768.0 * ANGLE_SCALE_DEG
        self._partial["angles_deg"] = (roll, pitch, yaw)
        self._partial["raw_angle"] = payload.hex(" ").upper()

        if "accel_g" not in self._partial or "gyro_dps" not in self._partial:
            return

        now = time.time()
        self._rate_times.append(now)
        while self._rate_times and (now - self._rate_times[0]) > 2.0:
            self._rate_times.popleft()
        if len(self._rate_times) >= 2:
            span = self._rate_times[-1] - self._rate_times[0]
            if span > 0:
                self._sample_rate_hz = (len(self._rate_times) - 1) / span

        sample = Sample(
            timestamp=now,
            iso_time=iso_now(),
            accel_g=self._partial["accel_g"],  # type: ignore[arg-type]
            gyro_dps=self._partial["gyro_dps"],  # type: ignore[arg-type]
            angles_deg=self._partial["angles_deg"],  # type: ignore[arg-type]
            temperature_c=self._partial.get("temperature_c"),  # type: ignore[arg-type]
            raw_acc=str(self._partial.get("raw_acc", "")),
            raw_gyro=str(self._partial.get("raw_gyro", "")),
            raw_angle=str(self._partial.get("raw_angle", "")),
        )

        with self._lock:
            self._latest_sample = sample
            self.history.append(sample)
            self._sample_count += 1

        if now - self._last_snapshot_time >= self.snapshot_interval:
            self._write_snapshot(sample)
            self._last_snapshot_time = now

        if now - self._last_print_time >= self.print_interval:
            self._print_status(sample)
            self._last_print_time = now

    def _write_snapshot(self, sample: Sample) -> None:
        payload = {
            "sensor": "WT61C",
            "port": self.port,
            "baudrate": self.baudrate,
            "timestamp_unix": sample.timestamp,
            "timestamp_iso": sample.iso_time,
            "temperature_c": sample.temperature_c,
            "sample_count": self._sample_count,
            "sample_rate_hz": round(self._sample_rate_hz, 2),
            "accel_g": {
                "x": round(sample.accel_g[0], 6),
                "y": round(sample.accel_g[1], 6),
                "z": round(sample.accel_g[2], 6),
            },
            "gyro_dps": {
                "x": round(sample.gyro_dps[0], 6),
                "y": round(sample.gyro_dps[1], 6),
                "z": round(sample.gyro_dps[2], 6),
            },
            "angles_deg": {
                "roll": round(sample.angles_deg[0], 6),
                "pitch": round(sample.angles_deg[1], 6),
                "yaw": round(sample.angles_deg[2], 6),
            },
            "raw_payloads": {
                "accel": sample.raw_acc,
                "gyro": sample.raw_gyro,
                "angle": sample.raw_angle,
            },
        }
        atomic_write_json(self.snapshot_path, payload)

    def _print_status(self, sample: Sample) -> None:
        msg = (
            f"[{sample.iso_time}] "
            f"RPY=({sample.angles_deg[0]:7.2f}, {sample.angles_deg[1]:7.2f}, {sample.angles_deg[2]:7.2f}) deg  "
            f"A=({sample.accel_g[0]:6.3f}, {sample.accel_g[1]:6.3f}, {sample.accel_g[2]:6.3f}) g  "
            f"G=({sample.gyro_dps[0]:7.2f}, {sample.gyro_dps[1]:7.2f}, {sample.gyro_dps[2]:7.2f}) dps  "
            f"T={sample.temperature_c:5.2f} C  "
            f"rate={self._sample_rate_hz:6.1f} Hz"
        )
        print(msg)


def rotate_points(points: list[tuple[float, float]], roll_deg: float) -> list[tuple[float, float]]:
    roll_rad = math.radians(roll_deg)
    cos_r = math.cos(roll_rad)
    sin_r = math.sin(roll_rad)
    return [(x * cos_r - y * sin_r, x * sin_r + y * cos_r) for x, y in points]


def compute_limits(values: list[float], minimum_span: float, center_hint: float = 0.0) -> tuple[float, float]:
    if not values:
        half = minimum_span / 2.0
        return center_hint - half, center_hint + half
    vmin = min(values)
    vmax = max(values)
    span = max(vmax - vmin, minimum_span)
    center = (vmin + vmax) / 2.0
    margin = span * 0.15
    return center - span / 2.0 - margin, center + span / 2.0 + margin


class Dashboard:
    def __init__(self, reader: WT61CReader, history_seconds: float, snapshot_path: Path) -> None:
        self.reader = reader
        self.history_seconds = history_seconds
        self.snapshot_path = snapshot_path
        plt.style.use("seaborn-v0_8-darkgrid")

        self.fig = plt.figure(figsize=(15, 8.6), constrained_layout=True)
        self.fig.canvas.manager.set_window_title("WT61C Live Viewer")
        self.fig.patch.set_facecolor("#0F172A")
        gs = GridSpec(2, 3, figure=self.fig, width_ratios=[1.15, 1.45, 1.45])
        self.ax_att = self.fig.add_subplot(gs[:, 0])
        self.ax_angles = self.fig.add_subplot(gs[0, 1:])
        self.ax_acc = self.fig.add_subplot(gs[1, 1])
        self.ax_gyro = self.fig.add_subplot(gs[1, 2])

        self._setup_attitude_axis()
        self._setup_time_axis(self.ax_angles, "Euler Angles", "deg")
        self._setup_time_axis(self.ax_acc, "Acceleration", "g")
        self._setup_time_axis(self.ax_gyro, "Angular Velocity", "deg/s")

        self.angle_lines = {
            "roll": self.ax_angles.plot([], [], color="#22C55E", lw=2.2, label="Roll")[0],
            "pitch": self.ax_angles.plot([], [], color="#38BDF8", lw=2.2, label="Pitch")[0],
            "yaw": self.ax_angles.plot([], [], color="#F59E0B", lw=2.2, label="Yaw")[0],
        }
        self.acc_lines = {
            "ax": self.ax_acc.plot([], [], color="#22C55E", lw=2.0, label="Ax")[0],
            "ay": self.ax_acc.plot([], [], color="#38BDF8", lw=2.0, label="Ay")[0],
            "az": self.ax_acc.plot([], [], color="#F43F5E", lw=2.0, label="Az")[0],
        }
        self.gyro_lines = {
            "gx": self.ax_gyro.plot([], [], color="#22C55E", lw=2.0, label="Gx")[0],
            "gy": self.ax_gyro.plot([], [], color="#38BDF8", lw=2.0, label="Gy")[0],
            "gz": self.ax_gyro.plot([], [], color="#F59E0B", lw=2.0, label="Gz")[0],
        }
        self.ax_angles.legend(loc="upper left", ncol=3, frameon=False)
        self.ax_acc.legend(loc="upper left", ncol=3, frameon=False)
        self.ax_gyro.legend(loc="upper left", ncol=3, frameon=False)

        self.header_text = self.fig.text(
            0.02,
            0.98,
            "",
            va="top",
            ha="left",
            color="#E2E8F0",
            fontsize=12,
            family="Consolas",
        )
        self.footer_text = self.fig.text(
            0.02,
            0.02,
            "",
            va="bottom",
            ha="left",
            color="#94A3B8",
            fontsize=10.5,
            family="Consolas",
        )

    def _setup_attitude_axis(self) -> None:
        self.ax_att.set_title("Attitude Indicator", color="#E2E8F0", fontsize=14, weight="bold")
        self.ax_att.set_facecolor("#111827")
        self.ax_att.set_aspect("equal")
        self.ax_att.set_xlim(-1.15, 1.15)
        self.ax_att.set_ylim(-1.15, 1.15)
        self.ax_att.axis("off")

        self.clip_circle = Circle((0.0, 0.0), 1.0, transform=self.ax_att.transData)
        self.horizon_sky = Polygon([[0, 0]], closed=True, facecolor="#60A5FA", alpha=0.95)
        self.horizon_ground = Polygon([[0, 0]], closed=True, facecolor="#B45309", alpha=0.95)
        self.horizon_sky.set_clip_path(self.clip_circle)
        self.horizon_ground.set_clip_path(self.clip_circle)
        self.ax_att.add_patch(self.horizon_sky)
        self.ax_att.add_patch(self.horizon_ground)

        border = Circle((0.0, 0.0), 1.0, fill=False, lw=3.0, edgecolor="#E2E8F0")
        self.ax_att.add_patch(border)
        self.ax_att.plot([-0.16, 0.16], [0, 0], color="#F8FAFC", lw=2.5)
        self.ax_att.plot([0, 0], [-0.08, 0.08], color="#F8FAFC", lw=2.5)
        self.attitude_text = self.ax_att.text(
            0.0,
            -1.18,
            "",
            ha="center",
            va="top",
            color="#E2E8F0",
            fontsize=11,
            family="Consolas",
        )

    def _setup_time_axis(self, ax, title: str, ylabel: str) -> None:
        ax.set_facecolor("#111827")
        ax.set_title(title, color="#E2E8F0", fontsize=13, weight="bold")
        ax.set_xlabel("Seconds Ago", color="#CBD5E1")
        ax.set_ylabel(ylabel, color="#CBD5E1")
        ax.tick_params(colors="#CBD5E1")
        for spine in ax.spines.values():
            spine.set_color("#334155")
        ax.grid(True, color="#334155", alpha=0.45)
        ax.axhline(0.0, color="#64748B", lw=1.0, alpha=0.7)

    def update(self, _frame_idx: int):
        history, latest, sample_count, sample_rate_hz = self.reader.snapshot()
        if latest is None or not history:
            self.header_text.set_text("Waiting for WT61C samples...")
            self.footer_text.set_text(f"Snapshot file: {self.snapshot_path}")
            return []

        t0 = latest.timestamp
        times = [sample.timestamp - t0 for sample in history]

        rolls = [sample.angles_deg[0] for sample in history]
        pitches = [sample.angles_deg[1] for sample in history]
        yaws = [sample.angles_deg[2] for sample in history]
        axs = [sample.accel_g[0] for sample in history]
        ays = [sample.accel_g[1] for sample in history]
        azs = [sample.accel_g[2] for sample in history]
        gxs = [sample.gyro_dps[0] for sample in history]
        gys = [sample.gyro_dps[1] for sample in history]
        gzs = [sample.gyro_dps[2] for sample in history]

        self.angle_lines["roll"].set_data(times, rolls)
        self.angle_lines["pitch"].set_data(times, pitches)
        self.angle_lines["yaw"].set_data(times, yaws)
        self.acc_lines["ax"].set_data(times, axs)
        self.acc_lines["ay"].set_data(times, ays)
        self.acc_lines["az"].set_data(times, azs)
        self.gyro_lines["gx"].set_data(times, gxs)
        self.gyro_lines["gy"].set_data(times, gys)
        self.gyro_lines["gz"].set_data(times, gzs)

        x_min = -self.history_seconds
        x_max = 0.0
        self.ax_angles.set_xlim(x_min, x_max)
        self.ax_acc.set_xlim(x_min, x_max)
        self.ax_gyro.set_xlim(x_min, x_max)

        self.ax_angles.set_ylim(*compute_limits(rolls + pitches + yaws, minimum_span=20.0))
        self.ax_acc.set_ylim(*compute_limits(axs + ays + azs, minimum_span=1.5, center_hint=0.0))
        self.ax_gyro.set_ylim(*compute_limits(gxs + gys + gzs, minimum_span=20.0, center_hint=0.0))

        self._update_attitude(latest.angles_deg[0], latest.angles_deg[1], latest.angles_deg[2])

        self.header_text.set_text(
            f"WT61C Live Viewer  |  Port={self.reader.port}  Baud={self.reader.baudrate}  "
            f"Samples={sample_count}  Rate={sample_rate_hz:5.1f} Hz  Temp={latest.temperature_c:5.2f} C"
        )
        self.footer_text.set_text(
            f"Latest snapshot: {self.snapshot_path}  |  "
            f"Roll={latest.angles_deg[0]:7.2f} deg  Pitch={latest.angles_deg[1]:7.2f} deg  "
            f"Yaw={latest.angles_deg[2]:7.2f} deg"
        )
        return list(self.angle_lines.values()) + list(self.acc_lines.values()) + list(self.gyro_lines.values())

    def _update_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        pitch_norm = clamp(pitch_deg / 45.0, -0.85, 0.85)
        sky = [(-2.2, pitch_norm), (2.2, pitch_norm), (2.2, 2.2), (-2.2, 2.2)]
        ground = [(-2.2, -2.2), (2.2, -2.2), (2.2, pitch_norm), (-2.2, pitch_norm)]
        self.horizon_sky.set_xy(rotate_points(sky, roll_deg))
        self.horizon_ground.set_xy(rotate_points(ground, roll_deg))
        self.attitude_text.set_text(
            f"Roll  {roll_deg:7.2f} deg\nPitch {pitch_deg:7.2f} deg\nYaw   {yaw_deg:7.2f} deg"
        )


def default_snapshot_path() -> Path:
    return Path(__file__).resolve().with_name("wt61c_latest.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WT61C live reader and dashboard")
    parser.add_argument("--port", default="COM4", help="Serial port, default: COM4")
    parser.add_argument("--baud", type=int, default=9600, help="Baudrate, default: 9600")
    parser.add_argument(
        "--snapshot-file",
        type=Path,
        default=default_snapshot_path(),
        help="Overwrite-latest JSON snapshot file",
    )
    parser.add_argument("--snapshot-interval", type=float, default=0.20, help="Seconds between snapshot updates")
    parser.add_argument("--print-every", type=float, default=0.50, help="Seconds between console prints")
    parser.add_argument("--history-seconds", type=float, default=12.0, help="History shown in plots")
    parser.add_argument("--refresh-ms", type=int, default=100, help="GUI refresh interval in ms")
    parser.add_argument("--no-gui", action="store_true", help="Run in terminal only")
    parser.add_argument("--duration", type=float, default=0.0, help="Auto-exit after N seconds, 0 means infinite")
    return parser.parse_args()


def run_headless(reader: WT61CReader, duration: float) -> None:
    start = time.time()
    print("Headless mode. Press Ctrl+C to stop.")
    while True:
        time.sleep(0.10)
        if duration > 0 and (time.time() - start) >= duration:
            break


def main() -> int:
    args = parse_args()
    reader = WT61CReader(
        port=args.port,
        baudrate=args.baud,
        history_seconds=args.history_seconds,
        snapshot_path=args.snapshot_file,
        snapshot_interval=args.snapshot_interval,
        print_interval=args.print_every,
    )

    try:
        reader.start()
    except SerialException as exc:
        print(f"Failed to open {args.port} at {args.baud}: {exc}", file=sys.stderr)
        return 1

    print(f"WT61C reader started on {args.port} @ {args.baud}")
    print(f"Latest snapshot file: {args.snapshot_file}")
    print("Press Ctrl+C to stop.")

    try:
        if args.no_gui:
            run_headless(reader, args.duration)
        else:
            dashboard = Dashboard(reader, args.history_seconds, args.snapshot_file)
            anim = animation.FuncAnimation(dashboard.fig, dashboard.update, interval=args.refresh_ms, cache_frame_data=False)
            dashboard._anim = anim  # Keep a strong reference on the dashboard instance.
            if args.duration > 0:
                timer = dashboard.fig.canvas.new_timer(interval=int(args.duration * 1000))
                timer.add_callback(plt.close, dashboard.fig)
                timer.start()
            plt.show()
    except KeyboardInterrupt:
        print("\nStopping WT61C viewer...")
    finally:
        reader.stop()
        print("Closed serial port.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
