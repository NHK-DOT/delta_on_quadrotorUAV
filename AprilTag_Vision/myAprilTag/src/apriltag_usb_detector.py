from __future__ import annotations

import argparse
import json
import math
import statistics
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tomllib

import cv2
import numpy as np


FAMILY_TO_DICT = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

BACKENDS = {
    "auto": [("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF), ("ANY", cv2.CAP_ANY)],
    "dshow": [("DSHOW", cv2.CAP_DSHOW)],
    "msmf": [("MSMF", cv2.CAP_MSMF)],
    "any": [("ANY", cv2.CAP_ANY)],
}

REFINE_METHODS = {
    "none": cv2.aruco.CORNER_REFINE_NONE,
    "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
    "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG,
}

PIXEL_FORMATS = {
    "auto": None,
    "mjpg": "MJPG",
    "yuy2": "YUY2",
}
PIXEL_FORMAT_SCAN_CHOICES = ("auto", "mjpg", "yuy2", "all")
DIRECTSHOW_SUBTYPE_NAMES = {
    0xE436EB7D: "RGB24",
}

PROFILE_DEFAULTS = {
    "accuracy": {
        "detect_scale": 1.0,
        "refine": "apriltag",
        "snapshot_hz": 10.0,
        "draw_axes": True,
        "use_roi_tracking": False,
        "roi_padding": 96,
        "max_detect_hz": 0.0,
        "full_frame_interval": 1,
    },
    "balanced": {
        "detect_scale": 0.75,
        "refine": "subpix",
        "snapshot_hz": 5.0,
        "draw_axes": True,
        "use_roi_tracking": True,
        "roi_padding": 96,
        "max_detect_hz": 12.0,
        "full_frame_interval": 2,
    },
    "speed": {
        "detect_scale": 0.5,
        "refine": "none",
        "snapshot_hz": 2.0,
        "draw_axes": False,
        "use_roi_tracking": True,
        "roi_padding": 72,
        "max_detect_hz": 10.0,
        "full_frame_interval": 4,
    },
    "mp257": {
        "detect_scale": 0.5,
        "refine": "none",
        "snapshot_hz": 2.0,
        "draw_axes": False,
        "use_roi_tracking": True,
        "roi_padding": 72,
        "max_detect_hz": 8.0,
        "full_frame_interval": 6,
    },
}

ACCEL_BACKENDS = ("auto", "none", "opencl")
CONFIG_PATH_KEYS = {"calibration_file", "snapshot_file", "capture_dir"}


@dataclass
class DetectionRuntimeState:
    last_detection_bbox: tuple[int, int, int, int] | None = None
    detect_cycles: int = 0
    force_full_frame_search: bool = False


@dataclass
class DetectionResult:
    source_seq: int
    processed_at: float
    detection_latency_ms: float
    resize_latency_ms: float
    grayscale_latency_ms: float
    detector_latency_ms: float
    pose_latency_ms: float
    detections: list[dict]
    overlay_items: list[dict]
    base_roi: tuple[int, int, int, int] | None
    tracked_roi: tuple[int, int, int, int] | None
    active_roi: tuple[int, int, int, int] | None
    force_full_frame: bool
    scale_for_search: float
    frame_width: int
    frame_height: int
    processing_width: int
    processing_height: int


@dataclass
class RuntimeTelemetry:
    frame_samples: int = 0
    detect_samples: int = 0
    capture_wait_ms: float = 0.0
    gui_ms: float = 0.0
    resize_ms: float = 0.0
    grayscale_ms: float = 0.0
    detector_ms: float = 0.0
    pose_ms: float = 0.0
    total_detect_ms: float = 0.0

    def add_frame_sample(self, capture_wait_ms: float, gui_ms: float) -> None:
        self.frame_samples += 1
        self.capture_wait_ms += capture_wait_ms
        self.gui_ms += gui_ms

    def add_detect_sample(self, result: DetectionResult) -> None:
        self.detect_samples += 1
        self.resize_ms += result.resize_latency_ms
        self.grayscale_ms += result.grayscale_latency_ms
        self.detector_ms += result.detector_latency_ms
        self.pose_ms += result.pose_latency_ms
        self.total_detect_ms += result.detection_latency_ms

    def summarize(self) -> dict[str, float]:
        frame_divisor = max(1, self.frame_samples)
        detect_divisor = max(1, self.detect_samples)
        return {
            "capture_wait_ms": self.capture_wait_ms / frame_divisor,
            "gui_ms": self.gui_ms / frame_divisor,
            "resize_ms": self.resize_ms / detect_divisor,
            "grayscale_ms": self.grayscale_ms / detect_divisor,
            "detector_ms": self.detector_ms / detect_divisor,
            "pose_ms": self.pose_ms / detect_divisor,
            "total_detect_ms": self.total_detect_ms / detect_divisor,
        }

    def reset(self) -> None:
        self.frame_samples = 0
        self.detect_samples = 0
        self.capture_wait_ms = 0.0
        self.gui_ms = 0.0
        self.resize_ms = 0.0
        self.grayscale_ms = 0.0
        self.detector_ms = 0.0
        self.pose_ms = 0.0
        self.total_detect_ms = 0.0


class LatestFrameReader:
    def __init__(self, cap: cv2.VideoCapture) -> None:
        self.cap = cap
        self.lock = threading.Lock()
        self.frame = None
        self.ok = False
        self.seq = 0
        self.fail_count = 0
        self.stopped = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        while not self.stopped:
            ok, frame = self.cap.read()
            with self.lock:
                if ok and frame is not None:
                    self.ok = True
                    self.frame = frame
                    self.seq += 1
                    self.fail_count = 0
                else:
                    self.ok = False
                    self.fail_count += 1

    def read(self) -> tuple[bool, np.ndarray | None, int, int]:
        with self.lock:
            if self.frame is None:
                return self.ok, None, self.seq, self.fail_count
            return self.ok, self.frame.copy(), self.seq, self.fail_count

    def stop(self) -> None:
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)


class SnapshotWriter:
    def __init__(self, path: Path, pretty: bool) -> None:
        self.path = path
        self.pretty = pretty
        self.lock = threading.Lock()
        self.pending_text: str | None = None
        self.pending_version = 0
        self.written_version = 0
        self.stop_requested = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def submit(self, payload: dict) -> None:
        if self.pretty:
            text = json.dumps(payload, ensure_ascii=False, indent=2)
        else:
            text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

        with self.lock:
            self.pending_text = text
            self.pending_version += 1

    def _write_text(self, text: str) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self.path.with_name(f"{self.path.name}.tmp")
        try:
            temp_path.write_text(text, encoding="utf-8")
            temp_path.replace(self.path)
        except OSError:
            self.path.write_text(text, encoding="utf-8")

    def _run(self) -> None:
        while True:
            text_to_write = None
            version_to_write = 0
            with self.lock:
                if self.pending_version > self.written_version and self.pending_text is not None:
                    text_to_write = self.pending_text
                    version_to_write = self.pending_version
                elif self.stop_requested:
                    break

            if text_to_write is None:
                time.sleep(0.01)
                continue

            self._write_text(text_to_write)
            with self.lock:
                self.written_version = max(self.written_version, version_to_write)

    def stop(self) -> None:
        with self.lock:
            self.stop_requested = True
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)


def load_calibration(path: Path | None) -> tuple[np.ndarray | None, np.ndarray | None]:
    if path is None or not path.exists():
        return None, None
    data = json.loads(path.read_text(encoding="utf-8"))
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def flatten_config(data: dict[str, Any]) -> dict[str, Any]:
    flattened: dict[str, Any] = {}
    for key, value in data.items():
        if isinstance(value, dict):
            flattened.update(flatten_config(value))
        else:
            flattened[key] = value
    return flattened


def resolve_config_path(value: str | Path, project_root: Path) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return project_root / path


def load_config_defaults(config_path: Path, project_root: Path) -> dict[str, Any]:
    if not config_path.exists():
        return {}

    data = tomllib.loads(config_path.read_text(encoding="utf-8"))
    config = flatten_config(data)
    for key in CONFIG_PATH_KEYS:
        if key in config and config[key]:
            config[key] = resolve_config_path(config[key], project_root)
    return config


def build_arg_parser(project_root: Path, config_defaults: dict[str, Any], default_config_path: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="USB camera AprilTag detection and distance estimation.")
    parser.add_argument("--config", type=Path, default=default_config_path, help="Load settings from a TOML config file")
    parser.add_argument("--camera-index", type=int, default=int(config_defaults.get("camera_index", 0)))
    parser.add_argument("--backend", choices=sorted(BACKENDS), default=str(config_defaults.get("backend", "auto")))
    parser.add_argument("--profile", choices=sorted(PROFILE_DEFAULTS), default=str(config_defaults.get("profile", "balanced")))
    parser.add_argument("--accel", choices=ACCEL_BACKENDS, default=str(config_defaults.get("accel", "auto")))
    parser.add_argument("--pixel-format", choices=sorted(PIXEL_FORMATS), default=str(config_defaults.get("pixel_format", "auto")))
    parser.add_argument("--strict-pixel-format", action="store_true", help="Fail if actual opened pixel format does not match the requested one")
    parser.add_argument("--disable-rgb-convert", action="store_true", help="Ask backend to avoid automatic RGB conversion when possible")
    parser.add_argument("--buffer-size", type=int, default=int(config_defaults.get("buffer_size", 1)))
    parser.add_argument("--width", type=int, default=int(config_defaults.get("width", 1920)))
    parser.add_argument("--height", type=int, default=int(config_defaults.get("height", 1080)))
    parser.add_argument("--fps", type=int, default=int(config_defaults.get("fps", 30)))
    parser.add_argument("--process-width", type=int, default=int(config_defaults.get("process_width", 0)), help="Software downsample width for full-frame detection; keeps field of view")
    parser.add_argument("--process-height", type=int, default=int(config_defaults.get("process_height", 0)), help="Software downsample height for full-frame detection; keeps field of view")
    parser.add_argument("--family", choices=sorted(FAMILY_TO_DICT), default=str(config_defaults.get("family", "tag36h11")))
    parser.add_argument("--tag-size-m", type=float, default=float(config_defaults.get("tag_size_m", 0.08)), help="Measured black square edge size")
    parser.add_argument("--horizontal-fov-deg", type=float, default=float(config_defaults.get("horizontal_fov_deg", 70.0)))
    parser.add_argument("--focal-length-px", type=float, default=config_defaults.get("focal_length_px"))
    parser.add_argument("--calibration-file", type=Path, default=config_defaults.get("calibration_file", project_root / "calibration" / "camera_intrinsics.json"))
    parser.add_argument("--display-scale", type=float, default=float(config_defaults.get("display_scale", 0.75)))
    parser.add_argument("--snapshot-file", type=Path, default=config_defaults.get("snapshot_file", project_root / "output" / "apriltag_latest.json"))
    parser.add_argument("--capture-dir", type=Path, default=config_defaults.get("capture_dir", project_root / "output" / "captures"))
    parser.add_argument("--duration", type=float, default=float(config_defaults.get("duration", 0.0)), help="0 means run until q is pressed")
    parser.add_argument("--detect-scale", type=float, default=config_defaults.get("detect_scale"), help="Detection downscale factor in (0,1], lower is faster")
    parser.add_argument("--max-detect-hz", type=float, default=config_defaults.get("max_detect_hz"), help="0 means detect on every frame")
    parser.add_argument("--refine", choices=sorted(REFINE_METHODS), default=config_defaults.get("refine"))
    parser.add_argument("--snapshot-hz", type=float, default=config_defaults.get("snapshot_hz"), help="<=0 disables JSON snapshot writing")
    parser.add_argument("--snapshot-pretty", action="store_true", help="Write indented JSON instead of compact JSON")
    parser.add_argument("--status-hz", type=float, default=float(config_defaults.get("status_hz", 1.0)), help="Console status print frequency")
    parser.add_argument("--roi-padding", type=int, default=config_defaults.get("roi_padding"))
    parser.add_argument("--crop-width", type=int, default=int(config_defaults.get("crop_width", 0)), help="Center crop width for detection ROI; 0 disables")
    parser.add_argument("--crop-height", type=int, default=int(config_defaults.get("crop_height", 0)), help="Center crop height for detection ROI; 0 disables")
    parser.add_argument("--roi-detect-scale", type=float, default=float(config_defaults.get("roi_detect_scale", 1.0)), help="Detection scale used when a ROI is already known")
    parser.add_argument("--full-frame-interval", type=int, default=config_defaults.get("full_frame_interval"), help="Force a whole-frame search every N detection cycles")
    parser.add_argument("--list-cameras", action="store_true", help="Probe camera indices and exit")
    parser.add_argument("--max-camera-index", type=int, default=int(config_defaults.get("max_camera_index", 6)))
    parser.add_argument("--benchmark-capture", action="store_true", help="Measure real new-frame FPS without running AprilTag detection")
    parser.add_argument("--benchmark-seconds", type=float, default=float(config_defaults.get("benchmark_seconds", 5.0)), help="How long capture benchmark runs")
    parser.add_argument("--benchmark-grid", type=str, default=str(config_defaults.get("benchmark_grid", "")), help="Semicolon-separated WIDTHxHEIGHT list for capture scan")
    parser.add_argument("--benchmark-pixel-formats", choices=PIXEL_FORMAT_SCAN_CHOICES, default=str(config_defaults.get("benchmark_pixel_formats", "all")), help="Which pixel formats to benchmark")
    parser.add_argument("--diagnose-accel", action="store_true", help="Print OpenCV/OpenCL/CUDA diagnostics and exit")
    parser.add_argument("--print-config", action="store_true", help="Print resolved runtime configuration at startup")
    parser.add_argument("--show-roi-debug", action="store_true", help="Draw tracked ROI and active ROI boxes")
    parser.set_defaults(
        draw_axes=config_defaults.get("draw_axes"),
        use_roi_tracking=config_defaults.get("use_roi_tracking"),
        async_capture=config_defaults.get("async_capture"),
        snapshot_pretty=bool(config_defaults.get("snapshot_pretty", False)),
        list_cameras=bool(config_defaults.get("list_cameras", False)),
        benchmark_capture=bool(config_defaults.get("benchmark_capture", False)),
        diagnose_accel=bool(config_defaults.get("diagnose_accel", False)),
        print_config=bool(config_defaults.get("print_config", False)),
        show_roi_debug=bool(config_defaults.get("show_roi_debug", False)),
        no_gui=bool(config_defaults.get("no_gui", False)),
        strict_pixel_format=bool(config_defaults.get("strict_pixel_format", False)),
        disable_rgb_convert=bool(config_defaults.get("disable_rgb_convert", False)),
    )
    draw_axes_group = parser.add_mutually_exclusive_group()
    draw_axes_group.add_argument("--draw-axes", dest="draw_axes", action="store_true")
    draw_axes_group.add_argument("--no-draw-axes", dest="draw_axes", action="store_false")
    roi_group = parser.add_mutually_exclusive_group()
    roi_group.add_argument("--use-roi-tracking", dest="use_roi_tracking", action="store_true")
    roi_group.add_argument("--no-roi-tracking", dest="use_roi_tracking", action="store_false")
    async_group = parser.add_mutually_exclusive_group()
    async_group.add_argument("--async-capture", dest="async_capture", action="store_true")
    async_group.add_argument("--sync-capture", dest="async_capture", action="store_false")
    parser.add_argument("--no-gui", action="store_true")
    return parser


def resolve_profile(args: argparse.Namespace) -> argparse.Namespace:
    defaults = PROFILE_DEFAULTS[args.profile].copy()
    for key in (
        "detect_scale",
        "refine",
        "snapshot_hz",
        "draw_axes",
        "use_roi_tracking",
        "roi_padding",
        "max_detect_hz",
        "full_frame_interval",
    ):
        value = getattr(args, key)
        if value is not None:
            defaults[key] = value

    args.detect_scale = max(0.1, min(float(defaults["detect_scale"]), 1.0))
    args.refine = str(defaults["refine"])
    args.snapshot_hz = float(defaults["snapshot_hz"])
    args.draw_axes = bool(defaults["draw_axes"])
    args.use_roi_tracking = bool(defaults["use_roi_tracking"])
    args.roi_padding = int(defaults["roi_padding"])
    args.max_detect_hz = max(0.0, float(defaults["max_detect_hz"]))
    args.full_frame_interval = max(1, int(defaults["full_frame_interval"]))
    args.roi_detect_scale = max(0.1, min(float(args.roi_detect_scale), 1.0))
    args.status_hz = max(0.1, float(args.status_hz))
    args.process_width = max(0, int(args.process_width))
    args.process_height = max(0, int(args.process_height))
    return args


def derive_intrinsics(
    frame_width: int,
    frame_height: int,
    camera_matrix: np.ndarray | None,
    focal_length_px: float | None,
    horizontal_fov_deg: float,
) -> tuple[float, float, float, float]:
    if camera_matrix is not None:
        return (
            float(camera_matrix[0, 0]),
            float(camera_matrix[1, 1]),
            float(camera_matrix[0, 2]),
            float(camera_matrix[1, 2]),
        )

    if focal_length_px is not None:
        fx = float(focal_length_px)
    else:
        fx = (frame_width / 2.0) / math.tan(math.radians(horizontal_fov_deg / 2.0))
    fy = fx
    cx = frame_width / 2.0
    cy = frame_height / 2.0
    return fx, fy, cx, cy


def normalize_ids(ids: np.ndarray | cv2.UMat | None) -> np.ndarray | None:
    if ids is None:
        return None
    if hasattr(ids, "get"):
        ids = ids.get()
    if ids is None:
        return None

    try:
        ids_array = np.asarray(ids)
    except Exception:
        return None

    if ids_array.size == 0:
        return None

    if ids_array.dtype != object:
        try:
            ids_array = np.asarray(ids_array, dtype=np.int32)
        except (TypeError, ValueError):
            return None
        if ids_array.ndim == 0:
            ids_array = ids_array.reshape(1)
        return ids_array.reshape(-1)

    cleaned_ids: list[int] = []
    for item in ids_array.reshape(-1):
        if hasattr(item, "get"):
            item = item.get()
        if item is None:
            continue
        try:
            item_array = np.asarray(item, dtype=np.int32).reshape(-1)
        except (TypeError, ValueError):
            continue
        cleaned_ids.extend(int(value) for value in item_array)

    if not cleaned_ids:
        return None
    return np.asarray(cleaned_ids, dtype=np.int32)


def normalize_corners(corners: object) -> list[np.ndarray]:
    if corners is None:
        return []

    if hasattr(corners, "get"):
        corners = corners.get()

    normalized: list[np.ndarray] = []

    def append_corner_array(value: object) -> None:
        if hasattr(value, "get"):
            value = value.get()
        array = np.asarray(value)
        if array.size == 0:
            return
        if array.ndim == 4 and array.shape[-3:] == (1, 4, 2):
            for item in array:
                normalized.append(np.asarray(item, dtype=np.float32).reshape(1, 4, 2))
            return
        if array.ndim == 3 and array.shape[-2:] == (4, 2):
            if array.shape[0] == 1:
                normalized.append(np.asarray(array, dtype=np.float32).reshape(1, 4, 2))
            else:
                for item in array:
                    normalized.append(np.asarray(item, dtype=np.float32).reshape(1, 4, 2))
            return
        if array.ndim == 2 and array.shape == (4, 2):
            normalized.append(np.asarray(array, dtype=np.float32).reshape(1, 4, 2))

    if isinstance(corners, np.ndarray):
        append_corner_array(corners)
    else:
        for corner in corners:
            append_corner_array(corner)
    return normalized


def image_shape(image: np.ndarray | cv2.UMat) -> tuple[int, int]:
    if hasattr(image, "get"):
        image = image.get()
    return int(image.shape[0]), int(image.shape[1])


def configure_capture(
    cap: cv2.VideoCapture,
    width: int,
    height: int,
    fps: int,
    buffer_size: int,
    pixel_format: str,
    disable_rgb_convert: bool,
) -> None:
    if disable_rgb_convert:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if pixel_format != "auto":
        fourcc = PIXEL_FORMATS[pixel_format]
        if fourcc is not None:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if buffer_size > 0:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)


def build_open_params(width: int, height: int, fps: int, pixel_format: str) -> list[int]:
    params: list[int] = []
    if pixel_format != "auto":
        fourcc = PIXEL_FORMATS[pixel_format]
        if fourcc is not None:
            params.extend([cv2.CAP_PROP_FOURCC, int(cv2.VideoWriter_fourcc(*fourcc))])
    if width > 0:
        params.extend([cv2.CAP_PROP_FRAME_WIDTH, int(width)])
    if height > 0:
        params.extend([cv2.CAP_PROP_FRAME_HEIGHT, int(height)])
    if fps > 0:
        params.extend([cv2.CAP_PROP_FPS, int(fps)])
    return params


def decode_fourcc(value: float) -> str:
    fourcc_int = int(round(value))
    fourcc_uint = fourcc_int & 0xFFFFFFFF
    if fourcc_uint in DIRECTSHOW_SUBTYPE_NAMES:
        return DIRECTSHOW_SUBTYPE_NAMES[fourcc_uint]
    chars = [chr((fourcc_uint >> shift) & 0xFF) for shift in (0, 8, 16, 24)]
    text = "".join(chars).strip("\x00")
    if not text:
        return "unknown"
    if not all(32 <= ord(ch) <= 126 for ch in text):
        return f"0x{fourcc_uint:08X}"
    return text


def build_capture_info(cap: cv2.VideoCapture, frame: np.ndarray) -> dict:
    return {
        "width": int(frame.shape[1]),
        "height": int(frame.shape[0]),
        "fps": float(cap.get(cv2.CAP_PROP_FPS)),
        "buffer_size": int(cap.get(cv2.CAP_PROP_BUFFERSIZE)),
        "pixel_format": decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC)),
        "convert_rgb": float(cap.get(cv2.CAP_PROP_CONVERT_RGB)),
    }


def validate_capture_format(capture_info: dict, requested_pixel_format: str, strict_pixel_format: bool) -> None:
    if requested_pixel_format == "auto" or not strict_pixel_format:
        return

    expected = PIXEL_FORMATS[requested_pixel_format]
    actual = str(capture_info["pixel_format"]).upper()
    allowed_actuals = {expected}
    if requested_pixel_format == "mjpg":
        allowed_actuals.add("RGB24")
    if actual != expected:
        if actual in allowed_actuals:
            return
        raise RuntimeError(
            f"Strict pixel format check failed: requested {expected}, opened {actual}. "
            "Requested MJPG is only accepted if backend exposes MJPG itself or decoded RGB24."
        )


def parse_benchmark_grid(grid_text: str, fallback_width: int, fallback_height: int) -> list[tuple[int, int]]:
    if not grid_text.strip():
        return [(fallback_width, fallback_height)]

    parsed: list[tuple[int, int]] = []
    for item in grid_text.split(";"):
        token = item.strip().lower()
        if not token:
            continue
        separator = "x" if "x" in token else ","
        if separator not in token:
            raise ValueError(f"Invalid benchmark grid item: {item!r}")
        width_text, height_text = token.split(separator, 1)
        parsed.append((int(width_text), int(height_text)))
    if not parsed:
        return [(fallback_width, fallback_height)]
    return parsed


def resolve_benchmark_pixel_formats(selection: str) -> list[str]:
    if selection == "all":
        return [name for name in PIXEL_FORMATS if name != "auto"]
    return [selection]


def benchmark_camera_mode(
    camera_index: int,
    backend: str,
    width: int,
    height: int,
    fps: int,
    buffer_size: int,
    pixel_format: str,
    duration: float,
    disable_rgb_convert: bool,
    strict_pixel_format: bool,
) -> dict:
    cap, backend_name, first_frame, capture_info = open_camera(
        camera_index=camera_index,
        width=width,
        height=height,
        fps=fps,
        backend=backend,
        buffer_size=buffer_size,
        pixel_format=pixel_format,
        disable_rgb_convert=disable_rgb_convert,
        strict_pixel_format=strict_pixel_format,
    )

    frame_count = 1
    new_frame_count = 1
    repeated_count = 0
    interval_samples_ms: list[float] = []
    last_frame_signature = int(np.sum(first_frame, dtype=np.uint64) % (1 << 31))
    last_new_frame_time = time.perf_counter()
    start_time = last_new_frame_time

    try:
        while True:
            now = time.perf_counter()
            if (now - start_time) >= duration:
                break

            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            frame_count += 1
            signature = int(np.sum(frame, dtype=np.uint64) % (1 << 31))
            if signature != last_frame_signature:
                current_time = time.perf_counter()
                interval_samples_ms.append((current_time - last_new_frame_time) * 1000.0)
                last_new_frame_time = current_time
                last_frame_signature = signature
                new_frame_count += 1
            else:
                repeated_count += 1
    finally:
        cap.release()

    elapsed = max(time.perf_counter() - start_time, 1e-6)
    actual_new_fps = new_frame_count / elapsed
    delivered_fps = frame_count / elapsed
    median_interval_ms = statistics.median(interval_samples_ms) if interval_samples_ms else 0.0

    return {
        "backend": backend_name,
        "requested_width": width,
        "requested_height": height,
        "requested_fps": fps,
        "requested_pixel_format": pixel_format,
        "actual_width": capture_info["width"],
        "actual_height": capture_info["height"],
        "driver_reported_fps": capture_info["fps"],
        "actual_pixel_format": capture_info["pixel_format"],
        "convert_rgb": capture_info["convert_rgb"],
        "buffer_size": capture_info["buffer_size"],
        "delivered_fps": delivered_fps,
        "actual_new_fps": actual_new_fps,
        "median_new_frame_interval_ms": median_interval_ms,
        "repeated_reads": repeated_count,
        "elapsed_s": elapsed,
    }


def run_capture_benchmark(args: argparse.Namespace) -> None:
    resolutions = parse_benchmark_grid(args.benchmark_grid, args.width, args.height)
    pixel_formats = resolve_benchmark_pixel_formats(args.benchmark_pixel_formats)

    print(f"Capture benchmark camera={args.camera_index} backend={args.backend} duration={args.benchmark_seconds:.1f}s")
    print("Requested format and actual format can differ because drivers may fall back to another mode.")

    results: list[dict] = []
    for width, height in resolutions:
        for pixel_format in pixel_formats:
            print(f"\nBenchmarking request: {width}x{height} format={pixel_format} fps={args.fps}")
            try:
                result = benchmark_camera_mode(
                    camera_index=args.camera_index,
                    backend=args.backend,
                    width=width,
                    height=height,
                    fps=args.fps,
                    buffer_size=args.buffer_size,
                    pixel_format=pixel_format,
                    duration=args.benchmark_seconds,
                    disable_rgb_convert=args.disable_rgb_convert,
                    strict_pixel_format=args.strict_pixel_format,
                )
            except Exception as exc:
                print(f"  failed: {exc}")
                continue

            results.append(result)
            notes: list[str] = []
            if (result["actual_width"], result["actual_height"]) != (result["requested_width"], result["requested_height"]):
                notes.append("resolution_fallback")
            requested_format_text = result["requested_pixel_format"].upper()
            actual_format_text = result["actual_pixel_format"].upper()
            if requested_format_text != "AUTO" and requested_format_text != actual_format_text:
                notes.append("format_not_honored_or_backend_converted")
            print(
                "  "
                f"actual={result['actual_width']}x{result['actual_height']} "
                f"backend={result['backend']} format={result['actual_pixel_format']} "
                f"convert_rgb={result['convert_rgb']:.0f} driver_fps={result['driver_reported_fps']:.1f} delivered_fps={result['delivered_fps']:.1f} "
                f"new_fps={result['actual_new_fps']:.1f} median_interval={result['median_new_frame_interval_ms']:.1f}ms "
                f"repeats={result['repeated_reads']}"
            )
            if notes:
                print(f"  notes: {', '.join(notes)}")

    if not results:
        print("No benchmark mode opened successfully.")
        return

    ranked = sorted(results, key=lambda item: item["actual_new_fps"], reverse=True)
    print("\nBest modes by real new-frame FPS:")
    for index, result in enumerate(ranked[:5], start=1):
        print(
            f"{index}. request={result['requested_width']}x{result['requested_height']} {result['requested_pixel_format']} "
            f"-> actual={result['actual_width']}x{result['actual_height']} {result['actual_pixel_format']} "
            f"new_fps={result['actual_new_fps']:.1f} delivered_fps={result['delivered_fps']:.1f}"
        )


def open_camera(
    camera_index: int,
    width: int,
    height: int,
    fps: int,
    backend: str,
    buffer_size: int,
    pixel_format: str,
    disable_rgb_convert: bool,
    strict_pixel_format: bool,
) -> tuple[cv2.VideoCapture, str, np.ndarray, dict]:
    open_params = build_open_params(width, height, fps, pixel_format)
    for backend_name, backend_code in BACKENDS[backend]:
        cap = cv2.VideoCapture()
        opened = False
        if open_params:
            try:
                opened = bool(cap.open(camera_index, backend_code, open_params))
            except Exception:
                opened = False
        if not opened:
            cap.release()
            cap = cv2.VideoCapture(camera_index, backend_code)
            opened = cap.isOpened()
            if opened:
                configure_capture(cap, width, height, fps, buffer_size, pixel_format, disable_rgb_convert)
        else:
            if disable_rgb_convert:
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            if buffer_size > 0:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
        if not opened or not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        if ok and frame is not None:
            capture_info = build_capture_info(cap, frame)
            validate_capture_format(capture_info, pixel_format, strict_pixel_format)
            return cap, backend_name, frame, capture_info
        cap.release()

    raise RuntimeError(
        f"Unable to open camera index {camera_index}. "
        "Try --list-cameras, another index, unplug/replug the webcam, or switch --backend."
    )


def list_available_cameras(
    max_camera_index: int,
    backend: str,
    width: int,
    height: int,
    fps: int,
    buffer_size: int,
    pixel_format: str,
    disable_rgb_convert: bool,
) -> None:
    print(f"Scanning camera indices 0..{max_camera_index} using backend set '{backend}'")
    found_any = False
    open_params = build_open_params(width, height, fps, pixel_format)
    for camera_index in range(max_camera_index + 1):
        hits: list[str] = []
        for backend_name, backend_code in BACKENDS[backend]:
            cap = cv2.VideoCapture()
            opened = False
            if open_params:
                try:
                    opened = bool(cap.open(camera_index, backend_code, open_params))
                except Exception:
                    opened = False
            if not opened:
                cap.release()
                cap = cv2.VideoCapture(camera_index, backend_code)
                opened = cap.isOpened()
                if opened:
                    configure_capture(cap, width, height, fps, buffer_size, pixel_format, disable_rgb_convert)
            else:
                if disable_rgb_convert:
                    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
                if buffer_size > 0:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
            if not opened or not cap.isOpened():
                cap.release()
                continue
            ok, frame = cap.read()
            if ok and frame is not None:
                capture_info = build_capture_info(cap, frame)
                hits.append(
                    f"{backend_name}: {capture_info['width']}x{capture_info['height']} "
                    f"@ {capture_info['fps']:.1f} FPS, format={capture_info['pixel_format']}, convert_rgb={capture_info['convert_rgb']:.0f}"
                )
            cap.release()
        if hits:
            found_any = True
            print(f"[camera {camera_index}]")
            for hit in hits:
                print(f"  {hit}")
    if not found_any:
        print("No cameras opened successfully with the requested backend configuration.")


def print_accel_diagnostics() -> None:
    have_opencl = cv2.ocl.haveOpenCL()
    use_opencl = cv2.ocl.useOpenCL()
    cuda_count = cv2.cuda.getCudaEnabledDeviceCount() if hasattr(cv2, "cuda") else 0
    print(f"OpenCV version: {cv2.__version__}")
    print(f"OpenCL available: {have_opencl}")
    print(f"OpenCL enabled: {use_opencl}")
    print(f"CUDA devices: {cuda_count}")
    print("Acceleration note: standard OpenCV AprilTag detection remains CPU-bound.")
    print("OpenCL only helps some image operations such as resize / buffer handling.")


def compute_roi(
    frame_w: int,
    frame_h: int,
    bbox: tuple[int, int, int, int] | None,
    padding: int,
) -> tuple[int, int, int, int] | None:
    if bbox is None:
        return None

    x0, y0, x1, y1 = bbox
    x0 = max(0, x0 - padding)
    y0 = max(0, y0 - padding)
    x1 = min(frame_w, x1 + padding)
    y1 = min(frame_h, y1 + padding)
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def compute_center_crop(frame_w: int, frame_h: int, crop_w: int, crop_h: int) -> tuple[int, int, int, int] | None:
    if crop_w <= 0 or crop_h <= 0:
        return None

    crop_w = min(crop_w, frame_w)
    crop_h = min(crop_h, frame_h)
    x0 = max(0, (frame_w - crop_w) // 2)
    y0 = max(0, (frame_h - crop_h) // 2)
    x1 = x0 + crop_w
    y1 = y0 + crop_h
    return x0, y0, x1, y1


def intersect_roi(
    first: tuple[int, int, int, int] | None,
    second: tuple[int, int, int, int] | None,
) -> tuple[int, int, int, int] | None:
    if first is None:
        return second
    if second is None:
        return first

    x0 = max(first[0], second[0])
    y0 = max(first[1], second[1])
    x1 = min(first[2], second[2])
    y1 = min(first[3], second[3])
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def bbox_from_corners(corners: list[np.ndarray]) -> tuple[int, int, int, int] | None:
    if not corners:
        return None

    points = np.concatenate([corner.reshape(-1, 2) for corner in corners], axis=0)
    x0 = int(np.floor(points[:, 0].min()))
    y0 = int(np.floor(points[:, 1].min()))
    x1 = int(np.ceil(points[:, 0].max()))
    y1 = int(np.ceil(points[:, 1].max()))
    return x0, y0, x1, y1


def resolve_processing_size(frame_w: int, frame_h: int, process_w: int, process_h: int) -> tuple[int, int]:
    if process_w <= 0 and process_h <= 0:
        return frame_w, frame_h

    if process_w > 0 and process_h > 0:
        scale = min(process_w / frame_w, process_h / frame_h)
    elif process_w > 0:
        scale = process_w / frame_w
    else:
        scale = process_h / frame_h

    scale = max(1e-6, min(scale, 1.0))
    if scale >= 0.999:
        return frame_w, frame_h

    return max(1, int(round(frame_w * scale))), max(1, int(round(frame_h * scale)))


def scale_roi(
    roi: tuple[int, int, int, int] | None,
    scale_x: float,
    scale_y: float,
    max_w: int,
    max_h: int,
) -> tuple[int, int, int, int] | None:
    if roi is None:
        return None

    x0 = max(0, min(max_w - 1, int(math.floor(roi[0] * scale_x))))
    y0 = max(0, min(max_h - 1, int(math.floor(roi[1] * scale_y))))
    x1 = max(x0 + 1, min(max_w, int(math.ceil(roi[2] * scale_x))))
    y1 = max(y0 + 1, min(max_h, int(math.ceil(roi[3] * scale_y))))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def scale_corners(corners: list[np.ndarray], scale_x: float, scale_y: float) -> list[np.ndarray]:
    if not corners:
        return []

    scaled: list[np.ndarray] = []
    for corner in corners:
        points = np.asarray(corner, dtype=np.float32).reshape(-1, 2).copy()
        points[:, 0] *= scale_x
        points[:, 1] *= scale_y
        scaled.append(points.reshape(1, 4, 2))
    return scaled


def detect_markers_with_scale(
    detector: cv2.aruco.ArucoDetector,
    gray: np.ndarray,
    detect_scale: float,
    roi: tuple[int, int, int, int] | None = None,
    accel: str = "none",
) -> tuple[list[np.ndarray], np.ndarray | None]:
    offset_x = 0
    offset_y = 0
    working = gray
    if roi is not None:
        x0, y0, x1, y1 = roi
        working = gray[y0:y1, x0:x1]
        offset_x = x0
        offset_y = y0

    if working.size == 0:
        return [], None

    use_opencl = accel == "opencl"
    source: np.ndarray | cv2.UMat = cv2.UMat(working) if use_opencl else working
    if detect_scale != 1.0:
        source = cv2.resize(source, None, fx=detect_scale, fy=detect_scale, interpolation=cv2.INTER_AREA)

    try:
        corners, ids, _ = detector.detectMarkers(source)
    except cv2.error:
        source = working
        if detect_scale != 1.0:
            source = cv2.resize(source, None, fx=detect_scale, fy=detect_scale, interpolation=cv2.INTER_AREA)
        corners, ids, _ = detector.detectMarkers(source)

    corners = normalize_corners(corners)
    ids = normalize_ids(ids)
    if not corners or ids is None or ids.size == 0:
        return [], None

    scaled_h, scaled_w = image_shape(source)
    scale_x = working.shape[1] / scaled_w
    scale_y = working.shape[0] / scaled_h
    restored: list[np.ndarray] = []
    for corner in corners:
        if hasattr(corner, "get"):
            corner = corner.get()
        points = np.asarray(corner).reshape(-1, 2).astype(np.float32)
        points[:, 0] = (points[:, 0] * scale_x) + offset_x
        points[:, 1] = (points[:, 1] * scale_y) + offset_y
        restored.append(points.reshape(1, 4, 2))

    count = min(len(restored), int(ids.size))
    if count <= 0:
        return [], None
    if count != len(restored) or count != int(ids.size):
        print(
            f"Detector returned mismatched data: corners={len(restored)} ids={int(ids.size)}. "
            f"Trimming to {count} matched detections."
        )
    return restored[:count], ids[:count]


def estimate_from_size(
    center_xy: np.ndarray,
    observed_size_px: float,
    tag_size_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float, float]:
    z = (fx * tag_size_m) / max(observed_size_px, 1e-6)
    x = ((float(center_xy[0]) - cx) * z) / fx
    y = ((float(center_xy[1]) - cy) * z) / fy
    return x, y, z


def roi_to_dict(roi: tuple[int, int, int, int] | None) -> dict | None:
    if roi is None:
        return None
    x0, y0, x1, y1 = roi
    return {
        "x": int(x0),
        "y": int(y0),
        "width": int(x1 - x0),
        "height": int(y1 - y0),
    }


def format_roi(roi: tuple[int, int, int, int] | None) -> str:
    if roi is None:
        return "full-frame"
    x0, y0, x1, y1 = roi
    return f"{x0},{y0} {x1 - x0}x{y1 - y0}"


def build_overlay_items(
    corners: list[np.ndarray],
    ids: np.ndarray | None,
    rvecs: np.ndarray | None,
    tvecs: np.ndarray | None,
    tag_size_m: float,
    intrinsics: tuple[float, float, float, float],
) -> tuple[list[dict], list[dict]]:
    detections: list[dict] = []
    overlay_items: list[dict] = []
    if ids is None or ids.size == 0 or not corners:
        return detections, overlay_items

    fx, fy, cx, cy = intrinsics
    count = min(len(corners), int(ids.size))
    if tvecs is not None:
        count = min(count, len(tvecs))
    if rvecs is not None:
        count = min(count, len(rvecs))
    if count <= 0:
        return detections, overlay_items
    if count != len(corners) or count != int(ids.size):
        print(
            f"Overlay inputs mismatched: corners={len(corners)} ids={int(ids.size)} "
            f"rvecs={0 if rvecs is None else len(rvecs)} tvecs={0 if tvecs is None else len(tvecs)}. "
            f"Using first {count} items."
        )

    ids_flat = ids[:count]
    for idx, marker_id in enumerate(ids_flat):
        points = corners[idx].reshape(4, 2).astype(np.float32)
        center = points.mean(axis=0)
        side_lengths = [float(np.linalg.norm(points[i] - points[(i + 1) % 4])) for i in range(4)]
        observed_size_px = float(np.mean(side_lengths))

        if tvecs is not None:
            x_m, y_m, z_m = [float(v) for v in tvecs[idx].reshape(-1)[:3]]
        else:
            x_m, y_m, z_m = estimate_from_size(
                center_xy=center,
                observed_size_px=observed_size_px,
                tag_size_m=tag_size_m,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
            )

        detections.append(
            {
                "id": int(marker_id),
                "center_px": {"x": float(center[0]), "y": float(center[1])},
                "size_px": observed_size_px,
                "position_m": {"x": x_m, "y": y_m, "z": z_m},
                "normalized_xy": {
                    "x": (float(center[0]) - cx) / max(cx, 1.0),
                    "y": (float(center[1]) - cy) / max(cy, 1.0),
                },
            }
        )

        overlay_items.append(
            {
                "id": int(marker_id),
                "points": points,
                "center": center,
                "position_m": (x_m, y_m, z_m),
                "rvec": None if rvecs is None else rvecs[idx],
                "tvec": None if tvecs is None else tvecs[idx],
            }
        )
    return detections, overlay_items


def detect_frame(
    frame: np.ndarray,
    source_seq: int,
    detector: cv2.aruco.ArucoDetector,
    camera_matrix: np.ndarray | None,
    dist_coeffs: np.ndarray | None,
    intrinsics: tuple[float, float, float, float],
    args: argparse.Namespace,
    state: DetectionRuntimeState,
) -> DetectionResult:
    started = time.perf_counter()
    frame_h, frame_w = frame.shape[:2]
    processing_w, processing_h = resolve_processing_size(frame_w, frame_h, args.process_width, args.process_height)
    resize_started = time.perf_counter()
    if processing_w != frame_w or processing_h != frame_h:
        frame_for_detection = cv2.resize(frame, (processing_w, processing_h), interpolation=cv2.INTER_AREA)
    else:
        frame_for_detection = frame
    resize_finished = time.perf_counter()
    grayscale_started = resize_finished
    gray_for_detection = cv2.cvtColor(frame_for_detection, cv2.COLOR_BGR2GRAY)
    grayscale_finished = time.perf_counter()
    roi_scale_x = processing_w / frame_w
    roi_scale_y = processing_h / frame_h
    restore_scale_x = frame_w / processing_w
    restore_scale_y = frame_h / processing_h

    base_roi = compute_center_crop(frame_w, frame_h, args.crop_width, args.crop_height)
    tracked_roi = compute_roi(frame_w, frame_h, state.last_detection_bbox, args.roi_padding) if args.use_roi_tracking else None

    state.detect_cycles += 1
    force_full_frame = (
        state.force_full_frame_search
        or args.full_frame_interval <= 1
        or ((state.detect_cycles - 1) % args.full_frame_interval == 0)
    )
    state.force_full_frame_search = False

    if force_full_frame:
        active_roi = base_roi
        scale_for_search = args.detect_scale
    else:
        active_roi = intersect_roi(base_roi, tracked_roi) if tracked_roi is not None else base_roi
        if tracked_roi is not None and active_roi is None:
            active_roi = base_roi if base_roi is not None else tracked_roi
        scale_for_search = args.roi_detect_scale

    detector_started = time.perf_counter()
    processing_roi = scale_roi(active_roi, roi_scale_x, roi_scale_y, processing_w, processing_h)
    corners, ids = detect_markers_with_scale(detector, gray_for_detection, scale_for_search, roi=processing_roi, accel=args.accel)
    if processing_w != frame_w or processing_h != frame_h:
        corners = scale_corners(corners, restore_scale_x, restore_scale_y)
    if ids is None and not force_full_frame:
        fallback_roi = base_roi
        fallback_processing_roi = scale_roi(fallback_roi, roi_scale_x, roi_scale_y, processing_w, processing_h)
        corners, ids = detect_markers_with_scale(detector, gray_for_detection, args.detect_scale, roi=fallback_processing_roi, accel=args.accel)
        if processing_w != frame_w or processing_h != frame_h:
            corners = scale_corners(corners, restore_scale_x, restore_scale_y)
        if ids is None and fallback_roi is not None:
            corners, ids = detect_markers_with_scale(detector, gray_for_detection, args.detect_scale, roi=None, accel=args.accel)
            if processing_w != frame_w or processing_h != frame_h:
                corners = scale_corners(corners, restore_scale_x, restore_scale_y)
            active_roi = None
        else:
            active_roi = fallback_roi
    detector_finished = time.perf_counter()

    rvecs = None
    tvecs = None
    pose_started = time.perf_counter()
    if ids is not None and len(ids) > 0 and camera_matrix is not None and dist_coeffs is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            args.tag_size_m,
            camera_matrix,
            dist_coeffs,
        )
    pose_finished = time.perf_counter()

    detections, overlay_items = build_overlay_items(corners, ids, rvecs, tvecs, args.tag_size_m, intrinsics)
    state.last_detection_bbox = bbox_from_corners(corners)
    finished = time.perf_counter()

    return DetectionResult(
        source_seq=source_seq,
        processed_at=finished,
        detection_latency_ms=(finished - started) * 1000.0,
        resize_latency_ms=(resize_finished - resize_started) * 1000.0,
        grayscale_latency_ms=(grayscale_finished - grayscale_started) * 1000.0,
        detector_latency_ms=(detector_finished - detector_started) * 1000.0,
        pose_latency_ms=(pose_finished - pose_started) * 1000.0,
        detections=detections,
        overlay_items=overlay_items,
        base_roi=base_roi,
        tracked_roi=tracked_roi,
        active_roi=active_roi,
        force_full_frame=force_full_frame,
        scale_for_search=scale_for_search,
        frame_width=frame_w,
        frame_height=frame_h,
        processing_width=processing_w,
        processing_height=processing_h,
    )


def annotate_frame(
    frame: np.ndarray,
    result: DetectionResult | None,
    args: argparse.Namespace,
    mode_name: str,
    display_fps: float,
    detect_fps: float,
    camera_matrix: np.ndarray | None,
    dist_coeffs: np.ndarray | None,
) -> np.ndarray:
    if result is not None:
        for item in result.overlay_items:
            points = item["points"]
            center = item["center"]
            x_m, y_m, z_m = item["position_m"]
            pts_int = points.astype(np.int32)
            cv2.polylines(frame, [pts_int], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.circle(frame, tuple(np.round(center).astype(int)), 4, (0, 0, 255), -1)

            text_lines = [
                f"ID {item['id']}",
                f"X {x_m:+.3f} m  Y {y_m:+.3f} m",
                f"Z {z_m:.3f} m",
            ]
            text_x = int(points[0][0])
            text_y = int(points[0][1]) - 12
            for line_no, text in enumerate(text_lines):
                y = text_y + line_no * 22
                cv2.putText(
                    frame,
                    text,
                    (text_x, max(20, y)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 180, 255),
                    2,
                    cv2.LINE_AA,
                )

            if args.draw_axes and camera_matrix is not None and dist_coeffs is not None and item["rvec"] is not None and item["tvec"] is not None:
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    item["rvec"],
                    item["tvec"],
                    args.tag_size_m * 0.5,
                    2,
                )

        if result.base_roi is not None:
            x0, y0, x1, y1 = result.base_roi
            cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 0), 1)
            cv2.putText(
                frame,
                f"center crop {x1 - x0}x{y1 - y0}",
                (x0, max(20, y0 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )

        if args.show_roi_debug and result.tracked_roi is not None:
            x0, y0, x1, y1 = result.tracked_roi
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 200, 255), 1)
            cv2.putText(
                frame,
                "tracked ROI",
                (x0, min(frame.shape[0] - 10, y1 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 200, 255),
                1,
                cv2.LINE_AA,
            )

        if args.show_roi_debug and result.active_roi is not None:
            x0, y0, x1, y1 = result.active_roi
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 120, 255), 1)
            cv2.putText(
                frame,
                "active ROI",
                (x0, min(frame.shape[0] - 30, y1 + 36)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 120, 255),
                1,
                cv2.LINE_AA,
            )

    age_ms = 0.0 if result is None else (time.perf_counter() - result.processed_at) * 1000.0
    detections_count = 0 if result is None else len(result.detections)
    roi_desc = "full-frame" if result is None else format_roi(result.active_roi)
    detect_scale = args.detect_scale if result is None else result.scale_for_search
    latency_ms = 0.0 if result is None else result.detection_latency_ms
    processing_desc = (
        f"{frame.shape[1]}x{frame.shape[0]}"
        if result is None
        else f"{result.processing_width}x{result.processing_height}"
    )

    top_line = (
        f"AprilTag {args.family} | detections: {detections_count} | display: {display_fps:.1f} FPS "
        f"| detect: {detect_fps:.1f} Hz | result age: {age_ms:.0f} ms"
    )
    bottom_line = (
        f"{mode_name} | profile: {args.profile} | accel: {args.accel} | "
        f"process: {processing_desc} | detect_scale: {detect_scale:.2f} | roi: {roi_desc} | latency: {latency_ms:.1f} ms"
    )

    cv2.putText(frame, top_line, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 180, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, bottom_line, (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(
        frame,
        "Hotkeys: q=quit s=save r=reset-roi f=force-full-frame p=print-status h=help",
        (20, frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    return frame


def build_snapshot_payload(
    args: argparse.Namespace,
    backend_name: str,
    capture_info: dict,
    mode_name: str,
    display_fps: float,
    detect_fps: float,
    result: DetectionResult,
) -> dict:
    return {
        "timestamp_unix": time.time(),
        "camera": {
            "index": args.camera_index,
            "backend": backend_name,
            "width": capture_info["width"],
            "height": capture_info["height"],
            "fps": capture_info["fps"],
            "pixel_format": capture_info["pixel_format"],
            "buffer_size": capture_info["buffer_size"],
        },
        "tag_family": args.family,
        "tag_size_m": args.tag_size_m,
        "estimation_mode": mode_name,
        "performance_profile": args.profile,
        "detect_scale": args.detect_scale,
        "roi_detect_scale": args.roi_detect_scale,
        "full_frame_interval": args.full_frame_interval,
        "max_detect_hz": args.max_detect_hz,
        "refine": args.refine,
        "roi_tracking": args.use_roi_tracking,
        "accel": args.accel,
        "async_capture": args.async_capture,
        "processing_frame": {
            "width": result.processing_width,
            "height": result.processing_height,
        },
        "center_crop": {
            "width": args.crop_width,
            "height": args.crop_height,
        },
        "timing": {
            "display_fps": display_fps,
            "detect_hz": detect_fps,
            "resize_ms": result.resize_latency_ms,
            "grayscale_ms": result.grayscale_latency_ms,
            "detector_ms": result.detector_latency_ms,
            "pose_ms": result.pose_latency_ms,
            "detection_latency_ms": result.detection_latency_ms,
            "result_age_ms": (time.perf_counter() - result.processed_at) * 1000.0,
        },
        "roi": {
            "base": roi_to_dict(result.base_roi),
            "tracked": roi_to_dict(result.tracked_roi),
            "active": roi_to_dict(result.active_roi),
            "force_full_frame": result.force_full_frame,
        },
        "detections": result.detections,
    }


def print_status(
    args: argparse.Namespace,
    backend_name: str,
    capture_info: dict,
    mode_name: str,
    display_fps: float,
    detect_fps: float,
    result: DetectionResult | None,
) -> None:
    processing_w, processing_h = resolve_processing_size(
        capture_info["width"],
        capture_info["height"],
        args.process_width,
        args.process_height,
    )
    print(
        f"camera={args.camera_index} backend={backend_name} size={capture_info['width']}x{capture_info['height']} "
        f"fps={capture_info['fps']:.1f} format={capture_info['pixel_format']} buffer={capture_info['buffer_size']}"
    )
    print(
        f"profile={args.profile} mode={mode_name} accel={args.accel} async_capture={args.async_capture} "
        f"process={processing_w}x{processing_h} detect_scale={args.detect_scale:.2f} roi_detect_scale={args.roi_detect_scale:.2f} "
        f"max_detect_hz={args.max_detect_hz:.1f} full_frame_interval={args.full_frame_interval}"
    )
    if result is None:
        print("no detection result available yet")
        return
    print(
        f"detections={len(result.detections)} display_fps={display_fps:.1f} detect_hz={detect_fps:.1f} "
        f"latency_ms={result.detection_latency_ms:.1f} roi={format_roi(result.active_roi)} "
        f"processing={result.processing_width}x{result.processing_height} "
        f"resize={result.resize_latency_ms:.1f}ms gray={result.grayscale_latency_ms:.1f}ms "
        f"detect={result.detector_latency_ms:.1f}ms pose={result.pose_latency_ms:.1f}ms"
    )


def print_help_hotkeys() -> None:
    print("Hotkeys:")
    print("  q  quit")
    print("  s  save current annotated frame")
    print("  r  clear tracked ROI and force reacquire")
    print("  f  force next detection to search the whole frame")
    print("  p  print current runtime status")
    print("  h  print hotkey help")


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    default_config_path = project_root / "config" / "apriltag_detector.toml"
    bootstrap_parser = argparse.ArgumentParser(add_help=False)
    bootstrap_parser.add_argument("--config", type=Path, default=default_config_path)
    bootstrap_args, _ = bootstrap_parser.parse_known_args()
    bootstrap_config_path = bootstrap_args.config.expanduser()
    if not bootstrap_config_path.is_absolute():
        bootstrap_config_path = Path.cwd() / bootstrap_config_path
    config_defaults = load_config_defaults(bootstrap_config_path.resolve(), project_root)
    parser = build_arg_parser(project_root, config_defaults, bootstrap_config_path.resolve())
    args = resolve_profile(parser.parse_args())
    args.config = args.config.expanduser()
    if not args.config.is_absolute():
        args.config = (Path.cwd() / args.config).resolve()

    if args.list_cameras:
        list_available_cameras(
            max_camera_index=args.max_camera_index,
            backend=args.backend,
            width=args.width,
            height=args.height,
            fps=args.fps,
            buffer_size=args.buffer_size,
            pixel_format=args.pixel_format,
            disable_rgb_convert=args.disable_rgb_convert,
        )
        return

    if args.benchmark_capture:
        run_capture_benchmark(args)
        return

    if args.diagnose_accel:
        print_accel_diagnostics()
        return

    if args.async_capture is None:
        args.async_capture = args.profile in {"speed", "mp257"}
    if args.accel == "auto":
        args.accel = "opencl" if (cv2.ocl.haveOpenCL() and args.profile in {"accuracy", "balanced"}) else "none"
    if args.accel == "opencl":
        cv2.ocl.setUseOpenCL(True)
        if not cv2.ocl.useOpenCL():
            print("OpenCL was requested but is not active. Falling back to CPU-only image ops.")
            args.accel = "none"

    camera_matrix, dist_coeffs = load_calibration(args.calibration_file)
    mode_name = "calibrated_pose" if camera_matrix is not None else "size_based_approximation"

    dictionary = cv2.aruco.getPredefinedDictionary(FAMILY_TO_DICT[args.family])
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = REFINE_METHODS[args.refine]
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    cap, backend_name, first_frame, capture_info = open_camera(
        args.camera_index,
        args.width,
        args.height,
        args.fps,
        args.backend,
        args.buffer_size,
        args.pixel_format,
        args.disable_rgb_convert,
        args.strict_pixel_format,
    )
    intrinsics = derive_intrinsics(
        frame_width=int(first_frame.shape[1]),
        frame_height=int(first_frame.shape[0]),
        camera_matrix=camera_matrix,
        focal_length_px=args.focal_length_px,
        horizontal_fov_deg=args.horizontal_fov_deg,
    )

    frame_reader = None
    if args.async_capture:
        frame_reader = LatestFrameReader(cap)
        frame_reader.start()

    snapshot_writer = None
    if args.snapshot_hz > 0:
        snapshot_writer = SnapshotWriter(args.snapshot_file, pretty=args.snapshot_pretty)
        snapshot_writer.start()

    startup_payload = {
        "config_file": str(args.config),
        "camera_index": args.camera_index,
        "backend": backend_name,
        "profile": args.profile,
        "mode": mode_name,
        "requested_size": {"width": args.width, "height": args.height, "fps": args.fps},
        "actual_capture": capture_info,
        "processing_size": {
            "width": resolve_processing_size(int(first_frame.shape[1]), int(first_frame.shape[0]), args.process_width, args.process_height)[0],
            "height": resolve_processing_size(int(first_frame.shape[1]), int(first_frame.shape[0]), args.process_width, args.process_height)[1],
        },
        "pixel_format_request": args.pixel_format,
        "detect_scale": args.detect_scale,
        "roi_detect_scale": args.roi_detect_scale,
        "max_detect_hz": args.max_detect_hz,
        "full_frame_interval": args.full_frame_interval,
        "roi_tracking": args.use_roi_tracking,
        "accel": args.accel,
        "async_capture": args.async_capture,
        "snapshot_hz": args.snapshot_hz,
        "snapshot_file": str(args.snapshot_file),
        "calibration_file": str(args.calibration_file),
    }

    processing_w, processing_h = resolve_processing_size(int(first_frame.shape[1]), int(first_frame.shape[0]), args.process_width, args.process_height)
    print(f"Opened camera {args.camera_index} with backend: {backend_name}")
    print(f"Config file: {args.config}")
    print(
        f"Capture: {capture_info['width']}x{capture_info['height']} @ {capture_info['fps']:.1f} FPS "
        f"| format={capture_info['pixel_format']} | convert_rgb={capture_info['convert_rgb']:.0f} | buffer={capture_info['buffer_size']}"
    )
    print(
        f"Mode: {mode_name} | profile={args.profile} | accel={args.accel} | async_capture={args.async_capture} "
        f"| process={processing_w}x{processing_h} | detect_scale={args.detect_scale:.2f} | roi_detect_scale={args.roi_detect_scale:.2f} "
        f"| max_detect_hz={args.max_detect_hz:.1f} | full_frame_interval={args.full_frame_interval}"
    )
    if args.crop_width > 0 and args.crop_height > 0:
        print(f"Center crop enabled: {args.crop_width}x{args.crop_height}")
    if processing_w != int(first_frame.shape[1]) or processing_h != int(first_frame.shape[0]):
        print("Software full-frame downsample enabled: field of view is preserved while detection runs on fewer pixels.")
    if args.snapshot_hz <= 0:
        print("Snapshot writing: disabled")
    else:
        print(f"Snapshot writing: {args.snapshot_hz:.1f} Hz -> {args.snapshot_file}")
    print_help_hotkeys()
    if args.print_config:
        print(json.dumps(startup_payload, ensure_ascii=False, indent=2))

    runtime_state = DetectionRuntimeState()
    start_time = time.perf_counter()
    frame_count = 0
    detect_count = 0
    source_seq = 1
    current_frame = first_frame
    current_seq = source_seq
    last_reader_seq = 0
    last_detect_finish = 0.0
    last_console_update = 0.0
    last_snapshot_update = 0.0
    no_frame_loops = 0
    last_result: DetectionResult | None = None
    telemetry = RuntimeTelemetry()

    try:
        while True:
            capture_wait_started = time.perf_counter()
            if current_frame is None:
                if frame_reader is not None:
                    ok, frame, seq, fail_count = frame_reader.read()
                    if frame is None:
                        no_frame_loops += 1
                        if fail_count > 30:
                            print("Camera frame grab failed, stopping.")
                            break
                        time.sleep(0.005)
                        continue
                    if seq == last_reader_seq:
                        no_frame_loops += 1
                        if no_frame_loops > 50:
                            time.sleep(0.002)
                        continue
                    last_reader_seq = seq
                    no_frame_loops = 0
                    current_seq = seq
                    ok = True
                else:
                    ok, frame = cap.read()
                    if ok:
                        source_seq += 1
                        current_seq = source_seq
                if not ok or frame is None:
                    print("Camera frame grab failed, stopping.")
                    break
            else:
                frame = current_frame
                current_frame = None

            frame_count += 1
            now = time.perf_counter()
            elapsed = now - start_time
            display_fps = frame_count / max(elapsed, 1e-6)
            capture_wait_ms = (now - capture_wait_started) * 1000.0

            if frame.shape[1] != capture_info["width"] or frame.shape[0] != capture_info["height"]:
                capture_info["width"] = int(frame.shape[1])
                capture_info["height"] = int(frame.shape[0])
                intrinsics = derive_intrinsics(
                    frame_width=int(frame.shape[1]),
                    frame_height=int(frame.shape[0]),
                    camera_matrix=camera_matrix,
                    focal_length_px=args.focal_length_px,
                    horizontal_fov_deg=args.horizontal_fov_deg,
                )

            detect_period = 0.0 if args.max_detect_hz <= 0 else 1.0 / args.max_detect_hz
            should_detect = (
                last_result is None
                or detect_period == 0.0
                or (now - last_detect_finish) >= detect_period
            )

            if should_detect:
                last_result = detect_frame(
                    frame=frame,
                    source_seq=current_seq,
                    detector=detector,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                    intrinsics=intrinsics,
                    args=args,
                    state=runtime_state,
                )
                last_detect_finish = last_result.processed_at
                detect_count += 1
                telemetry.add_detect_sample(last_result)

            detect_elapsed = max((last_detect_finish or now) - start_time, 1e-6)
            detect_fps = detect_count / detect_elapsed

            if snapshot_writer is not None and last_result is not None:
                snapshot_period = 1.0 / args.snapshot_hz
                if (now - last_snapshot_update) >= snapshot_period:
                    snapshot_writer.submit(
                        build_snapshot_payload(
                            args=args,
                            backend_name=backend_name,
                            capture_info=capture_info,
                            mode_name=mode_name,
                            display_fps=display_fps,
                            detect_fps=detect_fps,
                            result=last_result,
                        )
                    )
                    last_snapshot_update = now

            if now - last_console_update >= (1.0 / args.status_hz):
                detections_count = 0 if last_result is None else len(last_result.detections)
                latency_ms = 0.0 if last_result is None else last_result.detection_latency_ms
                age_ms = 0.0 if last_result is None else (now - last_result.processed_at) * 1000.0
                summary = telemetry.summarize()
                print(
                    f"display_fps={display_fps:.1f} detect_hz={detect_fps:.1f} detections={detections_count} "
                    f"latency_ms={latency_ms:.1f} age_ms={age_ms:.0f} "
                    f"capture_wait={summary['capture_wait_ms']:.1f}ms resize={summary['resize_ms']:.1f}ms "
                    f"gray={summary['grayscale_ms']:.1f}ms detect={summary['detector_ms']:.1f}ms "
                    f"pose={summary['pose_ms']:.1f}ms gui={summary['gui_ms']:.1f}ms"
                )
                last_console_update = now
                telemetry.reset()

            key = -1
            gui_started = time.perf_counter()
            if not args.no_gui:
                display = annotate_frame(
                    frame.copy(),
                    result=last_result,
                    args=args,
                    mode_name=mode_name,
                    display_fps=display_fps,
                    detect_fps=detect_fps,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                )
                if args.display_scale != 1.0:
                    display = cv2.resize(
                        display,
                        None,
                        fx=args.display_scale,
                        fy=args.display_scale,
                        interpolation=cv2.INTER_AREA,
                    )
                cv2.imshow("AprilTag USB Detector", display)
                key = cv2.waitKey(1) & 0xFF
            gui_finished = time.perf_counter()
            telemetry.add_frame_sample(
                capture_wait_ms=capture_wait_ms,
                gui_ms=(gui_finished - gui_started) * 1000.0,
            )

            if key == ord("q"):
                break
            if key == ord("s"):
                args.capture_dir.mkdir(parents=True, exist_ok=True)
                image_path = args.capture_dir / f"apriltag_capture_{int(time.time())}.png"
                frame_to_save = frame if args.no_gui else annotate_frame(
                    frame.copy(),
                    result=last_result,
                    args=args,
                    mode_name=mode_name,
                    display_fps=display_fps,
                    detect_fps=detect_fps,
                    camera_matrix=camera_matrix,
                    dist_coeffs=dist_coeffs,
                )
                cv2.imwrite(str(image_path), frame_to_save)
                print(f"Saved frame: {image_path}")
            if key == ord("r"):
                runtime_state.last_detection_bbox = None
                runtime_state.force_full_frame_search = True
                print("ROI tracking reset. Next detection will reacquire on the whole frame.")
            if key == ord("f"):
                runtime_state.force_full_frame_search = True
                print("Forced next detection to search the whole frame.")
            if key == ord("p"):
                print_status(
                    args=args,
                    backend_name=backend_name,
                    capture_info=capture_info,
                    mode_name=mode_name,
                    display_fps=display_fps,
                    detect_fps=detect_fps,
                    result=last_result,
                )
            if key == ord("h"):
                print_help_hotkeys()

            if args.duration > 0 and elapsed >= args.duration:
                break
    finally:
        if frame_reader is not None:
            frame_reader.stop()
        if snapshot_writer is not None:
            snapshot_writer.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
