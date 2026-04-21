from __future__ import annotations

import argparse
import json
import math
import threading
import time
from pathlib import Path

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

PROFILE_DEFAULTS = {
    "accuracy": {
        "detect_scale": 1.0,
        "refine": "apriltag",
        "snapshot_hz": 10.0,
        "draw_axes": True,
        "use_roi_tracking": False,
        "roi_padding": 96,
    },
    "balanced": {
        "detect_scale": 0.75,
        "refine": "subpix",
        "snapshot_hz": 5.0,
        "draw_axes": True,
        "use_roi_tracking": True,
        "roi_padding": 96,
    },
    "speed": {
        "detect_scale": 0.5,
        "refine": "none",
        "snapshot_hz": 2.0,
        "draw_axes": False,
        "use_roi_tracking": True,
        "roi_padding": 72,
    },
    "mp257": {
        "detect_scale": 0.5,
        "refine": "none",
        "snapshot_hz": 2.0,
        "draw_axes": False,
        "use_roi_tracking": True,
        "roi_padding": 72,
    },
}

ACCEL_BACKENDS = ("auto", "none", "opencl")


def load_calibration(path: Path | None) -> tuple[np.ndarray | None, np.ndarray | None]:
    if path is None or not path.exists():
        return None, None
    data = json.loads(path.read_text(encoding="utf-8"))
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def resolve_profile(args: argparse.Namespace) -> argparse.Namespace:
    defaults = PROFILE_DEFAULTS[args.profile].copy()
    for key in ("detect_scale", "refine", "snapshot_hz", "draw_axes", "use_roi_tracking", "roi_padding"):
        value = getattr(args, key)
        if value is not None:
            defaults[key] = value

    args.detect_scale = max(0.1, min(float(defaults["detect_scale"]), 1.0))
    args.refine = str(defaults["refine"])
    args.snapshot_hz = float(defaults["snapshot_hz"])
    args.draw_axes = bool(defaults["draw_axes"])
    args.use_roi_tracking = bool(defaults["use_roi_tracking"])
    args.roi_padding = int(defaults["roi_padding"])
    return args


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
    ids = np.asarray(ids)
    if ids.ndim == 0:
        ids = ids.reshape(1)
    return ids


def image_shape(image: np.ndarray | cv2.UMat) -> tuple[int, int]:
    if hasattr(image, "get"):
        image = image.get()
    return int(image.shape[0]), int(image.shape[1])


def open_camera(camera_index: int, width: int, height: int, fps: int, backend: str):
    for backend_name, backend_code in BACKENDS[backend]:
        cap = cv2.VideoCapture(camera_index, backend_code)
        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)

        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, backend_name, frame
        cap.release()

    raise RuntimeError(
        f"Unable to open camera index {camera_index}. "
        "Try another index, unplug/replug the webcam, or switch --backend."
    )


def compute_roi(frame_w: int, frame_h: int, bbox: tuple[int, int, int, int] | None, padding: int) -> tuple[int, int, int, int] | None:
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

    if accel == "opencl":
        source = cv2.UMat(working)
    else:
        source = working

    if detect_scale != 1.0:
        scaled = cv2.resize(
            source,
            None,
            fx=detect_scale,
            fy=detect_scale,
            interpolation=cv2.INTER_AREA,
        )
    else:
        scaled = source

    corners, ids, _ = detector.detectMarkers(scaled)
    ids = normalize_ids(ids)
    if ids is None or ids.size == 0:
        return [], None

    scaled_h, scaled_w = image_shape(scaled)
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
    return restored, ids


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


def write_snapshot(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="USB camera AprilTag detection and distance estimation.")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="auto")
    parser.add_argument("--profile", choices=sorted(PROFILE_DEFAULTS), default="balanced")
    parser.add_argument("--accel", choices=ACCEL_BACKENDS, default="auto")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--family", choices=sorted(FAMILY_TO_DICT), default="tag36h11")
    parser.add_argument("--tag-size-m", type=float, default=0.08, help="Measured black square edge size")
    parser.add_argument("--horizontal-fov-deg", type=float, default=70.0)
    parser.add_argument("--focal-length-px", type=float, default=None)
    parser.add_argument("--calibration-file", type=Path, default=project_root / "calibration" / "camera_intrinsics.json")
    parser.add_argument("--display-scale", type=float, default=0.75)
    parser.add_argument("--snapshot-file", type=Path, default=project_root / "output" / "apriltag_latest.json")
    parser.add_argument("--capture-dir", type=Path, default=project_root / "output" / "captures")
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until q is pressed")
    parser.add_argument("--detect-scale", type=float, default=None, help="Detection downscale factor in (0,1], lower is faster")
    parser.add_argument("--refine", choices=sorted(REFINE_METHODS), default=None)
    parser.add_argument("--snapshot-hz", type=float, default=None, help="How many times per second to write JSON snapshots")
    parser.add_argument("--roi-padding", type=int, default=None)
    parser.add_argument("--crop-width", type=int, default=0, help="Center crop width for detection ROI; 0 disables")
    parser.add_argument("--crop-height", type=int, default=0, help="Center crop height for detection ROI; 0 disables")
    parser.add_argument("--roi-detect-scale", type=float, default=1.0, help="Detection scale used when a ROI is already known")
    parser.add_argument("--full-frame-interval", type=int, default=1, help="Force a whole-frame search every N frames")
    parser.set_defaults(draw_axes=None, use_roi_tracking=None)
    parser.set_defaults(async_capture=None)
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
    args = resolve_profile(parser.parse_args())
    if args.async_capture is None:
        args.async_capture = args.profile in {"speed", "mp257"}
    if args.accel == "auto":
        args.accel = "opencl" if (cv2.ocl.haveOpenCL() and args.profile in {"accuracy", "balanced"}) else "none"
    if args.accel == "opencl":
        cv2.ocl.setUseOpenCL(True)

    camera_matrix, dist_coeffs = load_calibration(args.calibration_file)
    mode_name = "calibrated_pose" if camera_matrix is not None else "size_based_approximation"

    dictionary = cv2.aruco.getPredefinedDictionary(FAMILY_TO_DICT[args.family])
    detector_params = cv2.aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = REFINE_METHODS[args.refine]
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    cap, backend_name, first_frame = open_camera(args.camera_index, args.width, args.height, args.fps, args.backend)
    frame_reader = None
    if args.async_capture:
        frame_reader = LatestFrameReader(cap)
        frame_reader.start()
    print(f"Opened camera {args.camera_index} with backend: {backend_name}")
    print(f"Estimation mode: {mode_name}")
    print(f"Performance profile: {args.profile} | detect_scale={args.detect_scale:.2f} | roi_detect_scale={args.roi_detect_scale:.2f} | refine={args.refine} | roi_tracking={args.use_roi_tracking} | accel={args.accel} | async_capture={args.async_capture}")
    if args.crop_width > 0 and args.crop_height > 0:
        print(f"Center crop enabled: {args.crop_width}x{args.crop_height}")
    print("Hotkeys: q=quit, s=save annotated frame")

    start_time = time.perf_counter()
    frame_count = 0
    last_console_update = 0.0
    last_snapshot_update = 0.0
    current_frame = first_frame
    last_detection_bbox: tuple[int, int, int, int] | None = None
    last_seq = 0
    no_frame_loops = 0

    try:
        while True:
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
                    if seq == last_seq:
                        no_frame_loops += 1
                        if no_frame_loops > 200:
                            time.sleep(0.002)
                        continue
                    last_seq = seq
                    no_frame_loops = 0
                    ok = True
                else:
                    ok, frame = cap.read()
                if not ok or frame is None:
                    print("Camera frame grab failed, stopping.")
                    break
            else:
                frame = current_frame
                current_frame = None

            frame_count += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = []

            frame_h, frame_w = frame.shape[:2]
            fx, fy, cx, cy = derive_intrinsics(
                frame_width=frame_w,
                frame_height=frame_h,
                camera_matrix=camera_matrix,
                focal_length_px=args.focal_length_px,
                horizontal_fov_deg=args.horizontal_fov_deg,
            )

            base_roi = compute_center_crop(frame_w, frame_h, args.crop_width, args.crop_height)
            tracked_roi = compute_roi(frame_w, frame_h, last_detection_bbox, args.roi_padding) if args.use_roi_tracking else None
            force_full_frame = args.full_frame_interval <= 1 or ((frame_count - 1) % args.full_frame_interval == 0)
            if force_full_frame:
                roi = base_roi
                scale_for_search = args.detect_scale
            else:
                roi = intersect_roi(base_roi, tracked_roi) if tracked_roi is not None else base_roi
                if tracked_roi is not None and roi is None:
                    roi = base_roi if base_roi is not None else tracked_roi
                scale_for_search = max(0.1, min(float(args.roi_detect_scale), 1.0))

            corners, ids = detect_markers_with_scale(detector, gray, scale_for_search, roi=roi, accel=args.accel)
            if ids is None and not force_full_frame:
                fallback_roi = base_roi
                corners, ids = detect_markers_with_scale(detector, gray, args.detect_scale, roi=fallback_roi, accel=args.accel)
                if ids is None and fallback_roi is not None:
                    corners, ids = detect_markers_with_scale(detector, gray, args.detect_scale, roi=None, accel=args.accel)

            rvecs = None
            tvecs = None
            if ids is not None and len(ids) > 0 and camera_matrix is not None and dist_coeffs is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    args.tag_size_m,
                    camera_matrix,
                    dist_coeffs,
                )

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten()
                for idx, marker_id in enumerate(ids_flat):
                    points = corners[idx].reshape(4, 2).astype(np.float32)
                    center = points.mean(axis=0)
                    side_lengths = [
                        float(np.linalg.norm(points[i] - points[(i + 1) % 4]))
                        for i in range(4)
                    ]
                    observed_size_px = float(np.mean(side_lengths))

                    if tvecs is not None:
                        x_m, y_m, z_m = [float(v) for v in tvecs[idx].reshape(-1)[:3]]
                    else:
                        x_m, y_m, z_m = estimate_from_size(
                            center_xy=center,
                            observed_size_px=observed_size_px,
                            tag_size_m=args.tag_size_m,
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

                    pts_int = points.astype(np.int32)
                    cv2.polylines(frame, [pts_int], isClosed=True, color=(0, 255, 0), thickness=2)
                    cv2.circle(frame, tuple(np.round(center).astype(int)), 4, (0, 0, 255), -1)

                    text_lines = [
                        f"ID {int(marker_id)}",
                        f"X {x_m:+.3f} m  Y {y_m:+.3f} m",
                        f"Z {z_m:.3f} m  size {observed_size_px:.1f}px",
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

                    if args.draw_axes and camera_matrix is not None and dist_coeffs is not None and rvecs is not None and tvecs is not None:
                        cv2.drawFrameAxes(
                            frame,
                            camera_matrix,
                            dist_coeffs,
                            rvecs[idx],
                            tvecs[idx],
                            args.tag_size_m * 0.5,
                            2,
                        )

            last_detection_bbox = bbox_from_corners(corners)

            now = time.perf_counter()
            elapsed = now - start_time
            fps_value = frame_count / max(elapsed, 1e-6)

            if base_roi is not None:
                x0, y0, x1, y1 = base_roi
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

            cv2.putText(
                frame,
                f"AprilTag {args.family} | detections: {len(detections)} | FPS: {fps_value:.1f} | mode: {mode_name} | profile: {args.profile}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 180, 0),
                2,
                cv2.LINE_AA,
            )

            snapshot_period = 0.0 if args.snapshot_hz <= 0 else 1.0 / args.snapshot_hz
            if snapshot_period == 0.0 or (now - last_snapshot_update) >= snapshot_period:
                write_snapshot(
                    args.snapshot_file,
                    {
                        "timestamp_unix": time.time(),
                        "camera": {
                            "index": args.camera_index,
                            "backend": backend_name,
                            "width": frame_w,
                            "height": frame_h,
                        },
                        "tag_family": args.family,
                        "tag_size_m": args.tag_size_m,
                        "estimation_mode": mode_name,
                        "performance_profile": args.profile,
                        "detect_scale": args.detect_scale,
                        "refine": args.refine,
                        "roi_tracking": args.use_roi_tracking,
                        "roi_detect_scale": args.roi_detect_scale,
                        "full_frame_interval": args.full_frame_interval,
                        "accel": args.accel,
                        "async_capture": args.async_capture,
                        "center_crop": {
                            "width": args.crop_width,
                            "height": args.crop_height,
                        },
                        "fps": fps_value,
                        "detections": detections,
                    },
                )
                last_snapshot_update = now

            if now - last_console_update >= 1.0:
                print(f"fps={fps_value:.1f} detections={len(detections)} mode={mode_name}")
                last_console_update = now

            key = -1
            if not args.no_gui:
                display = frame
                if args.display_scale != 1.0:
                    display = cv2.resize(
                        frame,
                        None,
                        fx=args.display_scale,
                        fy=args.display_scale,
                        interpolation=cv2.INTER_AREA,
                    )
                cv2.imshow("AprilTag USB Detector", display)
                key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key == ord("s"):
                args.capture_dir.mkdir(parents=True, exist_ok=True)
                image_path = args.capture_dir / f"apriltag_capture_{int(time.time())}.png"
                cv2.imwrite(str(image_path), frame)
                print(f"Saved frame: {image_path}")

            if args.duration > 0 and elapsed >= args.duration:
                break
    finally:
        if frame_reader is not None:
            frame_reader.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
