from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


BACKENDS = {
    "auto": [("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF), ("ANY", cv2.CAP_ANY)],
    "dshow": [("DSHOW", cv2.CAP_DSHOW)],
    "msmf": [("MSMF", cv2.CAP_MSMF)],
    "any": [("ANY", cv2.CAP_ANY)],
}


def open_camera(camera_index: int, width: int, height: int, fps: int, backend: str):
    for backend_name, backend_code in BACKENDS[backend]:
        cap = cv2.VideoCapture(camera_index, backend_code)
        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, backend_name
        cap.release()

    raise RuntimeError("Unable to open camera for fisheye calibration.")


def make_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    objp = np.zeros((1, rows * cols, 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m
    return objp


def find_corners(gray: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray | None]:
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
        if found:
            return True, corners.astype(np.float64)

    found, corners = cv2.findChessboardCorners(
        gray,
        pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return False, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.0001)
    refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return True, refined.astype(np.float64)


def calibrate_fisheye(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    check_condition: bool,
) -> tuple[float, np.ndarray, np.ndarray, list[np.ndarray], list[np.ndarray]]:
    k = np.zeros((3, 3), dtype=np.float64)
    d = np.zeros((4, 1), dtype=np.float64)
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in object_points]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in object_points]

    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
    if check_condition:
        flags += cv2.fisheye.CALIB_CHECK_COND

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image_size,
        k,
        d,
        rvecs,
        tvecs,
        flags,
        criteria,
    )
    return rms, k, d, rvecs, tvecs


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Live OpenCV fisheye calibration for the YOLO/object USB camera.")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="auto")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--cols", type=int, default=9, help="Internal chessboard corners per row")
    parser.add_argument("--rows", type=int, default=6, help="Internal chessboard corners per column")
    parser.add_argument("--square-size-m", type=float, default=0.025, help="Checkerboard square edge size")
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--no-check-cond", action="store_true", help="Disable cv2.fisheye.CALIB_CHECK_COND")
    parser.add_argument(
        "--output-file",
        type=Path,
        default=project_root / "calibration" / "yolo_fisheye_camera_intrinsics.json",
    )
    parser.add_argument(
        "--capture-dir",
        type=Path,
        default=project_root / "calibration" / "captures_yolo_fisheye",
    )
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)
    objp = make_object_points(args.cols, args.rows, args.square_size_m)

    cap, backend_name = open_camera(args.camera_index, args.width, args.height, args.fps, args.backend)
    print(f"Opened YOLO/fisheye camera {args.camera_index} with backend: {backend_name}")
    print("Hotkeys: space=save a valid checkerboard sample, c=calibrate, q=quit")
    print("Move the checkerboard through center, edges, corners, tilt angles, and different distances.")

    image_points: list[np.ndarray] = []
    object_points: list[np.ndarray] = []
    image_size = None
    args.capture_dir.mkdir(parents=True, exist_ok=True)

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Camera frame grab failed, stopping.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, pattern_size)

            display = frame.copy()
            if found and corners is not None:
                cv2.drawChessboardCorners(display, pattern_size, corners, found)

            cv2.putText(
                display,
                f"samples: {len(image_points)} / {args.min_samples}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display,
                "space=save valid board | c=calibrate | q=quit",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 180, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("YOLO Fisheye Camera Calibration", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            if key == ord(" ") and found and corners is not None:
                image_points.append(corners.reshape(1, -1, 2).astype(np.float64))
                object_points.append(objp.copy())
                image_size = (gray.shape[1], gray.shape[0])
                capture_path = args.capture_dir / f"yolo_fisheye_{len(image_points):02d}_{int(time.time())}.png"
                cv2.imwrite(str(capture_path), frame)
                print(f"Saved sample {len(image_points)}: {capture_path}")

            if key == ord("c"):
                if len(image_points) < args.min_samples or image_size is None:
                    print(f"Need at least {args.min_samples} valid samples before calibration.")
                    continue

                try:
                    rms, camera_matrix, dist_coeffs, _, _ = calibrate_fisheye(
                        object_points,
                        image_points,
                        image_size,
                        check_condition=not args.no_check_cond,
                    )
                except cv2.error as exc:
                    if args.no_check_cond:
                        raise
                    print(f"Calibration failed with CHECK_COND enabled: {exc}")
                    print("Retrying without CHECK_COND. Add more varied samples if RMS is high.")
                    rms, camera_matrix, dist_coeffs, _, _ = calibrate_fisheye(
                        object_points,
                        image_points,
                        image_size,
                        check_condition=False,
                    )

                payload = {
                    "timestamp_unix": time.time(),
                    "model": "opencv_fisheye",
                    "camera_role": "yolo_object_camera",
                    "rms_reprojection_error": float(rms),
                    "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
                    "pattern_size": {"cols": args.cols, "rows": args.rows},
                    "square_size_m": args.square_size_m,
                    "camera_matrix": camera_matrix.tolist(),
                    "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
                    "num_samples": len(image_points),
                    "capture_dir": str(args.capture_dir),
                    "notes": [
                        "This file is for the 160-degree YOLO/object fisheye camera.",
                        "Do not use it as the base AprilTag camera calibration file unless that camera is also fisheye-calibrated.",
                    ],
                }
                args.output_file.parent.mkdir(parents=True, exist_ok=True)
                args.output_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"Fisheye calibration saved to: {args.output_file}")
                print(f"RMS reprojection error: {rms:.6f}")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
