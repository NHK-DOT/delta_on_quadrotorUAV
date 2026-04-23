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
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, backend_name
        cap.release()
    raise RuntimeError("Unable to open camera for calibration.")


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Live chessboard camera calibration for a USB webcam.")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="auto")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--cols", type=int, default=9, help="Internal chessboard corners per row")
    parser.add_argument("--rows", type=int, default=6, help="Internal chessboard corners per column")
    parser.add_argument("--square-size-m", type=float, default=0.025, help="Checkerboard square edge size")
    parser.add_argument("--min-samples", type=int, default=12)
    parser.add_argument("--output-file", type=Path, default=project_root / "calibration" / "camera_intrinsics.json")
    parser.add_argument("--capture-dir", type=Path, default=project_root / "calibration" / "captures")
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
    objp *= args.square_size_m

    cap, backend_name = open_camera(args.camera_index, args.width, args.height, args.fps, args.backend)
    print(f"Opened camera {args.camera_index} with backend: {backend_name}")
    print("Hotkeys: space=save a valid checkerboard sample, c=calibrate, q=quit")

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
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            display = frame.copy()
            refined = None
            if found:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                )
                refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                cv2.drawChessboardCorners(display, pattern_size, refined, found)

            cv2.putText(display, f"samples: {len(image_points)} / {args.min_samples}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(display, "space=save valid board | c=calibrate | q=quit", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 180, 0), 2, cv2.LINE_AA)
            cv2.imshow("Camera Calibration", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            if key == ord(" ") and found and refined is not None:
                image_points.append(refined)
                object_points.append(objp.copy())
                image_size = (gray.shape[1], gray.shape[0])
                capture_path = args.capture_dir / f"calib_{len(image_points):02d}_{int(time.time())}.png"
                cv2.imwrite(str(capture_path), frame)
                print(f"Saved sample {len(image_points)}: {capture_path}")

            if key == ord("c"):
                if len(image_points) < args.min_samples or image_size is None:
                    print(f"Need at least {args.min_samples} valid samples before calibration.")
                    continue

                rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
                    object_points,
                    image_points,
                    image_size,
                    None,
                    None,
                )

                payload = {
                    "timestamp_unix": time.time(),
                    "rms_reprojection_error": float(rms),
                    "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
                    "pattern_size": {"cols": args.cols, "rows": args.rows},
                    "square_size_m": args.square_size_m,
                    "camera_matrix": camera_matrix.tolist(),
                    "dist_coeffs": dist_coeffs.tolist(),
                    "num_samples": len(image_points),
                }
                args.output_file.parent.mkdir(parents=True, exist_ok=True)
                args.output_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"Calibration saved to: {args.output_file}")
                print(f"RMS reprojection error: {rms:.6f}")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
