#!/usr/bin/env python3
import argparse
import json
import select
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from calibrate_fisheye_camera import (
    BACKENDS,
    calibrate_fisheye,
    make_object_points,
    open_camera,
)


def describe_board(corners, image_size):
    pts = corners.reshape(-1, 2)
    width, height = image_size
    min_xy = pts.min(axis=0)
    max_xy = pts.max(axis=0)
    center = pts.mean(axis=0)
    board_w = max_xy[0] - min_xy[0]
    board_h = max_xy[1] - min_xy[1]
    coverage = max(board_w / max(width, 1), board_h / max(height, 1))

    if center[0] < width * 0.38:
        x_name = "left"
    elif center[0] > width * 0.62:
        x_name = "right"
    else:
        x_name = "center-x"

    if center[1] < height * 0.38:
        y_name = "top"
    elif center[1] > height * 0.62:
        y_name = "bottom"
    else:
        y_name = "center-y"

    if x_name == "center-x" and y_name == "center-y":
        area_name = "center"
    else:
        area_name = "{0}/{1}".format(x_name, y_name)

    return area_name, coverage, center


def find_corners_fast(gray, pattern_size):
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
    if not found:
        return False, None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)
    refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return True, refined.astype(np.float64)


def save_preview(frame, corners, found, pattern_size, preview_dir):
    preview_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(preview_dir / "latest_frame.jpg"), frame)
    display = frame.copy()
    if found and corners is not None:
        cv2.drawChessboardCorners(display, pattern_size, corners, found)
    cv2.imwrite(str(preview_dir / "latest_detected.jpg"), display)


def poll_command():
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return None
    return sys.stdin.readline().strip().lower()


def main():
    project_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="No-GUI OpenCV fisheye calibration for the YOLO/object camera.")
    parser.add_argument("--source", choices=("usb", "csi", "gstreamer"), default="csi")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="auto")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--flip-method", type=int, default=0)
    parser.add_argument("--gst-pipeline", default="")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size-m", type=float, default=0.020)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--status-interval", type=float, default=0.8)
    parser.add_argument("--auto-save", action="store_true", help="Automatically save valid checkerboard samples")
    parser.add_argument("--auto-save-interval", type=float, default=2.0)
    parser.add_argument("--output-file", type=Path, default=project_root / "calibration" / "yolo_fisheye_camera_intrinsics.json")
    parser.add_argument("--capture-dir", type=Path, default=project_root / "calibration" / "captures_yolo_fisheye")
    parser.add_argument("--preview-dir", type=Path, default=project_root / "preview")
    parser.add_argument("--no-check-cond", action="store_true")
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)
    objp = make_object_points(args.cols, args.rows, args.square_size_m)
    cap, backend_name = open_camera(args)

    image_points = []
    object_points = []
    image_size = None
    last_found = False
    last_corners = None
    last_frame = None
    last_status = 0.0
    last_auto_save = 0.0
    args.capture_dir.mkdir(parents=True, exist_ok=True)

    print("Opened camera with source: {0}".format(backend_name))
    print("No-GUI mode. Keep this terminal focused.")
    print("Commands: Enter=save current valid board, c=calibrate, p=save preview jpg, q=quit")
    if args.auto_save:
        print("Auto-save mode: every valid board is saved after {0:.1f}s spacing; calibration starts at target count.".format(args.auto_save_interval))
    print("Preview files: {0}".format(args.preview_dir))
    print("Target samples: {0}, square size: {1:.3f} m".format(args.min_samples, args.square_size_m))
    print("Waiting for camera frames...")
    sys.stdout.flush()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("\nCamera frame grab failed, stopping.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners_fast(gray, pattern_size)
            image_size = (gray.shape[1], gray.shape[0])
            last_found = bool(found)
            last_corners = corners
            last_frame = frame

            now = time.time()
            if now - last_status >= args.status_interval:
                if found and corners is not None:
                    area_name, coverage, center = describe_board(corners, image_size)
                    msg = (
                        "FOUND samples {0}/{1} pos={2} coverage={3:.2f} "
                        "center=({4:.0f},{5:.0f})"
                    ).format(
                        len(image_points),
                        args.min_samples,
                        area_name,
                        coverage,
                        center[0],
                        center[1],
                    )
                else:
                    msg = "NOT FOUND samples {0}/{1}; move full checkerboard into view".format(
                        len(image_points),
                        args.min_samples,
                    )
                print(msg)
                last_status = now

            if args.auto_save and found and corners is not None and now - last_auto_save >= args.auto_save_interval:
                image_points.append(corners.reshape(1, -1, 2).astype(np.float64))
                object_points.append(objp.copy())
                capture_path = args.capture_dir / "auto_fisheye_{0:02d}_{1}.png".format(
                    len(image_points),
                    int(time.time()),
                )
                cv2.imwrite(str(capture_path), frame)
                save_preview(frame, corners, True, pattern_size, args.preview_dir)
                area_name, coverage, _ = describe_board(corners, image_size)
                print("AUTO saved sample {0}/{1}: pos={2} coverage={3:.2f}".format(
                    len(image_points),
                    args.min_samples,
                    area_name,
                    coverage,
                ))
                last_auto_save = now
                if len(image_points) >= args.min_samples:
                    cmd = "c"
                else:
                    continue
            else:
                cmd = poll_command()

            if cmd is None:
                continue

            if cmd == "q":
                print("\nQuit without calibration.")
                break

            if cmd == "p":
                save_preview(frame, corners, found, pattern_size, args.preview_dir)
                print("\nSaved preview to: {0}".format(args.preview_dir))
                continue

            if cmd == "c":
                if len(image_points) < args.min_samples:
                    print("\nNeed at least {0} valid samples before calibration.".format(args.min_samples))
                    continue

                print("\nCalibrating with {0} samples...".format(len(image_points)))
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
                    print("Calibration failed with CHECK_COND enabled: {0}".format(exc))
                    print("Retrying without CHECK_COND.")
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
                    "camera_source": args.source,
                    "capture_mode": "nogui_terminal",
                    "rms_reprojection_error": float(rms),
                    "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
                    "pattern_size": {"cols": args.cols, "rows": args.rows},
                    "square_size_m": args.square_size_m,
                    "camera_matrix": camera_matrix.tolist(),
                    "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
                    "num_samples": len(image_points),
                    "capture_dir": str(args.capture_dir),
                }
                args.output_file.parent.mkdir(parents=True, exist_ok=True)
                args.output_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
                print("Fisheye calibration saved to: {0}".format(args.output_file))
                print("RMS reprojection error: {0:.6f}".format(rms))
                break

            if cmd == "":
                if not last_found or last_corners is None or last_frame is None:
                    print("\nCurrent frame has no valid checkerboard; not saved.")
                    continue

                image_points.append(last_corners.reshape(1, -1, 2).astype(np.float64))
                object_points.append(objp.copy())
                capture_path = args.capture_dir / "nogui_fisheye_{0:02d}_{1}.png".format(
                    len(image_points),
                    int(time.time()),
                )
                cv2.imwrite(str(capture_path), last_frame)
                save_preview(last_frame, last_corners, True, pattern_size, args.preview_dir)
                area_name, coverage, _ = describe_board(last_corners, image_size)
                print("\nSaved sample {0}: {1} pos={2} coverage={3:.2f}".format(
                    len(image_points),
                    capture_path,
                    area_name,
                    coverage,
                ))
                continue

            print("\nUnknown command: {0}".format(cmd))
    finally:
        cap.release()


if __name__ == "__main__":
    main()
