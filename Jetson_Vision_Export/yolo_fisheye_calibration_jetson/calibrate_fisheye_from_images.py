#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from calibrate_fisheye_camera import calibrate_fisheye, make_object_points


def find_corners(gray, pattern_size):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
    if not found:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.0001)
    refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return True, refined.astype(np.float64)


def describe(corners, image_size):
    pts = corners.reshape(-1, 2)
    width, height = image_size
    center = pts.mean(axis=0)
    min_xy = pts.min(axis=0)
    max_xy = pts.max(axis=0)
    coverage = max((max_xy[0] - min_xy[0]) / max(width, 1), (max_xy[1] - min_xy[1]) / max(height, 1))
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
    return "{0}/{1}".format(x_name, y_name), coverage


def main():
    project_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Calibrate fisheye camera from captured checkerboard images.")
    parser.add_argument("--image-dir", type=Path, default=project_root / "capture_stream")
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size-m", type=float, default=0.020)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--output-file", type=Path, default=project_root / "calibration" / "yolo_fisheye_camera_intrinsics.json")
    parser.add_argument("--valid-dir", type=Path, default=project_root / "calibration" / "valid_fisheye_frames")
    parser.add_argument("--max-images", type=int, default=120)
    parser.add_argument("--stop-after-valid", type=int, default=40)
    parser.add_argument("--no-check-cond", action="store_true")
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)
    objp = make_object_points(args.cols, args.rows, args.square_size_m)
    image_paths = sorted(args.image_dir.glob("*.jpg")) + sorted(args.image_dir.glob("*.png"))
    if not image_paths:
        raise SystemExit("No images found in {0}".format(args.image_dir))
    if args.max_images > 0 and len(image_paths) > args.max_images:
        step = max(1, len(image_paths) // args.max_images)
        image_paths = image_paths[::step][: args.max_images]

    args.valid_dir.mkdir(parents=True, exist_ok=True)
    object_points = []
    image_points = []
    image_size = None

    print("Scanning {0} images...".format(len(image_paths)))
    for path in image_paths:
        frame = cv2.imread(str(path))
        if frame is None:
            print("SKIP unreadable {0}".format(path.name))
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        found, corners = find_corners(gray, pattern_size)
        if not found or corners is None:
            print("NO   {0}".format(path.name))
            continue
        pos, coverage = describe(corners, image_size)
        object_points.append(objp.copy())
        image_points.append(corners.reshape(1, -1, 2).astype(np.float64))
        marked = frame.copy()
        try:
            cv2.drawChessboardCorners(marked, pattern_size, corners.astype(np.float32), True)
        except cv2.error as exc:
            print("WARN draw failed for {0}: {1}".format(path.name, exc))
        out = args.valid_dir / path.name
        cv2.imwrite(str(out), marked)
        print("OK   {0} valid={1}/{2} pos={3} coverage={4:.2f}".format(
            path.name,
            len(image_points),
            args.min_samples,
            pos,
            coverage,
        ))
        if len(image_points) >= args.stop_after_valid:
            print("Reached stop-after-valid={0}; stop scanning.".format(args.stop_after_valid))
            break

    if len(image_points) < args.min_samples:
        raise SystemExit("Only {0} valid samples; need at least {1}.".format(len(image_points), args.min_samples))

    print("Calibrating with {0} valid samples...".format(len(image_points)))
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
        print("CHECK_COND calibration failed: {0}".format(exc))
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
        "capture_mode": "gstreamer_preview_offline_images",
        "rms_reprojection_error": float(rms),
        "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
        "pattern_size": {"cols": args.cols, "rows": args.rows},
        "square_size_m": args.square_size_m,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "num_samples": len(image_points),
        "source_image_dir": str(args.image_dir),
        "valid_dir": str(args.valid_dir),
    }
    args.output_file.parent.mkdir(parents=True, exist_ok=True)
    args.output_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Saved calibration: {0}".format(args.output_file))
    print("RMS reprojection error: {0:.6f}".format(rms))


if __name__ == "__main__":
    main()
