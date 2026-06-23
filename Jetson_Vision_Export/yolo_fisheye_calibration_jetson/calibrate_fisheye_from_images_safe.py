#!/usr/bin/env python3
import argparse
import json
import multiprocessing as mp
import time
from pathlib import Path

import cv2
import numpy as np

from calibrate_fisheye_camera import calibrate_fisheye, make_object_points


def find_worker(path, cols, rows, queue):
    try:
        cv2.setNumThreads(1)
        frame = cv2.imread(str(path))
        if frame is None:
            queue.put(("bad", str(path), None, None))
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern_size = (cols, rows)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
        if not found or corners is None:
            queue.put(("no", str(path), None, (gray.shape[1], gray.shape[0])))
            return
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        queue.put(("ok", str(path), refined.astype(np.float64), (gray.shape[1], gray.shape[0])))
    except Exception as exc:
        queue.put(("err", str(path), repr(exc), None))


def detect_with_timeout(path, cols, rows, timeout):
    queue = mp.Queue(maxsize=1)
    proc = mp.Process(target=find_worker, args=(path, cols, rows, queue))
    proc.start()
    proc.join(timeout)
    if proc.is_alive():
        proc.terminate()
        proc.join(1.0)
        if proc.is_alive():
            proc.kill()
        return "timeout", str(path), None, None
    if queue.empty():
        return "empty", str(path), None, None
    return queue.get()


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
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Safe offline fisheye calibration with per-image detection timeout.")
    parser.add_argument("--image-dir", type=Path, default=root / "calibration" / "raw_apriltag_fullfov_1280x960")
    parser.add_argument("--cols", type=int, default=10)
    parser.add_argument("--rows", type=int, default=7)
    parser.add_argument("--square-size-m", type=float, default=0.020)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--stop-after-valid", type=int, default=60)
    parser.add_argument("--max-images", type=int, default=240)
    parser.add_argument("--per-image-timeout", type=float, default=4.0)
    parser.add_argument(
        "--output-file",
        type=Path,
        default=root / "calibration" / "usable_3k_downsample_1280x960" / "apriltag_fullfov_1280x960_intrinsics.json",
    )
    parser.add_argument("--valid-dir", type=Path, default=root / "calibration" / "valid_apriltag_fullfov_1280x960")
    parser.add_argument("--no-check-cond", action="store_true")
    args = parser.parse_args()

    image_paths = sorted(args.image_dir.glob("*.jpg")) + sorted(args.image_dir.glob("*.png"))
    if args.max_images > 0:
        image_paths = image_paths[: args.max_images]
    if not image_paths:
        raise SystemExit("No images found in {0}".format(args.image_dir))

    pattern_size = (args.cols, args.rows)
    objp = make_object_points(args.cols, args.rows, args.square_size_m)
    args.valid_dir.mkdir(parents=True, exist_ok=True)

    object_points = []
    image_points = []
    image_size = None
    counts = {"ok": 0, "no": 0, "timeout": 0, "bad": 0, "err": 0, "empty": 0}
    print("Scanning {0} images with timeout {1:.1f}s...".format(len(image_paths), args.per_image_timeout), flush=True)
    for path in image_paths:
        status, name, payload, size = detect_with_timeout(path, args.cols, args.rows, args.per_image_timeout)
        counts[status] = counts.get(status, 0) + 1
        if status != "ok":
            print("{0:7s} {1}".format(status.upper(), Path(name).name), flush=True)
            continue
        corners = payload
        image_size = size
        object_points.append(objp.copy())
        image_points.append(corners.reshape(1, -1, 2).astype(np.float64))
        frame = cv2.imread(name)
        if frame is not None:
            marked = frame.copy()
            cv2.drawChessboardCorners(marked, pattern_size, corners.astype(np.float32).reshape(-1, 1, 2), True)
            cv2.imwrite(str(args.valid_dir / Path(name).name), marked)
        pos, coverage = describe(corners, image_size)
        print("OK      {0} valid={1}/{2} pos={3} coverage={4:.2f}".format(
            Path(name).name, len(image_points), args.min_samples, pos, coverage
        ), flush=True)
        if len(image_points) >= args.stop_after_valid:
            break

    print("counts={0}".format(counts), flush=True)
    if len(image_points) < args.min_samples:
        raise SystemExit("Only {0} valid samples; need at least {1}.".format(len(image_points), args.min_samples))

    print("Calibrating with {0} valid samples...".format(len(image_points)), flush=True)
    try:
        rms, camera_matrix, dist_coeffs, _, _ = calibrate_fisheye(
            object_points, image_points, image_size, check_condition=not args.no_check_cond
        )
    except cv2.error as exc:
        if args.no_check_cond:
            raise
        print("CHECK_COND failed: {0}".format(exc), flush=True)
        print("Retrying without CHECK_COND.", flush=True)
        rms, camera_matrix, dist_coeffs, _, _ = calibrate_fisheye(
            object_points, image_points, image_size, check_condition=False
        )

    payload = {
        "timestamp_unix": time.time(),
        "model": "opencv_fisheye",
        "camera_role": "apriltag_base_camera_fullfov",
        "capture_mode": "3k_fullfov_downsample_1280x960_offline_images",
        "rms_reprojection_error": float(rms),
        "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
        "pattern_size": {"cols": args.cols, "rows": args.rows},
        "square_size_m": args.square_size_m,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "num_samples": len(image_points),
        "source_image_dir": str(args.image_dir),
        "valid_dir": str(args.valid_dir),
        "detection_counts": counts,
    }
    args.output_file.parent.mkdir(parents=True, exist_ok=True)
    args.output_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Saved calibration: {0}".format(args.output_file), flush=True)
    print("RMS reprojection error: {0:.6f}".format(rms), flush=True)


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
