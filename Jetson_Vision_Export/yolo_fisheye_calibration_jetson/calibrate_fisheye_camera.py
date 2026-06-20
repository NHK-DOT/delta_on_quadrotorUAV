#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


BACKENDS = {
    "auto": [
        ("DSHOW", getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY)),
        ("MSMF", getattr(cv2, "CAP_MSMF", cv2.CAP_ANY)),
        ("ANY", cv2.CAP_ANY),
    ],
    "dshow": [("DSHOW", getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY))],
    "msmf": [("MSMF", getattr(cv2, "CAP_MSMF", cv2.CAP_ANY))],
    "any": [("ANY", cv2.CAP_ANY)],
}


def make_csi_pipeline(sensor_id, width, height, fps, flip_method):
    return (
        "nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}, "
        "framerate=(fraction){fps}/1 ! "
        "nvvidconv flip-method={flip_method} ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=true sync=false"
    ).format(sensor_id=sensor_id, width=width, height=height, fps=fps, flip_method=flip_method)


def read_first_frame(cap):
    for _ in range(20):
        ok, frame = cap.read()
        if ok and frame is not None:
            return True, frame
        time.sleep(0.05)
    return False, None


def open_usb_camera(camera_index, width, height, fps, backend):
    for backend_name, backend_code in BACKENDS[backend]:
        cap = cv2.VideoCapture(camera_index, backend_code)
        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        ok, frame = read_first_frame(cap)
        if ok and frame is not None:
            return cap, backend_name
        cap.release()

    raise RuntimeError("Unable to open USB camera for fisheye calibration.")


def open_gstreamer_camera(pipeline, label):
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError("Unable to open {0} camera through GStreamer.".format(label))

    ok, frame = read_first_frame(cap)
    if ok and frame is not None:
        return cap, label

    cap.release()
    raise RuntimeError("{0} camera opened but did not return frames.".format(label))


def open_camera(args):
    if args.source == "usb":
        return open_usb_camera(args.camera_index, args.width, args.height, args.fps, args.backend)

    if args.source == "csi":
        pipeline = make_csi_pipeline(args.sensor_id, args.width, args.height, args.fps, args.flip_method)
        print("GStreamer CSI pipeline:")
        print(pipeline)
        return open_gstreamer_camera(pipeline, "CSI/nvarguscamerasrc")

    if not args.gst_pipeline:
        raise RuntimeError("--source gstreamer requires --gst-pipeline")

    print("Custom GStreamer pipeline:")
    print(args.gst_pipeline)
    return open_gstreamer_camera(args.gst_pipeline, "custom GStreamer")


def make_object_points(cols, rows, square_size_m):
    objp = np.zeros((1, rows * cols, 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size_m
    return objp


def find_corners(gray, pattern_size):
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


def calibrate_fisheye(object_points, image_points, image_size, check_condition):
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


def main():
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Live OpenCV fisheye calibration for the YOLO/object camera.")
    parser.add_argument("--source", choices=("usb", "csi", "gstreamer"), default="usb")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="auto")
    parser.add_argument("--sensor-id", type=int, default=0, help="Jetson CSI sensor-id for nvarguscamerasrc")
    parser.add_argument("--flip-method", type=int, default=0, help="Jetson nvvidconv flip-method")
    parser.add_argument("--gst-pipeline", default="", help="Custom GStreamer pipeline when --source gstreamer")
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

    cap, backend_name = open_camera(args)
    print("Opened YOLO/fisheye camera with source: {0}".format(backend_name))
    print("Hotkeys: space=save a valid checkerboard sample, c=calibrate, q=quit")
    print("Move the checkerboard through center, edges, corners, tilt angles, and different distances.")

    image_points = []
    object_points = []
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
                "samples: {0} / {1}".format(len(image_points), args.min_samples),
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
                capture_path = args.capture_dir / "yolo_fisheye_{0:02d}_{1}.png".format(
                    len(image_points),
                    int(time.time()),
                )
                cv2.imwrite(str(capture_path), frame)
                print("Saved sample {0}: {1}".format(len(image_points), capture_path))

            if key == ord("c"):
                if len(image_points) < args.min_samples or image_size is None:
                    print("Need at least {0} valid samples before calibration.".format(args.min_samples))
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
                    print("Calibration failed with CHECK_COND enabled: {0}".format(exc))
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
                    "camera_source": args.source,
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
                        "Keep the same lens, focus, resolution, and mount for downstream use.",
                        "Do not use it as the base AprilTag camera calibration file unless that camera is also fisheye-calibrated.",
                    ],
                }
                if args.source == "csi":
                    payload["jetson_csi"] = {
                        "sensor_id": args.sensor_id,
                        "flip_method": args.flip_method,
                        "pipeline": make_csi_pipeline(args.sensor_id, args.width, args.height, args.fps, args.flip_method),
                    }
                elif args.source == "gstreamer":
                    payload["gstreamer_pipeline"] = args.gst_pipeline

                args.output_file.parent.mkdir(parents=True, exist_ok=True)
                args.output_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
                print("Fisheye calibration saved to: {0}".format(args.output_file))
                print("RMS reprojection error: {0:.6f}".format(rms))
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
