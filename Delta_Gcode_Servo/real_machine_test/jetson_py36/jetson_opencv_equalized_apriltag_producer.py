#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3K fisheye downsampled AprilTag JSON producer for Jetson Xavier NX.

This is the conservative live detector used by the workspace sampler. It keeps
the same 3264x2464 -> 1280x960 full-FOV camera path as the GPU benchmark, then
applies a grayscale contrast pass before OpenCV's AprilTag detector. That makes
the base tag detectable in the current dark workspace where raw GPU detection
can produce an empty JSON stream.
"""

from __future__ import print_function

import argparse
import json
import os
import time

import cv2
import numpy as np


DEFAULT_OUTPUT_JSON = "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_latest_jetson.json"
DEFAULT_OUTPUT_IMAGE = "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/output/apriltag_opencv_equalized_annotated.jpg"
DEFAULT_CALIBRATION = (
    "/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/"
    "calibration/usable_3k_downsample_1280x960/apriltag_fullfov_1280x960_intrinsics.json"
)


def parse_size(value):
    if "x" not in value:
        raise argparse.ArgumentTypeError("size must be WxH")
    left, right = value.lower().split("x", 1)
    return int(left), int(right)


def make_pipeline(args):
    return (
        "nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! "
        "video/x-raw(memory:NVMM),width={sensor_w},height={sensor_h},framerate={fps}/1 ! "
        "nvvidconv flip-method={flip_method} ! "
        "video/x-raw,width={out_w},height={out_h},format=I420 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    ).format(
        sensor_id=args.sensor_id,
        sensor_mode=args.sensor_mode,
        sensor_w=args.sensor_size[0],
        sensor_h=args.sensor_size[1],
        fps=args.fps,
        flip_method=args.flip_method,
        out_w=args.output_size[0],
        out_h=args.output_size[1],
    )


def load_intrinsics(path, output_size):
    with open(path, "r") as fh:
        payload = json.load(fh)
    camera_matrix = np.array(payload["camera_matrix"], dtype=np.float64)
    dist = payload.get("dist_coeffs", payload.get("distortion_coefficients", [0, 0, 0, 0, 0]))
    dist_coeffs = np.array(dist, dtype=np.float64).reshape(-1, 1)
    src_w = int(payload.get("width", output_size[0]))
    src_h = int(payload.get("height", output_size[1]))
    if src_w > 0 and src_h > 0 and (src_w != output_size[0] or src_h != output_size[1]):
        camera_matrix[0, :] *= float(output_size[0]) / float(src_w)
        camera_matrix[1, :] *= float(output_size[1]) / float(src_h)
    return camera_matrix, dist_coeffs


def preprocess_gray(gray, mode):
    if mode == "raw":
        return gray
    if mode == "equalize":
        return cv2.equalizeHist(gray)
    if mode == "clahe":
        return cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
    if mode.startswith("gamma:"):
        gamma = float(mode.split(":", 1)[1])
        gamma = max(0.05, min(5.0, gamma))
        table = np.array([((i / 255.0) ** gamma) * 255 for i in range(256)], dtype=np.uint8)
        return cv2.LUT(gray, table)
    raise ValueError("unknown preprocess mode: %s" % mode)


def atomic_write_json(path, payload):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(payload, fh, indent=2)
        fh.write("\n")
    os.rename(tmp, path)


def normalized_center(center, width, height):
    return {
        "x": float((center[0] - width / 2.0) / (width / 2.0)),
        "y": float((center[1] - height / 2.0) / (height / 2.0)),
    }


def detection_payload(tag_id, points, camera_matrix, dist_coeffs, tag_size_m, output_size):
    obj = np.array(
        [
            [-tag_size_m / 2.0, tag_size_m / 2.0, 0.0],
            [tag_size_m / 2.0, tag_size_m / 2.0, 0.0],
            [tag_size_m / 2.0, -tag_size_m / 2.0, 0.0],
            [-tag_size_m / 2.0, -tag_size_m / 2.0, 0.0],
        ],
        dtype=np.float32,
    )
    ok, rvec, tvec = cv2.solvePnP(obj, points.astype(np.float32), camera_matrix, dist_coeffs)
    if ok:
        rotation, _ = cv2.Rodrigues(rvec)
        position = {"x": float(tvec[0][0]), "y": float(tvec[1][0]), "z": float(tvec[2][0])}
    else:
        rotation = np.eye(3, dtype=np.float64)
        position = {"x": 0.0, "y": 0.0, "z": 0.0}
    center = points.mean(axis=0)
    side_lengths = [np.linalg.norm(points[(i + 1) % 4] - points[i]) for i in range(4)]
    return {
        "id": int(tag_id),
        "center_px": {"x": float(center[0]), "y": float(center[1])},
        "normalized_xy": normalized_center(center, output_size[0], output_size[1]),
        "size_px": float(sum(side_lengths) / 4.0),
        "corners_px": [{"x": float(p[0]), "y": float(p[1])} for p in points],
        "position_m": position,
        "rotation_matrix": [[float(rotation[r, c]) for c in range(3)] for r in range(3)],
    }


def build_snapshot(args, frames, elapsed_s, detections, camera_matrix):
    return {
        "timestamp_unix": time.time(),
        "camera": {
            "source": "csi_nvargus_opencv_aruco_equalized",
            "sensor_id": args.sensor_id,
            "sensor_mode": args.sensor_mode,
            "sensor_width": args.sensor_size[0],
            "sensor_height": args.sensor_size[1],
            "sensor_fps_request": args.fps,
            "processing_width": args.output_size[0],
            "processing_height": args.output_size[1],
            "pixel_mode": "BGR_gray_%s" % args.preprocess,
        },
        "tag_family": args.family,
        "tag_size_m": args.tag_size_m,
        "calibration": {
            "file": args.calib_json,
            "fx": float(camera_matrix[0, 0]),
            "fy": float(camera_matrix[1, 1]),
            "cx": float(camera_matrix[0, 2]),
            "cy": float(camera_matrix[1, 2]),
        },
        "frames_read": frames,
        "elapsed_s": elapsed_s,
        "detections": detections,
    }


def main():
    parser = argparse.ArgumentParser(description="OpenCV equalized 3K AprilTag JSON producer.")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--sensor-mode", type=int, default=0)
    parser.add_argument("--sensor-size", type=parse_size, default=(3264, 2464))
    parser.add_argument("--output-size", type=parse_size, default=(1280, 960))
    parser.add_argument("--fps", type=int, default=21)
    parser.add_argument("--flip-method", type=int, default=0)
    parser.add_argument("--family", default="tag36h11", choices=["tag36h11"])
    parser.add_argument("--tag-size-m", type=float, default=0.0305)
    parser.add_argument("--calib-json", default=DEFAULT_CALIBRATION)
    parser.add_argument("--output-json", default=DEFAULT_OUTPUT_JSON)
    parser.add_argument("--output-image", default=DEFAULT_OUTPUT_IMAGE)
    parser.add_argument("--preprocess", default="equalize", help="raw, equalize, clahe, or gamma:N")
    parser.add_argument("--warmup", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=0.0, help="0 means run until interrupted")
    parser.add_argument("--print-every", type=int, default=20)
    args = parser.parse_args()

    family_code = getattr(cv2.aruco, "DICT_APRILTAG_36h11", getattr(cv2.aruco, "DICT_APRILTAG_36H11", None))
    if family_code is None:
        raise RuntimeError("OpenCV aruco has no AprilTag 36h11 dictionary")
    dictionary = cv2.aruco.Dictionary_get(family_code)
    params = cv2.aruco.DetectorParameters_create()
    if hasattr(params, "cornerRefinementMethod"):
        params.cornerRefinementMethod = getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1)

    camera_matrix, dist_coeffs = load_intrinsics(args.calib_json, args.output_size)
    pipeline = make_pipeline(args)
    print("pipeline=%s" % pipeline)
    print(
        "calibration=%s fx=%.3f fy=%.3f cx=%.3f cy=%.3f"
        % (args.calib_json, camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2])
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("failed to open CSI camera pipeline")

    frames = 0
    frames_with_tags = 0
    t0 = time.time()
    latest_frame = None
    try:
        index = 0
        while True:
            if args.seconds > 0 and time.time() - t0 >= args.seconds:
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                print("read failed at frame %d" % index)
                time.sleep(0.02)
                index += 1
                continue
            if index < args.warmup:
                index += 1
                continue
            frames += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            work = preprocess_gray(gray, args.preprocess)
            corners, ids, rejected = cv2.aruco.detectMarkers(work, dictionary, parameters=params)
            detections = []
            if ids is not None:
                for det_index, tag_id in enumerate(ids.reshape(-1)):
                    points = corners[det_index].reshape(4, 2).astype(np.float32)
                    detections.append(
                        detection_payload(tag_id, points, camera_matrix, dist_coeffs, args.tag_size_m, args.output_size)
                    )
                if detections:
                    frames_with_tags += 1
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            snapshot = build_snapshot(args, frames, time.time() - t0, detections, camera_matrix)
            atomic_write_json(args.output_json, snapshot)
            latest_frame = frame
            if args.print_every > 0 and frames % args.print_every == 0:
                print("frames=%d detections=%d ids=%s" % (frames, len(detections), [d["id"] for d in detections]))
            index += 1
    finally:
        cap.release()

    if latest_frame is not None and args.output_image:
        parent = os.path.dirname(os.path.abspath(args.output_image))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent)
        cv2.imwrite(args.output_image, latest_frame)
    print("summary frames=%d frames_with_tags=%d elapsed_s=%.2f" % (frames, frames_with_tags, time.time() - t0))


if __name__ == "__main__":
    main()
