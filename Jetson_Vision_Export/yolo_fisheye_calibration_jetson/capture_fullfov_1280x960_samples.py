#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import cv2


def make_pipeline():
    return (
        "nvarguscamerasrc sensor-id=0 sensor-mode=0 ! "
        "video/x-raw(memory:NVMM),width=3264,height=2464,framerate=21/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw,width=1280,height=960,format=I420 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Capture full-FOV 1280x960 checkerboard frames only; no corner detection.")
    parser.add_argument(
        "--capture-dir",
        type=Path,
        default=root / "calibration" / "raw_apriltag_fullfov_1280x960",
    )
    args = parser.parse_args()
    args.capture_dir.mkdir(parents=True, exist_ok=True)

    pipeline = make_pipeline()
    print("GStreamer pipeline:")
    print(pipeline)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Unable to open full-FOV CSI pipeline")

    window = "Capture Full-FOV 1280x960 Calibration Samples"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 960)
    cv2.moveWindow(window, 40, 40)

    count = len(list(args.capture_dir.glob("*.png")))
    print("space=save raw frame, q=quit")
    print("capture dir: {0}".format(args.capture_dir))
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("frame grab failed")
            break

        display = frame.copy()
        cv2.putText(
            display,
            "saved: {0} | space=save | q=quit".format(count),
            (20, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(window, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            count += 1
            path = args.capture_dir / "fullfov_1280x960_{0:03d}_{1}.png".format(count, int(time.time()))
            cv2.imwrite(str(path), frame)
            print("saved {0}".format(path))

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
