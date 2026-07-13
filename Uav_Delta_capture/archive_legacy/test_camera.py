#!/usr/bin/env python3
"""V4L2 camera + NPU inference test. No ROS 2 dependency.

Usage:
  python3 test_camera.py                        # default: /dev/video7, 640x480, 30 frames
  python3 test_camera.py --device /dev/video7 --width 640 --height 480
  python3 test_camera.py --save --keep           # save detection frames to /tmp/vision_test_frames/
  python3 test_camera.py --iterations 100       # run 100 frames
  python3 test_camera.py --model /path/to/model.nb  # use specific model
"""

import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np

# add vision_test package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "uav_delta_capture", "src", "vision_test"))
from vision_test.inference_engine import InferenceEngine


class SystemMonitor:
    """Background thread sampling CPU, memory, temperature."""

    def __init__(self, interval=0.5):
        self._interval = interval
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.cpu_samples = []
        self.mem_samples = []
        self.temp_samples = []

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)

    def _read_cpu(self):
        with open("/proc/stat") as f:
            parts = f.readline().split()[1:]
        vals = [int(x) for x in parts]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        total = sum(vals)
        return idle, total

    def _read_mem(self):
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, v = line.split(":")[:2]
                info[k.strip()] = int(v.split()[0])
        total = info.get("MemTotal", 0)
        avail = info.get("MemAvailable", 0)
        used = total - avail
        return used / 1024, total / 1024  # MB

    def _read_temp(self):
        try:
            with open("/sys/devices/platform/soc@0/44070000.thermal-sensor/hwmon/hwmon0/temp1_input") as f:
                return int(f.read().strip()) / 1000.0  # Celsius
        except Exception:
            return 0.0

    def _run(self):
        prev_idle, prev_total = self._read_cpu()
        while not self._stop.is_set():
            time.sleep(self._interval)
            idle, total = self._read_cpu()
            d_idle = idle - prev_idle
            d_total = total - prev_total
            cpu_pct = (1.0 - d_idle / max(d_total, 1)) * 100
            prev_idle, prev_total = idle, total

            used_mb, total_mb = self._read_mem()
            temp = self._read_temp()

            self.cpu_samples.append(cpu_pct)
            self.mem_samples.append(used_mb)
            self.temp_samples.append(temp)

    def summary(self):
        if not self.cpu_samples:
            return {}
        return {
            "cpu_mean": np.mean(self.cpu_samples),
            "cpu_max": np.max(self.cpu_samples),
            "mem_used_mb": np.mean(self.mem_samples),
            "mem_peak_mb": np.max(self.mem_samples),
            "temp_mean": np.mean(self.temp_samples),
            "temp_max": np.max(self.temp_samples),
        }


def main():
    parser = argparse.ArgumentParser(description="V4L2 camera + NPU inference test")
    parser.add_argument("--device", default="/dev/video7", help="V4L2 device path")
    parser.add_argument("--width", type=int, default=640, help="capture width")
    parser.add_argument("--height", type=int, default=480, help="capture height")
    parser.add_argument("--iterations", type=int, default=30, help="number of frames")
    parser.add_argument("--warmup", type=int, default=3, help="warmup frames")
    parser.add_argument("--model", default="", help="model path (default: auto-detect .nb)")
    parser.add_argument("--conf", type=float, default=0.25, help="confidence threshold")
    parser.add_argument("--save", action="store_true", help="save detection frames")
    parser.add_argument("--keep", action="store_true", help="keep saved frames after test")
    args = parser.parse_args()

    print("=" * 50)
    print("  V4L2 Camera + NPU Inference Test")
    print("=" * 50)

    # load model
    print(f"\nLoading model...", flush=True)
    engine = InferenceEngine(
        model_path=args.model,
        use_npu=True,
        conf_thresh=args.conf,
    )
    print(f"  Backend:  {engine.backend_type}")
    print(f"  Provider: {engine.provider_used}")
    print(f"  Input:    {engine.input_size}x{engine.input_size}")

    # read memory baseline
    with open("/proc/meminfo") as f:
        mem_info = {}
        for line in f:
            k, v = line.split(":")[:2]
            mem_info[k.strip()] = int(v.split()[0])
    mem_total_mb = mem_info.get("MemTotal", 0) / 1024
    mem_avail_mb = mem_info.get("MemAvailable", 0) / 1024
    print(f"  Memory:   {mem_total_mb:.0f} MB total, {mem_avail_mb:.0f} MB available")

    # open camera
    print(f"\nOpening camera: {args.device} ({args.width}x{args.height})", flush=True)
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: Cannot open {args.device}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"  Opened:   {actual_w}x{actual_h}")

    save_dir = None
    if args.save:
        save_dir = "/tmp/vision_test_frames"
        os.makedirs(save_dir, exist_ok=True)
        print(f"  Saving:   {save_dir}/")

    # warmup
    print(f"\nWarming up ({args.warmup} frames)...", flush=True)
    for _ in range(args.warmup):
        ret, frame_bgr = cap.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            engine.infer(frame_rgb)

    # start system monitor
    monitor = SystemMonitor(interval=0.3)
    monitor.start()

    # main loop
    print(f"\nRunning {args.iterations} frames...", flush=True)
    latencies = []
    det_count = 0
    det_classes = {}

    for i in range(args.iterations):
        ret, frame_bgr = cap.read()
        if not ret:
            print(f"  Frame {i}: capture failed")
            continue

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        result, latency_ms = engine.infer(frame_rgb)
        latencies.append(latency_ms)

        if result:
            det_count += 1
            cx, cy = result["center"]
            conf = result["conf"]
            x1, y1, x2, y2 = result["bbox"]
            label = result["class_name"]
            det_classes[label] = det_classes.get(label, 0) + 1
            print(
                f"  Frame {i:3d}: {latency_ms:6.1f} ms | "
                f"DET {label} conf={conf:.2f} "
                f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}) "
                f"center=({cx:.0f},{cy:.0f})"
            )
            if save_dir:
                ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(frame_bgr, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
                cv2.circle(frame_bgr, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                text = f"{label} {conf:.2f} ({cx:.0f},{cy:.0f})"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                ty = max(iy1 + th + 4, th + 4)
                cv2.rectangle(frame_bgr, (ix1, ty - th - 4), (ix1 + tw + 4, ty), (0, 255, 0), -1)
                cv2.putText(frame_bgr, text, (ix1 + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                cv2.imwrite(os.path.join(save_dir, f"frame_{i:04d}.jpg"), frame_bgr)
        else:
            print(f"  Frame {i:3d}: {latency_ms:6.1f} ms | no detection")
            if save_dir:
                cv2.imwrite(os.path.join(save_dir, f"frame_{i:04d}.jpg"), frame_bgr)

    cap.release()
    monitor.stop()
    sys_summary = monitor.summary()

    # stats
    if not latencies:
        print("\nERROR: No frames captured")
        sys.exit(1)

    arr = np.array(latencies)
    fps = 1000.0 / arr.mean() if arr.mean() > 0 else 0

    print(f"\n{'=' * 50}")
    print(f"  INFERENCE RESULTS")
    print(f"{'=' * 50}")
    print(f"  Backend:    {engine.backend_type}")
    print(f"  Provider:   {engine.provider_used}")
    print(f"  Model:      {os.path.basename(engine._resolve_model(args.model))}")
    print(f"  Input:      {engine.input_size}x{engine.input_size}")
    print(f"  Frames:     {len(latencies)}")
    print(f"  Detections: {det_count}/{len(latencies)} ({100*det_count/len(latencies):.0f}%)")
    if det_classes:
        for cls, cnt in sorted(det_classes.items(), key=lambda x: -x[1]):
            print(f"    {cls}: {cnt}")
    print(f"  Mean:       {arr.mean():.2f} ms")
    print(f"  Min:        {arr.min():.2f} ms")
    print(f"  Max:        {arr.max():.2f} ms")
    print(f"  P50:        {np.percentile(arr, 50):.2f} ms")
    print(f"  P95:        {np.percentile(arr, 95):.2f} ms")
    print(f"  FPS:        {fps:.1f}")

    print(f"\n{'=' * 50}")
    print(f"  SYSTEM RESOURCES")
    print(f"{'=' * 50}")
    if sys_summary:
        print(f"  CPU mean:   {sys_summary['cpu_mean']:.1f}%")
        print(f"  CPU peak:   {sys_summary['cpu_max']:.1f}%")
        print(f"  Mem used:   {sys_summary['mem_used_mb']:.0f} MB / {mem_total_mb:.0f} MB")
        print(f"  Mem peak:   {sys_summary['mem_peak_mb']:.0f} MB")
        print(f"  Temp mean:  {sys_summary['temp_mean']:.1f} C")
        print(f"  Temp peak:  {sys_summary['temp_max']:.1f} C")
        print(f"  NPU:        busy during inference (~{arr.mean():.1f} ms/frame)")
        print(f"              (no direct utilization% available on this platform)")
        # feasibility
        free_mb = mem_total_mb - sys_summary['mem_peak_mb']
        print(f"\n  Free memory for other tasks: ~{free_mb:.0f} MB")

    if save_dir:
        n_saved = len([f for f in os.listdir(save_dir) if f.endswith(".jpg")])
        if args.keep:
            print(f"\n  {n_saved} frames saved to: {save_dir}/")
        else:
            for f in os.listdir(save_dir):
                if f.endswith(".jpg"):
                    os.remove(os.path.join(save_dir, f))
            print(f"\n  {n_saved} frames captured (cleaned up)")

    print(f"\n  {'OK - real-time capable' if fps >= 10 else 'WARNING - below 10 FPS'}")


if __name__ == "__main__":
    main()
