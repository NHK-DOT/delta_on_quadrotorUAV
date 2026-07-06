"""Vision inference test ROS 2 node.

Three modes:
  - synthetic: generate random images, pure inference benchmark
  - subscribe: subscribe to /camera/image_raw, test with real camera data
  - v4l2: direct camera capture via OpenCV, no ROS 2 camera driver needed

Outputs:
  - Terminal: real-time FPS / latency per iteration
  - Report file: full statistical summary + system resource assessment
"""

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .inference_engine import InferenceEngine
from .system_monitor import SystemMonitor


class VisionTestNode(Node):
    def __init__(self):
        super().__init__("vision_test_node")

        # parameters
        self.declare_parameter("model_path", "")
        self.declare_parameter("input_size", 320)
        self.declare_parameter("use_npu", True)
        self.declare_parameter("num_iterations", 100)
        self.declare_parameter("warmup_iterations", 10)
        self.declare_parameter("mode", "synthetic")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("report_path", "/tmp/vision_test_report.txt")
        self.declare_parameter("conf_thresh", 0.25)
        self.declare_parameter("camera_device", "/dev/video7")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("save_frames", False)

        self.model_path = self.get_parameter("model_path").value
        self.input_size = self.get_parameter("input_size").value
        self.use_npu = self.get_parameter("use_npu").value
        self.num_iter = self.get_parameter("num_iterations").value
        self.warmup = self.get_parameter("warmup_iterations").value
        self.mode = self.get_parameter("mode").value
        self.image_topic = self.get_parameter("image_topic").value
        self.report_path = self.get_parameter("report_path").value
        self.conf_thresh = self.get_parameter("conf_thresh").value
        self.camera_device = self.get_parameter("camera_device").value
        self.camera_width = self.get_parameter("camera_width").value
        self.camera_height = self.get_parameter("camera_height").value
        self.save_frames = self.get_parameter("save_frames").value

        # system monitor
        self.monitor = SystemMonitor(interval_sec=1.0)
        self.monitor.start()

        self.get_logger().info(
            f"Vision test starting: mode={self.mode} input_size={self.input_size} "
            f"use_npu={self.use_npu} iterations={self.num_iter} warmup={self.warmup}"
        )

        # start after a short delay to let system settle
        self.create_timer(2.0, self._start_bench, clock=self.get_clock())
        self._bench_started = False
        self._latest_image = None

        if self.mode == "subscribe":
            self._image_sub = self.create_subscription(
                Image, self.image_topic, self._image_callback, 1
            )

    def _image_callback(self, msg: Image):
        """Convert ROS Image to numpy RGB array."""
        try:
            h, w = msg.height, msg.width
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "rgb8":
                self._latest_image = raw.reshape(h, w, 3)
            elif msg.encoding == "bgr8":
                img = raw.reshape(h, w, 3)
                self._latest_image = img[:, :, ::-1].copy()
            elif msg.encoding == "mono8":
                img = raw.reshape(h, w)
                self._latest_image = np.stack([img, img, img], axis=-1)
            else:
                self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def _start_bench(self):
        if self._bench_started:
            return
        self._bench_started = True
        self.get_logger().info("Collecting system baseline ...")
        self.monitor.set_baseline()

        # load model
        try:
            engine = InferenceEngine(
                model_path=self.model_path,
                input_size=self.input_size,
                use_npu=self.use_npu,
                conf_thresh=self.conf_thresh,
                logger=self.get_logger(),
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self._write_report({"error": str(e)})
            rclpy.shutdown()
            return

        if self.mode == "v4l2":
            self._run_v4l2_mode(engine)
        elif self.mode == "subscribe":
            self._run_subscribe_mode(engine)
        else:
            self._run_synthetic_mode(engine)

    def _run_v4l2_mode(self, engine):
        """Direct V4L2 camera capture + inference loop."""
        import cv2

        device = self.camera_device
        w, h = self.camera_width, self.camera_height
        self.get_logger().info(
            f"Opening camera: {device} ({w}x{h})"
        )

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open camera: {device}")
            self._write_report({"error": f"Cannot open camera: {device}"})
            rclpy.shutdown()
            return

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Camera opened: {actual_w}x{actual_h}")

        save_dir = None
        if self.save_frames:
            save_dir = "/tmp/vision_test_frames"
            os.makedirs(save_dir, exist_ok=True)
            self.get_logger().info(f"Saving frames to: {save_dir}")

        # warmup
        self.get_logger().info(f"Warming up ({self.warmup} frames)...")
        for _ in range(self.warmup):
            ret, frame_bgr = cap.read()
            if not ret:
                continue
            frame_rgb = frame_bgr[:, :, ::-1].copy()
            engine.infer(frame_rgb)

        # main loop
        self.get_logger().info(f"Running {self.num_iter} iterations...")
        latencies = []
        det_count = 0
        for i in range(self.num_iter):
            ret, frame_bgr = cap.read()
            if not ret:
                self.get_logger().warn(f"Frame {i}: capture failed")
                continue

            frame_rgb = frame_bgr[:, :, ::-1].copy()
            result, latency_ms = engine.infer(frame_rgb)
            latencies.append(latency_ms)

            if result:
                det_count += 1
                cx, cy = result["center"]
                conf = result["conf"]
                x1, y1, x2, y2 = result["bbox"]
                label = result["class_name"]
                self.get_logger().info(
                    f"  Frame {i:3d}: {latency_ms:6.1f} ms | "
                    f"DET {label} conf={conf:.2f} "
                    f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}) "
                    f"center=({cx:.0f},{cy:.0f})"
                )
            else:
                self.get_logger().info(
                    f"  Frame {i:3d}: {latency_ms:6.1f} ms | no detection"
                )

            if save_dir and result:
                cx, cy = result["center"]
                conf = result["conf"]
                x1, y1, x2, y2 = result["bbox"]
                label = result["class_name"]
                cv2.rectangle(
                    frame_bgr, (int(x1), int(y1)), (int(x2), int(y2)),
                    (0, 255, 0), 2
                )
                cv2.circle(frame_bgr, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                text = f"{label} {conf:.2f} ({cx:.0f},{cy:.0f})"
                cv2.putText(
                    frame_bgr, text, (int(x1), int(y1) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
                cv2.imwrite(
                    os.path.join(save_dir, f"frame_{i:04d}.jpg"), frame_bgr
                )

        cap.release()

        # compute stats
        if latencies:
            arr = np.array(latencies)
            stats = {
                "n": len(latencies),
                "warmup": self.warmup,
                "mean_ms": float(arr.mean()),
                "std_ms": float(arr.std()),
                "min_ms": float(arr.min()),
                "max_ms": float(arr.max()),
                "p50_ms": float(np.percentile(arr, 50)),
                "p95_ms": float(np.percentile(arr, 95)),
                "p99_ms": float(np.percentile(arr, 99)),
                "fps": 1000.0 / float(arr.mean()) if arr.mean() > 0 else 0,
                "detections": det_count,
            }
            self._print_results(engine, stats)
        else:
            self.get_logger().error("No frames captured")
            self._write_report({"error": "No frames captured"})

        rclpy.shutdown()

    def _run_subscribe_mode(self, engine):
        """ROS 2 topic subscription mode."""
        self.get_logger().info(
            f"Waiting for image on {self.image_topic} ..."
        )
        deadline = time.time() + 30.0
        while self._latest_image is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._latest_image is None:
            self.get_logger().error("No image received within 30s")
            self._write_report({"error": "No image received"})
            rclpy.shutdown()
            return
        img = self._latest_image
        self.get_logger().info(f"Got image: {img.shape[1]}x{img.shape[0]}")
        self._run_benchmark(engine, img)

    def _run_synthetic_mode(self, engine):
        """Synthetic random image mode."""
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        self.get_logger().info("Using synthetic 640x480 image")
        self._run_benchmark(engine, img)

    def _run_benchmark(self, engine, img):
        """Run N-iteration benchmark on a single image."""
        self.get_logger().info(
            f"Running benchmark: {self.warmup} warmup + {self.num_iter} iterations ..."
        )
        stats = engine.benchmark(img, n=self.num_iter, warmup=self.warmup)
        self._print_results(engine, stats)
        rclpy.shutdown()

    def _print_results(self, engine, stats):
        """Print and report benchmark results."""
        self.get_logger().info("=== BENCHMARK RESULTS ===")
        self.get_logger().info(f"  Backend:    {engine.backend_type}")
        self.get_logger().info(f"  Provider:   {engine.provider_used}")
        self.get_logger().info(f"  Input size: {self.input_size}x{self.input_size}")
        self.get_logger().info(f"  Iterations: {stats['n']} (+ {stats['warmup']} warmup)")
        self.get_logger().info(f"  Mean:       {stats['mean_ms']:.2f} ms")
        self.get_logger().info(f"  Std:        {stats['std_ms']:.2f} ms")
        self.get_logger().info(f"  Min:        {stats['min_ms']:.2f} ms")
        self.get_logger().info(f"  Max:        {stats['max_ms']:.2f} ms")
        self.get_logger().info(f"  P50:        {stats['p50_ms']:.2f} ms")
        self.get_logger().info(f"  P95:        {stats['p95_ms']:.2f} ms")
        self.get_logger().info(f"  P99:        {stats['p99_ms']:.2f} ms")
        self.get_logger().info(f"  FPS:        {stats['fps']:.1f}")
        if "detections" in stats:
            self.get_logger().info(f"  Detections: {stats['detections']}/{stats['n']}")

        # system resources
        self.monitor.stop()
        sys_summary = self.monitor.get_summary()
        baseline = self.monitor.get_baseline()
        peak_mem = self.monitor.get_peak_mem_mb()

        self.get_logger().info("=== SYSTEM RESOURCES ===")
        if baseline:
            self.get_logger().info(f"  Baseline:   {baseline}")
        if sys_summary:
            self.get_logger().info(f"  CPU mean:   {sys_summary['cpu_mean']:.1f}%")
            self.get_logger().info(f"  CPU peak:   {sys_summary['cpu_max']:.1f}%")
            self.get_logger().info(
                f"  Mem peak:   {peak_mem:.0f} MB / {sys_summary['mem_total_mb']:.0f} MB"
            )

        # feasibility
        self.get_logger().info("=== FEASIBILITY ===")
        fps = stats["fps"]
        total_mem = sys_summary.get("mem_total_mb", 4096)
        available = total_mem - peak_mem
        ros_est = 300
        can_run = fps >= 10.0 and available > ros_est
        self.get_logger().info(
            f"  Vision FPS >= 10: {'YES' if fps >= 10 else 'NO'} ({fps:.1f})"
        )
        self.get_logger().info(
            f"  Free memory > {ros_est}MB: "
            f"{'YES' if available > ros_est else 'NO'} ({available:.0f} MB free)"
        )
        self.get_logger().info(
            f"  Overall: {'YES - can run full pipeline' if can_run else 'NO - consider offload'}"
        )

        # write report
        report_data = {
            "stats": stats,
            "system": sys_summary,
            "baseline": str(baseline) if baseline else None,
            "peak_mem_mb": peak_mem,
            "provider": engine.provider_used,
            "backend_type": engine.backend_type,
            "feasible": can_run,
        }
        self._write_report(report_data)
        self.get_logger().info(f"Report saved to: {self.report_path}")

    def _write_report(self, data: dict):
        try:
            os.makedirs(os.path.dirname(self.report_path), exist_ok=True)
        except OSError:
            pass
        with open(self.report_path, "w") as f:
            f.write("=" * 60 + "\n")
            f.write("  STM32MP257F-DK Vision Test Report\n")
            f.write("=" * 60 + "\n\n")
            if "error" in data:
                f.write(f"ERROR: {data['error']}\n")
                return
            stats = data.get("stats", {})
            sys_info = data.get("system", {})
            f.write(f"Backend:      {data.get('backend_type', 'unknown')}\n")
            f.write(f"Provider:     {data.get('provider', 'unknown')}\n")
            f.write(f"Input size:   {self.input_size}x{self.input_size}\n")
            f.write(f"Mode:         {self.mode}\n")
            f.write(f"Iterations:   {stats.get('n', 0)} + {stats.get('warmup', 0)} warmup\n\n")
            f.write("--- Inference ---\n")
            f.write(f"  Mean:  {stats.get('mean_ms', 0):.2f} ms\n")
            f.write(f"  Std:   {stats.get('std_ms', 0):.2f} ms\n")
            f.write(f"  Min:   {stats.get('min_ms', 0):.2f} ms\n")
            f.write(f"  Max:   {stats.get('max_ms', 0):.2f} ms\n")
            f.write(f"  P50:   {stats.get('p50_ms', 0):.2f} ms\n")
            f.write(f"  P95:   {stats.get('p95_ms', 0):.2f} ms\n")
            f.write(f"  P99:   {stats.get('p99_ms', 0):.2f} ms\n")
            f.write(f"  FPS:   {stats.get('fps', 0):.1f}\n")
            if "detections" in stats:
                f.write(f"  Detections: {stats['detections']}/{stats.get('n', 0)}\n")
            f.write("\n--- System ---\n")
            f.write(f"  CPU mean:    {sys_info.get('cpu_mean', 0):.1f}%\n")
            f.write(f"  CPU peak:    {sys_info.get('cpu_max', 0):.1f}%\n")
            f.write(f"  Mem peak:    {data.get('peak_mem_mb', 0):.0f} MB\n")
            f.write(f"  Mem total:   {sys_info.get('mem_total_mb', 0):.0f} MB\n\n")
            f.write("--- Feasibility ---\n")
            feasible = data.get("feasible", False)
            f.write(
                f"  Vision FPS >= 10:       "
                f"{'YES' if stats.get('fps', 0) >= 10 else 'NO'}\n"
            )
            available = sys_info.get("mem_total_mb", 0) - data.get("peak_mem_mb", 0)
            f.write(f"  Free mem for ROS:       {available:.0f} MB\n")
            f.write(f"  CAN RUN FULL PIPELINE:  {'YES' if feasible else 'NO'}\n")
            if feasible:
                f.write(
                    "\nConclusion: Board can run vision + kinematics + ROS.\n"
                )
            else:
                f.write(
                    "\nConclusion: Consider offloading vision to Jetson.\n"
                )


def main(args=None):
    rclpy.init(args=args)
    node = VisionTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
