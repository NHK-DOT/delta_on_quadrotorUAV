#!/usr/bin/env python3
"""STM32MP257F-DK NPU Vision Benchmark using ST X-LINUX-AI runtime.

Tests YOLOv8n 320x320 INT8 on NPU (stai_mpu) vs CPU (onnxruntime).
No ROS 2 dependency. No GUI.
"""

import os
import sys
import time
import numpy as np

# ── Paths ────────────────────────────────────────────────────────
NPU_MODEL = "/usr/local/x-linux-ai/people-tracking-heatmap/models/yolov8n_people/yolov8n_320_quant_pt_uf_od_coco-person-st.nb"
CPU_MODEL = "/usr/local/Uav_Delta_capture/models/yolov8n.onnx"
REPORT_PATH = "/tmp/vision_bench_report.txt"
NUM_ITER = 100
WARMUP = 10


# ── System monitor ──────────────────────────────────────────────

def read_meminfo():
    info = {}
    with open("/proc/meminfo") as f:
        for line in f:
            parts = line.split(":")
            if len(parts) == 2:
                key = parts[0].strip()
                val = parts[1].strip().split()[0]
                try:
                    info[key] = int(val)
                except ValueError:
                    pass
    return info


def mem_snapshot():
    m = read_meminfo()
    total = m.get("MemTotal", 0) / 1024.0
    avail = m.get("MemAvailable", 0) / 1024.0
    return total, total - avail, (total - avail) / total * 100 if total else 0


# ── Preprocessing ───────────────────────────────────────────────

def letterbox(img, size=320):
    h, w = img.shape[:2]
    scale = min(size / h, size / w)
    nw, nh = int(w * scale), int(h * scale)
    row_idx = (np.arange(nh) / scale).clip(0, h - 1).astype(np.int32)
    col_idx = (np.arange(nw) / scale).clip(0, w - 1).astype(np.int32)
    resized = img[np.ix_(row_idx, col_idx)]
    pad_w, pad_h = size - nw, size - nh
    top, left = pad_h // 2, pad_w // 2
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[top:top + nh, left:left + nw] = resized
    return padded, scale, (left, top)


# ── NMS ─────────────────────────────────────────────────────────

def nms(boxes, scores, iou_thresh=0.45):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[np.where(iou <= iou_thresh)[0] + 1]
    return keep


# ── NPU Benchmark (stai_mpu) ───────────────────────────────────

def bench_npu(img_rgb, num_iter=NUM_ITER, warmup=WARMUP):
    print(f"\n{'='*50}")
    print(f"  Backend: NPU (stai_mpu / OVX)")
    print(f"{'='*50}")

    try:
        from stai_mpu import stai_mpu_network
    except ImportError:
        print("  stai_mpu not available")
        return None

    net = stai_mpu_network(model_path=NPU_MODEL, use_hw_acceleration=True)
    inp_info = net.get_input_infos()[0]
    out_info = net.get_output_infos()[0]
    shape = inp_info.get_shape()
    backend = net.get_backend_engine()

    print(f"  Model: {os.path.basename(NPU_MODEL)}")
    print(f"  Backend: {backend.name}")
    print(f"  Input:  shape={shape} dtype={inp_info.get_dtype()}")
    print(f"  Output: shape={out_info.get_shape()} dtype={out_info.get_dtype()}")

    input_size = shape[1]
    print(f"  Input size: {input_size}x{input_size}")

    # preprocess: uint8 NHWC (no normalization needed for INT8 quantized model)
    padded, scale, pad = letterbox(img_rgb, input_size)
    blob = padded[np.newaxis]  # add batch dim -> (1, H, W, 3)

    # warmup
    print(f"  Warming up ({warmup} iterations)...")
    for _ in range(warmup):
        net.set_input(0, blob)
        net.run()

    # benchmark
    print(f"  Benchmarking ({num_iter} iterations)...")
    latencies = []
    mem_peaks = []
    for _ in range(num_iter):
        net.set_input(0, blob)
        t0 = time.perf_counter()
        net.run()
        latencies.append((time.perf_counter() - t0) * 1000.0)
        _, used, _ = mem_snapshot()
        mem_peaks.append(used)

    # get final output for detection test
    output = net.get_output(0)

    arr = np.array(latencies)
    stats = {
        "provider": backend.name,
        "mean_ms": float(arr.mean()),
        "std_ms": float(arr.std()),
        "min_ms": float(arr.min()),
        "max_ms": float(arr.max()),
        "p50_ms": float(np.percentile(arr, 50)),
        "p95_ms": float(np.percentile(arr, 95)),
        "p99_ms": float(np.percentile(arr, 99)),
        "fps": 1000.0 / float(arr.mean()) if arr.mean() > 0 else 0,
        "mem_peak_mb": max(mem_peaks),
    }

    print(f"\n  Results:")
    print(f"    Mean:  {stats['mean_ms']:.2f} ms")
    print(f"    Std:   {stats['std_ms']:.2f} ms")
    print(f"    Min:   {stats['min_ms']:.2f} ms")
    print(f"    Max:   {stats['max_ms']:.2f} ms")
    print(f"    P50:   {stats['p50_ms']:.2f} ms")
    print(f"    P95:   {stats['p95_ms']:.2f} ms")
    print(f"    P99:   {stats['p99_ms']:.2f} ms")
    print(f"    FPS:   {stats['fps']:.1f}")
    print(f"    Mem:   {stats['mem_peak_mb']:.0f} MB (peak during bench)")

    # test detection
    det = postprocess_npu(output, scale, pad, img_rgb.shape[1], img_rgb.shape[0])
    if det:
        print(f"    Det:   cx={det[0]:.1f} cy={det[1]:.1f} conf={det[2]:.3f}")
    else:
        print(f"    Det:   none (expected for random image)")

    return stats


def postprocess_npu(output, scale, pad, img_w, img_h, conf_thresh=0.25):
    """Post-process YOLOv8n person-only model output: (1, 5, N)."""
    # output shape: (1, 5, 2100)
    preds = output[0]  # (5, 2100)
    bbox_xywh = preds[:4, :].T  # (2100, 4)
    scores = preds[4, :]  # (2100,)
    mask = scores >= conf_thresh
    if not mask.any():
        return None
    bbox_xywh = bbox_xywh[mask]
    scores = scores[mask]
    # xywh -> xyxy
    boxes = np.empty_like(bbox_xywh)
    boxes[:, 0] = bbox_xywh[:, 0] - bbox_xywh[:, 2] / 2
    boxes[:, 1] = bbox_xywh[:, 1] - bbox_xywh[:, 3] / 2
    boxes[:, 2] = bbox_xywh[:, 0] + bbox_xywh[:, 2] / 2
    boxes[:, 3] = bbox_xywh[:, 1] + bbox_xywh[:, 3] / 2
    keep = nms(boxes, scores)
    if not keep:
        return None
    best = keep[np.argmax(scores[keep])]
    box = boxes[best]
    cx = ((box[0] + box[2]) / 2 - pad[0]) / scale
    cy = ((box[1] + box[3]) / 2 - pad[1]) / scale
    return float(np.clip(cx, 0, img_w)), float(np.clip(cy, 0, img_h)), float(scores[best])


# ── CPU Benchmark (onnxruntime) ────────────────────────────────

def bench_cpu(img_rgb, num_iter=NUM_ITER, warmup=WARMUP):
    print(f"\n{'='*50}")
    print(f"  Backend: CPU (onnxruntime)")
    print(f"{'='*50}")

    try:
        import onnxruntime as ort
    except ImportError:
        print("  onnxruntime not available")
        return None

    if not os.path.isfile(CPU_MODEL):
        print(f"  Model not found: {CPU_MODEL}")
        return None

    sess = ort.InferenceSession(CPU_MODEL, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    out = sess.get_outputs()[0]
    shape = inp.shape
    print(f"  Model: {os.path.basename(CPU_MODEL)}")
    print(f"  Provider: {sess.get_providers()[0]}")
    print(f"  Input:  {inp.name} shape={shape}")
    print(f"  Output: {out.name} shape={out.shape}")

    input_size = shape[2] if len(shape) == 4 else 640
    print(f"  Input size: {input_size}x{input_size}")

    padded, scale, pad = letterbox(img_rgb, input_size)
    blob = (padded.astype(np.float32) / 255.0).transpose(2, 0, 1)[np.newaxis]

    # warmup
    print(f"  Warming up ({warmup} iterations)...")
    for _ in range(warmup):
        sess.run([out.name], {inp.name: blob})

    # benchmark
    print(f"  Benchmarking ({num_iter} iterations)...")
    latencies = []
    mem_peaks = []
    for _ in range(num_iter):
        t0 = time.perf_counter()
        result = sess.run([out.name], {inp.name: blob})
        latencies.append((time.perf_counter() - t0) * 1000.0)
        _, used, _ = mem_snapshot()
        mem_peaks.append(used)

    arr = np.array(latencies)
    stats = {
        "provider": sess.get_providers()[0],
        "mean_ms": float(arr.mean()),
        "std_ms": float(arr.std()),
        "min_ms": float(arr.min()),
        "max_ms": float(arr.max()),
        "p50_ms": float(np.percentile(arr, 50)),
        "p95_ms": float(np.percentile(arr, 95)),
        "p99_ms": float(np.percentile(arr, 99)),
        "fps": 1000.0 / float(arr.mean()) if arr.mean() > 0 else 0,
        "mem_peak_mb": max(mem_peaks),
    }

    print(f"\n  Results:")
    print(f"    Mean:  {stats['mean_ms']:.2f} ms")
    print(f"    Std:   {stats['std_ms']:.2f} ms")
    print(f"    Min:   {stats['min_ms']:.2f} ms")
    print(f"    Max:   {stats['max_ms']:.2f} ms")
    print(f"    P50:   {stats['p50_ms']:.2f} ms")
    print(f"    P95:   {stats['p95_ms']:.2f} ms")
    print(f"    P99:   {stats['p99_ms']:.2f} ms")
    print(f"    FPS:   {stats['fps']:.1f}")
    print(f"    Mem:   {stats['mem_peak_mb']:.0f} MB (peak during bench)")

    # test detection
    output = result[0]
    preds = output[0].T
    bbox = preds[:, :4]
    scores = preds[:, 4:].max(axis=1)
    mask = scores >= 0.25
    if mask.any():
        boxes_xyxy = np.empty_like(bbox[mask])
        boxes_xyxy[:, 0] = bbox[mask][:, 0] - bbox[mask][:, 2] / 2
        boxes_xyxy[:, 1] = bbox[mask][:, 1] - bbox[mask][:, 3] / 2
        boxes_xyxy[:, 2] = bbox[mask][:, 0] + bbox[mask][:, 2] / 2
        boxes_xyxy[:, 3] = bbox[mask][:, 1] + bbox[mask][:, 3] / 2
        keep = nms(boxes_xyxy, scores[mask])
        if keep:
            best = keep[np.argmax(scores[mask][keep])]
            box = boxes_xyxy[best]
            cx = ((box[0] + box[2]) / 2 - pad[0]) / scale
            cy = ((box[1] + box[3]) / 2 - pad[1]) / scale
            print(f"    Det:   cx={cx:.1f} cy={cy:.1f} conf={scores[mask][best]:.3f}")
        else:
            print(f"    Det:   none (expected for random image)")
    else:
        print(f"    Det:   none (expected for random image)")

    return stats


# ── Xnnpack Benchmark ──────────────────────────────────────────

def bench_xnnpack(img_rgb, num_iter=NUM_ITER, warmup=WARMUP):
    print(f"\n{'='*50}")
    print(f"  Backend: XNNPACK (onnxruntime)")
    print(f"{'='*50}")

    try:
        import onnxruntime as ort
    except ImportError:
        print("  onnxruntime not available")
        return None

    available = ort.get_available_providers()
    if "XnnpackExecutionProvider" not in available:
        print("  XnnpackExecutionProvider not available")
        return None

    if not os.path.isfile(CPU_MODEL):
        print(f"  Model not found: {CPU_MODEL}")
        return None

    sess = ort.InferenceSession(
        CPU_MODEL,
        providers=["XnnpackExecutionProvider", "CPUExecutionProvider"]
    )
    inp = sess.get_inputs()[0]
    out = sess.get_outputs()[0]
    shape = inp.shape
    actual = sess.get_providers()[0]
    print(f"  Model: {os.path.basename(CPU_MODEL)}")
    print(f"  Provider: {actual}")
    print(f"  Input:  {inp.name} shape={shape}")

    input_size = shape[2] if len(shape) == 4 else 640
    print(f"  Input size: {input_size}x{input_size}")

    padded, scale, pad = letterbox(img_rgb, input_size)
    blob = (padded.astype(np.float32) / 255.0).transpose(2, 0, 1)[np.newaxis]

    # warmup
    print(f"  Warming up ({warmup} iterations)...")
    for _ in range(warmup):
        sess.run([out.name], {inp.name: blob})

    # benchmark
    print(f"  Benchmarking ({num_iter} iterations)...")
    latencies = []
    for _ in range(num_iter):
        t0 = time.perf_counter()
        sess.run([out.name], {inp.name: blob})
        latencies.append((time.perf_counter() - t0) * 1000.0)

    arr = np.array(latencies)
    stats = {
        "provider": actual,
        "mean_ms": float(arr.mean()),
        "std_ms": float(arr.std()),
        "min_ms": float(arr.min()),
        "max_ms": float(arr.max()),
        "p50_ms": float(np.percentile(arr, 50)),
        "p95_ms": float(np.percentile(arr, 95)),
        "p99_ms": float(np.percentile(arr, 99)),
        "fps": 1000.0 / float(arr.mean()) if arr.mean() > 0 else 0,
    }

    print(f"\n  Results:")
    print(f"    Mean:  {stats['mean_ms']:.2f} ms")
    print(f"    Std:   {stats['std_ms']:.2f} ms")
    print(f"    Min:   {stats['min_ms']:.2f} ms")
    print(f"    Max:   {stats['max_ms']:.2f} ms")
    print(f"    P50:   {stats['p50_ms']:.2f} ms")
    print(f"    P95:   {stats['p95_ms']:.2f} ms")
    print(f"    P99:   {stats['p99_ms']:.2f} ms")
    print(f"    FPS:   {stats['fps']:.1f}")

    return stats


# ── Report ──────────────────────────────────────────────────────

def write_report(cpu_stats, npu_stats, xnn_stats, sys_info):
    with open(REPORT_PATH, "w") as f:
        f.write("=" * 60 + "\n")
        f.write("  STM32MP257F-DK Vision Benchmark Report\n")
        f.write("=" * 60 + "\n\n")
        f.write("Board:       STM32MP257F-DK (Cortex-A35 x2 @ 1.5GHz)\n")
        f.write("NPU:         ST Neural-ART 1.35 TOPS (VeriSilicon)\n")
        f.write("RAM:         4 GB LPDDR4\n")
        f.write(f"Iterations:  {NUM_ITER} + {WARMUP} warmup\n\n")

        f.write("--- System ---\n")
        f.write(f"  Mem total:  {sys_info['total_mb']:.0f} MB\n")
        f.write(f"  Mem used:   {sys_info['used_mb']:.0f} MB ({sys_info['pct']:.1f}%)\n\n")

        for label, stats in [
            ("NPU (stai_mpu)", npu_stats),
            ("CPU (onnxruntime)", cpu_stats),
            ("XNNPACK (onnxruntime)", xnn_stats),
        ]:
            if stats is None:
                f.write(f"--- {label}: NOT AVAILABLE ---\n\n")
                continue
            f.write(f"--- {label} ---\n")
            f.write(f"  Provider:  {stats.get('provider', 'unknown')}\n")
            f.write(f"  Mean:      {float(stats.get('mean_ms', 0)):.2f} ms\n")
            f.write(f"  Std:       {float(stats.get('std_ms', 0)):.2f} ms\n")
            f.write(f"  Min:       {float(stats.get('min_ms', 0)):.2f} ms\n")
            f.write(f"  Max:       {float(stats.get('max_ms', 0)):.2f} ms\n")
            f.write(f"  P50:       {float(stats.get('p50_ms', 0)):.2f} ms\n")
            f.write(f"  P95:       {float(stats.get('p95_ms', 0)):.2f} ms\n")
            f.write(f"  P99:       {float(stats.get('p99_ms', 0)):.2f} ms\n")
            f.write(f"  FPS:       {float(stats.get('fps', 0)):.1f}\n\n")

        # feasibility
        all_stats = [("NPU", npu_stats), ("CPU", cpu_stats), ("XNNPACK", xnn_stats)]
        best_label, best = None, None
        for label, s in all_stats:
            if s:
                s_fps = float(s.get("fps", 0))
                if best is None or s_fps > float(best.get("fps", 0)):
                    best = s
                    best_label = label
        if best:
            fps = float(best["fps"])
            ros_est_mb = 400
            free_after = sys_info["total_mb"] - sys_info["used_mb"] - ros_est_mb
            feasible = fps >= 10 and free_after > 0
            f.write("--- Feasibility (best backend) ---\n")
            f.write(f"  Best backend:     {best_label} ({best.get('provider', '')})\n")
            f.write(f"  Best FPS:         {fps:.1f}\n")
            f.write(f"  Vision FPS >= 10: {'YES' if fps >= 10 else 'NO'}\n")
            f.write(f"  Free after ROS:   {free_after:.0f} MB\n")
            f.write(f"  CAN RUN FULL:     {'YES' if feasible else 'NO'}\n\n")
            if feasible:
                f.write("Conclusion: Board can run vision + kinematics + ROS locally.\n")
                if fps >= 20:
                    f.write("  Performance is excellent - real-time tracking is feasible.\n")
                elif fps >= 10:
                    f.write("  Performance is adequate - basic real-time detection works.\n")
            else:
                f.write("Conclusion: Consider offloading vision to Jetson or using smaller model.\n")

    print(f"\nReport saved to: {REPORT_PATH}")


# ── Main ───────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  STM32MP257F-DK Vision Benchmark (NPU + CPU)")
    print("=" * 60)

    # system baseline
    total, used, pct = mem_snapshot()
    print(f"\nSystem: {used:.0f}/{total:.0f} MB ({pct:.1f}% used)")
    print(f"NPU model: {NPU_MODEL}")
    print(f"CPU model: {CPU_MODEL}")

    # check available tools
    try:
        from stai_mpu import stai_mpu_network
        print("stai_mpu: AVAILABLE")
    except ImportError:
        print("stai_mpu: NOT AVAILABLE")

    try:
        import onnxruntime as ort
        print(f"onnxruntime providers: {ort.get_available_providers()}")
    except ImportError:
        print("onnxruntime: NOT AVAILABLE")

    # synthetic test image (640x480 RGB)
    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # run benchmarks
    npu_stats = bench_npu(img)
    cpu_stats = bench_cpu(img)
    xnn_stats = bench_xnnpack(img)

    # system info
    total, used, pct = mem_snapshot()
    sys_info = {"total_mb": total, "used_mb": used, "pct": pct}

    # summary
    print("\n" + "=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    for label, stats in [("NPU", npu_stats), ("CPU", cpu_stats), ("XNNPACK", xnn_stats)]:
        if stats:
            fps = float(stats.get('fps', 0))
            mean = float(stats.get('mean_ms', 0))
            print(f"  {label}: {fps:.1f} FPS ({mean:.1f} ms)")
        else:
            print(f"  {label}: N/A")
    print(f"  Memory: {used:.0f}/{total:.0f} MB")

    write_report(cpu_stats, npu_stats, xnn_stats, sys_info)


if __name__ == "__main__":
    main()
