#!/usr/bin/env python3
"""Standalone vision benchmark for STM32MP257F-DK.

No ROS 2 dependency. Tests YOLOv8n ONNX inference on both CPU and NPU.
"""

import os
import sys
import time
import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    print("ERROR: onnxruntime not installed. pip3 install onnxruntime")
    sys.exit(1)

MODEL_PATH = "/usr/local/Uav_Delta_capture/models/yolov8n_320.onnx"
INPUT_SIZE = None  # auto-detect from model
NUM_ITER = 100
WARMUP = 10
REPORT_PATH = "/tmp/vision_bench_report.txt"


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


def read_cpu():
    with open("/proc/stat") as f:
        parts = f.readline().split()
    vals = [int(x) for x in parts[1:]]
    return vals[3] + (vals[4] if len(vals) > 4 else 0), sum(vals)


def mem_snapshot():
    m = read_meminfo()
    total = m.get("MemTotal", 0) / 1024.0
    avail = m.get("MemAvailable", 0) / 1024.0
    return total, total - avail, (total - avail) / total * 100 if total else 0


# ── Pre/post processing ────────────────────────────────────────

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


def preprocess(img_rgb, size=640):
    padded, scale, pad = letterbox(img_rgb, size)
    blob = padded.astype(np.float32) / 255.0
    blob = blob.transpose(2, 0, 1)[np.newaxis]
    return blob, scale, pad


def xywh2xyxy(x):
    y = np.empty_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


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


def postprocess(output, scale, pad, img_w, img_h, conf_thresh=0.25):
    preds = output[0].T
    bbox = preds[:, :4]
    scores = preds[:, 4:].max(axis=1)
    mask = scores >= conf_thresh
    if not mask.any():
        return None
    bbox, scores = bbox[mask], scores[mask]
    boxes = xywh2xyxy(bbox)
    keep = nms(boxes, scores)
    if not keep:
        return None
    best = keep[np.argmax(scores[keep])]
    box = boxes[best]
    cx = ((box[0] + box[2]) / 2 - pad[0]) / scale
    cy = ((box[1] + box[3]) / 2 - pad[1]) / scale
    return float(np.clip(cx, 0, img_w)), float(np.clip(cy, 0, img_h)), float(scores[best])


# ── Benchmark ──────────────────────────────────────────────────

def run_bench(provider_name, providers, img_rgb, model_path, input_size=None):
    print(f"\n{'='*50}")
    print(f"  Backend: {provider_name}")
    print(f"{'='*50}")

    try:
        sess = ort.InferenceSession(model_path, providers=providers)
    except Exception as e:
        print(f"  FAILED to load model: {e}")
        return None

    actual = sess.get_providers()[0]
    print(f"  Actual provider: {actual}")
    inp = sess.get_inputs()[0]
    out = sess.get_outputs()[0]
    print(f"  Input:  {inp.name} {inp.shape}")
    print(f"  Output: {out.name} {out.shape}")

    # auto-detect input size from model
    shape = inp.shape
    if input_size is None and len(shape) == 4:
        input_size = shape[2]  # NCHW
    if input_size is None:
        input_size = 640
    print(f"  Input size: {input_size}x{input_size}")

    blob, scale, pad = preprocess(img_rgb, input_size)

    # warmup
    print(f"  Warming up ({WARMUP} iterations)...")
    for _ in range(WARMUP):
        sess.run([out.name], {inp.name: blob})

    # benchmark
    print(f"  Benchmarking ({NUM_ITER} iterations)...")
    latencies = []
    mem_peaks = []
    for _ in range(NUM_ITER):
        t0 = time.perf_counter()
        result = sess.run([out.name], {inp.name: blob})
        latencies.append((time.perf_counter() - t0) * 1000.0)
        _, used, _ = mem_snapshot()
        mem_peaks.append(used)

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

    # test detection on synthetic image
    det = postprocess(result[0], scale, pad, img_rgb.shape[1], img_rgb.shape[0])
    if det:
        print(f"    Det:   cx={det[0]:.1f} cy={det[1]:.1f} conf={det[2]:.3f}")
    else:
        print(f"    Det:   none (expected for random image)")

    return stats


def write_report(cpu_stats, npu_stats, sys_info):
    with open(REPORT_PATH, "w") as f:
        f.write("=" * 60 + "\n")
        f.write("  STM32MP257F-DK Vision Benchmark Report\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"Board:       STM32MP257F-DK (Cortex-A35 x2 @ 1.5GHz)\n")
        f.write(f"Model:       {os.path.basename(MODEL_PATH)}\n")
        f.write(f"Input size:  auto-detected from model\n")
        f.write(f"Iterations:  {NUM_ITER} + {WARMUP} warmup\n\n")

        f.write("--- System ---\n")
        f.write(f"  Mem total:  {sys_info['total_mb']:.0f} MB\n")
        f.write(f"  Mem used:   {sys_info['used_mb']:.0f} MB ({sys_info['pct']:.1f}%)\n\n")

        for label, stats in [("CPU", cpu_stats), ("NPU", npu_stats)]:
            if stats is None:
                f.write(f"--- {label}: NOT AVAILABLE ---\n\n")
                continue
            f.write(f"--- {label} Inference ---\n")
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
        best = None
        for s in [npu_stats, cpu_stats]:
            if s:
                s_fps = float(s.get("fps", 0))
                if best is None or s_fps > float(best.get("fps", 0)):
                    best = s
        if best:
            fps = best["fps"]
            ros_est_mb = 400
            free_after = sys_info["total_mb"] - sys_info["used_mb"] - ros_est_mb
            feasible = fps >= 10 and free_after > 0
            f.write("--- Feasibility (best backend) ---\n")
            f.write(f"  Best FPS:         {fps:.1f} ({best['provider']})\n")
            f.write(f"  Vision FPS >= 10: {'YES' if fps >= 10 else 'NO'}\n")
            f.write(f"  Free after ROS:   {free_after:.0f} MB\n")
            f.write(f"  CAN RUN FULL:     {'YES' if feasible else 'NO'}\n")
            if feasible:
                f.write("\nConclusion: Board can run vision + kinematics + ROS.\n")
            else:
                f.write("\nConclusion: Offload vision to Jetson or use smaller model.\n")

    print(f"\nReport saved to: {REPORT_PATH}")


# ── Main ───────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  STM32MP257F-DK Vision Benchmark")
    print("=" * 60)

    if not os.path.isfile(MODEL_PATH):
        print(f"ERROR: Model not found: {MODEL_PATH}")
        sys.exit(1)

    # system baseline
    total, used, pct = mem_snapshot()
    print(f"\nSystem: {used:.0f}/{total:.0f} MB ({pct:.1f}% used)")
    print(f"Model:  {MODEL_PATH} ({os.path.getsize(MODEL_PATH)/1024/1024:.1f} MB)")
    print(f"Providers available: {ort.get_available_providers()}")

    # synthetic test image (640x480 RGB)
    img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # CPU benchmark
    cpu_stats = run_bench("CPU", ["CPUExecutionProvider"], img, MODEL_PATH, INPUT_SIZE)

    # NPU benchmark (separate process to isolate segfaults)
    npu_stats = None
    available = ort.get_available_providers()
    if "VSINPUExecutionProvider" in available:
        print("\n  NPU: Testing in isolated subprocess...")
        import subprocess
        npu_script = '''
import onnxruntime as ort, numpy as np, time, sys, os
MODEL_PATH = sys.argv[1]
try:
    sess = ort.InferenceSession(MODEL_PATH, providers=["VSINPUExecutionProvider", "CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    out = sess.get_outputs()[0]
    shape = inp.shape
    sz = shape[2] if len(shape)==4 and shape[2] else 640
    img = np.random.randint(0,255,(480,640,3),dtype=np.uint8)
    h,w = img.shape[:2]
    sc = min(sz/h,sz/w)
    nw,nh = int(w*sc),int(h*sc)
    ri = (np.arange(nh)/sc).clip(0,h-1).astype(np.int32)
    ci = (np.arange(nw)/sc).clip(0,w-1).astype(np.int32)
    rsz = img[np.ix_(ri,ci)]
    pw,ph = sz-nw,sz-nh
    t,l = ph//2,pw//2
    pad = np.full((sz,sz,3),114,dtype=np.uint8)
    pad[t:t+nh,l:l+nw] = rsz
    blob = (pad.astype(np.float32)/255.0).transpose(2,0,1)[np.newaxis]
    # warmup
    for _ in range(5):
        sess.run([out.name],{inp.name:blob})
    # bench
    lats = []
    for _ in range(50):
        t0 = time.perf_counter()
        sess.run([out.name],{inp.name:blob})
        lats.append((time.perf_counter()-t0)*1000.0)
    a = np.array(lats)
    print(f"PROVIDER={sess.get_providers()[0]}")
    print(f"MEAN={a.mean():.2f}")
    print(f"STD={a.std():.2f}")
    print(f"MIN={a.min():.2f}")
    print(f"MAX={a.max():.2f}")
    print(f"P50={np.percentile(a,50):.2f}")
    print(f"P95={np.percentile(a,95):.2f}")
    print(f"P99={np.percentile(a,99):.2f}")
    print(f"FPS={1000.0/a.mean():.1f}")
    print("NPU_OK")
except Exception as e:
    print(f"NPU_FAIL={e}")
'''
        try:
            proc = subprocess.run(
                ["python3", "-c", npu_script, MODEL_PATH],
                capture_output=True, text=True, timeout=300
            )
            out_text = proc.stdout + proc.stderr
            print(f"  NPU output: {out_text.strip()}")
            if "NPU_OK" in out_text:
                lines = out_text.strip().split("\n")
                npu_stats = {}
                for line in lines:
                    if "=" in line:
                        k, v = line.split("=", 1)
                        try:
                            npu_stats[k.lower()] = float(v)
                        except ValueError:
                            npu_stats[k.lower()] = v
                npu_stats["provider"] = npu_stats.get("provider", "VSINPUExecutionProvider")
        except subprocess.TimeoutExpired:
            print("  NPU: Timed out")
        except Exception as e:
            print(f"  NPU: Error - {e}")
    else:
        print("\nNPU: VSINPUExecutionProvider not available, skipping.")
        npu_stats = None

    # system info
    total, used, pct = mem_snapshot()
    sys_info = {"total_mb": total, "used_mb": used, "pct": pct}

    # summary
    print("\n" + "=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    for label, stats in [("CPU", cpu_stats), ("NPU", npu_stats)]:
        if stats:
            fps = stats.get('fps', 0)
            mean = stats.get('mean_ms', 0)
            print(f"  {label}: {float(fps):.1f} FPS ({float(mean):.1f} ms)")
        else:
            print(f"  {label}: N/A")
    print(f"  Memory: {used:.0f}/{total:.0f} MB")

    write_report(cpu_stats, npu_stats, sys_info)


if __name__ == "__main__":
    main()
