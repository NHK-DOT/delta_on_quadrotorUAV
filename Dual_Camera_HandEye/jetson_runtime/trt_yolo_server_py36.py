#!/usr/bin/env python3
import argparse
import ctypes
import copy
import json
import os
import socket
import socketserver
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import cv2
import numpy as np
import tensorrt as trt


CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2

COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic_light",
    "fire_hydrant", "stop_sign", "parking_meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports_ball", "kite", "baseball_bat", "baseball_glove", "skateboard", "surfboard",
    "tennis_racket", "bottle", "wine_glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot_dog", "pizza", "donut", "cake", "chair", "couch",
    "potted_plant", "bed", "dining_table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell_phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy_bear", "hair_drier", "toothbrush",
]


class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True


def cuda_check(code, name):
    if code != 0:
        raise RuntimeError(f"{name} failed with CUDA error {code}")


class CudaRuntime:
    def __init__(self):
        self.lib = ctypes.CDLL("libcudart.so")

    def malloc(self, nbytes):
        ptr = ctypes.c_void_p()
        cuda_check(self.lib.cudaMalloc(ctypes.byref(ptr), ctypes.c_size_t(nbytes)), "cudaMalloc")
        return ptr

    def free(self, ptr):
        if ptr:
            cuda_check(self.lib.cudaFree(ptr), "cudaFree")

    def stream_create(self):
        stream = ctypes.c_void_p()
        cuda_check(self.lib.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def stream_destroy(self, stream):
        cuda_check(self.lib.cudaStreamDestroy(stream), "cudaStreamDestroy")

    def memcpy_async(self, dst, src, nbytes, kind, stream):
        cuda_check(
            self.lib.cudaMemcpyAsync(
                ctypes.c_void_p(dst),
                ctypes.c_void_p(src),
                ctypes.c_size_t(nbytes),
                ctypes.c_int(kind),
                stream,
            ),
            "cudaMemcpyAsync",
        )

    def stream_synchronize(self, stream):
        cuda_check(self.lib.cudaStreamSynchronize(stream), "cudaStreamSynchronize")


class TensorRTEngine:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(logger)
        data = Path(engine_path).read_bytes()
        self.engine = runtime.deserialize_cuda_engine(data)
        if self.engine is None:
            raise RuntimeError(f"Could not deserialize engine: {engine_path}")
        self.context = self.engine.create_execution_context()
        self.cuda = CudaRuntime()
        self.stream = self.cuda.stream_create()

        self.bindings = []
        self.input_index = None
        self.output_index = None
        for index in range(self.engine.num_bindings):
            shape = tuple(self.engine.get_binding_shape(index))
            dtype = trt.nptype(self.engine.get_binding_dtype(index))
            nbytes = int(np.prod(shape) * np.dtype(dtype).itemsize)
            device = self.cuda.malloc(nbytes)
            self.bindings.append({"shape": shape, "dtype": dtype, "nbytes": nbytes, "device": device})
            if self.engine.binding_is_input(index):
                self.input_index = index
            else:
                self.output_index = index

        if self.input_index is None or self.output_index is None:
            raise RuntimeError("Expected one input and one output binding")
        self.input_shape = self.bindings[self.input_index]["shape"]
        self.output_shape = self.bindings[self.output_index]["shape"]
        self.output_host = np.empty(self.output_shape, dtype=self.bindings[self.output_index]["dtype"])

    def infer(self, input_array):
        input_array = np.ascontiguousarray(input_array)
        output_array = self.output_host
        in_ptr = input_array.ctypes.data
        out_ptr = output_array.ctypes.data
        in_binding = self.bindings[self.input_index]
        out_binding = self.bindings[self.output_index]

        self.cuda.memcpy_async(in_binding["device"].value, in_ptr, in_binding["nbytes"], CUDA_MEMCPY_HOST_TO_DEVICE, self.stream)
        self.context.execute_async_v2([b["device"].value for b in self.bindings], self.stream.value)
        self.cuda.memcpy_async(out_ptr, out_binding["device"].value, out_binding["nbytes"], CUDA_MEMCPY_DEVICE_TO_HOST, self.stream)
        self.cuda.stream_synchronize(self.stream)
        return output_array.copy()


def parse_source(value):
    return int(value) if value.isdigit() else value


def letterbox(frame, size):
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x = (size - nw) // 2
    pad_y = (size - nh) // 2
    canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
    return canvas, scale, pad_x, pad_y


def preprocess(frame, size):
    image, scale, pad_x, pad_y = letterbox(frame, size)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    image = np.transpose(image, (2, 0, 1))[None, ...]
    return image, scale, pad_x, pad_y


def decode(output, frame_shape, scale, pad_x, pad_y, conf, iou):
    preds = np.squeeze(output)
    if preds.ndim != 2:
        return []
    if preds.shape[0] < preds.shape[1]:
        preds = preds.T

    frame_h, frame_w = frame_shape[:2]
    boxes = []
    scores = []
    class_ids = []
    for row in preds:
        if row.shape[0] < 5:
            continue
        class_scores = row[4:]
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])
        if score < conf:
            continue
        cx, cy, w, h = [float(x) for x in row[:4]]
        x1 = (cx - w / 2 - pad_x) / scale
        y1 = (cy - h / 2 - pad_y) / scale
        x2 = (cx + w / 2 - pad_x) / scale
        y2 = (cy + h / 2 - pad_y) / scale
        x1 = int(max(0, min(frame_w - 1, x1)))
        y1 = int(max(0, min(frame_h - 1, y1)))
        x2 = int(max(0, min(frame_w - 1, x2)))
        y2 = int(max(0, min(frame_h - 1, y2)))
        if x2 <= x1 or y2 <= y1:
            continue
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(score)
        class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, scores, conf, iou)
    if len(indices) == 0:
        return []
    return [(boxes[i], scores[i], class_ids[i]) for i in np.array(indices).flatten()]


def label_for(label_name, class_id):
    if label_name == "coco" and 0 <= int(class_id) < len(COCO_NAMES):
        return COCO_NAMES[int(class_id)]
    return label_name


def draw(frame, detections, infer_fps, label_name, depth=None):
    h, w = frame.shape[:2]
    center_x, center_y = w // 2, h // 2
    distance_m, distance_method = depth_distance(depth)
    depth_status = (depth or {}).get("status", "disabled")
    stale = (depth or {}).get("stale")

    cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 1)
    cv2.line(frame, (0, center_y), (w, center_y), (255, 255, 255), 1)
    cv2.circle(frame, (center_x, center_y), 4, (255, 255, 255), -1)
    cv2.putText(frame, f"screen center=({center_x},{center_y})", (center_x + 8, center_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
    cv2.putText(frame, f"{label_name} TensorRT fps={infer_fps:.1f}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 255), 2)
    if distance_m is not None:
        suffix = " stale" if stale else ""
        cv2.putText(frame, f"scene/center distance={distance_m:.3f}m {distance_method}{suffix}", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.64, (255, 255, 0), 2)
    else:
        cv2.putText(frame, f"distance unavailable depth={depth_status}", (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.64, (0, 165, 255), 2)
    if not detections:
        cv2.putText(frame, "not found", (12, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 255), 2)
        return frame

    for det in detections:
        box, score, class_id = det
        det_label = label_for(label_name, class_id)
        x, y, bw, bh = box
        cx = x + bw // 2
        cy = y + bh // 2
        dx = cx - center_x
        dy = cy - center_y
        item_distance_m = grid_distance_at(depth, cx, cy, w, h, box)
        if item_distance_m is None:
            item_distance_m = distance_m
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(frame, (center_x, center_y), (cx, cy), (0, 255, 255), 2)
        distance_text = f" dist={item_distance_m:.2f}m" if item_distance_m is not None else ""
        text_y = max(24, y - 8)
        cv2.putText(
            frame,
            f"{det_label} {score:.2f}{distance_text}",
            (x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"center=({cx},{cy}) offset=({dx:+d},{dy:+d})",
            (x, min(h - 8, text_y + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 255, 255),
            2,
        )
    return frame


class DepthPoller:
    def __init__(self, command, samples, interval, timeout):
        self.command = command
        self.samples = samples
        self.interval = interval
        self.timeout = timeout
        self.lock = threading.Lock()
        self.latest = {
            "enabled": bool(command),
            "ok": False,
            "status": "disabled" if not command else "starting",
            "timestamp": time.time(),
        }

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.latest)

    def _set(self, value):
        now = time.time()
        if not value.get("ok"):
            with self.lock:
                previous = copy.deepcopy(self.latest)
            if previous.get("ok") and now - previous.get("timestamp", 0) <= 10.0:
                previous["stale"] = True
                previous["stale_age_sec"] = round(now - previous.get("timestamp", now), 3)
                previous["status"] = "stale_last_ok"
                previous["timestamp"] = now
                value = previous
        value["enabled"] = bool(self.command)
        value["timestamp"] = now
        with self.lock:
            self.latest = value

    def start(self):
        if not self.command:
            return
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        env = os.environ.copy()
        lib_path = "/home/nvidia/orbbec_sdk/v1/OrbbecSDK_v1.10.35/SDK/lib"
        env["LD_LIBRARY_PATH"] = lib_path + ":" + env.get("LD_LIBRARY_PATH", "")
        while True:
            try:
                result = subprocess.run(
                    [self.command, str(self.samples)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=self.timeout,
                    env=env,
                )
                payload = None
                for line in reversed(result.stdout.splitlines()):
                    line = line.strip()
                    if line.startswith("{") and line.endswith("}"):
                        payload = json.loads(line)
                        break
                if payload is None:
                    payload = {"ok": False, "status": "no_json", "returncode": result.returncode}
                else:
                    payload["status"] = "ok" if payload.get("ok") else "no_valid_center_depth"
                    payload["returncode"] = result.returncode
                self._set(payload)
            except Exception as exc:
                self._set({"ok": False, "status": "error", "error": repr(exc)})
            time.sleep(self.interval)


class DepthFileReader:
    def __init__(self, path, max_age):
        self.path = Path(path) if path else None
        self.max_age = max_age
        self.lock = threading.Lock()
        self.latest = {
            "enabled": bool(path),
            "ok": False,
            "status": "disabled" if not path else "starting",
            "timestamp": time.time(),
        }

    def snapshot(self):
        if not self.path:
            with self.lock:
                return copy.deepcopy(self.latest)
        try:
            payload = json.loads(self.path.read_text())
            now = time.time()
            age = now - float(payload.get("timestamp", 0))
            payload["enabled"] = True
            payload["age_sec"] = round(age, 3)
            if age > self.max_age:
                payload["ok"] = False
                payload["status"] = "stale_depth_grid"
            elif payload.get("ok"):
                payload["status"] = "ok"
            else:
                with self.lock:
                    previous = copy.deepcopy(self.latest)
                if previous.get("ok") and now - float(previous.get("timestamp", 0)) <= max(5.0, self.max_age):
                    previous["status"] = "stale_last_ok"
                    previous["stale"] = True
                    previous["stale_age_sec"] = round(now - float(previous.get("timestamp", now)), 3)
                    previous["timestamp"] = now
                    payload = previous
            with self.lock:
                self.latest = payload
        except Exception as exc:
            with self.lock:
                previous = copy.deepcopy(self.latest)
            now = time.time()
            if previous.get("ok") and now - float(previous.get("timestamp", 0)) <= self.max_age:
                previous["status"] = "stale_last_ok"
                previous["stale"] = True
                previous["stale_age_sec"] = round(now - float(previous.get("timestamp", now)), 3)
                with self.lock:
                    self.latest = previous
            else:
                with self.lock:
                    self.latest = {
                        "enabled": True,
                        "ok": False,
                        "status": "read_error",
                        "error": repr(exc),
                        "timestamp": now,
                    }
        with self.lock:
            return copy.deepcopy(self.latest)


def grid_distance_at(depth, rgb_x, rgb_y, rgb_w, rgb_h, box=None):
    if not depth or not depth.get("ok") or not depth.get("grid_mm"):
        return None
    grid_w = int(depth.get("grid_w", 0))
    grid_h = int(depth.get("grid_h", 0))
    if grid_w <= 0 or grid_h <= 0:
        return None

    if box:
        x, y, bw, bh = [int(v) for v in box]
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(rgb_w - 1, x + bw)
        y2 = min(rgb_h - 1, y + bh)
        gx1 = max(0, min(grid_w - 1, int(x1 * grid_w / max(1, rgb_w))))
        gx2 = max(0, min(grid_w - 1, int(x2 * grid_w / max(1, rgb_w))))
        gy1 = max(0, min(grid_h - 1, int(y1 * grid_h / max(1, rgb_h))))
        gy2 = max(0, min(grid_h - 1, int(y2 * grid_h / max(1, rgb_h))))
    else:
        gx = max(0, min(grid_w - 1, int(rgb_x * grid_w / max(1, rgb_w))))
        gy = max(0, min(grid_h - 1, int(rgb_y * grid_h / max(1, rgb_h))))
        gx1, gx2 = max(0, gx - 1), min(grid_w - 1, gx + 1)
        gy1, gy2 = max(0, gy - 1), min(grid_h - 1, gy + 1)

    values = []
    grid = depth.get("grid_mm") or []
    for gy in range(min(gy1, gy2), max(gy1, gy2) + 1):
        for gx in range(min(gx1, gx2), max(gx1, gx2) + 1):
            idx = gy * grid_w + gx
            if 0 <= idx < len(grid):
                value = float(grid[idx])
                if value > 0:
                    values.append(value)
    if not values:
        return None
    values.sort()
    return round(values[len(values) // 2] / 1000.0, 3)


def depth_distance(depth):
    if not depth or not depth.get("ok"):
        return None, "unavailable"
    if depth.get("grid_mm"):
        distance = grid_distance_at(depth, depth.get("width", 0) / 2, depth.get("height", 0) / 2, depth.get("width", 1), depth.get("height", 1))
        if distance is not None:
            return distance, "depth_grid_center"
    if depth.get("center_distance_m") is not None:
        return round(float(depth["center_distance_m"]), 3), "depth_center_roi"
    if depth.get("full_avg_mm") is not None:
        return round(float(depth["full_avg_mm"]) / 1000.0, 3), "depth_frame_average"
    return None, "unavailable"


def camera_position_m(cx, cy, distance_m, image_w, image_h, args):
    if distance_m is None:
        return None
    fx = args.camera_fx
    fy = args.camera_fy
    px = args.camera_cx if args.camera_cx is not None else image_w / 2.0
    py = args.camera_cy if args.camera_cy is not None else image_h / 2.0
    method = "pinhole_intrinsics"
    if fx <= 0 or fy <= 0:
        if args.camera_hfov_deg <= 0 or args.camera_vfov_deg <= 0:
            return None
        fx = image_w / (2.0 * np.tan(np.deg2rad(args.camera_hfov_deg) / 2.0))
        fy = image_h / (2.0 * np.tan(np.deg2rad(args.camera_vfov_deg) / 2.0))
        method = "pinhole_fov_estimate"
    z = float(distance_m)
    x = (float(cx) - float(px)) * z / float(fx)
    y = (float(cy) - float(py)) * z / float(fy)
    return {
        "x": round(x, 4),
        "y": round(y, 4),
        "z": round(z, 4),
        "frame": "camera",
        "axis": {"x": "right", "y": "down", "z": "forward"},
        "method": method,
        "intrinsics": {
            "fx": round(float(fx), 3),
            "fy": round(float(fy), 3),
            "cx": round(float(px), 3),
            "cy": round(float(py), 3),
        },
    }


def build_message(detections, frame_shape, infer_fps, label_name, source, depth=None, args=None):
    h, w = frame_shape[:2]
    center_x, center_y = w // 2, h // 2
    items = []
    distance_m, distance_method = depth_distance(depth)
    for det in detections:
        box, score, class_id = det
        det_label = label_for(label_name, class_id)
        x, y, bw, bh = [int(v) for v in box]
        cx = x + bw // 2
        cy = y + bh // 2
        item_distance_m = grid_distance_at(depth, cx, cy, w, h, box)
        item_distance_method = "depth_grid_box" if item_distance_m is not None else distance_method
        if item_distance_m is None:
            item_distance_m = distance_m
        item = {
            "class": det_label,
            "class_id": int(class_id),
            "conf": round(float(score), 4),
            "box": {"x": x, "y": y, "w": bw, "h": bh},
            "center": {"x": cx, "y": cy},
            "offset": {"dx": cx - center_x, "dy": cy - center_y},
        }
        if item_distance_m is not None:
            item["distance_m"] = item_distance_m
            item["distance_method"] = item_distance_method
            position = camera_position_m(cx, cy, item_distance_m, w, h, args) if args is not None else None
            if position is not None:
                item["position_camera_m"] = position
        items.append(item)
    message = {
        "valid": bool(items),
        "status": "ok",
        "source": str(source),
        "timestamp": time.time(),
        "image": {"w": int(w), "h": int(h), "center_x": int(center_x), "center_y": int(center_y)},
        "fps": round(float(infer_fps), 2),
        "target": items[0] if items else None,
        "detections": items,
    }
    if depth is not None:
        public_depth = copy.deepcopy(depth)
        public_depth.pop("grid_mm", None)
        message["depth"] = public_depth
        if distance_m is not None:
            message["distance_m"] = distance_m
            message["distance_method"] = distance_method
    return message


class TargetSmoother:
    def __init__(self, alpha):
        self.alpha = max(0.0, min(1.0, float(alpha)))
        self.target = None

    @staticmethod
    def _blend(old, new, alpha):
        if old is None or new is None:
            return new
        return (1.0 - alpha) * float(old) + alpha * float(new)

    @staticmethod
    def _round(value, digits=4):
        return round(float(value), digits) if value is not None else None

    def update(self, message):
        if self.alpha <= 0:
            return message
        raw = message.get("target")
        if not raw:
            self.target = None
            return message

        old = self.target or {}
        raw_center = raw.get("center") or {}
        raw_offset = raw.get("offset") or {}
        old_center = old.get("center") or {}
        old_offset = old.get("offset") or {}
        smoothed = copy.deepcopy(raw)
        smoothed["center"] = {
            "x": self._round(self._blend(old_center.get("x"), raw_center.get("x"), self.alpha), 2),
            "y": self._round(self._blend(old_center.get("y"), raw_center.get("y"), self.alpha), 2),
        }
        smoothed["offset"] = {
            "dx": self._round(self._blend(old_offset.get("dx"), raw_offset.get("dx"), self.alpha), 2),
            "dy": self._round(self._blend(old_offset.get("dy"), raw_offset.get("dy"), self.alpha), 2),
        }
        smoothed["distance_m"] = self._round(
            self._blend(old.get("distance_m"), raw.get("distance_m"), self.alpha),
            4,
        )
        raw_position = raw.get("position_camera_m") or {}
        old_position = old.get("position_camera_m") or {}
        if raw_position:
            position = copy.deepcopy(raw_position)
            for axis in ("x", "y", "z"):
                position[axis] = self._round(
                    self._blend(old_position.get(axis), raw_position.get(axis), self.alpha),
                    4,
                )
            smoothed["position_camera_m"] = position
        smoothed["source"] = "ema"
        smoothed["alpha"] = self.alpha
        smoothed["raw_center"] = raw_center
        smoothed["raw_offset"] = raw_offset
        self.target = smoothed
        message["target_smoothed"] = smoothed
        return message


class VisionState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {
            "valid": False,
            "status": "starting",
            "timestamp": time.time(),
            "target": None,
            "detections": [],
        }
        self.jpeg = None

    def set_error(self, status):
        with self.lock:
            self.latest = {
                "valid": False,
                "status": status,
                "timestamp": time.time(),
                "target": None,
                "detections": [],
            }

    def update(self, message, jpeg):
        with self.lock:
            self.latest = message
            self.jpeg = jpeg

    def get_latest(self):
        with self.lock:
            return copy.deepcopy(self.latest)

    def get_jpeg(self):
        with self.lock:
            return self.jpeg


class UdpPublisher:
    def __init__(self, host, port, rate_hz):
        self.enabled = bool(host and port)
        self.addr = (host, port) if self.enabled else None
        self.min_interval = 1.0 / rate_hz if rate_hz and rate_hz > 0 else 0.0
        self.last_sent = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.enabled else None

    def maybe_send(self, message):
        if not self.enabled:
            return
        now = time.time()
        if self.min_interval and now - self.last_sent < self.min_interval:
            return
        payload = (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")
        try:
            self.sock.sendto(payload, self.addr)
            self.last_sent = now
        except OSError as exc:
            message["udp_error"] = repr(exc)
            self.last_sent = now


def start_vision_worker(args):
    state = VisionState()
    detector = TensorRTEngine(args.engine)
    _, channels, input_h, input_w = detector.input_shape
    if input_h != input_w:
        raise RuntimeError(f"Expected square input, got {detector.input_shape}")
    input_size = input_h
    publisher = UdpPublisher(args.udp_host, args.udp_port, args.udp_rate)
    smoother = TargetSmoother(args.target_smooth_alpha)
    if args.depth_json:
        depth = DepthFileReader(args.depth_json, args.depth_max_age)
    else:
        depth = DepthPoller(args.depth_command, args.depth_samples, args.depth_interval, args.depth_timeout)
        depth.start()

    def run():
        source = parse_source(args.source)
        smoothed_fps = 0.0
        frame_interval = 1.0 / args.infer_fps if args.infer_fps else 0.0
        while True:
            cap = cv2.VideoCapture(source, cv2.CAP_V4L2 if isinstance(source, int) else cv2.CAP_ANY)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_FPS, args.camera_fps)
            if args.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
            if not cap.isOpened():
                state.set_error("camera_open_failed")
                time.sleep(1.0)
                continue

            try:
                while True:
                    started = time.time()
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        state.set_error("camera_read_failed")
                        time.sleep(0.05)
                        continue
                    inp, scale, pad_x, pad_y = preprocess(frame, input_size)
                    output = detector.infer(inp)
                    detections = decode(output, frame.shape, scale, pad_x, pad_y, args.conf, args.iou)
                    if args.max_detections > 0:
                        detections = sorted(detections, key=lambda item: item[1], reverse=True)[: args.max_detections]
                    elapsed = max(1e-6, time.time() - started)
                    instant_fps = 1.0 / elapsed
                    smoothed_fps = 0.85 * smoothed_fps + 0.15 * instant_fps if smoothed_fps else instant_fps
                    depth_info = depth.snapshot()
                    message = build_message(detections, frame.shape, smoothed_fps, args.label, args.source, depth_info, args)
                    message = smoother.update(message)
                    annotated = draw(frame, detections, smoothed_fps, args.label, depth_info)
                    ok, jpg = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
                    if ok:
                        state.update(message, jpg.tobytes())
                        publisher.maybe_send(message)
                    spent = time.time() - started
                    if frame_interval > spent:
                        time.sleep(frame_interval - spent)
            except Exception as exc:
                state.set_error(f"worker_error:{type(exc).__name__}:{exc}")
                time.sleep(1.0)
            finally:
                cap.release()

    thread = threading.Thread(target=run, name="vision-worker", daemon=True)
    thread.start()
    return state


def send_json(handler, code, payload):
    body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Cache-Control", "no-cache")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def make_handler(args, state):

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(
                    b"<!doctype html><title>Jetson TensorRT YOLO</title>"
                    b"<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
                    b"main{display:grid;place-items:center;min-height:100vh}"
                    b"a{position:fixed;left:12px;top:12px;color:#8fd;text-decoration:none}"
                    b"img{max-width:100vw;max-height:100vh}</style>"
                    b"<a href='/latest.json'>latest.json</a>"
                    b"<main><img src='/stream.mjpg' alt='TensorRT YOLO stream'></main>"
                )
                return
            if self.path == "/latest.json":
                send_json(self, 200, state.get_latest())
                return
            if self.path == "/healthz":
                latest = state.get_latest()
                age = time.time() - latest.get("timestamp", 0)
                send_json(self, 200 if age < 3 else 503, {"status": latest.get("status"), "age_sec": round(age, 3)})
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            frame_interval = 1.0 / args.display_fps if args.display_fps else 0.0
            try:
                while True:
                    started = time.time()
                    payload = state.get_jpeg()
                    if payload is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    spent = time.time() - started
                    if frame_interval > spent:
                        time.sleep(frame_interval - spent)
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass

        def log_message(self, fmt, *args):
            return

    return Handler


def main():
    parser = argparse.ArgumentParser(description="TensorRT YOLO MJPEG preview.")
    parser.add_argument("--engine", default="models/snow_king_320_fp16.engine")
    parser.add_argument("--source", default="0")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--display-fps", type=float, default=20)
    parser.add_argument("--infer-fps", type=float, default=30)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-detections", type=int, default=0, help="Limit detections after NMS; 0 keeps all.")
    parser.add_argument("--capture-thread", action="store_true", help="Accepted for compatibility with threaded capture builds.")
    parser.add_argument("--quality", type=int, default=70)
    parser.add_argument("--label", default="snow_king")
    parser.add_argument("--udp-host", default="", help="Optional STM32MP257 UDP target IP.")
    parser.add_argument("--udp-port", type=int, default=0, help="Optional STM32MP257 UDP target port.")
    parser.add_argument("--udp-rate", type=float, default=10, help="Max UDP messages per second.")
    parser.add_argument("--depth-command", default="", help="Optional Orbbec depth probe executable.")
    parser.add_argument("--depth-json", default="", help="Optional live Orbbec depth grid JSON path.")
    parser.add_argument("--depth-max-age", type=float, default=2.0, help="Max depth JSON age in seconds.")
    parser.add_argument("--depth-samples", type=int, default=20, help="Depth frames per probe.")
    parser.add_argument("--depth-interval", type=float, default=0.2, help="Delay between depth probes.")
    parser.add_argument("--depth-timeout", type=float, default=8.0, help="Depth probe timeout in seconds.")
    parser.add_argument("--camera-fx", type=float, default=0.0, help="RGB camera focal length in pixels on x axis.")
    parser.add_argument("--camera-fy", type=float, default=0.0, help="RGB camera focal length in pixels on y axis.")
    parser.add_argument("--camera-cx", type=float, default=None, help="RGB camera principal point x in pixels.")
    parser.add_argument("--camera-cy", type=float, default=None, help="RGB camera principal point y in pixels.")
    parser.add_argument("--camera-hfov-deg", type=float, default=0.0, help="Fallback horizontal FOV in degrees for approximate camera coordinates.")
    parser.add_argument("--camera-vfov-deg", type=float, default=0.0, help="Fallback vertical FOV in degrees for approximate camera coordinates.")
    parser.add_argument("--target-smooth-alpha", type=float, default=0.35, help="EMA alpha for optional target_smoothed control output; 0 disables.")
    args = parser.parse_args()

    state = start_vision_worker(args)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args, state))
    print(f"Serving TensorRT YOLO preview on http://{args.host}:{args.port}", flush=True)
    print(f"Detection JSON endpoint: http://{args.host}:{args.port}/latest.json", flush=True)
    if args.udp_host and args.udp_port:
        print(f"Publishing UDP detection JSON to {args.udp_host}:{args.udp_port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
