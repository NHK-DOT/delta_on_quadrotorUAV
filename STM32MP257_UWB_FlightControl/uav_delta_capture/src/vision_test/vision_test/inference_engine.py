"""Vision inference engine with dual backend support.

Backends:
- stai_mpu: ST X-LINUX-AI NPU runtime for .nb models (Neural-ART Binary)
- onnxruntime: CPU/NPU inference for .onnx models

Auto-selects backend based on model file extension:
- .nb → stai_mpu with NPU hardware acceleration
- .onnx → onnxruntime with CPUExecutionProvider

Post-processing auto-detects output shape:
- (1, 5, N) → person-only model (4 bbox + 1 confidence)
- (1, 84, N) → 80-class COCO YOLOv8 (4 bbox + 80 class scores)
"""

import os
import time
from typing import Optional, Tuple

import numpy as np

try:
    from stai_mpu import stai_mpu_network
    HAS_STAI_MPU = True
except ImportError:
    HAS_STAI_MPU = False

try:
    import onnxruntime as ort
    HAS_ORT = True
except ImportError:
    HAS_ORT = False

DEFAULT_NB_MODEL = "/usr/local/x-linux-ai/people-tracking-heatmap/models/yolov8n_people/yolov8n_320_quant_pt_uf_od_coco-person-st.nb"
DEFAULT_ONNX_MODEL = "/usr/local/Uav_Delta_capture/models/yolov8n.onnx"
SSD_LABEL_FILE = "/usr/local/x-linux-ai/object-detection/models/coco_ssd_mobilenet/labels_coco_dataset_80.txt"

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


def _letterbox(
    img: np.ndarray, new_shape: int = 320
) -> Tuple[np.ndarray, float, Tuple[int, int]]:
    """Resize with padding, returns (padded_img, scale, (pad_w, pad_h))."""
    h, w = img.shape[:2]
    scale = min(new_shape / h, new_shape / w)
    new_w, new_h = int(w * scale), int(h * scale)
    if scale != 1.0:
        row_idx = (np.arange(new_h) / scale).astype(np.int32).clip(0, h - 1)
        col_idx = (np.arange(new_w) / scale).astype(np.int32).clip(0, w - 1)
        resized = img[np.ix_(row_idx, col_idx)]
    else:
        resized = img.copy()
    pad_w = new_shape - new_w
    pad_h = new_shape - new_h
    top, left = pad_h // 2, pad_w // 2
    padded = np.full((new_shape, new_shape, 3), 114, dtype=np.uint8)
    padded[top : top + new_h, left : left + new_w] = resized
    return padded, scale, (left, top)


def _xywh2xyxy(x: np.ndarray) -> np.ndarray:
    y = np.empty_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y


def _load_labels(path: str) -> list:
    """Load label file (one label per line, line 0 = background)."""
    try:
        with open(path) as f:
            return [line.strip() for line in f.readlines()]
    except FileNotFoundError:
        return []


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thresh: float = 0.45) -> list:
    """Pure numpy NMS."""
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
        inds = np.where(iou <= iou_thresh)[0]
        order = order[inds + 1]
    return keep


class InferenceEngine:
    """Vision inference engine supporting stai_mpu (NPU) and onnxruntime (CPU)."""

    def __init__(
        self,
        model_path: str = "",
        input_size: int = 320,
        use_npu: bool = True,
        conf_thresh: float = 0.25,
        iou_thresh: float = 0.45,
        logger=None,
    ):
        self.input_size = input_size
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.logger = logger
        self.backend_type = None  # "stai_mpu" or "onnxruntime"

        # resolve model path
        model_path = self._resolve_model(model_path)

        ext = os.path.splitext(model_path)[1].lower()

        if ext == ".nb":
            self._init_stai_mpu(model_path)
        else:
            self._init_onnxruntime(model_path, use_npu)

    def _resolve_model(self, model_path: str) -> str:
        """Resolve model path with fallback."""
        if model_path and os.path.isfile(model_path):
            return model_path
        # fallback: try .nb model first, then .onnx
        for candidate in [DEFAULT_NB_MODEL, DEFAULT_ONNX_MODEL]:
            if os.path.isfile(candidate):
                if self.logger:
                    self.logger.info(f"Using default model: {candidate}")
                return candidate
        raise FileNotFoundError(
            f"Model not found: {model_path}. "
            f"Provide a valid path or place a model at {DEFAULT_NB_MODEL}"
        )

    def _init_stai_mpu(self, model_path: str):
        """Initialize stai_mpu NPU backend for .nb models."""
        if not HAS_STAI_MPU:
            raise RuntimeError(
                "stai_mpu not installed. Install X-LINUX-AI package."
            )

        self.stai_model = stai_mpu_network(
            model_path=model_path, use_hw_acceleration=True
        )
        self.backend_type = "stai_mpu"

        inp_info = self.stai_model.get_input_infos()[0]
        shape = inp_info.get_shape()
        backend = self.stai_model.get_backend_engine()

        self.input_size = shape[1]  # NHWC format
        self.provider_used = backend.name
        self._input_dtype = inp_info.get_dtype()
        self._num_outputs = len(self.stai_model.get_output_infos())
        self._output_shapes = [o.get_shape() for o in self.stai_model.get_output_infos()]

        if self.logger:
            self.logger.info(
                f"Model loaded (stai_mpu): {os.path.basename(model_path)} "
                f"input={shape} outputs={self._output_shapes} backend={self.provider_used}"
            )

    def _init_onnxruntime(self, model_path: str, use_npu: bool):
        """Initialize onnxruntime backend for .onnx models."""
        if not HAS_ORT:
            raise RuntimeError(
                "onnxruntime not installed. pip install onnxruntime"
            )

        available = ort.get_available_providers()
        providers = []
        if use_npu and "VSINPUExecutionProvider" in available:
            providers.append("VSINPUExecutionProvider")
            if self.logger:
                self.logger.info("Using NPU backend (VSINPUExecutionProvider)")
        elif use_npu and self.logger:
            self.logger.warn(
                "NPU requested but VSINPUExecutionProvider not available, "
                "falling back to CPU"
            )
        providers.append("CPUExecutionProvider")

        self.ort_session = ort.InferenceSession(model_path, providers=providers)
        self.backend_type = "onnxruntime"

        inp = self.ort_session.get_inputs()[0]
        out = self.ort_session.get_outputs()[0]
        self.ort_input_name = inp.name
        self.ort_output_name = out.name
        self.provider_used = self.ort_session.get_providers()[0]
        self._output_shape = out.shape

        # update input_size from model if dynamic
        shape = inp.shape
        if len(shape) == 4 and isinstance(shape[2], int):
            self.input_size = shape[2]

        if self.logger:
            self.logger.info(
                f"Model loaded (onnxruntime): {os.path.basename(model_path)} "
                f"input={shape} output={out.shape} provider={self.provider_used}"
            )

    def preprocess(self, img_rgb: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        """Letterbox resize. Returns (blob, scale, pad).

        stai_mpu: uint8 NHWC (no normalization needed for INT8 quantized models)
        onnxruntime: float32 NCHW (normalized to [0,1])
        """
        padded, scale, pad = _letterbox(img_rgb, self.input_size)

        if self.backend_type == "stai_mpu":
            # stai_mpu expects uint8 NHWC with batch dim
            blob = np.expand_dims(padded, axis=0)
        else:
            # onnxruntime expects float32 NCHW
            blob = padded.astype(np.float32) / 255.0
            blob = blob.transpose(2, 0, 1)
            blob = np.expand_dims(blob, 0)

        return blob, scale, pad

    def _run_inference(self, blob: np.ndarray) -> list:
        """Run inference on the loaded model. Returns list of raw outputs."""
        if self.backend_type == "stai_mpu":
            self.stai_model.set_input(0, blob)
            self.stai_model.run()
            return [self.stai_model.get_output(i) for i in range(self._num_outputs)]
        else:
            results = self.ort_session.run(None, {self.ort_input_name: blob})
            return results

    def postprocess(
        self,
        outputs: list,
        scale: float,
        pad: Tuple[int, int],
        img_w: int,
        img_h: int,
    ) -> Optional[dict]:
        """Extract best detection. Returns dict or None.

        Auto-detects output format:
        - Single output (1, 5, N): person-only YOLO (4 bbox xywh + 1 confidence)
        - Single output (1, 84, N): 80-class COCO YOLOv8 (4 bbox xywh + 80 class scores)
        - Multi-output SSD: output[0]=scores (1, N, classes), output[-1]=bboxes (1, N, 4)

        Returns dict with keys:
        - bbox: (x1, y1, x2, y2) in original image pixels
        - center: (cx, cy) in original image pixels
        - conf: confidence score
        - class_id: class index
        - class_name: class label string
        """
        class_ids = None

        if len(outputs) == 1:
            output = outputs[0]
            shape = output.shape
            if len(shape) == 3 and shape[1] == 5:
                # person-only: (1, 5, N) -> (N, 5)
                preds = output[0]  # (5, N)
                bbox_xywh = preds[:4, :].T  # (N, 4)
                scores = preds[4, :]  # (N,)
            elif len(shape) == 3 and shape[1] == 84:
                # 80-class COCO: (1, 84, N) -> (N, 84)
                preds = output[0].T
                bbox_xywh = preds[:, :4]
                class_scores = preds[:, 4:]
                class_ids = class_scores.argmax(axis=1)
                scores = class_scores.max(axis=1)
            else:
                if self.logger:
                    self.logger.warn(f"Unknown output shape: {shape}")
                return None
        elif len(outputs) >= 2:
            # SSD format: output[0]=scores (1,N,81), output[1]=encoded_boxes (1,N,4), output[2]=anchors (1,N,4)
            scores_raw = outputs[0][0]       # (N, 81) including background at index 0
            encoded_boxes = outputs[1][0]    # (N, 4) encoded offsets
            anchors = outputs[2][0]          # (N, 4) anchor coords normalized [0,1]

            # skip background class (index 0)
            class_scores = scores_raw[:, 1:]
            # +1 because we skipped background: argmax 0 -> class_id 1 (person)
            class_ids = class_scores.argmax(axis=1) + 1
            scores = class_scores.max(axis=1)

            # confidence filter
            mask = scores >= self.conf_thresh
            if not mask.any():
                return None
            encoded_boxes = encoded_boxes[mask]
            anchors = anchors[mask]
            scores = scores[mask]
            class_ids = class_ids[mask]

            # anchor decode (ST official formula)
            aw = anchors[:, 2] - anchors[:, 0]
            ah = anchors[:, 3] - anchors[:, 1]
            decoded_xmin = encoded_boxes[:, 0] * aw + anchors[:, 0]
            decoded_ymin = encoded_boxes[:, 1] * ah + anchors[:, 1]
            decoded_xmax = encoded_boxes[:, 2] * aw + anchors[:, 2]
            decoded_ymax = encoded_boxes[:, 3] * ah + anchors[:, 3]

            # normalized [0,1] -> original image pixels
            boxes = np.column_stack([
                decoded_xmin * img_w,
                decoded_ymin * img_h,
                decoded_xmax * img_w,
                decoded_ymax * img_h,
            ])

            # NMS
            keep = _nms(boxes, scores, self.iou_thresh)
            if not keep:
                return None

            best_idx = keep[np.argmax(scores[keep])]
            box = boxes[best_idx]
            conf = float(scores[best_idx])
            cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

            cid = int(class_ids[best_idx])
            # load SSD labels if available, fallback to COCO_CLASSES
            if not hasattr(self, '_ssd_labels'):
                self._ssd_labels = _load_labels(SSD_LABEL_FILE)
            if self._ssd_labels and cid < len(self._ssd_labels):
                cname = self._ssd_labels[cid]
            elif cid < len(COCO_CLASSES):
                cname = COCO_CLASSES[cid]
            else:
                cname = f"class_{cid}"

            return {
                "bbox": (float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                "center": (float(cx), float(cy)),
                "conf": conf,
                "class_id": cid,
                "class_name": cname,
            }
        else:
            if self.logger:
                self.logger.warn(f"Unknown outputs: {len(outputs)} tensors")
            return None

        # bbox coords are normalized [0,1] relative to input_size
        bbox_xywh[:, :4] *= self.input_size

        # confidence filter
        mask = scores >= self.conf_thresh
        if not mask.any():
            return None
        bbox_xywh = bbox_xywh[mask]
        scores = scores[mask]
        if class_ids is not None:
            class_ids = class_ids[mask]

        # xywh -> xyxy
        boxes = _xywh2xyxy(bbox_xywh)

        # NMS
        keep = _nms(boxes, scores, self.iou_thresh)
        if not keep:
            return None

        # pick best
        best_idx = keep[np.argmax(scores[keep])]
        box = boxes[best_idx]
        conf = float(scores[best_idx])

        # map back to original image coords
        x1 = float(np.clip((box[0] - pad[0]) / scale, 0, img_w))
        y1 = float(np.clip((box[1] - pad[1]) / scale, 0, img_h))
        x2 = float(np.clip((box[2] - pad[0]) / scale, 0, img_w))
        y2 = float(np.clip((box[3] - pad[1]) / scale, 0, img_h))
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        # resolve class
        if class_ids is not None:
            cid = int(class_ids[best_idx])
            cname = COCO_CLASSES[cid] if cid < len(COCO_CLASSES) else f"class_{cid}"
        else:
            cid = 0
            cname = "person"

        return {
            "bbox": (x1, y1, x2, y2),
            "center": (cx, cy),
            "conf": conf,
            "class_id": cid,
            "class_name": cname,
        }

    def infer(self, img_rgb: np.ndarray) -> Tuple[Optional[dict], float]:
        """Full pipeline: preprocess -> run -> postprocess.

        Args:
            img_rgb: HWC uint8 RGB image

        Returns:
            (detection_dict, latency_ms) or (None, latency_ms)
            detection_dict has keys: bbox, center, conf, class_id, class_name
        """
        h, w = img_rgb.shape[:2]
        blob, scale, pad = self.preprocess(img_rgb)
        t0 = time.perf_counter()
        output = self._run_inference(blob)
        latency_ms = (time.perf_counter() - t0) * 1000.0
        result = self.postprocess(output, scale, pad, w, h)
        return result, latency_ms

    def benchmark(self, img_rgb: np.ndarray, n: int = 100, warmup: int = 10):
        """Run N inferences, return timing stats."""
        blob, scale, pad = self.preprocess(img_rgb)

        # warmup
        for _ in range(warmup):
            self._run_inference(blob)

        # benchmark
        latencies = []
        for _ in range(n):
            t0 = time.perf_counter()
            self._run_inference(blob)
            latencies.append((time.perf_counter() - t0) * 1000.0)

        arr = np.array(latencies)
        return {
            "n": n,
            "warmup": warmup,
            "mean_ms": float(arr.mean()),
            "std_ms": float(arr.std()),
            "min_ms": float(arr.min()),
            "max_ms": float(arr.max()),
            "p50_ms": float(np.percentile(arr, 50)),
            "p95_ms": float(np.percentile(arr, 95)),
            "p99_ms": float(np.percentile(arr, 99)),
            "fps": 1000.0 / float(arr.mean()) if arr.mean() > 0 else 0,
        }
