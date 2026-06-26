#!/usr/bin/env bash
set -euo pipefail

VISION_DIR="${VISION_DIR:-/home/nvidia/vision_starter}"
DEPTH_START="${DEPTH_START:-/home/nvidia/orbbec_sdk/start_depth_grid.sh}"
DEPTH_JSON="${DEPTH_JSON:-/tmp/orbbec_depth_grid.json}"
ENGINE="${ENGINE:-models/wrench_combined_20260626_320_trt7_fp16.engine}"
PORT="${PORT:-8090}"

cd "${VISION_DIR}"
mkdir -p outputs

if [ -f outputs/trt_yolo_wrench_orbbec.pid ]; then
  OLD_PID="$(cat outputs/trt_yolo_wrench_orbbec.pid 2>/dev/null || true)"
  if [ -n "${OLD_PID}" ]; then
    kill "${OLD_PID}" 2>/dev/null || true
  fi
fi

"${DEPTH_START}" >/tmp/start_orbbec_depth_grid.out

nohup python3 scripts/trt_yolo_server.py \
  --engine "${ENGINE}" \
  --source /dev/video1 \
  --width 640 --height 480 --camera-fps 30 --fourcc MJPG \
  --infer-fps 30 --display-fps 20 --capture-thread \
  --label wrench --conf 0.25 --iou 0.45 \
  --max-detections 1 \
  --depth-json "${DEPTH_JSON}" --depth-max-age 1.0 \
  --camera-hfov-deg 67 --camera-vfov-deg 52 \
  --host 0.0.0.0 --port "${PORT}" \
  > outputs/trt_yolo_wrench_orbbec.log 2>&1 < /dev/null &

echo "$!" > outputs/trt_yolo_wrench_orbbec.pid
echo "depth_json=${DEPTH_JSON}"
echo "preview=http://127.0.0.1:${PORT}/"
echo "latest=http://127.0.0.1:${PORT}/latest.json"
