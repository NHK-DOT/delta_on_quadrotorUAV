#!/bin/sh
set -eu

DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
cd "$DIR"

SENSOR_ID="${SENSOR_ID:-0}"
FLIP_METHOD="${FLIP_METHOD:-0}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
SAVE_FPS="${SAVE_FPS:-1/2}"
COLS="${COLS:-10}"
ROWS="${ROWS:-7}"
SQUARE_SIZE_M="${SQUARE_SIZE_M:-0.020}"
MIN_SAMPLES="${MIN_SAMPLES:-20}"

echo "YOLO fisheye one-click calibration"
echo "Board: 11x8 squares = ${COLS}x${ROWS} inner corners"
echo "Square size: ${SQUARE_SIZE_M} m"
echo "Resolution: ${WIDTH}x${HEIGHT}"
echo
echo "Step 1: camera preview and frame capture will start."
echo "Move the board slowly: center, edges, corners, near/far, slight tilts."
echo "Keep the full board visible and hold every pose for about 3 seconds."
echo "When enough images are captured, press Ctrl+C in this terminal."
echo

pkill -f "gst-launch-1.0 -e nvarguscamerasrc" 2>/dev/null || true
pkill -f "capture_csi_preview_frames.sh" 2>/dev/null || true
pkill -f "calibrate_fisheye_from_images.py" 2>/dev/null || true

echo "Restarting nvargus-daemon..."
printf 'nvidia\n' | sudo -S systemctl restart nvargus-daemon || true
sleep 3

rm -rf capture_stream calibration/valid_fisheye_frames calibration/yolo_fisheye_camera_intrinsics.json
mkdir -p capture_stream calibration

set +e
SENSOR_ID="$SENSOR_ID" \
FLIP_METHOD="$FLIP_METHOD" \
WIDTH="$WIDTH" \
HEIGHT="$HEIGHT" \
FPS="$FPS" \
SAVE_FPS="$SAVE_FPS" \
OUT_DIR="capture_stream" \
bash capture_csi_preview_frames.sh
capture_code=$?
set -e

echo
echo "Capture stopped with code: $capture_code"
frame_count="$(find capture_stream -maxdepth 1 -type f -name 'frame_*.jpg' 2>/dev/null | wc -l)"
echo "Captured frames: $frame_count"

if [ "$frame_count" -lt "$MIN_SAMPLES" ]; then
  echo "Not enough frames. Capture more images before calibration." >&2
  exit 1
fi

echo
echo "Step 2: offline checkerboard scan and fisheye calibration..."
python3 calibrate_fisheye_from_images.py \
  --image-dir capture_stream \
  --cols "$COLS" \
  --rows "$ROWS" \
  --square-size-m "$SQUARE_SIZE_M" \
  --min-samples "$MIN_SAMPLES" \
  --max-images 300 \
  --stop-after-valid 60

echo
echo "DONE"
echo "Calibration file:"
echo "$DIR/calibration/yolo_fisheye_camera_intrinsics.json"
