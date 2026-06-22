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
OUT_DIR="${OUT_DIR:-capture_stream}"

mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/frame_*.jpg

echo "NVIDIA preview + frame capture"
echo "Preview is for positioning only. Frames are saved to: $OUT_DIR"
echo "Move the checkerboard through center, corners, edges, near/far, tilted poses."
echo "Close this terminal or press Ctrl+C after about 60-90 seconds."

gst-launch-1.0 -e \
  nvarguscamerasrc sensor-id="$SENSOR_ID" ! \
  "video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT,framerate=$FPS/1" ! \
  tee name=t \
  t. ! queue ! nvvidconv flip-method="$FLIP_METHOD" ! nvegltransform ! nveglglessink sync=false \
  t. ! queue ! nvvidconv flip-method="$FLIP_METHOD" ! "video/x-raw,format=I420" ! \
  videorate ! "video/x-raw,framerate=$SAVE_FPS" ! jpegenc quality=95 ! \
  multifilesink location="$OUT_DIR/frame_%04d.jpg"
