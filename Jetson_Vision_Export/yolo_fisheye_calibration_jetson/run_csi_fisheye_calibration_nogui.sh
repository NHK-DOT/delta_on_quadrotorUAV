#!/bin/sh
set -eu

DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
cd "$DIR"

SENSOR_ID="${SENSOR_ID:-0}"
FLIP_METHOD="${FLIP_METHOD:-0}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
COLS="${COLS:-9}"
ROWS="${ROWS:-6}"
SQUARE_SIZE_M="${SQUARE_SIZE_M:-0.020}"

python3 calibrate_fisheye_camera_nogui.py \
  --source csi \
  --sensor-id "$SENSOR_ID" \
  --flip-method "$FLIP_METHOD" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --fps "$FPS" \
  --cols "$COLS" \
  --rows "$ROWS" \
  --square-size-m "$SQUARE_SIZE_M"
