#!/bin/sh
set -eu

DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
cd "$DIR"

CAMERA_INDEX="${CAMERA_INDEX:-0}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-30}"
COLS="${COLS:-9}"
ROWS="${ROWS:-6}"
SQUARE_SIZE_M="${SQUARE_SIZE_M:-0.025}"

python3 calibrate_fisheye_camera.py \
  --source usb \
  --camera-index "$CAMERA_INDEX" \
  --backend any \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --fps "$FPS" \
  --cols "$COLS" \
  --rows "$ROWS" \
  --square-size-m "$SQUARE_SIZE_M"
