#!/bin/sh
set -eu

DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
cd "$DIR"

COLS="${COLS:-9}"
ROWS="${ROWS:-6}"
SQUARE_SIZE_M="${SQUARE_SIZE_M:-0.020}"
MIN_SAMPLES="${MIN_SAMPLES:-20}"
IMAGE_DIR="${IMAGE_DIR:-capture_stream}"

python3 calibrate_fisheye_from_images.py \
  --image-dir "$IMAGE_DIR" \
  --cols "$COLS" \
  --rows "$ROWS" \
  --square-size-m "$SQUARE_SIZE_M" \
  --min-samples "$MIN_SAMPLES"
