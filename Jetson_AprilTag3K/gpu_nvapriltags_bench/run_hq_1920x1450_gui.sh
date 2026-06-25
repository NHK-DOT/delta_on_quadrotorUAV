#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# Higher-quality downsample experiment for small/noisy tags.
# Keeps the same GPU nvAprilTags + no-hold detector path, but raises the
# processing frame and asks nvvidconv for a better scaler than nearest.
export OUT_SIZE="${OUT_SIZE:-1920x1450}"
export PREPROCESS="${PREPROCESS:-gray_blur_gamma07}"
export GUI_SCALE="${GUI_SCALE:-0.55}"
export GUI_EVERY="${GUI_EVERY:-3}"
export GUI_HOLD_MS=0
export OUTPUT_HOLD_MS=0

# Jetson Xavier NX nvvidconv methods:
# 0 nearest, 1 bilinear, 2 5-tap, 3 10-tap, 4 smart, 5 nicest.
# Start with 10-tap because it is less jagged than nearest without jumping
# straight to the slowest path.
export NVVIDCONV_INTERPOLATION="${NVVIDCONV_INTERPOLATION:-3}"

exec ./run_fullfov_1280x960_gui.sh
