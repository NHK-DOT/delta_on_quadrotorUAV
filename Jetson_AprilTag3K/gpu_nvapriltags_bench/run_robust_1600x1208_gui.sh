#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# Higher-resolution full-FOV GPU path for noisy/small-tag scenes.
# Same no-hold detector input as the stable 1280x960 path, but with more pixels.
export OUT_SIZE="${OUT_SIZE:-1600x1208}"
export PREPROCESS="${PREPROCESS:-gray_blur_gamma07}"
export GUI_SCALE="${GUI_SCALE:-0.75}"
export GUI_EVERY="${GUI_EVERY:-2}"
export GUI_HOLD_MS=0
export OUTPUT_HOLD_MS=0

# 10-tap downsample was better than default nearest in the same-scene 2026-06-24
# A/B test, while keeping the loop near the 21 fps sensor limit.
export NVVIDCONV_INTERPOLATION="${NVVIDCONV_INTERPOLATION:-3}"

exec ./run_fullfov_1280x960_gui.sh
