#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# Moving-arm mode: no stale hold, no temporal ISP blending, shorter exposure.
# If the image is too dark, add light before raising the exposure ceiling.
export PREPROCESS="${PREPROCESS:-motion}"
export GUI_HOLD_MS=0
export OUTPUT_HOLD_MS=0
export TNR_MODE="${TNR_MODE:-0}"
export TNR_STRENGTH="${TNR_STRENGTH:-0}"
export EXPOSURE_COMPENSATION="${EXPOSURE_COMPENSATION:-0}"
export EXPOSURETIMERANGE="${EXPOSURETIMERANGE:-34000 8000000}"
export GAINRANGE="${GAINRANGE:-1 12}"
export ISPDIGITALGAINRANGE="${ISPDIGITALGAINRANGE:-1 4}"

exec ./run_fullfov_1280x960_gui.sh
