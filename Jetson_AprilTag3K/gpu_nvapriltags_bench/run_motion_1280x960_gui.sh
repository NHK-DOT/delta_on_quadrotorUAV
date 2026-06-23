#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# Moving-arm sampling mode: no stale hold. Keep the detector input on the
# measured no-hold GPU path unless a specific experiment overrides it.
export PREPROCESS="${PREPROCESS:-gray_blur_gamma07}"
export GUI_HOLD_MS=0
export OUTPUT_HOLD_MS=0

exec ./run_fullfov_1280x960_gui.sh
