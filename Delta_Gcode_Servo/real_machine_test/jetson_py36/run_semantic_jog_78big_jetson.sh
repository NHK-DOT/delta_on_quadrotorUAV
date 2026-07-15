#!/usr/bin/env bash
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$THIS_DIR/competition_78big/workspace_1_3_4.json}"

exec python3 "$THIS_DIR/jetson_gamepad_semantic_jog_py36.py" \
  --workspace "$WORKSPACE" \
  --raw-per-sec "${RAW_PER_SEC:-250}" \
  --command-rate-hz "${COMMAND_RATE_HZ:-50}" \
  --feedback-interval-sec "${FEEDBACK_INTERVAL_SEC:-0.15}" \
  --max-feedback-lead-ticks "${MAX_FEEDBACK_LEAD_TICKS:-40}" \
  "$@"
