#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

python3 "$ROOT_DIR/src/show_control_state.py" \
  --config "$ROOT_DIR/config/gamepad_8bitdo_bt.json" \
  "$@"
