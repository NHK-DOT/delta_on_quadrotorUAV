#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DURATION="${1:-30}"

python3 "$ROOT_DIR/src/test_gamepad_once.py" \
  --config "$ROOT_DIR/config/gamepad_8bitdo_bt.json" \
  --duration "$DURATION" \
  --log "$ROOT_DIR/logs/gamepad_once.log" \
  --json "$ROOT_DIR/logs/gamepad_once.json"
