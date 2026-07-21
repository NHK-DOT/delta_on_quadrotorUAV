#!/usr/bin/env bash
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"

PORT="${PORT:-/dev/ttyUSB0}"
SERVO_CONFIG="${SERVO_CONFIG:-$ROOT/lx225_tool_demo/config/lx225_tool.demo.toml}"
GAMEPAD_CONFIG="${GAMEPAD_CONFIG:-$ROOT/bt_8bitdo_min/config/gamepad_8bitdo_bt.json}"

cd "$THIS_DIR"
exec python3 jetson_gamepad_raw_jog_py36.py \
  --port "$PORT" \
  --servo-config "$SERVO_CONFIG" \
  --gamepad-config "$GAMEPAD_CONFIG" \
  "$@"
