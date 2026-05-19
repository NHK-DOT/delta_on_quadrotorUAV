#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! python3 -c "import serial" >/dev/null 2>&1; then
  echo "Missing Python module: serial"
  echo "Install it with:"
  echo "  sudo apt-get install -y python3-serial"
  echo "or:"
  echo "  python3 -m pip install --user pyserial==3.5"
  exit 2
fi

python3 "$ROOT_DIR/src/check_serial_readonly.py" "$@"
