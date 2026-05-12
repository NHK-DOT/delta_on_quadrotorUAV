#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="$ROOT_DIR/config/bluetooth_mac.conf"
UDEV_RULE="/etc/udev/rules.d/99-8bitdo-ultimate2.rules"

echo "[1/6] Python and package check"
python3 --version

if command -v apt-get >/dev/null 2>&1; then
  echo "[2/6] Install runtime tools"
  sudo apt-get update
  sudo apt-get install -y bluez evtest
else
  echo "apt-get not found; skip package install."
fi

echo "[3/6] Prepare local folders"
mkdir -p "$ROOT_DIR/logs"

echo "[4/6] Ensure input group"
if ! getent group input >/dev/null 2>&1; then
  sudo groupadd input
fi

echo "[5/6] Install udev rule"
sudo tee "$UDEV_RULE" >/dev/null <<'EOF'
SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="8BitDo Ultimate 2 Wireless", MODE="0660", GROUP="input"
SUBSYSTEM=="input", KERNEL=="event*", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="6012", MODE="0660", GROUP="input"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[6/6] Add current user to input group"
sudo usermod -aG input "$USER"

if [ -f "$CONFIG_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONFIG_FILE" || true
fi

echo "Bluetooth connect check"
if [ -n "${GAMEPAD_MAC:-}" ]; then
  if command -v bluetoothctl >/dev/null 2>&1; then
    bluetoothctl <<EOF
power on
agent on
default-agent
trust ${GAMEPAD_MAC}
connect ${GAMEPAD_MAC}
quit
EOF
  else
    echo "bluetoothctl not found."
  fi
else
  echo "GAMEPAD_MAC is empty."
  echo "Fill config/bluetooth_mac.conf, then run:"
  echo "  bluetoothctl"
  echo "  pair <MAC>"
  echo "  trust <MAC>"
  echo "  connect <MAC>"
fi

echo
echo "Done."
echo "Log out and back in so the input-group change takes effect."
