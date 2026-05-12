#!/bin/sh
set -eu

SCAN_SECONDS=${SCAN_SECONDS:-18}
MAC=${1:-}

if ! command -v bluetoothctl >/dev/null 2>&1; then
    echo "ERROR: bluetoothctl not found."
    echo "Install BlueZ tools first. On Debian/Ubuntu:"
    echo "  sudo apt-get install bluez"
    exit 1
fi

echo "[bluetooth] Put the Xbox controller in pairing mode now."
echo "[bluetooth] Usually: hold the Xbox button, then hold the small pair button until it blinks fast."
echo ""

if command -v rfkill >/dev/null 2>&1; then
    if rfkill list bluetooth 2>/dev/null | grep -qi "Soft blocked: yes"; then
        echo "[bluetooth] Bluetooth is soft-blocked by rfkill."
        echo "[bluetooth] Trying: sudo rfkill unblock bluetooth"
        if command -v sudo >/dev/null 2>&1; then
            sudo rfkill unblock bluetooth || true
        else
            echo "[bluetooth] sudo not found; run this manually:"
            echo "  rfkill unblock bluetooth"
        fi
    fi
fi

bluetoothctl power on >/dev/null || true
bluetoothctl agent on >/dev/null || true
bluetoothctl default-agent >/dev/null || true

if [ -z "$MAC" ]; then
    echo "[bluetooth] Scanning for $SCAN_SECONDS seconds..."
    bluetoothctl scan on >/dev/null 2>&1 || true
    sleep "$SCAN_SECONDS"
    bluetoothctl scan off >/dev/null 2>&1 || true

    echo ""
    echo "[bluetooth] Devices seen:"
    bluetoothctl devices || true
    echo ""
    echo "Pick the Xbox/Wireless Controller MAC address from the list."
    printf "MAC address: "
    read MAC
fi

if [ -z "$MAC" ]; then
    echo "ERROR: empty MAC address."
    exit 1
fi

echo "[bluetooth] Pair/trust/connect $MAC"
bluetoothctl pair "$MAC" || true
bluetoothctl trust "$MAC"
bluetoothctl connect "$MAC"

sleep 2

echo ""
echo "[bluetooth] Joystick devices:"
if ls -l /dev/input/js* 2>/dev/null; then
    echo ""
    echo "Use the matching device with:"
    echo "  ARM_JOYSTICK=/dev/input/js0 ./run_controller.sh"
    echo "or, if it appears as js1:"
    echo "  ARM_JOYSTICK=/dev/input/js1 ./run_controller.sh"
else
    echo "No /dev/input/js* device found yet."
    echo "Try pressing a controller button, wait a few seconds, then run:"
    echo "  ls -l /dev/input/js*"
fi
