#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PYTHON_BIN=${PYTHON:-python3}
SERIAL_PORT=${ARM_SERIAL_PORT:-/dev/ttyUSB0}
JOYSTICK_DEV=${ARM_JOYSTICK:-/dev/input/js0}
FIX_DEVICE_PERMS=0

if [ "${1:-}" = "--fix-device-permissions" ]; then
    FIX_DEVICE_PERMS=1
fi

echo "[arm_min_py36] directory: $SCRIPT_DIR"
echo "[arm_min_py36] python: $PYTHON_BIN"

"$PYTHON_BIN" - <<'PY'
import sys
print("Python executable:", sys.executable)
print("Python version:", sys.version.replace("\n", " "))
if sys.version_info[:2] != (3, 6):
    raise SystemExit("ERROR: this minimal package is pinned for Python 3.6.x; run it with python3=3.6.9")
PY

if ! "$PYTHON_BIN" -m pip --version >/dev/null 2>&1; then
    echo "[arm_min_py36] pip is missing; trying ensurepip --user"
    "$PYTHON_BIN" -m ensurepip --user || {
        echo "ERROR: pip is not available for python3."
        echo "Install python3-pip for Python 3.6, then run this script again."
        exit 1
    }
fi

NEED_INSTALL=$("$PYTHON_BIN" - <<'PY'
missing = []
try:
    import serial
except Exception:
    missing.append("pyserial")
print(",".join(missing))
PY
)

VENDOR_DIR="$SCRIPT_DIR/vendor"
if [ -n "$NEED_INSTALL" ]; then
    echo "[arm_min_py36] missing modules: $NEED_INSTALL"
    echo "[arm_min_py36] installing into local vendor directory, no global sudo install"
    mkdir -p "$VENDOR_DIR"
    "$PYTHON_BIN" -m pip install --no-cache-dir --target "$VENDOR_DIR" -r "$SCRIPT_DIR/requirements-py36.txt"
else
    echo "[arm_min_py36] pyserial is already importable."
fi

PYTHONPATH_CHECK="$VENDOR_DIR:$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"
PYTHONPATH="$PYTHONPATH_CHECK" "$PYTHON_BIN" - <<'PY'
import serial
print("pyserial:", serial.__version__)
PY

cat > "$SCRIPT_DIR/run_controller.sh" <<'SH'
#!/bin/sh
DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PYTHON_BIN=${PYTHON:-python3}
export PYTHONPATH="$DIR/vendor:$DIR${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_BIN" "$DIR/gamepad_controller.py" --port "${ARM_SERIAL_PORT:-/dev/ttyUSB0}" --joystick "${ARM_JOYSTICK:-/dev/input/js0}" "$@"
SH

cat > "$SCRIPT_DIR/run_calibration.sh" <<'SH'
#!/bin/sh
DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PYTHON_BIN=${PYTHON:-python3}
export PYTHONPATH="$DIR/vendor:$DIR${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_BIN" "$DIR/servo_calibration.py" --port "${ARM_SERIAL_PORT:-/dev/ttyUSB0}" "$@"
SH

chmod +x "$SCRIPT_DIR/run_controller.sh" "$SCRIPT_DIR/run_calibration.sh"
if [ -f "$SCRIPT_DIR/pair_xbox_bluetooth.sh" ]; then
    chmod +x "$SCRIPT_DIR/pair_xbox_bluetooth.sh"
fi
if [ -f "$SCRIPT_DIR/diagnose_xbox_bluetooth.sh" ]; then
    chmod +x "$SCRIPT_DIR/diagnose_xbox_bluetooth.sh"
fi

check_device() {
    DEV="$1"
    MODE="$2"
    if [ ! -e "$DEV" ]; then
        echo "[arm_min_py36] WARN: device not found: $DEV"
        return
    fi
    if [ "$MODE" = "rw" ]; then
        if [ ! -r "$DEV" ] || [ ! -w "$DEV" ]; then
            echo "[arm_min_py36] WARN: no read/write permission on $DEV"
        else
            echo "[arm_min_py36] OK: read/write permission on $DEV"
        fi
    else
        if [ ! -r "$DEV" ]; then
            echo "[arm_min_py36] WARN: no read permission on $DEV"
        else
            echo "[arm_min_py36] OK: read permission on $DEV"
        fi
    fi
}

if [ "$FIX_DEVICE_PERMS" = "1" ]; then
    if command -v sudo >/dev/null 2>&1; then
        [ -e "$SERIAL_PORT" ] && sudo chmod a+rw "$SERIAL_PORT" || true
        [ -e "$JOYSTICK_DEV" ] && sudo chmod a+r "$JOYSTICK_DEV" || true
    else
        echo "[arm_min_py36] sudo not found; cannot auto chmod device nodes."
    fi
fi

check_device "$SERIAL_PORT" rw
check_device "$JOYSTICK_DEV" r

echo ""
echo "Current serial devices:"
if ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null; then
    echo "If the servo adapter is not the default, set ARM_SERIAL_PORT=/dev/ttyUSBX or /dev/ttyACMX."
else
    echo "No /dev/ttyUSB* or /dev/ttyACM* devices found."
    echo "Plug in the USB serial adapter, then run:"
    echo "  dmesg | tail -30"
    echo "  ls -l /dev/ttyUSB* /dev/ttyACM*"
fi

echo ""
echo "Current joystick devices:"
if ls -l /dev/input/js* 2>/dev/null; then
    echo "If the Bluetooth controller is not the default, set ARM_JOYSTICK=/dev/input/jsX."
else
    echo "No /dev/input/js* devices found."
    echo "For Bluetooth Xbox controller pairing, run:"
    echo "  ./pair_xbox_bluetooth.sh"
fi

echo ""
echo "Long-term device permission fix, if needed:"
echo "  sudo usermod -a -G dialout,tty,input \$USER"
echo "  log out and log in again"
echo ""
echo "Run:"
echo "  cd $SCRIPT_DIR"
echo "  ./diagnose_xbox_bluetooth.sh  # Bluetooth diagnosis if pairing is unstable"
echo "  ./pair_xbox_bluetooth.sh    # first Bluetooth setup only"
echo "  ./run_calibration.sh"
echo "  ./run_controller.sh"
