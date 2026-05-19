# bt_8bitdo_min entrypoints

This package has separate test and real-machine paths.

## Setup

```bash
bash deploy/install_ubuntu18.sh
```

Installs Bluetooth/input tools, serial support, the udev rule, and tries to
connect the configured 8BitDo controller.

## Read-only test entrypoints

```bash
bash deploy/run_log_test.sh 30
```

Overwrites `logs/gamepad_once.log` and `logs/gamepad_once.json`. Use this to
capture the exact Linux event codes produced by the current controller mode.

```bash
bash deploy/run_show_state.sh
```

Shows live normalized axes and logical actions. It never opens the serial port.

```bash
bash deploy/run_mapping_check.sh
```

Checks whether `logs/gamepad_once.json` covers the axes and buttons required by
the real-machine controller. For a JSON exported somewhere else:

```bash
bash deploy/run_mapping_check.sh --json ~/Desktop/gamepad_once.json
```

## Real-machine entrypoint

Small writable serial motion check:

```bash
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0
```

This reads the current servo positions, sends one small movement command, and
returns to the starting positions by default. Use `--delta`, `--time-ms`,
`--baudrate`, `--ids`, `--targets`, `--timeout`, `--read-retries`,
`--no-return`, or `--trace` as needed.

Full Bluetooth gamepad control:

```bash
bash deploy/run_control_bt.sh --port /dev/ttyUSB0
```

This opens the serial port and can command the servo driver board. It uses:

- `src/gamepad_controller.py` for the realtime control loop
- `src/kinematics.py` for delta IK/FK
- `src/servo_driver.py` for the bus-servo board protocol
- `src/servo_mapping.py` and `src/config.py` for raw servo mapping
- `src/evdev_gamepad.py` for the 8BitDo event reader

## Control mapping

The real-machine controller consumes the old Xbox-style tuple:

```text
x, y, z, buttons
```

The current default mapping in `config/gamepad_8bitdo_bt.json` is:

- `x`: D-pad X
- `y`: D-pad Y
- `z`: right stick Y
- `a`: quit
- `b`: record
- `x`: safe scan
- `y`: sensor frame
- `lb/rb`: tooling close/open
