# bt_8bitdo_min

## Relation to Dual_Camera_HandEye

`Dual_Camera_HandEye/` is an offline coordinate-chain demo for the current base
camera + side object camera layout. It reuses AprilTag/IMU snapshot files and
does not replace this gamepad package.

Keep the boundary clear:

- this package owns operator input, serial preflight, kinematics, and real servo
  commands;
- `Dual_Camera_HandEye/` owns the math for checking `base_T_tool` from the
  top-side end-effector AprilTag and projecting side-camera object detections
  into the arm base frame.

Minimal 8BitDo Bluetooth gamepad package for Ubuntu 18.04 on the onboard Nano.

It reads Linux `/dev/input/event*` directly, so it does not depend on pygame or
SDL gamepad mappings. The package has separate read-only test entrypoints and a
real-machine entrypoint.

## Directory Layout

- `config/`: gamepad event mapping and Bluetooth MAC config.
- `deploy/`: install, test, status, mapping-check, and real-control scripts.
- `src/evdev_gamepad.py`: direct Linux evdev gamepad reader.
- `src/test_gamepad_once.py`: one-shot overwriting capture tool.
- `src/show_control_state.py`: live read-only state display.
- `src/check_gamepad_mapping.py`: checks whether the capture covers real-control inputs.
- `src/check_serial_readonly.py`: read-only servo driver board serial preflight.
- `src/gamepad_controller.py`: real-machine control loop.
- `src/kinematics.py`: delta robot IK/FK.
- `src/servo_driver.py`: LX bus-servo driver-board protocol.
- `src/servo_mapping.py`: raw servo mapping helpers.
- `logs/`: generated capture files.
- `docs/`: extra notes.

## Install

```bash
cd ~/Desktop/bt_8bitdo_min
bash deploy/install_ubuntu18.sh
```

If the package lives under the repo:

```bash
cd ~/Desktop/78arm/bt_8bitdo_min
bash deploy/install_ubuntu18.sh
```

After the first install, log out and log back in so the `input` group permission
works.

If the script cannot find a Bluetooth MAC, pair manually:

```bash
bluetoothctl
power on
agent on
default-agent
scan on
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
quit
```

Then write the MAC into `config/bluetooth_mac.conf`.

## Read-Only Gamepad Test

Run the capture:

```bash
bash deploy/run_log_test.sh 30
```

During the 30 seconds:

- move the D-pad left/right/up/down
- move the right stick Y axis up/down
- press `A`, `B`, `X`, `Y`, `LB`, and `RB` once
- optionally move both analog sticks and LT/RT fully for diagnostics

The capture overwrites:

- `logs/gamepad_once.log`
- `logs/gamepad_once.json`

Check whether the capture is enough for real-machine control:

```bash
bash deploy/run_mapping_check.sh
```

Expected result: all motion axes and action buttons show `OK`. If a control
shows `MISSING`, repeat the capture and press or move that control.

Live read-only display:

```bash
bash deploy/run_show_state.sh
```

This does not open the serial port and cannot move servos. Watch
`CTRL_X/CTRL_Y/CTRL_Z`; these are the values the real-machine controller uses.

## Read-Only Serial Preflight

Before real-machine control, verify the servo driver board serial link:

```bash
sudo apt-get install -y python3-serial
python3 -c "import serial; print(serial.__version__)"
bash deploy/run_serial_check.sh --port /dev/ttyUSB0
```

This opens the serial port and reads servo 1/2/3 positions. It does not send a
move command.

## Writable Serial Motion Check

After the read-only serial check passes, send a small write-motion command:

```bash
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0
```

This reads servo 1/2/3 positions, nudges them by a small raw tick delta, then
returns them to the starting positions. Useful options:

```bash
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0 --baudrate 115200 --delta 12 --time-ms 700 --trace
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0 --ids 1 --delta 16 --no-return
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0 --targets 1:500,2:500,3:500 --no-return
bash deploy/run_serial_move_check.sh --port /dev/ttyUSB0 --timeout 1.5 --read-retries 5 --trace
```

## Real-Machine Control

Only run this after `run_mapping_check.sh` and `run_serial_check.sh` pass:

```bash
bash deploy/run_control_bt.sh --port /dev/ttyUSB0
```

This opens the serial port and can command the servo driver board.

Default control mapping:

- D-pad X/Y -> arm X/Y
- right stick Y -> arm Z
- `A` -> quit
- `B` -> record current point
- `X` -> switch safe scan mode
- `Y` -> switch sensor frame mode
- `LB/RB` -> tooling servo close/open

## Device Detection

Do not hard-code `event8`. The event number can change after reconnecting the
controller. Leave `config/gamepad_8bitdo_bt.json` `device.device_path` empty;
the reader finds the current `/dev/input/eventX` by device name, bus, vendor,
and product.
