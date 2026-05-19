# 78arm

Delta-arm simulation, calibration tools, servo driver code, sensor tools, and
the minimal 8BitDo Bluetooth gamepad package for an onboard Ubuntu 18.04 Nano.

The minimal onboard Nano + 8BitDo controller package was first organized on
2026-05-12 and now lives under `bt_8bitdo_min/`.

## Main Folders

- `bt_8bitdo_min/`: minimal 8BitDo Bluetooth gamepad package. It has separate
  read-only test entrypoints and a real-machine control entrypoint.
- `Delta_Gcode_Servo/`: fuller delta robot G-code and servo control code.
- `Delta-Robot/`: original delta robot simulation and model resources.
- `lx225_tool_demo/`: LX-225 bus-servo tool/demo configuration.
- `IMU/`: WT61C IMU tools and latest snapshot output.
- `AprilTag_Vision/`: AprilTag camera detection tools.
- `Bus_Servo/`: bus-servo examples and utilities.

## 8BitDo Gamepad Package

Use `bt_8bitdo_min` for the current Bluetooth gamepad work:

```bash
cd ~/Desktop/bt_8bitdo_min
```

If the package is inside this repo on the Nano, use:

```bash
cd ~/Desktop/78arm/bt_8bitdo_min
```

The package is split into two paths:

- Test path: reads the gamepad only. It does not open the serial port and cannot
  move servos.
- Real-machine path: opens the servo serial port, runs kinematics, and can send
  commands to the servo driver board.

## Gamepad Test Steps

1. Install runtime tools and udev rules:

```bash
bash deploy/install_ubuntu18.sh
```

Log out and back in after the first install so the `input` group takes effect.

2. If the install script says `GAMEPAD_MAC is empty`, pair manually once:

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

Then write the MAC to `config/bluetooth_mac.conf`.

3. Run a read-only one-shot capture:

```bash
bash deploy/run_log_test.sh 30
```

During the 30 seconds, move the D-pad in all four directions, move the right
stick Y axis up/down, and press `A/B/X/Y/LB/RB` once. This overwrites:

- `logs/gamepad_once.log`
- `logs/gamepad_once.json`

4. Check whether the capture is complete for real-machine control:

```bash
bash deploy/run_mapping_check.sh
```

All motion axes and action buttons should show `OK`. If any action button says
`MISSING`, repeat the one-shot capture and press that button.

5. Optional live read-only display:

```bash
bash deploy/run_show_state.sh
```

This shows normalized axes and active logical actions without opening the servo
serial port. Watch `CTRL_X/CTRL_Y/CTRL_Z`; these are the values used by the
real-machine controller.

6. Check the servo driver board serial link without moving servos:

```bash
bash deploy/run_serial_check.sh --port /dev/ttyUSB0
```

This reads servo 1/2/3 feedback and battery voltage. It does not send a move
command.

## Real-Machine Control

Only after the mapping check and serial preflight are complete:

```bash
bash deploy/run_control_bt.sh --port /dev/ttyUSB0
```

This entrypoint can move the machine. The default mapping is:

- D-pad X/Y -> arm X/Y
- right stick Y -> arm Z
- `A` -> quit
- `B` -> record current point
- `X` -> safe scan mode
- `Y` -> sensor frame mode
- `LB/RB` -> tooling servo close/open

The gamepad event device is not fixed to `event8`. The code finds the current
`/dev/input/eventX` automatically by name, bus, vendor, and product. Leave
`config/gamepad_8bitdo_bt.json` `device.device_path` empty unless you are
debugging a specific event device.
