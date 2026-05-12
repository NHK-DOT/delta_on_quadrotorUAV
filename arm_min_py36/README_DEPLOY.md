# arm_min_py36

This is a minimal Python 3.6.9 package extracted from the 78arm control path:

- LX225 bus-servo serial packets
- servo raw/logical mapping
- delta-arm inverse/forward kinematics
- Bluetooth or wired Linux joystick control through `/dev/input/js0`
- optional servo4 tooling control with `LB/RB`

The package avoids Python 3.11-only modules such as `tomllib`, `dataclasses`, and `X | Y` type syntax. It also avoids `pygame`; the controller reads the Linux joystick device directly.
The kinematics are pure Python, so there is no heavy `numpy` dependency.

## Target

- `python3` must be Python 3.6.x, expected `3.6.9`
- serial adapter: default `/dev/ttyUSB0`
- joystick: default `/dev/input/js0`; Bluetooth controllers usually appear as `/dev/input/js0` or `/dev/input/js1`
- dependency: `pyserial==3.5`

## Deploy On The Robot Machine

Copy the whole `arm_min_py36` directory to the robot machine, then run:

```sh
cd arm_min_py36
sh deploy_py36.sh
```

If serial or joystick device permissions are blocked:

```sh
sh deploy_py36.sh --fix-device-permissions
```

For a long-term permission fix:

```sh
sudo usermod -a -G dialout,tty,input $USER
```

Then log out and log in again.

The deploy script installs missing Python packages into the local `vendor/` directory, not into the system Python. This avoids global pip permission errors.
It uses `pip --no-cache-dir`, so broken ownership under `~/.cache/pip` should not block deployment.

If the serial device is not `/dev/ttyUSB0`, inspect:

```sh
ls -l /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -30
```

Then run with the actual device:

```sh
ARM_SERIAL_PORT=/dev/ttyACM0 ./run_calibration.sh
ARM_SERIAL_PORT=/dev/ttyACM0 ./run_controller.sh
```

## Pair Xbox Controller Over Bluetooth

On the robot machine:

```sh
cd arm_min_py36
./diagnose_xbox_bluetooth.sh
./pair_xbox_bluetooth.sh
```

Put the controller in pairing mode before running it. For most Xbox controllers:

1. Hold the Xbox button until the controller turns on.
2. Hold the small pair button until the light blinks quickly.
3. Pick the `Xbox Wireless Controller` MAC address printed by the script.

After pairing, check which joystick device appears:

```sh
ls -l /dev/input/js*
```

If it is not `/dev/input/js0`, pass the right device:

```sh
ARM_JOYSTICK=/dev/input/js1 ./run_controller.sh
```

If Bluetooth itself has permission/service problems, check:

```sh
sudo systemctl enable --now bluetooth
sudo usermod -a -G bluetooth,input $USER
```

Then log out and log in again.

If the Bluetooth switch keeps toggling or the controller repeatedly pairs and
disconnects, collect a focused diagnosis:

```sh
./diagnose_xbox_bluetooth.sh --scan
```

For a full event capture while reproducing the Bluetooth toggle problem:

```sh
sudo ./diagnose_xbox_bluetooth.sh --capture 60
```

During the 60 seconds, turn Bluetooth on once, put the Xbox controller into
pairing mode, and try to connect. The script writes a directory named like
`xbox_bt_capture_YYYYMMDD_HHMMSS` and, if `tar` is available, an archive named
`xbox_bt_capture_YYYYMMDD_HHMMSS.tar.gz`.

If the log shows ERTM is not disabled, apply the common Xbox controller fix:

```sh
./diagnose_xbox_bluetooth.sh --apply-xbox-fixes
```

This also installs `xbox-bluetooth-ertm.service` and loads `uhid`/`joydev`,
because some Jetson 4.9 systems load Bluetooth before `/etc/modprobe.d` options
take effect, and Xbox BLE HID input needs `uhid`. After reboot, verify:

```sh
cat /sys/module/bluetooth/parameters/disable_ertm
ls -l /dev/uhid
```

The expected value is `Y` or `1`, not `N`.

If a stale pairing record exists, remove it and pair again:

```sh
./diagnose_xbox_bluetooth.sh --reset-device XX:XX:XX:XX:XX:XX
./pair_xbox_bluetooth.sh
```

## Run

First use the calibration helper:

```sh
./run_calibration.sh
```

Then run the realtime controller:

```sh
./run_controller.sh
```

Override devices if needed:

```sh
ARM_SERIAL_PORT=/dev/ttyUSB1 ARM_JOYSTICK=/dev/input/js1 ./run_controller.sh
```

You can also pass options through:

```sh
./run_controller.sh --dry-run
./run_controller.sh --no-tooling
./run_controller.sh --speed-xy 80 --speed-z 60 --servo-speed 300
```

## Controls

- D-pad or left stick: X/Y
- right stick Y: Z
- A: quit
- B: record current point to `workspace_points.csv`
- X: switch safe scan mode `FREE/X/Y/Z`
- LB/RB: optional servo4 tooling movement

## Safety Startup

`gamepad_controller.py` reads servo 1/2/3 current raw positions before motion. It refuses to start unless they are near the configured reference raw values:

- servo1: `834`
- servo2: `770`
- servo3: `816`

Use `servo_calibration.py` first if the startup check fails.
