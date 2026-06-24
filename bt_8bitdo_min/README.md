# bt_8bitdo_min

This directory is no longer a standalone robot-control package. It is kept as a
small compatibility dependency for the current Jetson Xavier NX sampling flow on
`192.168.1.80`.

The active field entrypoint is:

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
HAND_TAG_ID=3 bash run_sampler_py36_jetson.sh
```

## Kept Runtime Files

- `src/evdev_gamepad.py`: direct Linux `/dev/input/event*` reader for the
  8BitDo Ultimate 2 Wireless Bluetooth controller.
- `src/servo_driver.py`: Hiwonder xArm 1.6 bus-servo driver board protocol.
- `config/gamepad_8bitdo_bt.json`: current 8BitDo event mapping.
- `config/bluetooth_mac.conf`: optional Bluetooth pairing helper config.
- `deploy/install_ubuntu18.sh`: installs `bluez`, `python3-serial`, and the
  input udev rule/group permissions on Jetson Xavier NX.

The old standalone gamepad controller, old serial motion smoke tests, mapping
capture scripts, duplicate kinematics, and duplicate servo mapping helpers were
removed. The current controller and workspace sampling logic live under:

```text
Delta_Gcode_Servo/real_machine_test/jetson_py36/
Delta_Gcode_Servo/real_machine_test/workspace_model_tools.py
```

## Install Input Permissions

Run this once on the Jetson Xavier NX if the 8BitDo controller cannot be opened:

```bash
cd ~/Desktop/78arm/bt_8bitdo_min
bash deploy/install_ubuntu18.sh
```

Log out and back in so the `input` group change takes effect.

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

## Preflight

Use the current Jetson preflight instead of the removed legacy scripts:

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
python3 jetson_workspace_preflight.py --port /dev/ttyUSB0 --hand-tag-id 3
```

That check verifies 8BitDo input, servo feedback, AprilTag JSON freshness, and
whether the arm is near the configured `startup_check_raw` self-check position.
It sends no servo movement command. Controller-board `0x15` feedback is parsed
as signed int16, so `0xFF43` is reported as `-189` instead of unsigned `65347`.
