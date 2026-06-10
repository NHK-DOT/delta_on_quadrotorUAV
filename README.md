# 78arm

中文说明: [README.zh-CN.md](README.zh-CN.md)

Delta-arm simulation, calibration tools, servo driver code, sensor tools, and
the minimal 8BitDo Bluetooth gamepad package for an onboard Ubuntu 18.04 Nano.

License: GNU GPL v3.0. See [LICENSE](LICENSE). Upstream MIT notices are kept in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

The minimal onboard Nano + 8BitDo controller package was first organized on
2026-05-12 and now lives under `bt_8bitdo_min/`.

## Project Context

This repository is for a lightweight delta-arm mounted on or tested for a UAV
platform. The current real-machine control path is not just a gamepad demo: the
onboard Ubuntu 18.04 Nano reads a Bluetooth gamepad, converts operator input
into delta-arm XYZ motion, solves inverse kinematics, maps joint angles into
raw bus-servo positions, and sends those commands through a Hiwonder xArm 1.6
servo driver board to the physical servos.

The mechanical concept and part of the early delta-robot modeling/control
direction were based on Isaac Chasteau's MIT-licensed
[isaac879/Delta-Robot](https://github.com/isaac879/Delta-Robot). This project
changes the modeling, hardware layout, control code, sensors, and deployment
pipeline, but it still acknowledges that upstream project as the original idea
source.

The 8BitDo package exists because the Xbox Series wireless controller was not
stable on the Ubuntu 18.04 Nano Bluetooth stack used here. In local testing,
that controller repeatedly bounced between connected and disconnected states.
The workaround was to use an 8BitDo Ultimate 2 Wireless controller and read the
Linux input event device directly, without relying on pygame or SDL mappings.

## Arm Photos and Models

| Prototype frame and electronics | Prototype held for linkage inspection |
| --- | --- |
| <img src="images/1.jpg" alt="Delta arm prototype frame and electronics" width="420"><br>Prototype frame, links, control board, and onboard wiring. | <img src="images/884b798faf516a24bb9bb0af58b4d616.jpg" alt="Delta arm prototype held for linkage inspection" width="420"><br>Assembled lightweight delta arm during manual inspection. |
| <img src="images/9b5124927711c6a065732a5374151702.jpg" alt="Delta arm linkage and revised printed part" width="420"><br>Linkage/end-effector side with revised printed part installed. | <img src="images/0cb198a8a6041f6031b36bc2a0e89fff.jpg" alt="Revised CAD link mount concept" width="420"><br>Revised CAD concept for a link/mount part. |
| <img src="images/ed630aaf206b2373b458c409e840b7ce.jpg" alt="Revised end-effector plate CAD" width="420"><br>Revised end-effector plate and bearing/link mounting geometry. | <img src="images/bc97d03f7ef3bbd601feaae3bde8008b.jpg" alt="Revised plate model for print or CNC" width="420"><br>Flat plate model suitable for 3D printing or CNC after export. |
| <img src="images/6e8c580ad37580d7e83ef4b96af3ac27.jpg" alt="Revised link bracket CAD" width="420"><br>Revised single bracket model. | <img src="images/6ffa0ed538995f449159233e0b68eb6e.jpg" alt="3D print slicer layout for revised parts" width="420"><br>3D print slicer layout for revised plates, links, and brackets. |

## Mechanical Model Files

Revised mechanical files are stored in `part_model_rev/`. The folder contains
SolidWorks part files (`.SLDPRT`), a `.3mf` print layout, and the current
`999.STL` mount for fixing the IMU and the end-effector AprilTag. These files
are intended for iteration on the physical arm:

- Use the `.3mf` file as a direct 3D-print starting point.
- Use `999.STL` for the current IMU + top-side AprilTag fixture that supports
  the dual-camera hand-eye workflow.
- Use the `.SLDPRT` files for CAD edits and dimensional changes.
- Export STL/3MF for printing, or export STEP/DXF and generate CAM toolpaths
  for CNC machining where the part geometry is appropriate.
- Check hole diameters, bearing fits, servo clearance, and carbon-tube linkage
  dimensions against the real hardware before printing or machining.

## Hardware Control Path

```text
8BitDo Ultimate 2 Wireless
  -> Ubuntu 18.04 Nano Bluetooth /dev/input/eventX
  -> bt_8bitdo_min evdev reader
  -> realtime delta-arm controller
  -> XYZ target and inverse kinematics
  -> raw LX bus-servo position commands
  -> USB serial adapter
  -> Hiwonder xArm 1.6 servo driver board
  -> physical LX bus servos
```

The controller can move real servos. For that reason, the package keeps
read-only test entrypoints separate from the real-machine entrypoint. Always run
the mapping and serial preflight checks before opening the writable control
loop.

## Dual-Camera Hand-Eye Layout

`Dual_Camera_HandEye/` documents the current vision geometry:

- The base camera looks at the AprilTag on the top side of the end effector and
  estimates/checks `base_T_tool`.
- The underside of the end effector carries the grasping mechanism.
- The side camera on the actuator looks at the object to be grasped. Its fixed
  mount is modeled as `tool_T_object_camera`, measured from CAD/assembly rather
  than solved by making it look at a base tag.

The demo reuses existing outputs:

- `AprilTag_Vision/myAprilTag/output/apriltag_latest.json`
- `IMU/wt61c_latest.json`
- `Delta_Gcode_Servo/real_machine_test/gamepad_controller.py` sensor snapshot
  reading path

It does not open the servo serial port or command motion.

## Main Folders

- `bt_8bitdo_min/`: minimal 8BitDo Bluetooth gamepad package. It has separate
  read-only test entrypoints and a real-machine control entrypoint.
- `Delta_Gcode_Servo/`: fuller delta robot G-code and servo control code.
- `Delta-Robot/`: original delta robot simulation and model resources.
- `part_model_rev/`: revised SolidWorks/3MF mechanical files for printing or
  CNC-oriented manufacturing export, including `999.STL` for the IMU +
  AprilTag fixture.
- `Dual_Camera_HandEye/`: base-camera plus side-object-camera hand-eye
  coordinate-chain demo that reuses existing AprilTag/IMU snapshots.
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

## License and Attribution

This repository is distributed under the GNU General Public License v3.0. See
[LICENSE](LICENSE).

The upstream delta-robot idea and original reference project came from
[isaac879/Delta-Robot](https://github.com/isaac879/Delta-Robot), which is
MIT-licensed by Isaac Chasteau. The upstream MIT notice is preserved in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). The model files and code in
this repository have been modified for this project's hardware, control stack,
sensors, and manufacturing workflow.
