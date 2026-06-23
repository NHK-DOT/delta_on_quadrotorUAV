# 78arm

中文说明: [README.zh-CN.md](README.zh-CN.md)

Delta-arm simulation, calibration tools, servo driver code, sensor tools,
hand-eye vision experiments, and the current real-machine Delta arm controller.

License: GNU GPL v3.0. See [LICENSE](LICENSE). Upstream MIT notices are kept in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Current Jetson field mainline:
`Delta_Gcode_Servo/real_machine_test/jetson_py36/run_sampler_py36_jetson.sh`.
It runs on the Jetson Xavier NX at `192.168.1.80` and combines 3K fisheye
AprilTag detection, 8BitDo low-speed manual motion, servo feedback preflight,
and workspace sampling.

## Project Context

This repository is for a lightweight delta-arm mounted on or tested for a UAV
platform. The current real-machine control path is not just a gamepad demo:
`Delta_Gcode_Servo/real_machine_test/gamepad_controller.py` reads operator
input, converts it into delta-arm XYZ motion, solves inverse kinematics, maps
joint angles into raw bus-servo positions, and sends those commands through a
Hiwonder xArm 1.6 servo driver board to the physical servos.

The mechanical concept and part of the early delta-robot modeling/control
direction were based on Isaac Chasteau's MIT-licensed
[isaac879/Delta-Robot](https://github.com/isaac879/Delta-Robot). This project
changes the modeling, hardware layout, control code, sensors, and deployment
pipeline, but it still acknowledges that upstream project as the original idea
source.

The 8BitDo compatibility files exist because the Xbox Series wireless
controller was not stable on the Jetson Xavier NX Bluetooth stack used here. In
local testing, that controller repeatedly bounced between connected and
disconnected states. The workaround is to use an 8BitDo Ultimate 2 Wireless
controller and read the Linux input event device directly, without relying on
pygame or SDL mappings.

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

## Current Jetson Sampling Mainline

The actively maintained field path for the current hardware is:

```text
Jetson Xavier NX 192.168.1.80
  -> 3K fisheye full-FOV downsample AprilTag JSON
  -> Delta_Gcode_Servo/real_machine_test/jetson_py36/run_sampler_py36_jetson.sh
  -> read-only preflight: gamepad, AprilTag, servo feedback, home_raw
  -> 8BitDo low-speed manual target XYZ
  -> Delta IK/FK and raw LX bus-servo mapping
  -> Hiwonder xArm 1.6 servo driver board
  -> press B to sample AprilTag XYZ + servo raw for model fitting
```

Offline model fitting and workspace scanning are handled by:

```text
Delta_Gcode_Servo/real_machine_test/workspace_model_tools.py
```

The older `Delta_Gcode_Servo/real_machine_test/gamepad_controller.py` remains in
the repository as the previous local-controller implementation, but it is not
the preferred path for the `192.168.1.80` Jetson Xavier NX sampling workflow.

## Hardware Control Path

```text
Gamepad / operator input
  -> Jetson Xavier NX 192.168.1.80
  -> jetson_py36 sampler
  -> XYZ target and inverse kinematics
  -> raw LX bus-servo position commands
  -> USB serial adapter
  -> Hiwonder xArm 1.6 servo driver board
  -> physical LX bus servos
```

The sampler can move real servos after the typed `YES` confirmation. The
preflight path is read-only and checks 8BitDo input, AprilTag JSON freshness,
servo feedback, and whether servo 1/2/3 are near the configured initial
`home_raw` position.

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

## Jetson Vision Deployment Package

`Jetson_Vision_Export/` stores the recovered Jetson vision deployment archive
and its deployment notes. The package is tracked with Git LFS because the
archive is about 250 MB and exceeds normal GitHub file limits.

It includes the application-level vision stack exported from the Jetson SSD:
TensorRT YOLO service files, ONNX/engine model files, Orbbec SDK files, udev
rules, and systemd service units. The package does not contain or install a
kernel, DTB, DTBO, UEFI image, or camera device-tree overlay.

The included README records the tested migration path to `192.168.1.64`
JetPack 4 / Ubuntu 18.04, including TensorRT 7 engine rebuilds for the COCO,
wrench, wrench-public-negative, and snow-king models.

## Main Folders

- `Delta_Gcode_Servo/`: current Jetson sampling entrypoint, workspace fitting
  tools, G-code tools, Delta IK, raw servo mapping, and base-camera-to-tool
  helpers.
- `bt_8bitdo_min/`: minimal compatibility dependency for the Jetson Xavier NX
  sampler. It now keeps only the 8BitDo evdev reader, bus-servo driver,
  gamepad config, and Bluetooth/input permission installer.
- `Jetson_Vision_Export/`: Git LFS tracked Jetson vision deployment archive,
  checksum, installer, and deployment README for the TensorRT/Orbbec vision
  service package.
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

## Jetson Xavier NX Field Steps

Deploy the current package to `192.168.1.80`:

```powershell
cd C:\Users\hanjuncheng\Desktop\78arm
python Delta_Gcode_Servo\real_machine_test\jetson_py36\deploy_to_jetson.py --password nvidia
```

Install 8BitDo input permissions on the Jetson if needed:

```bash
ssh nvidia@192.168.1.80
cd ~/Desktop/78arm/bt_8bitdo_min
bash deploy/install_ubuntu18.sh
```

Run the read-only preflight:

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
python3 jetson_workspace_preflight.py --port /dev/ttyUSB0 --hand-tag-id 3
```

Run the sampler:

```bash
HAND_TAG_ID=3 bash run_sampler_py36_jetson.sh
```

The sampler writes `samples.csv` and `samples.jsonl` under:

```text
Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/YYYYMMDD_HHMMSS/
```

Fit the workspace model after sampling:

```bash
python3 Delta_Gcode_Servo/real_machine_test/workspace_model_tools.py fit \
  Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/YYYYMMDD_HHMMSS/samples.jsonl \
  --output-dir Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/YYYYMMDD_HHMMSS/model \
  --compute-workspace
```

## License and Attribution

This repository is distributed under the GNU General Public License v3.0. See
[LICENSE](LICENSE).

The upstream delta-robot idea and original reference project came from
[isaac879/Delta-Robot](https://github.com/isaac879/Delta-Robot), which is
MIT-licensed by Isaac Chasteau. The upstream MIT notice is preserved in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). The model files and code in
this repository have been modified for this project's hardware, control stack,
sensors, and manufacturing workflow.
