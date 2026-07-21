# Hardware And Asset Index

This index separates the finished 78arm project into hardware, electronics,
control, and evidence. It is the entry point for a GitHub reader who needs to
understand what belongs to the physical system before opening a code module.

## Mechanical Hardware

| Location | Scope |
| --- | --- |
| `part_model_rev/` | Current 78arm Delta-arm CAD, printable parts, and manufacturing exports. |
| `drawings/desktop_cad_import_20260714/` | Frozen CAD import; see `drawings/README.md` for the self-developed versus reference boundary. |
| `images/arm/` | Arm, end-effector, linkage, bearing, and camera-mount photos. |
| `images/airframe/` | Airframe overview, carbon-tube arm, propeller, and assembly photos. |

## Electronics And Boards

| Component | Hardware role | Related source |
| --- | --- | --- |
| STM32MP257F-DK | Onboard mission controller, ROS 2, UWB/FCU integration, mission logic | `STM32MP257_UWB_FlightControl/` |
| Flight controller and ESC | Attitude/local-position control and propulsion drive | `images/electronics/` and `Uav_Delta_capture/` |
| Jetson Xavier NX | AprilTag/YOLO perception, hand-eye processing, arm-side compute | `Jetson_Vision_Export/`, `Dual_Camera_HandEye/` |
| Hiwonder xArm 1.6 driver board and LX bus servos | Delta-arm actuator interface | `Delta_Gcode_Servo/`, `lx225_tool_demo/`, `Bus_Servo/` |
| Power distribution and UWB | Power harness, UWB beacon/AOA integration | `images/electronics/`, `STM32MP257_UWB_FlightControl/` |

## Control And Verification

| Location | Purpose |
| --- | --- |
| `Delta_Gcode_Servo/real_machine_test/` | Real-machine Delta-arm control, calibration, and Jetson field scripts. |
| `AprilTag_Vision/`, `Dual_Camera_HandEye/`, `IMU/` | Perception, hand-eye, and IMU support tools. |
| `Uav_Delta_capture/` | Nested UAV integration repository and NX-to-MP257 observation bridge. |
| `images/debug/` | Selected field/bench debugging photos and visual algorithm outputs. |

## Supporting Project Assets

| Location | Purpose | Publishing treatment |
| --- | --- | --- |
| `最新ALX-AOA-FIT跟随套件开发资料/` | Vendor UWB AOA/FIT kit manuals and example code. | Preserve vendor ownership and exclude the bundled training video. |
| `tool_vision_web/` | Small local web tool for vision result inspection. | Publish source and selected screenshots; ignore Python cache. |
| `TexTest/` | 78arm technical-report LaTeX source, figures, and final PDF. | Keep `.tex`, figures, and reviewed PDF; ignore LaTeX intermediate files. |
| `Gcode/` | Reproducible Delta path inputs and converted servo data. | Publish as source data, not as runtime output. |
| `runs/` and `78arm_recovery_backup_20260713/` | Local inference results and recovery copy. | Workstation-only; excluded from GitHub. |

## GitHub Publishing Rules

- Git LFS is required for CAD (`.SLDPRT`, `.SLDASM`, `.STEP`, `.STL`, `.3mf`) and Jetson deployment archives.
- Do not commit local recovery snapshots, virtual environments, editor folders, generated model runs, raw videos, or temporary SolidWorks lock files.
- Preserve third-party notices and source boundaries. RobotPilots CAD is reference-only and retains its original non-commercial condition.
- Prefer a short README-ready image in `images/`; keep raw captures and repeated test material in local archive storage.
