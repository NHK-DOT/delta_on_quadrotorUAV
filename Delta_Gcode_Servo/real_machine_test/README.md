# Delta real machine test

This folder is the current mainline for the real Delta arm hand-controller test.

Use `gamepad_controller.py` for Xbox control. The older minimal 8BitDo test package is not the control mainline.

## Safety model

`gamepad_controller.py` now treats the driver feedback as the real state source before control starts.

Startup flow:

1. Connect serial bus and Xbox controller.
2. Initialize the configured reference pose from `RobotParams.home_position`.
3. Read current servo feedback from the driver board.
4. If feedback is already near the reference raw positions, enter control directly.
5. If feedback is not near the reference pose, print current and target raw positions.
6. Only after typing `HOME`, slowly move from the current feedback state to the configured reference pose.

The startup HOME speed is intentionally lower than normal manual speed:

```python
self.startup_home_servo_speed_ticks_per_sec = 120.0
self.startup_home_timeout_sec = 20.0
```

Do not run this script with the arm powered if the mechanical linkage is blocked, assembled incorrectly, or outside a known safe posture. The script will command real servos after the `HOME` or `PLAY` confirmations.

## Xbox controls

| Control | Action |
| --- | --- |
| D-pad left/right | Move model X |
| D-pad up/down | Move model Y |
| Right stick up/down | Move model Z |
| A | Quit |
| B | Sample current feedback point |
| X | Cycle safe-scan axis mode |
| Y | Cycle sensor-frame mode |
| LB/RB | Move tooling servo if servo 4 is configured |
| BACK | Toggle A/B playback mode |
| START | Confirmed playback using the latest two sampled points |

The current playback mode and sample count are written to `runtime_status.log`.

## A/B playback modes

Press `B` at two positions to store the latest A/B pair. Press `BACK` to choose a mode, then press `START`. The terminal asks for `PLAY` before motion starts.

`LINE` mode:

- Uses the two latest sampled points.
- Requires the current feedback position to be close to either endpoint.
- Moves in a Cartesian straight-line interpolation to the other endpoint.

`PICK_PLACE` mode:

- Uses the two latest sampled points as A and B.
- Runs: current pose -> home -> A -> lifted A -> lifted B -> B -> lifted B -> home.
- The lift Z is the highest of home/A/B plus `playback_lift_clearance_mm`, clipped by `workspace_z_max`.

Relevant playback parameters in `gamepad_controller.py`:

```python
self.playback_step_mm = 2.0
self.playback_speed_mm_per_sec = 70.0
self.playback_endpoint_tolerance_mm = 18.0
self.playback_lift_clearance_mm = 30.0
```

## Drift and workspace guards

Manual control no longer lets an invisible virtual endpoint run far away from the real feedback pose.

The controller:

- Clamps target XYZ into the configured workspace before inverse kinematics.
- Also applies a circular XY radius clamp using `workspace_xy_max`.
- Rejects targets whose IK or servo raw mapping is outside configured limits.
- Uses feedback re-anchoring when the commanded target is too far ahead of feedback.
- Separates startup raw feedback from FK pose validity, so startup HOME can still use readable servo raw feedback before formal control.
- Keeps normal servo commands rate-limited by `max_servo_speed_ticks_per_sec`.

Main guard parameters:

```python
self.enforce_workspace_bounds = True
self.enable_stall_guard = True
self.max_target_lead_mm = 18.0
self.target_reanchor_error_mm = 45.0
self.max_servo_speed_ticks_per_sec = 400.0
```

## Geometry parameters currently used

The Delta geometry is currently not loaded from a TOML/YAML file. It is read from the `RobotParams` dataclass in:

`Delta_Gcode_Servo/delta_gcode_servo/config.py`

Parameters directly affecting IK/FK and workspace:

| Parameter | Current value | Meaning |
| --- | ---: | --- |
| `l1` | `100.0` | active upper arm length |
| `l2` | `150.0` | passive lower arm length |
| `l3` | `48.0` | platform/end-effector offset radius used by the model |
| `servo_offset_x` | `75.0` | servo axis X offset in each arm plane |
| `servo_offset_y` | `0.0` | currently defined but not materially used by the existing IK |
| `servo_offset_z` | `41.231` | servo axis Z offset |
| `workspace_z_min` | `110.0` | lower Z safety bound |
| `workspace_z_max` | `280.0` | upper Z safety bound |
| `workspace_xy_max` | `150.0` | X/Y square bound and XY radius bound |
| `ball_joint_angle_limit` | `34.1 deg` | passive joint angular limit used by IK |
| `servo_distribution` | `0, 120, 240 deg` | arm angular placement |
| `home_position` | `[0, 0, 240]` | reference pose used at startup |

Servo raw limits and mapping are loaded from:

`lx225_tool_demo/config/lx225_tool.demo.toml`

Current main arm mappings:

| Servo | raw min | raw max | step |
| --- | ---: | ---: | ---: |
| 1 | `0` | `834` | `4` |
| 2 | `0` | `770` | `4` |
| 3 | `0` | `816` | `4` |

Servo 4 is treated as optional tooling and uses its own mapping in the same TOML file.

## Should the arm dimensions be measured?

Yes, but do it in this order:

1. First verify the current controller can start, HOME slowly, sample two points, and replay at low speed.
2. Then measure and update the model dimensions.

The most important measurements are `l1`, `l2`, `l3`, `servo_offset_x`, `servo_offset_z`, the real raw position at the mechanical reference pose for each servo, and the actual safe raw min/max before hitting linkage limits. The workspace bounds should be made conservative after the geometry and raw mapping are correct.

## Runtime files

The controller may generate:

- `gamepad_diagnostic.log`
- `runtime_status.log`
- `workspace_points.csv`

These are runtime outputs and should not be committed. `.gitignore` already ignores logs and the two machine-generated real-machine-test files.
