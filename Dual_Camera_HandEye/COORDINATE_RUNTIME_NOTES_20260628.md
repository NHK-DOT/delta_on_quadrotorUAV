# Coordinate Runtime Notes - 2026-06-28

## Problem Found

The old fused-pose service could run from a simulated/static `base_T_tool`:

```bash
--base-tool-rpy 0 0 -0.28 0 0 0
```

That is useful for visualization, but it is not a real actuator pose. If the
physical Delta arm is not exactly at that assumed pose, the virtual arm and the
real arm diverge. The grasp planner then clamps the result into the workspace,
which can hide the frame error and look like target drift or lead.

There was also a JSON compatibility issue: the live YOLO service now publishes
`target.center`, while the fusion script only copied `target.center_px`. This
made downstream image-error fields disappear.

## Runtime Change

Added a read-only bridge:

```bash
Delta_Gcode_Servo/real_machine_test/jetson_py36/publish_base_tool_from_servo_feedback_py36.py
```

It reads LX-225 servo feedback, converts raw positions to Delta FK using the
existing Python 3.6 Jetson workspace mapper, and writes:

```text
Dual_Camera_HandEye/output/base_tool_from_servo_latest.json
```

The fused pose publisher now consumes that JSON as `BASE_TOOL_JSON` by default.
It also records transform source metadata, source age, target center, target
offset, and image size in `fused_wrench_pose_latest.json`.

The planner now refuses unsafe or ambiguous outputs by default:

- stale fused pose
- simulated/static `base_T_tool`
- target outside workspace

It no longer silently clamps an out-of-workspace target into a valid-looking
grasp sequence unless `--allow-workspace-clamp` is explicitly set.

## Current Live Result

Current servo feedback:

```text
raw = {1:542, 2:564, 3:694}
tool FK = x 19.5 mm, y -29.8 mm, z 144.3 mm
```

The tool Z is below the nominal planner range `155..263 mm`.

Current fused wrench result with real servo FK:

```text
wrench base = x -67.6 mm, y -63.5 mm, z 323.7 mm
```

The planner correctly returns:

```text
status = out_of_workspace
violation = z 323.7 > 263.0
```

This is a safer result than the old behavior, where a wrong transform could be
clamped to `z=155 mm` and treated as a planned grasp.

## Interpretation

The current error is not mainly detector jitter. The detector is stable. The
remaining issue is the physical-to-virtual coordinate chain:

1. `base_T_tool` must come from real servo feedback FK, not a fixed simulated
   pose.
2. The `tool_T_object_camera` mount transform may still need a sign/axis check
   because the fused Z currently places the wrench above the configured
   workspace.
3. The arm's current raw feedback is far from the old home/startup raw values,
   so physical pose must be confirmed before commanding motion.

## Start Commands

```bash
cd /home/nvidia/Desktop/78arm
bash Delta_Gcode_Servo/real_machine_test/jetson_py36/start_base_tool_feedback_publisher_jetson.sh
bash Dual_Camera_HandEye/tools/start_fused_wrench_pose_publisher_jetson.sh
bash Dual_Camera_HandEye/tools/start_wrench_grasp_planner_jetson.sh
```

Check outputs:

```bash
cat Dual_Camera_HandEye/output/base_tool_from_servo_latest.json
cat Dual_Camera_HandEye/output/fused_wrench_pose_latest.json
cat Dual_Camera_HandEye/output/wrench_grasp_sequence_latest.json
```

## Next Calibration Step

Use a known physical point or AprilTag target visible to the tool camera and
compare:

- measured point in tool-camera coordinates
- expected point in Delta base coordinates
- current `tool_T_object_camera`

If the Z direction is inverted or the camera mount pitch sign is wrong, fix
`tool_T_object_camera` before enabling any real grasp execution.

## Rough Motion Fit Update

After small remote servo motions around the current pose, a rough axis fit was
created from real feedback FK and live camera observations:

```text
samples = 14
tool feedback range = about 4 mm per axis
affine variation RMSE = about 0.65 mm
orthogonal rotation variation RMSE = about 1.16 mm
```

The motion showed actuator lag/slip: after returning to the start command, raw
feedback did not return exactly to the start raw values. Because of that, all
calibration math used feedback FK only, not commanded raw positions.

Temporary dry-run calibration:

```text
Dual_Camera_HandEye/output/tool_T_camera_rough_motion_fit_20260628.json
```

This file uses the fitted rotation and a deliberately marked rough translation
that normalizes the current object to about `z=200 mm`. It is not a precision
hand-eye calibration. It is only for dry-run planning and coordinate-direction
debugging.

With that rough transform, the live fused pose is stable, but the planner now
also gates on the real servo-feedback tool height. The current tool FK is below
the nominal range:

```text
tool FK z ~= 134.5 mm
planner status = tool_out_of_range
planner nominal tool-z range = 155..280 mm
```

Do not use this rough transform for closed-loop grasp execution until the
mechanical slipping/holding issue is understood and a known physical target is
used to solve translation properly. If the planner returns `tool_out_of_range`,
that is the expected safe behavior and should not be bypassed for real motion.

## Servo Follow Diagnostic Update

A servo follow check was run after the rough calibration motion. The serial
feedback and battery were stable:

```text
battery = 12238..12290 mV
readback jitter = about 1 tick on most samples
current FK z = about 132 mm
```

However, a small symmetric `+5 tick` command still did not fully follow:

```text
target raw = {1:522, 2:573, 3:630}
feedback raw = {1:517, 2:565, 3:624}
follow error = {1:5, 2:8, 3:6}
```

This means the main remaining drift/lead problem is actuator following, not
YOLO jitter. The software defaults were tightened so the virtual target cannot
run far ahead of real feedback:

```text
jetson_wrench_image_follower_py36.py:
  max_servo_raw_s default 30
  max_feedback_lead_ticks default 5

jetson_apriltag_workspace_sampler_py36.py:
  max_servo_raw_s default 45
  max_feedback_lead_ticks default 5
```

A reusable diagnostic was added:

```bash
cd /home/nvidia/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
python3 servo_feedback_follow_check_py36.py --samples 20
python3 servo_feedback_follow_check_py36.py --samples 0 --step-all-ticks 5
```

If the `+5 tick` check cannot pass with low error, do not run visual following
or workspace sampling. Check servo torque/holding mode, linkage load, power
headroom, and whether the servo controller is actually commanding position hold.

Later lifting diagnostics isolated the worst behavior to axis 3:

```text
target raw = {1:551, 2:599, 3:623}
feedback raw = {1:545, 2:592, 3:603}
follow error = {1:6, 2:7, 3:20}
```

The motion test was stopped and a hold-current command was sent. Treat axis 3
follow/holding as the main hardware blocker before any real grasp execution.

Follow-up isolation showed axis 3 can move, but it stalls several ticks short
and then sags back. A hold-refresh option was added to the follower and sampler
so unchanged command targets are resent periodically. This is a software
mitigation only; real execution should still wait for hardware-side axis 3
holding/following checks.
