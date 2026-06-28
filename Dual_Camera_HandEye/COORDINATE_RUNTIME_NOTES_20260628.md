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
