# Bottom stereo integration plan

The next hardware step is to add a bottom-facing stereo recognition camera and
merge it with the existing upper AprilTag hand-eye chain.

## Frame chain

Keep the Delta controller frame as the motion base. The current upper-camera
field data gives a coarse relationship between Delta FK positions and the
AprilTag vision frame:

```text
apriltag_vision_T_delta_fk
```

The bottom stereo path should produce detections in its own camera frame:

```text
bottom_stereo_T_object
```

After mounting the stereo camera on the end effector, calibrate:

```text
tool_T_bottom_stereo
tool_T_pickup
```

Then object position in the Delta base is:

```text
delta_base_T_object =
    delta_base_T_tool
  * tool_T_bottom_stereo
  * bottom_stereo_T_object
```

The pickup target is:

```text
delta_base_T_pickup_target =
    delta_base_T_object
  * inverse(tool_T_pickup)
```

## Practical order

1. Keep using the upper AprilTag to estimate and validate `delta_base_T_tool`.
2. Mount the bottom stereo rigidly and record its mechanical offset from the
   tool/pickup center.
3. Make the bottom stereo detector write a small JSON snapshot with timestamp,
   object ID, confidence, and either metric XYZ or a ray plus depth.
4. Run offline hand-eye checks before connecting the output to motion.
5. Start with XY-only visual following at fixed Z.
6. Add descent and pickup only after the XY sign and scale are verified.

## Current field result

The 2026-06-26 AprilTag workspace run is stored at:

```text
Delta_Gcode_Servo/real_machine_test/jetson_py36/samples/20260626_151933/
```

Use:

```text
model_z_ge_155/rigid_alignment_report.json
```

as a coarse prior only. Its RMS residual is about 40 mm, so it should not be
used as the final precision hand-eye transform.

