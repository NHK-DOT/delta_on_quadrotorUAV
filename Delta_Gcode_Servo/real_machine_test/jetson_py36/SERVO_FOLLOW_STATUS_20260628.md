# Servo Follow Status - 2026-06-28

## Current Status

Serial readback is stable when `/dev/ttyUSB0` is not shared by another process.
Battery voltage is stable around `12.25..12.32 V`.

Current restored runtime state after diagnostics:

```text
base_tool_feedback_publisher: running
fused_wrench_pose_publisher: running with rough tool camera transform
wrench_grasp_planner: running, dry-run only
```

Latest observed tool feedback after stopping motion tests:

```text
raw ~= {1:530, 2:588, 3:602}
tool FK ~= x 11 mm, y -4 mm, z 135 mm
```

The tool is still below the nominal planner range. Do not execute grasp motion.

## Diagnostics Run

Read-only stability:

```text
raw stayed around {1:517, 2:565, 3:625}
battery ~= 12277 mV
FK z ~= 132 mm
```

Small command test:

```text
command +5 ticks, move_ms=3000, settle=4s
target raw = {1:522, 2:570, 3:630}
feedback raw = {1:518, 2:565, 3:625}
follow error = {1:4, 2:5, 3:5}
result = fail with strict 4 tick limit
```

Larger command test:

```text
command +20 ticks, move_ms=3000, settle=4s
target raw = {1:537, 2:585, 3:645}
feedback raw = {1:533, 2:580, 3:637}
follow error = {1:4, 2:5, 3:8}
result = moved, but still lagging
```

Next lift step exposed the main issue:

```text
before raw = {1:531, 2:579, 3:603}
target raw = {1:551, 2:599, 3:623}
feedback raw = {1:545, 2:592, 3:603}
follow error = {1:6, 2:7, 3:20}
result = stop, axis 3 did not follow
```

## Conclusion

The remaining drift/lead problem is actuator-side. Axis 3 does not reliably
follow upward raw commands under the current load/state. Vision and coordinate
fusion should remain dry-run until axis 3 holding/following is fixed.

Likely checks:

- Axis 3 linkage binding or mechanical load.
- Axis 3 servo torque/holding mode.
- Servo controller output or cable for axis 3.
- Power headroom under motion, not just idle voltage.
- Whether controller commands are being rate-limited or ignored near this pose.

## Recovery

A recovery snapshot was saved on the Jetson:

```text
/home/nvidia/Desktop/78arm/recovery_snapshot_20260628_190811
```

Service recovery command:

```bash
bash /home/nvidia/Desktop/78arm/recovery_snapshot_20260628_190811/recover_services.sh
```

Before any future automatic movement, run:

```bash
cd /home/nvidia/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
python3 servo_feedback_follow_check_py36.py --samples 20
python3 servo_feedback_follow_check_py36.py --samples 0 --step-all-ticks 5 --move-ms 3000 --settle-sec 4 --max-follow-error-ticks 4
```

If axis 3 does not follow within the limit, do not run visual following,
workspace sampling, or grasp execution.
