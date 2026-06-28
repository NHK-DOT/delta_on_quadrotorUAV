# Axis Follow Diagnostic - 2026-06-28 22:10

## Purpose

Try to recover the Delta arm close to the prior start raw pose and diagnose
whether the follow problem is a total actuator failure or a bounded
deadband/load/holding issue.

No grasp execution was run. Motions were small and near the current pose.

## Closed-Loop Restore Result

Desired raw:

```text
{1:530, 2:583, 3:606}
```

Before compensation:

```text
feedback raw = {1:524, 2:576, 3:598}
error = {1:6, 2:7, 3:8}
tool FK z ~= 131.19 mm
```

Bounded command lead:

```text
command raw = {1:536, 2:590, 3:614}
lead over desired = {1:6, 2:7, 3:8}
```

After one closed-loop correction:

```text
feedback raw = {1:530, 2:584, 3:607}
error ~= {1:0, 2:-1, 3:-1}
tool FK z ~= 134.84 mm
```

This means the actuator path is not completely dead. A bounded feedback
correction can overcome part of the deadband/load issue near this pose.

## Single-Axis Probe

The single-axis probe started from:

```text
baseline raw = {1:530, 2:584, 3:607}
```

Axis 1 `+5 ticks` command did not move axis 1, and the other axes sagged:

```text
target raw = {1:535, 2:584, 3:607}
feedback raw = {1:530, 2:581, 3:603}
moved ticks = {1:0, 2:-3, 3:-4}
```

During the restore loop, one serial read failed with:

```text
controller response missing servo IDs: [2]
```

The probe was stopped at that point. The arm was then recovered to within about
`3 ticks` of the baseline and services were restarted.

## Interpretation

The issue is likely a mix of:

- static friction or load near the current pose
- servo deadband/holding weakness
- passive sag on axes not being actively moved
- occasional serial read packet loss or controller response loss

For calibration and control:

- Always use feedback FK, not commanded raw.
- Use bounded closed-loop raw correction for small recovery moves.
- Reject calibration samples when final follow error is above tolerance.
- Do not run larger workspace sampling until the serial read path and holding
  behavior are stable.

## New Tool

Reusable bounded correction tool:

```bash
cd /home/nvidia/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
python3 servo_feedback_closed_loop_nudge_py36.py \
  --target-raw 530,584,607 \
  --tolerance-ticks 3 \
  --max-lead-ticks 10
```

This is a diagnostic/recovery helper only. It should not be treated as a final
grasp executor.

## Later Sag and Recovery

After services were restored, feedback later showed a larger passive drop:

```text
raw = {1:476, 2:526, 3:551}
tool FK z ~= 111.36 mm
```

The new bounded correction tool was then used in three staged targets:

```text
target 500,550,575 -> passed, final error <= 4 ticks
target 520,570,595 -> passed, final error <= 3 ticks
target 530,584,607 -> passed, final error <= 5 ticks
```

Recovered runtime state:

```text
raw ~= {1:529, 2:588, 3:607}
tool FK z ~= 135.29 mm
```

After a 20 second hold check:

```text
raw ~= {1:530, 2:587, 3:608}
tool FK z ~= 135.46 mm
```

This confirms the bounded feedback nudge can recover the arm near the previous
pose, but the passive sag event means the actuator holding problem is real and
must be treated as a hardware/control blocker for precision calibration.

## Hold Mode Trial

The feedback FK publisher was extended with an optional hold mode so the same
serial owner can both publish real FK and periodically send bounded raw
corrections. A trial with:

```text
HOLD_TARGET_RAW=530,584,607
```

prevented a full collapse to `z ~= 111 mm`, but it did not hold the arm tightly
at the target. Stronger hold settings also failed to keep the feedback near the
target and interacted poorly with the current load/deadband.

Final decision:

- keep hold mode available for controlled diagnostics
- keep hold mode disabled by default
- do not use hold mode as proof that the arm is ready for precision calibration
- fix servo torque/holding/load before any real grasp execution
