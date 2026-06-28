# Two-Point Workspace Check - 2026-06-28 22:04

## Purpose

Run two very small arm motions to check whether workspace calibration can trust
commanded servo raw targets or must rely only on feedback FK.

This was not a grasp test and not visual following. The motion was limited to
`+3 ticks` and `-3 ticks` around the current pose, then a restore command back
to the starting raw target.

## Files

```text
two_point_workspace_check_20260628_220441.jsonl
two_point_workspace_check_20260628_220441_summary.json
```

## Result

Start feedback:

```text
raw = {1:530, 2:583, 3:606}
tool FK = x 11.507 mm, y -5.128 mm, z 134.530 mm
YOLO conf = 0.8787
```

Commanded `+3 ticks`:

```text
target raw = {1:533, 2:585, 3:609}
feedback raw = {1:530, 2:583, 3:606}
follow error = {1:3, 2:2, 3:3}
tool FK remained about z 134.530 mm
```

Commanded `-3 ticks`:

```text
target raw = {1:527, 2:579, 3:603}
feedback raw = {1:525, 2:576, 3:599}
follow error = {1:2, 2:3, 3:4}
tool FK moved to about z 131.512 mm
```

Restore command:

```text
target raw = {1:530, 2:583, 3:606}
feedback raw stayed around {1:525, 2:576, 3:599}
follow error after repeated refresh = about {1:5..6, 2:7, 3:7}
```

## Interpretation

The arm did not behave symmetrically around the current pose. A small positive
step did not visibly follow, while the negative step moved. Repeated restore
commands did not return the feedback to the starting raw values.

For workspace calibration this means:

- Do not use commanded raw positions as calibration poses.
- Use feedback FK only.
- Do not fit a precision hand-eye transform from this small motion set.
- Keep the planner dry-run gated by real feedback until the actuator holding
  issue is fixed.

The current rough hand-eye transform can still be used for direction/debugging,
but not for real grasp execution.

## Runtime After Test

Services were restored after the check:

```text
YOLO = ok
base_tool_feedback_publisher = running
fused_wrench_pose_publisher = running
wrench_grasp_planner = running
planner status = tool_out_of_range
```
