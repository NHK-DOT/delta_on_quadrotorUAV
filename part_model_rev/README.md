# Revised Mechanical Models

This folder contains revised mechanical model files for the current 78arm Delta
arm prototype.

Included file types:

- `.SLDPRT`: SolidWorks part files for continued CAD edits.
- `.3mf`: print layout / manufacturing export for revised parts.
- `999.STL`: current printable fixture for the IMU and the top-side AprilTag on
  the end effector.

Use:

- Export STL/3MF for 3D printing.
- Print/check `999.STL` when building the current dual-camera hand-eye layout:
  the base camera observes the top-side AprilTag, while the underside carries
  the grasping mechanism and the side camera observes the object.
- Export STEP/DXF from CAD when preparing CNC machining or CAM workflows.
- Recheck hole diameters, bearing fits, servo clearance, tube dimensions, and
  assembly tolerances against the real hardware before manufacturing.
- Measure the final assembled offsets from the tool frame to the AprilTag and
  side object camera, then write them as `tool_T_hand_tag` and
  `tool_T_object_camera` in `../Dual_Camera_HandEye` samples.

The mechanical direction is based on the MIT-licensed
`isaac879/Delta-Robot` project, with revised modeling for this repository's
hardware. See `../THIRD_PARTY_NOTICES.md` for attribution.
