# 2026-06-26 AprilTag workspace field samples

This run was captured on the Jetson sampler with the 8BitDo controller and hand
AprilTag ID 3.

## Data

- Raw run directory: `samples/20260626_151933`
- Raw accepted samples: 59
- Samples below the current Z floor (`fk_z_mm < 155`): 6
- Filtered samples for fitting: 53
- Filtered files:
  - `samples_z_ge_155.csv`
  - `samples_z_ge_155.jsonl`

Dropped sample indices: `43, 44, 45, 46, 47, 51`.

## Results

`model_z_ge_155/fit_report.json` is the existing geometry-fit attempt. Its
residual is too high for controller use, which indicates the AprilTag vision
frame is not just translated from the Delta FK frame.

`model_z_ge_155/rigid_alignment_report.json` estimates a coarse rigid transform:

```text
vision_xyz_mm ~= R * delta_fk_xyz_mm + t_mm
```

Rigid alignment residuals:

- RMS norm: 39.79 mm
- Mean norm: 36.14 mm
- Max norm: 72.25 mm

This is useful as a frame-sign and hand-eye prior, but it is not precise enough
for final pickup control.

## Safety notes

The sampler was updated after this run so that:

- command raw is limited relative to the latest feedback raw;
- command candidates are checked with FK before sending and held if they would
  continue below the configured Z floor;
- samples below `--z-min-mm` are refused.

