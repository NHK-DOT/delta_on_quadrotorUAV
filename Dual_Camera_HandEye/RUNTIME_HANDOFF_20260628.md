# Runtime Handoff - 2026-06-28

## Current Decision

Pause real-motion work until the UAV side is ready and the Delta arm axis-3
holding/following issue is fixed or at least rechecked.

Do not run real grasp execution from the current state. Vision and coordinate
fusion can keep running as dry-run telemetry.

## Verified Runtime

Jetson:

```text
host = 192.168.1.80
workspace = /home/nvidia/Desktop/78arm
vision workspace = /home/nvidia/vision_starter
preview = http://192.168.1.80:8090/
latest = http://192.168.1.80:8090/latest.json
```

Active model:

```text
/home/nvidia/vision_starter/models/wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine
```

Latest no-motion stability check:

```text
samples = 32
yolo_valid = 32
fused_valid = 32
planner status = tool_out_of_range for 32/32 samples
fps mean ~= 23.39
confidence mean ~= 0.8898
tool z mean ~= 134.50 mm
fused wrench position mean ~= x -40.03 mm, y 16.90 mm, z 202.22 mm
```

`tool_out_of_range` is expected and safe. The real tool FK z is below the
planner's configured safe range, so the planner must not output an executable
grasp sequence.

## What Was Changed

- YOLO runtime is recovered through `recover_wrench_runtime_jetson.sh`.
- Recovery now waits for YOLO `/healthz` readiness before starting fused pose
  and planner processes.
- Fused pose consumes real servo-feedback FK through
  `base_tool_from_servo_latest.json`.
- Fused output records transform source, real tool validity, and warnings.
- Planner rejects unsafe cases by default:
  - stale or invalid fused pose
  - simulated/static base tool transform
  - real tool z outside `155..280 mm`
  - target outside workspace unless explicit clamp override is used
- Status and stability scripts were added for repeatable read-only validation.

## Recovery Commands

Full recovery and validation:

```bash
bash /home/nvidia/Desktop/78arm/recovery_snapshot_20260628_201111/recover_services.sh
```

Manual recovery from the live workspace:

```bash
cd /home/nvidia/Desktop/78arm
TOOL_CAMERA_JSON=Dual_Camera_HandEye/output/tool_T_camera_rough_motion_fit_20260628.json \
  bash Dual_Camera_HandEye/tools/recover_wrench_runtime_jetson.sh
bash Dual_Camera_HandEye/tools/check_wrench_runtime_status_jetson.sh
SAMPLES=32 INTERVAL_SEC=0.25 bash Dual_Camera_HandEye/tools/sample_wrench_runtime_stability_jetson.sh
```

## Current Blocker

The main blocker is mechanical/actuator-side, not detector jitter.

Axis 3 can move, but it does not reliably reach or hold the commanded raw
target under the current load. Earlier diagnostics showed axis-3 error around
`7..20 ticks` depending on the test, with sag after settling.

Before any real automatic movement, run:

```bash
cd /home/nvidia/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
python3 servo_feedback_follow_check_py36.py --samples 20
python3 servo_feedback_follow_check_py36.py --samples 0 --step-all-ticks 5 --move-ms 3000 --settle-sec 4 --max-follow-error-ticks 4
```

If axis 3 still fails, do not run visual following, workspace sampling, or
grasp execution.

## Latest Axis Recovery Note

A later small-motion diagnostic showed the arm can passively sag to about:

```text
raw = {1:476, 2:526, 3:551}
tool FK z ~= 111 mm
```

The bounded feedback nudge tool recovered it in staged targets back to about:

```text
raw ~= {1:530, 2:587, 3:608}
tool FK z ~= 135 mm
```

This recovery is useful for diagnostics, but it is not a substitute for fixing
servo holding/torque/load before precision calibration or grasp execution.

An optional hold mode was added to the base-tool feedback publisher. It is
disabled by default because a live trial did not hold the arm tightly enough
near `530,584,607`. Use it only as a controlled diagnostic, not as a final
motion-control solution.

## GitHub Checkpoints

Shared repo:

```text
NHK-DOT/delta_on_quadrotorUAV
```

Relevant commits:

```text
dd01fb1071de98d90374fda64a0cca249e79081e
Gate planner on real tool feedback range

b3e31402ea2ae037a62dd830362662749d4252c9
Add runtime readiness and stability checks
```

## Next Work When UAV Side Is Ready

1. Recheck axis-3 holding/following before any automatic motion.
2. Confirm the arm's physical mounting pose on the UAV.
3. Replace the rough tool-camera transform with a known-target calibration.
4. Keep planner dry-run until tool FK, camera frame, and physical target agree.
5. Only then connect the planner output to a real motion executor.
