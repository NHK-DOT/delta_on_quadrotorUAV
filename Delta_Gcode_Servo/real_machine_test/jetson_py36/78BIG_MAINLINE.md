# 78big Competition Mainline

The competition control path is Bluetooth-first and uses the real arm bus IDs
`1/3/4`. It does not use the legacy Delta IK/FK, camera, AprilTag, or the
nonresponsive ID `2`.

On the NX, start the calibrated controller with:

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
bash run_semantic_jog_78big_jetson.sh
```

Controls:

| Input | Endpoint |
| --- | --- |
| D-pad left/right | left/right |
| D-pad up/down | front/back |
| Right stick up/down | top/bottom |
| Release | hold current feedback pose |
| A | quit |
| Y | immediate stop |

The runtime uses manually sampled raw endpoints, 50 Hz position updates, and a
maximum 5 raw increment per command. It reads all three axes before motion and
stops when feedback cannot be recovered after retries.

## Calibration

Use `jetson_structure_calibration_sampler_py36.py` with
`--vision-mode none --servo-ids 1,3,4` to collect three readings at each of:
`center_mid`, `left_mid`, `right_mid`, `front_mid`, `back_mid`, `top_home`, and
`bottom_safe`. Generate the workspace with:

```bash
python3 build_raw_semantic_workspace.py samples.jsonl --output workspace_1_3_4.json
```

The controller clamps every joint target to the sampled raw bounds. Re-sample
and rebuild before changing the mechanical structure or usable endpoints.
