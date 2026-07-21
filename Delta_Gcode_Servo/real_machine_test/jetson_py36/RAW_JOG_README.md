# 8BitDo Raw Jog

This is the no-vision manual control entrypoint. It bypasses IK/FK and all
camera or AprilTag inputs. It reads the current servo feedback, then changes
only the configured raw target of the selected arm servo.

The default arm IDs are `1,3,4`, matching `RobotParams.servo_ids` and the
current TOML mappings. Raw targets remain clamped to each servo's configured
range; this is the only motion boundary in this mode.

An arm axis whose startup feedback is outside its configured raw range is
disabled for that run. This prevents the first stick input from snapping it to
the nearest configured limit.

On the Jetson:

```bash
cd ~/Desktop/78arm/Delta_Gcode_Servo/real_machine_test/jetson_py36
bash run_raw_jog_py36_jetson.sh
```

Controls:

| Input | Action |
| --- | --- |
| Left stick X | Servo 1 raw jog |
| Left stick Y | Servo 3 raw jog |
| Right stick Y | Servo 4 raw jog |
| A or START | Quit without another motion command |
| Y | Emergency stop and exit |

The controller accepts input only after all three sticks have returned to
neutral once. It stops if arm-servo feedback cannot be read or a serial command
cannot be sent. Use `--arm-directions -1,1,1` only after verifying each axis
direction with the arm unloaded.

Landing-gear commands are excluded from this arm-only mode. Add
`--enable-landing-gear` only when the 5/6 servo feedback is present.
