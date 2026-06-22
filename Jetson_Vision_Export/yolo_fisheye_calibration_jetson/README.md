# YOLO Fisheye Calibration For Jetson

This package calibrates the end/side YOLO wrench camera on the Jetson at
`192.168.1.64`. It only opens the camera. It does not open the arm serial port
and does not move the robot.

## Current Board

The printed board says `11 x 8`. That is the number of squares.

OpenCV calibration needs inner corners:

```text
11 x 8 squares = 10 x 7 inner corners
square size = 20 mm = 0.020 m
```

Use:

```text
COLS=10
ROWS=7
SQUARE_SIZE_M=0.020
```

## One-Click Calibration

Run on the Jetson desktop:

```bash
cd ~/Desktop/yolo_fisheye_calibration_jetson
./one_click_fisheye_calibration.sh
```

The script will:

1. Restart `nvargus-daemon`.
2. Clear the current `capture_stream/` and temporary calibration output.
3. Open a GStreamer/NVIDIA preview window.
4. Save frames to `capture_stream/frame_*.jpg`.
5. Wait until you press `Ctrl+C` in the script terminal.
6. Scan saved images for valid checkerboards.
7. Write fisheye intrinsics.

During capture, watch the `gst-launch-1.0` preview window. Keep the full board
visible and sharp. Hold each pose for about 3 seconds.

Recommended poses:

```text
center
left-middle
right-middle
top-middle
bottom-middle
top-left
top-right
bottom-left
bottom-right
near
far
slight left tilt
slight right tilt
slight up tilt
slight down tilt
```

When enough frames are captured, click the script terminal and press:

```text
Ctrl+C
```

Then wait for offline calibration to finish.

## Outputs

Main calibration file:

```text
calibration/yolo_fisheye_camera_intrinsics.json
```

Valid checkerboard images:

```text
calibration/valid_fisheye_frames/
```

Raw captured frames:

```text
capture_stream/
```

Good runs should be archived under:

```text
calibration/saved_runs/
```

One usable saved run currently exists on the Jetson:

```text
calibration/saved_runs/20180129_001345_rms_2p09_samples29/
```

That run used:

```text
valid samples: 29
RMS: 2.09 px
image size: 1280 x 720
pattern: 10 x 7 inner corners
square size: 20 mm
```

## Preview Stutter

If the preview briefly freezes or stutters, it is usually the Jetson
Argus/GStreamer/EGL display path. It does not mean every saved frame is bad.

The calibration flow intentionally uses:

```text
GStreamer preview + frame saving -> offline checkerboard filtering
```

Only images where the complete checkerboard is detected are used. Bad frames are
skipped automatically.

If the camera path becomes stuck:

```bash
sudo systemctl restart nvargus-daemon
./one_click_fisheye_calibration.sh
```

## Useful Variants

Second CSI camera:

```bash
SENSOR_ID=1 ./one_click_fisheye_calibration.sh
```

Rotated camera mount:

```bash
FLIP_METHOD=2 ./one_click_fisheye_calibration.sh
```

Different capture resolution:

```bash
WIDTH=1920 HEIGHT=1080 ./one_click_fisheye_calibration.sh
```

Higher saved-frame rate:

```bash
SAVE_FPS=1/1 ./one_click_fisheye_calibration.sh
```

## Legacy/Backup Entry Points

OpenCV GUI calibration, if HighGUI works on the target:

```bash
bash run_csi_fisheye_calibration.sh
```

No-GUI OpenCV live mode:

```bash
bash run_csi_fisheye_calibration_nogui.sh
```

Offline calibration from existing frames:

```bash
bash run_offline_fisheye_calibration.sh
```

USB camera backup:

```bash
bash run_usb_fisheye_calibration.sh
```
