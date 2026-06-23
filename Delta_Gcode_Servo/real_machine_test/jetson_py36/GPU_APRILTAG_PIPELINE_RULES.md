# GPU AprilTag Pipeline Rules

## 2026-06-24 Incident

An incorrect change replaced the Jetson 3K fisheye AprilTag runtime path with a
Python/OpenCV CPU detector because the CPU detector recognized the tag after
histogram equalization under the current dark lighting. That was the wrong
engineering decision for this project.

The workspace sampler depends on the high-frame-rate Jetson GPU AprilTag path.
Improving one still-image recognition result is not a valid reason to make the
default runtime fall back to a slower CPU detector.

The CPU fallback commit was reverted. The default sampler launch must remain:

```text
/home/nvidia/Desktop/yolo_fisheye_calibration_jetson/nv_gpu_apriltags_bench/run_fullfov_1280x960_gui.sh
```

## Non-Negotiable Rule

Do not replace the default AprilTag runtime with a CPU detector to solve a
lighting or thresholding problem.

If raw grayscale GPU detection fails in the current lighting, fix the image
before GPU detection while preserving the GPU `nvAprilTags` runtime path.

## Correct Direction

The correct fix is to insert a lightweight image-processing stage between:

```text
3K fisheye camera -> 1280x960 downsampled frame -> GPU nvAprilTagsDetect
```

Allowed directions:

- GPU-side or low-cost pre-processing before `nvAprilTagsDetect`.
- Contrast/brightness/gamma/CLAHE/equalization experiments that keep the GPU
  detector as the production detector.
- Exposure, gain, lighting, and tag placement improvements.
- Benchmarking each change against the original 1280x960 GPU pipeline.

Not allowed as default behavior:

- Python/OpenCV CPU AprilTag detector.
- Any default path that materially reduces frame rate just to make one image
  recognize successfully.

## Baseline To Preserve

The intended production camera path is:

```text
sensor: 3264x2464 full-FOV 3K fisheye
processing frame: 1280x960
detector: NVIDIA nvAprilTags GPU bench
launch: run_fullfov_1280x960_gui.sh
```

Before accepting future changes, record:

- Processing resolution.
- Effective frames per second.
- Tag detection rate.
- Whether the detector is still GPU `nvAprilTags`.
- Exact pre-processing inserted before the GPU detector.

## Current Accepted Fix

The accepted fix for the current dark scene is a detector-input-only
preprocessing stage in the GPU bench:

```text
preprocess: gray_blur_gamma07
path: BGR frame -> grayscale -> 3x3 Gaussian blur -> gamma 0.70 -> BGRA upload -> nvAprilTagsDetect
display: original BGR frame with detection overlay
```

Verified on `192.168.1.80` at `1280x960`:

```text
frames=252
elapsed_s=12.02
fps=20.96
frames_with_tags=132
preprocess_ms avg=2.82
detect_ms avg=21.13
last tag id=3
```

This preserves the GPU detector and the 3K full-FOV downsampled runtime. It is
not a CPU AprilTag fallback.

The follow-up continuity fix keeps the same detector path and adds:

- duplicate-ID filtering in each frame, keeping the lower-hamming/larger-area tag
- bounded GUI last-good overlay, default `120 ms`
- bounded JSON last-good output, default `80 ms`, with `is_held` and `held_ms`

Latest verification:

```text
frames=252
elapsed_s=12.03
fps=20.95
frames_with_tags=206
preprocess_ms avg=2.75
detect_ms avg=23.19
latest JSON id=3
```

ISP exposure/TNR/gain controls were exposed for testing, but they are not enabled
by default because the tested combinations reduced recognition in the current
scene.
