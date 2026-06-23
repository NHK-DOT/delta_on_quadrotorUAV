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
