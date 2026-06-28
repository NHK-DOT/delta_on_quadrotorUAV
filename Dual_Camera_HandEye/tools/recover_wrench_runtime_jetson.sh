#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/nvidia/Desktop/78arm}"
VISION_DIR="${VISION_DIR:-/home/nvidia/vision_starter}"
ENGINE="${ENGINE:-models/wrench_current_manual_affine_gpu_e15_best_320_trt7_fp16.engine}"
TOOL_CAMERA_JSON="${TOOL_CAMERA_JSON:-Dual_Camera_HandEye/output/tool_T_camera_rough_motion_fit_20260628.json}"

cd "${VISION_DIR}"
mkdir -p outputs
if [ -f outputs/trt_yolo_server.pid ]; then
  OLD_PID="$(cat outputs/trt_yolo_server.pid 2>/dev/null || true)"
  if [ -n "${OLD_PID}" ]; then
    kill "${OLD_PID}" 2>/dev/null || true
  fi
fi

nohup python3 scripts/trt_yolo_server.py \
  --engine "${ENGINE}" \
  --source /dev/video1 --width 640 --height 480 --camera-fps 30 --fourcc MJPG \
  --infer-fps 30 --display-fps 15 --capture-thread \
  --label wrench --conf 0.20 --iou 0.45 --max-detections 1 \
  --depth-json /tmp/orbbec_depth_grid.json --depth-max-age 1.0 \
  --camera-hfov-deg 67 --camera-vfov-deg 52 --target-smooth-alpha 0.35 \
  --host 0.0.0.0 --port 8090 \
  > outputs/trt_yolo_server.log 2>&1 < /dev/null &
echo "$!" > outputs/trt_yolo_server.pid

cd "${REPO_DIR}"
bash Delta_Gcode_Servo/real_machine_test/jetson_py36/start_base_tool_feedback_publisher_jetson.sh
TOOL_CAMERA_JSON="${TOOL_CAMERA_JSON}" bash Dual_Camera_HandEye/tools/start_fused_wrench_pose_publisher_jetson.sh
bash Dual_Camera_HandEye/tools/start_wrench_grasp_planner_jetson.sh

echo "preview=http://$(hostname -I | awk '{print $1}'):8090/"
echo "latest=http://$(hostname -I | awk '{print $1}'):8090/latest.json"
echo "base_tool=${REPO_DIR}/Dual_Camera_HandEye/output/base_tool_from_servo_latest.json"
echo "fused=${REPO_DIR}/Dual_Camera_HandEye/output/fused_wrench_pose_latest.json"
echo "plan=${REPO_DIR}/Dual_Camera_HandEye/output/wrench_grasp_sequence_latest.json"
