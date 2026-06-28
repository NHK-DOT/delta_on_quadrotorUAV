#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/nvidia/Desktop/78arm}"

echo "== processes =="
ps -eo pid,ppid,cmd | grep -E 'trt_yolo|depth_grid|publish_base_tool|publish_fused|wrench_grasp' | grep -v grep || true

echo
echo "== yolo health =="
python3 - <<'PY'
import json
import urllib.request
for url in ("http://127.0.0.1:8090/healthz", "http://127.0.0.1:8090/latest.json"):
    try:
        with urllib.request.urlopen(url, timeout=2.0) as r:
            data=json.loads(r.read().decode("utf-8"))
        if url.endswith("latest.json"):
            target=data.get("target") or {}
            print(url, {"valid": data.get("valid"), "fps": data.get("fps"), "conf": target.get("conf"), "box": target.get("box")})
        else:
            print(url, data)
    except Exception as exc:
        print(url, "ERROR", repr(exc))
PY

echo
echo "== outputs =="
cd "${REPO_DIR}"
python3 - <<'PY'
import json
for path in (
    "Dual_Camera_HandEye/output/base_tool_from_servo_latest.json",
    "Dual_Camera_HandEye/output/fused_wrench_pose_latest.json",
    "Dual_Camera_HandEye/output/wrench_grasp_sequence_latest.json",
):
    try:
        data=json.load(open(path))
        print(path, json.dumps({
            "valid": data.get("valid"),
            "status": data.get("status") or data.get("mode"),
            "raw": data.get("raw") or data.get("transforms", {}).get("base_T_tool", {}).get("raw"),
            "position": data.get("object", {}).get("position_base_mm") or data.get("wrench_position_base_m") or data.get("tool_position_base_mm"),
        }, ensure_ascii=False))
    except Exception as exc:
        print(path, "ERROR", repr(exc))
PY
