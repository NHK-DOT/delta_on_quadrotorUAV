#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/nvidia/Desktop/78arm}"
SAMPLES="${SAMPLES:-40}"
INTERVAL_SEC="${INTERVAL_SEC:-0.25}"

cd "${REPO_DIR}"

python3 - "${SAMPLES}" "${INTERVAL_SEC}" <<'PY'
import json
import math
import sys
import time
import urllib.request

samples = int(sys.argv[1])
interval = float(sys.argv[2])

def read_json_path(path):
    with open(path) as f:
        return json.load(f)

def mean(values):
    return sum(values) / len(values) if values else None

def stdev(values):
    if len(values) < 2:
        return 0.0 if values else None
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))

rows = []
for _ in range(samples):
    row = {"ok": False}
    try:
        with urllib.request.urlopen("http://127.0.0.1:8090/latest.json", timeout=1.5) as r:
            yolo = json.loads(r.read().decode("utf-8"))
        base = read_json_path("Dual_Camera_HandEye/output/base_tool_from_servo_latest.json")
        fused = read_json_path("Dual_Camera_HandEye/output/fused_wrench_pose_latest.json")
        plan = read_json_path("Dual_Camera_HandEye/output/wrench_grasp_sequence_latest.json")
        target = yolo.get("target") or {}
        tool = fused.get("transforms", {}).get("base_T_tool", {})
        tool_pos = tool.get("tool_position_base_m") or {}
        wrench = fused.get("wrench_position_base_m") or {}
        row.update({
            "ok": True,
            "yolo_valid": bool(yolo.get("valid")),
            "fused_valid": bool(fused.get("valid")),
            "plan_valid": bool(plan.get("valid")),
            "plan_status": plan.get("status"),
            "fps": yolo.get("fps"),
            "conf": target.get("conf"),
            "tool_z_mm": (tool_pos.get("z") * 1000.0) if tool_pos.get("z") is not None else None,
            "wrench_x_mm": (wrench.get("x") * 1000.0) if wrench.get("x") is not None else None,
            "wrench_y_mm": (wrench.get("y") * 1000.0) if wrench.get("y") is not None else None,
            "wrench_z_mm": (wrench.get("z") * 1000.0) if wrench.get("z") is not None else None,
            "base_raw": base.get("raw"),
        })
    except Exception as exc:
        row["error"] = repr(exc)
    rows.append(row)
    time.sleep(interval)

ok = [r for r in rows if r.get("ok")]
status_counts = {}
for r in ok:
    status_counts[r.get("plan_status")] = status_counts.get(r.get("plan_status"), 0) + 1

summary = {
    "samples_requested": samples,
    "samples_ok": len(ok),
    "yolo_valid": sum(1 for r in ok if r.get("yolo_valid")),
    "fused_valid": sum(1 for r in ok if r.get("fused_valid")),
    "plan_valid": sum(1 for r in ok if r.get("plan_valid")),
    "plan_status_counts": status_counts,
}

for key in ("fps", "conf", "tool_z_mm", "wrench_x_mm", "wrench_y_mm", "wrench_z_mm"):
    values = [float(r[key]) for r in ok if r.get(key) is not None]
    summary[key] = {
        "mean": round(mean(values), 4) if values else None,
        "stdev": round(stdev(values), 4) if values else None,
        "min": round(min(values), 4) if values else None,
        "max": round(max(values), 4) if values else None,
    }

print(json.dumps(summary, indent=2, sort_keys=True))
PY
