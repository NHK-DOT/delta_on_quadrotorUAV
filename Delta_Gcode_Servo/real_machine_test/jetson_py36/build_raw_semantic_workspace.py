#!/usr/bin/env python3
"""Build a conservative raw-joint workspace from manually labelled poses."""

from __future__ import print_function

import argparse
import json
import os
import statistics


REQUIRED_LABELS = (
    "center_mid", "left_mid", "right_mid", "front_mid", "back_mid", "top_home", "bottom_safe",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples", help="raw-only sampler JSONL")
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    grouped = {}
    servo_ids = None
    with open(args.samples, "r") as fh:
        for line in fh:
            if not line.strip():
                continue
            item = json.loads(line)
            ids = tuple(int(value) for value in item.get("servo_ids", ()))
            raw = item.get("servo_raw", {})
            if len(ids) != 3 or any(str(servo_id) not in raw for servo_id in ids):
                continue
            if servo_ids is None:
                servo_ids = ids
            if ids != servo_ids:
                continue
            grouped.setdefault(item.get("label", ""), []).append(
                [int(raw[str(servo_id)]) for servo_id in servo_ids]
            )

    missing = [label for label in REQUIRED_LABELS if label not in grouped]
    if servo_ids is None or missing:
        raise SystemExit("missing required labels: %s" % ", ".join(missing))

    labels = {}
    for label in REQUIRED_LABELS:
        samples = grouped[label]
        labels[label] = [int(round(statistics.median(row[index] for row in samples))) for index in range(3)]

    lower = [min(labels[label][index] for label in REQUIRED_LABELS) for index in range(3)]
    upper = [max(labels[label][index] for label in REQUIRED_LABELS) for index in range(3)]
    payload = {
        "format": "raw_semantic_workspace_v1",
        "servo_ids": list(servo_ids),
        "labels": labels,
        "raw_bounds": {str(servo_ids[index]): [lower[index], upper[index]] for index in range(3)},
        "sample_counts": {label: len(grouped[label]) for label in REQUIRED_LABELS},
    }
    parent = os.path.dirname(os.path.abspath(args.output))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    with open(args.output, "w") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")
    print(json.dumps(payload, sort_keys=True))


if __name__ == "__main__":
    main()
