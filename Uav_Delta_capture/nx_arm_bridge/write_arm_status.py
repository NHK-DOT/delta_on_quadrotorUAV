#!/usr/bin/env python3
"""Write a local, non-flight-control arm state for the NX bridge."""

import argparse
import json
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state", choices=("IDLE", "GRASPING", "GRASPED", "FAILED"))
    parser.add_argument("--detail", default="")
    parser.add_argument("--output", type=Path, default=Path("/tmp/78arm_arm_status.json"))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"state": args.state, "detail": args.detail, "timestamp_unix": time.time()}, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
