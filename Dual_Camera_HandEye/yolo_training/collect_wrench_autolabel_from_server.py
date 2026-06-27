#!/usr/bin/env python3
"""Collect raw frames and YOLO labels from the live wrench detector service."""

import argparse
import base64
import json
import time
import urllib.request
from pathlib import Path


def fetch_json(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fetch_bytes(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


def fetch_snapshot(base_url, timeout):
    latest = fetch_json(base_url.rstrip("/") + "/snapshot.json", timeout=timeout)
    encoded = latest.get("raw_jpeg_base64")
    if not encoded:
        raise RuntimeError("snapshot has no raw_jpeg_base64")
    latest = dict(latest)
    latest.pop("raw_jpeg_base64", None)
    return latest, base64.b64decode(encoded)


def yolo_label_from_latest(latest, min_conf):
    if not latest.get("valid"):
        return None
    target = latest.get("target") or {}
    conf = float(target.get("conf", 0.0) or 0.0)
    if conf < min_conf:
        return None
    box = target.get("box") or {}
    image = latest.get("image") or latest.get("processing_frame") or {}
    w = float(image.get("w", image.get("width", 0)) or 0)
    h = float(image.get("h", image.get("height", 0)) or 0)
    if w <= 1 or h <= 1:
        return None
    x = float(box.get("x", 0))
    y = float(box.get("y", 0))
    bw = float(box.get("w", 0))
    bh = float(box.get("h", 0))
    if bw <= 1 or bh <= 1:
        return None
    cx = (x + bw / 2.0) / w
    cy = (y + bh / 2.0) / h
    return "0 {:.6f} {:.6f} {:.6f} {:.6f}\n".format(cx, cy, bw / w, bh / h), conf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8090")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split", default="train", choices=["train", "val"])
    parser.add_argument("--count", type=int, default=60)
    parser.add_argument("--interval-sec", type=float, default=0.25)
    parser.add_argument("--min-conf", type=float, default=0.35)
    parser.add_argument("--prefix", default="wrench_auto")
    parser.add_argument("--no-snapshot", action="store_true", help="Disable same-frame /snapshot.json capture.")
    args = parser.parse_args()

    image_dir = args.output / "images" / args.split
    label_dir = args.output / "labels" / args.split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    meta_path = args.output / "autolabel_meta.jsonl"

    saved = 0
    attempts = 0
    while saved < args.count and attempts < args.count * 5:
        attempts += 1
        try:
            if args.no_snapshot:
                raise RuntimeError("snapshot disabled")
            latest, jpg = fetch_snapshot(args.base_url, timeout=1.0)
        except Exception:
            try:
                latest = fetch_json(args.base_url.rstrip("/") + "/latest.json", timeout=1.0)
                jpg = fetch_bytes(args.base_url.rstrip("/") + "/raw.jpg", timeout=1.0)
            except Exception as exc:
                print("skip:", exc)
                time.sleep(args.interval_sec)
                continue

        try:
            label = yolo_label_from_latest(latest, args.min_conf)
            if label is None:
                time.sleep(args.interval_sec)
                continue
        except Exception as exc:
            print("skip:", exc)
            time.sleep(args.interval_sec)
            continue

        saved += 1
        stem = "{}_{}_{:05d}".format(args.prefix, int(time.time() * 1000), saved)
        image_path = image_dir / (stem + ".jpg")
        label_path = label_dir / (stem + ".txt")
        image_path.write_bytes(jpg)
        label_text, conf = label
        label_path.write_text(label_text, encoding="utf-8")
        with meta_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps({"image": str(image_path), "label": str(label_path), "conf": conf, "latest": latest}, ensure_ascii=False))
            fh.write("\n")
        print("saved", saved, "conf", round(conf, 3), image_path)
        time.sleep(args.interval_sec)

    data_yaml = args.output / "data.yaml"
    if not data_yaml.exists():
        data_yaml.write_text(
            "path: {}\ntrain: images/train\nval: images/val\nnc: 1\nnames: ['wrench']\n".format(args.output.as_posix()),
            encoding="utf-8",
        )
    print("done saved={} attempts={} dataset={}".format(saved, attempts, args.output))
    return 0 if saved else 1


if __name__ == "__main__":
    raise SystemExit(main())
