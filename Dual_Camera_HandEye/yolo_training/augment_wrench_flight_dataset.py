#!/usr/bin/env python3
"""Create flight-robust photometric augmentations for a YOLO wrench dataset.

Input layout is the normal Ultralytics layout:

dataset/
  images/train/*.jpg
  labels/train/*.txt
  images/val/*.jpg
  labels/val/*.txt

The script preserves labels because these augmentations do not change geometry.
Use YOLO's built-in training augmentations for scale, translation, perspective,
mosaic, and close/far object size variation.
"""

import argparse
import random
import shutil
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def motion_blur(image: np.ndarray, kernel_size: int, angle_deg: float) -> np.ndarray:
    kernel_size = max(3, int(kernel_size) | 1)
    kernel = np.zeros((kernel_size, kernel_size), dtype=np.float32)
    kernel[kernel_size // 2, :] = 1.0
    matrix = cv2.getRotationMatrix2D((kernel_size / 2 - 0.5, kernel_size / 2 - 0.5), angle_deg, 1.0)
    kernel = cv2.warpAffine(kernel, matrix, (kernel_size, kernel_size))
    kernel_sum = float(kernel.sum())
    if kernel_sum > 1e-6:
        kernel /= kernel_sum
    return cv2.filter2D(image, -1, kernel)


def adjust_gamma(image: np.ndarray, gamma: float) -> np.ndarray:
    table = np.array([((i / 255.0) ** gamma) * 255.0 for i in range(256)]).astype("uint8")
    return cv2.LUT(image, table)


def add_noise(image: np.ndarray, sigma: float) -> np.ndarray:
    noise = np.random.normal(0.0, sigma, image.shape).astype(np.float32)
    out = image.astype(np.float32) + noise
    return np.clip(out, 0, 255).astype(np.uint8)


def jpeg_reencode(image: np.ndarray, quality: int) -> np.ndarray:
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return image
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    return decoded if decoded is not None else image


def lowres_resample(image: np.ndarray, scale: float) -> np.ndarray:
    h, w = image.shape[:2]
    nw = max(16, int(w * scale))
    nh = max(16, int(h * scale))
    small = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_LINEAR)


def random_variant(image: np.ndarray, rng: random.Random) -> np.ndarray:
    out = image.copy()
    choice_count = rng.randint(2, 5)
    ops = ["motion", "gaussian", "gamma", "contrast", "noise", "jpeg", "lowres"]
    for op in rng.sample(ops, k=choice_count):
        if op == "motion":
            out = motion_blur(out, rng.choice([5, 7, 9, 11, 13, 17]), rng.uniform(0, 180))
        elif op == "gaussian":
            out = cv2.GaussianBlur(out, (rng.choice([3, 5, 7]), rng.choice([3, 5, 7])), 0)
        elif op == "gamma":
            out = adjust_gamma(out, rng.uniform(0.55, 1.65))
        elif op == "contrast":
            alpha = rng.uniform(0.65, 1.45)
            beta = rng.uniform(-35, 35)
            out = cv2.convertScaleAbs(out, alpha=alpha, beta=beta)
        elif op == "noise":
            out = add_noise(out, rng.uniform(3, 18))
        elif op == "jpeg":
            out = jpeg_reencode(out, rng.randint(35, 85))
        elif op == "lowres":
            out = lowres_resample(out, rng.uniform(0.35, 0.75))
    return out


def iter_images(image_dir: Path):
    for path in sorted(image_dir.rglob("*")):
        if path.suffix.lower() in IMAGE_EXTS:
            yield path


def copy_label(src_dataset: Path, dst_dataset: Path, split: str, image_path: Path, out_stem: str) -> None:
    rel = image_path.relative_to(src_dataset / "images" / split)
    label_src = src_dataset / "labels" / split / rel.with_suffix(".txt")
    label_dst = dst_dataset / "labels" / split / rel.parent / f"{out_stem}.txt"
    label_dst.parent.mkdir(parents=True, exist_ok=True)
    if label_src.exists():
        shutil.copy2(label_src, label_dst)
    else:
        label_dst.write_text("", encoding="utf-8")


def augment_split(src_dataset: Path, dst_dataset: Path, split: str, variants: int, seed: int) -> None:
    rng = random.Random(seed + hash(split) % 10000)
    src_image_dir = src_dataset / "images" / split
    dst_image_dir = dst_dataset / "images" / split
    dst_image_dir.mkdir(parents=True, exist_ok=True)

    count = 0
    for image_path in iter_images(src_image_dir):
        rel = image_path.relative_to(src_image_dir)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"skip unreadable image: {image_path}")
            continue

        original_stem = rel.stem
        original_dst = dst_image_dir / rel.parent / f"{original_stem}{image_path.suffix.lower()}"
        original_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image_path, original_dst)
        copy_label(src_dataset, dst_dataset, split, image_path, original_stem)
        count += 1

        for index in range(variants):
            out_stem = f"{original_stem}_flightaug_{index + 1:02d}"
            out_path = dst_image_dir / rel.parent / f"{out_stem}.jpg"
            out_path.parent.mkdir(parents=True, exist_ok=True)
            aug = random_variant(image, rng)
            cv2.imwrite(str(out_path), aug, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            copy_label(src_dataset, dst_dataset, split, image_path, out_stem)
            count += 1
    print(f"{split}: wrote {count} images")


def write_data_yaml(src_dataset: Path, dst_dataset: Path) -> None:
    src_yaml = src_dataset / "data.yaml"
    names = "['wrench']"
    nc = 1
    if src_yaml.exists():
        for line in src_yaml.read_text(encoding="utf-8").splitlines():
            if line.strip().startswith("names:"):
                names = line.split(":", 1)[1].strip()
            if line.strip().startswith("nc:"):
                nc = int(line.split(":", 1)[1].strip())
    (dst_dataset / "data.yaml").write_text(
        "\n".join(
            [
                f"path: {dst_dataset.as_posix()}",
                "train: images/train",
                "val: images/val",
                f"nc: {nc}",
                f"names: {names}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src", type=Path, required=True, help="Source YOLO dataset root.")
    parser.add_argument("--dst", type=Path, required=True, help="Output augmented YOLO dataset root.")
    parser.add_argument("--train-variants", type=int, default=4)
    parser.add_argument("--val-variants", type=int, default=1)
    parser.add_argument("--seed", type=int, default=78)
    args = parser.parse_args()

    args.dst.mkdir(parents=True, exist_ok=True)
    augment_split(args.src, args.dst, "train", args.train_variants, args.seed)
    augment_split(args.src, args.dst, "val", args.val_variants, args.seed)
    write_data_yaml(args.src, args.dst)
    print(f"wrote: {args.dst / 'data.yaml'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
