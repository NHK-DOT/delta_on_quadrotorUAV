from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


FAMILY_TO_DICT = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm / 25.4 * dpi))


def parse_ids(text: str) -> list[int]:
    result: list[int] = []
    for chunk in text.split(","):
        part = chunk.strip()
        if not part:
            continue
        if "-" in part:
            start_text, end_text = part.split("-", 1)
            start = int(start_text.strip())
            end = int(end_text.strip())
            if end < start:
                raise ValueError(f"Invalid id range: {part}")
            result.extend(range(start, end + 1))
        else:
            result.append(int(part))

    deduped: list[int] = []
    seen: set[int] = set()
    for tag_id in result:
        if tag_id not in seen:
            deduped.append(tag_id)
            seen.add(tag_id)
    return deduped


def build_single_tag_canvas(
    dictionary: cv2.aruco.Dictionary,
    family: str,
    tag_id: int,
    marker_size_px: int,
    margin_px: int,
    label_height_px: int,
) -> np.ndarray:
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_size_px)
    canvas_h = marker_size_px + (2 * margin_px) + label_height_px
    canvas_w = marker_size_px + (2 * margin_px)
    canvas = np.full((canvas_h, canvas_w), 255, dtype=np.uint8)
    canvas[margin_px : margin_px + marker_size_px, margin_px : margin_px + marker_size_px] = marker

    label = f"ID {tag_id:02d}"
    font_scale = max(0.7, label_height_px / 95.0)
    thickness = max(1, label_height_px // 36)
    text_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
    text_x = max(10, (canvas_w - text_size[0]) // 2)
    text_y = canvas_h - max(10, (label_height_px - text_size[1]) // 2) - baseline
    cv2.putText(
        canvas,
        label,
        (text_x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        0,
        thickness,
        cv2.LINE_AA,
    )

    family_scale = max(0.45, label_height_px / 140.0)
    family_thickness = max(1, thickness - 1)
    family_size, _ = cv2.getTextSize(family, cv2.FONT_HERSHEY_SIMPLEX, family_scale, family_thickness)
    family_x = max(10, (canvas_w - family_size[0]) // 2)
    family_y = margin_px - 12 if margin_px > 26 else margin_px + 22
    cv2.putText(
        canvas,
        family,
        (family_x, family_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        family_scale,
        0,
        family_thickness,
        cv2.LINE_AA,
    )
    return canvas


def save_with_dpi(image: np.ndarray, path: Path, dpi: int) -> None:
    Image.fromarray(image).save(path, dpi=(dpi, dpi))


def build_a4_pages(
    tag_images: list[tuple[int, np.ndarray]],
    dpi: int,
    page_margin_mm: float,
    title_height_mm: float,
) -> list[np.ndarray]:
    a4_w_px = mm_to_px(210.0, dpi)
    a4_h_px = mm_to_px(297.0, dpi)
    margin_px = mm_to_px(page_margin_mm, dpi)
    title_h_px = mm_to_px(title_height_mm, dpi)

    if not tag_images:
        return []

    cell_h, cell_w = tag_images[0][1].shape
    gap_px = mm_to_px(6.0, dpi)
    usable_w = a4_w_px - (2 * margin_px)
    usable_h = a4_h_px - (2 * margin_px) - title_h_px
    cols = max(1, usable_w // (cell_w + gap_px))
    rows = max(1, usable_h // (cell_h + gap_px))
    per_page = max(1, cols * rows)

    pages: list[np.ndarray] = []
    for page_index in range(math.ceil(len(tag_images) / per_page)):
        page = np.full((a4_h_px, a4_w_px), 255, dtype=np.uint8)
        cv2.putText(
            page,
            f"AprilTag Print Sheet  Page {page_index + 1}",
            (margin_px, margin_px + title_h_px - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            0,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            page,
            "Print with 100% scale and measure the black square edge size.",
            (margin_px, margin_px + title_h_px + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            0,
            1,
            cv2.LINE_AA,
        )

        start = page_index * per_page
        end = min(len(tag_images), start + per_page)
        current = tag_images[start:end]
        for offset, (_, tag_image) in enumerate(current):
            row = offset // cols
            col = offset % cols
            x = margin_px + col * (cell_w + gap_px)
            y = margin_px + title_h_px + row * (cell_h + gap_px)
            page[y : y + cell_h, x : x + cell_w] = tag_image

        pages.append(page)
    return pages


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Generate printable AprilTag images.")
    parser.add_argument("--family", choices=sorted(FAMILY_TO_DICT), default="tag36h11")
    parser.add_argument("--ids", default="0-11", help="e.g. 0-5,8,10-12")
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--marker-size-mm", type=float, default=80.0, help="Black square edge size for print")
    parser.add_argument("--margin-mm", type=float, default=10.0)
    parser.add_argument("--label-height-mm", type=float, default=12.0)
    parser.add_argument("--output-dir", type=Path, default=project_root / "image")
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    tag_ids = parse_ids(args.ids)
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILY_TO_DICT[args.family])
    marker_size_px = mm_to_px(args.marker_size_mm, args.dpi)
    margin_px = mm_to_px(args.margin_mm, args.dpi)
    label_height_px = mm_to_px(args.label_height_mm, args.dpi)

    generated: list[tuple[int, np.ndarray]] = []
    for tag_id in tag_ids:
        canvas = build_single_tag_canvas(
            dictionary=dictionary,
            family=args.family,
            tag_id=tag_id,
            marker_size_px=marker_size_px,
            margin_px=margin_px,
            label_height_px=label_height_px,
        )
        file_name = f"{args.family}_id_{tag_id:02d}_{int(round(args.marker_size_mm))}mm.png"
        save_with_dpi(canvas, output_dir / file_name, args.dpi)
        generated.append((tag_id, canvas))

    pages = build_a4_pages(
        tag_images=generated,
        dpi=args.dpi,
        page_margin_mm=10.0,
        title_height_mm=18.0,
    )
    for index, page in enumerate(pages, start=1):
        save_with_dpi(
            page,
            output_dir / f"{args.family}_sheet_page_{index:02d}_{int(round(args.marker_size_mm))}mm_A4.png",
            args.dpi,
        )

    print(f"Generated {len(generated)} tags in: {output_dir}")
    for tag_id, _ in generated:
        print(f"  - {args.family} id={tag_id:02d}")
    if pages:
        print(f"Generated {len(pages)} A4 print sheet(s).")


if __name__ == "__main__":
    main()
