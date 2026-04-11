#!/usr/bin/env python3
"""
Assign class labels to images by moving them into class subfolders (YOLO-cls style).

Expected layout after labeling:
  <dest>/train/<class_name>/*.jpg
  (optional) <dest>/val/<class_name>/...

You can later point Ultralytics YOLO classification training at <dest>.

Examples:
  python label_classification.py -s ./raw -d ./dataset
  python label_classification.py -s ./raw -d ./dataset --split 0.2

Default keys (four classes, YOLO-style indices): 0 PET, 1 Aluminium, 2 Other, 3 Empty
"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# Folder order matches YOLO class id: 0→PET, 1→Aluminium, 2→Other, 3→Empty
DEFAULT_CLASSES_STR = "PET,Aluminium,Other,Empty"

# When len(classes)==4, keys 0–3 map to class indices 0..3
_FOUR_CLASS_KEYS = (ord("0"), ord("1"), ord("2"), ord("3"))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Label images for classification: move/copy into class folders under train/val."
    )
    p.add_argument(
        "-s",
        "--source",
        type=Path,
        required=True,
        help="Folder containing unlabeled images (flat).",
    )
    p.add_argument(
        "-d",
        "--dest",
        type=Path,
        required=True,
        help="Dataset root (will contain train/ and optionally val/).",
    )
    p.add_argument(
        "-c",
        "--classes",
        default=DEFAULT_CLASSES_STR,
        help=(
            "Comma-separated class names. With exactly four classes, keys 0–3 match YOLO ids "
            f"(default: {DEFAULT_CLASSES_STR!r}). Otherwise keys 1–9 in order."
        ),
    )
    p.add_argument(
        "--split",
        type=float,
        default=None,
        help="If set (e.g. 0.2), fraction of each class sent to val/ (rest to train/).",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=42,
        help="RNG seed for val split (default: 42).",
    )
    p.add_argument(
        "--copy",
        action="store_true",
        help="Copy instead of move files.",
    )
    p.add_argument(
        "--max-width",
        type=int,
        default=960,
        help="Resize preview for display only (default: 960).",
    )
    return p.parse_args()


def list_images(folder: Path) -> list[Path]:
    out: list[Path] = []
    for p in sorted(folder.iterdir()):
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS:
            out.append(p)
    return out


def place_file(
    src: Path,
    dest_root: Path,
    class_name: str,
    split: float | None,
    rng: random.Random,
    copy: bool,
) -> Path:
    split_train = split is None or rng.random() >= split
    subset = "train" if split_train else "val"
    target_dir = dest_root / subset / class_name
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / src.name
    if target.exists():
        stem, suf = src.stem, src.suffix
        n = 1
        while True:
            candidate = target_dir / f"{stem}_{n}{suf}"
            if not candidate.exists():
                target = candidate
                break
            n += 1
    if copy:
        shutil.copy2(src, target)
    else:
        shutil.move(str(src), str(target))
    return target


BAR_H = 52
WIN = "label"


def _key_to_class_index(key: int, classes: list[str]) -> int | None:
    """Return class index for a key, or None if key does not select a class."""
    n = len(classes)
    if n == 4:
        for idx, k in enumerate(_FOUR_CLASS_KEYS):
            if key == k:
                return idx
        return None
    if ord("1") <= key <= ord("9"):
        i = key - ord("1")
        if i < n:
            return i
    if key == ord("0") and n >= 10:
        return 9
    return None


def _bar_key_label(class_index: int, n_classes: int) -> str:
    if n_classes == 4:
        return str(class_index)  # 0, 1, 2, 3 — matches keyboard and YOLO ids
    return str(class_index + 1)


def _draw_label_ui(
    img_show: np.ndarray,
    idx: int,
    total: int,
    path: Path,
    classes: list[str],
) -> tuple[np.ndarray, int]:
    """Stack image + clickable class bar. Returns (bgr image, height of image area in pixels)."""
    h, w = img_show.shape[:2]
    n = len(classes)
    bar = np.zeros((BAR_H, w, 3), dtype=np.uint8)
    cw = max(1, w // n)
    for i, name in enumerate(classes):
        x0 = i * cw
        x1 = min(w - 1, x0 + cw - 1)
        shade = (50 + (i % 2) * 25, 55, 60 + i * 15)
        cv2.rectangle(bar, (x0, 0), (x1, BAR_H - 1), shade, -1)
        cv2.rectangle(bar, (x0, 0), (x1, BAR_H - 1), (200, 200, 200), 1)
        short = name if len(name) <= 18 else name[:15] + "..."
        key_lbl = _bar_key_label(i, n)
        cv2.putText(
            bar,
            f"{key_lbl}  {short}",
            (x0 + 6, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    top = img_show.copy()
    line = f"{idx + 1}/{total}  {path.name}"
    cv2.putText(top, line[:90], (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    hint = (
        "0=PET 1=Aluminium 2=Other 3=Empty | Space skip | q quit | click bar"
        if n == 4
        else "Keys: 1-9 class | Space skip | q quit | click bar"
    )
    cv2.putText(
        top,
        hint,
        (8, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    vis = np.vstack([top, bar])
    return vis, h


def _mouse_callback(event: int, x: int, y: int, flags: int, param: dict[str, Any]) -> None:
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    h_img = param["h_img"]
    w_vis = param["w_vis"]
    n_cls = param["n_cls"]
    if y < h_img or y >= h_img + BAR_H:
        return
    cw = max(1, w_vis // n_cls)
    ci = min(n_cls - 1, x // cw)
    param["choice"] = ci


def main() -> int:
    args = parse_args()
    classes = [c.strip() for c in args.classes.split(",") if c.strip()]
    if not classes:
        print("Provide at least one class in --classes.", file=sys.stderr)
        return 1

    src = args.source.expanduser().resolve()
    if not src.is_dir():
        print(
            f"Source folder does not exist: {args.source} (resolved: {src})",
            file=sys.stderr,
        )
        print(
            "Create it and add images, e.g.  mkdir -p raw  "
            "then capture with:  python capture_realsense.py -o ./raw --interactive",
            file=sys.stderr,
        )
        print("Run from the project folder so ./raw points to the right place.", file=sys.stderr)
        return 1

    images = list_images(src)
    if not images:
        print(f"No images in {src}", file=sys.stderr)
        return 1

    rng = random.Random(args.seed)

    print("Interactive labeling: each image opens in a window.")
    if len(classes) == 4:
        print("  Keys: 0=PET  1=Aluminium  2=Other  3=Empty  (same as YOLO class ids)  |  Space=skip  q=quit  |  or click bar.")
    else:
        print("  Keys: 1–9 = class order  |  Space=skip  q=quit  |  or click the bar.")
    print("Classes (folder names):", ", ".join(classes))

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    idx = 0
    while idx < len(images):
        path = images[idx]
        img = cv2.imread(str(path))
        if img is None:
            print(f"Unreadable: {path}", file=sys.stderr)
            idx += 1
            continue

        h, w = img.shape[:2]
        scale = min(1.0, args.max_width / float(w)) if w > 0 else 1.0
        if scale < 1.0:
            img_show = cv2.resize(img, (int(w * scale), int(h * scale)))
        else:
            img_show = img

        h_img = img_show.shape[0]
        w_vis = img_show.shape[1]
        ui_param: dict[str, Any] = {
            "choice": None,
            "h_img": h_img,
            "w_vis": w_vis,
            "n_cls": len(classes),
        }
        cv2.setMouseCallback(WIN, _mouse_callback, ui_param)

        class_index: int | None = None
        quit_app = False
        while True:
            vis, _ = _draw_label_ui(img_show, idx, len(images), path, classes)
            cv2.imshow(WIN, vis)
            key = cv2.waitKey(30) & 0xFF

            pick = ui_param["choice"]
            if pick is not None:
                class_index = pick
                ui_param["choice"] = None
                break

            if key == ord("q") or key == 27:
                quit_app = True
                break
            if key == ord(" "):
                class_index = None
                break
            ci = _key_to_class_index(key, classes)
            if ci is not None:
                class_index = ci
                break

        if quit_app:
            print("Stopped by user.")
            break

        if class_index is None:
            idx += 1
            continue

        cls_name = classes[class_index]
        dest_path = place_file(path, args.dest, cls_name, args.split, rng, args.copy)
        print(f"  -> {cls_name}: {dest_path}")
        idx += 1

    cv2.destroyWindow(WIN)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
