#!/usr/bin/env python3
"""
Copy a classification dataset and split each class into train/ and val/.

Reads class folders from either:
  - <source>/train/<class_name>/   (--src is the dataset root), or
  - <source>/<class_name>/        (--src is already the train folder, e.g. dataset/train)

Writes copies to <dest>/train/<class_name>/ and <dest>/val/<class_name>/.

Examples:
  python split_train_val.py --src ./dataset --dst ./dataset_split --val-fraction 0.2
  python split_train_val.py --src ./dataset/train --dst ./dataset_split --val-fraction 0.2
"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
from pathlib import Path

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def list_images(class_dir: Path) -> list[Path]:
    out: list[Path] = []
    for p in sorted(class_dir.iterdir()):
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS:
            out.append(p)
    return out


def resolve_train_class_root(src: Path) -> Path | None:
    """
    Return the directory that contains one subfolder per class (each with images).
    """
    src = src.expanduser().resolve()
    nested = src / "train"
    if nested.is_dir():
        return nested
    # Already pointing at .../train or any folder whose children are class dirs with images
    subs = [p for p in src.iterdir() if p.is_dir()]
    if subs and any(list_images(d) for d in subs):
        return src
    return None


def split_count(n: int, val_fraction: float) -> int:
    if n == 0:
        return 0
    n_val = int(round(n * val_fraction))
    if n_val >= n:
        n_val = n - 1
    if n_val < 0:
        n_val = 0
    # keep at least one training image when we have more than one file
    if n > 1 and n_val == n:
        n_val = n - 1
    return n_val


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--src",
        type=Path,
        required=True,
        help="Dataset root (with train/<class>/...) or path to train/ itself (with <class>/...).",
    )
    p.add_argument(
        "--dst",
        type=Path,
        required=True,
        help="Destination root (created); train/ and val/ subfolders are filled.",
    )
    p.add_argument(
        "--val-fraction",
        type=float,
        default=0.2,
        help="Fraction of images per class that go to val (default: 0.2).",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for shuffling (default: 42).",
    )
    args = p.parse_args()

    if not (0.0 < args.val_fraction < 1.0):
        print("--val-fraction must be between 0 and 1 (exclusive).", file=sys.stderr)
        return 1

    src_train = resolve_train_class_root(args.src)
    if src_train is None:
        print(
            "Could not find class folders. Use --src <dataset_root> (with train/ inside) "
            "or --src <path/to/train> (folders PET, Empty, ...).",
            file=sys.stderr,
        )
        return 1

    rng = random.Random(args.seed)
    dst_root = args.dst.expanduser().resolve()
    dst_root.mkdir(parents=True, exist_ok=True)

    total_train = total_val = 0
    for class_dir in sorted(src_train.iterdir()):
        if not class_dir.is_dir():
            continue
        name = class_dir.name
        files = list_images(class_dir)
        if not files:
            continue

        rng.shuffle(files)
        n_val = split_count(len(files), args.val_fraction)
        val_files = files[:n_val]
        train_files = files[n_val:]

        for subset, subset_files in ("train", train_files), ("val", val_files):
            out_dir = dst_root / subset / name
            out_dir.mkdir(parents=True, exist_ok=True)
            for f in subset_files:
                shutil.copy2(f, out_dir / f.name)
            if subset == "train":
                total_train += len(subset_files)
            else:
                total_val += len(subset_files)

        print(f"{name}: {len(train_files)} train, {len(val_files)} val")

    print(f"Done -> {dst_root}  (train files: {total_train}, val files: {total_val})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
