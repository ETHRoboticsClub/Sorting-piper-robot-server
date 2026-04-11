#!/usr/bin/env python3
"""
Merge several YOLO-style classification dataset trees into one folder.

Each source should look like:
  <source>/train/<class_name>/*.jpg
  <source>/val/<class_name>/*.jpg   (optional)

Files are copied into --out (default). Name clashes get _1, _2, ... before the extension.

Example:
  python merge_classification_datasets.py -o ./dataset_all ./dataset_run1 ./dataset_run2
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def unique_dest(dest_dir: Path, filename: str) -> Path:
    target = dest_dir / filename
    if not target.exists():
        return target
    stem, suf = Path(filename).stem, Path(filename).suffix
    n = 1
    while True:
        cand = dest_dir / f"{stem}_{n}{suf}"
        if not cand.exists():
            return cand
        n += 1


def merge_one_file(src: Path, dest_dir: Path, copy: bool) -> Path:
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = unique_dest(dest_dir, src.name)
    if copy:
        shutil.copy2(src, dest)
    else:
        shutil.move(str(src), str(dest))
    return dest


def merge_source(out: Path, source: Path, copy: bool, dry_run: bool) -> int:
    n_files = 0
    for subset in ("train", "val"):
        sub = source / subset
        if not sub.is_dir():
            continue
        for class_dir in sorted(sub.iterdir()):
            if not class_dir.is_dir():
                continue
            class_name = class_dir.name
            for f in sorted(class_dir.iterdir()):
                if not f.is_file() or f.suffix.lower() not in IMAGE_EXTS:
                    continue
                dest_dir = out / subset / class_name
                if dry_run:
                    n_files += 1
                    continue
                merge_one_file(f, dest_dir, copy)
                n_files += 1
    return n_files


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "-o",
        "--out",
        type=Path,
        required=True,
        help="Output dataset root (train/ and val/ created under it).",
    )
    p.add_argument(
        "sources",
        nargs="+",
        type=Path,
        help="One or more dataset folders to merge.",
    )
    p.add_argument(
        "--move",
        action="store_true",
        help="Move files instead of copying (removes from sources).",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Count files only, do not copy or move.",
    )
    args = p.parse_args()

    copy = not args.move
    total = 0
    for src in args.sources:
        src = src.expanduser().resolve()
        if not src.is_dir():
            print(f"Skip (not a directory): {src}", file=sys.stderr)
            continue
        n = merge_source(args.out, src, copy, args.dry_run)
        print(f"{src}: {n} images")
        total += n

    if args.dry_run:
        print(f"Total: {total} images (dry run)")
    else:
        print(f"Done. Total merged: {total} -> {args.out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
