#!/usr/bin/env python3
"""
Train a YOLO *classification* model with Ultralytics.

Expects a dataset like:
  <data>/train/<class_name>/*.jpg
  <data>/val/<class_name>/*.jpg

Use split_train_val.py to build that layout from your labeled data.

Examples:
  source .venv/bin/activate
  pip install -r requirements-train.txt

  python train_yolo_cls.py --data ./dataset/train_val_gripper
  python train_yolo_cls.py --data ./dataset/train_val_gripper --model yolo11s-cls.pt --epochs 150 --batch 32
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _check_data(data: Path) -> str | None:
    data = data.resolve()
    tr, va = data / "train", data / "val"
    if not tr.is_dir():
        return f"Missing train folder: {tr}"
    if not va.is_dir():
        return f"Missing val folder: {va}"
    train_classes = [p for p in tr.iterdir() if p.is_dir()]
    val_classes = [p for p in va.iterdir() if p.is_dir()]
    if not train_classes:
        return f"No class subfolders under {tr}"
    if not val_classes:
        return f"No class subfolders under {va}"
    return None


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--data",
        type=Path,
        required=True,
        help="Dataset root containing train/ and val/ (each with class subfolders).",
    )
    p.add_argument(
        "--model",
        default="yolov8n-cls.pt",
        help="Starting weights (downloads if needed), e.g. yolov8n-cls.pt, yolo11n-cls.pt.",
    )
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=224, help="Classifier input size (default: 224).")
    p.add_argument("--batch", type=int, default=16)
    p.add_argument(
        "--device",
        default="",
        help="cuda, cuda:0, cpu, or omit for auto.",
    )
    p.add_argument("--workers", type=int, default=8, help="DataLoader workers.")
    p.add_argument("--project", default="runs/classify", help="Ultralytics project dir.")
    p.add_argument("--name", default="train", help="Run name under project.")
    p.add_argument(
        "--patience",
        type=int,
        default=50,
        help="Early stopping patience (epochs without improvement).",
    )
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    err = _check_data(args.data)
    if err:
        print(err, file=sys.stderr)
        return 1

    try:
        from ultralytics import YOLO
    except ImportError:
        print(
            "Install Ultralytics:  pip install -r requirements-train.txt",
            file=sys.stderr,
        )
        return 1

    model = YOLO(args.model)
    model.train(
        data=str(args.data.resolve()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device if args.device else None,
        workers=args.workers,
        project=args.project,
        name=args.name,
        patience=args.patience,
        seed=args.seed,
    )
    print(f"Finished. Weights and logs under {args.project}/{args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
