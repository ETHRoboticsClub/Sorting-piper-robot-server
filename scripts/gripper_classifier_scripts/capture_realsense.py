#!/usr/bin/env python3
"""
Capture color frames from an Intel RealSense camera and save as PNG/JPG.
Use for building a YOLO *classification* dataset (one image file per sample).

Examples:
  python capture_realsense.py -o ./raw --interactive   # you press Space to save each photo
  python capture_realsense.py -o ./raw --once           # one immediate grab, no window
  python capture_realsense.py -o ./raw -n 20 --interval 0.5
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Capture images from Intel RealSense (color stream).")
    p.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path("captured"),
        help="Directory to write images (created if missing).",
    )
    p.add_argument(
        "--prefix",
        default="img",
        help="Filename prefix (default: img).",
    )
    p.add_argument(
        "--ext",
        choices=("png", "jpg"),
        default="png",
        help="Image format (default: png).",
    )
    p.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help="Live preview: press Space or s to save a frame, q or Esc to quit.",
    )
    p.add_argument(
        "--once",
        action="store_true",
        help="Capture a single frame and exit (no window). Ignored if --interactive.",
    )
    p.add_argument(
        "-n",
        "--count",
        type=int,
        default=1,
        help="Number of frames to capture (default: 1). Ignored if --once.",
    )
    p.add_argument(
        "--interval",
        type=float,
        default=0.0,
        help="Seconds to wait between captures when count > 1.",
    )
    p.add_argument(
        "--width",
        type=int,
        default=640,
        help="Color stream width (default: 640).",
    )
    p.add_argument(
        "--height",
        type=int,
        default=480,
        help="Color stream height (default: 480).",
    )
    p.add_argument(
        "--fps",
        type=int,
        default=30,
        help="FPS hint for color stream (default: 30).",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("Install pyrealsense2: pip install pyrealsense2", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    pipeline.start(cfg)
    try:
        # Warm up: discard a few frames so exposure stabilizes
        for _ in range(5):
            pipeline.wait_for_frames()

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), 95]

        def save_frame(bgr: np.ndarray, index: int) -> bool:
            suffix = f"{args.prefix}_{stamp}_{index:04d}.{args.ext}"
            out_path = args.output_dir / suffix
            if args.ext == "png":
                ok = cv2.imwrite(str(out_path), bgr)
            else:
                ok = cv2.imwrite(str(out_path), bgr, jpeg_params)
            if ok:
                print(out_path.resolve())
            else:
                print(f"Failed to write {out_path}", file=sys.stderr)
            return bool(ok)

        if args.interactive:
            saved = 0
            win = "realsense_capture"
            print(
                "Interactive mode: live preview keeps running after each save; only q/Esc stops the camera."
            )
            while True:
                frames = pipeline.wait_for_frames()
                color = frames.get_color_frame()
                if not color:
                    continue
                bgr = np.asanyarray(color.get_data())
                vis = bgr.copy()
                hint = f"saved {saved}  |  Space/s: save frame  |  q: stop camera"
                cv2.putText(vis, hint, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(win, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord(" "), ord("s")):
                    if save_frame(bgr, saved):
                        saved += 1
                elif key in (ord("q"), 27):
                    break
            cv2.destroyWindow(win)
            return 0

        n = 1 if args.once else max(1, args.count)
        for i in range(n):
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                print("No color frame; retrying.", file=sys.stderr)
                continue
            bgr = np.asanyarray(color.get_data())

            if not save_frame(bgr, i):
                return 1

            if i < n - 1 and args.interval > 0:
                time.sleep(args.interval)
    finally:
        pipeline.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
