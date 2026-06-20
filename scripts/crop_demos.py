#!/usr/bin/env python
"""Crop + resize a demo dataset's camera videos to 128x128 for SAC seeding -- fast.

The policy sees the wrist camera at 128x128, so the offline demos must match. Rather
than recreate the dataset frame-by-frame through PIL (slow, and Pillow 12.x has a
PNG-save bug), this transcodes the mp4s directly with ffmpeg (crop+scale filter,
all-intra h264 for exact frame seeking) and reuses the existing parquet/meta -- only
the video resolution and the image feature shape change. Minutes, not an hour.

A wrist cam is already framed on the workspace, so ``--no-crop`` (resize only) is the
usual choice. Drop it to drag an ROI box per camera (cv2 window); the chosen params
are printed + saved -- put them in the SAC config's
``env.processor.image_preprocessing.crop_params_dict`` so the LIVE camera matches.

    python scripts/crop_demos.py \
        --root ~/2026-06-19_pet_alu_wrist_only_eedelta_labeled \
        --new-root ~/2026-06-19_pet_alu_wrist_only_eedelta_labeled_crop128 --no-crop
"""

import argparse
import json
import os
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import piper_teleop.lerobot_plugin  # noqa: F401
from piper_teleop.lerobot_plugin.patches import apply_lerobot_patches

apply_lerobot_patches()  # pyav backend so the demo videos decode (for ROI preview)


def find_ffmpeg() -> str:
    """First ffmpeg with libx264 (the conda env's stripped build lacks it)."""
    candidates = [
        os.environ.get("FFMPEG"),
        shutil.which("ffmpeg"),
        str(Path.home() / "miniconda3" / "bin" / "ffmpeg"),
        "/usr/bin/ffmpeg",
    ]
    for c in candidates:
        if c and Path(c).exists():
            out = subprocess.run([c, "-hide_banner", "-encoders"], capture_output=True, text=True).stdout
            if "libx264" in out:
                return c
    raise SystemExit("No ffmpeg with libx264 found. Set FFMPEG=/path/to/ffmpeg")


FFMPEG = find_ffmpeg()


def ffmpeg_transcode(src: Path, dst: Path, roi, resize):
    """crop (top,left,h,w) -> scale (h,w) -> all-intra h264."""
    top, left, h, w = roi
    vf = f"crop={w}:{h}:{left}:{top},scale={resize[1]}:{resize[0]}"
    dst.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [FFMPEG, "-y", "-loglevel", "error", "-i", str(src.resolve()),
         "-vf", vf, "-c:v", "libx264", "-pix_fmt", "yuv420p", "-g", "1", str(dst)],
        check=True,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--new-root", required=True)
    ap.add_argument("--no-crop", action="store_true", help="resize only, no ROI box")
    ap.add_argument("--crop-params-path", default=None, help="JSON of ROIs (skip interactive)")
    ap.add_argument("--resize", type=int, nargs=2, default=[128, 128], help="H W")
    ap.add_argument("--workers", type=int, default=3)
    args = ap.parse_args()

    src, dst = Path(args.root), Path(args.new_root)
    if dst.exists():
        raise SystemExit(f"{dst} exists -- remove it first")

    info = json.loads((src / "meta" / "info.json").read_text())
    video_keys = [k for k, f in info["features"].items() if f.get("dtype") == "video"]
    print("video keys:", video_keys)

    # --- choose ROI per camera ---
    rois = {}
    if args.crop_params_path:
        rois = {k: tuple(v) for k, v in json.loads(Path(args.crop_params_path).read_text()).items()}
    for key in video_keys:
        if key in rois:
            continue
        H = info["features"][key]["info"]["video.height"]
        W = info["features"][key]["info"]["video.width"]
        if args.no_crop:
            rois[key] = (0, 0, H, W)
        else:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            from lerobot.rl import crop_dataset_roi as crd
            ds = LeRobotDataset("local/src", root=str(src))
            img = ds[0][key].cpu().permute(1, 2, 0).numpy()
            img = (img * 255).astype("uint8")
            rois[key] = tuple(crd.select_square_roi_for_images({key: img})[key])

    print("ROIs (top, left, height, width):", {k: list(v) for k, v in rois.items()})

    # --- copy meta + data (small); transcode videos ---
    (dst / "data").mkdir(parents=True)
    shutil.copytree(src / "meta", dst / "meta")
    for p in (src / "data").rglob("*.parquet"):
        out = dst / "data" / p.relative_to(src / "data")
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, out)

    jobs = []
    for key in video_keys:
        for v in sorted((src / "videos" / key).rglob("*.mp4")):
            out = dst / "videos" / key / v.relative_to(src / "videos" / key)
            jobs.append((v, out, rois[key]))
    print(f"transcoding {len(jobs)} video files ({args.workers} parallel) ...")
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        list(ex.map(lambda j: ffmpeg_transcode(j[0], j[1], j[2], args.resize), jobs))

    # --- update info.json: resized video features ---
    rh, rw = args.resize
    for key in video_keys:
        f = info["features"][key]
        f["shape"] = [rh, rw, f["shape"][2]]
        f["info"].update({"video.height": rh, "video.width": rw, "video.codec": "h264"})
    (dst / "meta" / "info.json").write_text(json.dumps(info, indent=4))
    Path(str(dst) + ".crop_params.json").write_text(json.dumps({k: list(v) for k, v in rois.items()}, indent=2))

    print(f"\nWrote {dst}  ({len(jobs)} videos -> {rh}x{rw}).")
    if not args.no_crop:
        print("Put the ROIs above into config/sac_piper.json "
              "env.processor.image_preprocessing.crop_params_dict so live frames match.")


if __name__ == "__main__":
    main()
