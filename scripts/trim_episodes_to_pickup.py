#!/usr/bin/env python3
"""
trim_episodes_to_pickup.py
==========================

Post-process a LeRobot v3 dataset by trimming each episode down to the
"pickup window": [t_close - PRE_SECONDS, t_close + POST_SECONDS], where
t_close is the first frame at which the gripper command transitions
into the closed state.

Strategy: rather than surgically editing parquet/video files, build a
fresh v3 dataset using LeRobot's own LeRobotDataset.create + add_frame
+ save_episode + finalize pipeline. The original dataset is untouched.

Usage
-----
# 1) Inspect ONE episode first, dial in the gripper threshold:
python trim_episodes_to_pickup.py inspect \
    --src-repo-id bumblebee/piper_pickup_raw \
    --src-root /home/ethrc/datasets \
    --episode 0

# 2) Once you trust the detection, trim everything:
python trim_episodes_to_pickup.py trim \
    --src-repo-id bumblebee/piper_pickup_raw \
    --src-root /home/ethrc/datasets \
    --dst-repo-id bumblebee/piper_pickup_trimmed \
    --dst-root /home/ethrc/datasets \
    --pre 3.0 --post 1.0 \
    --gripper-idx -1 --closed-threshold 0.5

# 3) Sanity-check a trimmed episode in rerun:
lerobot-dataset-viz --repo-id bumblebee/piper_pickup_trimmed \
    --root /home/ethrc/datasets --mode local --episode-index 0
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import numpy as np
import torch

from lerobot.datasets import LeRobotDataset

logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

def find_gripper_close_idx(
    gripper_signal: np.ndarray,
    threshold: float,
    closed_below: bool = True,
) -> int | None:
    """First frame index where the gripper transitions from open to closed.

    We look for an edge (False -> True in `closed_mask`), not just any
    closed frame. This avoids being fooled by an episode that starts
    with the gripper already closed.

    Parameters
    ----------
    gripper_signal : 1-D array of gripper command values across the episode.
    threshold      : value separating open from closed.
    closed_below   : if True, values below `threshold` mean closed (typical
                     when 0 = closed, large = open). Flip if reversed.

    Returns
    -------
    Frame index of first close edge, or None if no edge found.
    """
    closed = gripper_signal < threshold if closed_below else gripper_signal > threshold
    edges = np.where((~closed[:-1]) & (closed[1:]))[0]
    if len(edges) == 0:
        return None
    return int(edges[0]) + 1  # +1 so idx points at the first closed frame


# ---------------------------------------------------------------------------
# Episode I/O helpers
# ---------------------------------------------------------------------------

def episode_frame_range(src: LeRobotDataset, ep_idx: int) -> tuple[int, int]:
    """Return (from, to) global frame indices for an episode.

    Tries `episode_data_index` first (legacy), falls back to deriving from
    `meta.episodes` lengths (v3). Verify on your installed lerobot version.
    """
    if hasattr(src, "episode_data_index") and src.episode_data_index is not None:
        return (
            int(src.episode_data_index["from"][ep_idx]),
            int(src.episode_data_index["to"][ep_idx]),
        )
    # v3 fallback: cumulative sum of per-episode lengths
    lengths = [int(src.meta.episodes[i]["length"]) for i in range(ep_idx)]
    start = sum(lengths)
    return start, start + int(src.meta.episodes[ep_idx]["length"])


def episode_gripper_signal(src: LeRobotDataset, ep_idx: int, gripper_idx: int) -> np.ndarray:
    """Return the 1-D gripper-command time series for one episode."""
    start, end = episode_frame_range(src, ep_idx)
    out = np.empty(end - start, dtype=np.float32)
    for k, i in enumerate(range(start, end)):
        a = src[i]["action"]
        if isinstance(a, torch.Tensor):
            a = a.cpu().numpy()
        out[k] = float(a[gripper_idx])
    return out


def to_addframe_value(v, feature_info: dict):
    """Convert a sample value to the format add_frame expects.

    Tensors -> numpy. CHW float images -> HWC uint8. Verify on your version.
    """
    if isinstance(v, torch.Tensor):
        v = v.cpu().numpy()
    if isinstance(feature_info, dict) and feature_info.get("dtype") in ("image", "video"):
        if isinstance(v, np.ndarray) and v.ndim == 3 and v.shape[0] in (1, 3) \
                and v.dtype != np.uint8:
            v = (v.transpose(1, 2, 0) * 255).clip(0, 255).astype(np.uint8)
    return v


def resolve_gripper_idx(src: LeRobotDataset, requested: int) -> int:
    action_dim = int(src.features["action"]["shape"][0])
    return requested if requested >= 0 else action_dim - 1


# ---------------------------------------------------------------------------
# Inspect
# ---------------------------------------------------------------------------

def inspect_episode(args) -> None:
    src = LeRobotDataset(args.src_repo_id, root=args.src_root)
    grip_idx = resolve_gripper_idx(src, args.gripper_idx)

    log.info(
        "Dataset: %d episodes, fps=%.1f, action_dim=%d, gripper_idx=%d",
        src.num_episodes, src.fps, int(src.features["action"]["shape"][0]), grip_idx,
    )

    g = episode_gripper_signal(src, args.episode, grip_idx)
    log.info("Episode %d: %d frames (%.2fs)", args.episode, len(g), len(g) / src.fps)
    log.info(
        "  action[%d] stats: min=%.3f max=%.3f mean=%.3f std=%.3f",
        grip_idx, g.min(), g.max(), g.mean(), g.std(),
    )
    log.info("  first 5: %s", np.round(g[:5], 3))
    log.info("  last  5: %s", np.round(g[-5:], 3))

    idx = find_gripper_close_idx(g, args.closed_threshold, closed_below=not args.closed_above)
    if idx is None:
        log.warning(
            "  No close edge detected with threshold=%.3f, closed_%s.",
            args.closed_threshold, "above" if args.closed_above else "below",
        )
        return

    keep_start = max(0, idx - int(round(args.pre * src.fps)))
    keep_end = min(len(g), idx + int(round(args.post * src.fps)) + 1)
    log.info("  Detected close edge at frame %d (t = %.2fs)", idx, idx / src.fps)
    log.info(
        "  Would keep frames [%d, %d) -- %d frames, %.2fs",
        keep_start, keep_end, keep_end - keep_start, (keep_end - keep_start) / src.fps,
    )


# ---------------------------------------------------------------------------
# Trim
# ---------------------------------------------------------------------------

def trim_dataset(args) -> None:
    src = LeRobotDataset(args.src_repo_id, root=args.src_root)
    grip_idx = resolve_gripper_idx(src, args.gripper_idx)
    log.info(
        "Source: %d episodes, fps=%.1f, gripper_idx=%d",
        src.num_episodes, src.fps, grip_idx,
    )

    use_videos = any(f.get("dtype") == "video" for f in src.features.values())
    dst = LeRobotDataset.create(
        repo_id=args.dst_repo_id,
        fps=src.fps,
        root=args.dst_root,
        features=src.features,
        use_videos=use_videos,
    )

    pre_frames = int(round(args.pre * src.fps))
    post_frames = int(round(args.post * src.fps))

    target_episodes = args.episodes if args.episodes else list(range(src.num_episodes))
    kept = skipped = 0

    # Bookkeeping fields the destination dataset will recompute itself.
    # subtask_index / subtask are also skipped: propagating them requires
    # registering subtasks at create() time, which is out of scope here.
    skip_keys = {
        "episode_index", "frame_index", "index", "timestamp",
        "task_index", "task", "subtask_index", "subtask",
    }

    for ep_idx in target_episodes:
        g = episode_gripper_signal(src, ep_idx, grip_idx)
        close_idx = find_gripper_close_idx(
            g, args.closed_threshold, closed_below=not args.closed_above
        )
        if close_idx is None:
            log.warning("Ep %3d: no close edge -- SKIPPING", ep_idx)
            skipped += 1
            continue

        ep_from, ep_to = episode_frame_range(src, ep_idx)
        n = ep_to - ep_from
        keep_start = max(0, close_idx - pre_frames)
        keep_end = min(n, close_idx + post_frames + 1)
        log.info(
            "Ep %3d: %d frames -> keep [%d, %d) (%d frames, %.2fs)",
            ep_idx, n, keep_start, keep_end,
            keep_end - keep_start, (keep_end - keep_start) / src.fps,
        )

        # Per-episode task string (v3 stores per-frame, but typically uniform per episode).
        ep_meta = src.meta.episodes[ep_idx]
        tasks = ep_meta.get("tasks") if isinstance(ep_meta, dict) else None
        task = tasks[0] if tasks else ""

        for local_i in range(keep_start, keep_end):
            sample = src[ep_from + local_i]
            frame = {}
            for k, finfo in src.features.items():
                if k in skip_keys:
                    continue
                if k not in sample:
                    continue
                frame[k] = to_addframe_value(sample[k], finfo)
            frame["task"] = task; dst.add_frame(frame)

        dst.save_episode()
        kept += 1

    dst.finalize()
    log.info(
        "Done. Kept %d episodes, skipped %d. Output at %s",
        kept, skipped, args.dst_root,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--src-repo-id", required=True)
    common.add_argument("--src-root", type=Path, default=None)
    common.add_argument(
        "--gripper-idx", type=int, default=-1,
        help="Index of gripper in the action vector. -1 = last (default).",
    )
    common.add_argument(
        "--closed-threshold", type=float, default=0.5,
        help="Threshold separating open from closed (default 0.5).",
    )
    common.add_argument(
        "--closed-above", action="store_true",
        help="Set if values ABOVE threshold mean closed (default: below = closed).",
    )
    common.add_argument(
        "--pre", type=float, default=3.0,
        help="Seconds to keep before the close event (default 3.0).",
    )
    common.add_argument(
        "--post", type=float, default=1.0,
        help="Seconds to keep after the close event (default 1.0).",
    )

    p_ins = sub.add_parser("inspect", parents=[common], help="Inspect one episode's gripper signal.")
    p_ins.add_argument("--episode", type=int, default=0)
    p_ins.set_defaults(func=inspect_episode)

    p_trim = sub.add_parser("trim", parents=[common], help="Trim episodes into a new dataset.")
    p_trim.add_argument("--dst-repo-id", required=True)
    p_trim.add_argument("--dst-root", type=Path, required=True)
    p_trim.add_argument(
        "--episodes", type=int, nargs="*", default=None,
        help="If set, only process these episode indices.",
    )
    p_trim.set_defaults(func=trim_dataset)

    return p


if __name__ == "__main__":
    args = build_parser().parse_args()
    args.func(args)


