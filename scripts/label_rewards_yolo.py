#!/usr/bin/env python
"""Offline YOLO grasp-reward labeling + post-grasp trimming for the demo dataset.

Replays the live ``GripperHoldYoloGraspRewardStep`` logic over each episode of an
EE-delta dataset: once the gripper has been *commanded* closed (action[-1]==0)
for ``hold_seconds``, classify the wrist frame. PET/Aluminium (conf >= threshold)
=> success: ``next.reward = 1``, ``next.done = True`` at that frame, and the
episode is **trimmed** there (the post-grasp lift / return-to-home is dropped).
Empty/Other (or no successful grasp) => the episode is kept in full with
``next.reward = 0`` (a negative example), ``next.done`` on its last frame.

Trimming is index-only: rows after the success frame are dropped from the data
parquet and each episode's ``length`` / video ``to_timestamp`` are shrunk, but the
mp4s are left untouched (videos are concatenated across episodes, so later
episodes' offsets are unaffected -- no re-encode). ``videos/`` is symlinked to the
source by default (identical bytes); pass ``--copy-videos`` for a standalone copy.

Usage:
  python scripts/label_rewards_yolo.py SRC DST [--dry-run] [--limit N] \
      [--weights W] [--hold-seconds 1.0] [--confidence 0.5] [--copy-videos]
"""

import argparse
import glob
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

import piper_teleop.lerobot_plugin  # noqa: F401  (registers plugin)
from piper_teleop.lerobot_plugin.patches import apply_lerobot_patches

DEFAULT_WEIGHTS = "cls_train_val_210526_nativeish2/weights/best.pt"
RECOMPUTE_FEATS = [
    "action", "observation.state", "next.reward", "next.done",
    "timestamp", "frame_index", "episode_index", "index", "task_index",
]


def stat_dict(arr):
    """Per-dimension stats matching the LeRobot v3 schema (arr: (N,) or (N, D))."""
    arr = np.asarray(arr, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr[:, None]
    return {
        "min": arr.min(0).tolist(),
        "max": arr.max(0).tolist(),
        "mean": arr.mean(0).tolist(),
        "std": arr.std(0).tolist(),
        "count": [int(len(arr))],
        "q01": np.quantile(arr, 0.01, axis=0).tolist(),
        "q10": np.quantile(arr, 0.10, axis=0).tolist(),
        "q50": np.quantile(arr, 0.50, axis=0).tolist(),
        "q90": np.quantile(arr, 0.90, axis=0).tolist(),
        "q99": np.quantile(arr, 0.99, axis=0).tolist(),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--weights", default=DEFAULT_WEIGHTS)
    ap.add_argument("--hold-seconds", type=float, default=1.0)
    ap.add_argument("--confidence", type=float, default=0.5)
    ap.add_argument("--success-classes", nargs="+", default=["PET", "Aluminium"])
    ap.add_argument("--camera-key", default="wrist")
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--copy-videos", action="store_true")
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    apply_lerobot_patches()  # pyav backend
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import lerobot.rl.gym_manipulator as gm

    src = args.src
    src_ds = LeRobotDataset("local/src", root=str(src))
    df = pd.read_parquet(glob.glob(str(src / "data" / "**" / "*.parquet"), recursive=True)[0])
    info = json.loads((src / "meta" / "info.json").read_text())
    fps = info["fps"]
    hold_frames = int(round(args.hold_seconds * fps))
    succ = tuple(args.success_classes)
    step = gm.GripperHoldYoloGraspRewardStep(
        weights_path=args.weights, device=args.device, success_classes=succ,
        confidence_threshold=args.confidence, camera_key=args.camera_key,
    )

    episodes = sorted(df["episode_index"].unique())
    if args.limit is not None:
        episodes = episodes[: args.limit]

    keep_pos, rewards, dones = [], [], []   # accumulated across episodes (for the new df)
    ep_records = {}                          # ep -> dict(orig_len, kept_len, outcome, cls, conf)
    n_succ = n_fail = 0

    for ep in episodes:
        rows = df[df["episode_index"] == ep].sort_values("frame_index")
        pos = rows.index.to_numpy()          # positional rows in df
        gidx = rows["index"].to_numpy()      # global dataset index (for image decode)
        A = np.stack([np.asarray(a, float) for a in rows["action"]])
        closed = A[:, 6] == 0

        closed_since = None
        fired = False
        sframe = None
        cls = conf = None
        for t in range(len(A)):
            if not closed[t]:
                closed_since, fired = None, False
                continue
            if closed_since is None:
                closed_since = t
            if fired or (t - closed_since) < hold_frames:
                continue
            img = src_ds[int(gidx[t])]["observation.images.wrist1"]
            cls, conf = step._classify(img)
            fired = True
            if cls.lower() in (c.lower() for c in succ) and conf >= args.confidence:
                sframe = t
                break

        kept_len = (sframe + 1) if sframe is not None else len(A)
        reward = np.zeros(kept_len, dtype=np.float32)
        done = np.zeros(kept_len, dtype=bool)
        if sframe is not None:
            reward[sframe] = 1.0
            done[sframe] = True
            n_succ += 1
            outcome = "SUCCESS"
        else:
            done[-1] = True
            n_fail += 1
            outcome = "fail-kept"
        ep_records[int(ep)] = dict(orig=len(A), kept=kept_len, outcome=outcome,
                                   cls=cls, conf=conf)
        keep_pos.append(pos[:kept_len])
        rewards.append(reward)
        dones.append(done)

    print(f"\nlabeled {len(episodes)} episodes: success={n_succ} fail-kept={n_fail} "
          f"| frames {sum(r['orig'] for r in ep_records.values())} -> "
          f"{sum(r['kept'] for r in ep_records.values())}")

    if args.dry_run:
        for ep, r in list(ep_records.items())[:50]:
            tag = f"{r['cls']}({r['conf']:.2f})" if r["cls"] else "(no close-hold)"
            print(f"  ep{ep:4d}: {r['outcome']:9s} {r['orig']:3d}->{r['kept']:3d} | {tag}")
        return

    # ---- assemble the trimmed + labeled dataframe ----
    keep_pos = np.concatenate(keep_pos)
    new_df = df.loc[keep_pos].reset_index(drop=True).copy()
    new_df["next.reward"] = np.concatenate(rewards).astype(np.float32)
    new_df["next.done"] = np.concatenate(dones)
    new_df["index"] = np.arange(len(new_df), dtype=np.int64)

    # ---- write dataset dir (meta copied/edited, data rewritten, videos linked/copied) ----
    if args.dst.exists():
        raise SystemExit(f"refusing to overwrite existing {args.dst}")
    (args.dst / "data" / "chunk-000").mkdir(parents=True)
    shutil.copytree(src / "meta", args.dst / "meta")
    if args.copy_videos:
        shutil.copytree(src / "videos", args.dst / "videos")
    else:
        (args.dst / "videos").symlink_to((src / "videos").resolve())
    new_df.to_parquet(args.dst / info["data_path"].format(chunk_index=0, file_index=0), index=False)

    # info.json: add reward/done features + new totals
    info["features"]["next.reward"] = {"dtype": "float32", "shape": [1], "names": None}
    info["features"]["next.done"] = {"dtype": "bool", "shape": [1], "names": None}
    info["total_frames"] = int(len(new_df))
    info["total_episodes"] = int(new_df["episode_index"].nunique())
    info["splits"] = {"train": f"0:{info['total_episodes']}"}
    (args.dst / "meta" / "info.json").write_text(json.dumps(info, indent=4))

    # global stats.json
    stats = json.loads((src / "meta" / "stats.json").read_text())
    for feat in RECOMPUTE_FEATS:
        col = new_df[feat]
        arr = np.stack([np.asarray(x) for x in col]) if col.iloc[0].__class__.__name__ == "ndarray" \
            else col.to_numpy(dtype=np.float64)
        stats[feat] = stat_dict(arr)
    (args.dst / "meta" / "stats.json").write_text(json.dumps(stats, indent=4))

    # per-episode meta: lengths, global row ranges, video to_timestamps, stats
    edf = pd.read_parquet(glob.glob(str(args.dst / "meta" / "episodes" / "**" / "*.parquet"),
                                    recursive=True)[0])
    edf = edf[edf["episode_index"].isin(episodes)].reset_index(drop=True)
    # Update only the indexing fields (lengths, global row ranges, video time
    # ranges). Per-episode stats are left as-is -- global stats.json drives
    # normalization, and the trimmed-episode stats are close enough; rewriting the
    # nested stat array cells is brittle and unnecessary.
    from_idx = 0
    ts_cols = [c for c in edf.columns if c.endswith("/to_timestamp")]
    lengths, froms, tos = [], [], []
    new_ts = {tc: [] for tc in ts_cols}
    for i in range(len(edf)):
        ep = int(edf.at[i, "episode_index"])
        kept = ep_records[ep]["kept"]
        lengths.append(kept)
        froms.append(from_idx)
        tos.append(from_idx + kept)
        from_idx += kept
        for tc in ts_cols:
            fc = tc.replace("/to_timestamp", "/from_timestamp")
            new_ts[tc].append(float(edf.at[i, fc]) + kept / fps)
    edf["length"] = lengths
    edf["dataset_from_index"] = froms
    edf["dataset_to_index"] = tos
    for tc in ts_cols:
        edf[tc] = new_ts[tc]
    edf.to_parquet(glob.glob(str(args.dst / "meta" / "episodes" / "**" / "*.parquet"),
                             recursive=True)[0], index=False)

    print(f"Wrote {args.dst} ({info['total_episodes']} eps, {info['total_frames']} frames, "
          f"videos {'copied' if args.copy_videos else 'symlinked'}).")


if __name__ == "__main__":
    main()
