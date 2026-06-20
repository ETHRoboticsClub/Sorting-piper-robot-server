#!/usr/bin/env python
"""Convert a joint-space LeRobot dataset into our HIL-SERL EE-delta action space.

The old stack records ``action = absolute joint targets`` (``joint_0..joint_6``).
Our ``gym_manipulator`` HIL-SERL pipeline expects ``action = 6-DOF EE delta +
gripper`` (the normalized command our gamepad emits). This rewrites **only** the
``action`` column so the demos can seed the SAC replay buffer; observations,
images, timestamps and episode structure are copied unchanged.

Per frame ``t`` (aligned to the original ``action[t]``), forward-kinematics the
commanded joints through this repo's Arm_IK and express the increment as:
  - translation: ``(FK(cmd[t]) - FK(cmd[t-1])).xyz`` in the **world** frame,
    normalized by ``end_effector_step_sizes`` (0.008 m) and clipped to [-1, 1]
  - rotation: ``R(cmd[t-1])^T @ R(cmd[t])`` as a rotvec in the **EE** frame,
    normalized by ``rotation_scale`` (0.0873 rad) and clipped to [-1, 1]
  - gripper: 0 (close) if the commanded gripper < closed_threshold else 2 (open)
For ``t == 0`` the reference is the measured pose ``state[0]`` (the env latches the
target to the measured pose at episode start).

Global ``stats.json`` and per-episode ``meta/episodes`` action stats are
recomputed. Videos are copied as-is (no re-encode).

Usage:
    python scripts/convert_joint_demos_to_ee.py SRC DST [--limit N]
"""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from piper_teleop.lerobot_plugin.arm_ik_kinematics import ArmIkKinematics

EE_NAMES = ["delta_x", "delta_y", "delta_z", "delta_wx", "delta_wy", "delta_wz", "gripper"]
STEP = 0.008          # end_effector_step_sizes (m per unit), matches the config
ROT = 0.0873          # rotation_scale (rad per unit), matches MapDelta6Dof
CLOSED_M = 0.035      # gripper target below this (metres) -> "close" command


def fk_all(K, joints6):
    """FK a stack of 6-joint vectors -> (positions (N,3), rotations (N,3,3))."""
    pos = np.empty((len(joints6), 3))
    rot = np.empty((len(joints6), 3, 3))
    for i, q in enumerate(joints6):
        T = K.forward_kinematics(q)
        pos[i] = T[:3, 3]
        rot[i] = T[:3, :3]
    return pos, rot


def convert_actions(K, actions, states):
    """actions/states: (T,7) joint vectors for one episode -> (T,7) EE-delta actions."""
    pos_cmd, rot_cmd = fk_all(K, actions[:, :6])
    T0 = K.forward_kinematics(states[0, :6])  # measured reference for frame 0
    pos_ref0, rot_ref0 = T0[:3, 3], T0[:3, :3]

    out = np.zeros((len(actions), 7), dtype=np.float32)
    for t in range(len(actions)):
        ref_p = pos_ref0 if t == 0 else pos_cmd[t - 1]
        ref_R = rot_ref0 if t == 0 else rot_cmd[t - 1]
        dp = (pos_cmd[t] - ref_p) / STEP
        dr = Rotation.from_matrix(ref_R.T @ rot_cmd[t]).as_rotvec() / ROT
        grip = 0.0 if actions[t, 6] < CLOSED_M else 2.0
        out[t, :3] = np.clip(dp, -1.0, 1.0)
        out[t, 3:6] = np.clip(dr, -1.0, 1.0)
        out[t, 6] = grip
    return out


def stat_dict(arr):
    """Per-dimension stats matching the LeRobot v3 schema (arr: (N, D))."""
    arr = np.asarray(arr, dtype=np.float64)
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
    ap.add_argument("--limit", type=int, default=None, help="convert only the first N episodes (dry run)")
    args = ap.parse_args()

    src, dst = args.src, args.dst
    print(f"Loading Arm_IK ...")
    K = ArmIkKinematics()

    data_files = sorted((src / "data").rglob("*.parquet"))
    assert len(data_files) == 1, f"expected a single data parquet, got {len(data_files)}"
    df = pd.read_parquet(data_files[0])
    if args.limit is not None:
        keep = sorted(df["episode_index"].unique())[: args.limit]
        df = df[df["episode_index"].isin(keep)].reset_index(drop=True)
    print(f"{df['episode_index'].nunique()} episodes, {len(df)} frames")

    # Convert action per episode (episodes are contiguous in v3).
    new_action = np.zeros((len(df), 7), dtype=np.float32)
    per_ep_stats = {}
    for ep, idx in df.groupby("episode_index").groups.items():
        idx = np.array(sorted(idx))
        A = np.stack([np.asarray(x, float) for x in df["action"].iloc[idx]])
        S = np.stack([np.asarray(x, float) for x in df["observation.state"].iloc[idx]])
        ee = convert_actions(K, A, S)
        new_action[idx] = ee
        per_ep_stats[int(ep)] = stat_dict(ee)
    df["action"] = list(new_action)

    # --- write the duplicate dataset ---
    if dst.exists():
        raise SystemExit(f"refusing to overwrite existing {dst}")
    print(f"Copying dataset -> {dst} (videos copied as-is)")
    shutil.copytree(src, dst)
    df.to_parquet(sorted((dst / "data").rglob("*.parquet"))[0], index=False)

    # info.json: rename action feature dims
    info_p = dst / "meta" / "info.json"
    info = json.loads(info_p.read_text())
    info["features"]["action"]["names"] = EE_NAMES
    info_p.write_text(json.dumps(info, indent=4))

    # global stats.json: recompute action
    stats_p = dst / "meta" / "stats.json"
    stats = json.loads(stats_p.read_text())
    stats["action"] = stat_dict(new_action)
    stats_p.write_text(json.dumps(stats, indent=4))

    # per-episode meta: overwrite stats/action/* columns
    ep_files = sorted((dst / "meta" / "episodes").rglob("*.parquet"))
    edf = pd.read_parquet(ep_files[0])
    for field in ["min", "max", "mean", "std", "count", "q01", "q10", "q50", "q90", "q99"]:
        col = f"stats/action/{field}"
        if col in edf.columns:
            edf[col] = [np.asarray(per_ep_stats[int(e)][field], dtype=np.float32 if field != "count" else np.int64)
                        for e in edf["episode_index"]]
    edf.to_parquet(ep_files[0], index=False)

    print("Done.")
    print("converted action stats (global):")
    a = new_action
    print("  min ", np.round(a.min(0), 3))
    print("  max ", np.round(a.max(0), 3))
    print("  mean", np.round(a.mean(0), 3))
    print(f"  fraction of trans/rot dims saturated at +-1: {100*np.mean(np.abs(a[:, :6]) >= 1.0):.2f}%")
    print(f"  gripper values: {sorted(set(a[:, 6].tolist()))}")


if __name__ == "__main__":
    main()
