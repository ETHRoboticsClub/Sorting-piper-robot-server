# Piper Robot Server

Teleoperation server for AgileX Piper arms with **keyboard**, **gamepad**, or learned-policy control, optional **cameras** and **LeRobot** recording. Runtime in this repo is **single-arm only** and uses `URDF/Piper/piper_description.urdf`.

**Optional setups:** trained policy (`--policy`) — **ACT**, **Diffusion**, **SmolVLA** (language-conditioned VLA), and **multi-task DiT** checkpoints are supported.

**Async deployment (`--async`):** run slow policies (Diffusion, multi-task DiT) through LeRobot's `policy_server` so inference happens *off* the control loop — the arm runs smoother and teleop/recording behave exactly as in sync mode. Example — **multi-task DiT, async**:

```bash
robotserver --show-cameras --policy --gamepad --async \
  --task aluminium \
  --policy-type multitask-Dit \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/current_models/pet_and_alu/multitask_Dit/09-06-26/150000/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/current_in_use/2026-06-08_PET_ALU_wrist_only
```

See [Async deployment](#async-deployment) for the flags and how it works. Drop `--async` to run synchronously (default).

## Prerequisites

| Requirement | Notes |
|-------------|--------|
| Hardware | One Piper **follower** + USB-CAN for real robot control (single-arm runtime). |
| Conda | Miniconda/Anaconda (Pinocchio is installed from conda-forge). |
| Python | `environment_piper_new.yml` creates the `piper_new` env (Python 3.12) — this is the current working environment for the lab. |

---
### Quickstart recording and deployment

Bring up the arm:

```bash
restart-can    # password: arc_user
conda activate piper_new
```

Record a session, specifying which task you are recording so the dataset gets the correct task sentence:

```bash
# Pick up PET bottle
robotserver --show-cameras --gamepad --record --task pet

# Pick up aluminium can
robotserver --show-cameras --gamepad --record --task aluminium

# Free-form task — you will be prompted for the description on stdin
robotserver --show-cameras --gamepad --record --task custom
```

Deploy a trained policy. **ACT**, **Diffusion Policy**, and **SmolVLA (VLA)** checkpoints are supported via `--policy-type`:

```bash
# ACT — current aluminium checkpoint
robotserver --show-cameras --policy --gamepad --record \
  --task aluminium \
  --policy-type act \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/alu/madi_plus_seb_wrist/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/madi_plus_aluminium_merged_wrist_only

# ACT — current PET checkpoint
robotserver --show-cameras --policy --gamepad --record \
  --task pet \
  --policy-type act \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/current_models/pet_and_alu/act/2026-06-08_pet_alu_wrist_act/checkpoints/200000 \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/current_in_use/2026-06-19_pet_alu_wrist_only

# Diffusion Policy — current PET checkpoint
robotserver --show-cameras --policy --gamepad --record \
  --task pet \
  --policy-type diffusion \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/current_models/PET/dp/200k/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/current_in_use/2026-05-28_PET_complete_v2_wrist

# SmolVLA — language-conditioned, one checkpoint can do both tasks; --task picks which
robotserver --show-cameras --policy --gamepad --record \
  --task pet \
  --policy-type smolvla \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/current_models/pet_and_alu/smolvla/11-05-26/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/current_in_use/2026-06-08_PET_ALU_wrist_only
```

# Multitaks Dit — current PET and alu checkpoint

robotserver --show-cameras --policy --gamepad --record \
  --task pet \
  --policy-type multitask-Dit \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/current_models/pet_and_alu/multitask_Dit/09-06-26/150000/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/current_in_use/2026-06-08_PET_ALU_wrist_only


#### Async deployment

Add `--async` to any `--policy` command to deploy through LeRobot's `policy_server`. The policy runs on a background server and the control loop pulls **action chunks** from a queue, so a slow forward pass (Diffusion, multi-task DiT, VLA) never stalls the arm. Everything else — teleop, recording, gamepad policy on/off — works the same as sync mode. Drop the flag to run synchronously (default).

```bash
# Diffusion, async
robotserver --show-cameras --policy --gamepad --async \
  --policy-type diffusion \
  --policy-path <ckpt> --policy-repo-id <dataset> \
  --actions-per-chunk 50 --chunk-threshold 0.5

# Multi-task DiT, async (multi_task_dit is auto-registered as async-capable)
robotserver --show-cameras --policy --gamepad --async \
  --task pet \
  --policy-type multitask-Dit \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/current_models/pet_and_alu/multitask_Dit/09-06-26/150000/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/current_in_use/2026-06-08_PET_ALU_wrist_only 
  --chunk-threshold 0.3
```

| Flag | Default | Purpose |
|------|---------|---------|
| `--async` | off | Enable async deployment (launches the `policy_server` automatically). |
| `--async-port` | 8080 | Port the `policy_server` binds to. |
| `--actions-per-chunk` | 50 | Actions predicted per chunk; match the policy's action horizon. |
| `--chunk-threshold` | 0.5 | Request the next chunk when the action queue drops below this fraction (raise toward 0.7 if you still see stalls). |

Requires `grpcio` in the env: `pip install grpcio protobuf matplotlib`.


For SmolVLA, the **PS / Share** buttons on the gamepad live-switch between `--task` (primary) and `task_secondary` so you can flip between PET and aluminium without restarting. See [SmolVLA](#smolvla) below for details.

Make a wrist-only copy of a `_complete` dataset (frequent — keeps the original intact, writes a sibling `_wrist` dataset with the top-down camera removed):

```bash
DSET=/home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/current_in_use/2026-05-28_ALU_complete   # <-- change this
NEW_DSET="${DSET%_complete}_wrist"
lerobot-edit-dataset \
  --repo_id "$(basename "$DSET")" \
  --root "$DSET" \
  --new_repo_id "$(basename "$NEW_DSET")" \
  --new_root "$NEW_DSET" \
  --operation.type remove_feature \
  --operation.feature_names "['observation.images.topdown']"

lerobot-edit-dataset \
  --repo_id "$(basename "$NEW_DSET")" \
  --root "$NEW_DSET" \
  --operation.type recompute_stats
```

See [Remove a camera feature](#remove-a-camera-feature-eg-drop-the-top-down-camera) for the in-place variant.

---

## Installation

1. **Create and activate the conda environment** (run from the repo root, `/home/arc_user/workspaces/Sorting-piper-robot-server`):

   ```bash
   conda env create -f environment_piper_new.yml
   conda activate piper_new
   ```

   This creates the `piper_new` env (Python 3.12) with Pinocchio from conda-forge and the LeRobot/Piper Python stack via pip.

2. **Install this repo into the env** (also from the repo root):

   ```bash
   pip install -e .
   ```

   The `robotserver` CLI is registered by this install — after it succeeds the command is available whenever the `piper_new` env is activated.

3. **Bringing up the arm.** Plug the USB-CAN adapter in, then run:

   ```bash
   restart-can    # password: arc_user
   ```

   That's all — no need to run `sudo ip link ...` by hand. If at any point the arm stops responding, unplug the USB-CAN, plug it back in, and run `restart-can` again. See [CAN bus setup](#can-bus-setup) for the manual fallback if `restart-can` isn't available on the machine.

---

## Quick test (no robot, no cameras)

Uses Meshcat visualization and local input:

```bash
conda activate piper_new
robotserver --no-robot --vis --keyboard --no-cameras
```

or

```bash
robotserver --no-robot --vis --gamepad --no-cameras
```

---

## CAN bus setup

Piper uses **CAN**; the SDK expects **stable interface names** (not only `can0`):

| Interface | Typical role |
|-----------|----------------|
| `left_piper` | Follower “left” bus |
| `right_piper` | Follower “right” bus |

**Tools (once):** `sudo apt update && sudo apt install -y ethtool can-utils`

### Day-to-day usage (recommended)

On this machine the whole CAN bring-up is wrapped in a single shell command:

```bash
restart-can    # password: arc_user
```

That's it — plug the arm in, run `restart-can`, and you're good to go. If the arm stops responding or CAN looks unstable: unplug the USB-CAN adapter, plug it back in, and run `restart-can` again.

> `restart-can` is a local alias / script on the lab machine. On a fresh machine without it, fall back to the manual bring-up below.

### Manual bring-up (fresh machine, no `restart-can`)

1. `ip -br link show type can` — note the interface (often `can0`).
2. Optional: bus-info for persistent naming — `bash src/piper_teleop/robot_server/find_all_can_port.sh`
3. Rename and bring up (example for `can0` → `left_piper`, 1 Mbit/s):

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 name left_piper
   sudo ip link set left_piper up
   ```

4. Or use the bundled script (edit `USB_PORTS` inside it first if needed):

   ```bash
   sudo bash src/piper_teleop/robot_server/can_config.sh
   ```

5. Verify:

   ```bash
   ip -br link show type can
   ```

6. If only **`left_piper`** exists, the right bus may **fail to connect** in logs; teleop still works for the connected arm. Use the **left** keyboard column for the arm on `left_piper`.

### Run after CAN is up

```bash
robotserver --keyboard
```

If an interface is DOWN: `sudo ip link set <iface> up type can bitrate 1000000` (use your iface name and bitrate).

---

## Configuration

- **`src/piper_teleop/config.py`** — defaults (`TelegripConfig`), including `single_arm`, steps, policy paths.
- **`config.yaml`** (project root, optional) — merged over defaults when present.

### Cameras

Defaults: wrist + top-down monocular (`config.py` / `DEFAULT_CONFIG`). Set `cam_index` after listing devices:

```bash
python scripts/find_cameras.py
```

Per camera, `mode` is one of:

- **`recording`** — dataset recording only.
- **`hybrid`** — recording plus live camera preview/streaming.
- **`streaming`** — live preview/streaming only (not written for recording).

Use `--no-cameras` when testing without devices (cannot combine with `--record` if recording requires camera frames — recording requires at least one camera in `recording` or `hybrid` mode).

---

## Control modes (CLI)

| Flag | Effect |
|------|--------|
| `--keyboard` | Local keyboard control. |
| `--gamepad` | Analog PS4-style gamepad (`gamepad_controller.py`). |
| `--no-robot` | No hardware; sim / visualization path. |
| `--vis` | Meshcat. |
| `--no-cameras` | No camera processes. |
| `--record` / `--resume` | LeRobot-style recording (needs cameras in recording/hybrid). |
| `--task pet\|aluminium\|custom` | Task sentence written into the dataset (and used by language-conditioned policies at inference). `pet` → `"Pick up PET bottle"`, `aluminium` → `"Pick up aluminium can"`, `custom` → prompts for free-form text on stdin. If omitted, falls back to `config.task`. |
| `--name <operator>` | Operator name appended to the session folder, e.g. `--name sebastien` → `data/raw/2026-05-28_14-22-01_sebastien/`. Used with `--record` to track who recorded each session. |
| `--policy` | Learned policy (see [Policy](#policy)); use with `--gamepad` to toggle policy on/off. |
| `--policy-type act\|smolvla\|diffusion\|multitask-Dit` | Policy family; controls camera feature renaming. Required with `--policy`. |
| `--policy-path` | Policy checkpoint path or Hugging Face repo id. Required with `--policy`. |
| `--policy-repo-id` | LeRobot dataset root/repo id used for feature metadata and normalization stats. Required with `--policy`. |
| `--policy-device` | `cuda`, `cpu`, or `auto` for policy inference. |
| `--async` | Deploy via LeRobot's `policy_server` (async, smoother for slow policies). See [Async deployment](#async-deployment). |
| `--async-port` | Port for the async `policy_server` (default 8080). |
| `--actions-per-chunk` | Actions per predicted chunk in async mode (default 50). |
| `--chunk-threshold` | Request the next chunk when the queue drops below this fraction (default 0.5). |
| `--repo-id` | Dataset repo id when recording. |
| `--log-level` | `debug` … `critical` (default `info`). |
| `--show-cameras` | Show local camera preview windows. |

Use **exactly one** of `--keyboard` or `--gamepad`.
Policy mode cannot be combined with `--keyboard`, `--leader`, or `--resume`. Recording while policy is enabled requires `--gamepad`, because Share toggles between manual gamepad control and policy control.

**Examples**

```bash
robotserver --keyboard
robotserver --no-robot --vis --keyboard --no-cameras
robotserver --no-robot --vis --gamepad --no-cameras

# Recording (set --task so the dataset gets the correct task sentence)
robotserver --show-cameras --gamepad --record --task pet
robotserver --show-cameras --gamepad --record --task aluminium
robotserver --show-cameras --gamepad --record --task custom   # prompts for a sentence

# Policy deployment with recording (Share toggles policy vs manual)
robotserver --show-cameras --policy --gamepad --record --task pet \
  --policy-type act \
  --policy-path /path/to/pretrained_model \
  --policy-repo-id /path/to/training_dataset
```

---

## Keyboard teleop

Focus the terminal (`pynput`). With one follower connected, both keyboard columns can drive the same active arm.

| Action | Left arm | Right arm |
|--------|----------|-----------|
| ±X | `w` / `s` | `t` / `g` |
| ±Y | `a` / `d` | `f` / `h` |
| ±Z | `q` / `e` | `r` / `z` |
| Gripper | `Space` or `v` | `Enter` or `n` |
| Reset pose | `x` | `b` |

---

## Gamepad teleop (PS4-style, SDL)

Plug the controller in **before** starting the server. Mapping matches `gamepad_controller.py`: **translation** (left stick + triggers) is in **world** frame; **roll / pitch / yaw** are in **end-effector** frame.

| Action | Control |
|--------|---------|
| Planar X / Y (world) | Left stick (axes 0/1; X/Y swapped after scaling to match world) |
| Z (world) | `L2` − `R2` (axes 2 / 5) |
| Yaw | `R1` − `L1` (EE frame) |
| Roll / pitch | Right stick horizontal / vertical (axes 3 / 4) (EE frame) |
| Gripper closed / open | Cross / Circle |
| Reset EE | Square |
| Primary dropoff pose | Triangle |
| Secondary dropoff pose | Options |

Step sizes come from `pos_step` / `angle_step` in config. Use `--ee-world` for world-fixed roll/pitch/yaw instead of end-effector frame.

Important usage notes:

- Start command: `robotserver --gamepad`.
- Keep one hand near gripper open (`Circle`) when testing new spaces or policies.
- If controls seem dead, restart with controller already plugged in.
- If the arm appears blocked, check collision-space limits (walls/floor/ceiling) and run with `--log-level debug` to see collision pair messages.

---

## Gripper frame vs simulation (optional tuning)

If the Meshcat gripper orientation looks **~90°** off the physical tool, the fixed **link6 → tool** transform in `src/piper_teleop/robot_server/core/piper.py` must match **`kinematics.py`** EE frames. Adjust roll/pitch/yaw (and offset) consistently in both places; see AgileX / Piper SDK docs for `GetArmEndPoseMsgs()` frame conventions.

---

## Recording workflow

1. Set cameras to `recording` or `hybrid` in config.
2. Start a recording session, picking the task preset that matches what you'll demonstrate and optionally tagging the operator:

   ```bash
   robotserver --show-cameras --gamepad --record --task pet --name sebastien
   robotserver --show-cameras --gamepad --record --task aluminium --name madi
   robotserver --show-cameras --gamepad --record --task custom                # prompts on stdin
   ```

   Omitting `--task` falls back to `config.task` (currently `"Pick up PET bottle"`). The chosen sentence is what gets written into `tasks.parquet` in the LeRobot dataset, so be consistent — language-conditioned policies (SmolVLA) train against this exact string. `--name` is purely organizational: it suffixes the session folder (e.g. `data/raw/2026-05-28_14-22-01_sebastien/`) so you can tell at a glance who recorded what.
3. **Recording hotkeys:** `→` cycle states, `←` discard episode, `Esc` stop session.
4. New recordings land under `data/raw/YYYY-MM-DD_HH-MM-SS/` (one folder per session) — see [Dataset folder layout](#dataset-folder-layout).

### Dataset folder layout

Newly recorded sessions are written into `data/raw/` automatically. The recommended workflow keeps raw recordings separate from curated/released datasets so that training is always pointed at an immutable, versioned dataset rather than at whatever was last recorded.

```
data/
  raw/                    # every new --record session lands here, untouched
    2026-05-28_14-22-01/
    2026-05-28_14-45-10/
  reviewed/               # episodes that passed manual QC (moved here by hand)
    2026-05-28_14-22-01/
  rejected/               # bad demos kept for reference; one-line note per folder
  1_good_datasets/        # release-quality merged LeRobot datasets (what training consumes)
    madi_plus_aluminium_merged_wrist_only/
    pet_aliser_plus_alisher_alu_madi_merged/
```

- **`raw/`** — the only place the recorder writes. Never train directly from here.
- **`reviewed/`** — after replaying a session you move (or symlink) the good episodes here.
- **`1_good_datasets/`** — frozen, named LeRobot datasets produced by merging `reviewed/` episodes. This is what `--policy-repo-id` should point at, and what `lerobot-train` should consume. Treat each release as immutable; if you find a bad episode, cut a new release rather than editing an existing one.

Post-process (optional):

```bash
python scripts/convert_images_to_video.py /path/to/dataset
python scripts/upload_to_huggingface.py /path/to/dataset_video ORG/dataset-name
```

Visualizer (web, requires the dataset to be on the Hub): [LeRobot dataset visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset).

### Reviewing episodes locally (triage loop)

To step through every episode of a local dataset and decide which ones to keep or drop, paste this into a terminal and edit only the `DSET=` line:

```bash
DSET=/home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/2026-05-28_PET_complete   # <-- change this
REPO=$(basename "$DSET")
VERDICTS_DIR=/home/arc_user/workspaces/Sorting-piper-robot-server/data/datasets_processings_txts
mkdir -p "$VERDICTS_DIR"
VERDICTS="$VERDICTS_DIR/triage_${REPO}.txt"
: > "$VERDICTS"

for i in $(seq 0 10000); do
  echo "=== Episode $i ==="
  lerobot-dataset-viz --root "$DSET" --repo-id "$REPO" --episode-index "$i" 2>/dev/null &
  PID=$!
  # Detect "episode index out of range" (the script exits within ~1s).
  sleep 1
  if ! kill -0 "$PID" 2>/dev/null; then
    wait "$PID" 2>/dev/null
    echo "No more episodes (stopped at $i)."
    break
  fi
  read -p "Verdict [k=keep / d=drop / s=skip / q=quit]: " v
  kill "$PID" 2>/dev/null; wait "$PID" 2>/dev/null
  case "$v" in
    q) break ;;
    d) echo "$i drop" >> "$VERDICTS" ;;
    k) echo "$i keep" >> "$VERDICTS" ;;
    *) echo "$i skip" >> "$VERDICTS" ;;
  esac
done

echo
echo "Verdicts written to $VERDICTS"
grep drop "$VERDICTS" || echo "(no episodes marked for drop)"
```

- The loop goes up to `10000` and breaks gracefully when the dataset runs out, so it works for any dataset size — no need to look up the episode count first.
- Each iteration opens a rerun window with the wrist video + joint states + actions on a synchronized timeline; scrub through it, then type `k` / `d` / `s` / `q` in the terminal to advance.
- At the end you get `~/triage_<dataset>.txt` listing keep/drop/skip per episode. The `grep drop` line prints just the indices you want to remove, which you can feed into a deletion script later.

---

## Policy

Policy deployment uses LeRobot checkpoints and the dataset metadata/statistics from the dataset used during training. When `--policy` is enabled you must pass the policy type, checkpoint path, and training dataset path/repo id on the command line.

Pass `--task` (`pet` / `aluminium` / `custom`) so the task sentence sent to the policy matches what it was trained on. ACT ignores the task string; SmolVLA / language-conditioned policies require an exact match.

### ACT

ACT checkpoints trained on this repo's wrist camera normally expect `observation.images.wrist1`. Use `--policy-type act`; this keeps the camera name unchanged.

Current PET policy:

```bash
robotserver --show-cameras --policy --gamepad --record --task pet \
  --policy-type act \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/070000/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/02052026_plus_multi_pet_dagger_wrist_only_merged
```

Current aluminium policy:

```bash
robotserver --show-cameras --policy --gamepad --record --task aluminium \
  --policy-type act \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/alu/madi_plus_seb_wrist/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/madi_plus_aluminium_merged_wrist_only
```

### SmolVLA

Use `--policy-type smolvla`; the server applies the `wrist1 -> camera1` camera rename automatically for the wrist-only SmolVLA checkpoints used here. LeRobot/SmolVLA then pads missing camera slots according to the checkpoint config.

```bash
robotserver --show-cameras --policy --gamepad --task pet \
  --policy-type smolvla \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/pet_and_alu_vla/240526/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/pet_aliser_plus_alisher_alu_madi_merged
```

For language-conditioned policies such as SmolVLA, the `--task` value (or the `config.task` fallback in `src/piper_teleop/config.py`) must match the exact sentence the dataset was recorded/trained with — otherwise the policy receives an out-of-distribution prompt.

### Debugging Policy Inputs

To print camera/state/action diagnostics:

```bash
POLICY_DEBUG=1 robotserver --show-cameras --policy --gamepad \
  --policy-type smolvla \
  --policy-path /path/to/pretrained_model \
  --policy-repo-id /path/to/training_dataset
```

To test whether a policy is using the wrist image, blank the wrist camera at inference time:

```bash
POLICY_BLANK_WRIST=1 POLICY_DEBUG=1 robotserver --show-cameras --policy --gamepad \
  --policy-type smolvla \
  --policy-path /path/to/pretrained_model \
  --policy-repo-id /path/to/training_dataset
```

Do not leave `POLICY_BLANK_WRIST=1` set for normal deployment.

Training environment note (current known-good setup for this repo):

```bash
conda activate piper_new
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
```

For video datasets, prefer `--dataset.video_backend=pyav` during `lerobot-train`.

---

## Internal model note

The stack is configured for single-arm model + single-arm IK/control only.

---

## LeRobot helpers

Common dataset maintenance commands using `lerobot-edit-dataset` (already installed in `piper_new`). All examples below operate on a local dataset via `--root`; the `--repo_id` is just a label and doesn't have to match anything on the Hub.

### Recompute statistics

Walks every episode and rebuilds `meta/stats.json` / `meta/episodes_stats/` from the actual data on disk. Use this whenever you've changed the contents of a dataset (added/removed episodes, removed a camera by hand, etc.) and downstream tools complain about stale or missing stats:

```bash
lerobot-edit-dataset \
  --repo_id madi_plus_aluminium_merged_wrist_only \
  --root /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/madi_plus_aluminium_merged_wrist_only \
  --operation.type recompute_stats
```

### Remove a camera feature (e.g. drop the top-down camera)

Removes a feature properly — deletes the corresponding video/parquet entries and updates `info.json` and stats in one shot. Check the `features` block of the dataset's `meta/info.json` to confirm the exact feature name first:

```bash
lerobot-edit-dataset \
  --repo_id madi_plus_aluminium_merged_wrist_only \
  --root /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/madi_plus_aluminium_merged_wrist_only \
  --operation.type remove_feature \
  --operation.feature_names "['observation.images.topdown']"
```

Safer variant — write the result to a new directory instead of mutating the original:

```bash
lerobot-edit-dataset \
  --repo_id madi_plus_aluminium_merged_wrist_only \
  --root /home/arc_user/.../madi_plus_aluminium_merged_wrist_only \
  --new_repo_id madi_plus_aluminium_merged_wrist_only_clean \
  --new_root /home/arc_user/.../madi_plus_aluminium_merged_wrist_only_clean \
  --operation.type remove_feature \
  --operation.feature_names "['observation.images.topdown']"
```

### Merge / combine datasets

Combine several local datasets into a single new release dataset. Provide `--operation.repo_ids` (labels) and `--operation.roots` (their actual paths), plus the destination via `--new_repo_id` + `--new_root`:

```bash
lerobot-edit-dataset \
  --new_repo_id pet_plus_aluminium_merged_wrist_only \
  --new_root /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/pet_plus_aluminium_merged_wrist_only \
  --operation.type merge \
  --operation.repo_ids "['madi_plus_aluminium_merged_wrist_only', 'pet_aliser_plus_alisher_alu_madi_merged']" \
  --operation.roots "['/home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/madi_plus_aluminium_merged_wrist_only', '/home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/pet_aliser_plus_alisher_alu_madi_merged']"
```

The merged dataset gets fresh stats computed across all inputs, so you usually don't need to `recompute_stats` afterwards. Source datasets must share the same feature schema (same cameras, same state/action shape) — if you removed the top-down camera from only one of them, run `remove_feature` on the other first so they match.

### Delete specific episodes

Drops episodes by index and renumbers the rest. Pair this with the [Reviewing episodes locally](#reviewing-episodes-locally-triage-loop) workflow — the `drop` lines in your triage `.txt` give you the indices to pass here:

```bash
lerobot-edit-dataset \
  --repo_id madi_plus_aluminium_merged_wrist_only \
  --root /home/arc_user/.../madi_plus_aluminium_merged_wrist_only \
  --operation.type delete_episodes \
  --operation.episode_indices "[2, 4, 17]"
```

### Convert image frames to video

If a dataset was recorded with `use_video=False` (raw PNG frames under `images/`), this converts it to the video-backed format that training/visualization expects:

```bash
lerobot-edit-dataset \
  --repo_id madi_plus_aluminium_merged_wrist_only \
  --root /home/arc_user/.../madi_plus_aluminium_merged_wrist_only \
  --operation.type convert_image_to_video
```

### Inspect a dataset

Quick overview of episode count, total frames, features, fps, etc. — useful before running any of the destructive operations above:

```bash
lerobot-edit-dataset \
  --repo_id 2026-05-28_PET_complete \
  --root /home/arc_user/workspaces/Sorting-piper-robot-server/data/1_good_datasets/2026-05-28_PET_complete \
  --operation.type info \
  --operation.show_features true
```
