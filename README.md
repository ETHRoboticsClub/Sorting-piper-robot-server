# Piper Robot Server

Teleoperation, data recording, and policy deployment for a single AgileX Piper arm.

One command drives everything: `robotserver`. It connects to the arm over CAN, runs a
control loop at fixed rate, optionally streams and records cameras, and takes input from a
gamepad, a keyboard, a leader arm, or a trained policy.

---

## Install

Run everything from the repo root.

```bash
conda env create -f environment_piper_new.yml
conda activate piper_new
pip install -e .
```

This creates the `piper_new` env (Pinocchio from conda-forge, LeRobot + Piper SDK via pip)
and registers the `robotserver` CLI. The CLI is available whenever the env is active.

Add a convenience alias for the CAN bring-up script:

```bash
echo 'alias restart-can="'"$PWD"'/scripts/restart_can.sh"' >> ~/.bash_aliases
```

---

## Quickstart

```bash
restart-can            # bring up the CAN bus (asks for your sudo password)
conda activate piper_new
robotserver --gamepad  # drive the arm
```

Check the bus came up before blaming the arm — you want `left_piper`, state `UP`:

```bash
ip -brief link show type can
```

If you see `can0` instead, or state `DOWN`, run `restart-can` again. If the arm stops
responding mid-session, unplug the USB-CAN adapter, plug it back in, and re-run it.

### Try it without hardware

```bash
robotserver --no-robot --vis --gamepad --no-cameras
```

Opens a Meshcat viewer at <http://127.0.0.1:7000/static/>. Useful for checking IK and
collision geometry with no arm and no cameras attached.

---

## Controlling the arm

### Gamepad (PS4-style, recommended)

```bash
robotserver --gamepad
```

| Input | Action |
|---|---|
| Left stick | Move in X / Y (world frame) |
| L2 / R2 | Move down / up (Z) |
| Right stick | Roll (↔) and pitch (↕) |
| L1 / R1 | Yaw |
| Cross ✕ | Close gripper |
| Circle ○ | Open gripper |
| Square ▢ | Reset to start pose |
| Triangle △ | Go to deposit pose |
| D-pad Up | Next episode (while `--record`) |
| D-pad Down | Discard episode (while `--record`) |

Translation is always in the world frame. Rotation defaults to the end-effector frame —
add `--ee-world` to rotate in the world frame instead.

### Keyboard

```bash
robotserver --keyboard
```

| Action | Key |
|---|---|
| Forward / back (±X) | `w` / `s` |
| Left / right (±Y) | `a` / `d` |
| Up / down (±Z) | `q` / `e` |
| Toggle gripper | `Space` (or `v`) |
| Reset position | `x` |

Steps are in the world/base frame. Use `v` if your terminal swallows `Space`.

### Cameras

Cameras start automatically when configured. Two flags adjust that:

```bash
robotserver --gamepad --show-cameras   # local preview windows
robotserver --gamepad --no-cameras     # skip capture entirely
```

Use `--no-cameras` when no RealSense is plugged in — otherwise startup fails with
`No Intel RealSense devices detected`.

---

## Recording data

```bash
# Pick up PET bottle
robotserver --show-cameras --gamepad --record --task pet --name sebastien

# Pick up aluminium can
robotserver --show-cameras --gamepad --record --task aluminium --name sebastien

# Free-form — you are prompted for the sentence on stdin
robotserver --show-cameras --gamepad --record --task custom --name sebastien
```

The three presets and the sentences they write:

| `--task` | Primary task | Secondary task |
|---|---|---|
| `pet` | Pick up PET bottle | Pick up aluminium can |
| `aluminium` | Pick up aluminium can | Pick up PET bottle |
| `custom` | *typed at the prompt* | — |

The two presets are the same pair in opposite order — the secondary is what the gamepad's
**PS** button selects when deploying a language-conditioned policy. Omit `--task` entirely
and it falls back to `config.task`.

| Flag | Meaning |
|---|---|
| `--record` | Write a LeRobot dataset |
| `--task` | `pet`, `aluminium`, or `custom` — see above |
| `--name` | Operator tag appended to the session folder |
| `--resume` | Continue the previous session |

**Hotkeys while recording:**

| Action | Keyboard | Gamepad |
|---|---|---|
| Next episode | `→` | D-pad **Up** |
| Discard episode | `←` | D-pad **Down** |
| End session | `Esc` | — |

Note the axes differ: the keyboard uses left/right, the gamepad uses up/down.

The `--task` string is written into `tasks.parquet` and is what language-conditioned
policies train against, so keep it consistent across sessions.

Sessions land in `data/raw/YYYY-MM-DD_HH-MM-SS_<name>/`. The convention is to keep raw
recordings immutable and promote good ones by hand:

```
data/
  raw/               # every --record session lands here, untouched
  reviewed/          # passed manual QC
  rejected/          # bad demos, kept for reference
  1_good_datasets/   # merged, release-quality — what training consumes
```

### Dataset workflow

Recording is the first step of five. The order matters — each step assumes the one before it.

#### 1. Discard bad demos *while* recording

Dump it the moment a demo goes wrong — `←` on the keyboard, **D-pad Down** on the gamepad.
This is by far the cheapest place to remove a bad episode: nothing is written, nothing has
to be renumbered, and no statistics go stale. Since you record with `--gamepad` in hand,
the D-pad is the one you will actually use.

#### 2. Delete the bad ones you kept anyway

If a bad demo made it to disk, drop it by index. This renumbers the survivors and updates
the metadata in one shot:

```bash
lerobot-edit-dataset \
  --repo_id 2026-08-18_pet_sebastien \
  --root data/raw/2026-08-18_14-22-01_sebastien \
  --operation.type delete_episodes \
  --operation.episode_indices "[2, 4, 17]"
```

To decide *which* indices, step through the episodes first:

```bash
lerobot-dataset-viz --root <dataset> --repo-id <name> --episode-index 0
```

Each call opens a rerun window with the wrist video, joint states, and actions on one
timeline. Increment `--episode-index` until it errors, which is how you find the end.

> **If `delete_episodes` aborts** with `Episode length mismatch`, the dataset has an episode
> whose video is longer than its data. See [Faulty recordings from before the
> switch](#faulty-recordings-from-before-the-switch) — it has to be handled before any
> deletion will succeed.

#### 3. Convert frames to video

Recording writes **PNG frames**, not video — `use_video = False` in
[`config.py`](src/piper_teleop/config.py). Encode afterwards, once the session is safely on
disk and deletions are done:

```bash
python scripts/convert_images_to_video.py data/raw/2026-08-18_14-22-01_sebastien
```

Writes a new dataset suffixed `_video`, leaving the frames untouched. `--output <dir>` to
redirect, `--dry-run` to preview. LeRobot has an equivalent operation if you prefer it:

```bash
lerobot-edit-dataset --repo_id <name> --root <dataset> \
  --operation.type convert_image_to_video
```

Setting `convert_images_to_video = True` in [`config.py`](src/piper_teleop/config.py)
converts automatically at the end of each session. The manual route is safer — automatic
conversion runs while the session is still open, so a crash there takes the session too.

> ### ⚠️ Do not set `use_video = True`
>
> It looks like it saves a post-processing step. It does not — it corrupts recordings.
>
> LeRobot's live streaming encoder runs in a thread parallel to the data buffer, and the two
> desync on discard/re-record and at episode boundaries. The result is episodes whose video
> holds **more frames than the data has rows**: an aborted take's video leaks into the saved
> episode while its data is correctly dropped. Verified by inspection — the excess frames are
> real moving video with a hard scene cut exactly where the data ends.
>
> With `use_video = False` each frame is written 1:1 with its data row, and
> `convert_images_to_video` rebuilds the video *from* the parquet rows, so video and data
> match by construction. It cannot glitch. The cost is disk space and one command.

#### Faulty recordings from before the switch

Datasets recorded while `use_video = True` may contain **glitched episodes** — video
frame-span greater than data row count. The data is clean and those episodes train fine;
only the video carries an unreferenced tail.

The damage shows up at deletion time. `delete_episodes` re-slices any mp4 holding both kept
and deleted episodes, and asserts `episode length == video frame span` for every *kept*
episode. One glitched kept episode aborts the whole operation:

```
AssertionError: Episode length mismatch: 412 vs 448
```

**Detect** — per episode, compare `length` against
`round((to_timestamp - from_timestamp) * fps)` for each camera in `meta/episodes/*.parquet`.

**Fix** — either include the glitched episodes in the same delete list as your real targets
(the re-encode trims the bad tails), or repair first by setting
`to_timestamp = from_timestamp + length/fps`.

Delete everything in **one** command with a multi-index list. Separate commands renumber
indices in between, so the second command targets the wrong episodes. In-place edits
auto-create a `<name>_old` backup.

#### 4. Merge sessions into a training dataset

Combine several sessions into one release dataset. Inputs must share a feature schema —
same cameras, same state/action shapes:

```bash
lerobot-edit-dataset \
  --new_repo_id pet_plus_aluminium_merged \
  --new_root data/1_good_datasets/pet_plus_aluminium_merged \
  --operation.type merge \
  --operation.repo_ids "['session_a', 'session_b']" \
  --operation.roots "['data/reviewed/session_a', 'data/reviewed/session_b']"
```

Merging computes fresh statistics across all inputs, so step 5 is usually unnecessary
afterwards.

#### 5. Recompute statistics

Rebuilds `meta/stats.json` from the data actually on disk. **Required whenever you change a
dataset's contents by any means other than the commands above** — editing parquet columns
by hand, for instance. Skipping it is a silent failure: normalization keeps dividing by the
old σ and nothing errors.

```bash
lerobot-edit-dataset --repo_id <name> --root <dataset> \
  --operation.type recompute_stats
```

#### Trimming episodes to the pickup window (optional)

[`scripts/trim_episodes_to_pickup.py`](scripts/trim_episodes_to_pickup.py) (by Aliser) cuts
each episode to `[t_close - pre, t_close + post]`, where `t_close` is the first frame the
gripper closes — dropping long approach segments. It writes a new dataset; the source is
untouched.

```bash
# check the detection on one episode first
python scripts/trim_episodes_to_pickup.py inspect \
  --src-repo-id <name> --src-root <dataset> --episode 0 --closed-threshold 0.048

# then trim everything
python scripts/trim_episodes_to_pickup.py trim \
  --src-repo-id <name> --src-root <dataset> \
  --dst-repo-id <name>_trimmed --dst-root <output> \
  --pre 3.0 --post 1.0 --closed-threshold 0.048
```

> **Override the default `--closed-threshold 0.5`** — it assumes a 0–1 gripper channel. Ours
> is metres, so every frame reads as closed and episodes get trimmed from frame 0, silently.
> Use **0.048** for datasets recorded after the gripper calibration (0 → 0.096 m), or
> **0.035** for older ones (0 → 0.07 raw).

#### Inspecting a dataset

Episode count, frames, features, fps — worth running before anything destructive:

```bash
lerobot-edit-dataset --repo_id <name> --root <dataset> \
  --operation.type info --operation.show_features true
```

#### Removing a camera feature

Drops a camera properly, updating videos, parquet, `info.json`, and stats together. Check
the exact feature name in `meta/info.json` first:

```bash
lerobot-edit-dataset --repo_id <name> --root <dataset> \
  --operation.type remove_feature \
  --operation.feature_names "['observation.images.topdown']"
```

Add `--new_repo_id` and `--new_root` to write a copy instead of mutating the original.


---

## Deploying a policy

```bash
robotserver --show-cameras --policy --gamepad --record \
  --task pet \
  --policy-type act \
  --policy-path <checkpoint dir> \
  --policy-repo-id <dataset the policy was trained on>
```

`--policy-type` is one of `act`, `diffusion`, `smolvla`, `multitask-Dit`. It controls how
camera features are renamed for the checkpoint, so it has to match how the model was
trained. `--policy-repo-id` supplies the normalization stats and must point at the
training dataset.

Keeping `--gamepad` alongside `--policy` gives you manual override: the arm starts in
gamepad mode, **Share** enables the policy with the primary task, **PS** with the secondary
task. Press again to hand control back. Always run with the gamepad connected so you can
take over.

### Async deployment

Slow policies (Diffusion, multi-task DiT, VLA) block the control loop on every forward
pass. `--async` runs the policy in LeRobot's `policy_server` and pulls action chunks from a
queue instead:

```bash
robotserver --show-cameras --policy --gamepad --async \
  --policy-type diffusion \
  --policy-path <ckpt> --policy-repo-id <dataset> \
  --actions-per-chunk 50 --chunk-threshold 0.5
```

Raise `--chunk-threshold` toward 0.7 if the arm still stalls between chunks.

---

## Configuration

[`config.yaml`](config.yaml) holds cameras, IK settings, deposit poses, and paths. Camera
setup is the part you are most likely to touch:

- OpenCV cameras: set `cam_index` after running `python scripts/find_cameras.py`
- RealSense cameras: set `backend: realsense` and `serial_number` after running
  `python scripts/find_realsense_cameras.py`

Binding by serial number is deterministic; `/dev/video*` ordering is not.

---

## Gripper calibration

The firmware misreports jaw opening — both its origin and its scale are wrong for the
gripper actually fitted. [`gripper_calibration.py`](src/piper_teleop/robot_server/core/gripper_calibration.py)
converts between firmware units and true metres, and is the only place the two meet.
Everything else in the codebase speaks true metres (0 closed → 0.096 m open).

After a gripper swap, re-fit it with two measurements and no code: drive the jaws to each
stop unloaded, read the firmware value, measure the opening with a tape, and replace the
three constants at the top of that file.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `CAN bus connection was not properly initialized` | Bus is down — run `restart-can`, then check `ip -brief link show type can` |
| `No Intel RealSense devices detected` | Camera unplugged — add `--no-cameras` or fix the connection |
| Arm enables but won't move | Another `robotserver` may still hold the bus; check for a stale process |
| Readings are all zeros | No CAN frames arriving — check 24 V power, E-stop, and cable at both ends |

---

## Repo layout

```
src/piper_teleop/robot_server/   control loop, controllers, cameras, policy, recorder
  core/                          IK, kinematics, Piper SDK interface, gripper calibration
scripts/                         CAN bring-up, camera discovery, dataset helpers
URDF/Piper/                      arm model + collision geometry for IK
config.yaml                      cameras, IK, paths, deposit poses
data/                            recordings (gitignored)
outputs/                         training checkpoints (gitignored)
```

Useful scripts:

| Script | Purpose |
|---|---|
| `scripts/restart_can.sh` | Bring up the CAN bus as `left_piper` at 1 Mbit/s |
| `scripts/find_cameras.py` | List OpenCV camera indices |
| `scripts/find_realsense_cameras.py` | List RealSense serial numbers |
| `scripts/piper_joint_diag.py` | Per-joint current, temperature, and fault diagnostics |
| `scripts/upload_to_huggingface.py` | Push a dataset to the Hub |
| `scripts/convert_images_to_video.py` | Convert recorded frames to video |
