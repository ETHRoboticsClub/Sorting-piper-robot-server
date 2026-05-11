# Piper Robot Server

Teleoperation server for AgileX Piper arms with **keyboard**, **gamepad**, or learned-policy control, optional **cameras** and **LeRobot** recording. Runtime in this repo is **single-arm only** and uses `URDF/Piper/piper_description.urdf`.

**Optional setups:** trained policy (`--policy`).

## Prerequisites

| Requirement | Notes |
|-------------|--------|
| Hardware | One Piper **follower** + USB-CAN for real robot control (single-arm runtime). |
| Conda | Miniconda/Anaconda (Pinocchio is installed from conda-forge). |
| Python | `environment.yml` pins **3.10**; copied lab environments such as `piper_new` may use 3.12 if all LeRobot/Piper dependencies are already installed. |

---

## Installation

1. **Create and activate the conda environment:**

   ```bash
   conda env create -f environment.yml
   conda activate piper_new
   ```

2. **If you did not create the env from `environment.yml`, install this repo from the repo root:**

   ```bash
   pip install -e .
   ```

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

### One follower + one adapter (recommended)

1. `ip -br link show type can` — note the interface (often `can0`).
2. Optional: bus-info for persistent naming — `bash src/piper_teleop/robot_server/find_all_can_port.sh`
3. Rename and bring up (example for `can0` → `left_piper`, 1 Mbit/s):

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 name left_piper
   sudo ip link set left_piper up
   ```

4. If only **`left_piper`** exists, the right bus may **fail to connect** in logs; teleop still works for the connected arm. Use the **left** keyboard column for the arm on `left_piper`.

### Bringup / restart checklist (important)

If the arm does not respond or CAN looks unstable:

1. Stop `robotserver` (`Ctrl+C`).
2. Unplug USB-CAN from the PC.
3. Wait 2-3 seconds, then plug USB-CAN back in.
4. Re-run CAN setup:

   ```bash
   sudo bash src/piper_teleop/robot_server/can_config.sh
   ```

5. Verify interfaces:

   ```bash
   ip -br link show type can
   ```

6. Start the server again (`robotserver --keyboard` or `robotserver --gamepad`).

### Legacy multi-arm notes (not used in this repo runtime)

Edit `USB_PORTS` in `src/piper_teleop/robot_server/can_config.sh`, then:

```bash
sudo bash src/piper_teleop/robot_server/can_config.sh
```

Use one follower CAN interface (`left_piper` or `right_piper`) for single-arm recording.

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
| `--policy` | Learned policy (see [Policy](#policy)); use with `--gamepad` to toggle policy on/off. |
| `--policy-type act\|smolvla` | Policy family; controls camera feature renaming. Required with `--policy`. |
| `--policy-path` | Policy checkpoint path or Hugging Face repo id. Required with `--policy`. |
| `--policy-repo-id` | LeRobot dataset root/repo id used for feature metadata and normalization stats. Required with `--policy`. |
| `--policy-device` | `cuda`, `cpu`, or `auto` for policy inference. |
| `--repo-id` | Dataset repo id when recording. |
| `--log-level` | `debug` … `critical` (default `info`). |
| `--show-cameras` | Show local camera preview windows. |

Use **exactly one** of `--keyboard` or `--gamepad`.
Policy mode cannot be combined with `--keyboard`, `--leader`, or `--resume`. Recording while policy is enabled requires `--gamepad`, because Share toggles between manual gamepad control and policy control.

**Examples**

```bash
robotserver --keyboard
robotserver --no-robot --vis --keyboard --no-cameras
robotserver --record
robotserver --no-robot --vis --gamepad --no-cameras
robotserver --show-cameras --policy --gamepad --policy-type act --policy-path /path/to/pretrained_model --policy-repo-id /path/to/training_dataset
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
2. `robotserver --record`.
3. **Recording hotkeys:** `→` cycle states, `←` discard episode, `Esc` stop session.
4. Data under `data/YYYY-MM-DD_HH-MM-SS/`.

Post-process (optional):

```bash
python scripts/convert_images_to_video.py /path/to/dataset
python scripts/upload_to_huggingface.py /path/to/dataset_video ORG/dataset-name
```

Visualizer: [LeRobot dataset visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset).

---

## Policy

Policy deployment uses LeRobot checkpoints and the dataset metadata/statistics from the dataset used during training. When `--policy` is enabled you must pass the policy type, checkpoint path, and training dataset path/repo id on the command line.

Copy-paste examples for the current local checkpoints:

```bash
# ACT
robotserver --show-cameras --policy --gamepad \
  --policy-type act \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/070000/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/02052026_plus_multi_pet_dagger_wrist_only_merged

# SmolVLA
robotserver --show-cameras --policy --gamepad \
  --policy-type smolvla \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/smolvla/11.5.26/base/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/02052026_plus_multi_pet_dagger_wrist_only_merged_trim_4s_1s
```

### ACT

ACT checkpoints trained on this repo's wrist camera normally expect `observation.images.wrist1`. Use `--policy-type act`; this keeps the camera name unchanged.

```bash
robotserver --show-cameras --policy --gamepad \
  --policy-type act \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/070000/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/02052026_plus_multi_pet_dagger_wrist_only_merged
```

### SmolVLA

Use `--policy-type smolvla`; the server applies the `wrist1 -> camera1` camera rename automatically for the wrist-only SmolVLA checkpoints used here. LeRobot/SmolVLA then pads missing camera slots according to the checkpoint config.

```bash
robotserver --show-cameras --policy --gamepad \
  --policy-type smolvla \
  --policy-path /home/arc_user/workspaces/Sorting-piper-robot-server/outputs/smolvla/11.5.26/base/pretrained_model \
  --policy-repo-id /home/arc_user/workspaces/Sorting-piper-robot-server/data/02052026_plus_multi_pet_dagger_wrist_only_merged_trim_4s_1s
```

The task sentence is defined in `src/piper_teleop/config.py` as `TelegripConfig.task`. For language-conditioned policies such as SmolVLA, keep it exactly aligned with the task string used during dataset recording/training.

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
