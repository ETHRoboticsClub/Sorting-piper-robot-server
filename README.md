# Piper Robot Server

## Installation

> **TODO:** Refresh this section for the environment you actually use. The project is currently developed and run with **Python 3.12**; the commands below still mention an older Python/conda recipe and should be updated to match `pyproject.toml`, pins, and conda packages you rely on today.

### Prerequisites

1. **Robot hardware**: AgileX Piper arm(s) and CAN adapters. The lab uses **either one arm** (follower teleop) **or two arms** in a leader–follower pair (one leader, one follower)—not larger arm counts. A full leader-on-mobile-base setup is **not implemented yet** and would require an **additional AMR** for the leader side.
2. **Conda environment**: Miniconda or Anaconda (often used for `pinocchio` from conda-forge).
3. **Python**: Align with your team’s environment (see note above).

### Package installation

Install tactile-teleop with conda environment setup:

```bash
# Clone the repository

# Create and activate conda environment
conda create -n piper python=3.10
conda activate piper

# Install teleop sdk
git clone https://github.com/TactileRoboticsAI/tactile-teleop-python-sdk.git
cd tactile-teleop-python-sdk
pip install -e .

# Install piper server
cd ..
git clone https://github.com/ETHRoboticsClub/Sorting-piper-robot-server.git  # Or clone over SSH
cd Sorting-piper-robot-server
pip install -e .
conda install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge
```

## Usage

### Enabling robot control (CAN)

To bring up and configure the CAN interfaces after connecting the arm(s) and CAN hardware, from the **repo root** run:

```bash
bash scripts/restart_can.sh
```

That script wraps `src/piper_teleop/robot_server/can_config.sh` (same as `sudo bash` on that file). Use **one CAN link per arm** in practice: a **single follower arm**, or **two arms** for leader–follower (one leader, one follower). Edit `can_config.sh` (e.g. `USB_PORTS`) if your USB layout differs.

**Leader–follower:** driving the follower from the leader in a complete, production-ready way is **still missing**; a full deployment would also mean **another AMR** (separate mobile base) for the leader, which is not in place yet.

Re-run it whenever you need to **restart the CAN connection** after reconnecting cables or power-cycling hardware. Adjust `USB_PORTS` and related settings inside `can_config.sh` if your USB layout changes (see comments in that file).

Renaming CAN ports via that script is **optional** in many lab setups; stable USB ports and a single run of the script are often enough.

### AMR / arm recovery (practical rules)

If the  arm stack misbehaves or crashes:

1. **Power off in order:** unplug **both** supplies you use for the arm side and the CAN side (exact plugs depend on your wiring).
2. **Power on in order:** connect **arm power first**, then **CAN** (or CAN adapter), then bring CAN up again with `bash scripts/restart_can.sh` as above.
3. **Sign of life:** the arm is usually ready when its **green light is blinking** as expected for your firmware.
4. **Following mode:** if someone puts the robot into **follow mode** with the on-robot button, treat it like a bad state: **power-cycle again** (unplug arm + CAN, then power + CAN, then re-run the CAN script).
5. When unplugging power, **support the arm** so it does not drop.

### Enabling cameras

Typical setup:

- **Wrist camera(s)** on the arm(s) you care about (often one wrist cam with a single arm; two arms in leader–follower may use one or two wrist cams depending on config).
- **One top-down** camera for scene context.
- **Optional:** any extra static cameras; add matching entries in configuration.

**Intel RealSense (or other cameras showing up as standard video devices):** use a single discovery pass to learn device indices, then put those indices into `src/piper_teleop/config.py` (or your `config.yaml`) under the `cameras` section so names match what the server expects (`wrist1`, `wrist2`, top-down, etc., depending on how you name them).

```bash
python scripts/find_cameras.py
```

Use the printed indices and, if needed, the preview step in that script to see which physical camera is which.

**Different or custom cameras (e.g. non–RealSense, odd drivers):** you can still use `scripts/find_cameras.py` as a starting point; if that is not enough, fall back to the **legacy** camera and CAN notes at the end of this file.

### Basic commands

**Default (recommended): gamepad teleoperation** via the Tactile stack (same entry point as before—no `--keyboard`):

```bash
robotserver
```

**Run without a physical robot (visualization only):**

```bash
robotserver --no-robot --vis
```

### Command-line options

#### Control modes (read this order)

**Gamepad / Tactile (default, recommended)**  
This is the primary, best-supported path. Other modes exist but are **not fully integrated** with every workflow yet.

- Do **not** pass `--keyboard`. The control loop uses the Tactile API (browser/session-based input, typically used with a **gamepad** in practice).

**`--keyboard`**  
Keyboard control instead of the default. Use for debugging or sim-only runs when you do not want the normal controller path.

**`--leader`**  
**Two arms:** a physical **leader** drives a **follower**. The full workflow is **not fully implemented yet**; a complete setup would add **another AMR** for the leader. The code expects devices on the configured leader ports—treat this as experimental until your hardware layout matches.

**`--policy`**  
Runs a trained LeRobot policy instead of teleop commands for the arm actions. Requires a valid checkpoint and dataset metadata in `src/piper_teleop/config.py` (`policy_path`, `policy_repo_id`) and at least one camera in **recording or hybrid** mode, same as in code.

  - Start the server with:
    ```bash
    robotserver --policy
    ```
  - Keep using the **same recommended gamepad / Tactile session** you use for normal runs so any session-level controls and connection behavior match your lab workflow. Joint targets sent to the arms come from the policy once this mode is active; do not combine with `--keyboard`, `--record`, `--resume`, or `--leader` (the program asserts that).

#### Robot connection

- **`--no-robot`**: No hardware; useful for visualization tests.
- **`--vis`**: Meshcat (or configured) visualization.

#### Recording

- **`--record`**: LeRobot dataset recording (needs at least one camera in recording or hybrid mode).
- **`--resume`**: Continue an existing dataset. --> Not implemented yet?
- **`--repo-id`**: Dataset / repo id string.

#### Logging

- **`--log-level`**: `debug`, `info`, `warning`, `error`, `critical`.

### Common examples

**Recomended for gamepad teleop, policy testing and recording:**

```bash
robotserver --record --policy --gamepad --show-cameras
```



### Dataset recording workflow

Recording uses LeRobot’s `init_keyboard_listener` in `recorder.py` to populate `exit_early`, `rerecord_episode`, and `stop_recording`. **Keyboard** bindings match common upstream LeRobot dataset recording. **Gamepad** bindings are **our custom mapping** (they are **not** LeRobot’s default gamepad layout); they are documented in the `handle_keyboard_event` / **Recorder** docstring in `src/piper_teleop/robot_server/recorder.py`.

| Event (internal name) | Keyboard | Gamepad (custom) |
|------------------------|----------|-------------------|
| `exit_early` — advance / save episode | **→** Right arrow | **Left D-pad Up** |
| `rerecord_episode` — discard episode & reset | **←** Left arrow | **Left D-pad Down** |
| `stop_recording` — end session (use **at the end** so the dataset finalizes) | **Esc** | **Esc** |

These gamepad labels are the **intended custom layout** for our stack; they still arrive through `init_keyboard_listener` as the same three event flags—ensure your deployed listener / controller mapping matches this table.

#### After recording: post-processing

- **Turn image datasets into video-backed datasets** (writes a sibling folder with `_video` in the name via `convert_image_dataset_to_video`):

  ```bash
  python scripts/convert_images_to_video.py /path/to/dataset
  ```

- **Remove bad episodes (already on disk)** and **merge datasets** use upstream LeRobot tooling (this repo does not ship those CLIs). See [Using Dataset Tools](https://huggingface.co/docs/lerobot/main/en/using_dataset_tools) and run `lerobot-edit-dataset --help` for flags on your install. **`lerobot-edit-dataset` and `lerobot.datasets.dataset_tools` are in recent LeRobot releases**—you may need a newer `pip install lerobot` than this package’s `lerobot==0.4.0` pin for `robotserver`; use a separate venv for dataset editing if needed.

  **Delete episodes** (Hub id example; add `--root /path/to/parent` when your dataset is local—exact flags depend on version):

  ```bash
  # Drop episodes 0, 2, and 5 in place
  lerobot-edit-dataset \
      --repo_id YOUR_HF_OR_LOCAL_REPO_ID \
      --operation.type delete_episodes \
      --operation.episode_indices "[0, 2, 5]"

  # Same, but write a new dataset and keep the original
  lerobot-edit-dataset \
      --repo_id YOUR_HF_OR_LOCAL_REPO_ID \
      --new_repo_id YOUR_ORG/my_dataset_cleaned \
      --operation.type delete_episodes \
      --operation.episode_indices "[0, 2, 5]"
  ```

  **Merge datasets** (identical feature schemas; episodes are concatenated in `repo_ids` order):

  ```bash
  lerobot-edit-dataset \
      --repo_id YOUR_ORG/merged_dataset \
      --operation.type merge \
      --operation.repo_ids "['YOUR_ORG/run_a', 'YOUR_ORG/run_b']"
  ```

  **Python API:** implementations live under `lerobot.datasets.dataset_tools`; upstream also provides `examples/dataset/use_dataset_tools.py`.

- **Deleting bad episodes during recording:** use **Left arrow** or **Left D-pad Down** (custom gamepad mapping for `rerecord_episode`) to discard the **current** episode before it is saved.

#### Upload and inspect **Currently not done, perhaps in the future**

```bash
python scripts/upload_to_huggingface.py /path/to/dataset_video ETHRC/my-dataset-name
```

Inspect at [LeRobot Dataset Visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset) with your repo id.

### Training a policy (LeRobot / ACT)

**Prepare the dataset before training:**

1. **Convert images to video** so training can stream frames efficiently (see [After recording: post-processing](#after-recording-post-processing)):
   ```bash
   python scripts/convert_images_to_video.py /path/to/dataset
   ```
   Point training at the resulting `*_video` directory (or whatever path you use as `dataset.repo_id`).
2. **Optionally** delete episodes or **merge** runs with `lerobot-edit-dataset` / `dataset_tools` as in [After recording: post-processing](#after-recording-post-processing).

**ACT with data augmentation** (example: slightly **larger rotation** and **smaller translation** than a naive default; tune for your setup). Set `HF_USER` if you push the policy to the Hub:

```bash
export HF_USER=your_hf_username

lerobot-train \
  --dataset.repo_id=/path/to/your/dataset_video \
  --policy.type=act \
  --output_dir=outputs/train/act_augmented \
  --policy.device=cuda \
  --policy.repo_id=${HF_USER}/act_policy \
  --batch_size=8 \
  --save_freq=20000 \
  --dataset.video_backend=pyav \
  --dataset.image_transforms.enable=true \
  --dataset.image_transforms.tfs='{
    "affine": {"type": "RandomAffine", "kwargs": {"degrees": [-5, 5], "translate": [0.04, 0.04]}, "weight": 1.0},
    "brightness": {"type": "ColorJitter", "kwargs": {"brightness": [0.7, 1.3]}, "weight": 1.5},
    "contrast": {"type": "ColorJitter", "kwargs": {"contrast": [0.7, 1.3]}, "weight": 1.5},
    "hue": {"type": "ColorJitter", "kwargs": {"hue": [-0.05, 0.05]}, "weight": 0.8},
    "saturation": {"type": "ColorJitter", "kwargs": {"saturation": [0.5, 1.5]}, "weight": 1.0},
    "sharpness": {"type": "SharpnessJitter", "kwargs": {"sharpness": [0.5, 1.5]}, "weight": 0.0}
  }'
```

**ACT without data augmentation** (same run, minus image transforms):

```bash
lerobot-train \
  --dataset.repo_id=/path/to/your/dataset_video \
  --policy.type=act \
  --output_dir=outputs/train/act_no_aug \
  --policy.device=cuda \
  --policy.repo_id=${HF_USER}/act_policy \
  --batch_size=8 \
  --save_freq=20000 \
  --dataset.video_backend=pyav
```

For **other policy types** (diffusion, VLA, etc.), flags and dataset requirements differ—use the official [LeRobot](https://github.com/huggingface/lerobot) documentation and examples for the architecture you choose.

### Running a policy (LeRobot / ACT)

> So far this path has mainly been exercised with **ACT**-style policies trained in LeRobot.

1. Train a policy as in [Training a policy (LeRobot / ACT)](#training-a-policy-lerobot--act). Copy the resulting checkpoint under `policy_checkpoint/` (or set **`policy_path`** in config to its location).
2. Set **`policy_path`** and **`policy_repo_id`** in `src/piper_teleop/config.py`.
3. Run:

   ```bash
   robotserver --policy
   ```

> **Note:** Longer term, policy logic may move out of this repo and consume the robot server from dedicated policy repositories (e.g. OpenPI or LeRobot).

---

## Legacy notes (older docs — may be outdated)

The following matched an **older** workflow and manual CAN port hunting. Treat it as **historical**; it may need edits to work with your current Python 3.12 stack, RealSense-only camera layouts, and gamepad-first operation.

- **CAN enumeration:** Unplug USB, plug one device at a time and run `src/piper_teleop/robot_server/find_all_can_port.sh`, then edit `can_config.sh` USB port maps (lines referenced in older docs around the `USB_PORTS` / device naming section). Prefer **`bash scripts/restart_can.sh`** (same as `can_config.sh`) unless you are debugging port identity.
- **Default `DEFAULT_CONFIG` cameras** in `config.py` still list a **stereo** entry alongside wrist entries; your deployment might only use **wrist + top-down + optional** cameras—trim or override in `config.yaml` accordingly.
- **`python scripts/find_cameras.py`** has long been the generic “old script” for OpenCV index discovery on Linux.

If something here disagrees with the current lab setup, update this section or remove it once the team no longer needs it.
