# Piper Robot Server

Teleoperation server for AgileX Piper arms: **VR (Tactile / LiveKit)**, **keyboard**, or **gamepad** goals, optional **cameras** and **LeRobot** recording. The URDF and IK stack are **dual-arm**; the default config (`single_arm: true`) matches **one** physical follower for typical datasets.

**Optional setups:** leader–follower (`--leader`, four CAN links), dual followers (`single_arm: false`), trained policy (`--policy`).

---

## Prerequisites

| Requirement | Notes |
|-------------|--------|
| Hardware | At least one Piper **follower** + USB-CAN for real robot control. Optional: second follower, leader arms (see [CAN](#can-bus-setup)). |
| Conda | Miniconda/Anaconda (Pinocchio is installed from conda-forge). |
| Python | **3.10** in the conda env (`pyproject.toml` allows ≥3.8; lab uses 3.10). |
| Tactile SDK | `tactile-teleop-python-sdk` (clone + `pip install -e .`). |
| VR | Only if you use default VR control (omit `--keyboard` / `--gamepad`). |

---

## Installation

1. **Install the [Tactile teleop Python SDK](https://github.com/TactileRoboticsAI/tactile-teleop-python-sdk):**

   ```bash
   git clone https://github.com/TactileRoboticsAI/tactile-teleop-python-sdk.git
   cd tactile-teleop-python-sdk
   conda create -n piper-teleop python=3.10 -y
   conda activate piper-teleop
   pip install -e .
   cd ..
   ```

2. **Clone and install this repo:**

   ```bash
   git clone https://github.com/ETHRoboticsClub/piper-robot-server.git
   cd piper-robot-server
   pip install -e .
   conda install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge
   ```

3. **Environment variables (VR):** set `TACTILE_API_KEY` in `.env` or the environment when using VR/Tactile (not needed for `--keyboard` / `--gamepad`).

---

## Quick test (no robot, no cameras)

Uses Meshcat visualization and local input (no LiveKit VR session):

```bash
conda activate piper
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
| `leader_left`, `leader_right` | Leader–follower only (`ENABLE_LEADER_ARMS=true` in `can_config.sh`) |

**Tools (once):** `sudo apt update && sudo apt install -y ethtool can-utils`

### One follower + one adapter

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

### Two followers

Edit `USB_PORTS` in `src/piper_teleop/robot_server/can_config.sh`, then:

```bash
sudo bash src/piper_teleop/robot_server/can_config.sh
```

Set `single_arm: false` in config when recording both arms. Confirm `ip -br link show type can` shows `left_piper` and `right_piper` UP.

### Leader–follower (four CAN devices)

Map all four interfaces in `can_config.sh`, then:

```bash
ENABLE_LEADER_ARMS=true sudo bash src/piper_teleop/robot_server/can_config.sh
robotserver --leader --keyboard
```

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

- **`recording`** — dataset only (no VR stream).
- **`hybrid`** — VR + record.
- **`streaming`** — VR only (not written for recording).

Use `--no-cameras` when testing without devices (cannot combine with `--record` if recording requires camera frames — recording requires at least one camera in `recording` or `hybrid` mode).

---

## Control modes (CLI)

| Flag | Effect |
|------|--------|
| *(default)* | VR / Tactile goals (`connect_vr_controller`); needs API key / session. |
| `--keyboard` | Local keyboard; no VR connection. |
| `--gamepad` | Analog PS4-style gamepad (`gamepad_controller.py`). |
| `--no-robot` | No hardware; sim / visualization path. |
| `--vis` | Meshcat. |
| `--no-cameras` | No camera processes. |
| `--record` / `--resume` | LeRobot-style recording (needs cameras in recording/hybrid). |
| `--leader` | Leader–follower (four CAN links configured). |
| `--policy` | Learned policy (see [Policy](#policy)); mutually exclusive with keyboard/gamepad/record. |
| `--repo-id` | Dataset repo id when recording. |
| `--log-level` | `debug` … `critical` (default `info`). |

Use **exactly one** of `--keyboard` or `--gamepad`.

**Examples**

```bash
robotserver --keyboard
robotserver --no-robot --vis --keyboard --no-cameras
robotserver --record
robotserver --no-robot --vis --gamepad --no-cameras
```

---

## Keyboard teleop

Focus the terminal (`pynput`). With **one** follower on `left_piper`, **both** columns drive that arm; with two followers, columns map left vs right.

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

Step sizes come from `pos_step` / `angle_step` in config. Use `--ee-world` for world-fixed roll/pitch/yaw instead of end-effector frame.

---

## Gripper frame vs simulation (optional tuning)

If the Meshcat gripper orientation looks **~90°** off the physical tool, the fixed **link6 → tool** transform in `src/piper_teleop/robot_server/core/piper.py` must match **`kinematics.py`** EE frames. Adjust roll/pitch/yaw (and offset) consistently in both places; see AgileX / Piper SDK docs for `GetArmEndPoseMsgs()` frame conventions.

---

## Recording workflow

1. Set cameras to `recording` or `hybrid` in config.
2. `robotserver --record` (or `--record --leader` if using leaders).
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

Tested with **ACT**-style policies trained in LeRobot. Copy checkpoints under `policy_checkpoint/`, set `policy_path` and `policy_repo_id` in `config.py`, then:

```bash
robotserver --policy
```

---

## Internal model note

The stack uses a **dual-arm URDF** even when only **one** arm is connected; IK and visualization still instantiate both chains. Single-arm operation is a **configuration** choice, not a separate binary.
