# Piper Robot Server

This README is written for the **default lab setup**: **one AgileX Piper follower arm**, **wrist + top-down cameras**, and **keyboard or VR** teleop. **`single_arm`** is on by default in `config.py` so datasets use one arm’s joints.

**Optional later:**

- **Leader–follower**: physical **leader** arms drive the **follower** (`--leader`, four CAN links).
- **Two followers**: dual-arm teleop / recording (`single_arm: false`, two follower CAN buses).

The simulation and control stack still use a **dual-arm URDF** internally; see [Enabling Robot Control (CAN)](#enabling-robot-control-can) for how that interacts with a **single** physical arm.

## Installation

### Prerequisites

1. **Robot hardware**
   - **Required for follower teleop:** at least **one** AgileX Piper **follower** arm and a **USB-CAN** interface to the PC.
   - **Optional — leader-follower:** two Piper **leader** arms (in addition to followers) and **four** CAN links total — see [Optional leader-follower (four CAN interfaces)](#optional-leader-follower-four-can-interfaces).
   - **Optional — second follower:** second Piper follower and second CAN link for dual-arm operation.
2. **Conda**: Miniconda or Anaconda (required for pinocchio from conda-forge)
3. **Python**: 3.10 (conda environment)
4. **VR** (optional): Meta Quest or other WebXR headset — only if you use Tactile VR instead of `--keyboard`

### Package Installation

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
git clone https://github.com/ETHRoboticsClub/piper-robot-server.git
cd piper-robot-server
pip install -e .
conda install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge
```

## Usage

### Enabling Robot Control (CAN)

Piper arms use **CAN** (USB-CAN adapters). The driver/SDK expects **stable interface names**, not only `can0`:

| Interface name   | Role (typical)        |
|------------------|------------------------|
| **`left_piper`** | Follower “left” bus    |
| **`right_piper`**| Follower “right” bus   |
| **`leader_left`**, **`leader_right`** | Only if you use leader–follower (`ENABLE_LEADER_ARMS=true`) |

**Install tools (once):**

```bash
sudo apt update && sudo apt install -y ethtool can-utils
```

---

#### One follower arm (typical)

You have **one** Piper follower and **one** USB-CAN adapter.

1. Plug the adapter, then check:

   ```bash
   ip -br link show type can
   ```

   You should see one interface (often `can0`).

2. Record **bus-info** (needed if you use `can_config.sh`):

   ```bash
   bash src/piper_teleop/robot_server/find_all_can_port.sh
   # or:
   sudo ethtool -i can0 | grep bus-info
   ```

3. **Name and bring up that interface as `left_piper`** (attach your arm to this bus). Example if the kernel still calls it `can0`:

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 name left_piper
   sudo ip link set left_piper up
   ```

   Adjust bitrate if your hardware requires it (often `1000000` for Piper).

4. The server **still attempts** `right_piper`. If you only have **`left_piper`**, the right connection will **fail** in the log, but **teleop works**: the stack treats **at least one** successful arm as connected and **sends commands only to that arm**. Use the **left** keyboard column (`wasd`, etc.) for the arm on `left_piper`.

---

#### Two follower arms (dual follower)

Use **two** CAN links (two adapters or two USB ports). Map each to **bus-info** in `src/piper_teleop/robot_server/can_config.sh` (`USB_PORTS` around lines 128–131):

```bash
USB_PORTS["<bus-info-for-first-dongle>"]="left_piper:1000000"
USB_PORTS["<bus-info-for-second-dongle>"]="right_piper:1000000"
```

Then:

```bash
sudo bash src/piper_teleop/robot_server/can_config.sh
```

Verify:

```bash
ip -br link show type can
```

You want **`left_piper`** and **`right_piper`**, both **UP**. Set **`single_arm: false`** in `config.py` if you record both arms.

---

#### Optional leader-follower (four CAN interfaces)

You have **two leader** arms and **two follower** arms (four USB-CAN devices total).

1. Edit **`USB_PORTS`** in `can_config.sh` for **all four** entries: **`left_piper`**, **`right_piper`**, **`leader_left`**, **`leader_right`** (see comments in the script for the leader keys).

2. Run:

   ```bash
   ENABLE_LEADER_ARMS=true sudo bash src/piper_teleop/robot_server/can_config.sh
   ```

3. Start teleop with **`--leader`**:

   ```bash
   robotserver --leader --keyboard
   # or with recording:
   robotserver --record --leader
   ```

Leaders must match the ports expected by your `can_config.sh` mapping.

---

#### Run the server (after CAN is up)

Example — **keyboard** on real hardware:

```bash
robotserver --keyboard
```

If only **`left_piper`** exists, **`right_piper`** will error in the log; teleop still runs for the connected arm (see one-arm steps above).

**If CAN is `DOWN`:**

```bash
sudo ip link set <iface> up type can bitrate 1000000
```

(use the iface name and bitrate you configured).

### Gripper / end-effector orientation

There is **no separate gripper calibration routine** in this server (`Piper.calibrate()` is unused). The **90°-looking twist** usually comes from a **fixed transform** between the Piper SDK’s reported end pose (link-6 / flange frame) and the **tool frame** used for IK and Meshcat.

1. **`src/piper_teleop/robot_server/core/piper.py`** — `get_end_effector_transform()` applies  
   `link6_to_gripper_transform = xyzrpy2transform(0.0, 0.0, 0.13, 0.0, -1.57, 0.0)`  
   after the SDK pose. The **−1.57 rad pitch** is a −90° rotation; if your official teleop script uses a different tool convention, this will look “wrong” in the sim or relative to the real gripper.

2. **`src/piper_teleop/robot_server/core/kinematics.py`** — the `ee` / `arm2_ee` frames use `first_matrix` / `second_matrix` (rotation **−90°** on one axis plus **0.13 m** offset). **IK targets must match the same convention** as `piper.py`, or the model and the arm disagree.

**What to do:** adjust **roll / pitch / yaw** (and optionally **0.13 m** offset) in **`piper.py`** until the Meshcat gripper matches what you expect, then apply the **same** geometric relationship in **`kinematics.py`** for the `ee` (and `arm2_ee`) frame so IK still tracks the real arm. Try small changes (e.g. add **±π/2** to **roll** or **yaw** instead of pitch) if the error is exactly 90° about one axis.

Check **AgileX / Piper SDK docs** for the definition of `GetArmEndPoseMsgs()` (which axis is “forward” for the gripper) so your edits match the manufacturer frame.

### Enabling Cameras

The defaults in `src/piper_teleop/config.py` assume **one wrist camera** (`wrist1`) and **one top-down camera** (`topdown`), both **monocular**. Recorded image tensors use **640×480**; capture is set to **1280×720** where the device supports it (e.g. Intel RealSense D455 RGB over V4L2). Lower `capture_frame_width` / `capture_frame_height` if opening the device fails.

1. **Connect** the cameras (USB).

2. **Find OpenCV indices** for each physical device:
   ```bash
   python scripts/find_cameras.py
   ```
   Use the summary indices to set `cam_index` for `wrist1` and `topdown`.

3. **Configure** either:
   - **`src/piper_teleop/config.py`** — `DEFAULT_CONFIG["cameras"]`, or
   - **`config.yaml`** in the project root (merged over defaults when present).

4. **Modes** (`mode` per camera):
   - **`recording`**: Images go to the recorder only (no VR streaming). Use when you are **not** using a headset.
   - **`hybrid`**: Stream to Tactile/VR **and** record.
   - **`streaming`**: VR stream only; that camera is not written to shared memory for recording.

5. **Optional — stereo VR**: For a **single wide frame** split into left/right for binocular VR, add a camera with `type: stereo` and matching `capture_frame_*` (see the commented example in `config.py`). You usually choose either a top-down **or** a stereo scene view, not both, unless your client needs both feeds.

6. **`single_arm`**: Default is **`true`** — matches **one** follower arm in the dataset. Set to **`false`** only if you run **two** physical followers and want **14** joint channels.

**Other layouts** (e.g. extra wrist cam): add entries under `cameras`; each needs a unique key, `type`, `mode`, and `cam_index`.

### Basic Commands

**Run the robot server with VR control (default):**
```bash
robotserver
```

**Record without a headset** (cameras in `recording` mode; no VR streaming):
```bash
robotserver --record
```

**Run without physical robot (visualization only):**

By default the server uses **VR / Tactile** for goals (`--keyboard` is off). So **`robotserver --no-robot --vis`** alone still **connects to Tactile LiveKit** and expects a VR session (API key / room). That is intentional for headset teleop with sim.

For **visualization only without a headset**, pass **`--keyboard`** so goals come from the keyboard, not VR:

```bash
robotserver --no-robot --vis --keyboard
```

If you also have no working cameras, add **`--no-cameras`** (see below).

### Keyboard teleop

1. Start with **`--keyboard`** so the server does not use VR/Tactile for goals.
2. **Focus the terminal** that runs `robotserver` so key events are received (`pynput`).
3. **Commands** — with **one** follower on **`left_piper`**, **both** key columns (**`wasd`** and **`tgfh`**, etc.) move that arm. With **one** follower on **`right_piper`**, both columns move that arm. With **two** followers, columns are **left** vs **right** arm as in the table.

   | | Left arm | Right arm |
   |--|----------|-----------|
   | Forward / back (+X / −X) | `w` / `s` | `t` / `g` |
   | Left / right (+Y / −Y) | `a` / `d` | `f` / `h` |
   | Up / down (+Z / −Z) | `q` / `e` | `r` / `z` |
   | Toggle gripper | `Space` or **`v`** | `Enter` or **`n`** |
   | Reset arm to initial pose | `x` | `b` |

   If **`Space` does nothing** (common on some Linux desktops), use **`v`** / **`n`**. Gripper also failed silently before when IK was in **collision**; that path is fixed so toggles still apply.

   Hold movement keys for repeated small steps (default **0.01 m**; see `KeyboardController` in `keyboard_controller.py`).

**Examples**

```bash
# Real robot (CAN connected), keyboard only
robotserver --keyboard

# Sim + Meshcat, no cameras
robotserver --no-robot --vis --keyboard --no-cameras
```

### Command Line Options

#### Control Modes

- **`--keyboard`**: Use **keyboard** for arm goals instead of **VR / Tactile**
  - Default is **`False`**: if you omit `--keyboard`, the control loop calls **`connect_vr_controller()`** and expects Tactile/LiveKit (e.g. `room_name` in the service response). That is why “sim only” without `--keyboard` still “expects VR.”
  - With **`--keyboard`**, that VR connection is skipped; use the terminal key map in `keyboard_controller.py`.

- **`--no-cameras`**: Skip all camera processes (no OpenCV / V4L2). Use with **`--keyboard --vis --no-robot`** when testing without cameras plugged in or when indices fail to open. Cannot be combined with **`--record`** (recording requires cameras in recording/hybrid mode).

- **`--leader`**: **Leader-follower** teleop (optional; requires four CAN interfaces and `ENABLE_LEADER_ARMS=true` in `can_config.sh` — see [Optional leader-follower](#optional-leader-follower-four-can-interfaces))
  - Physical **leader** arms drive **follower** arms
  - Leaders use **`leader_left`** / **`leader_right`** after renaming; followers use **`left_piper`** / **`right_piper`**

- **`--policy`**: Enable policy control
  - Uses a trained LeRobot policy to autonomously control the robot
  - Policy path and repo ID are configured in `config.py`
  - Requires robot observations and camera data for inference

#### Robot Connection

- **`--no-robot`**: Disable robot hardware connection
  - Runs in simulation/visualization mode only
  - Useful for testing without physical hardware
  - Can be combined with `--vis` for visualization

- **`--vis`**: Enable visualization
  - Enables Meshcat visualizer to display robot state
  - Shows robot pose and movements in 3D visualization
  - Useful for debugging and monitoring

#### Recording

- **`--record`**: Enable data recording
  - Records robot observations, actions, and camera data
  - Saves data in LeRobot dataset format
  - Requires at least one camera in recording or hybrid mode
  - Data is saved to `data/YYYY-MM-DD_HH-MM-SS/` directory

- **`--resume`**: Resume recording
  - Continues recording to an existing dataset
  - Appends new episodes to the current recording session

- **`--repo-id REPO_ID`**: Specify repository ID for dataset storage
  - Sets the HuggingFace repository ID for dataset storage
  - Default: `"default-piper"`
  - Used when uploading datasets to HuggingFace

#### Logging

- **`--log-level LEVEL`**: Set logging verbosity
  - Options: `debug`, `info`, `warning`, `error`, `critical`
  - Default: `info`
  - Use `debug` for detailed troubleshooting, `warning` for minimal output

### Common Usage Examples

**One follower, keyboard, sim only (no CAN / no cameras):**
```bash
robotserver --no-robot --vis --keyboard --no-cameras
```

**One follower, keyboard, real robot** (CAN up; harmless `right_piper` error if only one bus — see [CAN](#enabling-robot-control-can)):
```bash
robotserver --keyboard
```

**Recording only (no VR):** set cameras to `mode: recording` in config, then:
```bash
robotserver --record
```

**VR teleoperation with recording:** set cameras to `mode: hybrid`, then:
```bash
robotserver --record
```

**Leader–follower with recording** (four CAN links configured):
```bash
robotserver --record --leader
```

**Keyboard control with visualization and no robot** (no cameras / no Tactile VR):
```bash
robotserver --keyboard --vis --no-robot --no-cameras
```
With working cameras, omit `--no-cameras` and set `cam_index` in `config.py`.

**Policy control:**
```bash
robotserver --policy
```

### Common Data Recording Workflow

#### 1. Record a Dataset
```bash
robotserver --record
```
Use **VR + hybrid cameras** or **recording-only cameras** depending on your `mode` settings in `config.py` (see [Enabling Cameras](#enabling-cameras)).

With a leader arm:
```bash
robotserver --record --leader
```

**Keyboard Controls for Recording:**
- **`→` (Right Arrow)**: Navigate through states (reset env → start recording → save recording)
- **`←` (Left Arrow)**: Stop and discard the current recording
- **`Esc`**: Stop the recording session

#### 2. Locate Saved Data

Find the saved data under:
```
data/YYYY-MM-DD_HH-MM-SS/
```

#### 3. Convert to Video Format

Convert image dataset to video format:
```bash
python scripts/convert_images_to_video.py /path/to/dataset
```

#### 4. Upload to HuggingFace

Upload the dataset to HuggingFace Hub:
```bash
python scripts/upload_to_huggingface.py /path/to/dataset_video ETHRC/my-dataset-name
```

#### 5. Inspect Data

Inspect your dataset at [LeRobot Dataset Visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset) by entering your repository ID (e.g., `ETHRC/my-dataset-name`).

### Running a Policy

> **Note**: This has so far only been tested with ACT policies trained using LeRobot.

Follow these steps to run a trained policy:

1. **Train a policy** in LeRobot

2. **Copy the checkpoint** into `policy_checkpoint/` directory

3. **Configure the policy** in `src/piper_teleop/config.py` (`TelegripConfig`): set `policy_path` and `policy_repo_id` to your checkpoint and LeRobot dataset so observation keys (e.g. camera names) match training.

4. **Run the policy**:
   ```bash
   robotserver --policy
   ```

> **Future Note**: In the future, we will likely refactor the robot server to not contain any policy logic and will instead import the robot server as needed in the corresponding policy repos (e.g., in OpenPI or LeRobot).