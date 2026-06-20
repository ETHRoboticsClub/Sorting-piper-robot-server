# HIL-SERL on the Piper (through LeRobot)

Extends this repo from imitation learning to **HIL-SERL reinforcement learning**
using [LeRobot](https://huggingface.co/docs/lerobot/en/hilserl). The Piper arm +
PS4/PS5 gamepad are exposed as LeRobot `Robot` / `Teleoperator` plugins and driven
through `lerobot.rl.gym_manipulator`, reusing this repo's own pinocchio **Arm_IK**
(not placo) for end-effector control.

> For the supervised-learning / `robotserver` workflow, see [README.md](README.md).

## Environment

All RL work runs in the conda env **`piper_hilserl_rl`** (Python 3.12): a clone of
`piper_new` (so it has casadi-pinocchio for Arm_IK) + an **editable LeRobot clone at
`~/lerobot`** installed *without* placo, plus `gym-hil` and `ultralytics`. An
`activate.d` hook puts `$CONDA_PREFIX/lib` first on `LD_LIBRARY_PATH` (the system
libstdc++ lacks a symbol scipy needs); it applies automatically on
`conda activate piper_hilserl_rl`.

## Run

```bash
./scripts/restart-can                 # bring up the CAN bus as left_piper
conda activate piper_hilserl_rl

# A) Atomic teleop test — no cameras, no recording, no learning
python scripts/run_hilserl.py --config_path config/hilserl_piper_teleop_test.json

# B) Record reward-labeled demos — 2 RealSense cameras + YOLO grasp reward
python scripts/run_hilserl.py --config_path config/hilserl_piper_env.json
```

`run_hilserl.py` registers the plugins and monkeypatches six `gym_manipulator`
symbols (so **no LeRobot source edit** is needed for the teleop pipeline), then runs
`gym_manipulator.main`. `mode: null` = free teleop loop; `mode: "record"` = writes a
LeRobot dataset.

### Gamepad controls

| Input | Action |
|-------|--------|
| **Share (8)** | **toggle intervention** on/off (take over / release) |
| Left stick | X / Y translation (world frame) |
| R2 / L2 | Z up / down |
| Right stick / L1 / R1 | roll / pitch / yaw (EE frame) |
| Cross / Circle | gripper **close / open** (boolean, latched) |
| Square / Triangle | episode success / fail |

The arm holds when you're not intervening; press Share to drive. Translating holds
the gripper's world-frame orientation constant (so you can hold a top-down grasp).

## How it fits together

```
PiperGamepad6Dof ─▶ [6-DOF action pipeline] ─▶ Arm_IK ─▶ PiperFollower ─▶ CAN
                         (lerobot gym_manipulator, our steps monkeypatched in)
RealSense cams ─▶ observation ─▶ GripperHoldYoloGraspRewardStep ─▶ reward
```

Plugin (`src/piper_teleop/lerobot_plugin/`):

| File | Role |
|------|------|
| `piper_follower.py` | LeRobot `Robot` over the Piper CAN SDK (+ motor-bus shim for `gym_manipulator`; gripper motor named `gripper`) |
| `piper_gamepad.py` | 6-DOF gamepad `Teleoperator` (xyz + rpy + gripper, toggle intervention) |
| `arm_ik_kinematics.py` | `RobotKinematics` drop-in backed by Arm_IK (radians; matches robotserver's `solve_ik`; collision check disabled; solver noise suppressed) |
| `processors.py` | 6-DOF replacements for LeRobot's hardcoded-3-DOF action steps; **persistent-target** EE reference (orientation doesn't leak while translating); **boolean** latched gripper |

The launcher applies six patches: `RobotKinematics`, the three 6-DOF action steps,
`EEReferenceAndDelta` (persistent target), and `GripperVelocityToJoint` (boolean).

### YOLO grasp reward

`GripperHoldYoloGraspRewardStep` (in `~/lerobot`'s `processor/hil_processor.py`):
once the gripper is held closed for `hold_seconds`, it runs the YOLO-cls model
(`cls_train_val_210526_nativeish2/weights/best.pt`, classes
Aluminium/Empty/Other/PET) on the wrist camera. Top-1 in `success_classes`
(PET/Aluminium, conf ≥ threshold) → reward + episode end. **This replaces training a
separate LeRobot reward classifier.** Configured under `processor.yolo_grasp_reward`.

## Config knobs (`config/hilserl_piper_env.json`)

- `processor.reset.fixed_reset_joint_positions` — home pose (radians, 6 joints + gripper)
- `processor.inverse_kinematics.end_effector_bounds` — workspace box (no collision floor; this is the only limit besides reach)
- `processor.inverse_kinematics.end_effector_step_sizes` — metres per teleop step
- `processor.yolo_grasp_reward.{weights_path,success_classes,confidence_threshold,hold_seconds,camera_key}`
- `dataset.{repo_id,task,num_episodes_to_record}`

## Status

**Working / hardware-validated:** 6-DOF teleop through LeRobot, Arm_IK, persistent
top-down orientation, boolean gripper, cameras + dataset recording, YOLO grasp reward
wired into the env pipeline.

**Still missing for full HIL-SERL training:**
1. Record a reward-labeled demo dataset (teleop).
2. SAC actor/learner config (policy/vision encoder, image crop ROI, replay buffer seeded with demos, gRPC).
3. **Apply the six monkeypatches in the SAC actor process** — they currently live only in `run_hilserl.py`, so the actor would otherwise build the stock 3-DOF/placo pipeline.
4. Safety review for autonomous exploration (collision is off; EE bounds + human intervention are the net).
