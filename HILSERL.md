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

All patches live in one place — `patches.py`'s **`apply_lerobot_patches()`** — and
every entrypoint (teleop/record, and the SAC learner + actor) calls it, so every
process that builds an env behaves identically. The seven patches:
`RobotKinematics` (Arm_IK), the three 6-DOF action steps, `EEReferenceAndDelta`
(persistent target), `GripperVelocityToJoint` (boolean), the env **action space**
(stock `RobotEnv` hardcodes 3-DOF → bumped to 6), and the **pyav** video backend
(torchcodec fails to decode the recorded mp4s).

### YOLO grasp reward

`GripperHoldYoloGraspRewardStep` (in `~/lerobot`'s `processor/hil_processor.py`,
added by a colleague): once the gripper is held closed for `hold_seconds`, it runs
the YOLO-cls model (`cls_train_val_210526_nativeish2/weights/best.pt`, classes
Aluminium/Empty/Other/PET) on the wrist camera. Top-1 in `success_classes`
(PET/Aluminium, conf ≥ threshold) → reward + episode end. **This replaces training a
separate LeRobot reward classifier.** Configured under `processor.yolo_grasp_reward`.

## Reusing existing (old-stack) demos

The old `robotserver` datasets record **absolute joint targets**; HIL-SERL wants
**EE-delta** actions. Two offline tools bridge that (both reuse Arm_IK / the YOLO
step, so offline and online stay consistent):

```bash
# 1. joint targets -> EE-delta action space (FK via Arm_IK; videos copied as-is)
python scripts/convert_joint_demos_to_ee.py SRC ~/..._eedelta

# 2. add sparse next.reward/next.done by replaying the live YOLO grasp logic;
#    success episodes are trimmed at the grasp, failed grasps kept as negatives
python scripts/label_rewards_yolo.py ~/..._eedelta ~/..._eedelta_labeled
```

## Training (SAC)

LeRobot HIL-SERL is a distributed SAC: a **learner** (trains on streamed
transitions + the offline demo buffer, serves weights over gRPC) and an **actor**
(runs the policy on the robot, collects experience, takes human interventions).
Both use one config — `config/sac_piper.json` (gaussian-actor policy, 6 continuous
EE-delta + discrete gripper, wrist 128² + 7-joint state, SAC, our env + YOLO
reward, demos seeding the offline buffer).

```bash
# 0. one-time: resize the labeled demos' wrist cam to 128x128 (fast ffmpeg transcode,
#    ~2 min). A wrist cam is already framed on the workspace, so --no-crop (resize
#    only) is fine; drop it to drag a tighter ROI box and paste the params into the
#    config's image_preprocessing.crop_params_dict.
python scripts/crop_demos.py \
  --root ~/..._eedelta_labeled --new-root ~/..._eedelta_labeled_crop128 --no-crop

# 1. learner (no robot needed; validates the policy build + demo seeding)
python scripts/run_sac_learner.py --config_path config/sac_piper.json

# 2. actor, separate terminal, on the robot
./scripts/restart-can
python scripts/run_sac_actor.py --config_path config/sac_piper.json
```

**How it learns (important).** There is **no offline pre-training**. The actor starts
acting immediately with a randomly-initialised policy (so it flails at first); the
demos only *seed the replay buffer* and are mixed into every gradient batch
(`online_ratio 0.5`). The learner waits for `online_step_before_learning` *online*
transitions, then learns and acts concurrently. The real early-learning driver is
**you**: toggle Share and tele-operate good grasps generously — those interventions
are high-quality on-policy data. (For a non-random start, BC/ACT-pretrain a policy on
the demos and point `policy.pretrained_path` at it — a worthwhile future step given
the 1429 demos / existing ACT policy.)

**Demos need `complementary_info.discrete_penalty`.** The actor attaches it to every
online transition and the SAC discrete-critic loss adds it to the reward, so the
offline demos must carry the same (zero) column or the mixed batch mismatches.
`label_rewards_yolo.py` writes it; older datasets need it added.

**Memory matters.** The replay buffers pre-allocate `capacity × frame` up front, and
a float32 128² frame is ~0.4 MB (state + next). On a 12 GB GPU / 31 GB box we keep
buffers in RAM (`policy.storage_device: cpu`), cap them small (`offline 10k /
online 15k`), and **seed from a subset of demos** (`dataset.episodes: [0..49]` =
~9 k frames, the standard HIL-SERL 20-50 demo seed) — the full 195 k frames won't
fit. To use more demos: grow the episode list + capacities (watch RAM) or switch to
uint8 image storage.

Needs a CUDA GPU (`policy.device`) and the `hilserl` extra (grpcio). Note the actor
logs a low **policy FPS** (the per-step Arm_IK solve dominates) — drop `env.fps` or
speed up the IK if the loop runs far behind real time.

## Config knobs

`config/hilserl_piper_env.json` (teleop/record) and `config/sac_piper.json` (RL):

- `processor.reset.fixed_reset_joint_positions` — home pose (radians, 6 joints + gripper)
- `processor.inverse_kinematics.end_effector_bounds` — workspace box (no collision floor; the only limit besides reach)
- `processor.inverse_kinematics.end_effector_step_sizes` — metres per teleop step
- `processor.yolo_grasp_reward.{weights_path,success_classes,confidence_threshold,hold_seconds,camera_key}`
- `processor.image_preprocessing.{crop_params_dict,resize_size}` — must match what `crop_demos.py` produced
- SAC: `policy.{input_features,output_features,num_discrete_actions,device}`, `algorithm.*`, `dataset.root` (the crop128 demos)

## Status

**Working / hardware-validated:** 6-DOF teleop through LeRobot, Arm_IK, persistent
top-down orientation, boolean gripper, cameras + dataset recording, YOLO grasp
reward (live + offline labeling), old-demo → EE-delta conversion + reward labeling.

**SAC: learner + actor run on hardware; first training pass works.** The learner
builds the policy (gaussian-actor + resnet10, ~530 K params), seeds the offline
buffer, and trains on the online+offline mix without crashing; the actor runs the
policy on the robot, resets every episode, and streams transitions. Remaining: a real
training session with heavy early interventions, then tuning — speed up the control
loop (low policy FPS), BC/ACT warm-start instead of a random init, reward/crop/step
tuning, more demos, and a safety review for autonomous exploration (collision is off;
EE bounds + human intervention are the net).
