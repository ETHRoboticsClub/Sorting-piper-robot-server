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

### Quickstart — one command

```bash
./scripts/start_hilserl.sh
```

Runs preflight (conda env, config, CAN bring-up, **every configured camera actually
attached**, gamepad) and then opens three terminals: learner, actor, and tensorboard.
The actor waits on the learner's gRPC port, so start-up ordering takes care of itself.
No conda activation needed beforehand — each terminal activates the env itself.

```bash
./scripts/start_hilserl.sh --check                      # preflight only, launch nothing
./scripts/start_hilserl.sh --config config/sac_piper.json  # seed from demos instead
./scripts/start_hilserl.sh --no-can --no-tb             # bus already up, no plots
```

The camera check is the one that earns its keep: a configured-but-unplugged RealSense
takes down the entire camera streamer at runtime, and the failure looks like a stall
rather than an error. Preflight turns that into `camera(s) not attached: wrist1=...`
before anything starts. Falls back to `tmux` when no graphical terminal is available.

### Manual (what the launcher does)

```bash
bash scripts/restart_can.sh           # bring up the CAN bus as left_piper
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
| **Options (9)** | **pause / resume** the whole rollout loop |
| Left stick | X / Y translation (world frame) |
| R2 / L2 | Z up / down |
| Right stick / L1 / R1 | roll / pitch / yaw (EE frame) |
| Cross / Circle | gripper **close / open** (boolean, latched) |
| Square / Triangle | episode success / fail |

The arm holds when you're not intervening; press Share to drive. Translating holds
the gripper's world-frame orientation constant (so you can hold a top-down grasp).

**Every episode starts with intervention OFF**, even if you ended the previous one
mid-drive — the toggle is cleared on reset, along with any queued button presses made
during the reset motion. Press Share to take over again.

**Options pauses everything** — the arm holds its last pose and no transitions are
produced, so you can reposition the workspace or take a break mid-run. It blocks
*before* the action reaches the robot, and the episode limit counts steps rather than
seconds, so a pause never eats into the episode you resume into. Pause persists across
episode resets; the learner keeps training on the data it already has.

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
process that builds an env behaves identically. The eleven patches:
`RobotKinematics` (Arm_IK), the three 6-DOF action steps, `EEReferenceAndDelta`
(persistent target), `GripperVelocityToJoint` (boolean), the env **action space**
(stock `RobotEnv` hardcodes 3-DOF → bumped to 6), the **pyav** video backend
(torchcodec fails to decode the recorded mp4s), and **dataset-optional validation**
(`TrainPipelineConfig.validate` dereferences `self.dataset.repo_id` unconditionally,
so `dataset: null` — which the RL config explicitly allows and the learner guards
for at runtime — would otherwise crash before training starts), and an
**intervention reset** on `AddTeleopEventsAsInfoStep.reset` so the latched
intervention toggle starts every episode OFF instead of carrying over, a
**gamepad pause** on `RobotEnv.step` (Options freezes the rollout), and
**informative learner logging** + tensorboard (see below).

### YOLO grasp reward

`GripperHoldYoloGraspRewardStep` (in `~/lerobot`'s `processor/hil_processor.py`,
added by Sebastien): once the gripper is held closed for `hold_seconds`, it runs
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
bash scripts/restart_can.sh
python scripts/run_sac_actor.py --config_path config/sac_piper.json
```

### Running with no demos at all

`config/sac_piper_fresh.json` is the same config with `dataset: null` and
`online_ratio: 1.0` — nothing pre-existing is loaded, so learning comes purely from
online transitions and your interventions. Skip the `crop_demos.py` step entirely:

```bash
python scripts/run_sac_learner.py --config_path config/sac_piper_fresh.json
python scripts/run_sac_actor.py   --config_path config/sac_piper_fresh.json
```

This is the hardest setting — standard HIL-SERL seeds with 20-50 demos, so expect
slower convergence and intervene generously for the first many episodes. Note the
learner still writes checkpoints normally; only the offline buffer is skipped.

**How it learns (important).** There is **no offline pre-training**. The actor starts
acting immediately with a randomly-initialised policy (so it flails at first); the
demos only *seed the replay buffer* and are mixed into every gradient batch
(`online_ratio 0.5`). The learner waits for `online_step_before_learning` *online*
transitions, then learns and acts concurrently. The real early-learning driver is
**you**: toggle Share and tele-operate good grasps generously — those interventions
are high-quality on-policy data. (For a non-random start, BC/ACT-pretrain a policy on
the demos and point `policy.pretrained_path` at it — a worthwhile future step given
the 1429 demos / existing ACT policy.)

### Reading the learner output

**Learning is continuous, not per-episode.** The learner loop is fully decoupled from
episode boundaries: it drains whatever transitions the actor has sent, and as soon as
the buffer holds `online_step_before_learning` (100) samples it takes **one gradient
step per loop iteration**, forever — roughly 3-4 Hz on this box. Each step is
`utd_ratio` (2) critic updates on a batch of 256. Episodes only matter in that they
feed the buffer; nothing waits for one to end.

The one thing that *is* episodic-ish from the robot's point of view:
`policy_parameters_push_frequency: 50` is in **seconds**, so the actor only receives
updated weights every ~50 s. Behaviour on the robot changes in steps, not smoothly.

Stock logging is close to useless — the optimization-frequency line prints several
times a second while every loss and every episode result goes only to wandb, which is
disabled by default. The `_informative_learner_logging` patch drops that spam and
prints instead:

```
[LEARNER] step 1234 | 3.8 Hz | buffer 2456/15000 | loss_actor=-1.203 | loss_critic=0.4123 | ...
[LEARNER] EPISODE done @ interaction 500 | reward 1 | intervention 45% (human)
```

Training lines come every `PIPER_LEARNER_LOG_S` seconds (default 10, e.g.
`PIPER_LEARNER_LOG_S=30 python scripts/run_sac_learner.py ...`); episode lines come one
per finished episode.

### Plots (tensorboard)

The same patch mirrors every scalar into tensorboard — LeRobot's RL stack only supports
wandb, so this is our own sink. No account or login, everything stays local:

`start_hilserl.sh` opens this for you. To run it by hand:

```bash
conda activate piper_hilserl_rl               # tensorboard lives in this env only
tensorboard --logdir outputs/tensorboard      # then open http://localhost:6006
```

(In a fresh terminal the `conda activate` is required — without it you get
`command not found`, since tensorboard is installed into `piper_hilserl_rl`.)

Runs are written to `outputs/tensorboard/<timestamp>/` (override with `PIPER_TB_DIR`,
disable with `PIPER_TB=0`). Scalars: `train/loss_*`, `train/optimization_hz`,
`train/replay_buffer_size` against optimization step, and `episode/reward` +
`episode/intervention_rate` against interaction step.

**The two curves that matter are `episode/reward` and `episode/intervention_rate`** —
is it succeeding, and does it need you less over time. SAC's `loss_critic` is not
monotonic and tells you little about progress.

### Episode replay (Foxglove)

Tensorboard answers "is training progressing over hours". To ask "what actually
happened in *that* episode", every transition is also written to an **MCAP** file —
one per episode, under `outputs/foxglove/<session>/episode_NNNN.mcap` (the path is
printed when the actor starts and again as each file is closed).

```bash
python scripts/view_foxglove.py            # newest episode, in the local Foxglove app
python scripts/view_foxglove.py --list     # everything recorded
python scripts/view_foxglove.py outputs/foxglove/<session>/episode_0007.mcap
python scripts/view_foxglove.py --web      # browser app instead (no desktop install)
```

Uses the **local Foxglove desktop app** when one is installed — it just opens the
file, no server and no network. Otherwise it falls back to app.foxglove.dev, serving
the file from localhost with the CORS and HTTP Range support Foxglove needs.

Recordings are plain files, so **you do not need to run the pipeline to iterate on
the visualisation** — re-open any past episode as often as you like.

**Panels are set up for you.** Foxglove opens a file with whatever layout is
currently selected, which on a fresh install points at topics we never publish (the
stock `/depth`, `/left/image_rect` panels). So `view_foxglove.py` generates a layout
from the topics actually in the episode and installs it as **"HIL-SERL Transitions"**
— cameras and the grasp overlay down the left, reward/intervention plot and the
numeric panels down the right. Add a second camera and it gets its own panel with no
further work.

Select it once from Foxglove's layout menu (top-left); it is remembered from then on.
Installing is additive — existing layouts are never modified — and re-running updates
it in place rather than piling up copies. `--no-layout` skips it. If a newly
installed layout does not appear in the menu, restart Foxglove: it reads the local
layout store at startup.

Everything lands on one scrubbable timeline:

| Topic | Contents |
|-------|----------|
| `/camera/<name>` | one image stream per camera in the observation |
| `/grasp/overlay` | wrist view with the YOLO verdict burned in (green = success) |
| `/state` | `joint_0..N` + `gripper` |
| `/action` | executed `dx,dy,dz,wx,wy,wz,gripper` — the *teleop* action during an intervention |
| `/reward` | step reward + running episode return |
| `/control` | `is_intervention` (plot `value` for a policy-vs-human band) |
| `/grasp` | YOLO class / confidence / success, when the check ran |

Only the cameras actually in the observation appear — with the default wrist-only
config that is `wrist1` alone; add a camera to `env.robot.cameras` and it shows up
here too.

**Overhead.** The control loop only pays a dict copy and a queue put; JPEG encoding
and file I/O run on a background thread behind a bounded queue that *drops* frames
when full, so a slow disk can never pace the robot. Logging is deliberately lossy
under pressure rather than applying backpressure to the control loop. Tune with
`PIPER_FG_EVERY=N` (log every Nth step), `PIPER_FG_QUALITY` (JPEG quality, default
80), `PIPER_FG_MAXQ` (queue depth); `PIPER_FG=0` disables it entirely.

`view_foxglove.py` serves the file to app.foxglove.dev over localhost, implementing
the CORS and HTTP Range support Foxglove needs and `http.server` lacks. Keep it
running while you scrub.

(wandb also works — set `wandb.enable: true` — but viewing needs an account: offline
runs have no local viewer, only `wandb sync` to wandb.ai or a self-hosted server.)

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
