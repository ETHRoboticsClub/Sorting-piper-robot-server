# LeRobot Piper plugin (HIL-SERL enablement)

Wraps the AgileX Piper arm and a PS4/PS5 gamepad as LeRobot `Robot` /
`Teleoperator` plugins, so the arm can be driven through the LeRobot stack
(`gym_manipulator`, the HIL-SERL actor/learner) toward reinforcement learning.
This is the bridge from the repo's existing supervised-learning workflow to RL.

## Design: plugin + monkeypatch, no LeRobot source edits

LeRobot's whole RL pipeline is built around the `Robot` and `Teleoperator`
abstractions, plus a fixed EE→joint processor pipeline in `gym_manipulator`. We
expose the Piper through those abstractions and adapt the two places that don't
fit, **without editing the `~/lerobot` clone** — the launcher monkeypatches them
at runtime:

1. **IK** — the stock pipeline does EE↔joint kinematics through a placo
   `RobotKinematics`. We reuse the repo's own pinocchio `Arm_IK` instead (same
   solver `robotserver` uses: tool frame, collision checks, >30° jump-escape).
   `ArmIkKinematics` is a drop-in for `RobotKinematics`, and works in **radians**
   end-to-end (Piper SDK + Arm_IK are radians), so no deg/rad bridge is needed.
2. **6-DOF** — the stock action pipeline is hardcoded to 3-DOF at three points
   (intervention tensor build, tensor→delta map, delta→target map all drop
   rotation). `processors.py` provides 6-DOF replacements. Everything downstream
   (`EEReferenceAndDelta` …) is already 6-DOF ready: it applies the rotation
   target as an **EE-frame** rotation on top of a **world-frame** translation —
   exactly the repo's `apply_delta_world_trans_ee_rot` convention.

`scripts/run_hilserl.py` registers the plugin + applies the four monkeypatches,
then hands off to `gym_manipulator.main`.

## What's here

| File | Purpose |
|------|---------|
| `piper_follower.py` / `config_piper_follower.py` | `PiperFollower` LeRobot `Robot` (joint-space CAN wrapper + cameras + a `bus` shim for `gym_manipulator`), `type="piper_follower"` |
| `piper_gamepad.py` / `config_piper_gamepad.py` | `PiperGamepad6Dof` teleoperator emitting full 6-DOF deltas (`delta_x/y/z`, `delta_wx/wy/wz`) + gripper, `type="piper_gamepad_6dof"` |
| `arm_ik_kinematics.py` | `ArmIkKinematics` — drop-in for LeRobot's `RobotKinematics`, backed by the repo's `Arm_IK` (radians) |
| `processors.py` | 6-DOF replacements for the three hardcoded-3-DOF action steps (`InterventionAction6DofProcessorStep`, `MapTensorToDelta6DofActionDictStep`, `MapDelta6DofActionToRobotActionStep`) |

Motor naming: the 6 arm joints are `joint_0`..`joint_5`; the gripper is named
exactly **`gripper`** (metres). The literal name matters — LeRobot's IK steps
special-case the motor called `gripper` (IK is applied only to the other
motors). `PiperFollower` translates `gripper`↔ the Piper SDK's `joint_6` at the
CAN boundary.

## Environment

All RL/plugin work runs in the conda env **`piper_hilserl_rl`** (Python 3.12):
a clone of `piper_new` (so it has casadi-pinocchio for `Arm_IK`) plus an
**editable** LeRobot clone at `~/lerobot` installed *without* placo:

```bash
pip install -e ~/lerobot[transformers-dep,dataset,grpcio-dep]   # no placo
pip install gym-hil
```

The env's `activate.d` hook puts `$CONDA_PREFIX/lib` first on `LD_LIBRARY_PATH`
(the system libstdc++ lacks `CXXABI_1.3.15`, which scipy/`_highspy` — pulled in
via transformers — needs). It applies automatically on `conda activate`.

(The original `piper_new` env keeps the pip-installed LeRobot for the existing
`robotserver` record/train workflow.)

## Bring-up checks (no robot motion)

```bash
./scripts/restart-can                          # can0 -> left_piper @ 1 Mbit/s
conda activate piper_hilserl_rl
python scripts/find_gamepad_map.py             # identify axis/button indices (if remapping a controller)
python scripts/test_piper_gamepad_teleop.py    # confirm 6-DOF deltas + events (no robot)
python scripts/test_piper_follower.py          # CAN read path through the Robot wrapper (read-only; --nudge for a tiny move)
```

Control scheme (mirrors `robot_server/gamepad_controller.py`):

| Input | Action |
|-------|--------|
| Left stick | X / Y translation (world frame) |
| L2 / R2 triggers | Z up / down |
| Right stick ↔ / ↕ | roll / pitch (EE frame) |
| L1 / R1 (btn 4/5) | yaw − / + |
| Cross / Circle (btn 0/1) | gripper close / open |
| Hold Share (btn 8) | intervention / deadman: move |
| Square / Triangle (btn 2/3) | episode success / failure |

## Teleop through LeRobot (atomic de-risk, no learning)

`gym_manipulator` with `mode: null` runs the full real-robot control loop —
gamepad → 6-DOF EE delta → `Arm_IK` → Piper — with **no SAC, no reward
classifier, no dataset**. The camera-free config isolates the robot wrapper, IK,
and controller:

```bash
./scripts/restart-can
conda activate piper_hilserl_rl
python scripts/run_hilserl.py --config_path config/hilserl_piper_teleop_test.json
```

On start the arm resets to `fixed_reset_joint_positions` (radians); hold Share to
drive; `Ctrl+C` to stop.

> Gripper sign caveat: LeRobot's discrete gripper step is SO-100-signed (joint
> position increases on *close*); the Piper opens as position increases. If Cross
> opens instead of closing, swap `button_close_gripper`/`button_open_gripper` in
> `config_piper_gamepad.py`.

## Recording demos

`config/hilserl_piper_env.json` (`mode: record`, two RealSense cameras) records a
LeRobot dataset of EE-delta actions + camera/state observations for HIL-SERL:

```bash
python scripts/run_hilserl.py --config_path config/hilserl_piper_env.json
```

## Next — HIL-SERL actor/learner

Reward classifier training, then the SAC actor/learner, follow the standard
[HIL-SERL guide](https://huggingface.co/docs/lerobot/en/hilserl). RL action
dimensionality is a separate knob from teleop DOF — you can collect 6-DOF demos
and still train a reduced-DOF policy for sample efficiency.
