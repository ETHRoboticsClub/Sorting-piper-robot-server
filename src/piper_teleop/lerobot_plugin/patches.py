"""Runtime patches that adapt stock LeRobot to this repo's Piper HIL-SERL setup.

Call :func:`apply_lerobot_patches` from *any* entrypoint that builds a
``gym_manipulator`` env or loads a dataset -- the teleop/record launcher
(``scripts/run_hilserl.py``) and, later, the SAC actor/learner. Centralising them
here guarantees every process behaves identically: 6-DOF EE control via Arm_IK,
persistent-target orientation, boolean gripper, and the pyav video backend.

No LeRobot source is edited; we rebind module-level symbols at runtime.
"""

import importlib


def apply_lerobot_patches() -> None:
    """Apply all Piper HIL-SERL patches to the imported LeRobot modules."""
    import lerobot.rl.gym_manipulator as gm

    from . import processors as P
    from .arm_ik_kinematics import ArmIkKinematics

    # EE/IK pipeline: repo's Arm_IK + full 6-DOF + persistent target + boolean gripper.
    gm.RobotKinematics = ArmIkKinematics
    gm.InterventionActionProcessorStep = P.InterventionAction6DofProcessorStep
    gm.MapTensorToDeltaActionDictStep = P.MapTensorToDelta6DofActionDictStep
    gm.MapDeltaActionToRobotActionStep = P.MapDelta6DofActionToRobotActionStep
    gm.EEReferenceAndDelta = P.PersistentTargetEEReferenceAndDelta
    gm.GripperVelocityToJoint = P.BooleanGripperStep

    _patch_6dof_action_space(gm)
    _force_pyav_video_backend()
    _allow_datasetless_rl_config()
    _reset_intervention_each_episode()
    _informative_learner_logging()
    _gamepad_pause(gm)
    _foxglove_episode_logging(gm)
    _actor_reward_breakdown(gm)
    _warm_start_replay_buffer()
    _warm_start_policy_weights()
    _action_smoothness_reward(gm)
    _save_latest_policy()
    _skip_replay_buffer_dump()


def _patch_6dof_action_space(gm) -> None:
    """Make the RL env action space 6-DOF (the stock RobotEnv hardcodes 3).

    ``RobotEnv._setup_spaces`` builds a 3-translation (+gripper) action space; our
    pipeline + demos are full 6-DOF (``dx,dy,dz,wx,wy,wz`` + gripper). Run the
    original to set up the observation space, then override ``action_space`` to 6
    continuous dims in [-1, 1] plus the discrete gripper in [0, 2].
    """
    import gymnasium as gym
    import numpy as np

    _orig_setup_spaces = gm.RobotEnv._setup_spaces

    def _setup_spaces_6dof(self):
        _orig_setup_spaces(self)
        low = [-1.0] * 6
        high = [1.0] * 6
        if getattr(self, "use_gripper", False):
            low.append(0.0)
            high.append(2.0)
        self.action_space = gym.spaces.Box(
            low=np.array(low, dtype=np.float32),
            high=np.array(high, dtype=np.float32),
            shape=(len(low),),
            dtype=np.float32,
        )

    gm.RobotEnv._setup_spaces = _setup_spaces_6dof


def _actor_reward_breakdown(gm) -> None:
    """Log a per-episode reward breakdown to tensorboard, from the actor.

    Tensorboard otherwise shows only the *total* episodic reward, so the dense
    shaping terms are invisible. The breakdown is computed per transition in the
    actor (task vs smoothness vs collision vs gripper penalty), summed per episode,
    and written as separate ``reward/*`` scalars.

    The actor and learner both write to the same tensorboard dir (the launcher sets
    a shared ``PIPER_TB_DIR``), so their event files merge into one run -- the
    learner's ``train/*`` and ``episode/*`` alongside the actor's ``reward/*``.

    ``step_env_and_process_transition`` / ``reset_and_build_transition`` are bound by
    value in ``actor.py``, so the wrappers are rebound there too; both are already
    wrapped by the Foxglove patch, and these compose on top.
    """
    import logging
    import os

    if os.environ.get("PIPER_TB", "1") == "0":
        return
    if getattr(gm.step_env_and_process_transition, "_piper_reward_breakdown", False):
        return

    writer = _tensorboard_writer()
    if writer is None:
        return

    from lerobot.types import TransitionKey

    log = logging.getLogger(__name__)
    acc = {"total": 0.0, "smooth": 0.0, "collision": 0.0, "gripper": 0.0, "peak": 0.0, "n": 0, "step": 0, "ep": 0}

    def _flush():
        if acc["n"] == 0:
            return
        x = acc["step"]
        task = acc["total"] - acc["smooth"] - acc["collision"]  # REWARD = task + smoothness + collision
        writer.add_scalar("reward/task", task, x)
        writer.add_scalar("reward/smoothness", acc["smooth"], x)
        writer.add_scalar("reward/collision_penalty", acc["collision"], x)
        writer.add_scalar("reward/gripper_penalty", acc["gripper"], x)  # discrete_penalty, not in REWARD
        writer.add_scalar("reward/total", acc["total"], x)
        writer.add_scalar("episode/peak_effort_max", acc["peak"], x)
        writer.add_scalar("episode/length", acc["n"], x)
        writer.flush()
        for k in ("total", "smooth", "collision", "gripper", "peak", "n"):
            acc[k] = 0.0

    _orig_step = gm.step_env_and_process_transition
    _orig_reset = gm.reset_and_build_transition

    def step_env_and_process_transition(env, transition, action, env_processor, action_processor):
        new_transition = _orig_step(env, transition, action, env_processor, action_processor)
        try:
            info = new_transition.get(TransitionKey.INFO) or {}
            complementary = new_transition.get(TransitionKey.COMPLEMENTARY_DATA) or {}
            acc["total"] += float(new_transition.get(TransitionKey.REWARD, 0.0) or 0.0)
            acc["smooth"] += float(info.get("action_smoothness_reward") or 0.0)
            acc["collision"] += float(info.get("collision_current_penalty") or 0.0)
            acc["gripper"] += float(complementary.get("discrete_penalty") or 0.0)
            acc["peak"] = max(acc["peak"], float(info.get("peak_effort") or 0.0))
            acc["n"] += 1
            acc["step"] += 1  # matches the actor's per-transition interaction step
        except Exception as exc:  # noqa: BLE001 - metrics must never break the rollout
            log.debug("[TB] reward-breakdown accumulate failed: %s", exc)
        return new_transition

    def reset_and_build_transition(env, env_processor, action_processor):
        try:
            _flush()  # the just-finished episode
        except Exception as exc:  # noqa: BLE001
            log.debug("[TB] reward-breakdown flush failed: %s", exc)
        return _orig_reset(env, env_processor, action_processor)

    step_env_and_process_transition._piper_reward_breakdown = True
    if getattr(_orig_step, "_piper_foxglove", False):  # carry the other step-wrapper's guard
        step_env_and_process_transition._piper_foxglove = True
    reset_and_build_transition._piper_reward_breakdown = True
    if getattr(_orig_reset, "_piper_foxglove", False):
        reset_and_build_transition._piper_foxglove = True

    for modname in ("lerobot.rl.gym_manipulator", "lerobot.rl.actor"):
        try:
            module = importlib.import_module(modname)
        except Exception:  # noqa: BLE001
            continue
        if hasattr(module, "step_env_and_process_transition"):
            module.step_env_and_process_transition = step_env_and_process_transition
        if hasattr(module, "reset_and_build_transition"):
            module.reset_and_build_transition = reset_and_build_transition
    log.info("[TB] per-episode reward breakdown -> reward/{task,smoothness,collision_penalty,gripper_penalty,total}")


def _foxglove_episode_logging(gm) -> None:
    """Record every environment transition to a per-episode MCAP file for Foxglove.

    Hooks the two functions that bracket a rollout: ``step_env_and_process_transition``
    (which returns the fully processed transition -- observation, executed action,
    reward, info) and ``reset_and_build_transition`` (episode boundary).

    ``actor.py`` does ``from .gym_manipulator import step_env_and_process_transition``,
    binding these **by value**, so patching the ``gym_manipulator`` module alone has no
    effect on the actor -- the same trap ``_force_pyav_video_backend`` handles. Rebind
    in every module the names landed in.

    Only runs where an env actually exists (the actor); the learner never calls these.
    """
    import atexit
    import logging
    import os

    if os.environ.get("PIPER_FG", "1") == "0":
        return
    if getattr(gm.step_env_and_process_transition, "_piper_foxglove", False):
        return

    try:
        from .foxglove_logger import FoxgloveEpisodeLogger
    except Exception as exc:  # noqa: BLE001 - mcap is optional
        logging.getLogger(__name__).info("[FOXGLOVE] disabled (%s)", exc)
        return

    from lerobot.types import TransitionKey

    # Created on first use, NOT here. `apply_lerobot_patches()` runs in the learner
    # too, and constructing the logger eagerly would start its live WebSocket server
    # there -- the learner would win the race for the port, the actor would fail to
    # bind and silently fall back to record-only, and a viewer would attach to the
    # learner's server and see nothing. Only the actor ever calls these hooks, so
    # deferring construction puts the server in the process that has the data.
    holder: dict[str, Any] = {"recorder": None}

    def recorder():
        if holder["recorder"] is None:
            holder["recorder"] = FoxgloveEpisodeLogger()
        return holder["recorder"]

    _orig_step = gm.step_env_and_process_transition
    _orig_reset = gm.reset_and_build_transition

    def step_env_and_process_transition(env, transition, action, env_processor, action_processor):
        new_transition = _orig_step(env, transition, action, env_processor, action_processor)
        try:
            complementary = new_transition.get(TransitionKey.COMPLEMENTARY_DATA) or {}
            # The gripper penalty lands in complementary_data, not info; surface it so
            # Foxglove's /shaping shows it next to the other dense terms.
            info = dict(new_transition.get(TransitionKey.INFO) or {})
            if "discrete_penalty" in complementary:
                info.setdefault("discrete_penalty", complementary["discrete_penalty"])
            recorder().log_transition(
                observation=new_transition.get(TransitionKey.OBSERVATION),
                # the action actually executed -- the teleop one during an intervention
                action=complementary.get("teleop_action", action),
                reward=new_transition.get(TransitionKey.REWARD, 0.0),
                info=info,
                done=bool(new_transition.get(TransitionKey.DONE, False))
                or bool(new_transition.get(TransitionKey.TRUNCATED, False)),
            )
        except Exception as exc:  # noqa: BLE001 - never let visualisation break a rollout
            logging.getLogger(__name__).debug("[FOXGLOVE] skipped a transition: %s", exc)
        return new_transition

    def reset_and_build_transition(env, env_processor, action_processor):
        recorder().end_episode()
        result = _orig_reset(env, env_processor, action_processor)
        try:
            recorder().start_episode({"started_at": __import__("time").strftime("%Y-%m-%d %H:%M:%S")})
        except Exception:  # noqa: BLE001
            pass
        return result

    step_env_and_process_transition._piper_foxglove = True
    reset_and_build_transition._piper_foxglove = True

    for modname in ("lerobot.rl.gym_manipulator", "lerobot.rl.actor"):
        try:
            module = importlib.import_module(modname)
        except Exception:  # noqa: BLE001
            continue
        if hasattr(module, "step_env_and_process_transition"):
            module.step_env_and_process_transition = step_env_and_process_transition
        if hasattr(module, "reset_and_build_transition"):
            module.reset_and_build_transition = reset_and_build_transition

    atexit.register(lambda: holder["recorder"] and holder["recorder"].close())


def _gamepad_pause(gm) -> None:
    """Let the Options button freeze the rollout loop (robot + data collection).

    Blocks at the top of ``RobotEnv.step``, before the action reaches the arm, so the
    robot holds its last commanded pose and no transitions are produced. The learner
    keeps training on what it already has, which is harmless -- and lets you reposition
    the workspace or take a break without stopping the run.

    This is safe with respect to episode length: ``TimeLimitProcessorStep`` counts
    *steps*, not seconds, so a pause never truncates the episode you resume into.

    ``RobotEnv`` holds no teleoperator reference, so we read the pause flag off the
    connected gamepad (``PiperGamepad6Dof._active``).
    """
    if getattr(gm.RobotEnv.step, "_piper_pausable", False):
        return

    from .piper_gamepad import PiperGamepad6Dof

    _orig_step = gm.RobotEnv.step

    def step(self, action):
        device = PiperGamepad6Dof._active
        if device is not None:
            device.wait_while_paused()
        return _orig_step(self, action)

    step._piper_pausable = True
    gm.RobotEnv.step = step


def _warm_start_replay_buffer() -> None:
    """Seed a fresh learner's replay buffer from previous runs' recordings.

    Without this, every restart begins with an empty buffer and cannot learn until
    ``online_step_before_learning`` new transitions arrive -- so short iteration
    cycles throw away everything the robot has already done.

    Hooks ``initialize_replay_buffer``, which the learner calls once at startup, and
    fills the buffer it returns. Skipped when resuming (the buffer is restored from
    the checkpoint) and when ``PIPER_WARM_START=0``.
    """
    import logging
    import os

    if os.environ.get("PIPER_WARM_START", "1") == "0":
        logging.getLogger(__name__).info("[WARM START] disabled (PIPER_WARM_START=0) -- fresh buffer")
        return

    import lerobot.rl.learner as _learner

    if getattr(_learner.initialize_replay_buffer, "_piper_warm_start", False):
        return

    _orig_init = _learner.initialize_replay_buffer

    def initialize_replay_buffer(cfg, device: str, storage_device: str):
        buffer = _orig_init(cfg, device, storage_device)
        if getattr(cfg, "resume", False):
            return buffer
        try:
            from .warm_start import warm_start

            features = {k: tuple(v.shape) for k, v in cfg.policy.input_features.items()}
            warm_start(buffer, features)
        except Exception as exc:  # noqa: BLE001 - a bad recording must not stop training
            logging.getLogger(__name__).warning(
                "[WARM START] failed (%s: %s) -- continuing with an empty buffer",
                type(exc).__name__,
                exc,
            )
        return buffer

    initialize_replay_buffer._piper_warm_start = True
    _learner.initialize_replay_buffer = initialize_replay_buffer


def _save_latest_policy() -> None:
    """Persist the freshest policy after every learning step (learner only).

    The stock learner only writes a checkpoint every ``save_freq`` steps, so the
    newest weights on disk -- what policy warm start loads next run -- can lag by up
    to ``save_freq`` optimization steps. This keeps a single, always-current
    ``<output_dir>/checkpoints/latest/pretrained_model`` refreshed each step.

    Only the weights are rewritten per step (~3-5 ms, measured); the config and the
    pre/post processors do not change during training and are written once. `training_step`
    runs only in the learner, so this never touches the actor.

    ``PIPER_SAVE_LATEST_EVERY`` (default 1) throttles it; 0 disables. cfg and the
    processors are captured from the learner functions that receive them, both of
    which run once before the training loop.
    """
    import logging
    import os
    from pathlib import Path

    every = int(os.environ.get("PIPER_SAVE_LATEST_EVERY", "1"))
    if every <= 0:
        logging.getLogger(__name__).info("[CHECKPOINT] latest-policy saving disabled (PIPER_SAVE_LATEST_EVERY=0)")
        return

    import lerobot.rl.learner as _learner
    from lerobot.rl.trainer import RLTrainer

    if getattr(RLTrainer.training_step, "_piper_save_latest", False):
        return

    log = logging.getLogger(__name__)
    state: dict[str, Any] = {"cfg": None, "pre": None, "post": None, "wrote_once": False, "n": 0, "warned": False}

    # Capture cfg from initialize_replay_buffer(cfg, ...) -- always called once at startup.
    _orig_irb = _learner.initialize_replay_buffer

    def initialize_replay_buffer(cfg, *args, **kwargs):
        state["cfg"] = cfg
        return _orig_irb(cfg, *args, **kwargs)

    _learner.initialize_replay_buffer = initialize_replay_buffer

    # Capture the processors from make_pre_post_processors (imported by value into learner).
    _orig_mpp = _learner.make_pre_post_processors

    def make_pre_post_processors(*args, **kwargs):
        pre, post = _orig_mpp(*args, **kwargs)
        state["pre"], state["post"] = pre, post
        return pre, post

    _learner.make_pre_post_processors = make_pre_post_processors

    _orig_step = RLTrainer.training_step

    def training_step(self):
        stats = _orig_step(self)
        state["n"] += 1
        if state["n"] % every == 0:
            try:
                cfg = state["cfg"]
                policy = getattr(self.algorithm, "policy", None)
                if cfg is not None and getattr(cfg, "output_dir", None) and policy is not None:
                    latest = Path(cfg.output_dir) / "checkpoints" / "latest" / "pretrained_model"
                    latest.mkdir(parents=True, exist_ok=True)
                    policy.save_pretrained(latest)
                    if not state["wrote_once"]:  # config + processors are static during training
                        cfg.save_pretrained(latest)
                        if state["pre"] is not None:
                            state["pre"].save_pretrained(latest)
                        if state["post"] is not None:
                            state["post"].save_pretrained(latest)
                        state["wrote_once"] = True
                        log.info("[CHECKPOINT] saving latest policy every %d step(s) -> %s", every, latest)
            except Exception as exc:  # noqa: BLE001 - saving must never kill training
                if not state["warned"]:
                    state["warned"] = True
                    log.warning("[CHECKPOINT] latest-policy save failed (%s: %s)", type(exc).__name__, exc)
        return stats

    training_step._piper_save_latest = True
    # Two patches wrap training_step; carry the other's guard forward so the outermost
    # function reflects both and a re-apply skips both instead of double-wrapping.
    if getattr(_orig_step, "_piper_verbose", False):
        training_step._piper_verbose = True
    RLTrainer.training_step = training_step


def _action_smoothness_reward(gm) -> None:
    """Add the action-smoothness shaping term to the env reward pipeline.

    ``make_processors`` builds the env pipeline that ends with the sparse YOLO grasp
    reward, then ``AddBatchDimensionProcessorStep`` / ``DeviceProcessorStep``. We
    insert the smoothness step just before batching, so it sees the plain (unbatched,
    CPU) transition and its ``complementary_data['teleop_action']`` -- the normalized
    policy action. Only the actor builds this pipeline, so the learner is untouched.

    No-op when the weight is <= 0 (``PIPER_SMOOTH_WEIGHT=0``).
    """
    import logging
    import os

    smooth_weight = float(os.environ.get("PIPER_SMOOTH_WEIGHT", "0.005"))

    if getattr(gm.make_processors, "_piper_smoothness", False):
        return

    from lerobot.processor.batch_processor import AddBatchDimensionProcessorStep

    from .reward_shaping import ActionSmoothnessRewardStep, CollisionCurrentPenaltyStep

    _orig_make_processors = gm.make_processors
    log = logging.getLogger(__name__)

    def make_processors(env, teleop_device, cfg, device="cpu", *args, **kwargs):
        env_processor, action_processor = _orig_make_processors(
            env, teleop_device, cfg, device, *args, **kwargs
        )
        try:
            steps = env_processor.steps
            index = next(
                (i for i, s in enumerate(steps) if isinstance(s, AddBatchDimensionProcessorStep)),
                len(steps),
            )
            new_steps = []
            if smooth_weight > 0.0:
                new_steps.append(ActionSmoothnessRewardStep())

            # Collision-current penalty: bind the reader to the connected robot, so the
            # step pulls per-joint effort each transition. Always inserted (it streams
            # current to info even at weight 0, for threshold calibration); only the
            # penalty term is gated on weight.
            robot = getattr(env, "robot", None) or getattr(getattr(env, "unwrapped", None), "robot", None)
            reader = getattr(robot, "get_motor_currents", None)
            if callable(reader):
                new_steps.append(CollisionCurrentPenaltyStep(current_reader=reader))

            for offset, step in enumerate(new_steps):
                steps.insert(index + offset, step)

            smooth = next((s for s in new_steps if isinstance(s, ActionSmoothnessRewardStep)), None)
            if smooth is not None:
                log.info(
                    "[REWARD] smoothness on: weight=%.4g scale=%.4g (exp kernel on Δaction)",
                    smooth.weight,
                    smooth.scale,
                )
            else:
                log.info("[REWARD] smoothness off (PIPER_SMOOTH_WEIGHT<=0)")
            cur = next((s for s in new_steps if isinstance(s, CollisionCurrentPenaltyStep)), None)
            if cur is not None:
                state = "ON" if cur.weight > 0 else "OFF (streaming current for calibration)"
                log.info(
                    "[REWARD] collision-current penalty %s: weight=%.4g threshold=%.4g effort",
                    state,
                    cur.weight,
                    cur.threshold,
                )
            else:
                log.info("[REWARD] collision-current penalty unavailable (robot exposes no currents)")
        except Exception as exc:  # noqa: BLE001 - shaping must never break env construction
            log.warning("[REWARD] could not add reward shaping (%s: %s)", type(exc).__name__, exc)
        return env_processor, action_processor

    make_processors._piper_smoothness = True
    # actor.py does `from .gym_manipulator import make_processors`, binding by value,
    # so patching only gym_manipulator leaves the actor calling the original -- with no
    # shaping steps. Rebind everywhere it landed (this is the env-building entrypoint;
    # the actor is the process that actually runs it).
    for modname in ("lerobot.rl.gym_manipulator", "lerobot.rl.actor"):
        try:
            module = importlib.import_module(modname)
        except Exception:  # noqa: BLE001
            continue
        if hasattr(module, "make_processors"):
            module.make_processors = make_processors


def _warm_start_policy_weights() -> None:
    """Start from the newest saved policy instead of a random init.

    Reloading the buffer alone still leaves the *policy* random on every restart, so
    the robot flails through the first minutes of each session relearning what it
    already knew. Point ``pretrained_path`` at the newest checkpoint from a previous
    run when the caller has not set one.

    ``learner.py`` and ``actor.py`` both do ``from lerobot.policies import
    make_policy``, binding by value, so the wrapper is rebound in each -- patching
    the factory module alone would do nothing. Both are wrapped deliberately: the
    actor otherwise runs a random policy until the learner's first weight push
    (~50 s).

    Disable with ``PIPER_WARM_START_POLICY=0``. Checkpoints only appear every
    ``save_freq`` (5000) optimization steps, so finding none is normal.
    """
    import logging
    import os

    if os.environ.get("PIPER_WARM_START_POLICY", "1") == "0":
        logging.getLogger(__name__).info(
            "[WARM START] policy warm start disabled (PIPER_WARM_START_POLICY=0) -- random init"
        )
        return

    import lerobot.policies as _policies

    if getattr(_policies.make_policy, "_piper_warm_start", False):
        return

    _orig_make_policy = _policies.make_policy
    log = logging.getLogger(__name__)

    def make_policy(cfg, *args, **kwargs):
        try:
            if getattr(cfg, "pretrained_path", None) is None:
                from .warm_start import find_latest_checkpoint

                checkpoint = find_latest_checkpoint()
                if checkpoint:
                    cfg.pretrained_path = checkpoint
                    log.info("[WARM START] policy from %s", checkpoint)
                else:
                    log.info(
                        "[WARM START] no previous checkpoint found -- random init "
                        "(checkpoints are written every save_freq steps)"
                    )
        except Exception as exc:  # noqa: BLE001 - never block policy construction
            log.warning("[WARM START] policy warm start skipped (%s: %s)", type(exc).__name__, exc)
        return _orig_make_policy(cfg, *args, **kwargs)

    make_policy._piper_warm_start = True
    for modname in ("lerobot.policies", "lerobot.rl.learner", "lerobot.rl.actor"):
        try:
            module = importlib.import_module(modname)
        except Exception:  # noqa: BLE001
            continue
        if hasattr(module, "make_policy"):
            module.make_policy = make_policy


def _skip_replay_buffer_dump() -> None:
    """Don't rewrite the whole replay buffer to disk on every checkpoint.

    ``save_training_checkpoint`` rmtree's and re-exports the entire buffer as a
    LeRobot dataset, inline in the training loop. Measured here: ~8.3 s per 2000
    transitions, so ~62 s and ~320 MB for a full 15k buffer -- roughly 5% overhead at
    ``save_freq: 5000`` but ~23% at 1000, which is what makes frequent checkpoints
    impractical. LeRobot marks it temporary itself ("TODO : temporary save replay
    buffer here, remove later when on the robot").

    It is also now redundant: warm start rebuilds the buffer from the MCAP
    recordings, which are written continuously rather than only at checkpoints.

    The one thing this gives up is ``resume: true``, which restores the buffer from
    that export. Set ``PIPER_SAVE_BUFFER=1`` to keep the old behaviour if you need it.
    """
    import logging
    import os

    if os.environ.get("PIPER_SAVE_BUFFER", "0") == "1":
        return

    from lerobot.rl.buffer import ReplayBuffer

    if getattr(ReplayBuffer.to_lerobot_dataset, "_piper_skipped", False):
        return

    log = logging.getLogger(__name__)
    state = {"warned": False}

    def to_lerobot_dataset(self, *args, **kwargs):
        if not state["warned"]:
            state["warned"] = True
            log.info(
                "[CHECKPOINT] skipping the replay-buffer export (~62 s per full buffer); "
                "warm start rebuilds it from the recordings. PIPER_SAVE_BUFFER=1 to keep it."
            )
        return None

    to_lerobot_dataset._piper_skipped = True
    ReplayBuffer.to_lerobot_dataset = to_lerobot_dataset


_TB_WRITER_CACHE: dict = {}


def _tensorboard_writer():
    """A process-wide ``SummaryWriter``, or None if tensorboard isn't usable.

    LeRobot's RL stack only knows about wandb, so this is our own scalar sink. Cached
    per process so the learner-logging and reward-breakdown patches share one writer
    (and one event file) instead of each opening their own. Runs land in
    ``PIPER_TB_DIR``; the launcher sets the same dir for the learner and the actor so
    their event files merge into one tensorboard run.
    """
    import logging
    import os

    if "writer" in _TB_WRITER_CACHE:
        return _TB_WRITER_CACHE["writer"]

    if os.environ.get("PIPER_TB", "1") == "0":
        _TB_WRITER_CACHE["writer"] = None
        return None
    try:
        from torch.utils.tensorboard import SummaryWriter
    except Exception:  # noqa: BLE001 - tensorboard is optional
        _TB_WRITER_CACHE["writer"] = None
        return None

    logdir = os.environ.get("PIPER_TB_DIR")
    if not logdir:
        import datetime as _dt

        logdir = os.path.join("outputs", "tensorboard", _dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    try:
        writer = SummaryWriter(log_dir=logdir)
    except Exception:  # noqa: BLE001
        _TB_WRITER_CACHE["writer"] = None
        return None
    logging.info("[TB] tensorboard --logdir %s", os.path.dirname(logdir) or logdir)
    _TB_WRITER_CACHE["writer"] = writer
    return writer


def _write_buffer_status(count: int, capacity: int, allocated_bytes: int) -> None:
    """Drop learner buffer stats to a shared file for the actor's Foxglove /buffer.

    The buffer lives in the learner but the Foxglove server runs in the actor, so
    this is the cross-process bridge -- a fixed local path, written atomically so a
    concurrent reader never sees a half-written file. Best-effort.
    """
    import json
    import os

    path = os.environ.get("PIPER_BUFFER_STATUS", "outputs/live_buffer_status.json")
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        payload = {
            "count": int(count),
            "capacity": int(capacity),
            "fraction": (count / capacity) if capacity else 0.0,
            "gb": allocated_bytes / 1e9,
        }
        tmp = f"{path}.tmp"
        with open(tmp, "w") as handle:
            json.dump(payload, handle)
        os.replace(tmp, path)
    except Exception:  # noqa: BLE001 - purely informational
        pass


def _informative_learner_logging() -> None:
    """Replace the learner's per-iteration Hz spam with a useful periodic summary.

    Stock behaviour is nearly unreadable: the optimization-frequency line is logged
    on *every* iteration (several times a second), while everything worth reading --
    the SAC losses from ``stats.to_log_dict()`` and the per-episode reward /
    intervention rate from ``process_interaction_message`` -- is only ever sent to
    the wandb logger. With ``wandb.enable: false`` all of it is computed and thrown
    away, leaving only the Hz line on the console.

    So: drop the per-iteration Hz records, and instead emit
      * a training line every ``PIPER_LEARNER_LOG_S`` seconds (default 10) with the
        optimization step, throughput, replay-buffer fill, and the losses, and
      * one line per finished episode with its reward and intervention rate.
    """
    import logging
    import os
    import time

    interval_s = float(os.environ.get("PIPER_LEARNER_LOG_S", "10"))
    writer = _tensorboard_writer()

    # --- 1. silence the per-iteration Hz line (we report throughput ourselves) ---
    root = logging.getLogger()
    if not any(getattr(f, "_piper_hz_filter", False) for f in root.filters):

        class _DropHzSpam(logging.Filter):
            _piper_hz_filter = True

            def filter(self, record: logging.LogRecord) -> bool:
                return "Optimization frequency loop" not in record.getMessage()

        root.addFilter(_DropHzSpam())

    # --- 2. periodic training summary ---
    from lerobot.rl.trainer import RLTrainer

    if not getattr(RLTrainer.training_step, "_piper_verbose", False):
        _orig_training_step = RLTrainer.training_step

        def training_step(self):
            stats = _orig_training_step(self)

            now = time.time()
            state = getattr(self, "_piper_log_state", None)
            if state is None:
                state = {"last": now, "steps": 0}
                self._piper_log_state = state
            state["steps"] += 1

            elapsed = now - state["last"]
            if elapsed >= interval_s:
                step = getattr(self.algorithm, "optimization_step", 0)
                hz = state["steps"] / elapsed

                parts = [f"step {step}", f"{hz:.1f} Hz"]

                online = getattr(self.data_mixer, "online_buffer", None)
                if online is not None:
                    cap = getattr(online, "capacity", 0) or 0
                    pct = f" {100.0 * len(online) / cap:.0f}%" if cap else ""
                    parts.append(f"buffer {len(online)}/{cap}{pct}")
                    try:  # memory is pre-allocated, so this is the run's real cost
                        from .warm_start import buffer_bytes

                        allocated = buffer_bytes(online)
                        if allocated:
                            parts.append(f"{allocated / 1e9:.2f}GB")
                        _write_buffer_status(len(online), cap, allocated)  # -> actor's Foxglove /buffer
                    except Exception:  # noqa: BLE001
                        pass
                offline = getattr(self.data_mixer, "offline_buffer", None)
                if offline is not None:
                    parts.append(f"demos {len(offline)}")

                try:  # losses last: keys vary by algorithm, so stay defensive
                    logs = stats.to_log_dict()
                    named = [(k, v) for k, v in logs.items() if isinstance(v, (int, float))]
                    named.sort(key=lambda kv: ("loss" not in kv[0].lower(), kv[0]))
                    parts += [f"{k}={v:.4g}" for k, v in named[:5]]

                    if writer is not None:
                        for k, v in named:
                            writer.add_scalar(f"train/{k}", v, step)
                        writer.add_scalar("train/optimization_hz", hz, step)
                        if online is not None:
                            writer.add_scalar("train/replay_buffer_size", len(online), step)
                        writer.flush()
                except Exception:  # noqa: BLE001 - logging must never kill training
                    pass

                logging.info("[LEARNER] " + " | ".join(parts))
                state["last"] = now
                state["steps"] = 0

            return stats

        training_step._piper_verbose = True
        if getattr(_orig_training_step, "_piper_save_latest", False):
            training_step._piper_save_latest = True
        RLTrainer.training_step = training_step

    # --- 3. one line per finished episode ---
    import lerobot.rl.learner as _learner

    if not getattr(_learner.process_interaction_message, "_piper_verbose", False):
        _orig_process = _learner.process_interaction_message

        def process_interaction_message(message, interaction_step_shift: int, wandb_logger=None):
            msg = _orig_process(message, interaction_step_shift, wandb_logger)
            try:
                reward = msg.get("Episodic reward")
                if reward is not None:
                    rate = msg.get("Intervention rate", 0.0) or 0.0
                    took_over = "human" if msg.get("Episode intervention") else "autonomous"
                    logging.info(
                        "[LEARNER] EPISODE done @ interaction %s | reward %.3g | "
                        "intervention %.0f%% (%s)",
                        msg.get("Interaction step", "?"),
                        reward,
                        rate * 100.0,
                        took_over,
                    )
                    if writer is not None:
                        istep = msg.get("Interaction step") or 0
                        writer.add_scalar("episode/reward", reward, istep)
                        writer.add_scalar("episode/intervention_rate", rate, istep)
                        writer.flush()
            except Exception:  # noqa: BLE001 - never break the learner on a log line
                pass
            return msg

        process_interaction_message._piper_verbose = True
        _learner.process_interaction_message = process_interaction_message


def _reset_intervention_each_episode() -> None:
    """Start every episode with the intervention toggle OFF.

    Intervention is a *latched* toggle, so without this it carries across episode
    boundaries: end an episode while driving and the next one silently starts under
    human control, mislabelling autonomous steps as interventions.

    ``AddTeleopEventsAsInfoStep`` is the pipeline step that owns the teleop device,
    and ``reset_and_build_transition`` resets the processor pipelines on every episode
    (``env_processor.reset()`` -> ``step.reset()`` for each step). Hook that step's
    ``reset`` to also clear the teleoperator's latched state. This covers both the SAC
    actor and the teleop/record ``control_loop``, since both reset via that path.
    """
    from lerobot.processor.hil_processor import AddTeleopEventsAsInfoStep

    if getattr(AddTeleopEventsAsInfoStep.reset, "_piper_resets_intervention", False):
        return

    _orig_reset = AddTeleopEventsAsInfoStep.reset

    def _reset(self) -> None:
        _orig_reset(self)
        device_reset = getattr(getattr(self, "teleop_device", None), "reset", None)
        if callable(device_reset):
            device_reset()

    _reset._piper_resets_intervention = True
    AddTeleopEventsAsInfoStep.reset = _reset


def _allow_datasetless_rl_config() -> None:
    """Let ``dataset: null`` validate, for RL runs with no offline demos.

    ``TrainRLServerPipelineConfig`` declares ``dataset`` optional (its own comment
    says "In RL, we don't need an offline dataset") and the learner guards every
    runtime use with ``if cfg.dataset is not None``. But the inherited
    ``TrainPipelineConfig.validate`` dereferences ``self.dataset.repo_id``
    unconditionally, so a dataset-free config dies with ``AttributeError:
    'NoneType' object has no attribute 'repo_id'`` before training starts.

    Temporarily stand in a throwaway ``DatasetConfig`` for the duration of the
    parent's validate, then restore ``None`` so no offline buffer is created.
    """
    from lerobot.configs.train import TrainPipelineConfig

    if getattr(TrainPipelineConfig.validate, "_piper_datasetless_ok", False):
        return

    _orig_validate = TrainPipelineConfig.validate

    def _validate(self):
        if getattr(self, "dataset", None) is not None:
            return _orig_validate(self)

        from lerobot.configs.default import DatasetConfig

        self.dataset = DatasetConfig(repo_id="__none__")
        try:
            return _orig_validate(self)
        finally:
            self.dataset = None

    _validate._piper_datasetless_ok = True
    TrainPipelineConfig.validate = _validate


def _force_pyav_video_backend() -> None:
    """Make ``pyav`` the default video decoder.

    ``torchcodec`` (LeRobot's default when installed) fails to decode some of our
    recorded mp4s ("best video stream unknown"); pyav decodes them fine. The
    resolver ``get_safe_default_video_backend`` is imported by-value into several
    modules (``from ... import name``), so rebind it everywhere it landed.
    """

    def _pyav() -> str:
        return "pyav"

    for modname in (
        "lerobot.utils.import_utils",
        "lerobot.datasets.video_utils",
        "lerobot.datasets.lerobot_dataset",
    ):
        try:
            module = importlib.import_module(modname)
        except Exception:
            continue
        if hasattr(module, "get_safe_default_video_backend"):
            module.get_safe_default_video_backend = _pyav
