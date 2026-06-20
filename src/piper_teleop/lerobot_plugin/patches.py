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
