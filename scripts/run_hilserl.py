#!/usr/bin/env python
"""HIL-SERL launcher for the Piper arm (run inside the ``piper_hilserl_rl`` env).

This is a thin wrapper around LeRobot's ``lerobot.rl.gym_manipulator`` that:

1. imports ``piper_teleop.lerobot_plugin`` so the ``piper_follower`` robot and
   ``piper_gamepad_6dof`` teleop register with draccus (selectable from the JSON
   config by ``type``), and
2. monkeypatches four ``gym_manipulator`` symbols so the RL pipeline uses the
   repo's own IK and full 6-DOF teleop instead of LeRobot's placo + 3-DOF stock:
     - ``RobotKinematics``                -> ``ArmIkKinematics``      (pinocchio Arm_IK, radians)
     - ``InterventionActionProcessorStep``-> 6-DOF intervention override
     - ``MapTensorToDeltaActionDictStep`` -> 6-DOF tensor decode
     - ``MapDeltaActionToRobotActionStep``-> 6-DOF target (forwards rotation)

No LeRobot source is edited; the patches are applied here before ``main`` builds
the processors. Everything downstream (``EEReferenceAndDelta`` etc.) already
supports 6-DOF.

Usage (CAN must be up -- ./scripts/restart-can):

    conda activate piper_hilserl_rl
    python scripts/run_hilserl.py --config_path config/hilserl_piper_env.json

``--mode record`` records a dataset; omit ``mode`` (or set null) for a free
teleop / RL-step loop. Any GymManipulatorConfig field can be overridden on the
CLI, e.g. ``--mode=record --dataset.num_episodes_to_record=10``.
"""

import lerobot.rl.gym_manipulator as gm

# Register the camera choice types used by the JSON config ("intelrealsense",
# "opencv"). These modules only define dataclasses (no camera SDK import), so the
# import is cheap and safe even when no cameras are configured.
import lerobot.cameras.realsense.configuration_realsense  # noqa: F401
import lerobot.cameras.opencv.configuration_opencv  # noqa: F401

# Registers PiperFollowerConfig / PiperGamepad6DofConfig with the draccus choice
# registries (the import side effect is the point).
import piper_teleop.lerobot_plugin  # noqa: F401
from piper_teleop.lerobot_plugin.arm_ik_kinematics import ArmIkKinematics
from piper_teleop.lerobot_plugin import processors as P


def _apply_patches() -> None:
    gm.RobotKinematics = ArmIkKinematics
    gm.InterventionActionProcessorStep = P.InterventionAction6DofProcessorStep
    gm.MapTensorToDeltaActionDictStep = P.MapTensorToDelta6DofActionDictStep
    gm.MapDeltaActionToRobotActionStep = P.MapDelta6DofActionToRobotActionStep
    # Persistent absolute target -> held orientation doesn't leak toward level
    # while translating (lets you hold a top-down grasp). See processors.py.
    gm.EEReferenceAndDelta = P.PersistentTargetEEReferenceAndDelta
    # Boolean (latched) gripper instead of velocity integration.
    gm.GripperVelocityToJoint = P.BooleanGripperStep


if __name__ == "__main__":
    _apply_patches()
    gm.main()
