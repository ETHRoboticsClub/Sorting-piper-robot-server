"""LeRobot plugin package for the AgileX Piper arm + 6-DOF gamepad.

Importing this package registers the Piper robot, the 6-DOF gamepad teleoperator,
and the 6-DOF map step with LeRobot's draccus / processor registries. LeRobot
CLIs discover it via ``--robot.discover_packages_path=piper_teleop.lerobot_plugin``
(and the teleop/processor equivalents); the device classes themselves are then
resolved automatically by ``make_device_from_device_class``.
"""

from .config_piper_follower import PiperFollowerConfig
from .config_piper_gamepad import PiperGamepad6DofConfig
from .piper_follower import PiperFollower
from .piper_gamepad import PiperGamepad6Dof
from .processors import (
    BooleanGripperStep,
    InterventionAction6DofProcessorStep,
    MapDelta6DofActionToRobotActionStep,
    MapTensorToDelta6DofActionDictStep,
    PersistentTargetEEReferenceAndDelta,
)

__all__ = [
    "PiperFollower",
    "PiperFollowerConfig",
    "PiperGamepad6Dof",
    "PiperGamepad6DofConfig",
    "BooleanGripperStep",
    "InterventionAction6DofProcessorStep",
    "MapDelta6DofActionToRobotActionStep",
    "MapTensorToDelta6DofActionDictStep",
    "PersistentTargetEEReferenceAndDelta",
]
