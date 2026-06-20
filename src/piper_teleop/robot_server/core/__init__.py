"""
Core modules for the teleoperation system.
Contains robot interface, kinematics, and visualization components.
"""

from typing import TYPE_CHECKING

__all__ = ["RobotInterface"]

if TYPE_CHECKING:
    from .robot_interface import RobotInterface


def __getattr__(name):
    # Lazily import RobotInterface so that importing lightweight submodules of
    # this package (e.g. ``piper_sdk_interface``) does not pull in the pinocchio
    # / casadi IK stack. Keeps ``from ...core import RobotInterface`` working.
    if name == "RobotInterface":
        from .robot_interface import RobotInterface

        return RobotInterface
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
