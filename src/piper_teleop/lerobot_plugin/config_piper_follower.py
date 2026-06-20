"""LeRobot ``RobotConfig`` for the AgileX Piper single follower arm.

Registered under the choice name ``piper_follower`` so it can be selected from
LeRobot CLIs/JSON configs (``--robot.type=piper_follower``). For the choice to be
discoverable, this module must be imported before draccus parses the config,
which LeRobot does via ``--robot.discover_packages_path=piper_teleop.lerobot_plugin``.
"""

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("piper_follower")
@dataclass
class PiperFollowerConfig(RobotConfig):
    # CAN interface / port handed to the Piper SDK. Our bring-up (scripts/restart-can)
    # renames can0 -> left_piper, matching ``robotserver``; a raw "can0" also works.
    port: str = "left_piper"

    # Cameras owned by the robot (LeRobot OpenCV/RealSense camera configs).
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Safety: cap the magnitude of the per-step joint move (radians for the 6 arm
    # joints, metres for the gripper). Float = same cap for every joint, a dict maps
    # "joint_0".."joint_6" individually, None disables capping.
    max_relative_target: float | dict[str, float] | None = None

    # Max gripper opening in metres (Piper hardware ~0.07 m == 70 mm).
    max_gripper_pos: float = 0.07
