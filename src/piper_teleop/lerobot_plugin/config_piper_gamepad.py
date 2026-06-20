"""Config for the 6-DOF Piper gamepad teleoperator.

Registered as ``piper_gamepad_6dof``. Unlike LeRobot's stock ``gamepad`` teleop
(which only emits x/y/z translation deltas), this device also emits rotation
deltas (wx, wy, wz) so the full 6-DOF end-effector pipeline can be driven.
"""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("piper_gamepad_6dof")
@dataclass
class PiperGamepad6DofConfig(TeleoperatorConfig):
    use_gripper: bool = True

    # Ignore small stick noise (normalized stick units, -1..1).
    deadzone: float = 0.12

    # --- PS5 DualSense / SDL 2.28 axis indices (verified via find_gamepad_map.py:
    #     axes 2 and 5 rest at -1.0 == the two triggers) ---
    axis_lx: int = 0  # left stick X   -> -dy
    axis_ly: int = 1  # left stick Y   -> -dx
    axis_l2: int = 2  # left trigger   -> -dz (rests at -1, pressed +1)
    axis_rx: int = 3  # right stick X  ->  wy (roll)
    axis_ry: int = 4  # right stick Y  -> -wx (pitch)
    axis_r2: int = 5  # right trigger  -> +dz

    # --- Button indices (match the repo's verified PS5 mapping in
    #     robot_server/gamepad_controller.py) ---
    button_l1: int = 4   # yaw negative (wz-)
    button_r1: int = 5   # yaw positive (wz+)
    button_close_gripper: int = 0  # Cross
    button_open_gripper: int = 1   # Circle
    # HIL-SERL controls (no repo equivalent); chosen to not clash with motion buttons.
    button_intervention: int = 8   # Share: press to toggle intervention on/off (take over / release)
    button_success: int = 2   # Square -> episode success
    button_failure: int = 3   # Triangle -> episode failure
