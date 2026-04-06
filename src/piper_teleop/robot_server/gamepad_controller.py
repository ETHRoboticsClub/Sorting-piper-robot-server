"""
Gamepad teleop: 6-DOF EE deltas as ArmGoal, always authored for the left arm.

Gripper: Cross = fully closed, Circle = fully open (``gripper_gap_m`` for ``ControlLoop``).
"""

from __future__ import annotations

import logging
import os

import numpy as np
import pygame
from tactile_teleop_sdk.inputs.base import ArmGoal

from piper_teleop.robot_server.core.geometry import (
    apply_delta_world_frame,
    apply_delta_world_trans_ee_rot,
    xyzrpy2transform,
)

logger = logging.getLogger(__name__)

GRIPPER_OPEN_M = 0.07

_BTN_CROSS = 0
_BTN_CIRCLE = 1
_BTN_SQUARE = 2
_BTN_L1 = 4
_BTN_R1 = 5


class GamepadController:
    """
    PS4-style (SDL): left stick + L2/R2 + right stick + L1/R1 + face buttons.

    - Left stick → planar motion (axes 0/1), swapped to world X/Y; L2/R2 → Z.
    - R1/L1 → yaw (R1−L1); right stick ↔ = roll, ↕ = pitch.
    - Translation (X/Y/Z) always in **world** frame.
    - Rotations: **end-effector** frame by default, or **world** frame if ``rotation_world_frame=True``
      (``apply_delta_world_trans_ee_rot`` vs ``apply_delta_world_frame``).
    - Cross = gripper fully closed; Circle = fully open; Square = reset pose.
    """

    def __init__(
        self,
        max_linear_step: float = 0.01,
        angle_step: float = 5.0,
        deadzone: float = 0.12,
        joystick_index: int = 0,
        rotation_world_frame: bool = False,
    ):
        self.max_linear_step = max_linear_step
        self._rotation_world_frame = bool(rotation_world_frame)
        self._lin_step = float(max_linear_step * 0.8)
        self._rot_step_rad = float(np.deg2rad(angle_step))
        self.deadzone = deadzone
        # Triggers are analog; rest position can leave tiny residual pressure on one side.
        self._trigger_deadzone = 0.12
        self._trigger_balance_snip = 0.03  # ignore tiny L2/R2 imbalance when "released"

        self._target_transform: np.ndarray | None = None
        self._origin_transform: np.ndarray | None = None
        self._gripper_gap_m: float = 0.0

        self._initialized = False
        self.joystick: pygame.joystick.Joystick | None = None

        if os.environ.get("DISPLAY") is None and os.environ.get("SDL_VIDEODRIVER") is None:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

        pygame.init()
        pygame.joystick.init()
        n = pygame.joystick.get_count()
        if n <= joystick_index:
            pygame.quit()
            raise RuntimeError(
                f"No gamepad at index {joystick_index} (found {n} joystick(s)). "
                "Plug in a controller or set SDL_JOYSTICK_DEVICE / a different index."
            )
        self.joystick = pygame.joystick.Joystick(joystick_index)
        self.joystick.init()
        logger.info(
            "Gamepad: %s (axes=%s, buttons=%s, hats=%s); rotation=%s",
            self.joystick.get_name(),
            self.joystick.get_numaxes(),
            self.joystick.get_numbuttons(),
            self.joystick.get_numhats(),
            "world" if self._rotation_world_frame else "end-effector",
        )
        self._initialized = True

    @property
    def gripper_gap_m(self) -> float:
        return float(np.clip(self._gripper_gap_m, 0.0, GRIPPER_OPEN_M))

    def _stick_axis(self, index: int) -> float:
        if index < 0 or self.joystick is None or index >= self.joystick.get_numaxes():
            return 0.0
        v = float(self.joystick.get_axis(index))
        if abs(v) < self.deadzone:
            return 0.0
        mag = (abs(v) - self.deadzone) / max(1e-6, 1.0 - self.deadzone)
        return float(np.sign(v) * mag)

    def _trigger_pressure_ps(self, index: int) -> float:
        if index < 0 or self.joystick is None or index >= self.joystick.get_numaxes():
            return 0.0
        v = float(self.joystick.get_axis(index))
        p = float(np.clip((1.0 - v) * 0.5, 0.0, 1.0))
        tdz = self._trigger_deadzone
        if p < tdz:
            return 0.0
        return (p - tdz) / max(1e-6, 1.0 - tdz)

    def _trigger_vertical_m(self) -> float:
        if self.joystick is None or self.joystick.get_numaxes() < 6:
            return 0.0
        l2 = self._trigger_pressure_ps(2)
        r2 = self._trigger_pressure_ps(5)
        # Both fully released → no Z drift from asymmetric trigger noise.
        if l2 <= 0.0 and r2 <= 0.0:
            return 0.0
        d = l2 - r2
        if abs(d) < self._trigger_balance_snip:
            return 0.0
        return d * self._lin_step

    def _button(self, index: int) -> bool:
        if self.joystick is None or index < 0 or index >= self.joystick.get_numbuttons():
            return False
        return bool(self.joystick.get_button(index))

    def _pump(self) -> None:
        pygame.event.pump()

    @staticmethod
    def _scaled_planar(ax_a: float, ax_b: float, cap: float) -> tuple[float, float]:
        va, vb = float(ax_a), float(ax_b)
        mag = float(np.hypot(va, vb))
        if mag < 1e-9:
            return 0.0, 0.0
        scale = min(mag, 1.0) / mag
        return va * scale * cap, vb * scale * cap

    def _delta_transform(self) -> np.ndarray:
        sx, sy = self._scaled_planar(
            -self._stick_axis(0),
            -self._stick_axis(1),
            self._lin_step,
        )
        tx, ty = sy, sx
        tz = self._trigger_vertical_m()
        yaw = (float(self._button(_BTN_R1)) - float(self._button(_BTN_L1))) * self._rot_step_rad
        roll = self._stick_axis(3) * self._rot_step_rad
        pitch = self._stick_axis(4) * self._rot_step_rad
        return xyzrpy2transform(tx, ty, tz, roll, pitch, yaw)

    def get_goal(self, current_left_target_transform: np.ndarray) -> ArmGoal:
        self._pump()
        if self.joystick is None:
            return ArmGoal(arm="left", gripper_closed=True)

        if self._target_transform is None:
            self._target_transform = current_left_target_transform.copy()
            self._origin_transform = current_left_target_transform.copy()

        cross = self._button(_BTN_CROSS)
        circle = self._button(_BTN_CIRCLE)
        if cross and not circle:
            self._gripper_gap_m = 0.0
        elif circle and not cross:
            self._gripper_gap_m = GRIPPER_OPEN_M
        elif cross and circle:
            self._gripper_gap_m = 0.0

        if self._button(_BTN_SQUARE):
            if self._origin_transform is not None and self._target_transform is not None:
                self._target_transform = self._origin_transform.copy()
            self._gripper_gap_m = 0.0
            return ArmGoal(arm="left", gripper_closed=True, reset_to_init=True)

        delta = self._delta_transform()

        if np.any(delta != np.eye(4)) and self._target_transform is not None:
            if self._rotation_world_frame:
                apply_delta_world_frame(self._target_transform, delta)
            else:
                apply_delta_world_trans_ee_rot(self._target_transform, delta)

        arm_goal = ArmGoal(arm="left", gripper_closed=(self.gripper_gap_m <= 1e-6))
        if self._origin_transform is not None and self._target_transform is not None:
            arm_goal.relative_transform = np.linalg.inv(self._origin_transform) @ self._target_transform
        return arm_goal

    def stop(self) -> None:
        if self._initialized:
            pygame.quit()
            self._initialized = False
            self.joystick = None
        logger.info("Gamepad controller stopped.")
