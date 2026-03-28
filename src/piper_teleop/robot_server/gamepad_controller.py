"""
Gamepad teleop: 6-DOF EE deltas as ArmGoal, always authored for the left arm.

The control loop deep-copies that goal to the right arm. Gripper uses an incremental finger opening
(see ``gripper_gap_m``) applied in ``ControlLoop.update_robot``.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass

import numpy as np
import pygame
from tactile_teleop_sdk.inputs.base import ArmGoal

from piper_teleop.robot_server.core.geometry import xyzrpy2transform

logger = logging.getLogger(__name__)

# Matches control_loop finger command when "fully open" (toggle mode).
GRIPPER_OPEN_M = 0.07

# PlayStation-style button indices (SDL / pygame).
_BTN_CROSS = 0   # X — hold to open gripper (incremental)
_BTN_CIRCLE = 1  # O — hold to close gripper (incremental)
_BTN_SQUARE = 2  # hold to reset EE pose


@dataclass
class _GamepadState:
    reset_to_init: bool = False


class GamepadController:
    """
    Layout (PS names; Xbox is usually the same diamond with A≈Cross, B≈Circle):

    - **Left stick**: forward/back → ±tool **X**; left/right → ±tool **Y**
    - **R2**: EE up (+Z); **L2**: EE down (−Z) (axis indices vary: we **auto-detect** at init)
    - **Right stick**: **yaw** (horizontal), **pitch** (vertical) — **not** the same axes as L2/R2
    - **L1 / R1**: **roll** − / + while held
    - **Cross (X)**: hold — open gripper a little each frame (not one-shot full open)
    - **Circle (O)**: hold — close gripper a little each frame (not one-shot full closed)
    - **Square**: hold — reset EE to pose from when teleop started (gripper closes to 0 gap)

    ``gripper_gap_m`` is 0 = closed … ``GRIPPER_OPEN_M`` = max spread; copied to both arms in the loop.

    **Speed:** One linear cap ``max_linear_step`` (metres per frame at full stick or full trigger).
    Translation and tool-Z scale with **how far** you push sticks/triggers. Rotation uses the same
    “feel” via ``max_angular_step = max_linear_step * angular_scale_rad_per_m`` (radians per frame
    at full right-stick deflection). Diagonal sticks are **normalized** so diagonals are not faster
    than cardinals. L1/R1 roll uses the same angular cap while held (on/off).
    """

    def __init__(
        self,
        max_linear_step: float = 0.01,
        angular_scale_rad_per_m: float = 2.5,
        deadzone: float = 0.15,
        joystick_index: int = 0,
        gripper_inc_m: float | None = None,
    ):
        self.max_linear_step = max_linear_step
        self.max_angular_step = float(max_linear_step * angular_scale_rad_per_m)
        self.deadzone = deadzone
        self._trigger_deadzone = 0.12
        self._gripper_inc_m = (
            gripper_inc_m
            if gripper_inc_m is not None
            else max(0.0005, float(max_linear_step) * 0.25)
        )

        self._state = _GamepadState()
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
        name = self.joystick.get_name()
        axes = self.joystick.get_numaxes()
        hats = self.joystick.get_numhats()
        buttons = self.joystick.get_numbuttons()
        logger.info("Gamepad: %s (axes=%s, buttons=%s, hats=%s)", name, axes, buttons, hats)
        self._idx_l2 = 4
        self._idx_r2 = 5
        self._idx_rs_x = 2
        self._idx_rs_y = 3
        self._discover_axis_layout(name)
        self._initialized = True

    def _discover_axis_layout(self, name: str) -> None:
        """
        SDL maps differ: PS4/DualSense on Linux often has **L2/R2 on axes 2–3** (rest −1) and
        **right stick on 4–5**; Xbox-style often uses **right stick 2–3** and **triggers 4–5**.
        Using the wrong pairing makes R2 and right-stick vertical both drive Z.
        """
        if self.joystick is None or self.joystick.get_numaxes() < 6:
            logger.info(
                "Gamepad axes: <6 axes — using RS on 2,3, L2/R2 on 4,5 if present; Z may be unavailable."
            )
            return

        self._pump()
        a = [float(self.joystick.get_axis(i)) for i in range(6)]
        name_l = name.lower()
        ps_family = any(
            k in name_l
            for k in (
                "sony",
                "wireless controller",
                "dualshock",
                "dualsense",
                "playstation",
                "ps4",
                "ps5",
            )
        )

        # At rest: triggers usually −1, sticks ~0.
        trig23 = a[2] < -0.35 and a[3] < -0.35
        trig45 = a[4] < -0.35 and a[5] < -0.35
        stick23 = abs(a[2]) < 0.45 and abs(a[3]) < 0.45
        stick45 = abs(a[4]) < 0.45 and abs(a[5]) < 0.45

        if trig23 and stick45:
            self._idx_l2, self._idx_r2, self._idx_rs_x, self._idx_rs_y = 2, 3, 4, 5
            logger.info("Gamepad axes: L2/R2 → 2,3; right stick → 4,5 (detected at rest)")
        elif trig45 and stick23:
            self._idx_l2, self._idx_r2, self._idx_rs_x, self._idx_rs_y = 4, 5, 2, 3
            logger.info("Gamepad axes: right stick → 2,3; L2/R2 → 4,5 (detected at rest)")
        elif ps_family:
            self._idx_l2, self._idx_r2, self._idx_rs_x, self._idx_rs_y = 2, 3, 4, 5
            logger.info(
                "Gamepad axes: PlayStation-style default — L2/R2 → 2,3; right stick → 4,5 "
                "(rest pose was ambiguous; release sticks/triggers and restart if wrong)"
            )
        else:
            logger.info(
                "Gamepad axes: default right stick → 2,3; L2/R2 → 4,5 "
                "(ambiguous rest pose; use a PlayStation controller name or neutral pose at startup)"
            )

    @property
    def gripper_gap_m(self) -> float:
        """Finger opening in metres for ``ControlLoop`` (0 = closed)."""
        return float(np.clip(self._gripper_gap_m, 0.0, GRIPPER_OPEN_M))

    def _stick_axis(self, index: int) -> float:
        """Analog stick: rest ~0. Released triggers on this index read ~−1 → no stick input."""
        if index < 0 or self.joystick is None or index >= self.joystick.get_numaxes():
            return 0.0
        v = float(self.joystick.get_axis(index))
        if v <= -0.9:
            return 0.0
        if abs(v) < self.deadzone:
            return 0.0
        mag = (abs(v) - self.deadzone) / max(1e-6, 1.0 - self.deadzone)
        return float(np.sign(v) * mag)

    def _trigger_pressure(self, index: int) -> float:
        """Normalize trigger to [0, 1] pressed. Supports −1..1 (PS) and 0..1 (Xbox-style)."""
        if index < 0 or self.joystick is None or index >= self.joystick.get_numaxes():
            return 0.0
        v = float(self.joystick.get_axis(index))
        if v >= -0.2:
            p = float(np.clip(v, 0.0, 1.0))
        else:
            p = float(np.clip((v + 1.0) * 0.5, 0.0, 1.0))
        tdz = self._trigger_deadzone
        if p < tdz:
            return 0.0
        return (p - tdz) / max(1e-6, 1.0 - tdz)

    def _trigger_vertical_m(self) -> float:
        """R2 up (+Z), L2 down (−Z); uses layout indices from _discover_axis_layout."""
        if self.joystick is None or self.joystick.get_numaxes() < 6:
            return 0.0
        r2 = self._trigger_pressure(self._idx_r2)
        l2 = self._trigger_pressure(self._idx_l2)
        return (r2 - l2) * self.max_linear_step

    def _button(self, index: int) -> bool:
        if self.joystick is None or index < 0 or index >= self.joystick.get_numbuttons():
            return False
        return bool(self.joystick.get_button(index))

    def _pump(self) -> None:
        pygame.event.pump()

    @staticmethod
    def _scaled_planar(ax_a: float, ax_b: float, cap: float) -> tuple[float, float]:
        """Unit direction × min(||v||, 1) so diagonals are not faster than cardinals; scales with cap."""
        va = float(ax_a)
        vb = float(ax_b)
        mag = float(np.hypot(va, vb))
        if mag < 1e-9:
            return 0.0, 0.0
        scale = min(mag, 1.0) / mag
        return va * scale * cap, vb * scale * cap

    def _delta_transform(self, roll_sign: float) -> np.ndarray:
        # Left stick: forward/back → X, left/right → Y; speed ∝ tilt, one shared linear cap.
        fx = -self._stick_axis(1)
        lr = -self._stick_axis(0)
        tx, ty = self._scaled_planar(fx, lr, self.max_linear_step)
        tz = self._trigger_vertical_m()
        # Right stick: yaw / pitch (indices from layout — never L2/R2 axes).
        raw_yaw = self._stick_axis(self._idx_rs_x)
        raw_pitch = -self._stick_axis(self._idx_rs_y)
        yaw, pitch = self._scaled_planar(raw_yaw, raw_pitch, self.max_angular_step)
        roll = roll_sign * self.max_angular_step
        return xyzrpy2transform(tx, ty, tz, roll, pitch, yaw)

    def get_goal(self, current_left_target_transform: np.ndarray) -> ArmGoal:
        self._pump()
        if self.joystick is None:
            return ArmGoal(arm="left", gripper_closed=True)

        if self._target_transform is None:
            self._target_transform = current_left_target_transform.copy()
            self._origin_transform = current_left_target_transform.copy()

        if self._button(_BTN_CROSS):
            self._gripper_gap_m = min(GRIPPER_OPEN_M, self._gripper_gap_m + self._gripper_inc_m)
        if self._button(_BTN_CIRCLE):
            self._gripper_gap_m = max(0.0, self._gripper_gap_m - self._gripper_inc_m)

        self._state.reset_to_init = self._button(_BTN_SQUARE)

        if self._state.reset_to_init:
            if self._origin_transform is not None and self._target_transform is not None:
                self._target_transform = self._origin_transform.copy()
            self._gripper_gap_m = 0.0
            return ArmGoal(arm="left", gripper_closed=True, reset_to_init=True)

        roll_sign = float(self._button(4)) - float(self._button(5))
        delta = self._delta_transform(roll_sign)

        if np.any(delta != np.eye(4)) and self._target_transform is not None:
            self._target_transform = self._target_transform @ delta

        arm_goal = ArmGoal(arm="left", gripper_closed=(self.gripper_gap_m < 0.003))
        if self._origin_transform is not None and self._target_transform is not None:
            arm_goal.relative_transform = np.linalg.inv(self._origin_transform) @ self._target_transform
        return arm_goal

    def stop(self) -> None:
        if self._initialized:
            pygame.quit()
            self._initialized = False
            self.joystick = None
        logger.info("Gamepad controller stopped.")
