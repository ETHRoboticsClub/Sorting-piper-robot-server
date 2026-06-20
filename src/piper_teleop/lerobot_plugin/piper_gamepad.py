"""6-DOF gamepad teleoperator for the Piper arm, as a LeRobot ``Teleoperator``.

LeRobot's stock ``gamepad`` teleop only produces x/y/z translation deltas; the
downstream ``MapDeltaActionToRobotActionStep`` then hardcodes the rotation
targets to zero. This device instead produces a full 6-DOF command
(``delta_x/y/z`` + ``delta_wx/wy/wz``) plus a gripper command, and is meant to be
paired with :class:`MapDelta6DofActionToRobotActionStep` (in ``processors.py``)
which forwards the rotation channels into the EE pipeline.

All deltas are emitted as normalized values in roughly [-1, 1]; the metric
scaling is applied downstream (``end_effector_step_sizes`` for translation, the
map step's ``rotation_step_size`` for rotation).

NOTE: the axis/button indices in :class:`PiperGamepad6DofConfig` are PS5/SDL2
defaults and must be confirmed on the actual controller
(see ``scripts/find_gamepad_map.py``).
"""

import logging
from enum import IntEnum
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.utils import TeleopEvents

from .config_piper_gamepad import PiperGamepad6DofConfig

logger = logging.getLogger(__name__)


class GripperAction(IntEnum):
    CLOSE = 0
    STAY = 1
    OPEN = 2


def _deadzone(value: float, dz: float) -> float:
    # Rescale so motion ramps smoothly from 0 at the deadzone edge (matches the
    # repo's gamepad_controller._stick_axis), instead of jumping to ~dz.
    if abs(value) < dz:
        return 0.0
    mag = (abs(value) - dz) / max(1e-6, 1.0 - dz)
    return mag if value > 0 else -mag


def _trigger_01(value: float) -> float:
    """Map an SDL trigger axis (rest -1, pressed +1) to [0, 1]."""
    return (value + 1.0) / 2.0


class PiperGamepad6Dof(Teleoperator):
    """Pygame-based 6-DOF gamepad reader."""

    config_class = PiperGamepad6DofConfig
    name = "piper_gamepad_6dof"

    def __init__(self, config: PiperGamepad6DofConfig):
        super().__init__(config)
        self.config = config
        self._joystick = None
        self._intervention = False
        self._episode_end: str | None = None

    @property
    def action_features(self) -> dict:
        names = {"delta_x": 0, "delta_y": 1, "delta_z": 2, "delta_wx": 3, "delta_wy": 4, "delta_wz": 5}
        if self.config.use_gripper:
            names["gripper"] = 6
        return {"dtype": "float32", "shape": (len(names),), "names": names}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._joystick is not None

    def connect(self, calibrate: bool = True) -> None:
        import pygame

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad detected. Connect a controller and retry.")
        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()
        logger.info("Initialized 6-DOF gamepad: %s", self._joystick.get_name())

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _pump(self) -> None:
        """Drain the pygame event queue and latch button edges.

        Intervention is a *toggle*: each press of the intervention button flips it
        on/off (press once to take over, again to release) -- no need to hold it.
        """
        import pygame

        c = self.config
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == c.button_intervention:
                    self._intervention = not self._intervention
                elif event.button == c.button_success:
                    self._episode_end = TeleopEvents.SUCCESS
                elif event.button == c.button_failure:
                    self._episode_end = TeleopEvents.FAILURE
            elif event.type == pygame.JOYBUTTONUP:
                if event.button in (c.button_success, c.button_failure):
                    self._episode_end = None

    def _axis(self, idx: int) -> float:
        return _deadzone(self._joystick.get_axis(idx), self.config.deadzone)

    def get_action(self) -> dict[str, Any]:
        if self._joystick is None:
            raise RuntimeError(f"{self} is not connected.")
        self._pump()
        c = self.config

        # Translation (left stick -> X/Y world; R2 raises Z, L2 lowers Z).
        delta_x = -self._axis(c.axis_ly)
        delta_y = -self._axis(c.axis_lx)
        delta_z = _trigger_01(self._joystick.get_axis(c.axis_r2)) - _trigger_01(
            self._joystick.get_axis(c.axis_l2)
        )

        # Rotation (matches repo: right stick horizontal=roll, vertical=pitch; R1-L1=yaw).
        delta_wx = self._axis(c.axis_rx)   # roll
        delta_wy = self._axis(c.axis_ry)   # pitch
        delta_wz = float(self._joystick.get_button(c.button_r1)) - float(
            self._joystick.get_button(c.button_l1)
        )

        action: dict[str, Any] = {
            "delta_x": float(delta_x),
            "delta_y": float(delta_y),
            "delta_z": float(delta_z),
            "delta_wx": float(delta_wx),
            "delta_wy": float(delta_wy),
            "delta_wz": float(delta_wz),
        }
        if self.config.use_gripper:
            close = self._joystick.get_button(c.button_close_gripper)
            open_ = self._joystick.get_button(c.button_open_gripper)
            if close and not open_:
                action["gripper"] = float(GripperAction.CLOSE)
            elif open_ and not close:
                action["gripper"] = float(GripperAction.OPEN)
            else:
                action["gripper"] = float(GripperAction.STAY)
        return action

    def get_teleop_events(self) -> dict[str, Any]:
        """Intervention / episode-control signals consumed by the HIL-SERL loop."""
        if self._joystick is None:
            return {
                TeleopEvents.IS_INTERVENTION: False,
                TeleopEvents.TERMINATE_EPISODE: False,
                TeleopEvents.SUCCESS: False,
                TeleopEvents.RERECORD_EPISODE: False,
            }
        self._pump()
        end = self._episode_end
        return {
            TeleopEvents.IS_INTERVENTION: self._intervention,
            TeleopEvents.TERMINATE_EPISODE: end in (TeleopEvents.SUCCESS, TeleopEvents.FAILURE),
            TeleopEvents.SUCCESS: end == TeleopEvents.SUCCESS,
            TeleopEvents.RERECORD_EPISODE: False,
        }

    def send_feedback(self, feedback: dict) -> None:
        pass

    def disconnect(self) -> None:
        import pygame

        if self._joystick is not None:
            self._joystick.quit()
            self._joystick = None
        if pygame.joystick.get_init():
            pygame.joystick.quit()
        pygame.quit()
