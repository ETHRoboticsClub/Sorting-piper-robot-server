"""Dense reward-shaping terms added on top of the sparse YOLO grasp reward.

The task reward is sparse (+1 on a confirmed PET/aluminium grasp, 0 otherwise).
These terms nudge *how* the arm gets there without competing with that signal --
each is small relative to the +1 success.

``ActionSmoothnessRewardStep`` rewards consecutive actions that are close together,
through an exponential (RBF) kernel ``w * exp(-||Δa|| / scale)``: full ``w`` when the
action is unchanged, decaying toward 0 as the step-to-step change grows. It reads the
**policy** action (the normalized 6-DOF EE deltas the agent actually chose, the same
vector stored in the replay buffer), not the metric robot command, so the scale is
consistent regardless of ``end_effector_step_sizes``. The gripper channel is excluded
-- gripper chatter is handled separately by ``GripperPenaltyProcessorStep``.

The gripper-change penalty is not here: LeRobot already ships
``GripperPenaltyProcessorStep``, which fires only on an open<->closed transition and
writes ``complementary_info.discrete_penalty`` (consumed by the SAC discrete critic).
Enable it by setting ``processor.gripper.gripper_penalty`` in the config.
"""

from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass, field
from typing import Any

from lerobot.configs import PipelineFeatureType, PolicyFeature
from lerobot.processor import ProcessorStep
from lerobot.types import EnvTransition, TransitionKey

TELEOP_ACTION_KEY = "teleop_action"

# Defaults are deliberately small relative to the +1 sparse success reward, and are a
# per-step bonus -- with discount 0.97 the effective horizon is ~33 steps, so the
# realized contribution is well under the task reward. Override per run via env.
DEFAULT_WEIGHT = float(os.environ.get("PIPER_SMOOTH_WEIGHT", "0.005"))
DEFAULT_SCALE = float(os.environ.get("PIPER_SMOOTH_SCALE", "0.5"))

# Collision penalty. The Piper has no torque sensor, so joint motor current (amps,
# from the high-speed CAN feedback) is the proxy for load. The penalty is the
# per-joint current exceedance summed over joints:
#     penalty = -weight * sum_j max(0, |current_j| - threshold_j)
# Per-joint thresholds because idle load differs a lot by joint (the shoulder holds
# the arm's weight: ~0.65 A vs <0.3 A for the distal joints). Current reads ~0 while
# the motors are disabled, so this only bites under load.
DEFAULT_CURRENT_WEIGHT = float(os.environ.get("PIPER_CURRENT_WEIGHT", "0.05"))
# One value = all joints; or six comma-separated values (joint_0..5), in amps.
DEFAULT_CURRENT_THRESHOLDS = os.environ.get("PIPER_CURRENT_THRESHOLDS", "1.5,1.0,1.0,1.0,1.0,1.0")


def _parse_thresholds(spec: str) -> list[float]:
    parts = [float(x) for x in str(spec).split(",") if x.strip() != ""]
    if not parts:
        parts = [1.0]
    if len(parts) == 1:
        parts = parts * 6
    return (parts + [parts[-1]] * 6)[:6]


@dataclass
class ActionSmoothnessRewardStep(ProcessorStep):
    """Add ``weight * exp(-||Δa_continuous|| / scale)`` to the transition reward.

    ``weight <= 0`` makes the step a no-op. The previous action is per-episode state,
    cleared on ``reset`` so the first step of an episode is never penalised for a jump
    from the last episode's final action.
    """

    weight: float = DEFAULT_WEIGHT
    scale: float = DEFAULT_SCALE
    num_continuous: int = 6

    _prev: Any = field(default=None, init=False, repr=False)

    def __call__(self, transition: EnvTransition) -> EnvTransition:
        if self.weight <= 0.0:
            return transition

        complementary = transition.get(TransitionKey.COMPLEMENTARY_DATA) or {}
        action = complementary.get(TELEOP_ACTION_KEY)
        if action is None:  # e.g. the reset transition, which carries no action
            return transition

        try:
            current = action.detach().flatten()[: self.num_continuous].float()
        except Exception:  # noqa: BLE001 - malformed action must not break the rollout
            return transition

        prev = self._prev
        self._prev = current
        if prev is None or prev.shape != current.shape:
            return transition  # first step of the episode: no delta yet

        distance = float((current - prev).norm().item())
        bonus = self.weight * math.exp(-distance / max(self.scale, 1e-6))

        new_transition = transition.copy()
        new_transition[TransitionKey.REWARD] = float(new_transition.get(TransitionKey.REWARD, 0.0)) + bonus

        # Surface it for logging / Foxglove without affecting control.
        info = dict(new_transition.get(TransitionKey.INFO, {}) or {})
        info["action_smoothness_reward"] = bonus
        new_transition[TransitionKey.INFO] = info
        return new_transition

    def reset(self) -> None:
        self._prev = None

    def get_config(self) -> dict[str, Any]:
        return {"weight": self.weight, "scale": self.scale, "num_continuous": self.num_continuous}

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features


@dataclass
class CollisionCurrentPenaltyStep(ProcessorStep):
    """Penalise motor-current exceedance as a collision proxy (single reward term).

    ``penalty = -weight * sum_j max(0, |current_j| - threshold_j)`` over the six arm
    joints. One number added to the reward -- not split into separate effort/flag
    terms. Per-joint thresholds handle the very different idle loads (the shoulder
    holds the arm's weight). Current reads ~0 while the motors are disabled, so the
    penalty only bites under load.

    ``current_reader`` (``PiperFollower.get_motor_currents``) returns per-joint
    ``current`` plus ``arm_enabled`` and the controller's ``any_collision`` /
    ``any_stall`` flags. All are written to ``info`` (even when empty) so /current and
    the shaping signals stay advertised in Foxglove; the flags are exposed for
    monitoring but do **not** contribute to the reward here.
    """

    current_reader: Any = None
    weight: float = DEFAULT_CURRENT_WEIGHT
    thresholds: list = field(default_factory=lambda: _parse_thresholds(DEFAULT_CURRENT_THRESHOLDS))

    _warned: bool = field(default=False, init=False, repr=False)

    def __call__(self, transition: EnvTransition) -> EnvTransition:
        if self.current_reader is None:
            return transition
        try:
            currents = self.current_reader() or {}
        except Exception as exc:  # noqa: BLE001 - telemetry read must never break the rollout
            currents = {}
            if not self._warned:
                self._warned = True
                logging.getLogger(__name__).warning("[REWARD] current reader failed: %s", exc)
        if not currents and not self._warned:
            self._warned = True
            logging.getLogger(__name__).warning(
                "[REWARD] current reader returned no data -- /current will be empty. "
                "Is the arm connected and are motors reporting?"
            )
        # Do NOT early-return on empty currents: the info fields below are always
        # written so /current and the shaping signals stay advertised in Foxglove
        # (an absent topic shows as "does not exist").

        # Per-joint current exceedance, summed. One threshold per joint.
        violation = 0.0
        for i in range(6):
            current_i = abs(float(currents.get(f"joint_{i}.current", 0.0) or 0.0))
            violation += max(0.0, current_i - self.thresholds[i])

        penalty = -self.weight * violation if self.weight > 0.0 else 0.0

        new_transition = transition.copy()
        if penalty:
            new_transition[TransitionKey.REWARD] = (
                float(new_transition.get(TransitionKey.REWARD, 0.0)) + penalty
            )
        info = dict(new_transition.get(TransitionKey.INFO, {}) or {})
        info["collision_current_penalty"] = penalty
        info["current_violation"] = violation
        info["collision_flag"] = float(bool(currents.get("any_collision") or currents.get("any_stall")))
        info["arm_enabled"] = float(currents.get("arm_enabled") or 0.0)
        for key, value in currents.items():  # per-joint current for monitoring/calibration
            if key.endswith(".current"):
                info[key] = value
        new_transition[TransitionKey.INFO] = info
        return new_transition

    def reset(self) -> None:
        pass

    def get_config(self) -> dict[str, Any]:
        return {"weight": self.weight, "thresholds": list(self.thresholds)}

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features
