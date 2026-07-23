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
