"""6-DOF replacements for LeRobot's hardcoded-3-DOF HIL-SERL action steps.

LeRobot's stock HIL-SERL action pipeline collapses every teleop command to
translation + gripper at three separate points:

1. ``InterventionActionProcessorStep`` builds the override tensor from only
   ``delta_x/y/z`` (+ gripper), dropping any rotation the teleop emitted.
2. ``MapTensorToDeltaActionDictStep`` reads ``action[0:3]`` (+ gripper) and
   comments ``# TODO: add rotation``.
3. ``MapDeltaActionToRobotActionStep`` hardcodes ``target_wx/wy/wz = 0.0``.

Everything *downstream* of these already supports full 6-DOF: ``EEReferenceAndDelta``
consumes ``target_wx/wy/wz`` and applies them as an **EE-frame** rotation
(``ref[:3,:3] @ R``) on top of a **world-frame** translation (``ref[:3,3] + dp``) --
exactly the ``apply_delta_world_trans_ee_rot`` convention the repo's teleop uses.

So we only need to swap the three steps above. The launcher monkeypatches
``gym_manipulator``'s references to them (no LeRobot source edit):

    import lerobot.rl.gym_manipulator as gm
    from piper_teleop.lerobot_plugin import processors as P
    gm.InterventionActionProcessorStep = P.InterventionAction6DofProcessorStep
    gm.MapTensorToDeltaActionDictStep   = P.MapTensorToDelta6DofActionDictStep
    gm.MapDeltaActionToRobotActionStep  = P.MapDelta6DofActionToRobotActionStep

Action-vector convention (use_gripper=True): ``[dx, dy, dz, wx, wy, wz, gripper]``
(7-dim). The translation/rotation channels are normalized (~[-1, 1]); translation
is scaled to metres downstream by ``end_effector_step_sizes``, rotation is scaled
to radians here by ``rotation_scale``. The gripper channel is the discrete
``{0=close, 1=stay, 2=open}`` class GripperVelocityToJoint expects.
"""

from dataclasses import dataclass, field

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from lerobot.configs import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.processor.delta_action_processor import (
    MapDeltaActionToRobotActionStep,
    MapTensorToDeltaActionDictStep,
)
from lerobot.processor.hil_processor import (
    GRIPPER_KEY,
    TELEOP_ACTION_KEY,
    InterventionActionProcessorStep,
)
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEReferenceAndDelta,
    GripperVelocityToJoint,
)
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.types import EnvTransition, PolicyAction, RobotAction, TransitionKey

# Order of the rotation channels in the flat action vector (after x/y/z).
_ROT_KEYS = ("delta_wx", "delta_wy", "delta_wz")


@dataclass
class InterventionAction6DofProcessorStep(InterventionActionProcessorStep):
    """Override the policy action with the *full 6-DOF* teleop command.

    Mirrors the stock step but (a) carries rotation channels and (b) always emits
    a fixed-width tensor ``[dx,dy,dz,wx,wy,wz,(gripper)]`` so the rest of the
    pipeline -- and the recorded dataset, whose ACTION feature is 7-dim -- sees a
    consistent shape whether or not the human is intervening. The stock control
    loop's neutral action is only 3(+1)-dim, so we pad it here.
    """

    def __call__(self, transition: EnvTransition) -> EnvTransition:
        action = transition.get(TransitionKey.ACTION)
        if not isinstance(action, PolicyAction):
            raise ValueError(f"Action should be a PolicyAction type got {type(action)}")

        info = transition.get(TransitionKey.INFO, {})
        complementary_data = transition.get(TransitionKey.COMPLEMENTARY_DATA, {})
        teleop_action = complementary_data.get(TELEOP_ACTION_KEY, {})
        is_intervention = info.get(TeleopEvents.IS_INTERVENTION, False)
        terminate_episode = info.get(TeleopEvents.TERMINATE_EPISODE, False)
        success = info.get(TeleopEvents.SUCCESS, False)
        rerecord_episode = info.get(TeleopEvents.RERECORD_EPISODE, False)

        new_transition = transition.copy()

        if is_intervention and isinstance(teleop_action, dict):
            # Build the override tensor straight from the gamepad's 6-DOF dict.
            action_list = [
                teleop_action.get("delta_x", 0.0),
                teleop_action.get("delta_y", 0.0),
                teleop_action.get("delta_z", 0.0),
                teleop_action.get("delta_wx", 0.0),
                teleop_action.get("delta_wy", 0.0),
                teleop_action.get("delta_wz", 0.0),
            ]
            if self.use_gripper:
                action_list.append(teleop_action.get(GRIPPER_KEY, 1.0))
            new_action = torch.tensor(action_list, dtype=action.dtype, device=action.device)
        else:
            # Not intervening (or teleop already a tensor): pad whatever we have to
            # the 6-DOF width with zero rotation, keeping translation + gripper.
            flat = action.squeeze(0) if action.dim() > 1 else action
            flat = flat.to(dtype=action.dtype, device=action.device)
            trans = flat[:3]
            zeros_rot = torch.zeros(3, dtype=action.dtype, device=action.device)
            parts = [trans, zeros_rot]
            if self.use_gripper:
                gripper = flat[-1:] if flat.shape[-1] > 3 else torch.tensor(
                    [1.0], dtype=action.dtype, device=action.device
                )
                parts.append(gripper)
            new_action = torch.cat(parts)

        new_transition[TransitionKey.ACTION] = new_action

        new_transition[TransitionKey.DONE] = bool(terminate_episode) or (
            self.terminate_on_success and success
        )
        new_transition[TransitionKey.REWARD] = float(success)

        info = new_transition.get(TransitionKey.INFO, {})
        info[TeleopEvents.IS_INTERVENTION] = is_intervention
        info[TeleopEvents.RERECORD_EPISODE] = rerecord_episode
        info[TeleopEvents.SUCCESS] = success
        new_transition[TransitionKey.INFO] = info

        complementary_data = new_transition.get(TransitionKey.COMPLEMENTARY_DATA, {})
        complementary_data[TELEOP_ACTION_KEY] = new_transition.get(TransitionKey.ACTION)
        new_transition[TransitionKey.COMPLEMENTARY_DATA] = complementary_data

        return new_transition


@dataclass
class MapTensorToDelta6DofActionDictStep(MapTensorToDeltaActionDictStep):
    """Decompose a flat 6-DOF action tensor into a delta-command dict.

    Robust to the action width: translation is always ``[0:3]``; when
    ``use_gripper`` the gripper is the last element and rotation is everything in
    between (so a stock 4-dim neutral action -> zero rotation, a 7-dim 6-DOF
    action -> wx/wy/wz). Missing rotation channels default to 0.
    """

    def action(self, action: PolicyAction) -> RobotAction:
        if not isinstance(action, PolicyAction):
            raise ValueError("Only PolicyAction is supported for this processor")

        if action.dim() > 1:
            action = action.squeeze(0)
        values = action.tolist()

        delta_action = {
            "delta_x": values[0],
            "delta_y": values[1],
            "delta_z": values[2],
        }
        if self.use_gripper:
            rot = values[3:-1]
            delta_action[GRIPPER_KEY] = values[-1] if len(values) > 3 else 1.0
        else:
            rot = values[3:]
        rot = (list(rot) + [0.0, 0.0, 0.0])[:3]
        delta_action["delta_wx"], delta_action["delta_wy"], delta_action["delta_wz"] = rot
        return delta_action

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for axis in ["x", "y", "z", "wx", "wy", "wz"]:
            features[PipelineFeatureType.ACTION][f"delta_{axis}"] = PolicyFeature(
                type=FeatureType.ACTION, shape=(1,)
            )
        if self.use_gripper:
            features[PipelineFeatureType.ACTION][GRIPPER_KEY] = PolicyFeature(
                type=FeatureType.ACTION, shape=(1,)
            )
        return features


@dataclass
class MapDelta6DofActionToRobotActionStep(MapDeltaActionToRobotActionStep):
    """Convert a 6-DOF delta dict into the ``target_*`` robot action.

    Identical to the stock step for translation/gripper, but forwards the rotation
    channels (scaled to radians by ``rotation_scale``) instead of zeroing them, and
    treats rotation input as activity for the ``enabled`` flag. Downstream
    ``EEReferenceAndDelta`` turns ``target_wx/wy/wz`` into an EE-frame rotation.
    """

    # Radians of EE rotation per unit of (normalized) rotation input. Matches the
    # repo's RepoIkEeController rotation step (~5 deg).
    rotation_scale: float = 0.0873

    def action(self, action: RobotAction) -> RobotAction:
        delta_x = action.pop("delta_x")
        delta_y = action.pop("delta_y")
        delta_z = action.pop("delta_z")
        delta_wx = action.pop("delta_wx", 0.0)
        delta_wy = action.pop("delta_wy", 0.0)
        delta_wz = action.pop("delta_wz", 0.0)
        gripper = action.pop(GRIPPER_KEY, 1.0)

        position_magnitude = (delta_x**2 + delta_y**2 + delta_z**2) ** 0.5
        rotation_magnitude = (delta_wx**2 + delta_wy**2 + delta_wz**2) ** 0.5
        enabled = (position_magnitude > self.noise_threshold) or (
            rotation_magnitude > self.noise_threshold
        )

        return {
            "enabled": enabled,
            "target_x": delta_x * self.position_scale,
            "target_y": delta_y * self.position_scale,
            "target_z": delta_z * self.position_scale,
            "target_wx": delta_wx * self.rotation_scale,
            "target_wy": delta_wy * self.rotation_scale,
            "target_wz": delta_wz * self.rotation_scale,
            "gripper_vel": float(gripper),
        }


@dataclass
class PersistentTargetEEReferenceAndDelta(EEReferenceAndDelta):
    """``EEReferenceAndDelta`` that accumulates onto a *persistent absolute target*.

    The stock step recomputes its reference pose from the current robot/IK pose
    every step (``use_ik_solution`` / ``use_latched_reference=False``). Because
    Arm_IK weights orientation far below position (0.1 vs 1.0) and regularises
    joints toward zero, it slightly under-tracks a commanded orientation; feeding
    that back as the next reference makes the error compound, so a pitched gripper
    visibly *leaks back toward level* while translating -- you can't hold a
    top-down grasp. (Verified: pitch to 60 deg then translate -> drifts to ~31 deg.)

    robotserver avoids this by keeping a single absolute target transform that only
    ever changes by explicit teleop deltas; the IK tracks it but never feeds back
    into it. We do the same: initialise the target to the measured pose once, then
    accumulate world-frame translation + EE-frame rotation deltas onto it. No FK
    feedback -> no leak (verified: drift ~0 deg), and because translation leaves the
    orientation block untouched, the world-frame orientation is held constant while
    translating.
    """

    # Absolute target pose; initialised once to the measured pose (first step /
    # after reset) and thereafter moved only by explicit teleop deltas.
    _target: np.ndarray | None = field(default=None, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Joints observation is required for computing robot kinematics")

        # Always use the MEASURED joints for (re)latching -- never the previous IK
        # solution (that is the feedback path that leaks).
        q_raw = np.array(
            [
                float(v)
                for k, v in observation.items()
                if isinstance(k, str)
                and k.endswith(".pos")
                and k.removesuffix(".pos") in self.motor_names
            ],
            dtype=float,
        )
        t_curr = self.kinematics.forward_kinematics(q_raw)

        enabled = bool(action.pop("enabled"))
        tx = float(action.pop("target_x"))
        ty = float(action.pop("target_y"))
        tz = float(action.pop("target_z"))
        wx = float(action.pop("target_wx"))
        wy = float(action.pop("target_wy"))
        wz = float(action.pop("target_wz"))
        gripper_vel = float(action.pop("gripper_vel"))

        # Initialise the target to the actual pose ONCE (first step / after reset),
        # then keep it persistent. We must NOT re-snap to the measured pose on every
        # intervention restart: the arm's tracking error/sag means measured differs
        # from the last commanded pose by a few cm, which downstream EEBoundsAndSafety
        # rejects as an ">5cm EE jump". Initialising once means the target only ever
        # moves by one step (~8 mm) from its own previous value -> never a jump.
        if self._target is None:
            self._target = t_curr.copy()

        if enabled:
            # Translation accumulates in the WORLD frame; rotation in the EE frame.
            # Crucially, when there is no rotation command the orientation block is
            # left untouched, so the (absolute, world-frame) orientation is held
            # constant while translating -- exactly what we want for a steady grasp.
            delta_p = np.array(
                [
                    tx * self.end_effector_step_sizes["x"],
                    ty * self.end_effector_step_sizes["y"],
                    tz * self.end_effector_step_sizes["z"],
                ],
                dtype=float,
            )
            self._target[:3, 3] = self._target[:3, 3] + delta_p
            self._target[:3, :3] = self._target[:3, :3] @ Rotation.from_rotvec([wx, wy, wz]).as_matrix()
        # When not enabled, hold the target frozen (no accumulation).

        desired = self._target
        pos = desired[:3, 3]
        tw = Rotation.from_matrix(desired[:3, :3]).as_rotvec()
        action["ee.x"] = float(pos[0])
        action["ee.y"] = float(pos[1])
        action["ee.z"] = float(pos[2])
        action["ee.wx"] = float(tw[0])
        action["ee.wy"] = float(tw[1])
        action["ee.wz"] = float(tw[2])
        action["ee.gripper_vel"] = gripper_vel

        self._prev_enabled = enabled
        return action

    def reset(self):
        super().reset()
        self._target = None


@dataclass
class BooleanGripperStep(GripperVelocityToJoint):
    """Discrete open/closed gripper -- no velocity integration.

    The stock ``GripperVelocityToJoint`` integrates a (signed) velocity into a
    position with an SO-100 sign convention, which on the Piper both inverts the
    direction and produces interpolated intermediate openings. The operator wants a
    plain boolean: a press latches the gripper fully open or fully closed and it
    stays there.

    Input ``ee.gripper_vel`` is the discrete class from the gamepad
    (``0=close, 1=stay, 2=open``). We map close -> ``clip_min`` (0 m = closed) and
    open -> ``clip_max`` (= ``max_gripper_pos``, fully open), and latch the target
    so a single tap completes even after the button is released.
    """

    _gripper_target: float | None = field(default=None, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        gripper_cmd = round(float(action.pop("ee.gripper_vel")))

        if self._gripper_target is None:
            # Initialise to the current measured gripper so we don't jerk on start.
            observation = self.transition.get(TransitionKey.OBSERVATION)
            q_raw = np.array(
                [float(v) for k, v in observation.items() if isinstance(k, str) and k.endswith(".pos")],
                dtype=float,
            )
            self._gripper_target = float(q_raw[-1])

        if gripper_cmd == 0:  # close
            self._gripper_target = float(self.clip_min)
        elif gripper_cmd == 2:  # open
            self._gripper_target = float(self.clip_max)
        # gripper_cmd == 1 (stay): keep the latched target.

        action["ee.gripper_pos"] = self._gripper_target
        return action

    def reset(self):
        if hasattr(super(), "reset"):
            super().reset()
        self._gripper_target = None
