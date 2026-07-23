"""LeRobot ``Robot`` plugin for the AgileX Piper single follower arm.

This is the joint-space wrapper (Milestone 1): it exposes the Piper arm + cameras
through LeRobot's ``Robot`` interface so the rest of the LeRobot stack
(``lerobot-teleoperate``, ``gym_manipulator``, the HIL-SERL actor/learner) can
drive it. End-effector / IK control is layered on top by LeRobot's processor
pipeline (Milestone 2); this class only ever reads/writes the 6 arm joints +
gripper over CAN.

Motor naming: the 6 arm joints are ``joint_0``..``joint_5`` and the gripper is
named exactly ``gripper`` (metres). The literal name ``gripper`` matters --
LeRobot's HIL-SERL kinematic steps (``InverseKinematicsRLStep``, the FK
observation step, ``GripperVelocityToJoint``) special-case the motor named
``gripper`` and apply IK only to the other motors. The underlying Piper SDK
indexes the gripper as ``joint_6``; we translate at the SDK boundary so the rest
of the world sees ``gripper``.
"""

import logging
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from .config_piper_follower import PiperFollowerConfig

logger = logging.getLogger(__name__)

# Canonical motor names exposed to LeRobot: 6 arm joints + gripper (metres).
ARM_DOF = 6
MOTOR_NAMES = [f"joint_{i}" for i in range(ARM_DOF)] + ["gripper"]
DOF = len(MOTOR_NAMES)  # 7
# Piper SDK status/command keys, positionally aligned with MOTOR_NAMES
# (index 6 == "gripper" <-> SDK "joint_6").
_SDK_KEYS = [f"joint_{i}.pos" for i in range(DOF)]


class _PiperMotorBus:
    """Minimal motor-bus shim so PiperFollower satisfies LeRobot's HIL-SERL env.

    ``lerobot.rl.gym_manipulator`` reads ``robot.bus.motors`` and calls
    ``robot.bus.sync_read("Present_Position")`` / ``sync_write("Goal_Position", ...)``
    (it was written for Feetech/Dynamixel motor-bus robots). We back those with the
    Piper CAN SDK. Values are the Piper's native units -- radians for the 6 arm
    joints, metres for the gripper -- i.e. identical to ``get_observation`` /
    ``send_action`` and to Arm_IK (which also works in radians).
    """

    def __init__(self, follower: "PiperFollower"):
        self._follower = follower
        # Ordered like the observation/action vectors the env builds from it.
        self.motors: dict[str, None] = {name: None for name in MOTOR_NAMES}

    def sync_read(self, data_name: str = "Present_Position") -> dict[str, float]:
        status = self._follower._require_sdk().get_status()
        return {name: float(status[sdk_key]) for name, sdk_key in zip(MOTOR_NAMES, _SDK_KEYS)}

    def sync_write(self, data_name: str, values: dict[str, float]) -> None:
        positions = [float(values[name]) for name in MOTOR_NAMES]
        self._follower._require_sdk().set_joint_positions(positions)

    @property
    def is_connected(self) -> bool:
        return self._follower._sdk is not None


class PiperFollower(Robot):
    config_class = PiperFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PiperFollowerConfig):
        super().__init__(config)
        self.config = config
        # The Piper SDK connects eagerly in its constructor, so we defer creating
        # it until connect() to keep __init__ cheap and side-effect free.
        self._sdk: Any | None = None
        self.cameras = make_cameras_from_configs(config.cameras)
        # Motor-bus shim consumed by lerobot's HIL-SERL gym_manipulator.
        self.bus = _PiperMotorBus(self)

    def _require_sdk(self):
        if self._sdk is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        return self._sdk

    # ------------------------------------------------------------------ features
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in MOTOR_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # ------------------------------------------------------------------ lifecycle
    @property
    def is_connected(self) -> bool:
        return self._sdk is not None and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        # Lazy import so the module imports (and registers with draccus) without
        # piper_sdk present, and without pulling the pinocchio IK stack.
        from piper_teleop.robot_server.core.piper_sdk_interface import PiperSDKInterface

        if self._sdk is None:
            self._sdk = PiperSDKInterface(port=self.config.port)
        for cam in self.cameras.values():
            cam.connect()
        logger.info("%s connected on %s", self, self.config.port)

    @property
    def is_calibrated(self) -> bool:
        # Piper homes itself on power-up; no host-side calibration file.
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    # ------------------------------------------------------------------ io
    def get_observation(self) -> dict[str, Any]:
        if self._sdk is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        status = self._sdk.get_status()  # SDK keys: joint_0.pos .. joint_6.pos
        obs: dict[str, Any] = {
            f"{name}.pos": float(status[sdk_key])
            for name, sdk_key in zip(MOTOR_NAMES, _SDK_KEYS)
        }
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()
        return obs

    def get_motor_currents(self) -> dict[str, float]:
        """Per-joint current (A) and effort (torque proxy) for the 6 arm joints.

        Empty dict if unavailable, so callers (e.g. the collision-current reward)
        degrade gracefully rather than crash. See ``PiperSDKInterface.get_motor_currents``.
        """
        if self._sdk is None:
            return {}
        try:
            return self._sdk.get_motor_currents()
        except Exception as exc:  # noqa: BLE001 - telemetry must never break the control loop
            logger.debug("get_motor_currents failed: %s", exc)
            return {}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if self._sdk is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {k: float(v) for k, v in action.items() if k.endswith(".pos")}

        # Fill any missing motor with the present position (never default to 0,
        # which would command a large, unsafe move), and optionally cap the
        # per-step magnitude for safety.
        if self.config.max_relative_target is not None or len(goal_pos) < DOF:
            status = self._sdk.get_status()
            present = {
                f"{name}.pos": float(status[sdk_key])
                for name, sdk_key in zip(MOTOR_NAMES, _SDK_KEYS)
            }
            for name in MOTOR_NAMES:
                goal_pos.setdefault(f"{name}.pos", present[f"{name}.pos"])
            if self.config.max_relative_target is not None:
                goal_present = {k: (goal_pos[k], present[k]) for k in goal_pos}
                goal_pos = ensure_safe_goal_position(goal_present, self.config.max_relative_target)

        positions = [goal_pos[f"{name}.pos"] for name in MOTOR_NAMES]
        self._sdk.set_joint_positions(positions)
        return {f"{name}.pos": positions[i] for i, name in enumerate(MOTOR_NAMES)}

    def disconnect(self) -> None:
        if self._sdk is not None:
            self._sdk.disconnect()
            self._sdk = None
        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()
        logger.info("%s disconnected.", self)
