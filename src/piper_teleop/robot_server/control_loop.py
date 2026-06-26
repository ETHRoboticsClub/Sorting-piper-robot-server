import asyncio
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import torch
import yaml

from piper_teleop.config import TelegripConfig
from piper_teleop.robot_server.arm_goal import ArmGoal
from piper_teleop.robot_server.camera import SharedCameraData
from piper_teleop.robot_server.keyboard_controller import KeyboardController
from piper_teleop.utils import get_absolute_path

from .core.geometry import xyzrpy2transform
from .core.robot_interface import RobotInterface, arm_angles_to_action_dict
from .recorder import Recorder
from .robot_leader import PiperLeader

logger = logging.getLogger(__name__)

# Prismatic finger command when gripper is "open" in toggle mode (matches gamepad max gap).
GRIPPER_FINGER_OPEN_M = 0.07


def precise_sleep(duration_s: float) -> None:
    """Best-effort sleep helper without requiring lerobot."""
    if duration_s <= 0.0:
        return
    time.sleep(duration_s)


@dataclass
class ArmState:
    arm_name: str
    initial_transform: np.ndarray = field(
        default_factory=lambda: xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
    )
    origin_transform: np.ndarray = field(
        default_factory=lambda: xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
    )
    target_transform: np.ndarray | None = None
    deposit_transform: np.ndarray | None = None
    secondary_deposit_transform: np.ndarray | None = None
    gripper_closed: bool = True
    # Gamepad: incremental opening in metres; None = use gripper_closed / GRIPPER_FINGER_OPEN_M
    gripper_gap_m: float | None = None


class ControlLoop:
    """Control loop for the teleoperation system."""

    def __init__(
        self,
        config: TelegripConfig,
        shared_data: SharedCameraData,  #
        robot_interface: RobotInterface | None = None,
    ):
        self.config = config
        self.robot_interface = robot_interface or RobotInterface(config)
        self.robot_enabled = config.enable_robot
        self.use_keyboard = config.enable_keyboard
        self.use_gamepad = config.enable_gamepad
        self.use_leader = config.use_leader
        self.use_policy = config.use_policy
        if self.use_keyboard:
            self.keyboard_controller = KeyboardController()
        if self.use_gamepad:
            try:
                from piper_teleop.robot_server.gamepad_controller import GamepadController
            except ModuleNotFoundError as e:
                if getattr(e, "name", None) == "pygame":
                    raise RuntimeError(
                        "Gamepad mode needs pygame. Install with: pip install 'pygame>=2.5'"
                    ) from e
                raise
            # Translation: pos_step; rotation: angle_step (deg/frame at full stick).
            self.gamepad_controller = GamepadController(
                max_linear_step=config.pos_step,
                angle_step=config.angle_step,
                rotation_world_frame=config.gamepad_rotation_world_frame,
                primary_task=getattr(config, "task", ""),
                secondary_task=getattr(config, "task_secondary", ""),
            )
        self.visualize = config.enable_visualization
        self.use_async = getattr(config, "use_async", False)
        if self.use_policy:
            policy_path = config.policy_path
            repo_id = config.policy_repo_id
            pd = getattr(config, "policy_device", "cuda") or "cuda"
            if pd == "auto":
                pd = "cuda" if torch.cuda.is_available() else "cpu"
            elif pd == "cuda" and not torch.cuda.is_available():
                logger.warning("policy_device=cuda but CUDA not available; using cpu")
                pd = "cpu"
            logger.info("LeRobot policy inference device: %s", pd)
            if self.use_async:
                # Async deployment: launches LeRobot's policy_server + a gRPC client whose
                # .predict() matches LerobotPolicy.predict(), so the loop below is unchanged.
                from piper_teleop.robot_server.async_policy import AsyncPolicyClient

                logger.info("Async policy deployment enabled (policy_server on %s:%d).",
                            config.async_host, config.async_port)
                self.policy = AsyncPolicyClient(
                    policy_path=policy_path,
                    repo_id=repo_id,
                    policy_type=config.policy_type,
                    device=pd,
                    host=config.async_host,
                    port=config.async_port,
                    fps=config.fps,
                    actions_per_chunk=config.async_actions_per_chunk,
                    chunk_size_threshold=config.async_chunk_size_threshold,
                    aggregate_fn_name=config.async_aggregate_fn,
                    dof=config.dof,
                    task=getattr(config, "task", ""),
                )
            else:
                from piper_teleop.robot_server.lerobot_policy import LerobotPolicy

                self.policy = LerobotPolicy(
                    policy_path,
                    repo_id,
                    device=pd,
                    rename_map=getattr(config, "policy_rename_map", None),
                    task=getattr(config, "task", ""),
                )
            self._policy_override_active_last = False
            self._policy_task_last = None

        self.shared_data = shared_data
        if self.config.record:
            self.recorder = Recorder(
                repo_id=config.repo_id,
                resume=config.resume,
                task=config.task,
                root=config.root,
                single_arm=config.single_arm,
                cameras=config.camera_configs,
                dof=config.dof,
                fps=config.fps,
                robot_type=config.robot_type,
                use_video=config.use_video,
                display_data=config.display_data,
                convert_images_to_video=config.convert_images_to_video,
                image_writer_processes=config.image_writer_processes,
                image_writer_threads=config.image_writer_threads,
            )
            self.recorder.start_recording()
        if self.use_leader:
            self.robot_leader = PiperLeader()

    def _handle_gamepad_recording_shortcuts(self) -> None:
        if not (self.config.record and self.use_gamepad):
            return

        command = self.gamepad_controller.consume_recording_command()
        if command == "cycle":
            self.recorder.request_cycle_episode()
        elif command == "discard":
            self.recorder.request_discard_episode()


    def _record_side_for_single_arm(self) -> str:
        """Choose which logical side to use for single-arm datasets."""
        cfg = self.config.single_arm_record_side
        if cfg in ("left", "right"):
            return cfg
        return self.robot_interface.active_arm_side

    def _policy_action_side(self) -> str:
        return self._record_side_for_single_arm()

    def _policy_action_dicts_to_joint_angles(self, dict_left: dict, dict_right: dict) -> np.ndarray:
        joint_angles = np.array(self.robot_interface.arm_angles, copy=True)
        source = dict_right if dict_right else dict_left
        for i in range(6):
            key = f"joint_{i}.pos"
            if key in source:
                joint_angles[i] = float(source[key])
        if "joint_6.pos" in source:
            joint_angles[6] = float(source["joint_6.pos"])
        return joint_angles

    def update_arm_state(self, arm_goal, arm_state: ArmState) -> ArmState:
        if arm_goal.reset_to_init:
            arm_state.target_transform = arm_state.initial_transform
            arm_state.origin_transform = arm_state.initial_transform
            arm_state.gripper_closed = False
            arm_state.gripper_gap_m = GRIPPER_FINGER_OPEN_M
        elif arm_goal.reset_reference:
            if self.robot_enabled:
                # NOTE: We use the last target transform as the origin transform since there is an offset between the target and the EEF transform
                arm_state.origin_transform = (
                    arm_state.target_transform
                    if arm_state.target_transform is not None
                    else arm_state.initial_transform
                )
        elif arm_goal.relative_transform is not None:
            relative_transform = arm_goal.relative_transform

            # Gamepad goals accumulate target in world frame; relative = inv(origin) @ target is already correct.
            # Keyboard/VR: relative is in body-local frame; conjugate to world before composing.
            if not self.use_gamepad:
                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = arm_state.origin_transform[:3, :3]
                relative_transform = np.linalg.inv(transformation_matrix) @ (relative_transform @ transformation_matrix)
            arm_state.target_transform = arm_state.origin_transform @ relative_transform

        if arm_goal.gripper_closed is not None and not arm_goal.reset_to_init:
            arm_state.gripper_closed = arm_goal.gripper_closed
        return arm_state

    @staticmethod
    def _gripper_finger_m(arm: ArmState) -> float:
        if arm.gripper_gap_m is not None:
            return float(np.clip(arm.gripper_gap_m, 0.0, GRIPPER_FINGER_OPEN_M))
        return 0.0 if arm.gripper_closed else GRIPPER_FINGER_OPEN_M

    def update_robot_from_leader(self, obs_dict_leader: dict):
        side = self._record_side_for_single_arm()
        source = obs_dict_leader[side]
        joint_values = np.array([source[f"joint_{i}.pos"] for i in range(7)], dtype=np.float64)
        if self.visualize:
            self.robot_interface.ik_solver.vis.display(joint_values[:6])
        self.robot_interface.update_arm_angles(joint_values)
        if self.robot_enabled:
            self.robot_interface.send_command()

    def update_robot(self, arm: ArmState):
        """Update robot with current control goal."""
        start_time_total = time.perf_counter()
        # Measure all IK time together
        start_time_ik = time.perf_counter()

        ik_solution, is_collision = self.robot_interface.solve_ik(
            arm.target_transform, visualize=self.visualize
        )
        current_gripper = self._gripper_finger_m(arm)

        if ik_solution is None:
            logger.debug("IK did not converge; keeping last joint targets, still applying gripper.")
            qa = self.robot_interface.arm_angles
            joint6 = qa[:6] if len(qa) >= 6 else np.zeros(6)
            self.robot_interface.update_arm_angles(np.concatenate([joint6, [current_gripper]]))
        elif not is_collision:
            self.robot_interface.update_arm_angles(np.concatenate([ik_solution, [current_gripper]]))
        else:
            # Do not move arm through a bad IK pose, but still apply gripper (keyboard toggle).
            logger.debug("IK in collision; holding arm pose, applying gripper only.")
            qa = self.robot_interface.arm_angles
            joint6 = qa[:6] if len(qa) >= 6 else ik_solution
            self.robot_interface.update_arm_angles(np.concatenate([joint6, [current_gripper]]))

        ik_time = time.perf_counter() - start_time_ik

        # Send commands
        start_time_send = time.perf_counter()
        if self.robot_enabled:
            self.robot_interface.send_command()
        send_time = time.perf_counter() - start_time_send

        total_time = time.perf_counter() - start_time_total
        overhead_time = total_time - ik_time - send_time

        # # Print all at once to minimize timing impact
        logger.debug(
            f"IK: {ik_time*1000:.1f}ms, CAN: {send_time*1000:.1f}ms, "
            f"Overhead: {overhead_time*1000:.1f}ms, Total: {total_time*1000:.1f}ms"
        )

    def _load_deposit_pose_file(self, arm: ArmState, pose_file: str | None, target_attr: str, label: str) -> None:
        if not pose_file:
            return
        pose_path = get_absolute_path(pose_file)
        if not pose_path.exists():
            logger.warning("%s pose file not found: %s", label, pose_path)
            return

        try:
            with open(pose_path, "r") as handle:
                pose_data = yaml.safe_load(handle) or {}
        except Exception as exc:
            logger.error("Failed to load %s pose file %s: %s", label, pose_path, exc)
            return

        side = self._record_side_for_single_arm()
        arm_payload = pose_data.get(side, pose_data.get("arm", {}))
        transform_rows = arm_payload.get("transform")
        if transform_rows is None:
            logger.warning("%s pose file %s has no transform for %s/arm", label, pose_path, side)
            return

        transform = np.asarray(transform_rows, dtype=float)
        if transform.shape != (4, 4):
            logger.error(
                "%s pose for %s arm in %s has invalid shape %s",
                label,
                side,
                pose_path,
                transform.shape,
            )
            return

        setattr(arm, target_attr, transform)
        logger.info("Loaded %s arm %s pose from %s", side, label.lower(), pose_path)

    async def run(self):
        """Control loop for the teleoperation system."""
        arm = ArmState(arm_name="arm")

        self.robot_interface.setup_kinematics()
        if self.use_keyboard or self.use_gamepad or self.use_policy or self.use_leader:
            logger.info(
                "Local/policy control mode: %s",
                "leader"
                if self.use_leader
                else ("policy" if self.use_policy else ("keyboard" if self.use_keyboard else "gamepad")),
            )
        else:
            logger.warning("No control source selected; node will hold the current target pose.")
        if self.robot_enabled:
            try:
                self.robot_interface.connect()
            except Exception as e:
                logger.error(f"Error connecting to robot: {e}")
                return
            finally:
                self.robot_enabled = self.robot_interface.is_connected
        self._load_deposit_pose_file(arm, self.config.deposit_pose_file, "deposit_transform", "Primary deposit")
        self._load_deposit_pose_file(
            arm,
            self.config.secondary_deposit_pose_file,
            "secondary_deposit_transform",
            "Secondary deposit",
        )
        if self.robot_enabled:
            self.robot_interface.return_to_initial_position()
        if self.use_leader:
            self.robot_leader.connect()

        arm.target_transform = arm.initial_transform

        while True:
            iteration_start = time.perf_counter()
            commands_start = time.perf_counter()

            commands_time = time.perf_counter() - commands_start

            # Policy + gamepad: Share/PS toggle policy on/off and select its task; when off,
            # gamepad drives the arm (no stick takeover).
            if self.use_policy and self.use_gamepad:
                self.gamepad_controller.update_policy_toggle()
                override_active = not self.gamepad_controller.policy_engaged()
                # Push the selected task onto the live policy (matters for VLA; ignored by ACT).
                self.policy.task = self.gamepad_controller.selected_task()
            else:
                override_active = False

            if self.use_policy:
                task_now = getattr(self.policy, "task", None)
                # Reset on engage/disengage and when the task switches, so the action queue is rebuilt.
                if override_active != self._policy_override_active_last or task_now != self._policy_task_last:
                    self.policy.reset()
                self._policy_override_active_last = override_active
                self._policy_task_last = task_now

            if self.use_gamepad and not self.use_policy:
                arm_goal = self.gamepad_controller.get_goal(
                    arm.target_transform, arm.deposit_transform, arm.secondary_deposit_transform
                )
            elif override_active:
                arm_goal = self.gamepad_controller.get_goal(
                    arm.target_transform, arm.deposit_transform, arm.secondary_deposit_transform
                )
            elif self.use_keyboard:
                keyboard_side = self._record_side_for_single_arm()
                self.keyboard_controller.single_follower_side = keyboard_side
                arm_goal = self.keyboard_controller.get_goal(keyboard_side, arm.target_transform)
            elif self.use_policy or self.use_leader:
                arm_goal = ArmGoal(arm=self._record_side_for_single_arm())
            else:
                arm_goal = ArmGoal(arm=self._record_side_for_single_arm())

            arm = self.update_arm_state(arm_goal, arm)

            if self.use_gamepad:
                g = self.gamepad_controller.gripper_gap_m
                arm.gripper_gap_m = g
            else:
                arm.gripper_gap_m = None

            if self.config.record or self.use_policy:
                obs_dict = self.robot_interface.get_observation()
                cams = self.shared_data.get_camera_dict()

            # Simulates blocking robot communication
            robot_start = time.perf_counter()
            if self.use_leader:
                obs_dict_leader = self.robot_leader.get_observations()
                self.update_robot_from_leader(obs_dict_leader)
            elif self.use_policy and not override_active:
                dict_left, dict_right = self.policy.predict(
                    obs_dict["left"],
                    obs_dict["right"],
                    cams,
                    single_arm_side=self._policy_action_side(),
                )
                if self.visualize:
                    left_vis = dict_left if dict_left else dict_right
                    q_1 = [left_vis[k] for k in sorted(left_vis)[:6]]
                    self.robot_interface.ik_solver.vis.display(np.array(q_1))
                if self.robot_enabled:
                    self.robot_interface.update_arm_angles(
                        self._policy_action_dicts_to_joint_angles(dict_left, dict_right)
                    )
                    self.robot_interface.send_command()
            else:
                self.update_robot(arm)
            robot_time = time.perf_counter() - robot_start

            if self.config.record:
                self._handle_gamepad_recording_shortcuts()
                action_dict = arm_angles_to_action_dict(self.robot_interface.arm_angles)
                self.recorder.add_observation(
                    left_joints=obs_dict["left"],
                    right_joints=obs_dict["right"],
                    left_joints_target=action_dict["active"],
                    right_joints_target=action_dict["active"],
                    cams=cams,
                    record_side=self._record_side_for_single_arm(),
                )
                self.recorder.handle_keyboard_event()
                if self.config.display_data:
                    self.recorder.show_data(
                        left_joints=obs_dict["left"],
                        right_joints=obs_dict["right"],
                        left_joints_target=action_dict["active"],
                        right_joints_target=action_dict["active"],
                        cams=cams,
                    )

            sleep_start = time.perf_counter()
            await asyncio.sleep(0.001)
            sleep_time = time.perf_counter() - sleep_start

            dt_s = time.perf_counter() - iteration_start
            if self.config.record:
                rec_state = self.recorder.state.name
                extra = ""
                if self.use_policy and self.use_gamepad:
                    pol = "ON" if self.gamepad_controller.policy_engaged() else "OFF"
                    extra = f" | POLICY: {pol}"
                print(f"\rFPS: {1/dt_s:.1f} | REC: {rec_state}{extra}", end="", flush=True)
            elif self.use_policy and self.use_gamepad:
                pol = "ON" if self.gamepad_controller.policy_engaged() else "OFF"
                print(f"\rFPS: {1/dt_s:.1f} | POLICY: {pol}", end="", flush=True)
            else:
                print(f"\rFPS: {1/dt_s:.1f}", end="", flush=True)
            precise_sleep(1 / self.config.fps - dt_s)

            total_time = time.perf_counter() - iteration_start
            overhead_time = total_time - commands_time - robot_time - sleep_time

            # Single consolidated logging statement
            logger.debug(
                f"Loop: {total_time*1000:.1f}ms ({1/total_time:.1f}Hz) | "
                f"Cmd: {commands_time*1000:.1f}ms | "
                f"Robot: {robot_time*1000:.1f}ms | "
                f"Sleep: {sleep_time*1000:.1f}ms | "
                f"Overhead: {overhead_time*1000:.1f}ms"
                "\n================================================================================="
            )

    async def stop(self):
        """Stop the control loop."""
        if self.use_keyboard:
            self.keyboard_controller.stop()
        if self.use_gamepad:
            self.gamepad_controller.stop()
        if self.robot_enabled:
            self.robot_interface.disconnect(return_to_initial=False)
        if self.use_leader:
            self.robot_leader.disconnect()
