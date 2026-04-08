import asyncio
import copy
import logging
import os
import time
from dataclasses import dataclass
from pathlib import Path

import dotenv
import numpy as np
import yaml
from lerobot.utils.robot_utils import precise_sleep
from tactile_teleop_sdk import TactileAPI

from piper_teleop.config import TelegripConfig
from piper_teleop.robot_server.camera import SharedCameraData
from piper_teleop.robot_server.keyboard_controller import KeyboardController
from piper_teleop.robot_server.lerobot_policy import LerobotPolicy
from piper_teleop.utils import get_absolute_path

from .core.geometry import xyzrpy2transform
from .core.robot_interface import RobotInterface, arm_angles_to_action_dict
from .recorder import Recorder
from .robot_leader import PiperLeader

dotenv.load_dotenv()

logger = logging.getLogger(__name__)

# Prismatic finger command when gripper is "open" in toggle mode (matches gamepad max gap).
GRIPPER_FINGER_OPEN_M = 0.07


@dataclass
class ArmState:
    arm_name: str
    initial_transform: np.ndarray = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
    origin_transform: np.ndarray = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
    target_transform: np.ndarray | None = None
    deposit_transform: np.ndarray | None = None
    gripper_closed: bool = True
    # Gamepad: incremental opening in metres; None = use gripper_closed / GRIPPER_FINGER_OPEN_M
    gripper_gap_m: float | None = None


class ControlLoop:
    """Control loop for the teleoperation system."""

    def __init__(
        self,
        config: TelegripConfig,
        shared_data: SharedCameraData,  #
    ):
        self.config = config
        self.robot_interface = RobotInterface(config)
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
            )
        self.visualize = config.enable_visualization
        self.api = TactileAPI(api_key=os.getenv("TACTILE_API_KEY"))

        if self.use_policy:
            policy_path = config.policy_path
            repo_id = config.policy_repo_id
            self.policy = LerobotPolicy(policy_path, repo_id)

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

    def _record_side_for_single_arm(self) -> str:
        """Which arm to log when config.single_arm (matches dataset 7-D features)."""
        cfg = self.config.single_arm_record_side
        if cfg == "right":
            return "right"
        if cfg == "left":
            return "left"
        # auto: mirror README — only right follower → use right; otherwise left
        if self.robot_interface.right_arm_connected and not self.robot_interface.left_arm_connected:
            return "right"
        return "left"

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
        dict_left = obs_dict_leader["left"]
        dict_right = obs_dict_leader["right"]
        q_1 = [dict_left[k] for k in sorted(dict_left)]
        q_2 = [dict_right[k] for k in sorted(dict_right)]
        joints_wo_gripper = np.array(q_2[:-1] + q_1[:-1])
        joint_gripper_s = [q_1[-1], q_2[-1]]
        if self.visualize:
            self.robot_interface.ik_solver.vis.display(joints_wo_gripper)
        self.robot_interface.update_arm_angles(np.concatenate([joints_wo_gripper, joint_gripper_s]))
        if self.robot_enabled:
            self.robot_interface.send_command()

    def update_robot(self, left_arm: ArmState, right_arm: ArmState):
        """Update robot with current control goals."""
        start_time_total = time.perf_counter()
        # Measure all IK time together
        start_time_ik = time.perf_counter()

        ik_solution, is_collision = self.robot_interface.solve_ik(
            left_arm.target_transform, right_arm.target_transform, visualize=self.visualize
        )

        current_gripper_1 = self._gripper_finger_m(left_arm)
        current_gripper_2 = self._gripper_finger_m(right_arm)

        if ik_solution is None:
            logger.debug("IK did not converge; keeping last joint targets, still applying gripper.")
            qa = self.robot_interface.arm_angles
            joint12 = qa[:12] if len(qa) >= 12 else np.zeros(12)
            self.robot_interface.update_arm_angles(
                np.concatenate([joint12, [current_gripper_1, current_gripper_2]])
            )
        elif not is_collision:
            self.robot_interface.update_arm_angles(
                np.concatenate([ik_solution, [current_gripper_1, current_gripper_2]])
            )
        else:
            # Do not move arm through a bad IK pose, but still apply gripper (keyboard toggle).
            logger.debug("IK in collision; holding arm pose, applying gripper only.")
            qa = self.robot_interface.arm_angles
            joint12 = qa[:12] if len(qa) >= 12 else ik_solution
            self.robot_interface.update_arm_angles(
                np.concatenate([joint12, [current_gripper_1, current_gripper_2]])
            )

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

    def _load_deposit_pose_file(self, left_arm: ArmState, right_arm: ArmState) -> None:
        pose_path = get_absolute_path(self.config.deposit_pose_file)
        if not pose_path.exists():
            logger.warning("Deposit pose file not found: %s", pose_path)
            return

        try:
            with open(pose_path, "r") as handle:
                pose_data = yaml.safe_load(handle) or {}
        except Exception as exc:
            logger.error("Failed to load deposit pose file %s: %s", pose_path, exc)
            return

        arm_mappings = [("left", left_arm), ("right", right_arm)]
        for arm_name, arm_state in arm_mappings:
            arm_payload = pose_data.get(arm_name, {})
            transform_rows = arm_payload.get("transform")
            if transform_rows is None:
                continue

            transform = np.asarray(transform_rows, dtype=float)
            if transform.shape != (4, 4):
                logger.error(
                    "Deposit pose for %s arm in %s has invalid shape %s",
                    arm_name,
                    pose_path,
                    transform.shape,
                )
                continue

            arm_state.deposit_transform = transform
            logger.info("Loaded %s arm deposit pose from %s", arm_name, pose_path)

    async def run(self):
        """Control loop for the teleoperation system."""
        left_arm = ArmState(arm_name="left")
        right_arm = ArmState(arm_name="right")

        right_arm.initial_transform = xyzrpy2transform(0.19, -0.57, 0.2, 0, 1.57, 0)
        right_arm.origin_transform = right_arm.initial_transform

        self.robot_interface.setup_kinematics()
        if self.use_keyboard or self.use_gamepad:
            logger.info(
                "Local control (%s): skipping Tactile VR connection (no LiveKit room required).",
                "keyboard" if self.use_keyboard else "gamepad",
            )
        else:
            await self.api.connect_vr_controller()
        if self.robot_enabled:
            try:
                self.robot_interface.connect()
            except Exception as e:
                logger.error(f"Error connecting to robot: {e}")
                return
            finally:
                # One physical follower arm is enough to drive teleop (other side may have no CAN)
                self.robot_enabled = (
                    self.robot_interface.left_arm_connected or self.robot_interface.right_arm_connected
                )
        self._load_deposit_pose_file(left_arm, right_arm)
        if self.robot_enabled:
            self.robot_interface.return_to_initial_position()
        if self.use_leader:
            self.robot_leader.connect()

        left_arm.target_transform = left_arm.initial_transform
        right_arm.target_transform = right_arm.initial_transform

        while True:
            iteration_start = time.perf_counter()
            commands_start = time.perf_counter()

            commands_time = time.perf_counter() - commands_start

            if self.use_gamepad:
                # Gamepad always drives the left-arm goal; the same goal is copied for the right IK chain.
                left_arm_goal = self.gamepad_controller.get_goal(left_arm.target_transform, left_arm.deposit_transform)
                right_arm_goal = copy.deepcopy(left_arm_goal)
            elif self.use_keyboard:
                # Single follower: only call get_goal for the connected arm (merged WASD+TG keys in
                # KeyboardController), then duplicate for IK. Otherwise right-column keys would only
                # move the other arm's joint block, which has no hardware on left_piper-only setups.
                if self.robot_enabled and self.robot_interface.left_arm_connected and not self.robot_interface.right_arm_connected:
                    self.keyboard_controller.single_follower_side = "left"
                    left_arm_goal = self.keyboard_controller.get_goal("left", left_arm.target_transform)
                    right_arm_goal = copy.deepcopy(left_arm_goal)
                elif self.robot_enabled and self.robot_interface.right_arm_connected and not self.robot_interface.left_arm_connected:
                    self.keyboard_controller.single_follower_side = "right"
                    right_arm_goal = self.keyboard_controller.get_goal("right", right_arm.target_transform)
                    left_arm_goal = copy.deepcopy(right_arm_goal)
                else:
                    self.keyboard_controller.single_follower_side = None
                    left_arm_goal = self.keyboard_controller.get_goal("left", left_arm.target_transform)
                    right_arm_goal = self.keyboard_controller.get_goal("right", right_arm.target_transform)
            else:
                left_arm_goal = await self.api.get_controller_goal("left")
                right_arm_goal = await self.api.get_controller_goal("right")

            left_arm = self.update_arm_state(left_arm_goal, left_arm)
            right_arm = self.update_arm_state(right_arm_goal, right_arm)

            if self.use_gamepad:
                g = self.gamepad_controller.gripper_gap_m
                left_arm.gripper_gap_m = g
                right_arm.gripper_gap_m = g
            else:
                left_arm.gripper_gap_m = None
                right_arm.gripper_gap_m = None

            if self.config.record or self.use_policy:
                obs_dict = self.robot_interface.get_observation()
                cams = self.shared_data.get_camera_dict()

            # Simulates blocking robot communication
            robot_start = time.perf_counter()
            if self.use_leader:
                obs_dict_leader = self.robot_leader.get_observations()
                self.update_robot_from_leader(obs_dict_leader)
            elif self.use_policy:
                dict_left, dict_right = self.policy.predict(obs_dict["left"], obs_dict["right"], cams)
                if self.visualize:
                    q_1 = [dict_left[k] for k in sorted(dict_left)[:6]]
                    q_2 = [dict_right[k] for k in sorted(dict_right)[:6]]
                    self.robot_interface.ik_solver.vis.display(np.array(q_1 + q_2))
                if self.robot_enabled:
                    if self.robot_interface.left_robot:
                        self.robot_interface.left_robot.send_action(dict_left)
                    if self.robot_interface.right_robot:
                        self.robot_interface.right_robot.send_action(dict_right)
            else:
                self.update_robot(left_arm, right_arm)
            robot_time = time.perf_counter() - robot_start

            if self.config.record:
                action_dict = arm_angles_to_action_dict(self.robot_interface.arm_angles)
                self.recorder.add_observation(
                    left_joints=obs_dict["left"],
                    right_joints=obs_dict["right"],
                    left_joints_target=action_dict["left"],
                    right_joints_target=action_dict["right"],
                    cams=cams,
                    record_side=self._record_side_for_single_arm()
                    if self.config.single_arm
                    else "left",
                )
                self.recorder.handle_keyboard_event()
                if self.config.display_data:
                    self.recorder.show_data(
                        left_joints=obs_dict["left"],
                        right_joints=obs_dict["right"],
                        left_joints_target=action_dict["left"],
                        right_joints_target=action_dict["right"],
                        cams=cams,
                    )

            sleep_start = time.perf_counter()
            await asyncio.sleep(0.001)
            sleep_time = time.perf_counter() - sleep_start

            dt_s = time.perf_counter() - iteration_start
            print(f"\rFPS: {1/dt_s}", end="", flush=True)
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
        if not self.use_keyboard and not self.use_gamepad:
            await self.api.disconnect_vr_controller()
        if self.robot_enabled:
            self.robot_interface.disconnect()
        if self.use_leader:
            self.robot_leader.disconnect()
