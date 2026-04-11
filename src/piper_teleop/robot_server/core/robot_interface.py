"""
Robot interface module.
Provides a clean wrapper around robot devices with safety checks and convenience methods.
"""

import contextlib
import logging
import os
import sys
import time
from typing import Any

import numpy as np
import pinocchio as pin

from piper_teleop.config import NUM_JOINTS, TelegripConfig

from .geometry import transform2pose, xyzrpy2transform
from .kinematics import Arm_IK
from .piper import Piper, PiperConfig

logger = logging.getLogger(__name__)


def arm_angles_to_action_dict(arm_angles: np.ndarray) -> dict[str, dict[str, float]]:
    """Convert single-arm 7D joint vector to action dictionaries."""
    action_dict = {
        f"joint_{i}.pos": float(arm_angles[i]) for i in range(6)
    }
    action_dict["joint_6.pos"] = float(arm_angles[6])
    # Keep legacy keys for recorder/policy interfaces.
    return {"active": action_dict, "left": action_dict, "right": action_dict}


@contextlib.contextmanager
def suppress_stdout_stderr():
    """Context manager to suppress stdout and stderr output at the file descriptor level."""
    # Save original file descriptors
    stdout_fd = sys.stdout.fileno()
    stderr_fd = sys.stderr.fileno()

    # Save original file descriptors
    saved_stdout_fd = os.dup(stdout_fd)
    saved_stderr_fd = os.dup(stderr_fd)

    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)

        # Redirect stdout and stderr to devnull
        os.dup2(devnull_fd, stdout_fd)
        os.dup2(devnull_fd, stderr_fd)

        yield

    finally:
        # Restore original file descriptors
        os.dup2(saved_stdout_fd, stdout_fd)
        os.dup2(saved_stderr_fd, stderr_fd)

        # Close saved file descriptors
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)
        os.close(devnull_fd)


class RobotInterface:
    """High-level interface for robot control with safety features."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.robot = None
        self.active_arm_side = "left"
        self.is_enabled = config.enable_robot
        self.is_connected = False

        # Compatibility flags used by policy/recorder path.
        self.left_arm_connected = False
        self.right_arm_connected = False

        # Single-arm state: 6 arm joints + 1 gripper.
        self.arm_angles = np.zeros(NUM_JOINTS)

        self.ik_solver = None

        # Control timing
        self.last_send_time = 0

        # Error tracking
        self.arm_errors = 0
        self.general_errors = 0
        self.max_arm_errors = 3
        self.max_general_errors = 8  # Allow more general errors before full disconnection

        self.initial_arm = xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)

    def connect(self) -> bool:
        """Connect to robot hardware."""
        if self.is_connected:
            logger.info("Robot interface already connected")
            return True

        if not self.is_enabled:
            logger.info("Robot control is not enabled")
            return False

        try:
            logger.info("Connecting to robot...")

            for side, port in (("left", "left_piper"), ("right", "right_piper")):
                try:
                    with suppress_stdout_stderr():
                        robot = Piper(PiperConfig(port=port, id=f"{side}_follower"))
                        robot.connect()
                    self.robot = robot
                    self.active_arm_side = side
                    self.is_connected = True
                    self.left_arm_connected = side == "left"
                    self.right_arm_connected = side == "right"
                    logger.info("✅ Connected single follower arm on %s (%s)", side, port)
                    break
                except Exception as e:
                    logger.warning("Connection failed on %s (%s): %s", side, port, e)

            if not self.is_connected:
                logger.error("❌ Failed to connect follower arm on left_piper or right_piper")

            return self.is_connected

        except Exception as e:
            logger.error(f"❌ Robot connection failed with exception: {e}")
            self.is_connected = False
            return False

    def setup_kinematics(self):
        """Setup single-arm kinematics solver."""
        ground_height = self.config.ground_height
        self.ik_solver = Arm_IK(self.config.urdf_path, ground_height, self.config.collision_space_urdf)
        logger.info("Single-arm IK initialized with ground plane at height %.3f", ground_height)

    def get_end_effector_transform(self, arm: str) -> np.ndarray:
        """Get end effector pose for specified arm.

        Returns:
            np.ndarray: 4x4 transform matrix.
        """
        if arm == "left":
            return (
                self.robot.get_end_effector_transform()
                if self.robot and self.left_arm_connected
                else xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
            )
        elif arm == "right":
            return (
                self.robot.get_end_effector_transform()
                if self.robot and self.right_arm_connected
                else xyzrpy2transform(0.19, 0.0, 0.2, 0, 1.57, 0)
            )
        else:
            raise ValueError(f"Invalid arm: {arm}")

    def solve_ik(self, target_pose_1: np.ndarray, _unused_target_pose_2: np.ndarray | None = None, visualize: bool = True) -> np.ndarray:
        """Solve inverse kinematics for a single arm."""
        position_1, quaternion_1 = transform2pose(target_pose_1)
        # transform2pose returns XYZW, but pin.Quaternion expects WXYZ
        target_1 = pin.SE3(
            pin.Quaternion(quaternion_1[3], quaternion_1[0], quaternion_1[1], quaternion_1[2]),
            position_1,
        )
        sol_q, is_collision = self.ik_solver.ik_fun(target_1.homogeneous, visualize=visualize)
        return sol_q, is_collision

    def update_arm_angles(self, joint_angles: np.ndarray):
        """Update joint angles for specified arm."""
        self.arm_angles = joint_angles

    def send_command(self) -> bool:
        """Send current joint angles to robot using dictionary format."""
        if not self.is_connected or not self.is_enabled:
            return False

        current_time = time.time()
        if current_time - self.last_send_time < self.config.send_interval:
            return True  # Don't send too frequently

        try:
            success = True
            if self.robot and self.is_connected:
                try:
                    action_dict = arm_angles_to_action_dict(self.arm_angles)
                    with suppress_stdout_stderr():
                        self.robot.send_action(action_dict["active"])
                except Exception as e:
                    logger.error("Error sending arm command: %s", e)
                    self.arm_errors += 1
                    if self.arm_errors > self.max_arm_errors:
                        self.is_connected = False
                        self.left_arm_connected = False
                        self.right_arm_connected = False
                        logger.error("❌ Arm disconnected due to repeated errors")
                    success = False

            self.last_send_time = current_time
            return success

        except Exception as e:
            logger.error(f"Error sending robot command: {e}")
            self.general_errors += 1
            if self.general_errors > self.max_general_errors:
                self.is_connected = False
                logger.error("❌ Robot interface disconnected due to repeated errors")
            return False

    def set_gripper(self, arm: str, closed: bool):
        """Set gripper state for specified arm."""
        angle = 0.0 if closed else 0.07
        self.arm_angles[6] = angle

    def return_to_initial_position(self):
        """Return connected arm to initial position."""
        logger.info("⏪ Returning arm to initial position...")

        arm_angles, collision = self.solve_ik(self.initial_arm, visualize=True)

        if collision:
            logger.error("❌ Cannot return to initial position due to collision in IK solution")
            self.arm_angles = np.zeros(NUM_JOINTS)
            return

        self.arm_angles = np.concatenate((arm_angles, [0.0]))

        try:
            # Send commands for a few iterations to ensure movement
            for i in range(10):
                self.send_command()
                time.sleep(0.1)

            logger.info("✅ Robot returned to initial position")
        except Exception as e:
            logger.error(f"Error returning to initial position: {e}")

    def disconnect(self, return_to_initial: bool = True):
        """Disconnect from robot hardware."""
        if not self.is_connected:
            return

        logger.info("Disconnecting from robot...")

        # Return to initial positions if engaged
        if self.is_enabled and return_to_initial:
            try:
                self.return_to_initial_position()
            except Exception as e:
                logger.error(f"Error returning to initial position: {e}")

        if self.robot:
            try:
                self.robot.disconnect()
            except Exception as e:
                logger.error(f"Error disconnecting arm: {e}")
            self.robot = None

        self.is_connected = False
        self.is_enabled = False
        self.left_arm_connected = False
        self.right_arm_connected = False
        logger.info("🔌 Robot disconnected")

    def get_arm_connection_status(self, arm: str) -> bool:
        """Get connection status for specific arm based on device file existence."""
        if arm == "left":
            return self.left_arm_connected
        if arm == "right":
            return self.right_arm_connected
        return False

    def update_arm_connection_status(self):
        """Update individual arm connection status based on device file existence."""
        if self.is_connected and self.robot:
            connected = self.robot.is_connected
            self.left_arm_connected = connected and self.active_arm_side == "left"
            self.right_arm_connected = connected and self.active_arm_side == "right"

    def get_observation(self) -> dict[str, Any]:
        """Get observation from robot."""
        action_dict = arm_angles_to_action_dict(self.arm_angles)
        obs = self.robot.get_observation() if self.robot else action_dict["active"]
        # Keep legacy keys so policy/recorder code paths remain compatible.
        return {"left": obs, "right": obs}
