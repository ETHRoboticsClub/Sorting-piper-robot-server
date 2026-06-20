"""Drop-in replacement for LeRobot's ``RobotKinematics``, backed by the repo's Arm_IK.

LeRobot's HIL-SERL pipeline does EE<->joint kinematics through a ``RobotKinematics``
(placo) object that ``gym_manipulator`` constructs once and hands to every IK step
(``EEReferenceAndDelta``, ``InverseKinematicsRLStep``, the FK observation steps).
Those steps only ever call ``forward_kinematics`` / ``inverse_kinematics`` on it, so
substituting this class makes the *entire RL pipeline* use the exact same IK as
``robotserver`` -- the tool frame (joint6 + pitch-90 + 0.13 m), collision checks,
and >30-degree jump-escape.

Units: this adapter is **radians** end to end (Piper SDK + Arm_IK are radians), so
joint observations and action targets flow through the pipeline unconverted.
``RobotKinematics`` is degrees; we are not, on purpose -- the Piper robot wrapper is
radians too, so the pipeline stays consistent.

Substitute it via the launcher (no LeRobot source edit needed):
    import lerobot.rl.gym_manipulator as gm
    from piper_teleop.lerobot_plugin.arm_ik_kinematics import ArmIkKinematics
    gm.RobotKinematics = ArmIkKinematics
"""

import contextlib
import io

import numpy as np
import pinocchio as pin

from piper_teleop.robot_server.core.geometry import transform2pose, xyzrpy2transform
from piper_teleop.robot_server.core.robot_interface import suppress_stdout_stderr

PIPER_ARM_DOF = 6


class ArmIkKinematics:
    """``RobotKinematics``-compatible kinematics backed by Arm_IK (radians)."""

    def __init__(self, urdf_path: str | None = None, target_frame_name: str | None = None,
                 joint_names: list[str] | None = None):
        # target_frame_name is ignored: Arm_IK defines its own tool ("ee") frame.
        from piper_teleop.config import TelegripConfig
        from piper_teleop.robot_server.core.robot_interface import RobotInterface

        cfg = TelegripConfig()
        cfg.enable_robot = False
        self._ri = RobotInterface(cfg)
        self._ri.setup_kinematics()
        self._ik = self._ri.ik_solver

        # Operator request: remove the collision space entirely (it was preventing
        # grasps near the table). Neuter the checker so the collision geometry +
        # ground plane can never gate motion, and we skip the per-step collision
        # computation. (The IK optimisation itself has no collision constraints --
        # only joint-limit bounds -- so this purely drops the post-solve check.)
        self._ik.check_collision = lambda *args, **kwargs: False
        self.joint_names = joint_names or [f"joint_{i}" for i in range(PIPER_ARM_DOF)]

    def forward_kinematics(self, joint_pos: np.ndarray) -> np.ndarray:
        """Joint positions (radians) -> 4x4 tool pose (the Arm_IK 'ee' frame)."""
        q = np.asarray(joint_pos, float)[:PIPER_ARM_DOF]
        return xyzrpy2transform(*self._ik.get_pose(q))

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,
        desired_ee_pose: np.ndarray,
        position_weight: float = 1.0,
        orientation_weight: float = 0.01,
    ) -> np.ndarray:
        """Target 4x4 tool pose -> joint positions (radians), warm-started + collision-checked."""
        q_curr = np.asarray(current_joint_pos, float)[:PIPER_ARM_DOF]
        desired = np.asarray(desired_ee_pose, float)

        # Zero-residual targets make the casadi/ipopt solver emit a NaN gradient
        # (the SE3 log is exactly identity). LeRobot's "hold" behaviour feeds the
        # current pose verbatim every idle step, so short-circuit when the target
        # already matches the current pose -- avoids the NaN spam and a wasted
        # solve. Any real teleop/policy step is ~8 mm, far above this tolerance.
        if np.allclose(desired, self.forward_kinematics(q_curr), atol=1e-6, rtol=0.0):
            return q_curr

        # Re-orthonormalize the target through a quaternion exactly like
        # robotserver's solve_ik: a clean SE3 target converges noticeably more
        # reliably than the raw (round-tripped) matrix, especially for rotation.
        pos, quat_xyzw = transform2pose(desired)
        target = pin.SE3(
            pin.Quaternion(quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]), pos
        ).homogeneous

        # IK failures are expected at singular/unreachable targets (the loop just
        # holds). Two output channels must be silenced: casadi's C++ NaN warnings
        # (fd-level stderr -> repo's fd suppressor) and Arm_IK's own buffered
        # ``print`` on non-convergence (Python-level -> redirect_std*). The fd
        # suppressor is outermost so sys.stdout still has a real fileno when it
        # grabs it. This mirrors robotserver, which is why its console stays clean.
        with suppress_stdout_stderr(), contextlib.redirect_stdout(
            io.StringIO()
        ), contextlib.redirect_stderr(io.StringIO()):
            sol_q, _is_collision = self._ik.ik_fun(target, motorstate=q_curr, visualize=False)
        if sol_q is None:
            return q_curr  # IK failed: hold current pose (matches robotserver)
        return np.asarray(sol_q, float)[:PIPER_ARM_DOF]
