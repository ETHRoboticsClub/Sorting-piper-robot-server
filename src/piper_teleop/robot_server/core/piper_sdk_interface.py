# Piper SDK interface

import time
from typing import Any, Dict

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Is the piper_sdk installed: pip install piper_sdk")
    C_PiperInterface_V2: Any = None  # For type checking and docs

JOINT_LIMITS_RAD = {
    "min": [-2.6179, 0.0, -2.967, -1.745, -1.22, -1.5],
    "max": [2.6179, 3.14, 0.0, 1.745, 1.22, 1.5],
}
DEG_TO_RAD = 0.017444
RAD_TO_DEG = 1 / DEG_TO_RAD
GRIPPER_ANGLE_MAX = 0.07  # 70mm


class PiperSDKInterface:
    def __init__(self, port: str = "can0"):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed.")
        self.piper = C_PiperInterface_V2(port)
        self.piper.ConnectPort()
        while not self.piper.EnablePiper():
            time.sleep(0.01)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)

        # Get the min and max positions for each joint and gripper
        angel_status = self.piper.GetAllMotorAngleLimitMaxSpd()
        self.min_pos = [pos.min_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [0]
        self.max_pos = [pos.max_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [
            10
        ]  # Gripper max position in mm

    def set_joint_positions(self, positions):

        joint_angles = []
        for i, pos in enumerate(positions[:6]):
            min_rad, max_rad = JOINT_LIMITS_RAD["min"][i], JOINT_LIMITS_RAD["max"][i]
            clipped_pos = min(max(pos, min_rad), max_rad)
            pos_deg = clipped_pos * RAD_TO_DEG
            joint_angle = round(pos_deg * 1e3)  # Convert to millidegrees
            joint_angles.append(joint_angle)

        gripper_position = min(max(positions[6], 0.0), GRIPPER_ANGLE_MAX)
        gripper_position_int = round(gripper_position * 1e6)

        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper.JointCtrl(*joint_angles)
        self.piper.GripperCtrl(gripper_position_int, 1000, 0x01, 0)

    def get_status(self) -> Dict[str, Any]:
        joint_status = self.piper.GetArmJointMsgs()
        gripper = self.piper.GetArmGripperMsgs()
        gripper.gripper_state.grippers_angle

        joint_state = joint_status.joint_state
        obs_dict = {
            "joint_0.pos": (joint_state.joint_1 / 1000) * DEG_TO_RAD,
            "joint_1.pos": (joint_state.joint_2 / 1000) * DEG_TO_RAD,
            "joint_2.pos": (joint_state.joint_3 / 1000) * DEG_TO_RAD,
            "joint_3.pos": (joint_state.joint_4 / 1000) * DEG_TO_RAD,
            "joint_4.pos": (joint_state.joint_5 / 1000) * DEG_TO_RAD,
            "joint_5.pos": (joint_state.joint_6 / 1000) * DEG_TO_RAD,
        }
        obs_dict.update(
            {
                "joint_6.pos": gripper.gripper_state.grippers_angle / 1000000,
            }
        )

        return obs_dict

    def get_motor_currents(self) -> Dict[str, float]:
        """Per-joint load telemetry for the 6 arm joints, plus controller fault flags.

        The Piper has no torque sensors. Two proxies are exposed:

        * **Current / effort** from the high-speed feedback (current unit 0.001 A;
          effort = current x per-joint coefficient via ``cal_effort``). NOTE these read
          ~0 while the motors are **disabled** (no current through an unpowered motor);
          they only carry load once the arm is enabled and commanded.

        * **Fault flags** from the low-speed feedback -- ``collision_status`` and
          ``stall_status`` are the controller's OWN collision/stall decisions (trip
          thresholds set by the per-joint collision-protection level), plus
          ``driver_overcurrent``. These are valid regardless of enable state and are a
          more reliable collision signal than any current magnitude. Aggregated into
          ``any_collision`` / ``any_stall`` / ``any_overcurrent`` and ``arm_enabled``.

        Reads are non-blocking (SDK background thread caches the latest messages).
        The gripper motor is not part of this feedback.
        """
        out: Dict[str, float] = {}

        hi = self.piper.GetArmHighSpdInfoMsgs()
        for i in range(6):
            motor = getattr(hi, f"motor_{i + 1}", None)
            if motor is None:
                continue
            out[f"joint_{i}.current"] = float(motor.current) * 1e-3  # 0.001 A -> A
            try:
                out[f"joint_{i}.effort"] = float(motor.cal_effort())
            except Exception:
                out[f"joint_{i}.effort"] = 0.0

        any_collision = any_stall = any_overcurrent = False
        enabled = False
        lo = self.piper.GetArmLowSpdInfoMsgs()
        for i in range(6):
            motor = getattr(lo, f"motor_{i + 1}", None)
            foc = getattr(motor, "foc_status", None) if motor is not None else None
            if foc is None:
                continue
            col = bool(getattr(foc, "collision_status", False))
            stl = bool(getattr(foc, "stall_status", False))
            over = bool(getattr(foc, "driver_overcurrent", False))
            out[f"joint_{i}.collision"] = float(col)
            out[f"joint_{i}.stall"] = float(stl)
            any_collision = any_collision or col
            any_stall = any_stall or stl
            any_overcurrent = any_overcurrent or over
            enabled = enabled or bool(getattr(foc, "driver_enable_status", False))

        out["any_collision"] = float(any_collision)
        out["any_stall"] = float(any_stall)
        out["any_overcurrent"] = float(any_overcurrent)
        out["arm_enabled"] = float(enabled)
        return out

    def get_crash_protection_levels(self) -> Dict[str, int]:
        """Per-joint collision-protection level (0-8; 0 = off, higher = more sensitive)."""
        try:
            msg = self.piper.GetCrashProtectionLevelFeedback()
            fb = msg.crash_protection_level_feedback
            return {f"joint_{i}": int(getattr(fb, f"joint_{i + 1}_protection_level", 0)) for i in range(6)}
        except Exception:
            return {}

    def set_crash_protection_levels(self, levels) -> None:
        """Set the per-joint collision-protection level (0-8; 0 = off).

        Without this the controller does not detect collisions at all, so the
        ``collision_status`` fault flag never trips. ``levels`` is a single int applied
        to all six joints, or a 6-element sequence. Higher = more sensitive (trips at a
        lower load). This is a persistent controller setting, so set it once.
        """
        if isinstance(levels, (int, float)):
            levels = [int(levels)] * 6
        levels = [max(0, min(8, int(v))) for v in list(levels)[:6]]
        levels += [0] * (6 - len(levels))
        self.piper.CrashProtectionConfig(*levels)

    def get_end_effector_pose(self) -> Dict[str, float]:
        """
        Returns the current end-effector pose as a sequence of floats.

        Returns:
        Sequence[float]: (x, y, z, roll, pitch, yaw) in meters and radians.
        """
        pose = self.piper.GetArmEndPoseMsgs()
        x = pose.end_pose.X_axis * 1e-6  # Convert from mm to m
        y = pose.end_pose.Y_axis * 1e-6
        z = pose.end_pose.Z_axis * 1e-6
        roll = pose.end_pose.RX_axis * 1e-3 * DEG_TO_RAD
        pitch = pose.end_pose.RY_axis * 1e-3 * DEG_TO_RAD
        yaw = pose.end_pose.RZ_axis * 1e-3 * DEG_TO_RAD
        return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}

    def get_connection_status(self):
        return self.piper.get_connect_status()

    def disconnect(self):
        self.piper.DisconnectPort()
