"""
Kinematics utilities for the SO100 robot.
Contains forward and inverse kinematics solvers using PyBullet.
"""

import logging
import math
import os
import tempfile
import xml.etree.ElementTree as ET

import casadi
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer

logger = logging.getLogger(__name__)


def matrix_to_xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix


def quaternion_from_matrix(matrix):
    qw = math.sqrt(1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2]) / 2
    qx = (matrix[2, 1] - matrix[1, 2]) / (4 * qw)
    qy = (matrix[0, 2] - matrix[2, 0]) / (4 * qw)
    qz = (matrix[1, 0] - matrix[0, 1]) / (4 * qw)
    return np.array([qx, qy, qz, qw])


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _make_mesh_paths_absolute(urdf_path: str) -> str:
    """
    Convert relative mesh paths in URDF to absolute paths.
    Returns path to a temporary URDF file with absolute paths.
    """
    urdf_abs_path = os.path.abspath(urdf_path)
    urdf_dir = os.path.dirname(urdf_abs_path)

    # Read the original URDF
    with open(urdf_abs_path, "r") as f:
        urdf_content = f.read()

    # Replace relative mesh paths with absolute paths
    # Look for patterns like: filename="assets/something.STL"
    import re

    def replace_mesh_path(match):
        relative_path = match.group(1)
        if not os.path.isabs(relative_path):
            # Convert to absolute path
            absolute_path = os.path.join(urdf_dir, relative_path)
            return f'filename="{absolute_path}"'
        return match.group(0)  # Keep absolute paths as-is

    # Replace all mesh filename attributes
    urdf_content = re.sub(r'filename="([^"]+\.STL)"', replace_mesh_path, urdf_content, flags=re.IGNORECASE)

    # Create temporary file with absolute paths
    temp_fd, temp_path = tempfile.mkstemp(suffix=".urdf", text=True)
    try:
        with os.fdopen(temp_fd, "w") as f:
            f.write(urdf_content)
        return temp_path
    except Exception:
        os.close(temp_fd)
        raise


class Arm_IK:
    def __init__(self, urdf_path: str, ground_height: float = 0.0, collision_space_urdf: str | None = None):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        # Create temporary URDF with absolute mesh paths
        temp_urdf_path = _make_mesh_paths_absolute(urdf_path)

        try:
            # Load URDF with absolute paths
            self.robot = pin.RobotWrapper.BuildFromURDF(temp_urdf_path)

            # Build geometry model
            self.geom_model = pin.buildGeomFromUrdf(self.robot.model, temp_urdf_path, pin.GeometryType.COLLISION)

        finally:
            # Clean up temporary file
            if os.path.exists(temp_urdf_path):
                os.unlink(temp_urdf_path)

        for joint in self.robot.model.names:
            print("Joint name:", joint)

        for link in self.robot.model.frames:
            print("Link name:", link.name)

        # Single-arm model: lock gripper mimic joints in IK optimization.
        self.mixed_jointsToLockIDs = ["joint7", "joint8"]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        # Add environment geometry used by collision checks.
        self.ground_height = ground_height
        self.environment_geom_names: set[str] = set()
        self.environment_boxes: list[tuple[str, np.ndarray, list[float]]] = []
        self.exclude_from_ground_collision = ["base_link_0"]
        self._add_ground_plane()
        self._add_collision_space_from_urdf(collision_space_urdf)
        self._init_collision_pairs()

        self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, 0.0) #self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, 0) original, adjusted to keep jaw at 0 , perhaps a mounting issue
        # Transform from joint6 to end-effector gripper coordinates
        self.second_matrix = create_transformation_matrix(0.13, 0.0, 0.0, 0, 0, 0)
        self.last_matrix = np.dot(self.first_matrix, self.second_matrix)
        q = quaternion_from_matrix(self.last_matrix)
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "ee",
                self.reduced_robot.model.getJointId("joint6"),
                pin.SE3(
                    pin.Quaternion(q[3], q[0], q[1], q[2]),
                    np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        self.geometry_data = pin.GeometryData(self.geom_model)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)

        # Initialize the Meshcat visualizer for visualization
        self.vis = MeshcatVisualizer(
            self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model
        )
        self.vis.initViewer(open=True)
        self.vis.loadViewerModel("pinocchio")
        # Keep frame clutter off in normal use; we visualize explicit EE targets below.
        self.vis.displayFrames(False)
        self.vis.display(pin.neutral(self.reduced_robot.model))
        self._visualize_environment_boxes()

        # Enable display of end effector target frames with short axis lengths
        frame_viz_names = ["ee_target_1"]
        FRAME_AXIS_POSITIONS = (
            np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        FRAME_AXIS_COLORS = (
            np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        axis_length = 0.1
        axis_width = 10
        for frame_viz_name in frame_viz_names:
            self.vis.viewer[frame_viz_name].set_object(
                mg.LineSegments(
                    mg.PointsGeometry(
                        position=axis_length * FRAME_AXIS_POSITIONS,
                        color=FRAME_AXIS_COLORS,
                    ),
                    mg.LineBasicMaterial(
                        linewidth=axis_width,
                        vertexColors=True,
                    ),
                )
            )

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Create generic symbolic variables
        cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, cq)

        # Create a single-arm solver
        self.solver = self._create_single_ik_solver(cq)

    def _create_single_ik_solver(self, cq):
        """Create and configure single-arm IK solver."""
        cTf = casadi.SX.sym("tf", 4, 4)

        gripper_id = self.reduced_robot.model.getFrameId("ee")
        error_expr = casadi.vertcat(cpin.log6(self.cdata.oMf[gripper_id].inverse() * cpin.SE3(cTf)).vector)
        error_fun = casadi.Function("error", [cq, cTf], [error_expr])

        # Define the optimization problem
        opti = casadi.Opti()
        var_q = opti.variable(self.reduced_robot.model.nq)
        param_tf = opti.parameter(4, 4)

        error_vec = error_fun(var_q, param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]

        weight_position = 1.0
        weight_orientation = 0.1

        total_cost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(weight_orientation * ori_error)
        regularization = casadi.sumsqr(var_q)

        # Add constraints
        opti.subject_to(
            opti.bounded(
                self.reduced_robot.model.lowerPositionLimit, var_q, self.reduced_robot.model.upperPositionLimit
            )
        )

        # Set the objective
        opti.minimize(20 * total_cost + 0.01 * regularization)

        # Configure the solver
        opts = {"ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-4}, "print_time": False}
        opti.solver("ipopt", opts)

        return {"opti": opti, "var_q": var_q, "param_tf": param_tf}

    def _init_collision_pairs(self):
        for i in range(self.geom_model.ngeoms):
            print("Geometry object:", self.geom_model.geometryObjects[i].name)

        # Conservative self-collision checks for a single arm, mirroring the previous
        # dual-arm strategy where only non-neighbor link groups were tested.
        geom_name_to_idx = {self.geom_model.geometryObjects[i].name: i for i in range(self.geom_model.ngeoms)}
        arm_geom_order = [
            "base_link_0",
            "link1_0",
            "link2_0",
            "link3_0",
            "link4_0",
            "link5_0",
            "link6_0",
            "gripper_base_0",
            "link7_0",
            "link8_0",
        ]
        arm_indices = [geom_name_to_idx[name] for name in arm_geom_order if name in geom_name_to_idx]
        if len(arm_indices) >= 9:
            for i in range(0, 3):
                for j in range(4, 9):
                    geom1_idx = arm_indices[i]
                    geom2_idx = arm_indices[j]
                    if abs(i - j) > 1:
                        self.geom_model.addCollisionPair(pin.CollisionPair(geom1_idx, geom2_idx))

        env_indices = [
            i
            for i in range(self.geom_model.ngeoms)
            if self.geom_model.geometryObjects[i].name in self.environment_geom_names
        ]
        for i in range(self.geom_model.ngeoms):
            if i in env_indices:
                continue
            geom_name = self.geom_model.geometryObjects[i].name
            if geom_name in self.exclude_from_ground_collision:
                logger.debug("Excluding %s from environment collision detection", geom_name)
                continue
            for env_idx in env_indices:
                self.geom_model.addCollisionPair(pin.CollisionPair(i, env_idx))

    def _add_ground_plane(self):
        """Add a ground plane geometry to the collision model for ground collision detection."""
        ground_size = [10.0, 10.0, 0.1]
        ground_pose = pin.SE3.Identity()
        ground_pose.translation = np.array([0.0, 0.0, self.ground_height - 0.05])

        ground_geometry = pin.GeometryObject("ground_plane", 0, pin.hppfcl.Box(*ground_size), ground_pose)
        self.geom_model.addGeometryObject(ground_geometry)
        self.environment_geom_names.add("ground_plane")
        ground_tf = np.eye(4, dtype=float)
        ground_tf[:3, 3] = ground_pose.translation
        self.environment_boxes.append(("ground_plane", ground_tf, ground_size))
        logger.info(f"Added ground plane at height {self.ground_height}")

    def _add_collision_space_from_urdf(self, collision_space_urdf: str | None):
        """Load collision-only box geometries from an environment URDF."""
        if not collision_space_urdf:
            return
        env_urdf = os.path.abspath(collision_space_urdf)
        if not os.path.exists(env_urdf):
            logger.warning("Collision space URDF not found: %s", env_urdf)
            return

        try:
            tree = ET.parse(env_urdf)
            root = tree.getroot()
        except Exception as exc:
            logger.error("Failed to parse collision space URDF %s: %s", env_urdf, exc)
            return

        joint_origins: dict[str, np.ndarray] = {}
        for joint in root.findall("joint"):
            child = joint.find("child")
            origin = joint.find("origin")
            if child is None or child.get("link") is None:
                continue
            xyz = np.array([0.0, 0.0, 0.0], dtype=float)
            rpy = np.array([0.0, 0.0, 0.0], dtype=float)
            if origin is not None:
                if origin.get("xyz"):
                    xyz = np.array([float(v) for v in origin.get("xyz").split()], dtype=float)
                if origin.get("rpy"):
                    rpy = np.array([float(v) for v in origin.get("rpy").split()], dtype=float)
            tf = np.eye(4, dtype=float)
            tf[:3, :3] = _rpy_to_matrix(rpy[0], rpy[1], rpy[2])
            tf[:3, 3] = xyz
            joint_origins[child.get("link")] = tf

        added = 0
        for link in root.findall("link"):
            link_name = link.get("name")
            if not link_name:
                continue
            link_tf = joint_origins.get(link_name, np.eye(4, dtype=float))
            for idx, collision in enumerate(link.findall("collision")):
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                box = geometry.find("box")
                if box is None or not box.get("size"):
                    continue
                size = [float(v) for v in box.get("size").split()]

                local_tf = np.eye(4, dtype=float)
                origin = collision.find("origin")
                if origin is not None:
                    if origin.get("xyz"):
                        local_tf[:3, 3] = [float(v) for v in origin.get("xyz").split()]
                    if origin.get("rpy"):
                        rpy = [float(v) for v in origin.get("rpy").split()]
                        local_tf[:3, :3] = _rpy_to_matrix(rpy[0], rpy[1], rpy[2])

                world_tf = link_tf @ local_tf
                se3 = pin.SE3(world_tf[:3, :3], world_tf[:3, 3])
                geom_name = f"env_{link_name}_{idx}"
                geom_obj = pin.GeometryObject(geom_name, 0, pin.hppfcl.Box(*size), se3)
                self.geom_model.addGeometryObject(geom_obj)
                self.environment_geom_names.add(geom_name)
                tf = np.eye(4, dtype=float)
                tf[:3, :3] = world_tf[:3, :3]
                tf[:3, 3] = world_tf[:3, 3]
                self.environment_boxes.append((geom_name, tf, size))
                added += 1

        logger.info("Added %d collision-space geometries from %s", added, env_urdf)

    def _visualize_environment_boxes(self):
        """Render collision-space boxes in Meshcat for debugging."""
        for name, tf, size in self.environment_boxes:
            node_name = f"collision_space/{name}"
            self.vis.viewer[node_name].set_object(
                mg.Box(size),
                mg.MeshBasicMaterial(color=0x3A7DFF, opacity=0.25, transparent=True),
            )
            self.vis.viewer[node_name].set_transform(tf)

    def ik_fun(self, target_pose, gripper=0, motorstate=None, motorV=None, visualize=True):
        opti = self.solver["opti"]
        var_q = self.solver["var_q"]
        param_tf = self.solver["param_tf"]

        gripper_vec = np.array([gripper / 2.0, -gripper / 2.0])
        if motorstate is not None:
            self.init_data = motorstate
        opti.set_initial(var_q, self.init_data)

        if visualize:
            self.vis.viewer["ee_target_1"].set_transform(target_pose)

        opti.set_value(param_tf, target_pose)

        try:
            opti.solve_limited()
            sol_q = opti.value(var_q)

            if self.init_data is not None:
                max_diff = max(abs(self.history_data - sol_q))
                self.init_data = sol_q
                if max_diff > 30.0 / 180.0 * 3.1415:
                    self.init_data = np.zeros(self.reduced_robot.model.nq)
            else:
                self.init_data = sol_q
            self.history_data = sol_q

            if visualize:
                # print("sol_q:", sol_q)
                self.vis.display(sol_q)

            is_collision = self.check_collision(sol_q, gripper=gripper_vec)
            return sol_q, is_collision

        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")
            return None, False

    def check_collision(self, q, gripper=np.array([0, 0])):
        """Check for collisions including self-collision and ground plane collision."""
        full_q = np.concatenate([q[0:6], gripper], axis=0)
        pin.forwardKinematics(self.robot.model, self.robot.data, full_q)
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        pin.computeCollisions(self.geom_model, self.geometry_data, False)

        has_collision = False
        first_pair = None
        for i, pair in enumerate(self.geom_model.collisionPairs):
            if self.geometry_data.collisionResults[i].isCollision():
                has_collision = True
                if first_pair is None:
                    g1 = self.geom_model.geometryObjects[pair.first].name
                    g2 = self.geom_model.geometryObjects[pair.second].name
                    first_pair = (g1, g2)
        if first_pair:
            logger.debug("Collision detected: %s <-> %s", first_pair[0], first_pair[1])
        return has_collision

    def get_dist(self, q, xyz):
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        dist = math.sqrt(
            pow((xyz[0] - self.reduced_robot.data.oMi[6].translation[0]), 2)
            + pow((xyz[1] - self.reduced_robot.data.oMi[6].translation[1]), 2)
            + pow((xyz[2] - self.reduced_robot.data.oMi[6].translation[2]), 2)
        )
        return dist

    def get_pose(self, q):
        index = 6
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        end_pose = create_transformation_matrix(
            self.reduced_robot.data.oMi[index].translation[0],
            self.reduced_robot.data.oMi[index].translation[1],
            self.reduced_robot.data.oMi[index].translation[2],
            math.atan2(
                self.reduced_robot.data.oMi[index].rotation[2, 1], self.reduced_robot.data.oMi[index].rotation[2, 2]
            ),
            math.asin(-self.reduced_robot.data.oMi[index].rotation[2, 0]),
            math.atan2(
                self.reduced_robot.data.oMi[index].rotation[1, 0], self.reduced_robot.data.oMi[index].rotation[0, 0]
            ),
        )
        end_pose = np.dot(end_pose, self.last_matrix)
        return matrix_to_xyzrpy(end_pose)
