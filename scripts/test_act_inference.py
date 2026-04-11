import argparse
import asyncio
import logging
from pathlib import Path

import cv2
import numpy as np

from piper_teleop.config import config
from piper_teleop.robot_server.camera.camera_config import CameraType

logger = logging.getLogger(__name__)


async def capture_camera_observations(show_cameras: bool) -> dict[str, np.ndarray]:
    from piper_teleop.robot_server.camera.camera_factory import create_camera

    cameras = [create_camera(camera_config) for camera_config in config.camera_configs]
    captured: dict[str, np.ndarray] = {}

    try:
        for camera in cameras:
            if camera.type == CameraType.STEREO:
                raise ValueError(
                    f'Camera "{camera.name}" is configured as stereo. '
                    "This smoke test currently supports only monocular policy inputs."
                )

            camera.init_camera()
            frame = await camera.capture_frame()
            if frame is None:
                raise RuntimeError(f'Failed to capture a frame from camera "{camera.name}"')
            captured[f"observation.images.{camera.name}"] = frame

            if show_cameras:
                cv2.imshow(camera.name, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

        if show_cameras and captured:
            logger.info("Showing captured frames. Press any key in an image window to continue.")
            cv2.waitKey(0)

        return captured
    finally:
        for camera in cameras:
            try:
                camera.stop_camera()
            except Exception as exc:
                logger.warning("Failed to stop camera %s cleanly: %s", camera.name, exc)
        if show_cameras:
            cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one live ACT inference pass without moving the robot.")
    parser.add_argument(
        "--policy-path",
        default=config.policy_path,
        help="Path to the trained policy pretrained_model directory.",
    )
    parser.add_argument(
        "--policy-repo-id",
        default=config.policy_repo_id,
        help="Dataset root or repo_id used for the policy metadata and stats.",
    )
    parser.add_argument("--device", default="cpu", help="Torch device for inference, e.g. cpu or cuda.")
    parser.add_argument(
        "--no-robot",
        action="store_true",
        help="Skip robot connection and use the default zeroed joint observation fallback.",
    )
    parser.add_argument(
        "--show-cameras",
        action="store_true",
        help="Show the captured camera frames before running the policy.",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", force=True)

    from piper_teleop.robot_server.core import RobotInterface
    from piper_teleop.robot_server.lerobot_policy import LerobotPolicy

    if not config.camera_configs:
        raise RuntimeError("No cameras are configured. ACT inference needs at least one configured policy camera.")

    policy_path = Path(args.policy_path).expanduser().resolve()
    if not policy_path.exists():
        raise FileNotFoundError(f"Policy path does not exist: {policy_path}")

    robot = RobotInterface(config)
    if not args.no_robot:
        connected = robot.connect()
        if not connected:
            raise RuntimeError("Failed to connect to the robot. Re-run with --no-robot for a camera-only smoke test.")

    try:
        cams = asyncio.run(capture_camera_observations(show_cameras=args.show_cameras))
        obs_dict = robot.get_observation()
        if robot.left_arm_connected and not robot.right_arm_connected:
            single_arm_side = "left"
        elif robot.right_arm_connected and not robot.left_arm_connected:
            single_arm_side = "right"
        else:
            single_arm_side = "right"

        policy = LerobotPolicy(policy_path=str(policy_path), repo_id=args.policy_repo_id, device=args.device)
        dict_left, dict_right = policy.predict(
            obs_dict["left"],
            obs_dict["right"],
            cams,
            single_arm_side=single_arm_side,
        )

        logger.info("Inference completed successfully.")
        logger.info("Captured cameras: %s", ", ".join(sorted(cams)))
        logger.info("Predicted left action: %s", dict_left if dict_left else "<empty>")
        logger.info("Predicted right action: %s", dict_right if dict_right else "<empty>")
    finally:
        robot.disconnect(return_to_initial=False)


if __name__ == "__main__":
    main()
