"""
The main entry point for the teleoperation system.
"""

import argparse
import asyncio
import datetime
import logging
import multiprocessing as mp
from importlib.metadata import version

from packaging.version import Version

from piper_teleop.config import TelegripConfig, config
from piper_teleop.robot_server.camera import CameraStreamer, SharedCameraData
from piper_teleop.robot_server.camera.camera_config import CameraMode
from piper_teleop.robot_server.control_loop import ControlLoop

logger = logging.getLogger(__name__)


def _camera_process_wrapper(
    config: TelegripConfig,
    shared_data: SharedCameraData,
) -> None:
    """Wrapper to run camera streamer in a separate process with asyncio"""

    async def run_camera():
        camera_streamer = CameraStreamer(
            configs=config.camera_configs,
            shared_data=shared_data,
            show_camera_feeds=config.show_camera_feeds,
        )
        try:
            await camera_streamer.start(config.livekit_room, config.camera_streamer_participant)
        finally:
            await camera_streamer.stop()

    asyncio.run(run_camera())


def _control_process_wrapper(config: TelegripConfig, shared_data: SharedCameraData) -> None:
    """Wrapper to run control process in a separate process with asyncio"""
    asyncio.run(
        _run_control_process(
            config=config,
            shared_data=shared_data,
        )
    )


async def _run_control_process(config: TelegripConfig, shared_data: SharedCameraData) -> None:
    """
    Run the controll process (receiving, processing and sending commands to the robot)
    """
    control_loop = ControlLoop(config=config, shared_data=shared_data)
    try:
        control_loop_task = asyncio.create_task(control_loop.run())
        await asyncio.gather(control_loop_task)
    finally:
        await control_loop.stop()


def main():
    parser = argparse.ArgumentParser(description="Robot Server - Tactile Robotics Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--vis", action="store_true", help="Enable visualization")
    parser.add_argument(
        "--keyboard", action="store_true", default=False, help="Enable keyboard control (defaults to False)"
    )
    parser.add_argument(
        "--gamepad",
        action="store_true",
        default=False,
        help="Enable USB gamepad / joystick control (6-DOF EE + gripper); skips VR like --keyboard",
    )
    parser.add_argument(
        "--ee-world",
        action="store_true",
        dest="ee_world",
        help="With --gamepad: apply rotations in world/base frame (default: end-effector frame)",
    )
    parser.add_argument("--record", action="store_true", help="Enable recording")
    parser.add_argument("--resume", action="store_true", help="Resume recording")
    parser.add_argument("--repo-id", type=str, default="default-piper", help="repo_id for dataset storage")
    parser.add_argument("--leader", action="store_true", help="Enable Leader-Follower setup")
    parser.add_argument("--policy", action="store_true", help="Enable policy control")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error", "critical"],
        help="Set logging level",
    )
    parser.add_argument(
        "--no-cameras",
        action="store_true",
        help="Do not start any camera capture (useful for keyboard + --vis when cameras are unavailable)",
    )
    parser.add_argument(
        "--show-cameras",
        action="store_true",
        help="Show local preview windows for configured cameras while robotserver is running",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()), format="%(asctime)s - %(message)s", datefmt="%H:%M:%S"
    )

    config.enable_robot = not args.no_robot
    config.enable_visualization = args.vis
    config.record = args.record
    config.resume = args.resume
    config.repo_id = args.repo_id
    config.enable_keyboard = args.keyboard
    config.enable_gamepad = args.gamepad
    if args.ee_world:
        assert args.gamepad, "--ee-world requires --gamepad"
    config.gamepad_rotation_world_frame = args.ee_world
    config.use_leader = args.leader
    config.use_policy = args.policy
    config.show_camera_feeds = args.show_cameras
    config.root = config.root / f'{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}'

    if args.no_cameras:
        config.camera_configs = []

    if config.use_policy:
        assert not args.keyboard, "Keyboard control cannot be used when policy control is enabled"
        assert not args.record, "Recording cannot be used when policy control is enabled"
        assert not args.resume, "Resume recording cannot be used when policy control is enabled"
        assert not args.leader, "Leader control cannot be used when policy control is enabled"

    num_local_modes = int(args.keyboard) + int(args.gamepad)
    assert num_local_modes <= 1, "Use only one of --keyboard or --gamepad"

    if config.record:
        num_recording_cams = 0
        for cam_config in config.camera_configs:
            if cam_config.mode == CameraMode.RECORDING or cam_config.mode == CameraMode.HYBRID:
                num_recording_cams += 1
        assert num_recording_cams > 0, "There must be at least one camera in recording or hybrid mode"
        assert Version(version("lerobot")) >= Version("0.4.0"), f"lerobot >= 0.4.0 required"

    shared_data = SharedCameraData(configs=config.camera_configs)

    logger.info("Initializing server components...")
    try:
        # running background (daemon) processes
        logger.info("Starting camera streamer and control loop in parallel...")
        logger.info(f"Camera participant: {config.camera_streamer_participant}")
        logger.info(f"Controller participant: {config.controllers_processing_participant}")

        # run camera task as process
        camera_process = mp.Process(
            target=_camera_process_wrapper,
            args=(
                config,
                shared_data,
            ),
        )
        camera_process.start()

        # run control loop as process
        control_process = mp.Process(
            target=_control_process_wrapper,
            args=(
                config,
                shared_data,
            ),
        )
        control_process.start()

        # Wait for processes to complete
        camera_process.join()
        control_process.join()

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")

    finally:
        logger.info("Shutting down...")

        # Terminate and wait for processes to finish
        if "camera_process" in locals() and camera_process.is_alive():
            logger.info("Terminating camera process...")
            camera_process.terminate()
            camera_process.join(timeout=5)

        if "control_process" in locals() and control_process.is_alive():
            logger.info("Terminating control process...")
            control_process.terminate()
            control_process.join(timeout=5)

        logger.info("All processes stopped")


def main_cli():
    try:
        main()
    except KeyboardInterrupt:
        logging.info("👋 tactile teleop stopped")


if __name__ == "__main__":
    main_cli()
