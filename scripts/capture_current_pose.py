#!/usr/bin/env python3
import argparse
import logging

import yaml

from piper_teleop.config import config
from piper_teleop.robot_server.core.robot_interface import RobotInterface
from piper_teleop.utils import get_absolute_path


def parse_args():
    parser = argparse.ArgumentParser(description="Capture the current robot end-effector pose(s) to a deposit pose file.")
    parser.add_argument(
        "--output",
        default=config.deposit_pose_file,
        help="Path to the deposit pose file, relative to the repo root.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")

    robot_interface = RobotInterface(config)
    output_path = get_absolute_path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not robot_interface.connect():
        raise RuntimeError("Could not connect to any robot arms to capture the current pose.")

    payload = {}
    try:
        if robot_interface.left_arm_connected:
            payload["left"] = {"transform": robot_interface.get_end_effector_transform("left").tolist()}
        if robot_interface.right_arm_connected:
            payload["right"] = {"transform": robot_interface.get_end_effector_transform("right").tolist()}

        if not payload:
            raise RuntimeError("No connected arms were available to capture a pose.")

        with open(output_path, "w") as handle:
            yaml.safe_dump(payload, handle, sort_keys=False)

        logging.info("Saved current pose to %s", output_path)
    finally:
        robot_interface.disconnect(return_to_initial=False)


if __name__ == "__main__":
    main()
