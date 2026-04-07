#!/usr/bin/env python3
import argparse
import asyncio

import cv2

from piper_teleop.config import config
from piper_teleop.robot_server.camera.camera_factory import create_camera


def parse_args():
    parser = argparse.ArgumentParser(description="Show a live preview window for a configured camera.")
    parser.add_argument(
        "--camera",
        default=None,
        help="Configured camera name to preview. Defaults to all configured cameras.",
    )
    return parser.parse_args()


async def main():
    args = parse_args()

    camera_configs = []
    if args.camera is None:
        if not config.camera_configs:
            raise RuntimeError("No cameras configured in config.yaml")
        camera_configs = list(config.camera_configs)
    else:
        for candidate in config.camera_configs:
            if candidate.name == args.camera:
                camera_configs = [candidate]
                break
        if not camera_configs:
            available = ", ".join(cam.name for cam in config.camera_configs)
            raise RuntimeError(f'Camera "{args.camera}" not found. Available: {available}')

    cameras = []
    for camera_config in camera_configs:
        camera = create_camera(camera_config)
        camera.init_camera()
        cameras.append(camera)

    try:
        while True:
            for camera in cameras:
                frame = await camera.capture_frame()
                if frame is None:
                    continue

                window_name = f"Camera Preview: {camera.name}"
                cv2.imshow(window_name, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
    finally:
        for camera in cameras:
            camera.stop_camera()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    asyncio.run(main())
