import asyncio

import cv2
import numpy as np
import pyrealsense2 as rs

from piper_teleop.robot_server.camera.camera_config import CameraConfig
from piper_teleop.robot_server.camera.monocular_camera import MonocularCamera
from piper_teleop.robot_server.camera.realsense_device import resolve_realsense_device


class RealSenseMonocularCamera(MonocularCamera):
    """Monocular camera backed by the Intel RealSense SDK instead of Video4Linux indices."""

    def __init__(self, config: CameraConfig):
        super().__init__(config)
        self.pipeline = None
        self.pipeline_profile = None
        self.resolved_serial_number: str | None = None

    def is_connected(self) -> bool:
        return self.pipeline is not None

    def init_camera(self) -> None:
        if self.pipeline is not None:
            return

        device = resolve_realsense_device(self.serial_number)

        pipeline = rs.pipeline()
        pipeline_config = rs.config()
        pipeline_config.enable_device(device.serial_number)
        pipeline_config.enable_stream(
            rs.stream.color,
            self.capture_frame_width,
            self.capture_frame_height,
            rs.format.rgb8,
            self.fps,
        )

        self.pipeline_profile = pipeline.start(pipeline_config)
        self.pipeline = pipeline
        self.resolved_serial_number = device.serial_number
        self.logger.info(
            'realsense camera "%s" initialised with serial %s using %sx%s@%s %s',
            self.name,
            self.resolved_serial_number,
            self.capture_frame_width,
            self.capture_frame_height,
            self.fps,
            rs.format.rgb8,
        )

    async def capture_frame(self) -> np.ndarray:
        if self.pipeline is None:
            raise RuntimeError(f'RealSense camera "{self.name}" is not initialised')

        loop = asyncio.get_event_loop()
        frames = await loop.run_in_executor(None, lambda: self.pipeline.wait_for_frames(timeout_ms=1000))
        color_frame = frames.get_color_frame()

        if color_frame is None:
            self.logger.warning("cannot receive RealSense color frame. retrying...")
            await asyncio.sleep(0.1)
            return None

        frame_rgb = np.asanyarray(color_frame.get_data()).copy()
        if frame_rgb.ndim == 3 and frame_rgb.shape[2] == 3:
            stream_format = color_frame.profile.as_video_stream_profile().format()
            if stream_format == rs.format.bgr8:
                frame_rgb = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB)

        if frame_rgb.shape[:2] != (self.frame_height, self.frame_width):
            frame_rgb = cv2.resize(frame_rgb, (self.frame_width, self.frame_height))

        return frame_rgb

    def stop_camera(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            self.pipeline_profile = None
            self.logger.info(
                'realsense camera "%s" with serial %s stopped',
                self.name,
                self.resolved_serial_number,
            )
