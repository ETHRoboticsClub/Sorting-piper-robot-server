import asyncio
import logging
import os

import cv2
import numpy as np
from dotenv import load_dotenv
from tactile_teleop_sdk import TactileAPI

from piper_teleop.robot_server.camera.camera import Camera
from piper_teleop.robot_server.camera.camera_config import CameraConfig, CameraMode, CameraType
from piper_teleop.robot_server.camera.camera_factory import create_camera
from piper_teleop.robot_server.camera.camera_shared_data import SharedCameraData

# Load environment variables from the project root
load_dotenv()

class CameraStreamer:

    def __init__(
        self,
        configs: list[CameraConfig],
        shared_data: SharedCameraData = None,
        show_camera_feeds: bool = False,
    ):
        self.logger = logging.getLogger(__name__)
        self.api = TactileAPI(api_key=os.getenv("TACTILE_API_KEY"))
        self.cameras = []
        self.shared_data = shared_data
        self.tasks = []
        self.is_running = False
        self.show_camera_feeds = show_camera_feeds
        self.preview_frames: dict[str, np.ndarray] = {}
        self.preview_window_name = "Robotserver Camera Preview"

        for config in configs:
            self.cameras.append(create_camera(config))

    def _build_preview_panel(self, camera_name: str, frame: np.ndarray) -> np.ndarray:
        preview = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        label_height = 32
        panel = cv2.copyMakeBorder(preview, label_height, 0, 0, 0, cv2.BORDER_CONSTANT, value=(24, 24, 24))
        cv2.putText(
            panel,
            camera_name,
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return panel

    def _render_preview_grid(self):
        if not self.preview_frames:
            return

        panels = [self.preview_frames[name] for name in sorted(self.preview_frames)]
        max_height = max(panel.shape[0] for panel in panels)
        max_width = max(panel.shape[1] for panel in panels)

        normalized = []
        for panel in panels:
            height_pad = max_height - panel.shape[0]
            width_pad = max_width - panel.shape[1]
            normalized.append(
                cv2.copyMakeBorder(panel, 0, height_pad, 0, width_pad, cv2.BORDER_CONSTANT, value=(0, 0, 0))
            )

        if len(normalized) == 1:
            grid = normalized[0]
        else:
            cols = 2
            rows = []
            for index in range(0, len(normalized), cols):
                row_panels = normalized[index : index + cols]
                if len(row_panels) < cols:
                    filler = np.zeros((max_height, max_width, 3), dtype=np.uint8)
                    row_panels.append(filler)
                rows.append(np.hstack(row_panels))
            grid = np.vstack(rows)

        cv2.imshow(self.preview_window_name, grid)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            self.logger.info("Camera preview requested shutdown")
            self.is_running = False

    def _show_frame(self, camera: Camera, frame):
        if not self.show_camera_feeds or frame is None:
            return

        if camera.type == CameraType.STEREO:
            left_frame, right_frame = frame
            self.preview_frames[f"{camera.name} [left]"] = self._build_preview_panel(f"{camera.name} [left]", left_frame)
            self.preview_frames[f"{camera.name} [right]"] = self._build_preview_panel(f"{camera.name} [right]", right_frame)
        else:
            self.preview_frames[camera.name] = self._build_preview_panel(camera.name, frame)

        self._render_preview_grid()

    async def _run_camera(self, camera: Camera):
        """Run a camera to capture frames and stream them to the Tactile API or save them to shared memory."""
        camera.init_camera()

        if camera.mode == CameraMode.STREAMING or camera.mode == CameraMode.HYBRID:
            await self.api.connect_camera_streamer(camera.frame_height, camera.get_cropped_width())

        while self.is_running:
            try:
                if camera.type == CameraType.STEREO:
                    left_frame, right_frame, cropped_left, cropped_right = await camera.capture_frame()
                    if left_frame is None or right_frame is None or cropped_left is None or cropped_right is None:
                        continue
                    self._show_frame(camera, (left_frame, right_frame))
                    if camera.mode == CameraMode.STREAMING or camera.mode == CameraMode.HYBRID:
                        await self.api.send_stereo_frame(cropped_left, cropped_right)
                    if camera.mode == CameraMode.RECORDING or camera.mode == CameraMode.HYBRID:
                        self.shared_data.copy(camera.name, left_frame)

                elif camera.type == CameraType.MONOCULAR:
                    frame = await camera.capture_frame()
                    if frame is None:
                        continue
                    self._show_frame(camera, frame)
                    if camera.mode == CameraMode.STREAMING or camera.mode == CameraMode.HYBRID:
                        await self.api.send_single_frame(frame)
                    if camera.mode == CameraMode.RECORDING or camera.mode == CameraMode.HYBRID:
                        self.shared_data.copy(camera.name, frame)

            except Exception as e:
                self.logger.error(f'error streaming camera "{camera.name}": {e}')
                await asyncio.sleep(0.1)

    async def start(self, room_name: str, participant_name: str):
        """Starts the camera streamer and waits for all camera jobs to complete."""
        if self.is_running:
            self.logger.info("Camera streamer already running")
            return
        self.logger.info("Starting camera streamer...")
        self.is_running = True

        self.tasks = [asyncio.create_task(self._run_camera(camera)) for camera in self.cameras]
        self.logger.info("Camera streamer started.")

        await asyncio.gather(*self.tasks)

    async def stop(self, timeout: float = 5.0):
        """Stops the running cameras and waits for them to complete."""
        if not self.is_running:
            return

        self.logger.info("Stopping all cameras...")
        self.is_running = False

        for task in self.tasks:
            task.cancel()

        try:
            await asyncio.wait_for(asyncio.gather(*self.tasks), timeout=timeout)
        except asyncio.CancelledError:
            self.logger.info("Camera tasks cancelled")
        except asyncio.TimeoutError:
            self.logger.warning(f"Camera tasks did not stop within {timeout}s")
        except Exception as e:
            self.logger.error(f"Error stopping camera streamer tasks: {e}")

        for camera in self.cameras:
            camera.stop_camera()

        if self.show_camera_feeds:
            self.preview_frames.clear()
            cv2.destroyAllWindows()

        self.tasks = []
        self.logger.info("Camera streamer stopped.")
