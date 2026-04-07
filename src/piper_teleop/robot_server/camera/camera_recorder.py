from piper_teleop.robot_server.camera import (
    CameraConfig,
    CameraMode,
    SharedCameraData,
    create_camera,
)


class CameraRecorder:

    def __init__(self, configs: list[CameraConfig], shared_data: SharedCameraData):
        self.cameras = []

        for config in configs:
            if config.mode == CameraMode.RECORDING or config.mode == CameraMode.HYBRID:
                self.cameras.append(create_camera(config))

        self.shared_data = shared_data
