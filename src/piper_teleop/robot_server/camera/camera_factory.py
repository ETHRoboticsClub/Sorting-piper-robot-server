from piper_teleop.robot_server.camera.camera import Camera
from piper_teleop.robot_server.camera.camera_config import CameraBackend, CameraConfig, CameraType
from piper_teleop.robot_server.camera.monocular_camera import MonocularCamera
from piper_teleop.robot_server.camera.realsense_monocular_camera import RealSenseMonocularCamera
from piper_teleop.robot_server.camera.stereo_camera import StereoCamera


def create_camera(config: CameraConfig) -> Camera:
    if config.backend == CameraBackend.REALSENSE:
        if config.type != CameraType.MONOCULAR:
            raise ValueError("The RealSense backend currently supports only monocular camera configs.")
        return RealSenseMonocularCamera(config)

    if config.type == CameraType.STEREO:
        return StereoCamera(config)
    if config.type == CameraType.MONOCULAR:
        return MonocularCamera(config)

    raise ValueError(f"unsupported camera type: {config.type.value}")
