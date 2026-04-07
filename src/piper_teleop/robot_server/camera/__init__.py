from importlib import import_module

__all__ = [
    "Camera",
    "CameraBackend",
    "CameraConfig",
    "CameraMode",
    "CameraStreamer",
    "CameraType",
    "MonocularCamera",
    "RealSenseMonocularCamera",
    "SharedCameraData",
    "StereoCamera",
    "create_camera",
    "from_config",
]

_LAZY_IMPORTS = {
    "Camera": (".camera", "Camera"),
    "CameraBackend": (".camera_config", "CameraBackend"),
    "CameraConfig": (".camera_config", "CameraConfig"),
    "CameraMode": (".camera_config", "CameraMode"),
    "CameraStreamer": (".camera_streamer", "CameraStreamer"),
    "CameraType": (".camera_config", "CameraType"),
    "MonocularCamera": (".monocular_camera", "MonocularCamera"),
    "RealSenseMonocularCamera": (".realsense_monocular_camera", "RealSenseMonocularCamera"),
    "SharedCameraData": (".camera_shared_data", "SharedCameraData"),
    "StereoCamera": (".stereo_camera", "StereoCamera"),
    "create_camera": (".camera_factory", "create_camera"),
    "from_config": (".camera_config", "from_config"),
}


def __getattr__(name: str):
    if name not in _LAZY_IMPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attribute_name = _LAZY_IMPORTS[name]
    module = import_module(module_name, __name__)
    value = getattr(module, attribute_name)
    globals()[name] = value
    return value
