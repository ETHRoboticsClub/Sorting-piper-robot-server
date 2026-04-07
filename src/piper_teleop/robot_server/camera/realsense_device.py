from dataclasses import dataclass

import pyrealsense2 as rs


@dataclass(frozen=True)
class RealSenseDeviceInfo:
    serial_number: str
    name: str | None = None
    product_line: str | None = None
    usb_type: str | None = None


def _get_camera_info(device, camera_info) -> str | None:
    if device.supports(camera_info):
        return device.get_info(camera_info)
    return None


def list_realsense_devices() -> list[RealSenseDeviceInfo]:
    context = rs.context()

    devices = []
    for device in context.query_devices():
        devices.append(
            RealSenseDeviceInfo(
                serial_number=_get_camera_info(device, rs.camera_info.serial_number),
                name=_get_camera_info(device, rs.camera_info.name),
                product_line=_get_camera_info(device, rs.camera_info.product_line),
                usb_type=_get_camera_info(device, rs.camera_info.usb_type_descriptor),
            )
        )

    return sorted(devices, key=lambda device: device.serial_number or "")


def resolve_realsense_device(serial_number: str | None) -> RealSenseDeviceInfo:
    devices = list_realsense_devices()
    if not devices:
        raise RuntimeError("No Intel RealSense devices detected.")

    if serial_number is not None:
        for device in devices:
            if device.serial_number == serial_number:
                return device
        available = ", ".join(device.serial_number for device in devices if device.serial_number)
        raise RuntimeError(
            f'RealSense device with serial "{serial_number}" was not found. '
            f"Available serials: {available or 'none'}."
        )

    if len(devices) > 1:
        available = ", ".join(device.serial_number for device in devices if device.serial_number)
        raise RuntimeError(
            "Multiple Intel RealSense devices detected. Set camera.serial_number in config to bind "
            f"each logical camera deterministically. Available serials: {available}."
        )

    return devices[0]
