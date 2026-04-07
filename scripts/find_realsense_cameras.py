#!/usr/bin/env python3
"""
List connected Intel RealSense cameras and their stable serial numbers.
Use these serials in config.yaml with backend: realsense.
"""

from piper_teleop.robot_server.camera.realsense_device import list_realsense_devices


def main():
    devices = list_realsense_devices()
    if not devices:
        print("No Intel RealSense cameras detected.")
        return

    print(f"Found {len(devices)} Intel RealSense camera(s):\n")
    for device in devices:
        print(f"Serial:       {device.serial_number}")
        print(f"Name:         {device.name or 'unknown'}")
        print(f"Product line: {device.product_line or 'unknown'}")
        print(f"USB type:     {device.usb_type or 'unknown'}")
        print()

    print("Example config:")
    print("  wrist1:")
    print("    type: monocular")
    print("    backend: realsense")
    print("    serial_number: '<serial here>'")
    print("    mode: recording")
    print("    fps: '30'")
    print("    frame_width: '640'")
    print("    frame_height: '480'")
    print("    capture_frame_width: '1280'")
    print("    capture_frame_height: '720'")


if __name__ == "__main__":
    main()
