#!/usr/bin/env python
"""Inspect or set the Piper's per-joint collision-protection level.

The controller only raises the ``collision`` / ``stall`` fault flags (used by the
HIL-SERL collision penalty) when protection is enabled. Out of the box every joint is
level 0 = OFF, so nothing ever trips. Set a level once; it persists on the controller.

    python scripts/crash_protection.py                 # show current levels + live load
    python scripts/crash_protection.py --set 3         # all joints to level 3
    python scripts/crash_protection.py --set 0         # disable again
    python scripts/crash_protection.py --set 2,2,3,3,4,4   # per joint

Levels are 0-8; higher = more sensitive (trips at a lower load). Start low (2-3) and
raise if real collisions are missed, lower if normal motion trips it. Run with the CAN
bus up (`bash scripts/restart_can.sh`) and no other process holding the arm.
"""

import argparse
import sys
import time

from piper_teleop.robot_server.core.piper_sdk_interface import PiperSDKInterface


def main() -> int:
    ap = argparse.ArgumentParser(description="Inspect/set Piper crash-protection levels")
    ap.add_argument("--port", default="left_piper")
    ap.add_argument("--set", dest="level", help="level 0-8 for all joints, or 6 comma-separated levels")
    args = ap.parse_args()

    sdk = PiperSDKInterface(port=args.port)
    time.sleep(0.5)

    if args.level is not None:
        parts = [int(x) for x in args.level.split(",")]
        levels = parts[0] if len(parts) == 1 else parts
        sdk.set_crash_protection_levels(levels)
        time.sleep(0.5)
        print(f"set -> {sdk.get_crash_protection_levels()}")
        return 0

    print(f"crash-protection levels: {sdk.get_crash_protection_levels()}")
    print("  (0 = collision detection OFF; the collision flag can only fire above 0)\n")
    print("live load (2 s) -- current per joint (A), enabled, collision/stall flags:")
    for _ in range(4):
        d = sdk.get_motor_currents()
        cur = {k.split(".")[0]: round(v, 3) for k, v in d.items() if k.endswith(".current")}
        print(
            f"  enabled={int(d.get('arm_enabled', 0))} "
            f"collision={int(d.get('any_collision', 0))} stall={int(d.get('any_stall', 0))} | {cur}"
        )
        time.sleep(0.5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
