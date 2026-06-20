#!/usr/bin/env python
"""Print the 6-DOF action stream from the Piper gamepad teleoperator (no robot).

Validates that the PS5 controller produces sensible translation + rotation deltas
and gripper/intervention events before wiring it to the arm. If the axes feel
wrong, fix the indices with scripts/find_gamepad_map.py and update
``PiperGamepad6DofConfig``.

    conda activate piper_hilserl
    python scripts/test_piper_gamepad_teleop.py
"""

import time

from lerobot.teleoperators.utils import TeleopEvents

from piper_teleop.lerobot_plugin import PiperGamepad6Dof, PiperGamepad6DofConfig


def main() -> None:
    teleop = PiperGamepad6Dof(PiperGamepad6DofConfig())
    teleop.connect()
    print("Connected. Move sticks/triggers/bumpers; Cross/Circle = gripper,")
    print("hold intervention button, Square/Triangle = success/failure. Ctrl-C to exit.\n")
    try:
        while True:
            a = teleop.get_action()
            ev = teleop.get_teleop_events()
            print(
                "\r"
                f"d=[{a['delta_x']:+.2f} {a['delta_y']:+.2f} {a['delta_z']:+.2f}]  "
                f"w=[{a['delta_wx']:+.2f} {a['delta_wy']:+.2f} {a['delta_wz']:+.2f}]  "
                f"grip={a.get('gripper', '-')}  "
                f"intervene={int(ev[TeleopEvents.IS_INTERVENTION])} "
                f"success={int(ev[TeleopEvents.SUCCESS])} term={int(ev[TeleopEvents.TERMINATE_EPISODE])}   ",
                end="",
                flush=True,
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        teleop.disconnect()


if __name__ == "__main__":
    main()
