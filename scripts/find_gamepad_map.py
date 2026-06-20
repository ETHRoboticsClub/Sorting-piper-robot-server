#!/usr/bin/env python
"""Print live gamepad axis/button indices to verify the PS5/SDL2 mapping.

Move each stick/trigger and press each button; note the index that changes, then
set the matching fields in ``PiperGamepad6DofConfig``.

    conda activate piper_hilserl
    python scripts/find_gamepad_map.py
"""

import time

import pygame


def main() -> None:
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("No gamepad detected.")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}  axes={js.get_numaxes()} buttons={js.get_numbuttons()}")
    print("Move sticks/triggers and press buttons. Ctrl-C to exit.\n")

    try:
        while True:
            pygame.event.pump()
            axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
            pressed = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            print(f"\raxes={axes}  buttons_down={pressed}        ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
