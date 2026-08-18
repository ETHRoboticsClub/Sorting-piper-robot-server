#!/usr/bin/env python3
"""Per-joint health diagnostic for the AgileX Piper arm.

Reads the driver-level (0x261-0x266) and high-speed (0x251-0x256) CAN feedback
so you can tell whether a "clicking"/notchy joint is an electrical fault
(encoder / FOC commutation / driver error) or a mechanical one (gearbox, bearing).

Usage:
    python scripts/piper_joint_diag.py                 # 10 s snapshot of all joints
    python scripts/piper_joint_diag.py --joint 6 -d 30 # watch J6 for 30 s
    python scripts/piper_joint_diag.py --clear-errors  # clear latched joint error codes

Leave the arm DISABLED and back-drive the suspect joint by hand while this runs:
    - current/effort stays ~0 and position tracks smoothly  -> mechanical (gearbox/bearing)
    - position jumps, or driver_error/stall/collision flips  -> encoder or driver
"""

import argparse
import time

from piper_sdk import C_PiperInterface_V2

FOC_FLAGS = (
    "voltage_too_low",
    "motor_overheating",
    "driver_overcurrent",
    "driver_overheating",
    "collision_status",
    "driver_error_status",
    "stall_status",
)


def foc_faults(status) -> list[str]:
    """Names of the asserted fault bits (driver_enable_status is not a fault)."""
    return [f for f in FOC_FLAGS if getattr(status, f, False)]


def joint_slice(msgs, idx: int):
    """msgs is the low- or high-speed container; idx is 1-based joint number."""
    return getattr(msgs, f"motor_{idx}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="can0")
    ap.add_argument("--joint", type=int, choices=range(1, 7), help="focus a single joint")
    ap.add_argument("-d", "--duration", type=float, default=10.0, help="seconds to sample")
    ap.add_argument("--hz", type=float, default=5.0, help="print rate")
    ap.add_argument("--clear-errors", action="store_true", help="send clear-error to all joints and exit")
    args = ap.parse_args()

    piper = C_PiperInterface_V2(args.port)
    piper.ConnectPort()
    time.sleep(0.5)

    if args.clear_errors:
        piper.JointConfig(joint_num=7, clear_err=0xAE)
        print("Sent clear-error (0xAE) to all 6 joints. Re-run without --clear-errors to verify.")
        return

    print(f"firmware: {piper.GetPiperFirmwareVersion()}")
    joints = [args.joint] if args.joint else list(range(1, 7))
    period = 1.0 / args.hz
    deadline = time.time() + args.duration
    # highest |effort| and worst faults seen per joint over the whole run
    peak = {j: 0.0 for j in joints}
    seen_faults = {j: set() for j in joints}

    while time.time() < deadline:
        status = piper.GetArmStatus().arm_status
        low = piper.GetArmLowSpdInfoMsgs()
        high = piper.GetArmHighSpdInfoMsgs()

        print(
            f"\n[{time.strftime('%H:%M:%S')}] arm_status={status.arm_status} "
            f"err_code=0x{status.err_code:04X}"
        )
        for j in joints:
            lo = joint_slice(low, j)
            hi = joint_slice(high, j)
            faults = foc_faults(lo.foc_status)
            seen_faults[j].update(faults)
            peak[j] = max(peak[j], abs(hi.effort))
            flag = "  <-- FAULT" if faults else ""
            print(
                f"  J{j}: pos={hi.pos:>8} spd={hi.motor_speed:>6} "
                f"cur={hi.current * 0.001:>6.2f}A eff={hi.effort:>7.3f}Nm "
                f"vol={lo.vol * 0.1:>5.1f}V foc={lo.foc_temp:>3}C mot={lo.motor_temp:>3}C "
                f"enabled={lo.foc_status.driver_enable_status} "
                f"faults={','.join(faults) or 'none'}{flag}"
            )
        time.sleep(period)

    print("\n--- summary ---")
    for j in joints:
        print(
            f"  J{j}: peak |effort| = {peak[j]:.3f} Nm, "
            f"faults seen = {','.join(sorted(seen_faults[j])) or 'none'}"
        )


if __name__ == "__main__":
    main()
