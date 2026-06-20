#!/usr/bin/env python
"""Milestone 1 hardware test for the LeRobot Piper follower plugin.

Exercises the ``PiperFollower`` LeRobot ``Robot`` wrapper in pure joint space (no
IK, no teleop): connect -> read observations -> (optional) tiny safe nudge ->
read back -> disconnect. This validates the CAN read/write path through the
LeRobot interface.

Run inside the piper_hilserl env, after bringing up CAN:

    ./scripts/restart-can
    conda activate piper_hilserl
    python scripts/test_piper_follower.py                 # read-only (safe)
    python scripts/test_piper_follower.py --nudge --joint 0 --delta 0.05
"""

import argparse
import time

from piper_teleop.lerobot_plugin import PiperFollower, PiperFollowerConfig


def fmt(obs: dict) -> str:
    return "  ".join(f"j{i}={obs[f'joint_{i}.pos']:+.4f}" for i in range(7))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="left_piper", help="CAN interface name")
    ap.add_argument("--reads", type=int, default=20, help="number of observation reads")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--nudge", action="store_true", help="perform a small safe joint move")
    ap.add_argument("--joint", type=int, default=0, help="joint index to nudge (0-5)")
    ap.add_argument("--delta", type=float, default=0.05, help="nudge size in radians")
    args = ap.parse_args()

    # Safety cap: never move any joint more than 0.1 rad / 0.01 m per command.
    cfg = PiperFollowerConfig(port=args.port, max_relative_target=0.1)
    robot = PiperFollower(cfg)

    print(f"observation_features: {list(robot.observation_features)}")
    print(f"action_features:      {list(robot.action_features)}")
    print(f"Connecting on {args.port} ...")
    robot.connect()
    print(f"connected: {robot.is_connected}")

    try:
        print("\n-- reading observations --")
        for _ in range(args.reads):
            print(fmt(robot.get_observation()))
            time.sleep(1.0 / args.hz)

        if args.nudge:
            assert 0 <= args.joint <= 5, "only arm joints 0-5 are nudgeable"
            present = robot.get_observation()
            target = {f"joint_{i}.pos": present[f"joint_{i}.pos"] for i in range(7)}
            start = present[f"joint_{args.joint}.pos"]
            print(f"\n-- nudging joint_{args.joint}: {start:+.4f} -> {start + args.delta:+.4f} --")
            target[f"joint_{args.joint}.pos"] = start + args.delta
            for _ in range(15):
                robot.send_action(target)
                time.sleep(1.0 / args.hz)
            print("reached:", fmt(robot.get_observation()))

            print("-- restoring --")
            target[f"joint_{args.joint}.pos"] = start
            for _ in range(15):
                robot.send_action(target)
                time.sleep(1.0 / args.hz)
            print("restored:", fmt(robot.get_observation()))
    finally:
        robot.disconnect()
        print("\ndisconnected.")


if __name__ == "__main__":
    main()
