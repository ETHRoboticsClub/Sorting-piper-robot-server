#!/usr/bin/env python
"""Launch the HIL-SERL SAC **actor** for the Piper (runs on the robot).

The actor builds the real ``gym_manipulator`` env (so it needs every repo patch:
Arm_IK, 6-DOF EE control, persistent target, boolean gripper, pyav, and the 6-DOF
action space), executes the policy, collects transitions for the learner, and lets
the human intervene with the gamepad (toggle Share). Start the learner first.

    bash scripts/restart_can.sh
    conda activate piper_hilserl_rl
    python scripts/run_sac_actor.py --config_path config/sac_piper.json
"""

import lerobot.cameras.realsense.configuration_realsense  # noqa: F401
import lerobot.cameras.opencv.configuration_opencv  # noqa: F401
import piper_teleop.lerobot_plugin  # noqa: F401  (registers Piper robot/teleop choices)
from piper_teleop.lerobot_plugin.patches import apply_lerobot_patches
from lerobot.rl.actor import actor_cli

if __name__ == "__main__":
    apply_lerobot_patches()
    actor_cli()
