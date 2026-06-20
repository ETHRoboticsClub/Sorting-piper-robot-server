#!/usr/bin/env python
"""Launch the HIL-SERL SAC **learner** for the Piper.

The learner trains the SAC policy on transitions streamed from the actor (plus the
offline demo buffer) and serves updated weights over gRPC. Run this first, then
``run_sac_actor.py`` in another terminal with the same config.

    conda activate piper_hilserl_rl
    python scripts/run_sac_learner.py --config_path config/sac_piper.json

We apply the repo's LeRobot patches before the CLI runs so the config parses
(piper_follower / piper_gamepad_6dof / intelrealsense choices) and the dataset
loads with the pyav video backend. The learner does not touch the robot.
"""

import lerobot.cameras.realsense.configuration_realsense  # noqa: F401
import lerobot.cameras.opencv.configuration_opencv  # noqa: F401
import piper_teleop.lerobot_plugin  # noqa: F401  (registers Piper robot/teleop choices)
from piper_teleop.lerobot_plugin.patches import apply_lerobot_patches
from lerobot.rl.learner import train_cli

if __name__ == "__main__":
    apply_lerobot_patches()
    train_cli()
