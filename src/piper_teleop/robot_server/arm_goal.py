from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ArmGoal:
    arm: str
    gripper_closed: bool | None = None
    reset_to_init: bool = False
    reset_reference: bool = False
    relative_transform: np.ndarray | None = None
