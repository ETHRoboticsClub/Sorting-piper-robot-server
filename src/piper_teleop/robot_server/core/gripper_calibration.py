"""Jaw opening in metres, for a gripper whose firmware reports neither.

The Piper's gripper feedback is documented as millimetres of opening and is
exposed by :meth:`PiperSDKInterface.get_status` as ``grippers_angle / 1e6``
metres. On this arm both halves of that are wrong: the origin sits open of the
closed stop, and the scale is short by about a quarter. Measured 2026-08-18
against a tape:

===============  ==============  =============
stop             firmware value  true opening
===============  ==============  =============
closed           -0.002100 m     0.000 m
open             +0.071470 m     0.096 m
===============  ==============  =============

Both faults are structural rather than drift. AgileX ship exactly two gripper
strokes -- 70 mm and 100 mm, selected by ``max_range_config`` -- and the
firmware's counts-to-millimetres constant belongs to whichever it thinks is
fitted. This gripper is neither, and the arm does not answer the 0x47E query
that would report the setting, so the firmware cannot be told otherwise. The
protocol's only calibration is ``GripperCtrl(set_zero=0xAE)``, which moves the
origin and not the scale; its fixed point on this hardware leaves the closed
stop reading about -0.0014 rather than 0, so even the origin cannot be written
away. Hence a conversion here.

Linearity is not assumed. The jaws were measured at both stops, tips and base:
0 -> 96 mm at the tips and 4 -> 100 mm at the base, spanning 96 mm either way,
which is what parallel jaws do and what makes a single scale factor honest.

The scale is only as good as the tape it came from -- roughly +/-1% -- so this
is right to the millimetre and not beyond. The firmware end contributes nothing:
both stops read with zero spread.

Anything in this workspace that means a physical jaw width uses true metres.
The two conversions below are the only places firmware units are allowed to
cross into it, and :mod:`piper_teleop.robot_server.core.piper_sdk_interface` is
the only module that calls them -- see ``set_joint_positions`` and
``get_status`` there.

Re-fitting after a gripper swap needs two numbers and no code: drive the jaws to
each stop unloaded, read the firmware value, and measure the opening with a
tape. Replace the three constants below.
"""

#: Firmware value treated as the closed origin, in the firmware's claimed metres.
#: The jaws actually read about -0.002100 at the closed stop, but this is pinned
#: to 0.0 deliberately: a negative command is not reliably honoured, so the
#: origin is placed where the firmware can be driven rather than where the stop
#: is. The cost is that the closed stop reports about -0.0028 true metres rather
#: than 0.0, and the last ~2.8 mm of closing travel is not commandable.
RAW_CLOSED_M = 0.0

#: Firmware value with the jaws at their open stop, unloaded.
RAW_OPEN_M = 0.071470

#: Jaw opening at that open stop, measured with a tape. The closed stop is
#: 0.0 by definition -- the tips touch.
TRUE_OPEN_M = 0.096

#: How far the firmware value travels between the stops. Not a constant worth
#: editing: it is what the two above imply.
RAW_SPAN_M = RAW_OPEN_M - RAW_CLOSED_M

#: True metres per firmware unit. About 1.30 on this gripper.
SCALE = TRUE_OPEN_M / RAW_SPAN_M


def to_true_m(raw_m: float) -> float:
    """Convert a firmware gripper value to true jaw opening in metres.

    Args:
        raw_m: The value the SDK reports as ``grippers_angle`` divided by 1e6.

    Returns:
        Jaw opening in metres, 0.0 at the closed stop. Values outside the
        stops are converted rather than clamped, because a reading past a
        stop means the calibration has drifted and silently hiding that
        would remove the evidence.
    """
    return (float(raw_m) - RAW_CLOSED_M) * SCALE


def to_raw_m(true_m: float) -> float:
    """Convert a true jaw opening in metres to the value the firmware wants.

    The exact inverse of :func:`to_true_m`, which matters: the policy's
    commands are round-tripped through both, and any asymmetry between them
    would show up as a gripper that does not go where it is told.

    Args:
        true_m: Jaw opening in metres, 0.0 being closed.

    Returns:
        The value to send to ``GripperCtrl``, before scaling to its 1e6 units.
    """
    return float(true_m) / SCALE + RAW_CLOSED_M
