"""Refill the replay buffer from previous runs' episode recordings.

Stopping and restarting the pipeline normally throws away everything the robot
did: the learner comes up with an empty buffer and waits for
``online_step_before_learning`` fresh transitions before it can learn at all. That
makes short iteration cycles nearly useless.

The MCAP recordings are a faithful source for this. They store the *processed*
observation -- 128x128 images, the same 7-dim state and 7-dim action the policy
saw -- so a replayed transition is what the actor originally sent, not an
approximation. They also exist for every run, unlike learner checkpoints, which
are only written every ``save_freq`` (5000) optimization steps and so never appear
during short sessions.

Episodes are consumed **newest first**, on the assumption that recent behaviour is
the most relevant, and loading stops once the buffer is full.

Two details that would silently corrupt the buffer if got wrong:

* ``optimize_memory=True`` means ``next_state`` is not stored -- ``sample()`` reads
  index ``i+1`` instead. Transitions within an episode must therefore be added in
  forward temporal order, and the final transition of an episode must be flagged
  ``done``/``truncated`` so no sample straddles an episode boundary.
* ``ReplayBuffer._initialize_storage`` allocates ``complementary_info`` from the
  **first** transition it ever sees. If warm-start data omits ``discrete_penalty``,
  the field is never allocated and every later online transition's penalty is
  dropped -- the same failure HILSERL.md documents for offline demos.
"""

from __future__ import annotations

import base64
import glob
import json
import logging
import os
from typing import Any, Iterator

logger = logging.getLogger(__name__)

DEFAULT_ROOT = os.path.join("outputs", "foxglove")


def find_recordings(root: str = DEFAULT_ROOT) -> list[str]:
    """Every recorded episode, newest first."""
    paths = glob.glob(os.path.join(root, "**", "*.mcap"), recursive=True)
    return sorted(paths, key=os.path.getmtime, reverse=True)


def _decode_image(payload: dict) -> Any:
    """MCAP CompressedImage -> float32 CHW in [0, 1], as the buffer stores it."""
    import cv2
    import numpy as np

    buf = np.frombuffer(base64.b64decode(payload["data"]), dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))


def topics_of(mcap_path: str) -> set[str]:
    from mcap.reader import make_reader

    with open(mcap_path, "rb") as handle:
        return {ch.topic for ch in make_reader(handle).get_summary().channels.values()}


def validate_recording(mcap_path: str, features: dict[str, tuple]) -> tuple[bool, str]:
    """Is this recording compatible with the policy's observation space?

    Recordings accumulate across config changes -- a run made before a camera was
    added, or at a different resolution, is not the same observation space and must
    not be loaded. Silently skipping the mismatched *rows* (the earlier behaviour)
    is worse than skipping the file: it quietly biases the buffer toward whichever
    runs happened to match.

    Checked cheaply from the summary first, then by decoding a single frame.
    """
    image_keys = {k.split(".")[-1]: tuple(shape) for k, shape in features.items() if "image" in k}
    state_dim = next((int(shape[0]) for k, shape in features.items() if "image" not in k), None)

    try:
        topics = topics_of(mcap_path)
    except Exception as exc:  # noqa: BLE001
        return False, f"unreadable ({type(exc).__name__})"

    for required in ("/state", "/action", "/reward"):
        if required not in topics:
            return False, f"missing {required}"
    missing = [name for name in image_keys if f"/camera/{name}" not in topics]
    if missing:
        return False, f"missing camera(s): {', '.join(sorted(missing))}"

    # Shapes: a resolution or state-dim change silently corrupts the buffer.
    import json

    from mcap.reader import make_reader

    checked_images: set[str] = set()
    checked_state = state_dim is None
    with open(mcap_path, "rb") as handle:
        for _schema, channel, message in make_reader(handle).iter_messages():
            if channel.topic == "/state" and not checked_state:
                payload = json.loads(message.data)
                found = len([k for k in payload if k.startswith("joint_")]) + ("gripper" in payload)
                if found != state_dim:
                    return False, f"state dim {found}, expected {state_dim}"
                checked_state = True
            elif channel.topic.startswith("/camera/"):
                name = channel.topic.rsplit("/", 1)[-1]
                if name in image_keys and name not in checked_images:
                    got = _decode_image(json.loads(message.data)).shape
                    if tuple(got) != image_keys[name]:
                        return False, f"{name} is {tuple(got)}, expected {image_keys[name]}"
                    checked_images.add(name)
            if checked_state and len(checked_images) == len(image_keys):
                return True, "ok"

    return False, "no complete frame"


def iter_transitions(mcap_path: str, state_keys: list[str]) -> Iterator[dict]:
    """Yield one dict per recorded step, in forward temporal order.

    Messages are grouped by timestamp: the logger publishes every topic for a step
    with an identical ``log_time``, so a step is complete once its state/action/
    reward have all arrived.
    """
    import numpy as np
    from mcap.reader import make_reader

    wanted_images = {k.split(".")[-1] for k in state_keys if "image" in k}

    steps: dict[int, dict] = {}
    with open(mcap_path, "rb") as handle:
        for _schema, channel, message in make_reader(handle).iter_messages():
            topic = channel.topic
            if topic == "/episode":
                continue
            payload = json.loads(message.data)
            step = steps.setdefault(message.log_time, {"images": {}})

            if topic.startswith("/camera/"):
                name = topic.rsplit("/", 1)[-1]
                if name in wanted_images:
                    step["images"][name] = payload
            elif topic == "/state":
                step["state"] = [payload.get(f"joint_{i}", 0.0) for i in range(6)] + [
                    payload.get("gripper", 0.0)
                ]
            elif topic == "/action":
                step["action"] = [
                    payload.get(k, 0.0) for k in ("dx", "dy", "dz", "wx", "wy", "wz", "gripper")
                ]
            elif topic == "/reward":
                step["reward"] = float(payload.get("reward", 0.0))
                step["done"] = bool(payload.get("done", False))

    ordered = sorted(steps.items())
    for index, (_stamp, step) in enumerate(ordered):
        if "state" not in step or "action" not in step:
            continue  # partially-written step (the logger drops frames under load)
        if len(step["images"]) != len(wanted_images):
            continue
        yield {
            "images": {name: _decode_image(p) for name, p in step["images"].items()},
            "state": np.asarray(step["state"], dtype=np.float32),
            "action": np.asarray(step["action"], dtype=np.float32),
            "reward": step.get("reward", 0.0),
            # The last usable step of a file ends the episode: without this a sample
            # could pair the final state with the *next* episode's first frame.
            "done": step.get("done", False) or index == len(ordered) - 1,
        }


def warm_start(buffer, features: dict[str, tuple], root: str = DEFAULT_ROOT) -> tuple[int, int]:
    """Fill ``buffer`` from past recordings, newest first. Returns (added, episodes).

    Every recording is validated against the policy's observation space first;
    incompatible ones are skipped whole, with the reason reported.
    """
    import collections

    import torch

    state_keys = list(features)
    recordings = find_recordings(root)
    if not recordings:
        logger.info("[WARM START] no previous recordings under %s -- starting empty", root)
        return 0, 0

    image_keys = {k.split(".")[-1]: k for k in state_keys if "image" in k}
    state_key = next((k for k in state_keys if "image" not in k), "observation.state")

    added = 0
    episodes = 0
    rejected: collections.Counter = collections.Counter()
    for path in recordings:
        if len(buffer) >= buffer.capacity:
            break

        ok, reason = validate_recording(path, features)
        if not ok:
            rejected[reason] += 1
            continue

        try:
            transitions = list(iter_transitions(path, state_keys))
        except Exception as exc:  # noqa: BLE001 - a corrupt file must not stop the run
            rejected[f"read error ({type(exc).__name__})"] += 1
            continue
        if not transitions:
            rejected["no usable transitions"] += 1
            continue

        before = len(buffer)
        for transition in transitions:
            if len(buffer) >= buffer.capacity:
                break
            state = {
                image_keys[name]: torch.from_numpy(img).unsqueeze(0)
                for name, img in transition["images"].items()
                if name in image_keys
            }
            state[state_key] = torch.from_numpy(transition["state"]).unsqueeze(0)
            buffer.add(
                state=state,
                action=torch.from_numpy(transition["action"]).unsqueeze(0),
                reward=transition["reward"],
                next_state=state,  # ignored: optimize_memory reads index i+1
                done=transition["done"],
                truncated=transition["done"],
                # Must be present on the very first add -- see module docstring.
                complementary_info={"discrete_penalty": torch.tensor([0.0])},
            )
            added += 1
        if len(buffer) > before:
            episodes += 1

    if added:
        logger.info(
            "[WARM START] loaded %d transitions from %d episode(s); buffer %d/%d (%s)",
            added,
            episodes,
            len(buffer),
            buffer.capacity,
            describe_memory(buffer),
        )
    else:
        logger.info("[WARM START] no compatible recordings -- starting empty")
    for reason, count in rejected.most_common():
        logger.info("[WARM START] skipped %d recording(s): %s", count, reason)
    return added, episodes


TRAIN_ROOT = os.path.join("outputs", "train")


def find_latest_checkpoint(root: str = TRAIN_ROOT) -> str | None:
    """Newest saved policy across previous runs, or None.

    LeRobot writes ``<run>/checkpoints/<step>/pretrained_model`` with a ``last``
    symlink. Note these only appear every ``save_freq`` optimization steps (5000 by
    default), so short sessions produce none -- an empty result is normal, not an
    error.
    """
    candidates = glob.glob(os.path.join(root, "*", "*", "checkpoints", "*", "pretrained_model"))
    candidates = [c for c in candidates if os.path.isdir(c)]
    if not candidates:
        return None
    # Resolve so the "last" symlink and its target do not both rank.
    unique = {os.path.realpath(c) for c in candidates}
    return max(unique, key=os.path.getmtime)


def buffer_bytes(buffer) -> int:
    """Bytes actually allocated by the buffer's pre-allocated tensors.

    Capacity is pre-allocated up front, so this does not grow as the buffer fills --
    which is the point: it tells you what the run costs, not what it costs so far.
    """
    total = 0
    seen: set[int] = set()
    for attr in ("states", "next_states", "complementary_info"):
        for tensor in (getattr(buffer, attr, None) or {}).values():
            if id(tensor) in seen:  # next_states aliases states under optimize_memory
                continue
            seen.add(id(tensor))
            total += tensor.numel() * tensor.element_size()
    for attr in ("actions", "rewards", "dones", "truncateds", "episode_ends"):
        tensor = getattr(buffer, attr, None)
        if tensor is not None and id(tensor) not in seen:
            seen.add(id(tensor))
            total += tensor.numel() * tensor.element_size()
    return total


def describe_memory(buffer) -> str:
    total = buffer_bytes(buffer)
    if not total:
        return "unallocated"
    per_frame = total / max(1, buffer.capacity)
    return f"{total / 1e9:.2f} GB allocated, {per_frame / 1e3:.0f} kB/transition"
