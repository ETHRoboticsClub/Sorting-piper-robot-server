"""Write each RL episode to an MCAP file for scrubbing in Foxglove.

One file per episode under ``outputs/foxglove/<session>/``. Open it in Foxglove
(https://app.foxglove.dev, or ``scripts/view_foxglove.sh``) and every transition
lines up on a single timeline: what the cameras saw, what the policy did, what it
got paid, and whether a human was driving.

Topics
------
``/camera/<name>``   foxglove.CompressedImage -- one per camera in the observation
``/grasp/overlay``   foxglove.CompressedImage -- wrist view + YOLO verdict banner
``/state``           joints + gripper (plottable)
``/action``          executed EE deltas + gripper (plottable)
``/reward``          step reward + running episode return (plottable)
``/control``         intervention vs policy, as a number so it plots as a band
``/grasp``           YOLO class / confidence / success
``/episode``         one message at episode start with the run metadata

Overhead
--------
The control loop only pays for a dict copy and a queue put. JPEG encoding and all
file I/O happen on a background thread behind a **bounded** queue that *drops*
frames when full, so a slow disk can never stall or pace the robot. Logging is
therefore lossy under pressure by design -- the alternative is backpressure into
the control loop, which is worse.

Env vars: ``PIPER_FG=0`` disables; ``PIPER_FG_DIR`` overrides the output dir;
``PIPER_FG_EVERY=N`` logs every Nth step; ``PIPER_FG_QUALITY`` sets JPEG quality
(default 80); ``PIPER_FG_MAXQ`` sets the queue depth (default 64).
"""

from __future__ import annotations

import base64
import json
import logging
import os
import queue
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)

# Foxglove's well-known JSON schemas. Matching the *name* is what makes Foxglove
# render these in the Image panel rather than as raw JSON.
_IMAGE_SCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {
            "type": "object",
            "properties": {"sec": {"type": "integer"}, "nsec": {"type": "integer"}},
        },
        "frame_id": {"type": "string"},
        "data": {"type": "string", "contentEncoding": "base64"},
        "format": {"type": "string"},
    },
}


def _to_uint8_hwc(img: Any):
    """Normalise a torch/numpy image (CHW or HWC, float [0,1] or uint8) to HWC uint8 RGB."""
    import numpy as np

    if hasattr(img, "detach"):
        img = img.detach().cpu().numpy()
    img = np.asarray(img)
    if img.ndim == 4:  # drop a leading batch dim
        img = img[0]
    if img.ndim == 3 and img.shape[0] in (1, 3) and img.shape[-1] not in (1, 3):
        img = np.transpose(img, (1, 2, 0))  # CHW -> HWC
    if img.dtype != np.uint8:
        peak = float(img.max()) if img.size else 0.0
        img = (img * 255.0 if peak <= 1.0 else img).clip(0, 255).astype(np.uint8)
    if img.ndim == 2:
        img = np.stack([img] * 3, axis=-1)
    return img


def _tensor_to_floats(value: Any) -> list[float]:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    try:
        import numpy as np

        arr = np.asarray(value, dtype=float).reshape(-1)
        return [float(x) for x in arr]
    except Exception:  # noqa: BLE001
        try:
            return [float(value)]
        except Exception:  # noqa: BLE001
            return []


class FoxgloveEpisodeLogger:
    """Background MCAP writer, one file per episode."""

    def __init__(self, root: str | None = None):
        self.enabled = os.environ.get("PIPER_FG", "1") != "0"
        self._every = max(1, int(os.environ.get("PIPER_FG_EVERY", "1")))
        self._quality = int(os.environ.get("PIPER_FG_QUALITY", "80"))
        maxq = int(os.environ.get("PIPER_FG_MAXQ", "64"))

        self._queue: queue.Queue = queue.Queue(maxsize=maxq)
        self._thread: threading.Thread | None = None
        self._writer = None
        self._stream = None
        self._channels: dict[str, int] = {}
        self._live_server = None
        self._live_channels: dict[str, Any] = {}
        self._live = os.environ.get("PIPER_FG_LIVE", "1") != "0"
        self._live_port = int(os.environ.get("PIPER_FG_PORT", "8765"))

        # Buffer fill lives in the learner; it drops this file, the actor's logger
        # reads it (same machine, fixed path) so /buffer appears alongside the
        # rollout topics. Cached and re-read at most ~once/second.
        self._buffer_status_path = os.environ.get("PIPER_BUFFER_STATUS", "outputs/live_buffer_status.json")
        self._buffer_status: dict | None = None
        self._buffer_status_read_ns = 0
        self._episode = 0
        self._step = 0
        self._return = 0.0
        self._dropped = 0
        self._path: str | None = None

        if not self.enabled:
            self.root = None
            return

        session = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.root = root or os.environ.get("PIPER_FG_DIR") or os.path.join("outputs", "foxglove", session)
        os.makedirs(self.root, exist_ok=True)
        logger.info("[FOXGLOVE] logging episodes to %s", os.path.abspath(self.root))

        if self._live:
            try:
                import foxglove

                self._live_server = foxglove.start_server(
                    name="piper-hilserl", host="127.0.0.1", port=self._live_port
                )
                logger.info("[FOXGLOVE] live stream on ws://127.0.0.1:%d", self._live_port)
            except Exception as exc:  # noqa: BLE001 - live viz is never worth failing a run
                logger.error(
                    "[FOXGLOVE] LIVE STREAM DISABLED on port %d (%s: %s). Another process "
                    "probably holds the port -- a stale actor, or a viewer will attach to it "
                    "and see nothing. Recording to MCAP continues. Set PIPER_FG_PORT to "
                    "another port, or stop the process holding it (ss -ltnp | grep %d).",
                    self._live_port,
                    type(exc).__name__,
                    exc,
                    self._live_port,
                )
                self._live = False

        self._thread = threading.Thread(target=self._run, name="foxglove-mcap", daemon=True)
        self._thread.start()

    # ---- public API (called from the control loop; must stay cheap) ----------

    def start_episode(self, metadata: dict | None = None) -> None:
        if not self.enabled:
            return
        self._episode += 1
        self._step = 0
        self._return = 0.0
        self._submit(("open", self._episode, metadata or {}))

    def log_transition(
        self,
        observation: dict | None,
        action: Any,
        reward: float,
        info: dict | None,
        done: bool = False,
    ) -> None:
        """Queue one transition. Never blocks; drops if the writer is behind."""
        if not self.enabled:
            return
        self._step += 1
        if self._step % self._every:
            return
        self._return += float(reward or 0.0)

        # Copy only what we need, on this thread, so the writer can't race the env.
        images: dict[str, Any] = {}
        state: list[float] = []
        if observation:
            for key, value in observation.items():
                if "image" in key:
                    images[key.split(".")[-1]] = _to_uint8_hwc(value)
                elif key.endswith("state") or "state" in key:
                    state = _tensor_to_floats(value)

        self._submit(
            (
                "step",
                {
                    "t": time.time_ns(),
                    "step": self._step,
                    "images": images,
                    "state": state,
                    "action": _tensor_to_floats(action),
                    "reward": float(reward or 0.0),
                    "ret": self._return,
                    "info": _clean_info(info),
                    "done": bool(done),
                },
            )
        )

    def end_episode(self) -> None:
        if self.enabled:
            self._submit(("close", None, None))

    def close(self) -> None:
        if not self.enabled:
            return
        self._submit(("close", None, None))
        self._submit(("stop", None, None))
        if self._thread is not None:
            self._thread.join(timeout=10)
        if self._live_server is not None:
            try:
                self._live_server.stop()
            except Exception:  # noqa: BLE001
                pass
            self._live_server = None
        if self._dropped:
            logger.warning("[FOXGLOVE] dropped %d frames (writer behind)", self._dropped)

    # ---- internals (background thread) --------------------------------------

    def _submit(self, item) -> None:
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self._dropped += 1

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                kind = item[0]
                if kind == "stop":
                    self._close_file()
                    return
                if kind == "open":
                    self._open_file(item[1], item[2])
                elif kind == "close":
                    self._close_file()
                elif kind == "step":
                    self._write_step(item[1])
            except Exception as exc:  # noqa: BLE001 - visualisation must never kill a run
                logger.warning("[FOXGLOVE] writer error: %s: %s", type(exc).__name__, exc)

    def _open_file(self, episode: int, metadata: dict) -> None:
        from mcap.writer import Writer

        self._close_file()
        self._path = os.path.join(self.root, f"episode_{episode:04d}.mcap")
        self._stream = open(self._path, "wb")
        self._writer = Writer(self._stream)
        self._writer.start()
        self._channels = {}
        self._publish("/episode", {"episode": episode, **metadata}, time.time_ns())

    def _close_file(self) -> None:
        if self._writer is None:
            return
        try:
            self._writer.finish()
            self._stream.close()
            logger.info("[FOXGLOVE] wrote %s", os.path.abspath(self._path))
        except Exception as exc:  # noqa: BLE001
            logger.warning("[FOXGLOVE] failed closing %s: %s", self._path, exc)
        finally:
            self._writer = None
            self._stream = None

    def _channel(self, topic: str, schema_name: str, schema: dict) -> int:
        if topic not in self._channels:
            schema_id = self._writer.register_schema(
                name=schema_name, encoding="jsonschema", data=json.dumps(schema).encode()
            )
            self._channels[topic] = self._writer.register_channel(
                topic=topic, message_encoding="json", schema_id=schema_id
            )
        return self._channels[topic]

    def _live_channel(self, topic: str, schema_name: str, schema: dict):
        """Cached live channel; None when live streaming is off or unavailable."""
        if not self._live:
            return None
        if topic not in self._live_channels:
            try:
                import foxglove
                from foxglove import Channel, Schema

                self._live_channels[topic] = Channel(
                    topic,
                    schema=Schema(
                        name=schema_name, encoding="jsonschema", data=json.dumps(schema).encode()
                    ),
                    message_encoding="json",
                )
                del foxglove
            except Exception as exc:  # noqa: BLE001
                logger.warning("[FOXGLOVE] live channel %s failed: %s", topic, exc)
                self._live_channels[topic] = None
        return self._live_channels[topic]

    def _emit(self, topic: str, data: bytes, stamp_ns: int, schema_name: str, schema: dict) -> None:
        """Send one already-serialised message to both sinks.

        The MCAP writer only exists between ``start_episode`` and ``end_episode``;
        the live stream is always on, so a viewer attached between episodes (or
        before the first one) still sees data.
        """
        if self._writer is not None:
            cid = self._channel(topic, schema_name, schema)
            self._writer.add_message(
                channel_id=cid, log_time=stamp_ns, publish_time=stamp_ns, data=data
            )
        channel = self._live_channel(topic, schema_name, schema)
        if channel is not None:
            try:
                channel.log(data, log_time=stamp_ns)
            except Exception as exc:  # noqa: BLE001
                logger.debug("[FOXGLOVE] live publish on %s failed: %s", topic, exc)

    def _publish(self, topic: str, payload: dict, stamp_ns: int, schema_name: str | None = None) -> None:
        name = schema_name or topic.strip("/").replace("/", "_")
        self._emit(topic, json.dumps(payload).encode(), stamp_ns, name, {"type": "object"})

    def _publish_image(self, topic: str, rgb, stamp_ns: int, frame_id: str) -> None:
        import cv2

        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), self._quality]
        )
        if not ok:
            return
        payload = json.dumps(
            {
                "timestamp": {"sec": stamp_ns // 1_000_000_000, "nsec": stamp_ns % 1_000_000_000},
                "frame_id": frame_id,
                "data": base64.b64encode(buf.tobytes()).decode("ascii"),
                "format": "jpeg",
            }
        ).encode()
        self._emit(topic, payload, stamp_ns, "foxglove.CompressedImage", _IMAGE_SCHEMA)

    def _read_buffer_status(self) -> dict | None:
        """Latest learner buffer stats from the shared file, cached ~1 s."""
        now = time.time_ns()
        if now - self._buffer_status_read_ns < 1_000_000_000:
            return self._buffer_status
        self._buffer_status_read_ns = now
        try:
            with open(self._buffer_status_path) as handle:
                self._buffer_status = json.load(handle)
        except Exception:  # noqa: BLE001 - absent until the learner writes it
            pass  # keep the last value rather than dropping the topic
        return self._buffer_status

    def _write_step(self, s: dict) -> None:
        # No early return on a missing MCAP writer: _emit() still streams live, so a
        # viewer attached between episodes keeps receiving data.
        t = s["t"]
        info = s["info"]

        for name, rgb in s["images"].items():
            self._publish_image(f"/camera/{name}", rgb, t, name)

        wrist = next((v for k, v in s["images"].items() if "wrist" in k), None)
        if wrist is not None:
            self._publish_image("/grasp/overlay", _draw_grasp_overlay(wrist, info), t, "wrist_overlay")

        state = s["state"]
        self._publish(
            "/state",
            {**{f"joint_{i}": v for i, v in enumerate(state[:-1])}, "gripper": state[-1] if state else 0.0},
            t,
        )

        action = s["action"]
        names = ["dx", "dy", "dz", "wx", "wy", "wz", "gripper"]
        self._publish("/action", {n: (action[i] if i < len(action) else 0.0) for i, n in enumerate(names)}, t)

        self._publish("/reward", {"reward": s["reward"], "episode_return": s["ret"], "done": s["done"]}, t)

        intervening = bool(info.get("is_intervention") or info.get("IS_INTERVENTION"))
        self._publish(
            "/control",
            {"is_intervention": intervening, "source": "intervention" if intervening else "policy",
             "value": 1.0 if intervening else 0.0},
            t,
        )

        if "yolo_grasp_class" in info:
            self._publish(
                "/grasp",
                {
                    "grasp_class": info.get("yolo_grasp_class"),
                    "confidence": float(info.get("yolo_grasp_confidence") or 0.0),
                    "success": bool(info.get("yolo_grasp_success")),
                },
                t,
            )

        # Reward-shaping breakdown: each dense term, so their scale vs the sparse
        # task reward is visible on one plot.
        self._publish(
            "/shaping",
            {
                "smoothness": float(info.get("action_smoothness_reward") or 0.0),
                "gripper_penalty": float(info.get("discrete_penalty") or 0.0),
                "collision_current_penalty": float(info.get("collision_current_penalty") or 0.0),
                "peak_effort": float(info.get("peak_effort") or 0.0),
            },
            t,
        )

        # Per-joint motor current (torque proxy). Useful on its own and for
        # calibrating the collision-current threshold.
        currents = {k.replace(".current", ""): v for k, v in info.items() if k.endswith(".current")}
        if currents:
            self._publish("/current", currents, t)

        buffer_status = self._read_buffer_status()
        if buffer_status is not None:
            self._publish("/buffer", buffer_status, t)


def _clean_info(info: dict | None) -> dict:
    """JSON-safe copy of ``info``.

    LeRobot keys intervention state with the ``TeleopEvents`` enum *object* (a plain
    Enum, not a str-Enum), which ``json.dumps`` rejects and a plain ``"is_intervention"``
    lookup misses. Unwrap enum keys to their values and keep only primitives.
    """
    out: dict[str, Any] = {}
    for key, value in (info or {}).items():
        name = getattr(key, "value", key)
        if not isinstance(name, str):
            name = str(name)
        if hasattr(value, "item"):  # 0-d tensor / numpy scalar
            try:
                value = value.item()
            except Exception:  # noqa: BLE001
                pass
        if isinstance(value, (str, int, float, bool, type(None))):
            out[name] = value
    return out


def _draw_grasp_overlay(rgb, info: dict):
    """Wrist image with the YOLO verdict burned in (green = success, red = rejected).

    The classifier only runs once the gripper has been held closed, so most frames
    carry no verdict -- those are labelled so a blank banner is not mistaken for a
    failed classification.
    """
    import cv2
    import numpy as np

    img = np.ascontiguousarray(rgb.copy())
    h, w = img.shape[:2]

    if "yolo_grasp_class" in info:
        success = bool(info.get("yolo_grasp_success"))
        colour = (0, 200, 0) if success else (220, 40, 40)
        text = f"{info.get('yolo_grasp_class')} {float(info.get('yolo_grasp_confidence') or 0.0):.2f}"
        text += " OK" if success else " x"
    else:
        colour = (110, 110, 110)
        text = "no grasp check"

    cv2.rectangle(img, (0, 0), (w - 1, h - 1), colour, 3)
    cv2.rectangle(img, (0, 0), (w, 22), colour, -1)
    cv2.putText(img, text, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img
