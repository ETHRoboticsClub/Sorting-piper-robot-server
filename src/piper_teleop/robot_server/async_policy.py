"""Asynchronous policy deployment via LeRobot's async-inference server/client.

This is the *async* counterpart to :class:`LerobotPolicy`. It launches LeRobot's
``policy_server`` in a background thread and embeds a thin gRPC *client* that:

  1. sends the latest observation to the server (which runs the heavy policy
     forward pass off the control loop), and
  2. pops the next action from a locally-maintained queue that a background
     thread keeps filled with the server's predicted action chunks.

Because inference happens on the server (in parallel with execution), the control
loop never blocks waiting for a slow policy (diffusion / multi-task DiT / VLA) —
that's the whole point. The robot keeps executing a smooth stream of actions.

Design note — Route B: :meth:`AsyncPolicyClient.predict` has the **same signature
and return type** as :meth:`LerobotPolicy.predict`, so the control loop only swaps
which object's ``predict`` it calls. Teleop, recording, IK, viz are untouched.

⚠️ ON-HARDWARE VERIFICATION POINTS (cannot be unit-tested without robot + server):
  - the raw-observation key format the server expects (see ``_build_raw_observation``)
  - the action-tensor → joint-dict mapping (see ``_action_to_dicts``)
  - chunk timing / queue threshold tuning (``chunk_size_threshold``)
"""

import logging
import threading
import time
from queue import Empty

import numpy as np

logger = logging.getLogger(__name__)

# robot_server policy_type -> the policy name LeRobot's async server expects.
_POLICY_NAME_MAP = {
    "act": "act",
    "diffusion": "diffusion",
    "smolvla": "smolvla",
    "multitask-Dit": "multi_task_dit",
    "multi_task_dit": "multi_task_dit",
}


def lerobot_policy_name(policy_type: str) -> str:
    """Map the robot_server policy_type to LeRobot's async policy name."""
    return _POLICY_NAME_MAP.get(policy_type, policy_type)


def enable_multitask_dit_async() -> None:
    """Register multi-task DiT as an async-capable policy.

    ``MultiTaskDiTPolicy`` already implements ``predict_action_chunk()`` (the method
    the async server calls); it is simply absent from the hard-coded allow-list.
    Mutating the list in place is seen by ``policy_server`` because it imported the
    same list object (``from .constants import SUPPORTED_POLICIES``).
    """
    from lerobot.async_inference import constants

    if "multi_task_dit" not in constants.SUPPORTED_POLICIES:
        constants.SUPPORTED_POLICIES.append("multi_task_dit")
        logger.info("Registered 'multi_task_dit' as an async-capable policy.")


def patch_chunk_policy_for_async(policy_type: str) -> None:
    """Make a queue-based chunking policy usable by the async server.

    The async server calls ``policy.predict_action_chunk(obs)`` directly, but DiT/diffusion's
    ``predict_action_chunk`` ignores the incoming obs and stacks ``self._queues`` instead — a
    queue that only ``select_action`` populates. Called directly, the queue is empty -> torch.stack
    on an empty list -> "stack expects a non-empty TensorList". We wrap the method to populate the
    queue from the incoming observation first (what ``select_action`` would do). Patched on the
    CLASS before the server loads the policy (server runs in a thread in this same process).
    """
    from lerobot.policies.utils import populate_queues
    from lerobot.utils.constants import ACTION

    name = lerobot_policy_name(policy_type)
    classes = []
    try:
        if name == "multi_task_dit":
            from lerobot.policies.multi_task_dit.modeling_multi_task_dit import MultiTaskDiTPolicy

            classes.append(MultiTaskDiTPolicy)
        elif name == "diffusion":
            from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

            classes.append(DiffusionPolicy)
    except Exception as e:  # pragma: no cover
        logger.warning("Async queue-patch: could not import policy class: %s", e)
        return

    for P in classes:
        if getattr(P.predict_action_chunk, "_async_queue_patched", False):
            continue
        _orig = P.predict_action_chunk

        def _patched(self, batch, *args, __orig=_orig, **kwargs):
            self.eval()
            b = dict(batch)
            b.pop(ACTION, None)
            if hasattr(self, "_prepare_batch"):
                b = self._prepare_batch(b)
            self._queues = populate_queues(self._queues, b)
            # Return the FULL action horizon, not just n_action_steps. The policy generates `horizon`
            # actions and normally keeps only the first n_action_steps; for async we want longer
            # chunks so each chunk lasts longer than one inference -> the queue stays ahead -> smooth
            # motion (otherwise the queue starves and the arm holds-then-jumps every chunk).
            cfg = getattr(self, "config", None)
            orig_n = getattr(cfg, "n_action_steps", None)
            horizon = getattr(cfg, "horizon", None)
            n_obs = int(getattr(cfg, "n_obs_steps", 1) or 1)
            try:
                if orig_n is not None and horizon is not None:
                    cfg.n_action_steps = max(int(orig_n), int(horizon) - (n_obs - 1))
                return __orig(self, b, *args, **kwargs)
            finally:
                if orig_n is not None:
                    cfg.n_action_steps = orig_n

        _patched._async_queue_patched = True
        P.predict_action_chunk = _patched
        logger.info("Async queue-patch applied to %s.predict_action_chunk", P.__name__)


def launch_policy_server_thread(host: str, port: int, fps: int, inference_latency: float | None = None) -> threading.Thread:
    """Start LeRobot's ``policy_server`` in a daemon thread (blocks internally on serve())."""
    from lerobot.async_inference.configs import PolicyServerConfig
    from lerobot.async_inference.policy_server import serve

    kwargs = {"host": host, "port": port, "fps": fps}
    if inference_latency is not None:
        kwargs["inference_latency"] = inference_latency
    cfg = PolicyServerConfig(**kwargs)

    def _run():
        logger.info("Starting async policy_server on %s:%d", host, port)
        serve(cfg)

    t = threading.Thread(target=_run, name="async-policy-server", daemon=True)
    t.start()
    return t


class AsyncPolicyClient:
    """Drop-in async replacement for :class:`LerobotPolicy` (same ``predict`` interface)."""

    def __init__(
        self,
        policy_path: str,
        repo_id: str,
        policy_type: str,
        device: str = "cuda",
        host: str = "127.0.0.1",
        port: int = 8080,
        fps: int = 30,
        actions_per_chunk: int = 50,
        chunk_size_threshold: float = 0.5,
        aggregate_fn_name: str = "weighted_average",
        dof: int = 7,
        task: str = "",
        launch_server: bool = True,
    ):
        # Lazy imports: nothing here is touched unless async is actually enabled.
        import grpc
        from lerobot.async_inference.configs import get_aggregate_function
        from lerobot.async_inference.helpers import RemotePolicyConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.transport import services_pb2, services_pb2_grpc
        from lerobot.transport.utils import grpc_channel_options

        self._grpc = grpc
        self._pb2 = services_pb2
        self.dof = dof
        self.task = task
        self.policy_type = policy_type
        self.actions_per_chunk = actions_per_chunk
        self.chunk_size_threshold = chunk_size_threshold
        self.aggregate_fn = get_aggregate_function(aggregate_fn_name)

        if lerobot_policy_name(policy_type) == "multi_task_dit":
            enable_multitask_dit_async()
        # DiT/diffusion need their obs queue populated before predict_action_chunk (see function).
        patch_chunk_policy_for_async(policy_type)

        # Dataset metadata gives us the lerobot_features mapping + action names.
        self.dataset = LeRobotDataset(repo_id, batch_encoding_size=1)
        self.lerobot_features = self.dataset.features
        self.action_names = self.dataset.features["action"]["names"]

        # Same robot-observation processor LerobotPolicy applies before build_dataset_frame.
        # The async server runs build_dataset_frame itself, so we must hand it the *processed*
        # obs — otherwise it finds no image/state tensors ("stack expects a non-empty TensorList").
        from lerobot.processor import make_default_processors

        _, _, self.robot_observation_processor = make_default_processors()

        if launch_server:
            launch_policy_server_thread(host, port, fps)
            time.sleep(2.0)  # give the server a moment to bind before we connect

        # gRPC channel + handshake (mirrors RobotClient.start()).
        environment_dt = 1.0 / fps
        self.channel = grpc.insecure_channel(
            f"{host}:{port}", grpc_channel_options(initial_backoff=f"{environment_dt:.4f}s")
        )
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)

        self.policy_config = RemotePolicyConfig(
            lerobot_policy_name(policy_type),
            policy_path,
            self.lerobot_features,
            actions_per_chunk,
            device,
        )

        from lerobot.async_inference.robot_client import RobotClient

        self._RobotClient = RobotClient  # reuse its tested send/receive/aggregate methods

        # State shared with the receiver thread.
        from queue import Queue

        self.action_queue = Queue()
        self.action_queue_lock = threading.Lock()
        self.action_queue_size = []
        self.shutdown_event = threading.Event()
        self.latest_action_lock = threading.Lock()
        self.latest_action = -1
        self.action_chunk_size = actions_per_chunk
        self._chunk_size_threshold = chunk_size_threshold
        self.must_go = threading.Event()
        self.must_go.set()
        # receive_actions() waits on this barrier to sync with the control loop. We have no
        # second party (the control loop is the robot_server's, not a thread here), so a
        # Barrier(1) lets the receiver's .wait() return immediately.
        self.start_barrier = threading.Barrier(1)
        self._timestep = 0
        self._last_left, self._last_right = {}, {}  # held when the queue is momentarily empty
        self.logger = logger
        # The borrowed RobotClient.receive_actions reads self.config.{aggregate_fn,client_device}.
        import types

        self.config = types.SimpleNamespace(aggregate_fn=self.aggregate_fn, client_device="cpu")

        self._connect()
        self._receiver = threading.Thread(target=self._receive_actions, name="async-action-receiver", daemon=True)
        self._receiver.start()

    # ---- connection ----
    def _connect(self) -> None:
        import pickle  # nosec

        self.stub.Ready(self._pb2.Empty())
        self.stub.SendPolicyInstructions(self._pb2.PolicySetup(data=pickle.dumps(self.policy_config)))
        self.shutdown_event.clear()
        logger.info("Async client connected; policy=%s device=%s", self.policy_config.policy_type, self.policy_config.device)

    # Borrow RobotClient's queue/aggregation/receive logic (they only touch self.* attrs we set above).
    def _receive_actions(self):
        self._RobotClient.receive_actions(self)

    def _aggregate_action_queues(self, incoming_actions, aggregate_fn=None):
        return self._RobotClient._aggregate_action_queues(self, incoming_actions, aggregate_fn or self.aggregate_fn)

    def _inspect_action_queue(self):
        return self._RobotClient._inspect_action_queue(self)

    @property
    def running(self):
        return not self.shutdown_event.is_set()

    # ---- observation / action plumbing ----
    def _build_raw_observation(self, left_joints: dict, right_joints: dict, cams: dict, single_arm_side: str) -> dict:
        """Build the raw obs dict the server turns into a policy input via lerobot_features.

        ⚠️ VERIFY ON HARDWARE: keys must match what build_dataset_frame(lerobot_features, .)
        expects — same construction as LerobotPolicy.predict (joint_i + camera keys).
        """
        obs = {k.replace("observation.images.", ""): v for k, v in cams.items()}
        joint_names = [f"joint_{i}" for i in range(self.dof)]
        right_has_all = all(f"{n}.pos" in right_joints for n in joint_names)
        left_has_all = all(f"{n}.pos" in left_joints for n in joint_names)
        state = right_joints if right_has_all else (left_joints if left_has_all else right_joints)
        for n in joint_names:
            if f"{n}.pos" in state:
                obs[n] = state[f"{n}.pos"]
        for k, v in left_joints.items():
            obs["L." + k.replace(".pos", "")] = v
        for k, v in right_joints.items():
            obs["R." + k.replace(".pos", "")] = v
        # Apply the same processor LerobotPolicy uses, then attach the task (the server reads
        # it from the raw observation for language-conditioned policies).
        batch = self.robot_observation_processor(obs)
        if self.task:
            batch["task"] = self.task
        return batch

    def _send_observation(self, raw_obs: dict, must_go: bool) -> None:
        from lerobot.async_inference.helpers import TimedObservation

        # The obs timestep MUST track action consumption (== latest executed action), like
        # RobotClient does. The server timestamps the chunk's actions starting at this value;
        # if it lags behind latest_action the aggregator discards the whole chunk as stale and
        # the queue starves (arm runs one chunk then stops).
        with self.latest_action_lock:
            ts = max(self.latest_action, 0)
        timed = TimedObservation(
            timestamp=time.time(), observation=raw_obs, timestep=ts, must_go=must_go
        )
        self._RobotClient.send_observation(self, timed)

    def _action_to_dicts(self, action_tensor, single_arm_side: str):
        """Map an action tensor -> (dict_left, dict_right), mirroring LerobotPolicy.convert_actions_to_dict.

        ⚠️ VERIFY ON HARDWARE: action order must match self.action_names.
        """
        dict_left, dict_right = {}, {}
        for i, name in enumerate(self.action_names):
            v = float(action_tensor[i])
            if name.startswith("L."):
                dict_left[name.replace("L.", "") + ".pos"] = v
            elif name.startswith("R."):
                dict_right[name.replace("R.", "") + ".pos"] = v
            elif name.startswith("joint_"):
                key = name if name.endswith(".pos") else f"{name}.pos"
                (dict_left if single_arm_side == "left" else dict_right)[key] = v
        return dict_left, dict_right

    # ---- the LerobotPolicy-compatible entry point ----
    def predict(self, left_joints: dict, right_joints: dict, cams: dict, dof: int = 7, single_arm_side: str = "right"):
        """Non-blocking: send the obs if the queue is low, return the next queued action.

        Never blocks — if the queue is momentarily empty (startup, or the server fell behind)
        it returns the last action so the control loop and gamepad stay responsive and the arm
        holds its pose until fresh actions arrive.
        """
        raw_obs = self._build_raw_observation(left_joints, right_joints, cams, single_arm_side)

        with self.action_queue_lock:
            qsize = self.action_queue.qsize()
        # must_go=True makes the SERVER bypass its dedup/similarity filter and force inference.
        # It MUST be set only when our queue is truly EMPTY (mirroring RobotClient). The old code
        # forced must_go on every send: the server then ran inference on every observation we sent
        # (~8/s) but can only do ~3/s, so its observation queue backed up with stale low-timestep
        # observations until every returned chunk was entirely <= latest_action -> the aggregator
        # dropped it all -> permanent starvation ("moves a bit, then freezes"). With must_go gated
        # on empty, non-empty-queue sends go must_go=False and the server's filter throttles them
        # to its own throughput, so no backup and no starvation.
        must_go = self.must_go.is_set() and qsize == 0
        if must_go or qsize <= int(self.chunk_size_threshold * self.actions_per_chunk):
            self._send_observation(raw_obs, must_go)
            if must_go:
                self.must_go.clear()

        try:
            with self.action_queue_lock:
                timed_action = self.action_queue.get_nowait()
        except Empty:
            return self._last_left, self._last_right  # hold last pose, stay responsive

        with self.latest_action_lock:
            self.latest_action = timed_action.get_timestep()
        self._last_left, self._last_right = self._action_to_dicts(timed_action.get_action(), single_arm_side)
        return self._last_left, self._last_right

    def reset(self) -> None:
        from queue import Queue

        with self.action_queue_lock:
            self.action_queue = Queue()
        with self.latest_action_lock:
            self.latest_action = -1  # restart obs timesteps at 0 for the new engage
        self.must_go.set()
        self._timestep = 0
        self._last_left, self._last_right = {}, {}  # don't snap back to the previous run's pose

    def stop(self) -> None:
        self.shutdown_event.set()
        try:
            self.channel.close()
        except Exception:
            pass
