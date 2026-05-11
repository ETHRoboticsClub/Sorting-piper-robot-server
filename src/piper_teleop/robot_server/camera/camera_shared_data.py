import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np
from piper_teleop.robot_server.camera.camera_config import CameraConfig, CameraMode

class SingleCameraSharedData:
    def __init__(self, name: str,
                 frame_shape: tuple[int, int, int],
                 capacity: int = 8,
                 ctx: mp.context.BaseContext | None = None,
                 ):

        assert 2<=capacity<16, "capacity must be between 2 and 16"
        self.name = name
        self.capacity = capacity
        self.frame_shape = frame_shape
        self.dtype = np.uint8
        self._shm_owner = True

        # Calculate memory requirements
        shape = (capacity,) + frame_shape
        frame_bytes = np.prod(frame_shape) * np.dtype(self.dtype).itemsize
        total_bytes = frame_bytes * capacity

        # Create or attach to shared memory
        shm_name = f"cam_{name}_dat"
        try:
            existing_shm = shared_memory.SharedMemory(name=shm_name, create=False)
            existing_shm.close()
            existing_shm.unlink()
        except FileNotFoundError:
            pass  # Doesn't exist, which is fine

        self.shm = shared_memory.SharedMemory(name=shm_name, create=True, size=total_bytes)
        self.shm.buf[:] = b"\x00" * total_bytes  # optional zeroing
        mp_ctx = ctx or mp.get_context()
        self.write_counter = mp_ctx.Value("Q", 0, lock=True)

        # Create numpy array view of shared memory
        self.array = np.ndarray(shape, dtype=self.dtype, buffer=self.shm.buf)

    def __getstate__(self):
        state = self.__dict__.copy()
        state["_shm_name"] = self.shm.name
        state.pop("shm", None)
        state.pop("array", None)
        state["_shm_owner"] = False
        return state

    def __setstate__(self, state):
        shm_name = state.pop("_shm_name")
        self.__dict__.update(state)
        self.shm = shared_memory.SharedMemory(name=shm_name, create=False)
        shape = (self.capacity,) + self.frame_shape
        self.array = np.ndarray(shape, dtype=self.dtype, buffer=self.shm.buf)

    def write(self, frame: np.ndarray) -> int:
        assert frame.shape == self.frame_shape, 'wrong frame shape'

        if not frame.flags['C_CONTIGUOUS']:
            frame = np.ascontiguousarray(frame)

        if frame.dtype != self.dtype:
            frame = frame.astype(self.dtype)

        with self.write_counter.get_lock():
            count = self.write_counter.value
            slot = count % self.capacity
            np.copyto(self.array[slot], frame)
            self.write_counter.value = (slot + 1) % self.capacity
        return slot

    def read(self) -> np.ndarray:
        count = self.write_counter.value
        slot = (count - 1) % self.capacity
        frame = self.array[slot]
        return frame

    def __del__(self):
        try:
            self.shm.close()
            if getattr(self, "_shm_owner", False):
                try:
                    self.shm.unlink()
                except FileNotFoundError:
                    pass
        except Exception:
            pass


class SharedCameraData:
    def __init__(
        self,
        configs: list[CameraConfig],
        capacity: int = 8,
        ctx: mp.context.BaseContext | None = None,
    ):
        self.cameras: dict[str, SingleCameraSharedData] = {}
        for cfg in configs:
            if cfg.mode not in (CameraMode.RECORDING, CameraMode.HYBRID):
                continue
            frame_shape = (cfg.frame_height, cfg.frame_width, 3)
            self.cameras[cfg.name] = SingleCameraSharedData(
                name=cfg.name,
                frame_shape=frame_shape,
                capacity=capacity,
                ctx=ctx,
            )
    def read(self, name: str) -> np.ndarray:
        return self.cameras[name].read()

    def copy(self, name: str, frame: np.ndarray) -> int:
        return self.cameras[name].write(frame)

    def get_camera_dict(self) -> dict[str, np.ndarray]:
        result = {}
        for name, camera in self.cameras.items():
            frame = camera.read()
            result[f'observation.images.{name}'] = frame
        return result

