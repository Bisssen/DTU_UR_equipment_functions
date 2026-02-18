import threading
import time
from enum import Enum, auto
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..ur.class_ur import UR


class BufferState(Enum):
    IDLE = auto()
    EXECUTING = auto()
    COLLISION_STOPPED = auto()


class TrajectoryBuffer:
    """
    Thread-safe buffer for batching joint waypoints before sending
    them to the UR controller via path().
    """

    def __init__(
        self,
        ur: 'UR',
        flush_interval: float = 0.1,
        min_batch_size: int = 2,
        max_batch_size: int = 50,
        max_buffer_age: float = 0.2,
        blend_radius: float = 0.05,
    ):
        self._ur = ur

        self.flush_interval = flush_interval
        self.min_batch_size = min_batch_size
        self.max_batch_size = max_batch_size
        self.max_buffer_age = max_buffer_age
        self.blend_radius = blend_radius

        self._lock = threading.Lock()
        self._buffer: list[list] = []
        self._oldest_item_time: float | None = None
        self._state = BufferState.IDLE
        self._last_batch_target: list[float] | None = None
        self._collision_stopped = False

    def append(self, waypoint: list) -> None:
        """Called from subscriber callback. Thread-safe."""
        with self._lock:
            if self._collision_stopped:
                return
            self._buffer.append(waypoint)
            if self._oldest_item_time is None:
                self._oldest_item_time = time.time()

    def clear(self) -> None:
        """Clear buffer and reject new waypoints until resume()."""
        with self._lock:
            self._buffer.clear()
            self._oldest_item_time = None
            self._state = BufferState.COLLISION_STOPPED
            self._collision_stopped = True

    def resume(self) -> None:
        """Resume accepting waypoints after collision recovery."""
        with self._lock:
            self._collision_stopped = False
            self._state = BufferState.IDLE

    def try_flush(self) -> bool:
        """
        Called from the executor timer. Returns True if a batch was sent.

        1. If currently EXECUTING, check if batch finished. If not, return.
        2. If IDLE with buffer items, check flush conditions and send batch.
        """
        with self._lock:
            if self._collision_stopped:
                return False

            # Check if previous batch finished
            if self._state == BufferState.EXECUTING:
                if self._last_batch_target is not None:
                    if not self._ur.check_if_joints_is_reached(
                        self._last_batch_target
                    ):
                        return False  # Still executing
                self._state = BufferState.IDLE

            if len(self._buffer) == 0:
                return False

            now = time.time()
            buffer_age = (now - self._oldest_item_time) if self._oldest_item_time else 0

            should_flush = (
                len(self._buffer) >= self.max_batch_size
                or len(self._buffer) >= self.min_batch_size
                or buffer_age >= self.max_buffer_age
            )

            if not should_flush:
                return False

            # Extract batch
            batch = self._buffer[:self.max_batch_size]
            self._buffer = self._buffer[self.max_batch_size:]
            if len(self._buffer) == 0:
                self._oldest_item_time = None
            else:
                self._oldest_item_time = now

            self._last_batch_target = list(batch[-1][:6])
            self._state = BufferState.EXECUTING

        # Send batch outside lock to avoid blocking subscriber callback
        self._ur.path(
            batch,
            False,
            False,
            None,
            None,
            False,
            r=self.blend_radius,
        )
        return True

    @property
    def buffer_size(self) -> int:
        with self._lock:
            return len(self._buffer)

    @property
    def state(self) -> BufferState:
        with self._lock:
            return self._state
