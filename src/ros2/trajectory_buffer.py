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
        check_tolerance: float = 0.01,
        logger=None,
    ):
        self._ur = ur
        self._logger = logger

        self.flush_interval = flush_interval
        self.min_batch_size = min_batch_size
        self.max_batch_size = max_batch_size
        self.max_buffer_age = max_buffer_age
        self.blend_radius = blend_radius
        self.check_tolerance = check_tolerance

        self._lock = threading.Lock()
        self._buffer: list[list] = []
        self._tick_ids: list[int | None] = []
        self._oldest_item_time: float | None = None
        self._last_append_time: float | None = None
        self._state = BufferState.IDLE
        self._last_batch_target: list[float] | None = None
        self._collision_stopped = False

        # Tick tracking for the currently executing batch
        self._current_batch_joints: list[list[float]] = []
        self._current_batch_ticks: list[int | None] = []
        self._current_tick: int | None = None

        # Per-batch stats for logging
        self._batch_size: int = 0
        self._batch_ticks_seen: set[int] = set()
        self._batch_send_time: float = 0.0
        self._batch_check_count: int = 0

    def append(self, waypoint: list, tick_id: int | None = None) -> None:
        """Called from subscriber callback. Thread-safe."""
        with self._lock:
            if self._collision_stopped:
                return
            self._buffer.append(waypoint)
            self._tick_ids.append(tick_id)
            now = time.time()
            if self._oldest_item_time is None:
                self._oldest_item_time = now
            self._last_append_time = now

    def clear(self) -> None:
        """Clear buffer and reject new waypoints until resume()."""
        with self._lock:
            self._buffer.clear()
            self._tick_ids.clear()
            self._oldest_item_time = None
            self._last_append_time = None
            self._state = BufferState.COLLISION_STOPPED
            self._collision_stopped = True
            self._current_batch_joints = []
            self._current_batch_ticks = []
            self._current_tick = None

    def resume(self) -> None:
        """Resume accepting waypoints after collision recovery."""
        with self._lock:
            self._collision_stopped = False
            self._state = BufferState.IDLE

    def try_flush(self) -> bool:
        """
        Called from the executor timer. Returns True if a batch was sent.

        1. If currently EXECUTING, check if batch finished. If not, update
           _current_tick based on where the arm is in the batch.
        2. If IDLE with buffer items, check flush conditions and send batch.
        """
        with self._lock:
            if self._collision_stopped:
                return False

            # Check if previous batch finished
            if self._state == BufferState.EXECUTING:
                if self._last_batch_target is not None:
                    if not self._ur.check_if_joints_is_reached(
                        self._last_batch_target, tolerance=self.check_tolerance
                    ):
                        # Still executing — update which waypoint we're heading toward
                        self._batch_check_count += 1
                        current_joints = self._ur.get_joints(read=False)
                        self._current_tick = self._compute_tick(
                            current_joints,
                            self._current_batch_joints,
                            self._current_batch_ticks,
                        )
                        if self._current_tick is not None:
                            self._batch_ticks_seen.add(self._current_tick)
                        return False  # Still executing
                elapsed = time.time() - self._batch_send_time
                self._state = BufferState.IDLE
                final_tick = self._current_batch_ticks[-1] if self._current_batch_ticks else None
                self._current_batch_joints = []
                self._current_batch_ticks = []
                self._current_tick = final_tick  # publish last tick with current joints
                if self._logger is not None:
                    ticks = sorted(self._batch_ticks_seen)
                    tick_range = f"ticks {ticks[0]}-{ticks[-1]}" if ticks else "no ticks"
                    hz = self._batch_check_count / elapsed if elapsed > 0 else 0
                    self._logger.info(
                        f"Batch complete: {self._batch_size} waypoints | "
                        f"elapsed={elapsed:.2f}s | "
                        f"checks={self._batch_check_count} ({hz:.0f}Hz) | "
                        f"{len(ticks)} unique ticks ({tick_range})"
                    )

            if len(self._buffer) == 0:
                return False

            now = time.time()
            # Age since the sender last appended a waypoint — goes stale when the
            # trajectory stream pauses/ends, triggering a trailing flush even if
            # the buffer hasn't reached min_batch_size yet.
            idle_since_last_append = (now - self._last_append_time) if self._last_append_time else 0

            should_flush = (
                len(self._buffer) >= self.max_batch_size
                or len(self._buffer) >= self.min_batch_size
                or idle_since_last_append >= self.max_buffer_age
            )

            if not should_flush:
                return False

            # Extract batch
            batch = self._buffer[:self.max_batch_size]
            batch_ticks = self._tick_ids[:self.max_batch_size]
            self._buffer = self._buffer[self.max_batch_size:]
            self._tick_ids = self._tick_ids[self.max_batch_size:]
            if len(self._buffer) == 0:
                self._oldest_item_time = None
            else:
                self._oldest_item_time = now

            self._last_batch_target = list(batch[-1][:6])
            self._current_batch_joints = [list(w[:6]) for w in batch]
            self._current_batch_ticks = list(batch_ticks)
            self._state = BufferState.EXECUTING
            self._batch_size = len(batch)
            self._batch_ticks_seen = set()
            self._batch_send_time = time.time()
            self._batch_check_count = 0
            if self._logger is not None:
                self._logger.info(
                    f"Batch sent: {len(batch)} waypoints "
                    f"(ticks {batch_ticks[0]}-{batch_ticks[-1]})"
                )
                # if len(batch) >= 2:
                #     pair_means = []
                #     lines = []
                #     for i in range(len(batch) - 1):
                #         diffs = [abs(batch[i + 1][j] - batch[i][j]) for j in range(6)]
                #         mean = sum(diffs) / 6
                #         pair_means.append(mean)
                #         joints_str = " ".join(f"j{j+1}:{diffs[j]:.4f}" for j in range(6))
                #         lines.append(f"  {i+1}-{i+2}: {joints_str}  mean:{mean:.4f}")
                #     lines.append(
                #         f"  min_mean:{min(pair_means):.4f}  max_mean:{max(pair_means):.4f}"
                #     )
                #     self._logger.info("Waypoint diffs:\n" + "\n".join(lines))

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

    @staticmethod
    def _compute_tick(
        current_joints: list[float],
        batch_joints: list[list[float]],
        batch_ticks: list[int | None],
    ) -> int | None:
        """
        Project current joint position onto each segment of the executing batch.
        Returns the tick_id of the target waypoint of the active segment.
        Returns None if fewer than 2 waypoints or all ticks are None.
        """
        if len(batch_joints) < 2 or all(t is None for t in batch_ticks):
            return None

        for i in range(len(batch_joints) - 1):
            qi = batch_joints[i]
            qi1 = batch_joints[i + 1]
            d = [qi1[j] - qi[j] for j in range(6)]
            d_sq = sum(x * x for x in d)
            if d_sq < 1e-9:
                continue
            dot = sum((current_joints[j] - qi[j]) * d[j] for j in range(6))
            alpha = dot / d_sq
            if alpha < 0.0:
                # Before this segment — arm hasn't reached waypoint i yet
                return batch_ticks[i]
            if alpha <= 1.0:
                # On this segment — heading toward waypoint i+1
                return batch_ticks[i + 1]

        # Past all segments — at or beyond the final waypoint
        return batch_ticks[-1]

    def update_current_tick(self) -> None:
        """
        Recompute _current_tick from the arm's current joint position.
        Call at high frequency (e.g. 200 Hz) to track every waypoint
        without waiting for the batch-flush cycle.
        """
        with self._lock:
            if self._state == BufferState.EXECUTING and self._current_batch_joints:
                current_joints = self._ur.get_joints(read=False)
                self._current_tick = self._compute_tick(
                    current_joints,
                    self._current_batch_joints,
                    self._current_batch_ticks,
                )
                if self._current_tick is not None:
                    self._batch_ticks_seen.add(self._current_tick)

    @property
    def current_tick(self) -> int | None:
        """The tick_id of the waypoint currently being executed toward. None if idle."""
        with self._lock:
            return self._current_tick

    @property
    def buffer_size(self) -> int:
        with self._lock:
            return len(self._buffer)

    @property
    def state(self) -> BufferState:
        with self._lock:
            return self._state
