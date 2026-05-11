"""Experiment state machine.

Drives a single ``VelocityProfile`` over wall time and optionally captures
IMU samples while the profile is running. The state machine has four
states:

  IDLE              -> publish 0 m/s
  RUNNING           -> publish profile.v(t - t_start) at each tick
  CANCELLING        -> linear ramp from the last commanded v down to 0
  IDLE (post-run)   -> emit completion event, return to IDLE

All transitions are guarded by a single lock. The runner does *not* know
about the shaper -- it returns the *raw* commanded velocity. The caller
applies shaping outside the runner so that runner outputs are usable both
for plotting (raw command) and for the shaped publication path.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .motion import VelocityProfile


@dataclass(frozen=True)
class ExperimentResult:
    profile: VelocityProfile
    started_at: float    # host time, seconds
    finished_at: float
    cancelled: bool
    raw_v: np.ndarray    # what we commanded each tick (size = #ticks)
    tick_t: np.ndarray   # host-time of each tick


_COMPLETION_CB = Callable[[ExperimentResult], None]


class ExperimentRunner:
    """Single-experiment-at-a-time runner.

    Designed for cooperative use from one ROS timer (~50 Hz) plus the Flask
    request thread. The runner never blocks; ``tick`` always returns
    promptly.
    """

    # Safety: when cancelling, slow down at this acceleration (m/s^2).
    CANCEL_DECELERATION = 0.5

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = 'IDLE'
        self._profile: VelocityProfile | None = None
        self._start_t: float = 0.0
        self._last_v: float = 0.0
        self._on_complete: _COMPLETION_CB | None = None
        self._raw_log: list[float] = []
        self._tick_log: list[float] = []
        self._cancel_v: float = 0.0
        self._cancel_t0: float = 0.0

    # -------- public API --------

    @property
    def is_idle(self) -> bool:
        with self._lock:
            return self._state == 'IDLE'

    def start(self, profile: VelocityProfile,
              on_complete: _COMPLETION_CB | None = None) -> None:
        """Start a profile. Raises if another experiment is in progress."""
        with self._lock:
            if self._state != 'IDLE':
                raise RuntimeError(f'cannot start: state is {self._state}')
            self._profile = profile
            self._start_t = time.monotonic()
            self._last_v = 0.0
            self._on_complete = on_complete
            self._raw_log = []
            self._tick_log = []
            self._state = 'RUNNING'

    def cancel(self) -> None:
        """Request graceful cancellation: ramp down to 0 over CANCEL_DECEL."""
        with self._lock:
            if self._state != 'RUNNING':
                return
            self._cancel_v = self._last_v
            self._cancel_t0 = time.monotonic()
            self._state = 'CANCELLING'

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'state': self._state,
                'profile_label': self._profile.label if self._profile else None,
                'elapsed': (time.monotonic() - self._start_t
                            if self._state in ('RUNNING', 'CANCELLING')
                            else 0.0),
                'duration': (self._profile.duration if self._profile
                             else 0.0),
                'last_v': self._last_v,
            }

    def tick(self) -> float:
        """Compute the velocity command for this control period.

        Returns the *raw* commanded velocity (m/s). The publisher should
        apply the shaper to the returned value before publishing.
        """
        now = time.monotonic()
        deferred: tuple[_COMPLETION_CB, ExperimentResult] | None = None
        with self._lock:
            if self._state == 'IDLE':
                self._last_v = 0.0
                return 0.0

            if self._state == 'CANCELLING':
                dt = now - self._cancel_t0
                v = self._cancel_v - np.sign(self._cancel_v) * \
                    self.CANCEL_DECELERATION * dt
                if (np.sign(v) != np.sign(self._cancel_v)) or v == 0.0:
                    deferred = self._prepare_finish(cancelled=True)
                else:
                    self._last_v = float(v)
                    return self._last_v
            else:
                # RUNNING.
                assert self._profile is not None
                elapsed = now - self._start_t
                if elapsed >= self._profile.duration:
                    deferred = self._prepare_finish(cancelled=False)
                else:
                    v = float(np.interp(elapsed, self._profile.t,
                                        self._profile.v))
                    self._last_v = v
                    self._raw_log.append(v)
                    self._tick_log.append(now)
                    return v

        # Run completion callback outside the lock so it can safely call
        # back into the runner without re-entrancy worries.
        if deferred is not None:
            cb, result = deferred
            cb(result)
        return 0.0

    # -------- internals --------

    def _prepare_finish(self, cancelled: bool
                        ) -> tuple[_COMPLETION_CB, ExperimentResult] | None:
        """Snapshot the run, reset state, return ``(callback, result)``.

        Caller must invoke ``callback(result)`` after releasing the lock.
        Returns ``None`` if no completion callback was registered.
        """
        profile = self._profile
        on_complete = self._on_complete
        result = ExperimentResult(
            profile=profile,  # type: ignore[arg-type]
            started_at=self._start_t,
            finished_at=time.monotonic(),
            cancelled=cancelled,
            raw_v=np.array(self._raw_log, dtype=float),
            tick_t=np.array(self._tick_log, dtype=float),
        )
        self._profile = None
        self._on_complete = None
        self._raw_log = []
        self._tick_log = []
        self._state = 'IDLE'
        self._last_v = 0.0
        if on_complete is None:
            return None
        return on_complete, result
