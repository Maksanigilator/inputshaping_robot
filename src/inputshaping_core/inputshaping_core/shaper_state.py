"""Thread-safe wrapper around the input shaper.

The shaper is reconfigured at runtime from the GUI; the publisher timer
samples it from a ROS callback thread; the experiment runner reads it from
another thread. A single ``RLock`` guards reads and writes.
"""

from __future__ import annotations

import threading
from collections import deque

from . import shapers as _shapers


class ShaperState:
    """Encapsulates the active shaper plus the FIR-like sample buffer.

    The buffer stores velocity samples at a fixed period ``dt``. On each
    ``step`` the latest raw velocity is pushed and the convolution with the
    shaper impulses is evaluated; the result is the shaped velocity command
    for *this* tick.
    """

    def __init__(self, dt: float = 0.02,
                 max_delay_s: float = 1.0) -> None:
        self._lock = threading.RLock()
        self._dt = float(dt)
        self._max_delay_s = float(max_delay_s)
        # Buffer length sized for the worst-case shaper delay we allow.
        self._buf_len = max(2, int(round(self._max_delay_s / self._dt)) + 1)
        self._buf: deque[float] = deque(maxlen=self._buf_len)
        # Active shaper. ``None`` means pure bypass.
        self._shaper: _shapers.InputShaper | None = None
        self._enabled = False

    @property
    def dt(self) -> float:
        return self._dt

    def configure(self, name: str | None, frequency: float,
                  damping_ratio: float, enabled: bool) -> None:
        """Update the active shaper.

        ``name=None`` or ``enabled=False`` disables shaping (pass-through).
        """
        with self._lock:
            if name and enabled:
                self._shaper = _shapers.make_shaper(
                    name, frequency, damping_ratio)
                if self._shaper.delay > self._max_delay_s:
                    raise ValueError(
                        f'shaper delay {self._shaper.delay:.3f} s exceeds '
                        f'configured max_delay_s={self._max_delay_s:.3f} s')
            else:
                self._shaper = None
            self._enabled = bool(enabled and name)

    def reset(self) -> None:
        """Clear the buffer (e.g. between experiments)."""
        with self._lock:
            self._buf.clear()

    def snapshot(self) -> dict:
        with self._lock:
            if self._shaper is None:
                return {
                    'enabled': self._enabled,
                    'name': None,
                    'frequency': None,
                    'damping_ratio': None,
                    'delay': 0.0,
                    'smoothing': 0.0,
                }
            d = self._shaper.to_dict()
            d['enabled'] = self._enabled
            return d

    def step(self, raw_velocity: float) -> float:
        """Advance one ``dt`` step and return the shaped velocity."""
        with self._lock:
            self._buf.append(float(raw_velocity))
            if self._shaper is None or not self._enabled:
                return float(raw_velocity)
            # Buffer convention: index ``-1`` is now (the just-appended raw
            # sample), index ``-1-k`` is k samples in the past.
            shaped = 0.0
            n = len(self._buf)
            for imp in self._shaper.impulses:
                # Impulse at time t_i delays the input by (t_max - t_i).
                shift_s = self._shaper.delay - imp.time
                k = int(round(shift_s / self._dt))
                idx = -1 - k
                # Clamp to the oldest sample we have (warm-up transient).
                if -idx > n:
                    idx = -n
                shaped += imp.amplitude * self._buf[idx]
            return shaped
