"""Thread-safe ring buffer for IMU samples used during experiments."""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ImuSample:
    t: float   # seconds, host monotonic
    ax: float
    ay: float
    az: float


class ImuBuffer:
    """Lock-free-on-read ring buffer of recent IMU samples.

    Sized by maximum capture duration; at 1 kHz an 8-second window is 8000
    samples, well within memory and quick to copy out.
    """

    def __init__(self, max_seconds: float = 20.0,
                 sample_rate_hz: float = 1000.0) -> None:
        self._capacity = int(max_seconds * sample_rate_hz)
        self._buf: deque[ImuSample] = deque(maxlen=self._capacity)
        self._lock = threading.Lock()
        self._active = False
        self._start_t: float | None = None

    def push(self, sample: ImuSample) -> None:
        with self._lock:
            self._buf.append(sample)

    def start(self, t0: float) -> None:
        """Begin a capture window at host time ``t0``."""
        with self._lock:
            self._buf.clear()
            self._active = True
            self._start_t = t0

    def stop(self) -> None:
        with self._lock:
            self._active = False

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def snapshot(self, since_t: float | None = None,
                 until_t: float | None = None
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return ``(t, ax, ay, az)`` filtered to ``[since_t, until_t]``."""
        with self._lock:
            items = list(self._buf)
        if not items:
            empty = np.zeros(0, dtype=float)
            return empty, empty, empty, empty
        ts = np.fromiter((s.t for s in items), dtype=float, count=len(items))
        ax = np.fromiter((s.ax for s in items), dtype=float, count=len(items))
        ay = np.fromiter((s.ay for s in items), dtype=float, count=len(items))
        az = np.fromiter((s.az for s in items), dtype=float, count=len(items))
        mask = np.ones_like(ts, dtype=bool)
        if since_t is not None:
            mask &= ts >= since_t
        if until_t is not None:
            mask &= ts <= until_t
        return ts[mask], ax[mask], ay[mask], az[mask]
