"""Input shapers for mobile-robot velocity commands.

Formulas follow the classic Singer & Seering derivations and match the
implementation in Klipper's ``klippy/extras/shaper_calibrate.py`` so that
calibration results are directly comparable.

For a single resonance of natural frequency ``f`` (rad: ``omega = 2*pi*f``) and
damping ratio ``zeta``, a shaper is a list of impulses ``(A_i, t_i)``. The
shaped command is the convolution of the raw command with these impulses.

Residual-vibration metric ``V(f, zeta)`` is the magnitude of the system's
post-input oscillation after a unit step input through the shaper; ``V = 0``
exactly at the design frequency for a ZV shaper.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Impulse:
    amplitude: float
    time: float


# Note: damping ratio is normally a couple percent for our IMU-on-flexure mount.
# Klipper defaults to 0.1 because it is robust across many printer mechanics;
# a mobile robot with a flexible mast is usually closer to 0.02-0.05.
DEFAULT_DAMPING_RATIO = 0.05
DEFAULT_VIBR_TOLERANCE = 0.05


class InputShaper:
    """Discrete-impulse input shaper.

    The implementation operates entirely on velocity samples (m/s vs time).
    The output is delayed by ``self.delay`` relative to the input so that
    the very first impulse (``t_i = 0``) takes effect on the latest sample
    and the rest reach back into the buffer.
    """

    def __init__(self, name: str, impulses: list[Impulse], frequency: float,
                 damping_ratio: float) -> None:
        if not impulses:
            raise ValueError('at least one impulse is required')
        self.name = name
        self.frequency = frequency
        self.damping_ratio = damping_ratio
        # Normalize amplitudes so they sum to 1 (DC gain = 1).
        total = sum(i.amplitude for i in impulses)
        if total <= 0:
            raise ValueError('shaper amplitudes must sum to a positive value')
        # Shift times so the earliest impulse is at t=0 (no extra delay).
        t0 = min(i.time for i in impulses)
        self.impulses = [Impulse(i.amplitude / total, i.time - t0)
                         for i in impulses]

    @property
    def delay(self) -> float:
        """Worst-case delay introduced by the shaper, in seconds."""
        return max(i.time for i in self.impulses)

    @property
    def smoothing(self) -> float:
        """Klipper-style smoothing metric. Larger = smoother but more lag.

        smoothing = sum_i A_i * (t_max - t_i)^2 / 2
        """
        t_max = self.delay
        return 0.5 * sum(i.amplitude * (t_max - i.time) ** 2
                         for i in self.impulses)

    def apply(self, times: np.ndarray, values: np.ndarray) -> np.ndarray:
        """Convolve a uniformly-sampled signal with the shaper impulses.

        Samples must be uniform in time. Output is the same length; the
        leading ``self.delay`` seconds contain a warm-up transient.
        """
        if times.shape != values.shape:
            raise ValueError('times and values must have the same shape')
        if len(times) < 2:
            return values.copy()
        dt = float(times[1] - times[0])
        out = np.zeros_like(values, dtype=float)
        for imp in self.impulses:
            k = int(round(imp.time / dt))
            if k <= 0:
                out += imp.amplitude * values
            else:
                out[k:] += imp.amplitude * values[:-k]
                # First k samples: hold the first input value scaled by A.
                out[:k] += imp.amplitude * values[0]
        return out

    def residual_vibration(self, freq: float,
                           damping_ratio: float | None = None) -> float:
        """Residual vibration amplitude at frequency ``freq``.

        See Klipper's ``_estimate_shaper`` and Singer et al. 1990 for the
        derivation. Output is a unitless ratio: 0 means perfect cancellation,
        1 means no cancellation, can exceed 1 near anti-resonances of the
        shaper.
        """
        if damping_ratio is None:
            damping_ratio = self.damping_ratio
        omega = 2.0 * math.pi * freq
        damped = omega * math.sqrt(max(0.0, 1.0 - damping_ratio ** 2))
        s = 0.0
        c = 0.0
        for imp in self.impulses:
            decay = math.exp(-damping_ratio * omega * (self.delay - imp.time))
            phase = damped * (self.delay - imp.time)
            s += imp.amplitude * decay * math.sin(phase)
            c += imp.amplitude * decay * math.cos(phase)
        return math.sqrt(s * s + c * c)

    def residual_vibration_curve(self, freqs: np.ndarray,
                                 damping_ratio: float | None = None
                                 ) -> np.ndarray:
        return np.array([self.residual_vibration(float(f), damping_ratio)
                         for f in freqs])

    def to_dict(self) -> dict:
        return {
            'name': self.name,
            'frequency': self.frequency,
            'damping_ratio': self.damping_ratio,
            'delay': self.delay,
            'smoothing': self.smoothing,
            'impulses': [
                {'amplitude': i.amplitude, 'time': i.time}
                for i in self.impulses
            ],
        }


# ---------------------------------------------------------------------------
# Shaper factories. Formulas follow Klipper's shaper_defs.py so calibration
# results are directly comparable across firmwares.
# ---------------------------------------------------------------------------


def _damped_period(freq: float, damping_ratio: float) -> float:
    df = math.sqrt(max(1e-9, 1.0 - damping_ratio ** 2))
    return 1.0 / (freq * df)


def zv(frequency: float, damping_ratio: float = DEFAULT_DAMPING_RATIO
       ) -> InputShaper:
    """Zero-Vibration (ZV) shaper. 2 impulses, minimal lag."""
    df = math.sqrt(max(1e-9, 1.0 - damping_ratio ** 2))
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = _damped_period(frequency, damping_ratio)
    return InputShaper(
        'ZV',
        [Impulse(1.0, 0.0), Impulse(K, 0.5 * t_d)],
        frequency, damping_ratio,
    )


def zvd(frequency: float, damping_ratio: float = DEFAULT_DAMPING_RATIO
        ) -> InputShaper:
    """ZVD shaper. 3 impulses, more robust than ZV at the cost of more lag."""
    df = math.sqrt(max(1e-9, 1.0 - damping_ratio ** 2))
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = _damped_period(frequency, damping_ratio)
    return InputShaper(
        'ZVD',
        [Impulse(1.0, 0.0),
         Impulse(2.0 * K, 0.5 * t_d),
         Impulse(K * K, t_d)],
        frequency, damping_ratio,
    )


def mzv(frequency: float, damping_ratio: float = DEFAULT_DAMPING_RATIO
        ) -> InputShaper:
    """Modified ZV (MZV). Less lag than ZVD, broader frequency response."""
    df = math.sqrt(max(1e-9, 1.0 - damping_ratio ** 2))
    K = math.exp(-0.75 * damping_ratio * math.pi / df)
    t_d = _damped_period(frequency, damping_ratio)
    a1 = 1.0 - 1.0 / math.sqrt(2.0)
    a2 = (math.sqrt(2.0) - 1.0) * K
    a3 = a1 * K * K
    return InputShaper(
        'MZV',
        [Impulse(a1, 0.0),
         Impulse(a2, 0.375 * t_d),
         Impulse(a3, 0.75 * t_d)],
        frequency, damping_ratio,
    )


def ei(frequency: float, damping_ratio: float = DEFAULT_DAMPING_RATIO,
       vibr_tolerance: float = DEFAULT_VIBR_TOLERANCE) -> InputShaper:
    """Extra-Insensitive (EI) shaper. Robust to ~5% frequency mismatch."""
    v_tol = vibr_tolerance
    df = math.sqrt(max(1e-9, 1.0 - damping_ratio ** 2))
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = _damped_period(frequency, damping_ratio)
    a1 = 0.25 * (1.0 + v_tol)
    a2 = 0.5 * (1.0 - v_tol) * K
    a3 = a1 * K * K
    return InputShaper(
        'EI',
        [Impulse(a1, 0.0),
         Impulse(a2, 0.5 * t_d),
         Impulse(a3, t_d)],
        frequency, damping_ratio,
    )


def ei_2hump(frequency: float, damping_ratio: float = DEFAULT_DAMPING_RATIO,
             vibr_tolerance: float = DEFAULT_VIBR_TOLERANCE) -> InputShaper:
    """Two-Hump EI shaper. Even more robust, larger lag."""
    v_tol = vibr_tolerance
    df = math.sqrt(max(1e-9, 1.0 - damping_ratio ** 2))
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = _damped_period(frequency, damping_ratio)
    V2 = v_tol * v_tol
    X = (V2 * (math.sqrt(1.0 - V2) + 1.0)) ** (1.0 / 3.0)
    a1 = (3.0 * X * X + 2.0 * X + 3.0 * V2) / (16.0 * X)
    a2 = (0.5 - a1) * K
    a3 = a2 * K
    a4 = a1 * K * K * K
    return InputShaper(
        '2HUMP_EI',
        [Impulse(a1, 0.0),
         Impulse(a2, 0.5 * t_d),
         Impulse(a3, t_d),
         Impulse(a4, 1.5 * t_d)],
        frequency, damping_ratio,
    )


SHAPER_FACTORIES = {
    'ZV': zv,
    'ZVD': zvd,
    'MZV': mzv,
    'EI': ei,
    '2HUMP_EI': ei_2hump,
}


def make_shaper(name: str, frequency: float,
                damping_ratio: float = DEFAULT_DAMPING_RATIO) -> InputShaper:
    try:
        factory = SHAPER_FACTORIES[name.upper()]
    except KeyError as exc:
        raise ValueError(f'unknown shaper {name!r}; '
                         f'expected one of {list(SHAPER_FACTORIES)}') from exc
    return factory(frequency, damping_ratio)
