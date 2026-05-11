"""Velocity-profile generators.

All generators produce a uniformly-sampled velocity series ``(t, v)`` where
``v(t)`` is the linear velocity to publish to the chassis. Sign of ``v``
encodes direction (forward = positive).

The PSD-collection profiles intentionally avoid crossing zero velocity: the
robot ramps up smoothly, cruises through a perturbation, and ramps down --
this keeps the gearbox under load and side-steps backlash artefacts that
contaminate a stationary chirp.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


# Period of the periodic publisher driving cmd_vel. 50 Hz matches typical
# ROS 2 teleop rates and is well above any resonance we expect to excite.
DEFAULT_DT = 0.02


@dataclass
class VelocityProfile:
    t: np.ndarray            # seconds, starts at 0
    v: np.ndarray            # m/s
    label: str               # short human-readable label
    metadata: dict           # generator parameters, for logging

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0

    @property
    def dt(self) -> float:
        return float(self.t[1] - self.t[0]) if len(self.t) > 1 else 0.0


# ---------------------------------------------------------------------------
# Helpers.
# ---------------------------------------------------------------------------


def _times(duration: float, dt: float) -> np.ndarray:
    """Inclusive time grid with step ``dt`` covering [0, duration]."""
    n = max(1, int(round(duration / dt)) + 1)
    return np.linspace(0.0, (n - 1) * dt, n)


def _trapezoid_segment(v_start: float, v_end: float, a: float,
                       dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Linear ramp from v_start to v_end at acceleration magnitude ``a``."""
    if a <= 0:
        raise ValueError('acceleration must be positive')
    duration = abs(v_end - v_start) / a
    if duration <= 0:
        return np.array([0.0]), np.array([v_start])
    t = _times(duration, dt)
    v = np.linspace(v_start, v_end, len(t))
    return t, v


# ---------------------------------------------------------------------------
# Public profile generators.
# ---------------------------------------------------------------------------


def trapezoid_distance(distance: float, v_max: float, a_max: float,
                       dt: float = DEFAULT_DT) -> VelocityProfile:
    """Trapezoidal (or triangular) velocity profile covering ``distance``.

    Distance is signed: negative -> robot drives backwards. The profile is
    closed (starts and ends at zero), so it is the right thing to send for a
    "move forward 1 m" button.

    For the diploma it is worth noting: this profile guarantees position
    accuracy at constant a_max regardless of v_max selection, modulo
    integration error of the velocity command on the robot side. We do not
    correct for drivetrain dead-zones here; the robot's own controller does
    that.
    """
    if distance == 0.0:
        return VelocityProfile(np.array([0.0]), np.array([0.0]),
                               'trapezoid (zero)',
                               {'distance': 0.0, 'v_max': v_max,
                                'a_max': a_max})

    direction = 1.0 if distance > 0 else -1.0
    abs_dist = abs(distance)
    # Distance covered during acceleration if we reach v_max.
    d_acc = v_max * v_max / (2.0 * a_max)
    if 2.0 * d_acc >= abs_dist:
        # Triangular: we never reach v_max.
        v_peak = math.sqrt(a_max * abs_dist)
        t_acc = v_peak / a_max
        t1, v1 = _trapezoid_segment(0.0, v_peak, a_max, dt)
        t2, v2 = _trapezoid_segment(v_peak, 0.0, a_max, dt)
        t = np.concatenate([t1, t2[1:] + t1[-1]])
        v = direction * np.concatenate([v1, v2[1:]])
    else:
        t_acc = v_max / a_max
        t_cruise = (abs_dist - 2.0 * d_acc) / v_max
        t1, v1 = _trapezoid_segment(0.0, v_max, a_max, dt)
        t_c = _times(t_cruise, dt)
        v_c = np.full_like(t_c, v_max)
        t3, v3 = _trapezoid_segment(v_max, 0.0, a_max, dt)
        # Stitch, ensuring no duplicated boundary samples.
        t = np.concatenate([t1, t_c[1:] + t1[-1],
                            t3[1:] + t1[-1] + t_c[-1] - t_c[0]])
        v = direction * np.concatenate([v1, v_c[1:], v3[1:]])

    return VelocityProfile(
        t=t, v=v, label='trapezoid',
        metadata={'distance': float(distance),
                  'v_max': float(v_max), 'a_max': float(a_max)},
    )


def moving_chirp(v_cruise: float, a_ramp: float,
                 settle: float, ringdown: float,
                 f_start: float, f_end: float, chirp_duration: float,
                 a_chirp_peak: float, dt: float = DEFAULT_DT,
                 direction: float = 1.0) -> VelocityProfile:
    """Smooth ramp -> cruise -> linear chirp -> cruise -> ramp down.

    The chirp amplitude at any instant is chosen so that the peak commanded
    acceleration equals ``a_chirp_peak`` regardless of the current frequency,
    which gives a flat excitation envelope over the swept band.

    Direction is +/-1 (forward/backward); the entire profile is mirrored.

    Returns the *commanded* velocity sequence. Total displacement is roughly
    ``direction * v_cruise * (settle + chirp_duration + ringdown)`` plus the
    two ramps; choose v_cruise and durations so the path fits the room.
    """
    if v_cruise <= 0 or a_ramp <= 0 or chirp_duration <= 0:
        raise ValueError('v_cruise, a_ramp, chirp_duration must be positive')
    if f_end <= f_start:
        raise ValueError('f_end must be greater than f_start')

    direction = 1.0 if direction >= 0 else -1.0

    # Ramp up.
    t_up, v_up = _trapezoid_segment(0.0, v_cruise, a_ramp, dt)
    # Settle (cruise before chirp).
    t_s = _times(settle, dt)
    v_s = np.full_like(t_s, v_cruise)
    # Chirp burst.
    t_c = _times(chirp_duration, dt)
    sweep = (f_end - f_start) / chirp_duration
    phase = 2.0 * math.pi * (f_start * t_c + 0.5 * sweep * t_c * t_c)
    # Peak amplitude such that peak |dv/dt| = a_chirp_peak at current freq.
    inst_f = f_start + sweep * t_c
    amp = a_chirp_peak / (2.0 * math.pi * np.maximum(inst_f, 1e-3))
    v_chirp = v_cruise + amp * np.sin(phase)
    # Ringdown cruise.
    t_r = _times(ringdown, dt)
    v_r = np.full_like(t_r, v_cruise)
    # Ramp down.
    t_dn, v_dn = _trapezoid_segment(v_cruise, 0.0, a_ramp, dt)

    # Stitch (skip duplicate boundary samples between adjacent segments).
    pieces_v = [v_up, v_s[1:], v_chirp[1:], v_r[1:], v_dn[1:]]
    pieces_t = [t_up,
                t_s[1:] + t_up[-1],
                t_c[1:] + t_up[-1] + t_s[-1],
                t_r[1:] + t_up[-1] + t_s[-1] + t_c[-1],
                t_dn[1:] + t_up[-1] + t_s[-1] + t_c[-1] + t_r[-1]]
    t = np.concatenate(pieces_t)
    v = direction * np.concatenate(pieces_v)

    return VelocityProfile(
        t=t, v=v, label='moving_chirp',
        metadata={
            'v_cruise': float(v_cruise), 'a_ramp': float(a_ramp),
            'settle': float(settle), 'ringdown': float(ringdown),
            'f_start': float(f_start), 'f_end': float(f_end),
            'chirp_duration': float(chirp_duration),
            'a_chirp_peak': float(a_chirp_peak), 'direction': direction,
        },
    )


def bounded_chirp(max_distance: float, v_cruise: float, a_ramp: float,
                  a_turn: float,
                  f_start: float, f_end: float, a_chirp_peak: float,
                  n_passes: int = 2,
                  settle: float = 0.5, ringdown: float = 0.5,
                  dt: float = DEFAULT_DT,
                  direction: float = 1.0) -> VelocityProfile:
    """Bounded back-and-forth chirp sweep (Klipper-style resonance test).

    The robot does ``n_passes`` one-way passes alternating direction, with a
    chirp overlaid on each pass while the chassis cruises at +/-v_cruise.
    Between passes a smooth turnaround at acceleration ``a_turn`` reverses
    direction; turnarounds add a tiny overshoot (``v_cruise**2/(2*a_turn)``,
    typically <1 cm at our limits) but no net displacement.

    The robot stays within +/- ``max_distance`` of its starting position. Per
    pass duration is auto-computed from ``max_distance / v_cruise`` minus a
    safety margin for ramps. The chirp amplitude is clamped so the commanded
    velocity never crosses zero, which is essential to keep the gearbox out
    of its backlash dead zone.

    Notes for the diploma:

    * ``n_passes >= 2`` is recommended even when the room is large. Two
      passes already give two near-independent PSDs (forward + backward);
      Welch averages them automatically across the time series.
    * The chirp during the backward pass is the same sine perturbation
      (``+amp*sin(phase)``) added on top of ``-v_cruise``. Sign of the
      excitation is irrelevant for the PSD: only ``dv/dt`` matters.
    """
    direction = 1.0 if direction >= 0 else -1.0
    if max_distance <= 0 or v_cruise <= 0:
        raise ValueError('max_distance and v_cruise must be positive')
    if n_passes < 1:
        raise ValueError('n_passes must be >= 1')
    if a_turn <= 0 or a_ramp <= 0:
        raise ValueError('a_turn and a_ramp must be positive')
    if f_end <= f_start:
        raise ValueError('f_end must be greater than f_start')

    # The forward excursion is reached at the *end* of the first chirp pass.
    # Budget breakdown for the +direction half of the envelope:
    #   ramp-up distance        : v_cruise^2 / (2*a_ramp)
    #   settle cruise           : v_cruise * settle
    #   first chirp pass        : v_cruise * T_pass         (must fit)
    #   turnaround overshoot    : v_cruise^2 / (2*a_turn)
    #   safety margin           : 5 cm
    ramp_dist = v_cruise * v_cruise / (2.0 * a_ramp)
    turn_overshoot = v_cruise * v_cruise / (2.0 * a_turn)
    safety = 0.05
    avail = max_distance - ramp_dist - v_cruise * settle - turn_overshoot \
        - safety
    if avail < v_cruise * 1.0:
        raise ValueError(
            f'max_distance={max_distance:.2f} m too small for '
            f'v_cruise={v_cruise:.2f}: available cruise budget is '
            f'{avail:.3f} m, need >= {v_cruise:.2f} m (>=1 s of chirp)'
        )
    t_pass = avail / v_cruise

    segments: list[np.ndarray] = []

    def add(v_seg: np.ndarray, skip_first: bool = True) -> None:
        if skip_first and segments and len(v_seg) > 0:
            segments.append(v_seg[1:])
        else:
            segments.append(v_seg)

    # 1) Ramp 0 -> +direction*v_cruise.
    _, v_up = _trapezoid_segment(0.0, direction * v_cruise, a_ramp, dt)
    add(v_up, skip_first=False)

    # 2) Settle (cruise, no chirp).
    if settle > 0:
        t_s = _times(settle, dt)
        add(np.full_like(t_s, direction * v_cruise))

    current_dir = direction
    for pass_idx in range(n_passes):
        # 3a) Cruise + chirp.
        t_c = _times(t_pass, dt)
        sweep = (f_end - f_start) / t_pass
        phase = 2.0 * math.pi * (f_start * t_c + 0.5 * sweep * t_c * t_c)
        inst_f = f_start + sweep * t_c
        amp = a_chirp_peak / (2.0 * math.pi * np.maximum(inst_f, 1e-3))
        # Guarantee we never invert direction: cap at half of v_cruise so the
        # gearbox stays loaded throughout.
        amp = np.minimum(amp, 0.5 * v_cruise)
        v_chirp = current_dir * v_cruise + amp * np.sin(phase)
        add(v_chirp)

        # 3b) Turnaround to -current_dir*v_cruise (skipped after last pass).
        if pass_idx < n_passes - 1:
            _, v_turn = _trapezoid_segment(
                current_dir * v_cruise, -current_dir * v_cruise, a_turn, dt)
            add(v_turn)
            current_dir = -current_dir

    # 4) Ringdown (cruise without chirp).
    if ringdown > 0:
        t_r = _times(ringdown, dt)
        add(np.full_like(t_r, current_dir * v_cruise))

    # 5) Ramp down current -> 0.
    _, v_dn = _trapezoid_segment(current_dir * v_cruise, 0.0, a_ramp, dt)
    add(v_dn)

    v = np.concatenate(segments)
    t = np.arange(len(v)) * dt

    return VelocityProfile(
        t=t, v=v, label='bounded_chirp',
        metadata={
            'max_distance': float(max_distance),
            'v_cruise': float(v_cruise),
            'a_ramp': float(a_ramp), 'a_turn': float(a_turn),
            'n_passes': int(n_passes),
            'f_start': float(f_start), 'f_end': float(f_end),
            'a_chirp_peak': float(a_chirp_peak),
            't_pass': float(t_pass),
            'settle': float(settle), 'ringdown': float(ringdown),
            'direction': float(direction),
        },
    )


def moving_impulse(v_cruise: float, a_ramp: float,
                   settle: float, ringdown: float,
                   pulse_width: float, pulse_dv: float,
                   dt: float = DEFAULT_DT,
                   direction: float = 1.0) -> VelocityProfile:
    """Ramp up -> cruise -> short positive pulse -> cruise -> ramp down.

    The pulse is a rectangle on the velocity command: v_cruise -> v_cruise +
    pulse_dv for ``pulse_width`` seconds, then back to v_cruise. The rising
    and falling edges are the actual broadband impulses; ``ringdown`` is when
    the IMU records the structure's free decay.
    """
    if pulse_width <= 0:
        raise ValueError('pulse_width must be positive')

    direction = 1.0 if direction >= 0 else -1.0

    t_up, v_up = _trapezoid_segment(0.0, v_cruise, a_ramp, dt)
    t_s = _times(settle, dt)
    v_s = np.full_like(t_s, v_cruise)
    t_p = _times(pulse_width, dt)
    v_p = np.full_like(t_p, v_cruise + pulse_dv)
    t_r = _times(ringdown, dt)
    v_r = np.full_like(t_r, v_cruise)
    t_dn, v_dn = _trapezoid_segment(v_cruise, 0.0, a_ramp, dt)

    pieces_v = [v_up, v_s[1:], v_p[1:], v_r[1:], v_dn[1:]]
    pieces_t = [t_up,
                t_s[1:] + t_up[-1],
                t_p[1:] + t_up[-1] + t_s[-1],
                t_r[1:] + t_up[-1] + t_s[-1] + t_p[-1],
                t_dn[1:] + t_up[-1] + t_s[-1] + t_p[-1] + t_r[-1]]
    t = np.concatenate(pieces_t)
    v = direction * np.concatenate(pieces_v)

    return VelocityProfile(
        t=t, v=v, label='moving_impulse',
        metadata={
            'v_cruise': float(v_cruise), 'a_ramp': float(a_ramp),
            'settle': float(settle), 'ringdown': float(ringdown),
            'pulse_width': float(pulse_width), 'pulse_dv': float(pulse_dv),
            'direction': direction,
        },
    )


def moving_step(v_cruise: float, a_ramp: float,
                settle: float, step_dv: float, ringdown: float,
                dt: float = DEFAULT_DT,
                direction: float = 1.0) -> VelocityProfile:
    """Ramp up -> cruise at v_cruise -> step to v_cruise+step_dv -> ramp down.

    Step transitions are not rate-limited here; the chassis controller will
    saturate at its physical a_max. This is the right thing for measuring
    the step response, since we want the fastest possible velocity change.
    """
    direction = 1.0 if direction >= 0 else -1.0
    if v_cruise + step_dv <= 0:
        raise ValueError('v_cruise + step_dv must be positive')

    t_up, v_up = _trapezoid_segment(0.0, v_cruise, a_ramp, dt)
    t_s = _times(settle, dt)
    v_s = np.full_like(t_s, v_cruise)
    t_r = _times(ringdown, dt)
    v_r = np.full_like(t_r, v_cruise + step_dv)
    t_dn, v_dn = _trapezoid_segment(v_cruise + step_dv, 0.0, a_ramp, dt)

    pieces_v = [v_up, v_s[1:], v_r[1:], v_dn[1:]]
    pieces_t = [t_up,
                t_s[1:] + t_up[-1],
                t_r[1:] + t_up[-1] + t_s[-1],
                t_dn[1:] + t_up[-1] + t_s[-1] + t_r[-1]]
    t = np.concatenate(pieces_t)
    v = direction * np.concatenate(pieces_v)

    return VelocityProfile(
        t=t, v=v, label='moving_step',
        metadata={
            'v_cruise': float(v_cruise), 'a_ramp': float(a_ramp),
            'settle': float(settle), 'step_dv': float(step_dv),
            'ringdown': float(ringdown), 'direction': direction,
        },
    )


def estimate_displacement(profile: VelocityProfile) -> float:
    """Net displacement of the robot if it tracks the profile exactly (m)."""
    if len(profile.t) < 2:
        return 0.0
    return float(np.trapezoid(profile.v, profile.t))
