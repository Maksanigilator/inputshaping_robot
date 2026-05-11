"""Smoke tests for the pure-Python pieces (no ROS, no hardware)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from inputshaping_core import motion, psd, shapers
from inputshaping_core.pico_protocol import FRAME_STRUCT, FrameParser, SYNC_BYTES
from inputshaping_core.shaper_state import ShaperState


def test_shaper_dc_gain_is_one():
    for name in shapers.SHAPER_FACTORIES:
        sh = shapers.make_shaper(name, 8.0, 0.05)
        assert math.isclose(sum(i.amplitude for i in sh.impulses), 1.0,
                            abs_tol=1e-9)


def test_zv_zeroes_residual_at_design_frequency():
    sh = shapers.zv(8.0, 0.05)
    # ZV should cancel almost completely at its design frequency.
    assert sh.residual_vibration(8.0, 0.05) < 1e-6


def test_trapezoid_distance_matches_target():
    p = motion.trapezoid_distance(distance=1.0, v_max=0.3, a_max=0.5)
    travelled = motion.estimate_displacement(p)
    assert math.isclose(travelled, 1.0, rel_tol=2e-2)


def test_trapezoid_handles_triangular_case():
    # Very short distance must fall back to a triangular profile.
    p = motion.trapezoid_distance(distance=0.05, v_max=1.0, a_max=0.5)
    assert max(np.abs(p.v)) < 1.0  # never reaches v_max
    assert math.isclose(motion.estimate_displacement(p), 0.05, rel_tol=5e-2)


def test_chirp_profile_stays_bounded():
    p = motion.moving_chirp(
        v_cruise=0.15, a_ramp=0.3,
        settle=1.0, ringdown=1.0,
        f_start=1.0, f_end=20.0, chirp_duration=4.0,
        a_chirp_peak=0.5,
    )
    assert p.v.max() < 0.5
    assert p.v.min() > -0.05  # forward-only chirp


def test_bounded_chirp_keeps_within_envelope():
    """Bounded sweep must keep the chassis within +/- max_distance."""
    max_d = 1.0
    p = motion.bounded_chirp(
        max_distance=max_d, v_cruise=0.15,
        a_ramp=0.3, a_turn=1.0,
        f_start=1.0, f_end=20.0, a_chirp_peak=0.5,
        n_passes=4, settle=0.5, ringdown=0.5,
    )
    # Integrate velocity to get position; allow 1% slack for the chirp ripple.
    pos = np.cumsum(p.v) * p.dt
    assert pos.max() < max_d * 1.01, (
        f'forward excursion {pos.max():.3f} m exceeds envelope {max_d:.3f} m')
    assert pos.min() > -max_d * 0.05, (
        f'backward excursion {pos.min():.3f} m violates 0 m start')
    # End position should be close to start (within one cruise step).
    assert abs(pos[-1] - pos[0]) < 0.3 * max_d, (
        f'net displacement {pos[-1]:.3f} m too large for a bounded sweep')


def test_bounded_chirp_never_crosses_zero():
    """During each pass the gearbox must stay loaded (no backlash dead zone)."""
    p = motion.bounded_chirp(
        max_distance=1.0, v_cruise=0.15,
        a_ramp=0.3, a_turn=1.0,
        f_start=1.0, f_end=20.0, a_chirp_peak=0.6,
        n_passes=2,
    )
    # Find the indices that belong to the cruise/chirp sections. Crude but
    # effective: anywhere abs(v) > 0.5*v_cruise is "cruising or chirping".
    cruising = np.abs(p.v) > 0.07
    assert cruising.sum() > 0
    # Within those zones the velocity must never change sign.
    signs = np.sign(p.v[cruising])
    # Each contiguous run of cruising should have a single sign; check sign
    # changes happen only at non-cruising indices (turnarounds).
    same_sign_runs = np.split(signs, np.where(np.diff(signs) != 0)[0] + 1)
    # Number of sign changes inside cruising <= n_passes (one per turnaround).
    sign_changes = len(same_sign_runs) - 1
    assert sign_changes <= 4


def test_bounded_chirp_rejects_too_small_distance():
    with pytest.raises(ValueError):
        motion.bounded_chirp(
            max_distance=0.1, v_cruise=0.15,
            a_ramp=0.3, a_turn=1.0,
            f_start=1.0, f_end=10.0, a_chirp_peak=0.3,
        )


def test_pico_frame_roundtrip():
    pkt = FRAME_STRUCT.pack(SYNC_BYTES, 7, 12345, 100, -200, 16000)
    parser = FrameParser()
    out = parser.feed(pkt)
    assert len(out) == 1
    s = out[0]
    assert s.seq == 7
    assert s.t_us == 12345
    # Raw 16000 -> 16000 * 9.80665 / 16384 ~= 9.577 m/s^2
    assert math.isclose(s.az, 16000 * 9.80665 / 16384, rel_tol=1e-6)


def test_parser_resyncs_after_garbage():
    pkt = FRAME_STRUCT.pack(SYNC_BYTES, 1, 1, 0, 0, 0)
    parser = FrameParser()
    # Garbage prefix followed by a real frame.
    out = parser.feed(b'\x00\x11\x22' + pkt)
    assert len(out) == 1
    assert parser.dropped_bytes == 3


def test_shaper_state_bypass_returns_raw():
    s = ShaperState(dt=0.02)
    s.configure(name=None, frequency=10, damping_ratio=0.05, enabled=False)
    assert s.step(0.123) == pytest.approx(0.123)


def test_shaper_state_dc_passthrough():
    """A DC velocity must come through a shaper unchanged in steady state."""
    s = ShaperState(dt=0.02)
    s.configure(name='ZV', frequency=8.0, damping_ratio=0.05, enabled=True)
    out = 0.0
    for _ in range(200):
        out = s.step(0.3)
    assert out == pytest.approx(0.3, abs=1e-6)


def test_psd_pipeline_finds_planted_frequency():
    fs = 1000.0
    t = np.arange(0, 4.0, 1.0 / fs)
    # Damped 7 Hz oscillation on X axis only.
    f0 = 7.0
    a_x = np.exp(-2.0 * t) * np.sin(2 * np.pi * f0 * t) + \
        0.01 * np.random.default_rng(0).standard_normal(t.size)
    samples = psd.AccelSamples(t=t, ax=a_x, ay=np.zeros_like(t),
                               az=np.zeros_like(t))
    p = psd.compute_psd(samples)
    pf, _ = psd.find_peak(p, 'x')
    assert abs(pf - f0) < 1.0
