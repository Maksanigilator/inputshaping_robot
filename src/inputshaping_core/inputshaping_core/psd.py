"""PSD computation and shaper recommendation.

Mirrors Klipper's calibration outputs:
  * ``calibration_data_<axis>_<ts>.csv`` -- raw accelerometer trace
  * ``resonances_<axis>_<ts>.csv``      -- frequency/PSD table
  * ``resonances_<axis>_<ts>.png``      -- PSD with peak markers
  * ``shaper_calibrate_<axis>_<ts>.png``-- candidate shapers' residual curves

We always treat the X axis (forward direction of the robot) as primary, but
also compute the other two and the total magnitude PSD for diagnostics.
"""

from __future__ import annotations

import csv
import datetime as _dt
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from scipy import signal  # noqa: E402

from . import shapers as _shapers  # noqa: E402

# Frequency band of interest for a mobile robot with a flexible IMU mount.
# Most resonances we expect to see live between ~3 Hz (whole-mast sway) and
# ~40 Hz (short flexure modes). 60 Hz is a safe upper bound: above this we
# enter wheel/tire noise territory and the motors can't follow anyway.
DEFAULT_F_MIN = 1.0
DEFAULT_F_MAX = 60.0


@dataclass
class AccelSamples:
    """Container for a captured triaxial accelerometer trace."""
    t: np.ndarray  # seconds, monotonically increasing
    ax: np.ndarray  # m/s^2
    ay: np.ndarray
    az: np.ndarray

    @property
    def sample_rate(self) -> float:
        if len(self.t) < 2:
            return 0.0
        # Estimate the effective sample rate from the median dt so spurious
        # gaps (USB hiccups, dropped frames) do not skew the result.
        dt = np.median(np.diff(self.t))
        return float(1.0 / dt) if dt > 0 else 0.0

    def __len__(self) -> int:
        return len(self.t)


@dataclass
class PsdResult:
    freqs: np.ndarray
    psd_x: np.ndarray
    psd_y: np.ndarray
    psd_z: np.ndarray
    sample_rate: float

    @property
    def psd_total(self) -> np.ndarray:
        # Sum-of-axes PSD; total power spectral density used by Klipper for
        # peak detection. Plain sum is correct: power adds linearly.
        return self.psd_x + self.psd_y + self.psd_z


@dataclass
class ShaperCandidate:
    name: str
    frequency: float
    vibration: float  # remaining vibration fraction at the measured peak
    smoothing: float  # Klipper-style smoothing metric (higher = more lag)
    max_accel_factor: float  # 1 - smoothing penalty heuristic
    score: float  # lower = better


@dataclass
class ShaperReport:
    candidates: list[ShaperCandidate]
    recommended: ShaperCandidate
    peak_frequency: float
    peak_value: float


def compute_psd(samples: AccelSamples,
                f_min: float = DEFAULT_F_MIN,
                f_max: float = DEFAULT_F_MAX) -> PsdResult:
    """Compute Welch PSD for each axis on the band [f_min, f_max].

    nperseg is chosen so that the frequency resolution is ~0.5 Hz, which is
    plenty to resolve the typical first resonance and not so fine that we
    starve the spectrum of averaging.
    """
    fs = samples.sample_rate
    if fs <= 0:
        raise ValueError('cannot compute PSD: sample rate is zero')

    # Target ~0.5 Hz bin width, round to a power of two for FFT efficiency.
    target = int(fs / 0.5)
    nperseg = 1
    while nperseg < target:
        nperseg <<= 1
    nperseg = max(256, min(nperseg, len(samples)))

    def _w(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        f, p = signal.welch(x - np.mean(x), fs=fs, nperseg=nperseg,
                            noverlap=nperseg // 2, detrend='linear')
        return f, p

    f, px = _w(samples.ax)
    _, py = _w(samples.ay)
    _, pz = _w(samples.az)

    mask = (f >= f_min) & (f <= f_max)
    return PsdResult(
        freqs=f[mask],
        psd_x=px[mask],
        psd_y=py[mask],
        psd_z=pz[mask],
        sample_rate=fs,
    )


def find_peak(psd: PsdResult, axis: str = 'x') -> tuple[float, float]:
    """Return ``(freq, value)`` of the dominant peak on ``axis``."""
    series = {
        'x': psd.psd_x, 'y': psd.psd_y, 'z': psd.psd_z,
        'total': psd.psd_total,
    }[axis]
    idx = int(np.argmax(series))
    return float(psd.freqs[idx]), float(series[idx])


def evaluate_shapers(psd: PsdResult,
                     axis: str = 'x',
                     damping_ratio: float = _shapers.DEFAULT_DAMPING_RATIO,
                     shaper_names: list[str] | None = None,
                     ) -> ShaperReport:
    """Evaluate a pool of shapers against the measured PSD.

    For each candidate shaper, sweep its design frequency from f_min..f_max
    in 0.2 Hz steps; the score is a weighted sum of (a) residual vibration
    integrated against the measured PSD and (b) Klipper-style smoothing.
    """
    if shaper_names is None:
        shaper_names = list(_shapers.SHAPER_FACTORIES.keys())

    f_axis = {
        'x': psd.psd_x, 'y': psd.psd_y, 'z': psd.psd_z,
        'total': psd.psd_total,
    }[axis]
    freqs = psd.freqs
    if np.sum(f_axis) <= 0:
        raise ValueError('PSD on the chosen axis is empty')

    # Normalize PSD so weights sum to 1.
    weight = f_axis / np.sum(f_axis)
    peak_f, peak_v = find_peak(psd, axis)

    candidates: list[ShaperCandidate] = []
    for name in shaper_names:
        best: ShaperCandidate | None = None
        for f_design in np.arange(max(2.0, freqs[0]),
                                  min(freqs[-1], 80.0), 0.2):
            try:
                shaper = _shapers.make_shaper(name, float(f_design),
                                              damping_ratio)
            except ValueError:
                continue
            v_curve = shaper.residual_vibration_curve(freqs, damping_ratio)
            # Weighted residual vibration against the measured PSD.
            residual = float(np.sum(weight * v_curve))
            smoothing = shaper.smoothing
            # Score: residual is the dominant term; smoothing only acts as a
            # tie-breaker so we don't pick excessively laggy shapers when two
            # candidates have similar suppression.
            score = residual + 0.05 * smoothing
            cand = ShaperCandidate(
                name=name,
                frequency=float(f_design),
                vibration=residual,
                smoothing=smoothing,
                max_accel_factor=1.0 / (1.0 + smoothing),
                score=score,
            )
            if best is None or cand.score < best.score:
                best = cand
        if best is not None:
            candidates.append(best)

    if not candidates:
        raise RuntimeError('no shaper candidates could be evaluated')

    recommended = min(candidates, key=lambda c: c.score)
    return ShaperReport(candidates=candidates, recommended=recommended,
                        peak_frequency=peak_f, peak_value=peak_v)


# ---------------------------------------------------------------------------
# File outputs.
# ---------------------------------------------------------------------------


def _ts() -> str:
    return _dt.datetime.now().strftime('%Y%m%d_%H%M%S')


def save_raw_csv(samples: AccelSamples, out_dir: Path,
                 axis: str = 'x', timestamp: str | None = None) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'calibration_data_{axis}_{ts}.csv'
    with path.open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'ax', 'ay', 'az'])
        for row in zip(samples.t, samples.ax, samples.ay, samples.az):
            w.writerow([f'{row[0]:.6f}', f'{row[1]:.6f}',
                        f'{row[2]:.6f}', f'{row[3]:.6f}'])
    return path


def save_psd_csv(psd: PsdResult, out_dir: Path,
                 axis: str = 'x', timestamp: str | None = None) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'resonances_{axis}_{ts}.csv'
    with path.open('w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['freq_hz', 'psd_x', 'psd_y', 'psd_z', 'psd_total'])
        for row in zip(psd.freqs, psd.psd_x, psd.psd_y, psd.psd_z,
                       psd.psd_total):
            w.writerow([f'{v:.6g}' for v in row])
    return path


def plot_psd(psd: PsdResult, out_dir: Path, axis: str = 'x',
             timestamp: str | None = None,
             peak: tuple[float, float] | None = None,
             title_suffix: str = '') -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'resonances_{axis}_{ts}.png'

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(psd.freqs, psd.psd_x, label='X', color='tab:red')
    ax.plot(psd.freqs, psd.psd_y, label='Y', color='tab:green', alpha=0.7)
    ax.plot(psd.freqs, psd.psd_z, label='Z', color='tab:blue', alpha=0.7)
    ax.plot(psd.freqs, psd.psd_total, label='Total', color='black',
            linewidth=1.5)
    if peak is not None:
        pf, pv = peak
        ax.axvline(pf, color='gray', linestyle='--', alpha=0.7)
        ax.annotate(f'peak {pf:.1f} Hz', xy=(pf, pv), xytext=(pf + 1, pv),
                    color='gray')
    ax.set_xlabel('Frequency, Hz')
    ax.set_ylabel('PSD, (m/s$^2$)$^2$/Hz')
    title = f'Resonances on axis {axis.upper()} (fs = {psd.sample_rate:.0f} Hz)'
    if title_suffix:
        title += f' -- {title_suffix}'
    ax.set_title(title)
    ax.set_yscale('log')
    ax.set_xlim(psd.freqs[0], psd.freqs[-1])
    ax.grid(True, which='both', alpha=0.3)
    ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def plot_shaper_candidates(psd: PsdResult, report: ShaperReport,
                           out_dir: Path, axis: str = 'x',
                           timestamp: str | None = None,
                           damping_ratio: float = _shapers.DEFAULT_DAMPING_RATIO
                           ) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'shaper_calibrate_{axis}_{ts}.png'

    f_series = {
        'x': psd.psd_x, 'y': psd.psd_y, 'z': psd.psd_z,
        'total': psd.psd_total,
    }[axis]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    psd_max = max(np.max(f_series), 1e-12)
    ax1.plot(psd.freqs, f_series / psd_max, color='black',
             label='measured PSD (norm)', linewidth=1.2)
    ax1.set_ylabel('PSD, normalized')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')

    for cand in report.candidates:
        shaper = _shapers.make_shaper(cand.name, cand.frequency, damping_ratio)
        v_curve = shaper.residual_vibration_curve(psd.freqs, damping_ratio)
        label = (f'{cand.name} @ {cand.frequency:.1f} Hz '
                 f'(V={cand.vibration:.2f}, S={cand.smoothing:.4f})')
        ax2.plot(psd.freqs, v_curve, label=label)

    ax2.axvline(report.peak_frequency, color='gray', linestyle='--',
                alpha=0.7, label=f'peak {report.peak_frequency:.1f} Hz')
    ax2.set_xlabel('Frequency, Hz')
    ax2.set_ylabel('Residual vibration')
    ax2.set_ylim(0, 1.05)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=8)
    fig.suptitle(
        f'Shaper calibration on axis {axis.upper()} -- '
        f'recommended: {report.recommended.name} @ '
        f'{report.recommended.frequency:.1f} Hz'
    )
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# Klipper-style plots
# ---------------------------------------------------------------------------
#
# Reproductions of the two plots that ``klipper/scripts/calibrate_shaper.py``
# produces, kept side-by-side with our own plots so the user can compare.
# Conventions copied verbatim from upstream Klipper:
#
#   * linear (not log) PSD axis
#   * X = red, Y = green, Z = blue, X+Y+Z = purple
#   * shaper residual curves on a twin axis on the right ("ratio")
#   * dash-dot line + cyan "after shaper" curve for the recommended shaper
#
# Reference: https://github.com/Klipper3d/klipper/blob/master/scripts/
#            calibrate_shaper.py


_KLIPPER_AXIS_COLORS = {
    'total': 'purple',
    'x':     'red',
    'y':     'green',
    'z':     'blue',
}


def plot_psd_klipper(psd: PsdResult, out_dir: Path, axis: str = 'x',
                     timestamp: str | None = None,
                     peak: tuple[float, float] | None = None,
                     title_suffix: str = '') -> Path:
    """Klipper ``calibrate_shaper.py`` style PSD plot.

    A single linear-scale figure with one curve per axis plus the X+Y+Z
    total. Frequency band is the same as ``compute_psd``.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'resonances_klipper_{axis}_{ts}.png'

    fig, ax_ = plt.subplots(figsize=(10, 5))
    ax_.set_xlabel('Frequency, Hz')
    ax_.set_xlim(psd.freqs[0], psd.freqs[-1])
    ax_.set_ylabel('Power spectral density')

    ax_.plot(psd.freqs, psd.psd_total, label='X+Y+Z',
             color=_KLIPPER_AXIS_COLORS['total'], linewidth=1.2)
    ax_.plot(psd.freqs, psd.psd_x, label='X',
             color=_KLIPPER_AXIS_COLORS['x'], alpha=0.85)
    ax_.plot(psd.freqs, psd.psd_y, label='Y',
             color=_KLIPPER_AXIS_COLORS['y'], alpha=0.85)
    ax_.plot(psd.freqs, psd.psd_z, label='Z',
             color=_KLIPPER_AXIS_COLORS['z'], alpha=0.85)

    if peak is not None:
        pf, pv = peak
        ax_.axvline(pf, color='gray', linestyle='--', alpha=0.7)
        ax_.annotate(f'peak {pf:.1f} Hz', xy=(pf, pv), xytext=(pf + 1.0, pv),
                     color='gray', fontsize=9)

    ax_.grid(which='major', color='grey', alpha=0.35)
    ax_.grid(which='minor', color='lightgrey', alpha=0.25)
    ax_.minorticks_on()
    ax_.legend(loc='upper right', fontsize=9)

    title = f'Frequency response (axis {axis.upper()}, fs={psd.sample_rate:.0f} Hz)'
    if title_suffix:
        title += f'  --  {title_suffix}'
    ax_.set_title(title)

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def plot_shaper_klipper(psd: PsdResult, report: ShaperReport,
                        out_dir: Path, axis: str = 'x',
                        timestamp: str | None = None,
                        damping_ratio: float = _shapers.DEFAULT_DAMPING_RATIO
                        ) -> Path:
    """Klipper ``calibrate_shaper.py`` style combined shaper plot.

    Left Y: linear PSD with per-axis curves and a cyan "after-shaper" curve
    for the recommended shaper. Right Y: each shaper's residual-vibration
    curve as a function of frequency.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'shaper_calibrate_klipper_{axis}_{ts}.png'

    f_series = {
        'x': psd.psd_x, 'y': psd.psd_y, 'z': psd.psd_z,
        'total': psd.psd_total,
    }[axis]

    fig, ax_psd = plt.subplots(figsize=(12, 6.5))
    ax_psd.set_xlabel('Frequency, Hz')
    ax_psd.set_xlim(psd.freqs[0], psd.freqs[-1])
    ax_psd.set_ylabel('Power spectral density')

    ax_psd.plot(psd.freqs, psd.psd_total, label='X+Y+Z',
                color=_KLIPPER_AXIS_COLORS['total'], linewidth=1.2)
    ax_psd.plot(psd.freqs, psd.psd_x, label='X',
                color=_KLIPPER_AXIS_COLORS['x'], alpha=0.85)
    ax_psd.plot(psd.freqs, psd.psd_y, label='Y',
                color=_KLIPPER_AXIS_COLORS['y'], alpha=0.85)
    ax_psd.plot(psd.freqs, psd.psd_z, label='Z',
                color=_KLIPPER_AXIS_COLORS['z'], alpha=0.85)

    ax_shaper = ax_psd.twinx()
    ax_shaper.set_ylabel('Shaper vibration reduction (ratio)')
    ax_shaper.set_ylim(0.0, 1.05)

    best_curve = None
    for cand in report.candidates:
        shaper = _shapers.make_shaper(cand.name, cand.frequency, damping_ratio)
        v_curve = shaper.residual_vibration_curve(psd.freqs, damping_ratio)
        is_best = cand is report.recommended
        if is_best:
            best_curve = v_curve
        label = (f'{cand.name.upper()} ({cand.frequency:.1f} Hz, '
                 f'vibr={cand.vibration * 100:.1f}%, '
                 f's={cand.smoothing:.2f})')
        ax_shaper.plot(psd.freqs, v_curve, label=label,
                       linestyle='dashdot' if is_best else 'dotted',
                       linewidth=1.5 if is_best else 1.0)

    if best_curve is not None:
        ax_psd.plot(psd.freqs, f_series * best_curve,
                    label='After\nshaper', color='cyan', linewidth=1.4)

    ax_psd.axvline(report.peak_frequency, color='gray', linestyle='--',
                   alpha=0.6)
    ax_psd.grid(which='major', color='grey', alpha=0.35)
    ax_psd.grid(which='minor', color='lightgrey', alpha=0.25)
    ax_psd.minorticks_on()

    # Combine legends from both axes into a single box, placed outside the
    # plotting area on the right so it doesn't cover any curves.
    h1, l1 = ax_psd.get_legend_handles_labels()
    h2, l2 = ax_shaper.get_legend_handles_labels()
    fig.legend(h1 + h2, l1 + l2, loc='center left',
               fontsize=8, bbox_to_anchor=(0.78, 0.55),
               framealpha=0.85)

    fig.suptitle(
        f'Shaper calibration (axis {axis.upper()}) -- '
        f'recommended: {report.recommended.name.upper()} @ '
        f'{report.recommended.frequency:.1f} Hz '
        f'(peak {report.peak_frequency:.1f} Hz)',
        fontsize=11,
    )
    # Leave 22% of the canvas on the right for the legend, and a strip on
    # top for the suptitle.
    fig.tight_layout(rect=(0, 0, 0.78, 0.94))
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def save_all(samples: AccelSamples, psd: PsdResult, report: ShaperReport,
             out_dir: Path, axis: str = 'x',
             damping_ratio: float = _shapers.DEFAULT_DAMPING_RATIO
             ) -> dict[str, Path]:
    """Convenience: write CSVs + PNGs in one Klipper-style bundle."""
    ts = _ts()
    return {
        'raw_csv': save_raw_csv(samples, out_dir, axis, ts),
        'psd_csv': save_psd_csv(psd, out_dir, axis, ts),
        'psd_png': plot_psd(psd, out_dir, axis, ts,
                            peak=(report.peak_frequency, report.peak_value),
                            title_suffix=f'recommended '
                                         f'{report.recommended.name} @ '
                                         f'{report.recommended.frequency:.1f} Hz'),
        'shaper_png': plot_shaper_candidates(psd, report, out_dir, axis, ts,
                                             damping_ratio),
        # Klipper-style equivalents -- saved next to the originals so they
        # both show up in the "Recent results" card.
        'psd_klipper_png': plot_psd_klipper(
            psd, out_dir, axis, ts,
            peak=(report.peak_frequency, report.peak_value),
            title_suffix=f'recommended {report.recommended.name.upper()} @ '
                         f'{report.recommended.frequency:.1f} Hz'),
        'shaper_klipper_png': plot_shaper_klipper(
            psd, report, out_dir, axis, ts, damping_ratio),
    }
