"""Motion-recording pipeline: command vs. measurement plots.

Captures a single trapezoidal motion (with or without an input shaper
applied to the command) together with the synchronized IMU stream and
produces:

  * CSV (``motion_record_<shaper-tag>_<ts>.csv``) with one row per IMU
    sample: ``t, cmd_v, cmd_a, meas_ax, meas_ay, meas_az,
    meas_a_scalar, meas_v_integrated``;
  * PNG (``motion_record_<shaper-tag>_<ts>.png``) with two stacked
    subplots:
        top    -- commanded acceleration vs. measured ``Σ_xyz − g_bias``;
        bottom -- commanded velocity vs. trapezoidally-integrated
                  measurement.

The ``meas_a_scalar = ax + ay + az − g_bias`` form was chosen by the
user; it survives accelerometer tilt better than picking a single axis
when the strip can pitch/roll, but its absolute magnitude is only
correct up to a geometric factor that depends on the strip's attitude.
The integrated velocity therefore tracks the *shape* of the cmd
profile (ramps, plateaus, ringdown) very well, but its numeric peak is
NOT a calibrated speed -- use it for shaper-vs-no-shaper comparison,
not as a substitute for odometry.

No drift correction is applied (the user opted out) -- on captures of
a few seconds the bias is small enough that the integrated velocity
qualitatively matches the command. Add a linear ``detrend`` here if
you ever extend the record window past ~5 s.
"""

from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


GRAVITY_M_S2 = 9.80665
# Length of the "robot is still stationary" window we use to estimate
# the bias of ``ax + ay + az``. Half a second is plenty for a 1 kHz IMU
# and short enough that the operator can hit Start without long delays.
DEFAULT_SETTLE_S = 0.5


@dataclass
class MotionRecording:
    """Synchronized command + IMU traces for a single motion run."""
    cmd_t: np.ndarray         # publisher tick timestamps, seconds (t=0 at run start)
    cmd_v: np.ndarray         # commanded (post-shaper) linear velocity, m/s
    imu_t: np.ndarray         # IMU sample timestamps, same time origin
    imu_ax: np.ndarray        # m/s^2
    imu_ay: np.ndarray
    imu_az: np.ndarray
    distance: float = 0.0
    v_max: float = 0.0
    a_max: float = 0.0
    post_roll: float = 0.0
    shaper_label: str | None = None
    settle_seconds: float = DEFAULT_SETTLE_S

    @property
    def duration(self) -> float:
        if len(self.imu_t) == 0:
            return 0.0
        return float(self.imu_t[-1] - self.imu_t[0])

    def cmd_acceleration(self) -> np.ndarray:
        """Numerical derivative of the (shaped) command velocity."""
        if len(self.cmd_v) < 2:
            return np.zeros_like(self.cmd_v)
        return np.gradient(self.cmd_v, self.cmd_t)

    def measured_acceleration_scalar(self) -> tuple[np.ndarray, float]:
        """Return ``(a_scalar, g_bias)``.

        ``g_bias`` is the mean of ``ax + ay + az`` over the first
        ``settle_seconds`` of the IMU window (robot still stationary,
        command pipeline already running so cmd_v == 0 by construction).
        This single offset captures both the 1 g gravity vector and any
        small DC sensor bias.

        ``a_scalar = (ax + ay + az) − g_bias`` is the quantity the user
        asked for. Its magnitude is correct only up to a tilt factor;
        the shape is what we care about for the cmd-vs-measured plot.
        """
        a_sum = self.imu_ax + self.imu_ay + self.imu_az
        if len(self.imu_t) == 0:
            return a_sum, GRAVITY_M_S2
        t0 = float(self.imu_t[0])
        mask = self.imu_t <= (t0 + self.settle_seconds)
        # Require at least 10 samples to compute a meaningful mean; for
        # an MPU6050 at 1 kHz that's 10 ms of settle, easily met.
        if int(mask.sum()) >= 10:
            g_bias = float(np.mean(a_sum[mask]))
        else:
            g_bias = GRAVITY_M_S2
        return a_sum - g_bias, g_bias

    def measured_velocity(self) -> np.ndarray:
        """Trapezoidal integral of ``a_scalar`` on the IMU time grid."""
        a_scalar, _ = self.measured_acceleration_scalar()
        if len(self.imu_t) < 2:
            return np.zeros_like(a_scalar)
        dt = np.diff(self.imu_t)
        # Trapezoidal rule: v[i] = v[i-1] + 0.5 * (a[i] + a[i-1]) * dt[i-1].
        increments = 0.5 * (a_scalar[1:] + a_scalar[:-1]) * dt
        return np.concatenate([[0.0], np.cumsum(increments)])


def _ts() -> str:
    return time.strftime('%Y%m%d_%H%M%S')


def save_motion_csv(rec: MotionRecording, out_dir: Path,
                    timestamp: str | None = None,
                    shaper_tag: str = 'noshaper') -> Path:
    """One-file CSV on the IMU time grid (cmd interpolated onto it).

    Two separate header sections (cmd and IMU at different rates) would
    be inconvenient for downstream tools, so we resample the publisher
    series (50 Hz) onto IMU timestamps (~1 kHz) with linear interp. The
    cmd derivative is computed *before* the resample so plateau-flatness
    survives interpolation.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'motion_record_{shaper_tag}_{ts}.csv'

    a_scalar, _ = rec.measured_acceleration_scalar()
    v_meas = rec.measured_velocity()
    cmd_a = rec.cmd_acceleration()
    # Interpolation requires monotonically increasing xp; cmd_t is by
    # construction.
    if len(rec.cmd_t) >= 2:
        cmd_v_interp = np.interp(rec.imu_t, rec.cmd_t, rec.cmd_v)
        cmd_a_interp = np.interp(rec.imu_t, rec.cmd_t, cmd_a)
    else:
        cmd_v_interp = np.zeros_like(rec.imu_t)
        cmd_a_interp = np.zeros_like(rec.imu_t)

    with path.open('w', newline='') as fp:
        w = csv.writer(fp)
        w.writerow(['t', 'cmd_v', 'cmd_a',
                    'meas_ax', 'meas_ay', 'meas_az',
                    'meas_a_scalar', 'meas_v_integrated'])
        for i in range(len(rec.imu_t)):
            w.writerow([
                f'{rec.imu_t[i]:.6f}',
                f'{cmd_v_interp[i]:.6f}',
                f'{cmd_a_interp[i]:.6f}',
                f'{rec.imu_ax[i]:.6f}',
                f'{rec.imu_ay[i]:.6f}',
                f'{rec.imu_az[i]:.6f}',
                f'{a_scalar[i]:.6f}',
                f'{v_meas[i]:.6f}',
            ])
    return path


def plot_motion(rec: MotionRecording, out_dir: Path,
                timestamp: str | None = None,
                shaper_tag: str = 'noshaper') -> Path:
    """Two-row figure: cmd-vs-measured acceleration, then velocity."""
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _ts()
    path = out_dir / f'motion_record_{shaper_tag}_{ts}.png'

    a_scalar, g_bias = rec.measured_acceleration_scalar()
    v_meas = rec.measured_velocity()
    cmd_a = rec.cmd_acceleration()

    fig, (ax_a, ax_v) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    ax_a.plot(rec.imu_t, a_scalar, color='tab:red', alpha=0.55,
              linewidth=0.7,
              label=f'measured a  (Σ_xyz − {g_bias:.2f})')
    ax_a.plot(rec.cmd_t, cmd_a, color='tab:blue', linewidth=1.4,
              label='commanded a (post-shaper)')
    ax_a.axhline(0.0, color='black', linewidth=0.5, alpha=0.5)
    ax_a.set_ylabel('Acceleration, m/s$^2$')
    ax_a.grid(True, alpha=0.3)
    ax_a.legend(loc='upper right', fontsize=9)
    title = (f'Motion record  --  distance {rec.distance:.2f} m, '
             f'v_max={rec.v_max:.2f} m/s, a_max={rec.a_max:.2f} m/s$^2$, '
             f'shaper={rec.shaper_label or "off"}, '
             f'post-roll={rec.post_roll:.1f} s')
    ax_a.set_title(title, fontsize=11)

    ax_v.plot(rec.imu_t, v_meas, color='tab:red', alpha=0.9,
              linewidth=1.1,
              label='measured v (∫ a · dt, uncorrected)')
    ax_v.plot(rec.cmd_t, rec.cmd_v, color='tab:blue', linewidth=1.4,
              label='commanded v (post-shaper)')
    ax_v.axhline(0.0, color='black', linewidth=0.5, alpha=0.5)
    ax_v.set_ylabel('Velocity, m/s')
    ax_v.set_xlabel('Time, s')
    ax_v.grid(True, alpha=0.3)
    ax_v.legend(loc='upper right', fontsize=9)

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def save_all(rec: MotionRecording, out_dir: Path,
             shaper_tag: str = 'noshaper') -> dict[str, Path]:
    """Bundle: CSV + PNG, share a single timestamp."""
    ts = _ts()
    return {
        'csv': save_motion_csv(rec, out_dir, timestamp=ts,
                               shaper_tag=shaper_tag),
        'png': plot_motion(rec, out_dir, timestamp=ts,
                           shaper_tag=shaper_tag),
    }
