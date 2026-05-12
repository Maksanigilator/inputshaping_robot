#!/usr/bin/env python3
"""Regenerate PSD/shaper plots from an existing calibration CSV.

Takes ``data/calibration_data_<axis>_<ts>.csv`` (raw t, ax, ay, az
samples written by ``experiment_node`` at the end of every PSD run),
re-runs the full Welch -> shaper recommendation -> plot pipeline using
the *current* code in ``inputshaping_core``, and writes the four PNG
plot variants alongside (linear PSD, log PSD, shaper candidates,
Klipper-style shaper). The original ``<ts>`` is preserved in the
output filenames so the new PNGs overwrite the old ones, which is
exactly what you want when you've tweaked a plotting routine and need
the old captures redrawn with the new style.

The script imports ``inputshaping_core`` directly from ``src/``, so a
freshly mounted source tree works without ``colcon build``. Run it
inside the project's Docker image, or any environment where numpy,
scipy and matplotlib are installed.

Usage::

    python3 regen_plots.py data/calibration_data_x_20260511_170551.csv

Multiple CSVs can be passed in one go (one PNG-bundle per CSV).
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

from inputshaping_core import psd as _psd


_CALIB_NAME = re.compile(
    r'calibration_data_(?P<axis>[xyz])_(?P<ts>\d{8}_\d{6})\.csv$')


def _load_samples(path: Path) -> _psd.AccelSamples:
    t: list[float] = []
    ax: list[float] = []
    ay: list[float] = []
    az: list[float] = []
    with path.open() as fp:
        for row in csv.DictReader(fp):
            t.append(float(row['t']))
            ax.append(float(row['ax']))
            ay.append(float(row['ay']))
            az.append(float(row['az']))
    return _psd.AccelSamples(
        t=np.asarray(t, dtype=float),
        ax=np.asarray(ax, dtype=float),
        ay=np.asarray(ay, dtype=float),
        az=np.asarray(az, dtype=float),
    )


def regen_one(csv_path: Path, axis_override: str | None = None) -> int:
    m = _CALIB_NAME.search(csv_path.name)
    if m is None and axis_override is None:
        print(f'cannot infer axis/timestamp from {csv_path.name}; '
              f'pass --axis explicitly', file=sys.stderr)
        return 2
    axis = (axis_override or m.group('axis')).lower()
    ts = m.group('ts') if m else 'regen'
    out_dir = csv_path.parent

    print(f'[regen] {csv_path}')
    samples = _load_samples(csv_path)
    print(f'  loaded {len(samples)} samples, '
          f'fs ~= {samples.sample_rate:.0f} Hz')

    psd = _psd.compute_psd(samples)
    print(f'  PSD: {len(psd.freqs)} bins, '
          f'[{psd.freqs[0]:.1f}..{psd.freqs[-1]:.1f}] Hz')

    report = _psd.evaluate_shapers(psd)
    print(f'  peak @ {report.peak_frequency:.1f} Hz, '
          f'recommended: {report.recommended.name.upper()} @ '
          f'{report.recommended.frequency:.1f} Hz '
          f'(V={report.recommended.vibration:.3f})')

    targets = [
        out_dir / f'resonances_{axis}_{ts}.png',
        out_dir / f'shaper_calibrate_{axis}_{ts}.png',
        out_dir / f'resonances_klipper_{axis}_{ts}.png',
        out_dir / f'shaper_calibrate_klipper_{axis}_{ts}.png',
    ]
    for p in targets:
        if p.exists():
            p.unlink()
            print(f'  - removed {p.name}')

    suffix = (f'recommended {report.recommended.name} @ '
              f'{report.recommended.frequency:.1f} Hz')
    written = [
        _psd.plot_psd(psd, out_dir, axis=axis, timestamp=ts,
                      peak=(report.peak_frequency, report.peak_value),
                      title_suffix=suffix),
        _psd.plot_shaper_candidates(psd, report, out_dir,
                                    axis=axis, timestamp=ts),
        _psd.plot_psd_klipper(psd, out_dir, axis=axis, timestamp=ts,
                              peak=(report.peak_frequency,
                                    report.peak_value),
                              title_suffix=suffix),
        _psd.plot_shaper_klipper(psd, report, out_dir,
                                 axis=axis, timestamp=ts),
    ]
    for p in written:
        print(f'  + wrote {p.name}')
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+', type=Path,
                    help='calibration_data_<axis>_<ts>.csv files')
    ap.add_argument('--axis', default=None,
                    help='override axis letter (defaults to filename match)')
    args = ap.parse_args()

    rc = 0
    for path in args.csv:
        if not path.is_file():
            print(f'no such file: {path}', file=sys.stderr)
            rc = 2
            continue
        rc = regen_one(path, args.axis) or rc
    return rc


if __name__ == '__main__':
    sys.exit(main())
