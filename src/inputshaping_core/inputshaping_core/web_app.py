"""Flask application that exposes the experiment controls.

Designed to be served over SSH local-port-forward::

    ssh -L 8080:localhost:8080 user@lab-laptop
    # then open http://localhost:8080 in a local browser

The page polls ``/api/status`` once a second to refresh the runtime state;
this is much friendlier over a flaky tunnel than a server-sent-events
stream.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from flask import (
    Flask, jsonify, render_template, request, send_from_directory,
)

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - allows pure-pytest runs
    get_package_share_directory = None  # type: ignore[assignment]


class _Controller:
    """Minimal interface that the web app expects from the ROS node."""

    def status(self) -> dict[str, Any]: ...
    def list_results(self) -> list[dict[str, Any]]: ...
    def get_recent_imu(self, duration: float,
                       max_points: int) -> dict[str, Any]: ...
    def configure_shaper(self, name: str | None, frequency: float,
                         damping_ratio: float, enabled: bool) -> dict: ...
    def start_motion(self, distance: float, v_max: float, a_max: float
                     ) -> dict: ...
    def start_psd(self, method: str, params: dict[str, Any]) -> dict: ...
    def cancel(self) -> dict: ...


def _resolve_template_dirs() -> tuple[Path, Path]:
    """Locate the bundled templates/static dirs both in source and installed.

    Order of preference:
      1. The package's installed share directory (set by colcon).
      2. The source tree adjacent to this file (works with --symlink-install
         and when running tests against an uninstalled checkout).
    """
    if get_package_share_directory is not None:
        try:
            share = Path(get_package_share_directory('inputshaping_core'))
            t, s = share / 'templates', share / 'static'
            if t.is_dir() and s.is_dir():
                return t, s
        except Exception:
            pass
    here = Path(__file__).resolve().parents[1]
    return here / 'templates', here / 'static'


def create_app(controller: _Controller, data_dir: Path) -> Flask:
    """Construct the Flask app bound to ``controller``."""
    template_dir, static_dir = _resolve_template_dirs()
    app = Flask(
        'inputshaping',
        template_folder=str(template_dir),
        static_folder=str(static_dir),
    )
    # Quiet down Flask's per-request "200 -" lines; we have our own logging.
    logging.getLogger('werkzeug').setLevel(logging.WARNING)

    @app.get('/')
    def index() -> Any:
        return render_template('index.html')

    @app.get('/api/status')
    def api_status() -> Any:
        return jsonify(controller.status())

    @app.get('/api/results')
    def api_results() -> Any:
        return jsonify({'results': controller.list_results()})

    @app.get('/api/imu/recent')
    def api_imu_recent() -> Any:
        try:
            duration = float(request.args.get('duration', '2.0'))
            max_points = int(request.args.get('max_points', '240'))
        except (TypeError, ValueError) as exc:
            return jsonify({'ok': False, 'error': str(exc)}), 400
        duration = max(0.1, min(duration, 10.0))
        max_points = max(20, min(max_points, 1200))
        return jsonify(controller.get_recent_imu(
            duration=duration, max_points=max_points))

    @app.post('/api/shaper')
    def api_shaper() -> Any:
        d = request.get_json(force=True) or {}
        try:
            res = controller.configure_shaper(
                name=d.get('name'),
                frequency=float(d.get('frequency', 8.0)),
                damping_ratio=float(d.get('damping_ratio', 0.05)),
                enabled=bool(d.get('enabled', False)),
            )
        except (ValueError, RuntimeError) as exc:
            return jsonify({'ok': False, 'error': str(exc)}), 400
        return jsonify({'ok': True, 'shaper': res})

    @app.post('/api/motion')
    def api_motion() -> Any:
        d = request.get_json(force=True) or {}
        try:
            res = controller.start_motion(
                distance=float(d.get('distance', 1.0)),
                v_max=float(d.get('v_max', 0.3)),
                a_max=float(d.get('a_max', 0.5)),
            )
        except (ValueError, RuntimeError) as exc:
            return jsonify({'ok': False, 'error': str(exc)}), 400
        return jsonify({'ok': True, 'experiment': res})

    @app.post('/api/psd')
    def api_psd() -> Any:
        d = request.get_json(force=True) or {}
        method = str(d.get('method', 'moving_chirp'))
        params = d.get('params', {})
        if not isinstance(params, dict):
            return jsonify({'ok': False, 'error': 'params must be object'}), 400
        try:
            res = controller.start_psd(method=method, params=params)
        except (ValueError, RuntimeError) as exc:
            return jsonify({'ok': False, 'error': str(exc)}), 400
        return jsonify({'ok': True, 'experiment': res})

    @app.post('/api/cancel')
    def api_cancel() -> Any:
        return jsonify({'ok': True, 'cancelled': controller.cancel()})

    # Serve the generated CSV/PNG artefacts so the user can download them
    # without an extra HTTP server.
    @app.get('/data/<path:fname>')
    def data_file(fname: str) -> Any:
        return send_from_directory(str(data_dir), fname, as_attachment=False)

    return app
