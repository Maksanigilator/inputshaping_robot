"""Single ROS 2 node that owns the bench logic.

Responsibilities:
  * Subscribe to ``/imu/data_raw`` (sensor_msgs/Imu) and push samples into a
    ring buffer.
  * Subscribe to ``/cmd_vel_raw`` (geometry_msgs/Twist) so external clients
    (e.g. teleop) can still get shaped output via ``/cmd_vel_shaped``.
  * Publish ``/cmd_vel_shaped`` at 50 Hz with the shaped velocity.
  * Run a Flask GUI in a background thread.
  * Spawn a background worker thread that processes completed PSD captures
    (Welch, plots, recommendation) without stalling the publisher.
"""

from __future__ import annotations

import os
import queue
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

from . import motion as _motion
from . import motion_record as _motion_record
from . import psd as _psd
from .experiment_runner import ExperimentResult, ExperimentRunner
from .imu_buffer import ImuBuffer, ImuSample
from .pico_protocol import SAMPLE_RATE_HZ
from .shaper_state import ShaperState
from .web_app import create_app

PUBLISH_RATE_HZ = 50.0
PUBLISH_PERIOD_S = 1.0 / PUBLISH_RATE_HZ

# If we have not heard anything (no /cmd_vel_raw and no experiment) in this
# long, publish a zero Twist on /cmd_vel_shaped so the robot doesn't latch
# the last value forever.
RAW_WATCHDOG_S = 0.5


class ExperimentNode(Node):
    def __init__(self) -> None:
        super().__init__('inputshaping_experiment_node')

        data_default = os.environ.get('INPUTSHAPING_DATA_DIR',
                                      '/workspace/data')
        gui_port_default = int(os.environ.get('INPUTSHAPING_GUI_PORT', 8080))

        self.declare_parameter('cmd_vel_raw_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_shaped_topic', '/cmd_vel_shaped')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('gui_host', '0.0.0.0')
        self.declare_parameter('gui_port', gui_port_default)
        self.declare_parameter('data_dir', data_default)
        self.declare_parameter('v_max_default', 0.6)
        self.declare_parameter('a_max_default', 2.0)
        # Mecanum-on-carpet workaround: roller friction is poor and a soft
        # ramp simply spins the wheels in place, so we let the user request
        # well above the platform's nominal ceiling -- the motors will
        # saturate on their own. Override via ROS params if you want
        # tighter safety on a different surface.
        self.declare_parameter('v_max_limit', 2.0)
        self.declare_parameter('a_max_limit', 5.0)

        raw_topic = self.get_parameter('cmd_vel_raw_topic').value
        shaped_topic = self.get_parameter('cmd_vel_shaped_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self._data_dir = Path(self.get_parameter('data_dir').value)
        self._data_dir.mkdir(parents=True, exist_ok=True)
        self._v_max_limit = float(self.get_parameter('v_max_limit').value)
        self._a_max_limit = float(self.get_parameter('a_max_limit').value)

        # Internal state.
        self._shaper = ShaperState(dt=PUBLISH_PERIOD_S, max_delay_s=2.0)
        self._runner = ExperimentRunner()
        self._imu = ImuBuffer(max_seconds=30.0,
                              sample_rate_hz=SAMPLE_RATE_HZ)
        self._state_lock = threading.RLock()
        self._raw_twist: Twist | None = None
        self._raw_twist_time: float = 0.0
        self._results: list[dict] = []
        # IMU stats.
        self._imu_count = 0
        self._imu_t0: float | None = None
        self._imu_rate_hz: float = 0.0
        self._imu_dropped_bytes = 0
        self._imu_dropped_frames = 0
        # Single time anchor for the IMU ring buffer and capture windows.
        # Using ``time.monotonic`` everywhere keeps comparisons consistent;
        # ``msg.header.stamp`` from /imu/data_raw is on ROS time which has a
        # different epoch from time.monotonic(), so we re-stamp on arrival.
        self._wall_t0 = time.monotonic()

        # ROS pub/sub.
        self._pub = self.create_publisher(Twist, shaped_topic, 10)
        self.create_subscription(
            Twist, raw_topic, self._on_raw_twist, 10)
        self.create_subscription(
            Imu, imu_topic, self._on_imu,
            QoSPresetProfiles.SENSOR_DATA.value)

        # 50 Hz publisher.
        self.create_timer(PUBLISH_PERIOD_S, self._publish_tick)
        # 1 Hz IMU-rate calc.
        self.create_timer(1.0, self._update_imu_rate)

        # Background worker for PSD processing.
        self._psd_queue: queue.Queue = queue.Queue()
        self._psd_worker = threading.Thread(
            target=self._psd_worker_loop, name='psd-worker', daemon=True)
        self._psd_worker.start()

        # Background worker for motion-record post-processing (CSV + plot).
        # Separate queue from PSD so a slow plot render never blocks the
        # next experiment.
        self._motion_queue: queue.Queue = queue.Queue()
        self._motion_worker = threading.Thread(
            target=self._motion_worker_loop, name='motion-record-worker',
            daemon=True)
        self._motion_worker.start()

        # Motion-record state. ``None`` means no recording is active; when
        # set, ``_publish_tick`` appends (t_rel, raw_v, shaped_v) to
        # ``_record_log`` until ``t_end`` is reached, then ships everything
        # to the worker thread for analysis.
        self._record_state: dict | None = None
        self._record_log: list[tuple[float, float, float]] = []

        # Flask GUI.
        app = create_app(_FlaskBridge(self), self._data_dir)
        host = self.get_parameter('gui_host').value
        port = int(self.get_parameter('gui_port').value)
        self._gui_thread = threading.Thread(
            target=lambda: app.run(host=host, port=port, threaded=True,
                                   use_reloader=False, debug=False),
            name='flask-gui', daemon=True,
        )
        self._gui_thread.start()
        self.get_logger().info(
            f'GUI at http://{host}:{port}/  '
            f'(tunnel: ssh -L {port}:localhost:{port} <host>)'
        )
        self.get_logger().info(
            f'topics: raw={raw_topic} shaped={shaped_topic} imu={imu_topic} '
            f'data_dir={self._data_dir}'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_raw_twist(self, msg: Twist) -> None:
        with self._state_lock:
            self._raw_twist = msg
            self._raw_twist_time = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        # Re-stamp on arrival in time.monotonic-relative-to-_wall_t0; the
        # original ROS header is on a different clock domain.
        t = time.monotonic() - self._wall_t0
        self._imu.push(ImuSample(
            t=t,
            ax=msg.linear_acceleration.x,
            ay=msg.linear_acceleration.y,
            az=msg.linear_acceleration.z,
        ))
        self._imu_count += 1
        if self._imu_t0 is None:
            self._imu_t0 = time.monotonic()

    def _update_imu_rate(self) -> None:
        if self._imu_t0 is None:
            return
        now = time.monotonic()
        elapsed = now - self._imu_t0
        if elapsed > 0:
            self._imu_rate_hz = self._imu_count / elapsed
        # Reset every 5 seconds so the rate reflects current state.
        if elapsed > 5.0:
            self._imu_t0 = now
            self._imu_count = 0

    def _publish_tick(self) -> None:
        now = time.monotonic()
        # Pick the raw velocity command.
        if not self._runner.is_idle:
            raw_v = self._runner.tick()
        else:
            with self._state_lock:
                if (self._raw_twist is not None
                        and (now - self._raw_twist_time) < RAW_WATCHDOG_S):
                    raw_v = float(self._raw_twist.linear.x)
                else:
                    raw_v = 0.0
        shaped_v = self._shaper.step(raw_v)
        out = Twist()
        out.linear.x = float(shaped_v)
        self._pub.publish(out)

        # Capture command samples for an in-flight motion recording. We
        # log the *shaped* velocity because that is what the chassis
        # actually receives, and the user's cmd-vs-measured plot must
        # match the actual command (otherwise the shaper's effect would
        # vanish from the "commanded" trace).
        if self._record_state is not None:
            elapsed = now - self._record_state['t_start']
            self._record_log.append((elapsed, raw_v, shaped_v))
            if elapsed >= self._record_state['t_end']:
                state = self._record_state
                log = self._record_log
                self._record_state = None
                self._record_log = []
                self._motion_queue.put((state, log))

    # ------------------------------------------------------------------
    # Controller interface (called from Flask threads)
    # ------------------------------------------------------------------

    def status(self) -> dict:
        return {
            'experiment': self._runner.snapshot(),
            'shaper': self._shaper.snapshot(),
            'imu': {
                'sample_rate': self._imu_rate_hz,
                'count': self._imu_count,
                'dropped_bytes': self._imu_dropped_bytes,
                'dropped_frames': self._imu_dropped_frames,
            },
            'limits': {
                'v_max': self._v_max_limit,
                'a_max': self._a_max_limit,
            },
            'results': self._snapshot_results(),
        }

    def _snapshot_results(self) -> list[dict]:
        with self._state_lock:
            return list(self._results[-20:])

    def list_results(self) -> list[dict]:
        return self._snapshot_results()

    def get_recent_imu(self, duration: float = 2.0,
                       max_points: int = 240) -> dict:
        """Return the last ``duration`` seconds of IMU samples for plotting.

        Downsamples by integer stride so the payload over an SSH tunnel
        stays well under 10 kB even at the default 1 kHz raw rate.
        """
        t_now = time.monotonic() - self._wall_t0
        t_start = t_now - max(0.1, duration)
        ts, ax, ay, az = self._imu.snapshot(since_t=t_start)
        if len(ts) == 0:
            return {'t': [], 'ax': [], 'ay': [], 'az': [], 'now': t_now}
        # Decimate to at most ``max_points``.
        step = max(1, len(ts) // max_points)
        ts = ts[::step]
        ax = ax[::step]
        ay = ay[::step]
        az = az[::step]
        return {
            't': (ts - t_now).tolist(),   # relative to now: newest = 0
            'ax': ax.tolist(),
            'ay': ay.tolist(),
            'az': az.tolist(),
            'now': float(t_now),
        }

    def configure_shaper(self, name: str | None, frequency: float,
                         damping_ratio: float, enabled: bool) -> dict:
        if name == '':
            name = None
        self._shaper.configure(name=name, frequency=frequency,
                               damping_ratio=damping_ratio, enabled=enabled)
        return self._shaper.snapshot()

    def start_motion(self, distance: float, v_max: float,
                     a_max: float) -> dict:
        # Remember what the caller asked for so we can flag silent clamping
        # in the response -- otherwise a "v_max = 50 m/s" typo just gets
        # quietly chopped to v_max_limit and the user wonders why the robot
        # didn't fly.
        v_req, a_req = float(v_max), float(a_max)
        v_max = float(np.clip(v_req, 0.02, self._v_max_limit))
        a_max = float(np.clip(a_req, 0.02, self._a_max_limit))
        warnings: list[str] = []
        if abs(v_max - v_req) > 1e-6:
            warnings.append(
                f'v_max clamped {v_req:.3f} -> {v_max:.3f} m/s '
                f'(limit {self._v_max_limit:.2f}).')
        if abs(a_max - a_req) > 1e-6:
            warnings.append(
                f'a_max clamped {a_req:.3f} -> {a_max:.3f} m/s^2 '
                f'(limit {self._a_max_limit:.2f}).')
        if warnings:
            for w in warnings:
                self.get_logger().warn(w)
        profile = _motion.trapezoid_distance(distance=distance,
                                             v_max=v_max, a_max=a_max,
                                             dt=PUBLISH_PERIOD_S)
        self._runner.start(profile)
        return {
            'label': profile.label,
            'duration': profile.duration,
            'metadata': profile.metadata,
            'requested': {'v_max': v_req, 'a_max': a_req},
            'applied': {'v_max': v_max, 'a_max': a_max},
            'limits': {'v_max': self._v_max_limit,
                       'a_max': self._a_max_limit},
            'warnings': warnings,
        }

    def start_motion_record(self, distance: float, v_max: float,
                            a_max: float, post_roll: float = 1.5) -> dict:
        """Trapezoidal motion + synchronized IMU capture + plot/CSV.

        Equivalent to :meth:`start_motion` but also records cmd_v and
        IMU samples for the full duration of:
            ramp + cruise + ramp + shaper.delay + post_roll
        and ships the bundle to a background worker that writes a
        ``motion_record_<tag>_<ts>.{csv,png}`` pair into the data dir.
        The currently configured shaper (whatever is enabled in the
        GUI) is applied verbatim; the "tag" in the filename encodes
        whether shaping was on so the user can keep both runs without
        clobbering.
        """
        if self._record_state is not None:
            raise RuntimeError('a motion recording is already in progress')
        if not self._runner.is_idle:
            raise RuntimeError('runner is busy; cancel the current run first')

        v_req, a_req = float(v_max), float(a_max)
        v_max = float(np.clip(v_req, 0.02, self._v_max_limit))
        a_max = float(np.clip(a_req, 0.02, self._a_max_limit))
        post_roll = float(np.clip(post_roll, 0.0, 10.0))
        warnings: list[str] = []
        if abs(v_max - v_req) > 1e-6:
            warnings.append(
                f'v_max clamped {v_req:.3f} -> {v_max:.3f} m/s '
                f'(limit {self._v_max_limit:.2f}).')
        if abs(a_max - a_req) > 1e-6:
            warnings.append(
                f'a_max clamped {a_req:.3f} -> {a_max:.3f} m/s^2 '
                f'(limit {self._a_max_limit:.2f}).')
        for w in warnings:
            self.get_logger().warn(w)

        profile = _motion.trapezoid_distance(distance=distance,
                                             v_max=v_max, a_max=a_max,
                                             dt=PUBLISH_PERIOD_S)

        shaper_snap = self._shaper.snapshot()
        shaper_delay = float(shaper_snap.get('delay') or 0.0)
        shaper_label = self._format_shaper_label(shaper_snap)
        shaper_tag = self._format_shaper_tag(shaper_snap)
        # Total recording window: the profile itself (already includes
        # symmetric ramps), the shaper's group delay (so the tail
        # impulses of e.g. 2HUMP_EI land inside the capture), and the
        # user-requested post-roll so the strip's ringdown is visible
        # in the plot.
        record_total = float(profile.duration + shaper_delay + post_roll)

        capture_t0 = time.monotonic() - self._wall_t0
        self._imu.start(capture_t0)

        self._record_log = []
        self._record_state = {
            't_start': time.monotonic(),
            't_end': record_total,
            'capture_t0': capture_t0,
            'profile_duration': float(profile.duration),
            'shaper_delay': shaper_delay,
            'post_roll': post_roll,
            'distance': float(distance),
            'v_max': v_max,
            'a_max': a_max,
            'shaper_label': shaper_label,
            'shaper_tag': shaper_tag,
        }

        self._runner.start(profile)

        return {
            'label': profile.label,
            'duration': profile.duration,
            'record_duration': record_total,
            'shaper_label': shaper_label,
            'shaper_tag': shaper_tag,
            'metadata': profile.metadata,
            'requested': {'v_max': v_req, 'a_max': a_req,
                          'post_roll': post_roll},
            'applied': {'v_max': v_max, 'a_max': a_max,
                        'post_roll': post_roll},
            'limits': {'v_max': self._v_max_limit,
                       'a_max': self._a_max_limit},
            'warnings': warnings,
        }

    @staticmethod
    def _format_shaper_label(snap: dict) -> str | None:
        if not snap.get('enabled') or not snap.get('name'):
            return None
        return f"{str(snap['name']).upper()} @ {float(snap['frequency']):.1f} Hz"

    @staticmethod
    def _format_shaper_tag(snap: dict) -> str:
        # File tag used as a stem in motion_record_<tag>_<ts>.{csv,png}.
        # Keep it short and shell-safe -- no dots, no spaces.
        if not snap.get('enabled') or not snap.get('name'):
            return 'noshaper'
        name = str(snap['name']).lower()
        freq = f"{float(snap['frequency']):.1f}".replace('.', 'p')
        return f'{name}_{freq}hz'

    def start_psd(self, method: str, params: dict) -> dict:
        direction = float(params.get('direction', 1))
        v_cruise = float(np.clip(params.get('v_cruise', 0.15),
                                 0.02, self._v_max_limit))
        a_ramp = float(np.clip(params.get('a_ramp', 0.3),
                               0.05, self._a_max_limit))
        settle = float(params.get('settle', 1.0))
        ringdown = float(params.get('ringdown', 2.0))

        if method in ('moving_chirp', 'bounded_chirp'):
            profile = _motion.bounded_chirp(
                max_distance=float(np.clip(
                    params.get('max_distance', 1.0), 0.2, 5.0)),
                v_cruise=v_cruise, a_ramp=a_ramp,
                a_turn=float(np.clip(params.get('a_turn', self._a_max_limit),
                                     0.1, self._a_max_limit)),
                f_start=float(params.get('f_start', 1.0)),
                f_end=float(params.get('f_end', 25.0)),
                n_passes=int(np.clip(params.get('n_passes', 2), 1, 8)),
                a_chirp_peak=float(np.clip(params.get('a_chirp_peak', 0.6),
                                           0.05, self._a_max_limit)),
                settle=settle, ringdown=ringdown,
                dt=PUBLISH_PERIOD_S, direction=direction,
            )
        elif method == 'moving_impulse':
            profile = _motion.moving_impulse(
                v_cruise=v_cruise, a_ramp=a_ramp,
                settle=settle, ringdown=ringdown,
                pulse_width=float(params.get('pulse_width', 0.05)),
                pulse_dv=float(params.get('pulse_dv', 0.05)),
                dt=PUBLISH_PERIOD_S, direction=direction,
            )
        elif method == 'moving_step':
            profile = _motion.moving_step(
                v_cruise=v_cruise, a_ramp=a_ramp,
                settle=settle,
                step_dv=float(params.get('step_dv', 0.1)),
                ringdown=ringdown,
                dt=PUBLISH_PERIOD_S, direction=direction,
            )
        else:
            raise ValueError(f'unknown PSD method: {method!r}')

        # Capture IMU starting now and finishing when the runner completes;
        # we use the host monotonic clock for the analysis window.
        capture_t0 = time.monotonic() - self._wall_t0
        self._imu.start(capture_t0)

        def on_complete(result: ExperimentResult) -> None:
            # Hand off to the worker thread *immediately*: we are running
            # inside the 50 Hz publisher's tick path, so we cannot afford to
            # sleep or do FFTs here. The worker waits for late IMU samples
            # to land before snapshotting.
            capture_t1 = time.monotonic() - self._wall_t0
            self._psd_queue.put((result, capture_t0, capture_t1, method))

        self._runner.start(profile, on_complete=on_complete)
        return {
            'label': profile.label,
            'duration': profile.duration,
            'metadata': profile.metadata,
        }

    def cancel(self) -> dict:
        self._runner.cancel()
        return {'cancelled_at': time.monotonic()}

    # ------------------------------------------------------------------
    # Background PSD worker
    # ------------------------------------------------------------------

    def _psd_worker_loop(self) -> None:
        while True:
            result, t0, t1, method = self._psd_queue.get()
            try:
                self._process_psd(result, t0, t1, method)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'PSD processing failed: {exc!r}')

    def _process_psd(self, result: ExperimentResult,
                     t0: float, t1: float, method: str) -> None:
        # Wait for any in-flight IMU samples to land in the buffer and then
        # stop the capture window before we snapshot.
        time.sleep(0.1)
        self._imu.stop()
        ts, ax, ay, az = self._imu.snapshot(since_t=t0, until_t=t1)
        if len(ts) < 256:
            self.get_logger().warn(
                f'PSD: only {len(ts)} IMU samples in window '
                f'[{t0:.2f}, {t1:.2f}] -- skipping')
            return
        samples = _psd.AccelSamples(t=ts, ax=ax, ay=ay, az=az)
        psd = _psd.compute_psd(samples)
        report = _psd.evaluate_shapers(psd, axis='x',
                                       damping_ratio=_psd._shapers
                                       .DEFAULT_DAMPING_RATIO)
        files = _psd.save_all(samples=samples, psd=psd, report=report,
                              out_dir=self._data_dir, axis='x')
        rel = {k: str(Path(v).name) for k, v in files.items()}
        entry = {
            'label': f'PSD ({method})',
            'method': method,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration': float(t1 - t0),
            'num_samples': int(len(ts)),
            'sample_rate_hz': float(psd.sample_rate),
            'peak_frequency': float(report.peak_frequency),
            'peak_value': float(report.peak_value),
            'recommended': {
                'name': report.recommended.name,
                'frequency': report.recommended.frequency,
                'vibration': report.recommended.vibration,
                'smoothing': report.recommended.smoothing,
            },
            'candidates': [
                {'name': c.name, 'frequency': c.frequency,
                 'vibration': c.vibration, 'smoothing': c.smoothing}
                for c in report.candidates
            ],
            'files': list(rel.values()),
        }
        with self._state_lock:
            self._results.append(entry)
        self.get_logger().info(
            f'PSD done: peak @ {report.peak_frequency:.1f} Hz, '
            f'recommended {report.recommended.name} @ '
            f'{report.recommended.frequency:.1f} Hz'
        )

    # ------------------------------------------------------------------
    # Background motion-record worker
    # ------------------------------------------------------------------

    def _motion_worker_loop(self) -> None:
        while True:
            state, log = self._motion_queue.get()
            try:
                self._process_motion_record(state, log)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'motion record processing failed: {exc!r}')

    def _process_motion_record(self, state: dict,
                               log: list[tuple[float, float, float]]) -> None:
        # Give the last IMU samples a tick to land in the ring buffer
        # before we snapshot. 100 ms is comfortably above the 1 kHz
        # period and matches what the PSD worker uses.
        time.sleep(0.1)
        capture_t0 = float(state['capture_t0'])
        capture_t1 = capture_t0 + float(state['t_end'])
        ts, ax, ay, az = self._imu.snapshot(since_t=capture_t0,
                                            until_t=capture_t1)
        if len(ts) < 64:
            self.get_logger().warn(
                f'motion record: only {len(ts)} IMU samples in window '
                f'[{capture_t0:.2f}, {capture_t1:.2f}] -- skipping')
            return
        if not log:
            self.get_logger().warn(
                'motion record: empty command log -- skipping')
            return

        cmd_arr = np.asarray(log, dtype=float)
        cmd_t = cmd_arr[:, 0]
        # Column 2 is the *shaped* velocity (what was actually published);
        # column 1 holds the pre-shaper command for diagnostics if we ever
        # need to add it back to the plot.
        cmd_v = cmd_arr[:, 2]
        # Re-base IMU timestamps to the same t=0 as cmd_t so the plot
        # axes line up exactly.
        imu_t_rel = ts - capture_t0

        rec = _motion_record.MotionRecording(
            cmd_t=cmd_t, cmd_v=cmd_v,
            imu_t=imu_t_rel, imu_ax=ax, imu_ay=ay, imu_az=az,
            distance=float(state['distance']),
            v_max=float(state['v_max']),
            a_max=float(state['a_max']),
            post_roll=float(state['post_roll']),
            shaper_label=state.get('shaper_label'),
        )
        shaper_tag = str(state.get('shaper_tag') or 'noshaper')
        files = _motion_record.save_all(rec, self._data_dir,
                                        shaper_tag=shaper_tag)
        rel = {k: str(Path(v).name) for k, v in files.items()}
        entry = {
            'label': (f'Motion record '
                      f'({state["shaper_label"] or "no shaper"})'),
            'kind': 'motion_record',
            'method': 'trapezoid',
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'distance': float(state['distance']),
            'v_max': float(state['v_max']),
            'a_max': float(state['a_max']),
            'post_roll': float(state['post_roll']),
            'shaper_label': state.get('shaper_label'),
            'shaper_tag': shaper_tag,
            'profile_duration': float(state['profile_duration']),
            'shaper_delay': float(state['shaper_delay']),
            'num_imu_samples': int(len(ts)),
            'num_cmd_samples': int(len(cmd_t)),
            'files': list(rel.values()),
        }
        with self._state_lock:
            self._results.append(entry)
        self.get_logger().info(
            f'motion record done: {len(ts)} IMU + {len(cmd_t)} cmd samples, '
            f'shaper={state["shaper_label"] or "off"}')


class _FlaskBridge:
    """Adapter that maps the GUI's Controller protocol to ExperimentNode."""

    def __init__(self, node: ExperimentNode) -> None:
        self._node = node

    def status(self) -> dict:
        return self._node.status()

    def list_results(self) -> list[dict]:
        return self._node.list_results()

    def get_recent_imu(self, duration: float, max_points: int):
        return self._node.get_recent_imu(duration=duration,
                                         max_points=max_points)

    def configure_shaper(self, name, frequency, damping_ratio, enabled):
        return self._node.configure_shaper(
            name=name, frequency=frequency,
            damping_ratio=damping_ratio, enabled=enabled,
        )

    def start_motion(self, distance, v_max, a_max):
        return self._node.start_motion(distance=distance,
                                       v_max=v_max, a_max=a_max)

    def start_motion_record(self, distance, v_max, a_max, post_roll):
        return self._node.start_motion_record(
            distance=distance, v_max=v_max, a_max=a_max,
            post_roll=post_roll)

    def start_psd(self, method, params):
        return self._node.start_psd(method=method, params=params)

    def cancel(self):
        return self._node.cancel()


def main() -> None:
    rclpy.init()
    node = ExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
