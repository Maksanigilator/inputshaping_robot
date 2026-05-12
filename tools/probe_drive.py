#!/usr/bin/env python3
"""Side-by-side observer for the drive chain.

Subscribes to:
  * /cmd_vel          (geometry_msgs/Twist)    -- what we asked for
  * /wheel_odometry   (geometry_msgs/Twist)    -- velocity integrated by
                                                   odroid_driver from the
                                                   motor encoders
  * /odom             (nav_msgs/Odometry)      -- full pose, if the driver
                                                   publishes it (optional)

Each 50 ms row shows the latest known value on each topic and how stale it
is. On Ctrl-C (or after --duration) the script prints a summary with
**three independent estimates of the travelled distance**:

  - integrated from /cmd_vel  (what we *commanded*)
  - integrated from /wheel_odometry  (what the encoders *think* happened)
  - direct delta of /odom.pose.position  (driver's own pose, if available)

Compare these against a tape-measure on the floor. If the encoder-based
estimates are close to the commanded value but the robot physically moves
much less, the wheels are slipping. If all three estimates are large but
the robot moved very little, the calibration (wheel_radius, gear ratio in
odroid_driver) is wrong -- everything in the ROS graph is scaled by the
same wrong factor.

Usage::

    python3 probe_drive.py                  # run until Ctrl-C
    python3 probe_drive.py --duration 12    # run for 12 seconds and exit
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('inputshaping_probe_drive')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._last_cmd: tuple[float, float, float] | None = None
        self._last_odom: tuple[float, float, float] | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._pose_origin: tuple[float, float] | None = None
        # Trapezoidal integration of linear.x from cmd_vel & wheel_odometry.
        self._cmd_dist = 0.0
        self._odom_dist = 0.0
        self._cmd_prev: tuple[float, float] | None = None
        self._odom_prev: tuple[float, float] | None = None
        # Max speeds we've seen (handy for sanity checking).
        self._cmd_vmax = 0.0
        self._odom_vmax = 0.0
        # /odom pose delta -- this is the driver's own integrator, the most
        # honest single-number estimate we can get without an external
        # ground-truth reference.
        self._pose_dx_abs = 0.0
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, qos)
        self.create_subscription(Twist, '/wheel_odometry', self._on_odom, qos)
        # /odom is optional -- some drivers only publish wheel_odometry.
        # Use BEST_EFFORT since many odom publishers use it.
        odom_qos = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/odom', self._on_pose, odom_qos)
        self.create_timer(0.05, self._tick)
        self._t0 = time.monotonic()
        self._row = 0

    def _on_cmd(self, m: Twist) -> None:
        now = time.monotonic()
        v = float(m.linear.x)
        if self._cmd_prev is not None:
            dt = max(0.0, now - self._cmd_prev[0])
            # Trapezoidal rule with the previous sample.
            self._cmd_dist += 0.5 * (v + self._cmd_prev[1]) * dt
        self._cmd_prev = (now, v)
        self._last_cmd = (now, v, float(m.angular.z))
        self._cmd_vmax = max(self._cmd_vmax, abs(v))

    def _on_odom(self, m: Twist) -> None:
        now = time.monotonic()
        v = float(m.linear.x)
        if self._odom_prev is not None:
            dt = max(0.0, now - self._odom_prev[0])
            self._odom_dist += 0.5 * (v + self._odom_prev[1]) * dt
        self._odom_prev = (now, v)
        self._last_odom = (now, v, float(m.angular.z))
        self._odom_vmax = max(self._odom_vmax, abs(v))

    def _on_pose(self, m: Odometry) -> None:
        x = float(m.pose.pose.position.x)
        y = float(m.pose.pose.position.y)
        if self._pose_origin is None:
            self._pose_origin = (x, y)
        dx = x - self._pose_origin[0]
        dy = y - self._pose_origin[1]
        self._pose_dx_abs = math.hypot(dx, dy)
        self._last_pose = (time.monotonic(), dx, dy)

    def _tick(self) -> None:
        now = time.monotonic()
        rel = now - self._t0
        cmd_v = '   ?  '
        cmd_age = '   ?  '
        if self._last_cmd is not None:
            cmd_v = f'{self._last_cmd[1]:+.3f}'
            cmd_age = f'{(now - self._last_cmd[0]) * 1000:5.0f}ms'
        odom_v = '   ?  '
        odom_age = '   ?  '
        if self._last_odom is not None:
            odom_v = f'{self._last_odom[1]:+.3f}'
            odom_age = f'{(now - self._last_odom[0]) * 1000:5.0f}ms'
        pose_dx = '   ?  '
        if self._last_pose is not None:
            pose_dx = f'{self._last_pose[1]:+.3f}'
        if self._row % 25 == 0:
            print('   t,s | cmd vx       odom vx     | age_cmd  age_odom | '
                  '   /odom dx')
        print(f'{rel:6.2f} | cmd vx={cmd_v}  odom vx={odom_v} '
              f'| {cmd_age}  {odom_age} | {pose_dx} m',
              flush=True)
        self._row += 1

    def print_summary(self) -> None:
        bar = '=' * 60
        print()
        print(bar)
        print('Drive-chain summary')
        print(bar)
        print(f'  peak |cmd_vel.linear.x|       : {self._cmd_vmax:.3f} m/s')
        print(f'  peak |wheel_odom.linear.x|    : {self._odom_vmax:.3f} m/s')
        print()
        print('  integrated distance (signed):')
        print(f'    from /cmd_vel              : {self._cmd_dist:+.3f} m')
        print(f'    from /wheel_odometry       : {self._odom_dist:+.3f} m')
        if self._last_pose is not None:
            dx, dy = self._last_pose[1], self._last_pose[2]
            print(f'    /odom net delta (x,y)      : '
                  f'({dx:+.3f}, {dy:+.3f}) m  '
                  f'|d|={self._pose_dx_abs:.3f} m')
        else:
            print('    /odom net delta             : (no Odometry msgs)')
        print(bar)
        print('Compare the numbers above with a tape measurement on the')
        print('floor. If all three rows show ~1.0 m but the robot only')
        print('travelled ~0.2 m, the chain is slipping. If they show ~0.2 m')
        print('matching the floor, odometry is honest and the command/profile')
        print('is the limit (raise distance, v_max or a_max). If /odom is')
        print('much smaller than /wheel_odometry, the driver itself already')
        print('knows about the slip via fused IMU/EKF.')
        print(bar, flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=0.0,
                    help='auto-exit after this many seconds (0 = run forever)')
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    try:
        if args.duration > 0:
            stop_at = time.monotonic() + args.duration
            while time.monotonic() < stop_at:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
