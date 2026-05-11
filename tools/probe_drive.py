#!/usr/bin/env python3
"""Side-by-side observer for /cmd_vel (post-mux command to the drive) and
/wheel_odometry (what the odroid_driver actually integrates from the
encoders). Run this on a host that sees the robot's ROS graph (typically
on the robot itself, inside aida_bot_ws-odroid_node-1) while you press
"Forward" in the GUI.

Output is one row per 50 ms with the latest known value on each topic and
how stale that value is. A long run looks like::

    19:31:02.105  cmd vx=+0.000  odom vx=+0.000 | age cmd=  20ms odom=  35ms
    19:31:02.205  cmd vx=+0.122  odom vx=+0.041 | age cmd=  10ms odom=  35ms
    19:31:02.305  cmd vx=+0.244  odom vx=+0.082 | ...

If `cmd` ramps up to v_max and stays there while `odom` lags or saturates
much lower, the bottleneck is the motor controller / wheel slip, not us.

Usage::

    python3 probe_drive.py                  # run until Ctrl-C
    python3 probe_drive.py --duration 12    # run for 12 seconds and exit
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
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
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, qos)
        self.create_subscription(Twist, '/wheel_odometry', self._on_odom, qos)
        self.create_timer(0.05, self._tick)
        self._t0 = time.monotonic()
        self._row = 0

    def _on_cmd(self, m: Twist) -> None:
        self._last_cmd = (time.monotonic(), m.linear.x, m.angular.z)

    def _on_odom(self, m: Twist) -> None:
        self._last_odom = (time.monotonic(), m.linear.x, m.angular.z)

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
        # Print a header every 25 rows so a long capture stays readable.
        if self._row % 25 == 0:
            print('   t,s | cmd vx       odom vx     | age_cmd  age_odom')
        print(f'{rel:6.2f} | cmd vx={cmd_v}  odom vx={odom_v} | {cmd_age}  {odom_age}',
              flush=True)
        self._row += 1


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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
