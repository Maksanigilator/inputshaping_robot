"""Bridge from the Pico W (MPU6050 over USB CDC) to ROS 2.

Reads the binary stream defined in :mod:`pico_protocol`, converts ADC
counts to m/s^2, and publishes ``sensor_msgs/Imu`` on ``/imu/data_raw``.

Robust to disconnects: if the serial device disappears (Pico re-plugged),
the node waits and reopens.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

from .pico_protocol import FrameParser, SAMPLE_PERIOD_S


class MpuSerialNode(Node):
    def __init__(self) -> None:
        super().__init__('mpu_serial_node')

        default_port = os.environ.get(
            'INPUTSHAPING_SERIAL_PORT', '/dev/ttyACM0')
        self.declare_parameter('port', default_port)
        self.declare_parameter('baudrate', 0)   # 0 = ignored for USB CDC
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('reconnect_period_sec', 1.0)

        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._baud = int(self.get_parameter('baudrate').get_parameter_value().integer_value)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._reconnect = self.get_parameter('reconnect_period_sec').get_parameter_value().double_value

        # Sensor data QoS: best-effort, depth 1000 to absorb USB bursts.
        self._pub = self.create_publisher(
            Imu, topic, QoSPresetProfiles.SENSOR_DATA.value)
        self._parser = FrameParser()
        self._serial: serial.Serial | None = None
        # Anchor: ROS time corresponding to seq=0 of the Pico. We compute it
        # from the first sample's arrival time; the host clock then provides
        # absolute timestamps even though the Pico itself has no RTC.
        self._t0_ns: int | None = None
        self._first_seq: int | None = None

        # Run as fast as possible. Serial.read with timeout=0 returns
        # whatever is in the OS buffer; we spin a 1 ms timer to drain it.
        self.create_timer(0.001, self._tick)
        self.get_logger().info(
            f'mpu_serial_node: port={self._port} topic={topic} '
            f'frame_id={self._frame_id}'
        )

    def _open(self) -> None:
        if self._serial is not None:
            return
        if not Path(self._port).exists():
            return
        try:
            kwargs = {'port': self._port, 'timeout': 0}
            if self._baud > 0:
                kwargs['baudrate'] = self._baud
            self._serial = serial.Serial(**kwargs)
            self._parser = FrameParser()
            self._t0_ns = None
            self._first_seq = None
            self.get_logger().info(f'opened {self._port}')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f'cannot open {self._port}: {exc}')
            self._serial = None
            time.sleep(self._reconnect)

    def _tick(self) -> None:
        if self._serial is None:
            self._open()
            return
        try:
            chunk = self._serial.read(4096)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warn(f'serial read failed: {exc}; reopening')
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            return
        if not chunk:
            return
        samples = self._parser.feed(chunk)
        if not samples:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self._t0_ns is None:
            self._first_seq = samples[0].seq
            # Anchor seq[0] to "now" so latency between Pico and host shows
            # up as a constant offset, not as drift.
            self._t0_ns = now_ns
        for s in samples:
            msg = Imu()
            # Header timestamp: t0 + (seq - first_seq) * SAMPLE_PERIOD_S.
            t_ns = self._t0_ns + int(
                (s.seq - self._first_seq) * SAMPLE_PERIOD_S * 1e9)
            msg.header.stamp.sec = t_ns // 1_000_000_000
            msg.header.stamp.nanosec = t_ns % 1_000_000_000
            msg.header.frame_id = self._frame_id
            msg.linear_acceleration.x = s.ax
            msg.linear_acceleration.y = s.ay
            msg.linear_acceleration.z = s.az
            # We don't have a gyro feed yet; mark orientation/gyro covariance
            # as "unknown" per REP-145.
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MpuSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._serial is not None:
            try:
                node._serial.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
