"""Wire format between the Pico firmware and the host MPU bridge node.

Each sample is a fixed 14-byte little-endian frame::

    +-------+-------+--------+--------+--------+--------+
    | sync  |  seq  |  t_us  |   ax   |   ay   |   az   |
    | 0xAA  | u16   | u32    | i16    | i16    | i16    |
    | 0x55  |       |        |        |        |        |
    +-------+-------+--------+--------+--------+--------+
      2 B     2 B     4 B      2 B      2 B      2 B    = 14 B

The raw accelerometer values are MPU6050 ADC counts at +/-2 g, so the
sensitivity is ``MPU6050_LSB_PER_G = 16384`` counts per g. The host converts
to m/s^2 with ``a = raw * G / MPU6050_LSB_PER_G``.

``seq`` is a 16-bit free-running counter and wraps every ~65 s at 1 kHz. The
host node tracks wrap-around to reconstruct a monotonic sample index.

``t_us`` is microseconds since the Pico boot; useful only to detect jitter
or USB stalls. The host normally relies on ``seq`` for the time axis, since
we know the sample period is 1 ms +/- crystal drift.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

SYNC_BYTES = b'\xAA\x55'
FRAME_STRUCT = struct.Struct('<2sHIhhh')
FRAME_SIZE = FRAME_STRUCT.size   # 14
assert FRAME_SIZE == 14

SAMPLE_RATE_HZ = 1000.0
SAMPLE_PERIOD_S = 1.0 / SAMPLE_RATE_HZ

# Standard gravity used to convert ADC counts to m/s^2.
G = 9.80665
MPU6050_LSB_PER_G = 16384.0
LSB_TO_MS2 = G / MPU6050_LSB_PER_G   # ~5.985e-4


@dataclass(frozen=True)
class Sample:
    seq: int          # monotonic sample index reconstructed by the host
    t_us: int         # microseconds since Pico boot (sender side)
    ax: float         # m/s^2
    ay: float         # m/s^2
    az: float         # m/s^2


class FrameParser:
    """Stream-oriented parser that resynchronises on missing/garbled bytes."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._seq_offset = 0
        self._last_seq16: int | None = None
        self.dropped_bytes = 0
        self.dropped_frames = 0

    def feed(self, data: bytes) -> list[Sample]:
        """Append raw bytes; return any complete samples decoded so far."""
        self._buf.extend(data)
        out: list[Sample] = []
        while True:
            # Find the next sync. If we are not aligned, scan forward.
            idx = self._buf.find(SYNC_BYTES)
            if idx < 0:
                # Keep one trailing byte in case it is the first half of sync.
                if len(self._buf) > 1:
                    self.dropped_bytes += len(self._buf) - 1
                    del self._buf[:-1]
                return out
            if idx > 0:
                self.dropped_bytes += idx
                del self._buf[:idx]
            if len(self._buf) < FRAME_SIZE:
                return out
            try:
                sync, seq16, t_us, ax, ay, az = FRAME_STRUCT.unpack_from(
                    self._buf, 0)
            except struct.error:
                self.dropped_frames += 1
                del self._buf[0]
                continue
            if sync != SYNC_BYTES:
                # Should never happen given the find(), but defensive.
                del self._buf[0]
                continue
            del self._buf[:FRAME_SIZE]

            # Reconstruct monotonic seq accounting for 16-bit wraparound.
            if self._last_seq16 is not None and seq16 < self._last_seq16:
                # Allow up to a 32 k step backwards before declaring wrap so
                # that a small reorder does not bump the offset.
                if (self._last_seq16 - seq16) > 32768:
                    self._seq_offset += 1 << 16
            self._last_seq16 = seq16
            seq = self._seq_offset + seq16

            out.append(Sample(
                seq=seq, t_us=t_us,
                ax=ax * LSB_TO_MS2,
                ay=ay * LSB_TO_MS2,
                az=az * LSB_TO_MS2,
            ))
