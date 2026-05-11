"""MicroPython firmware for Raspberry Pi Pico W with an MPU6050 accelerometer.

Streams a 14-byte binary frame per accelerometer sample (~1 kHz) to USB CDC.
The host node ``inputshaping_core.mpu_serial_node`` parses the stream and
republishes as ``sensor_msgs/Imu``.

Wiring (default I2C0)::

    Pico GP0 (pin 1)  -> MPU6050 SDA
    Pico GP1 (pin 2)  -> MPU6050 SCL
    Pico 3V3 (pin 36) -> MPU6050 VCC
    Pico GND (pin 38) -> MPU6050 GND, AD0 (selects address 0x68)

Sample rate is set by the MPU's internal DLPF: CONFIG=3 gives 1 kHz with a
44 Hz analog low-pass, which is what Klipper uses for its calibration. The
FIFO is read at ~500 Hz on the Pico side; up to ~170 samples fit in the
MPU's 1024-byte FIFO so a short USB stall does not lose data.

To upload: drop this file onto the Pico after flashing the latest
MicroPython UF2 (`rp2-pico-w-*.uf2`). The Pico will run ``main.py``
automatically on power-up.
"""

import machine
import struct
import sys
import time
from micropython import const

# ---------------------------------------------------------------------------
# Pin / I2C configuration
# ---------------------------------------------------------------------------

I2C_BUS = 0
I2C_SDA_PIN = 0
I2C_SCL_PIN = 1
# 100 kHz is safe for DuPont / breadboard wiring without external pull-ups.
# MPU6050 supports up to 400 kHz, but flaky wiring causes ETIMEDOUT bursts
# at the higher rate. The accel-only stream below stays well under the
# 1 kHz / 6 B = ~10 kbit budget on either speed.
I2C_FREQ = 100_000

# ---------------------------------------------------------------------------
# MPU6050 registers
# ---------------------------------------------------------------------------

MPU_ADDR = const(0x68)
REG_SMPLRT_DIV   = const(0x19)
REG_CONFIG       = const(0x1A)
REG_ACCEL_CONFIG = const(0x1C)
REG_FIFO_EN      = const(0x23)
REG_INT_ENABLE   = const(0x38)
REG_USER_CTRL    = const(0x6A)
REG_PWR_MGMT_1   = const(0x6B)
REG_FIFO_COUNT_H = const(0x72)
REG_FIFO_R_W     = const(0x74)
REG_WHO_AM_I     = const(0x75)

ACCEL_FIFO_EN_MASK = const(0x08)
USER_CTRL_FIFO_EN  = const(0x40)
USER_CTRL_FIFO_RST = const(0x04)

# Frame format: '<2sHIhhh' = sync(2) seq(2) t_us(4) ax(2) ay(2) az(2) = 14 B.
# NB: MicroPython's `ustruct` doesn't have the `Struct` class, only the
# `pack`/`unpack` functions -- so we keep the format string and use it
# directly each iteration.
FRAME_FMT = '<2sHIhhh'
SYNC = b'\xAA\x55'

# ---------------------------------------------------------------------------


def _i2c_retry(fn, *args, _attempts=8, _delay_ms=5):
    """Call an I2C op, retrying on OSError. DuPont/breadboard wiring on the
    GY-521 occasionally drops a transaction; a brief retry recovers cleanly
    instead of crashing the whole stream."""
    last_err = None
    for _ in range(_attempts):
        try:
            return fn(*args)
        except OSError as e:
            last_err = e
            time.sleep_ms(_delay_ms)
    raise last_err


def init_mpu(i2c):
    """Configure the MPU6050 for 1 kHz accel-only FIFO streaming."""
    # Probe. Many clone chips report a different WHO_AM_I but otherwise
    # behave identically; we accept any non-zero answer here. The diagnostic
    # script ``diagnose.py`` prints the value so you can pin yours down.
    who = _i2c_retry(i2c.readfrom_mem, MPU_ADDR, REG_WHO_AM_I, 1)[0]
    if who == 0xFF or who == 0x00:
        raise RuntimeError(
            'WHO_AM_I=0x%02X (likely bad wiring / pull-ups)' % who)

    def w(reg, val):
        _i2c_retry(i2c.writeto_mem, MPU_ADDR, reg, bytes((val,)))

    w(REG_PWR_MGMT_1, 0x80)         # device reset
    time.sleep_ms(100)
    w(REG_PWR_MGMT_1, 0x01)         # wake, PLL with X gyro as clock
    time.sleep_ms(20)
    w(REG_CONFIG, 0x03)             # DLPF cfg 3: accel BW 44 Hz, 1 kHz rate
    w(REG_SMPLRT_DIV, 0x00)         # sample rate = 1 kHz / (1 + 0) = 1 kHz
    w(REG_ACCEL_CONFIG, 0x00)       # +/-2 g full scale (16384 LSB/g)
    w(REG_INT_ENABLE, 0x00)
    # FIFO: reset, then enable accel-only FIFO.
    w(REG_USER_CTRL, USER_CTRL_FIFO_RST)
    time.sleep_ms(2)
    w(REG_FIFO_EN, ACCEL_FIFO_EN_MASK)
    w(REG_USER_CTRL, USER_CTRL_FIFO_EN)
    time.sleep_ms(2)


def stream():
    i2c = machine.I2C(I2C_BUS,
                      sda=machine.Pin(I2C_SDA_PIN),
                      scl=machine.Pin(I2C_SCL_PIN),
                      freq=I2C_FREQ)
    init_mpu(i2c)

    seq = 0
    out = sys.stdout.buffer.write
    # Local-bind hot-loop globals for speed.
    pack = struct.pack
    fmt = FRAME_FMT
    sync = SYNC
    sleep_us = time.sleep_us
    ticks_us = time.ticks_us
    read_mem = i2c.readfrom_mem
    write_mem = i2c.writeto_mem

    # Onboard LED blinks slowly while streaming so the user knows the Pico
    # is alive even though the REPL is silent. Try Pico W's 'LED' alias,
    # then Pico W's WiFi-chip GPIO directly, then plain Pico's GP25; if all
    # fail we just skip blinking instead of crashing the stream.
    led = None
    for spec in ('LED', 'WL_GPIO0', 25):
        try:
            led = machine.Pin(spec, machine.Pin.OUT)
            break
        except (TypeError, ValueError, RuntimeError):
            continue
    led_state = 0
    last_blink = ticks_us()

    while True:
        try:
            # FIFO count is a big-endian uint16.
            cnt_bytes = read_mem(MPU_ADDR, REG_FIFO_COUNT_H, 2)
            count = (cnt_bytes[0] << 8) | cnt_bytes[1]
            if count >= 1020:
                # Overflow: reset and drop. The host will notice via seq gaps.
                write_mem(MPU_ADDR, REG_USER_CTRL,
                          bytes((USER_CTRL_FIFO_RST | USER_CTRL_FIFO_EN,)))
                time.sleep_ms(1)
                continue
            n = count // 6
            if n == 0:
                sleep_us(200)
                continue
            # Cap per-burst to keep the USB write under one 64 B endpoint frame.
            if n > 32:
                n = 32
            data = read_mem(MPU_ADDR, REG_FIFO_R_W, n * 6)
        except OSError:
            # Spurious I2C timeout on flaky wiring: skip this tick and keep
            # streaming. The host sees a seq gap and just keeps going.
            time.sleep_ms(2)
            continue
        now = ticks_us() & 0xFFFFFFFF
        for i in range(n):
            o = i * 6
            ax = (data[o]     << 8) | data[o + 1]
            ay = (data[o + 2] << 8) | data[o + 3]
            az = (data[o + 4] << 8) | data[o + 5]
            if ax >= 32768: ax -= 65536
            if ay >= 32768: ay -= 65536
            if az >= 32768: az -= 65536
            out(pack(fmt, sync, seq & 0xFFFF, now, ax, ay, az))
            seq = (seq + 1) & 0xFFFF
        # Blink LED at ~2 Hz so it is visibly alive.
        if led is not None and ticks_us() - last_blink > 250_000:
            led_state ^= 1
            led.value(led_state)
            last_blink = ticks_us()


if __name__ == '__main__':
    # Startup grace window. Two purposes:
    #   1. Give the USB host a beat to enumerate before we spam binary data.
    #   2. Leave the REPL responsive for a moment so the user can press Ctrl-C
    #      and stop the stream for re-flashing.
    time.sleep_ms(1500)
    # If init_mpu raises (bad wiring etc.), retry a handful of times before
    # giving up -- the alternative is a silent REPL until the next power cycle.
    for _attempt in range(5):
        try:
            stream()
        except KeyboardInterrupt:
            break
        except Exception as _e:
            # Print to stdout so it's visible over USB CDC (REPL).
            print('[stream] crashed:', _e, '-- retrying')
            time.sleep_ms(500)
            continue
        break
