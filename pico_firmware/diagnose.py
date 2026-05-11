"""Pico diagnostics. Run via `mpremote run pico_firmware/diagnose.py`.

Prints, in order:
  1. LED test (visually verifies firmware reached this point and the LED
     pin alias works for this Pico variant).
  2. I2C bus scan on I2C0 @ GP0/GP1 -- the MPU should respond on 0x68.
  3. MPU6050 WHO_AM_I register.
  4. One raw accelerometer sample so you can check the wiring orientation.
"""

import machine
import time


def test_led():
    led = None
    for spec in ('LED', 'WL_GPIO0', 25):
        try:
            led = machine.Pin(spec, machine.Pin.OUT)
            print('LED  : OK on', repr(spec))
            break
        except (TypeError, ValueError, RuntimeError) as exc:
            print('LED  : %r not usable (%s)' % (spec, exc))
    if led is None:
        print('LED  : NO LED FOUND')
        return
    for _ in range(6):
        led.toggle()
        time.sleep_ms(150)
    led.value(0)


def test_i2c():
    i2c = machine.I2C(0,
                      sda=machine.Pin(0),
                      scl=machine.Pin(1),
                      freq=400_000)
    devs = i2c.scan()
    print('I2C  : scan ->', [hex(d) for d in devs] if devs else 'nothing')
    if 0x68 not in devs and 0x69 not in devs:
        print('I2C  : !! No MPU at 0x68/0x69. Check SDA=GP0, SCL=GP1, GND/3V3.')
        return None
    addr = 0x68 if 0x68 in devs else 0x69
    print('I2C  : using MPU at', hex(addr))
    return i2c, addr


def test_mpu(i2c, addr):
    who = i2c.readfrom_mem(addr, 0x75, 1)[0]
    print('MPU  : WHO_AM_I = 0x%02X' % who)

    # Wake up the device so a single sample read works without full init.
    i2c.writeto_mem(addr, 0x6B, b'\x01')
    time.sleep_ms(20)
    raw = i2c.readfrom_mem(addr, 0x3B, 6)
    ax = (raw[0] << 8) | raw[1]
    ay = (raw[2] << 8) | raw[3]
    az = (raw[4] << 8) | raw[5]
    if ax >= 32768: ax -= 65536
    if ay >= 32768: ay -= 65536
    if az >= 32768: az -= 65536
    g = 9.80665
    lsb = 16384.0
    print('MPU  : ax=%+.2f ay=%+.2f az=%+.2f m/s^2 (lying flat, az ~= %.1f)' %
          (ax * g / lsb, ay * g / lsb, az * g / lsb, g))


def main():
    print('=== Pico inputshaping diagnostics ===')
    test_led()
    res = test_i2c()
    if res is not None:
        i2c, addr = res
        test_mpu(i2c, addr)
    print('=== done ===')


main()
