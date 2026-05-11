# Pico W firmware (MicroPython)

Streams MPU6050 accelerometer samples over USB-CDC at ~1 kHz. The host node
`mpu_serial_node` decodes the binary frames and republishes as
`sensor_msgs/Imu`.

## Wiring

| Pico pin | MPU6050 pin | Function       |
|----------|-------------|----------------|
| GP0 (1)  | SDA         | I2C0 SDA       |
| GP1 (2)  | SCL         | I2C0 SCL       |
| 3V3 (36) | VCC         | 3.3 V power    |
| GND (38) | GND, AD0    | GND, sel. 0x68 |

Mount the MPU6050 on the stainless-steel flexure so its **X axis is
co-linear with the robot's forward direction**. Mark `+X` on the board
and the chassis with paint or tape; the diploma report will reference
this convention.

## Flash MicroPython

1. Hold `BOOTSEL` and plug the Pico into the laptop. It mounts as a USB
   drive `RPI-RP2`.
2. Download a **versioned** UF2 from
   <https://micropython.org/download/RPI_PICO_W/> (first link under
   *Releases*, e.g. `RPI_PICO_W-20260406-v1.28.0.uf2`) and copy it onto the
   drive. There is **no** `RPI_PICO_W-latest.uf2` URL on that server — a
   plain `wget …/RPI_PICO_W-latest.uf2` returns 404.
3. The Pico reboots into MicroPython.

## Upload the firmware

Easiest with `mpremote` (`pip install mpremote`):

```bash
mpremote connect /dev/ttyACM0 cp main.py :main.py
mpremote connect /dev/ttyACM0 reset
```

If `mpremote` reports *failed to access /dev/ttyACM0 (it may be in use)*,
something else has the serial port open — for example `docker compose up`,
`mpu_serial_node`, `screen`/`minicom`, or another `mpremote` session. Stop
that process or unplug the Pico, then retry. On Linux you can see the
holder with `lsof /dev/ttyACM0` or `fuser -v /dev/ttyACM0`.

The onboard LED blinks at ~2 Hz while the firmware is streaming.

## Wire format

See `inputshaping_core/pico_protocol.py` in the workspace. Each frame is 14
bytes little-endian: `2sHIhhh` (`sync, seq, t_us, ax, ay, az`). Raw accel
values are MPU6050 ADC counts at ±2 g (16384 LSB/g).
