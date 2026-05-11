# inputshaping_robot

Input-shaping bench for a mobile robot. Companion stand to
[`aida_bot_ws`](https://github.com/Ant346/Academic_Immersion_Skoltech): runs
on the laptop that talks to the robot, publishes `geometry_msgs/Twist` on
`/cmd_vel_shaped`, records accelerometer data from a Raspberry Pi Pico W
with an MPU6050, computes Welch PSD, and picks an input shaper -- all
through a small browser GUI.

## Quick start

1. Flash the Pico (see `pico_firmware/README.md`). Plug it into the laptop
   running the robot stack; it should enumerate as `/dev/ttyACM0`.
2. From this repository's root, on the lab laptop:

   ```bash
   docker compose build
   docker compose up
   ```

   On first run the container builds the ROS 2 workspace; subsequent
   starts skip the build.

3. From your local machine, tunnel the GUI:

   ```bash
   ssh -L 8080:localhost:8080 user@lab-laptop
   ```

   Open <http://localhost:8080> in your browser.

The container publishes `/cmd_vel_shaped` on the same `ROS_DOMAIN_ID` as
`aida_bot_ws` (default 0) using host networking. Have the robot's mux
either subscribe to `/cmd_vel_shaped` directly or feed it back through
`cmd_vel_mux` as another input.

## Architecture

```
+----------+   USB-CDC    +----------------+ /imu/data_raw +----------+
| Pico W + |------------->| mpu_serial_node|-------------->|          |
| MPU6050  |              +----------------+               |          |
+----------+                                               |          |
                                                           | experi-  |
                              /cmd_vel_raw (optional)      | ment_    |
   external publisher ------------------------------------>| node     |
                                                           |          |
                                /cmd_vel_shaped @ 50 Hz    |          |
   robot main project   <-----------------------------------|          |
                                                           |  Flask   |
                                                           |  GUI     |
                                                           +----------+
                                                                 ^
                                                          ssh -L 8080
                                                                 |
                                                            local browser
```

Everything (except the MPU bridge) lives in `experiment_node`. The shaper,
trajectory generator, and Welch pipeline are plain Python modules so the
algorithms can be unit-tested without rclpy.

## Methodology

A mobile robot with a soft mast and a backlashed gearbox does not behave
like a 3D-printer toolhead: the gearbox dead-zone dominates whenever the
commanded velocity crosses zero. Klipper's stationary chirp therefore
produces a spectrum cluttered with backlash thumps and subharmonics.

The bench supports three excitation profiles, all of which keep the chassis
loaded throughout the measurement:

* **Bounded chirp sweep** (recommended). Ramps up to `v_cruise`, then does
  `n_passes` back-and-forth passes within a `max_distance`-meter envelope,
  with the chirp overlaid on each cruising pass. Between passes a smooth
  turnaround at `a_turn` reverses direction; net spatial drift is zero, so
  the robot ends where it started. The chirp amplitude is auto-shaped so
  peak commanded acceleration is flat across the swept band and is clamped
  to half of `v_cruise` to ensure the gearbox never crosses its backlash
  dead zone. Defaults: 1 m envelope, `v_cruise = 0.15 m/s`, two passes.
  This is the same pattern Klipper switched to in late 2024 for its
  `RESONANCE_TEST` -- with one important difference: the per-pass cruise
  time is sized for our larger backlash, not for a printer's tens of mm.
* **Moving impulse**. Ramp up, settle, fire a short rectangular pulse on
  top of `v_cruise` (`pulse_width`, `pulse_dv`), then cruise through the
  ringdown. Best for time-domain ringdown analysis with one excitation
  event.
* **Moving step**. Ramp up, settle, step velocity from `v_cruise` to
  `v_cruise + step_dv` and hold. Easiest profile to fit an
  exponentially-damped sinusoid by least squares.

After each PSD run the bench writes four files into `data/` named exactly
like Klipper's:

```
data/calibration_data_x_<timestamp>.csv  -- raw acceleration trace
data/resonances_x_<timestamp>.csv        -- frequency / PSD table
data/resonances_x_<timestamp>.png        -- annotated PSD plot
data/shaper_calibrate_x_<timestamp>.png  -- candidate shaper comparison
```

## Topic contract

| Topic               | Type                        | Direction | Notes                |
|---------------------|-----------------------------|-----------|----------------------|
| `/cmd_vel_shaped`   | `geometry_msgs/Twist`       | out       | 50 Hz, always live   |
| `/cmd_vel_raw`      | `geometry_msgs/Twist`       | in        | optional external in |
| `/imu/data_raw`     | `sensor_msgs/Imu`           | out       | ~1 kHz from Pico     |

When the GUI's shaper toggle is off the `/cmd_vel_shaped` topic is still
populated, but with the raw command bypassed through. This keeps the
contract with the robot stable: it never has to switch subscribers.

## Layout

```
inputshaping_robot/
  docker/                 Dockerfile + entrypoint
  compose.yaml
  src/inputshaping_core/  ament_python ROS 2 package
    inputshaping_core/    library modules (shapers, motion, psd, ...)
    launch/               bringup.launch.py
    config/defaults.yaml
    templates/, static/   Flask GUI
  pico_firmware/          MicroPython main.py + README
  data/                   generated CSVs and PNGs (bind-mounted)
```

## Limits and overrides

Defaults are tuned for a ~0.5 m/s chassis with a_max ~1.0 m/s^2. Override
in `src/inputshaping_core/config/defaults.yaml` or via launch parameters.
Environment variables consumed by `compose.yaml`:

| Env var                       | Default          | Meaning                |
|-------------------------------|------------------|------------------------|
| `ROS_DOMAIN_ID`               | `0`              | Match the robot's      |
| `RMW_IMPLEMENTATION`          | `rmw_fastrtps_cpp` | DDS                  |
| `INPUTSHAPING_SERIAL_PORT`    | `/dev/ttyACM0`   | Pico CDC               |
| `INPUTSHAPING_GUI_PORT`       | `8080`           | Flask listen port      |
