# ESP32-WROOM motor/encoder coprocessor

A micro-ROS coprocessor that offloads the robot's real-time motor + encoder work
from the NanoPi (H5). It owns `/cmd_vel` → H-bridge PWM and publishes raw wheel
encoder counts; the SBC keeps doing odometry integration.

## Why
The SBC's `wheel_odometry` ran a libgpiod **edge-event thread** decoding
quadrature on 4 GPIOs, and `motor_control` did diff-drive + I2C PWM. Both are
real-time chores. Moving them to a $4 ESP32 frees CPU/RAM on the 1 GB board and
uses the ESP32's **hardware PCNT** for zero-CPU quadrature counting.

## Topic contract (standard messages only)
| dir | topic         | type                        | meaning                              |
|-----|---------------|-----------------------------|--------------------------------------|
| sub | `cmd_vel`     | `geometry_msgs/Twist`       | diff-drive mixed → H-bridge PWM      |
| pub | `wheel_ticks` | `std_msgs/Int64MultiArray`  | `[left, right]` cumulative raw counts |

Standard types on purpose: a stock `micro_ros_agent` (Docker image / board)
bridges this with **no custom-message rebuild**. No MCU time sync — the SBC
stamps and computes `dt` on its own timer (as it always did).

## Wiring (edit `include/config.h` to match your board)
H-bridge: DRV8833 / TB6612-style, two PWM inputs per motor.

| signal        | GPIO | notes                                  |
|---------------|------|----------------------------------------|
| LEFT_IN_FWD   | 25   | LEDC PWM                               |
| LEFT_IN_REV   | 26   | LEDC PWM                               |
| RIGHT_IN_FWD  | 32   | LEDC PWM                               |
| RIGHT_IN_REV  | 33   | LEDC PWM                               |
| MOTOR_STBY    | 27   | TB6612 STBY (HIGH=enable); `-1` if N/A |
| LEFT_ENC_A/B  | 18/19| PCNT, internal pull-ups                |
| RIGHT_ENC_A/B | 16/17| PCNT, internal pull-ups                |

UART0/USB is the micro-ROS link (115200) — don't use `Serial.print` for debug.
Avoid flash pins 6–11; input-only 34–39 have no pull-ups (kept off the encoders).

## Build & flash (from the Windows dev PC)
PlatformIO CLI (or the VS Code extension):
```sh
cd firmware/esp32_coprocessor
pio run                       # build (first build downloads + precompiles micro-ROS)
pio run -t upload --upload-port COM10
```

## Run the agent
The agent bridges the serial link into the ROS 2 graph. **It must use the same
RMW as the rest of your graph.**

### On the dev PC now (COM10)
COM ports aren't directly reachable from Docker/WSL. Easiest paths:
- **WSL2 + usbipd-win**: `usbipd attach --busid <esp32 busid> --wsl`, the device
  appears as `/dev/ttyUSB0`, then run the agent in WSL:
  ```sh
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
  # or: docker run -it --rm -v /dev:/dev --privileged microros/micro-ros-agent:humble \
  #         serial --dev /dev/ttyUSB0 -b 115200
  ```
- Quick smoke test (matching RMW): `ros2 topic echo /wheel_ticks`,
  `ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'`.

### On the board later (`/dev/ttyUSB*`, rmw_zenoh)
The agent has to join the **zenoh** graph, so run it under `rmw_zenoh_cpp` after
`rmw_zenohd` is up (see `scripts/stack.sh`):
```sh
RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
A udev symlink (e.g. `/dev/esp32`) by the USB-serial VID:PID keeps the path
stable across reboots/ports — mirror the IMU's `/dev/imu` rule in `deploy/`.

## Tuning
`include/config.h` holds pins, PWM freq/res, the diff-drive limits (keep these in
sync with `robot.yaml`'s `motor_control` block), the `cmd_vel` watchdog timeout,
and per-wheel/per-encoder direction inverts.
