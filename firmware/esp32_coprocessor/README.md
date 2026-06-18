# ESP32-WROOM motor/encoder coprocessor

A micro-ROS coprocessor that offloads the robot's real-time motor + encoder work
from the NanoPi (H5). It owns `/cmd_vel` → H-bridge PWM and publishes raw wheel
encoder counts; the SBC keeps doing odometry integration.

## Why
The SBC's `wheel_odometry` ran a libgpiod **edge-event thread** counting wheel
encoders on GPIO, and `motor_control` did diff-drive + I2C PWM. Both are real-time
chores. Moving them to a $4 ESP32 frees CPU/RAM on the 1 GB board; the encoders are
single-channel and counted with lightweight GPIO interrupts.

## Topic contract (standard messages only)
| dir | topic         | type                        | meaning                              |
|-----|---------------|-----------------------------|--------------------------------------|
| sub | `cmd_vel`     | `geometry_msgs/Twist`       | diff-drive mixed → H-bridge PWM      |
| sub | `led`         | `std_msgs/Bool`             | onboard LED (GPIO2): `true`=on — pipeline test |
| pub | `wheel_ticks` | `std_msgs/Int64MultiArray`  | `[left, right]` cumulative raw counts (unsigned, single-channel) |
| pub | `left_wheel_suspended`  | `std_msgs/Bool`   | left wheel off the ground (microswitch); `true`=suspended |
| pub | `right_wheel_suspended` | `std_msgs/Bool`   | right wheel off the ground (microswitch); `true`=suspended |

End-to-end smoke test once the agent is running:
`ros2 topic pub --once /led std_msgs/msg/Bool '{data: true}'` should light the
onboard LED; `{data: false}` turns it off. `LED_PIN` (`config.h`) = -1 disables it.

Standard types on purpose: a stock `micro_ros_agent` (Docker image / board)
bridges this with **no custom-message rebuild**. No MCU time sync — the SBC
stamps and computes `dt` on its own timer (as it always did).

## Wiring (edit `include/config.h` to match your board)
H-bridge: DRV8833 / TB6612-style, two PWM inputs per motor.

| signal        | GPIO | notes                                  |
|---------------|------|----------------------------------------|
| LEFT_IN_FWD       | 25   | LEDC PWM                           |
| LEFT_IN_REV       | 16   | LEDC PWM (moved off 26)            |
| RIGHT_IN_FWD      | 32   | LEDC PWM                           |
| RIGHT_IN_REV      | 33   | LEDC PWM                           |
| MOTOR_STBY        | 17   | TB6612 STBY (HIGH=enable, moved off 27); `-1` if N/A |
| LEFT_ENC          | 19   | single-channel, rising-edge IRQ, pull-up |
| RIGHT_ENC         | 26   | single-channel, rising-edge IRQ, pull-up |
| LEFT_SUSPEND_PIN  | 18   | suspension microswitch, INPUT_PULLUP, HIGH=suspended |
| RIGHT_SUSPEND_PIN | 27   | suspension microswitch, INPUT_PULLUP, HIGH=suspended |
| LED_PIN           | 2    | onboard LED                        |

UART0/USB is the micro-ROS link (115200) — don't use `Serial.print` for debug.
Avoid flash pins 6–11; input-only 34–39 have no pull-ups (kept off the encoder/switch).

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
per-wheel motor direction inverts, and the suspension switch's active level.
