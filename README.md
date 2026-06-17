# Nano Robot

A lightweight **ROS 2 Humble** robot stack for a **NanoPi NEO Plus2 (Allwinner H5)**
running Armbian. Software is managed entirely with **[pixi](https://pixi.sh)** +
**[RoboStack](https://robostack.github.io/)** (ROS 2 delivered as conda packages —
no `apt`, proper `aarch64` builds), and middleware is **`rmw_zenoh`** instead of the
default FastDDS to keep RAM/discovery cost low on the 1 GB board.

Hardware:

| Peripheral | Bus | Node | Topic |
|---|---|---|---|
| Roborock **LDS02RR** lidar | UART (`/dev/ttyS2`) | `lds_driver` (Rust) | `/scan` |
| Wheel **encoders** (quadrature ×2) | GPIO (`/dev/gpiochip0`) | `wheel_odometry` | `/odom`, `/joint_states`, `/wheel_encoders`, TF |
| **PCA9685** PWM (motor driver) | I2C (`/dev/i2c-1`) | `motor_control` | sub `/cmd_vel` |
| **SSD1306** OLED | I2C (`/dev/i2c-1`) | `oled_display` | sub `/odom`, `/scan` |
| Web control + map | — | `web_control` + rosbridge | browser |

## Architecture

Each subsystem is its own ROS 2 node, wired together **only through topics** — the
same separation ROS 2 itself encourages, so any node can be launched, restarted, or
debugged in isolation. Packages live under `src/`:

```
robot_msgs        custom interfaces (WheelEncoders, MotorCommand)         [ament_cmake]
robot_bringup     launch files + the single config/robot.yaml             [ament_python]
lds_driver        serial LDS02RR -> /scan  (the hot path)                 [Rust / r2r]
wheel_odometry    GPIO encoders -> /odom + TF                             [rclpy]
motor_control     /cmd_vel -> PCA9685 PWM                                 [rclpy]
oled_display      status dashboard -> SSD1306                             [rclpy]
web_control       rosbridge websocket + static control/visualiser page   [rclpy]
```

Topic graph:

```
 lds_driver ──/scan──┐                         ┌──> oled_display
                     ├──> web_control (rosbridge ws) ──> browser
 wheel_odometry ─/odom┘                         │ teleop
        ▲                                       └──/cmd_vel──> motor_control ──> PCA9685
        └ TF odom->base_link                                          │
                                                            (optional) └ LDS spin motor
```

## 1. Prepare Armbian (enable the buses)

The H5 buses must be muxed before Linux exposes `/dev/i2c-*`, `/dev/ttyS2`, etc.
See [`nanopi-neo-plus2-pinmap.md`](nanopi-neo-plus2-pinmap.md) for the full mapping.
Easiest: `sudo armbian-config` → *System → Hardware*, enable **i2c1**, **uart2**,
then reboot. Or edit `/boot/armbianEnv.txt`:

```
overlays=i2c1 uart2
```

Verify after reboot:

```bash
ls /dev/i2c-1 /dev/ttyS2 /dev/gpiochip0
i2cdetect -y 1          # expect 0x40 (PCA9685) and 0x3C (SSD1306)
```

Add your user to the `i2c`, `dialout`, and `gpio` groups so the nodes can open the
devices without root:

```bash
sudo usermod -aG i2c,dialout,gpio $USER   # re-login afterwards
```

## 2. Install pixi + the environment

```bash
curl -fsSL https://pixi.sh/install.sh | bash    # if not already installed
cd ~/Nano
pixi install          # resolves ROS 2 Humble + rmw_zenoh + Rust + hw libs (aarch64)
```

## 3. Build

```bash
pixi run build-all    # colcon build (msgs + python pkgs) + cargo build the Rust LDS node
```

> The Rust node is built with `cargo` (not colcon) against the active ROS env;
> `r2r` generates its message bindings at build time, so it must run inside the pixi
> environment. The binary lands at `src/lds_driver/target/release/lds_driver`.

## 4. Run

`rmw_zenoh` needs its router for discovery. In one terminal:

```bash
pixi run zenohd
```

In another:

```bash
pixi run bringup     # launches all nodes + the web stack
```

Open **`http://<robot-ip>:8080`** (the OLED shows the IP). The page auto-connects to
rosbridge on `:9090`, draws the live `/scan`, and teleops via the on-screen pad or
`WASD`/arrow keys.

Useful subsets:

```bash
pixi run web         # just rosbridge + the control page (UI development)
pixi run lds         # just the Rust LDS node
pixi run shell       # a shell with everything sourced, for ad-hoc ros2 commands
```

## Configuration

Everything tunable lives in **`src/robot_bringup/config/robot.yaml`**: serial port,
I2C addresses, encoder GPIO numbers, ticks/rev, wheel geometry, PCA9685 channel
mapping, drive limits, OLED size, web ports. **Set the encoder GPIO numbers and
PCA9685 channels to match your wiring before driving** — the defaults are
placeholders. GPIO numbers are *global* libgpiod offsets (`bank*32 + pin`); confirm
against the pinmap and avoid lines it lists as already claimed.

## Notes / gotchas

- **First boot debugging:** UART0 (`/dev/ttyS0`, PA4/5) stays the Armbian serial
  console — keep a USB-TTL adapter handy. Don't use it for the LDS.
- **r2r version:** parameter plumbing in `lds_driver/src/main.rs` targets r2r 0.9's
  `node.params` API. If you bump r2r and it stops compiling, that's the spot to fix.
- **Offline robot:** `web/index.html` loads `roslib` from a CDN. For a robot with no
  internet, download `roslib.min.js` into `src/web_control/web/` and change the
  `<script src>` to a local path, then rebuild.
- **Run as a service:** once it works, wrap `zenohd` + `bringup` in systemd units (or
  a single `pixi run` unit) so the robot comes up headless on power-on.
- **`LDS_Visualizer.html`** (repo root) is the original Web-Serial bench tool — plug
  the LDS straight into a laptop to sanity-check the sensor independently of ROS.

## Verified deployment (NanoPi NEO Plus2)

Brought up on the real board: **Armbian 26.8 / kernel 6.18.35**, aarch64 ×4, ~970 MiB
RAM, 7 GB rootfs. Notes specific to this image:

- **I2C** needs enabling: `overlays=… i2c0 i2c1 i2c2` in `/boot/armbianEnv.txt`
  (overlay_prefix is `sun50i-h5`), then reboot → `/dev/i2c-0/1/2`. The stock image has
  **no `i2c` group**, so non-root access is granted via a udev rule — install
  `deploy/udev/90-i2c.rules` (uses the `dialout` group, which the default user is in).
- **UARTs** `/dev/ttyS0–7` are present without overlays (`ttyS2` is free for the LDS;
  `ttyS0` is the console).
- Scan the bus with `pixi run python scripts/i2c_scan.py` (expect `0x40` PCA9685,
  `0x3c` SSD1306 on bus 1).
- The build needs CMake **3.x** (not 4) plus Ninja + explicit Python hints — handled by
  `scripts/build.sh` / `pixi.toml`; see that script's header for the why.

