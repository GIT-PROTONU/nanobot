# Nano Robot

A lightweight **ROS 2 Humble** robot stack for a **NanoPi NEO Plus2 (Allwinner H5)**
running Armbian. Software is managed entirely with **[pixi](https://pixi.sh)** +
**[RoboStack](https://robostack.github.io/)** (ROS 2 delivered as conda packages —
no `apt`, proper `aarch64` builds), and middleware is **`rmw_zenoh`** instead of the
default FastDDS to keep RAM/discovery cost low on the 1 GB board.

Hardware:

| Peripheral | Bus / port | Node | Topic |
|---|---|---|---|
| Roborock **LDS02RR** lidar (scan) | UART2 (`/dev/ttyS2`, PA0/PA1) | `lds_driver_py` (Python) | `/scan` |
| **ESP32-WROOM** coprocessor (motors + encoders + lidar RPM/spin) | UART1 (`/dev/ttyS1`, PG6/PG7 — zenoh-pico link) | firmware | sub `/cmd_vel`; pub `/wheel_ticks`, `/lds_rpm`, `/lds_hz`, `/esp32_*` |
| Wheel **encoders** (single-channel ×2) + **motors** | ESP32 GPIO (see firmware pinout) | `wheel_odometry` (`/wheel_ticks`→`/odom`) | `/odom`, TF |
| **SSD1306** OLED | I2C0 (`/dev/i2c-0`, PA11/PA12 @400 kHz) | `oled_display` | sub `/oled_face`, `/oled_word` |
| **Speaker** (text-to-speech) | USB/analog audio out (`aplay`) | `web_control` (`espeak-ng`) | `POST /tts`; pub `/oled_word` |
| **BWT901CL** IMU | USB-serial (`/dev/imu`, CH340) | `imu_driver` | `/imu/data`, `/imu/web` |
| **Logitech C270** webcam + mic | USB | `web_control` | `/stream.mjpg`, `/audio.pcm` |
| **PCA9685** PWM | I2C1 (`/dev/i2c-1`, 0x40) | — (retired; ESP32 owns motors) | — |
| Web control + map | — | `web_control` + rosbridge | browser |

## Architecture

Each subsystem is its own ROS 2 node, wired together **only through topics** — the
same separation ROS 2 itself encourages, so any node can be launched, restarted, or
debugged in isolation. Packages live under `src/`:

```
robot_msgs        custom interfaces (WheelEncoders, MotorCommand)         [ament_cmake]
robot_bringup     launch files + the single config/robot.yaml             [ament_python]
lds_driver_py     serial LDS02RR -> /scan                                 [rclpy]
lds_driver        abandoned Rust/r2r LDS node (doesn't build; kept for ref) [Rust / r2r]
wheel_odometry    /wheel_ticks (from ESP32) -> /odom + TF                 [rclpy]
motor_control     retired (ESP32 owns /cmd_vel->motors; PCA9685 aux only) [rclpy]
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

> **Reflash recovery / one-shot:** `sudo bash deploy/sbc-setup.sh` applies all the
> OS-level config automatically and idempotently — device-tree overlays
> (`i2c0 i2c1 i2c2 uart1 usbhost1 usbhost2`), the I2C udev rule, the `dialout` +
> `video` group memberships, and the scoped passwordless `poweroff`/`reboot` sudoers
> rule the web UI's **Shutdown** button uses. Then `sudo reboot` and continue at §2.
> The rest of this section is the manual equivalent / explanation.

The H5 buses must be muxed before Linux exposes `/dev/i2c-*`, `/dev/ttyS1`, etc.
See [`nanopi-neo-plus2-pinmap.md`](nanopi-neo-plus2-pinmap.md) for the full mapping.
Easiest: `sudo armbian-config` → *System → Hardware*, enable **i2c1**, **uart2**,
then reboot. Or edit `/boot/armbianEnv.txt`:

```
overlays=i2c0 i2c1 i2c2 uart1 uart2 usbhost0 usbhost1 usbhost2 usbhost3
user_overlays=i2c0-400k
```
(`deploy/sbc-setup.sh` writes exactly this — `i2c0-400k` raises the OLED bus to 400 kHz.)

Verify after reboot:

```bash
ls /dev/i2c-0 /dev/ttyS1 /dev/ttyS2 /dev/gpiochip0   # OLED bus, ESP32 link, LDS, GPIO
pixi run python scripts/i2c_scan.py                  # expect 0x3C SSD1306 on i2c-0
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
pixi install          # resolves ROS 2 Humble + rmw_zenoh + hw libs (aarch64)
```

## 3. Build

```bash
pixi run build        # colcon build (msgs + all python pkgs)
```

> The LDS is driven by the Python `lds_driver_py` node (built by `pixi run build`
> like the rest). The old Rust `lds_driver` is abandoned — it doesn't build against
> this RoboStack Humble, and its toolchain is intentionally **not** in `pixi.toml`
> (it cost ~1.6 GB). The source is kept under `src/lds_driver/` for reference only.

## 3b. (Optional) Text-to-speech

The web UI's **Speak** box reads a line aloud (English) and karaokes the
words onto the OLED. It needs `espeak-ng`. Install it — en-gb voice only —
once on the board:

```bash
sudo bash deploy/install-espeakng.sh
```

**Audio out — the SBC's internal H5 analog codec:** enable + un-mute it (it boots
muted) and make it the default ALSA device:

```bash
sudo bash deploy/enable-h5-audio.sh     # reboot once if it says the codec isn't up yet
```

Then `web_control: tts_device` can stay `""`. (For a USB speaker instead, skip that
script and set `tts_device` to the USB card — find it with `aplay -l`.) Skip TTS
entirely and everything else still runs — it just reports "unavailable".

## 3c. (Optional) Personality / "brain"

Nano has an autonomous personality layer — a statechart that decides *when* to act and an
OpenRouter LLM that decides *what* to say (spoken line + OLED face), with traits that evolve
over time. It's entirely best-effort: no key / no network = silent, and it can never make the
robot unsafe. See **[docs/brain.md](docs/brain.md)** for the full picture, and test it
off-robot with `scripts/dev_webui.py --behavior`.

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
- **Rust LDS node (abandoned):** `lds_driver/src/main.rs` targets r2r 0.9, which
  does not compile against this RoboStack Humble. `lds_driver_py` replaced it. Its
  build toolchain (rust/clang/llvm, ~1.6 GB) is deliberately excluded from
  `pixi.toml`; re-add those deps only if you revive the node.
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
- **UARTs**: `ttyS2` (PA0/PA1) is the **LDS scan** data line and `ttyS1` (PG6/PG7,
  normally on-board Bluetooth — now disabled) is the **ESP32 zenoh-pico link**; both need
  their overlays (`uart1`/`uart2`). `ttyS0` is the serial console.
- Scan the bus with `pixi run python scripts/i2c_scan.py` (expect `0x3c` SSD1306 on
  **bus 0** (i2c-0); `0x40` PCA9685 on bus 1 if still wired).
- The build needs CMake **3.x** (not 4) plus Ninja + explicit Python hints — handled by
  `scripts/build.sh` / `pixi.toml`; see that script's header for the why.

