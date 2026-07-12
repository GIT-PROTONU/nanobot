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
| Web control + map | — | `web_control` (HTTP + SSE gateway) | browser |

## Architecture

Each subsystem is its own ROS 2 node wired through topics, but **on the board the
nodes run packed into three single-process "hubs" matching the three fault domains**
(each hub = one executor = one interpreter's RAM, supervised by its own systemd unit):

```
nano-router    zenohd-serial — the rmw_zenoh graph + the ESP32's UART link
nano-sensors   sensor_hub:  imu_driver + sys_monitor + wheel_odometry + lds_driver_py
nano-nav       slam_nav:    super-light 2D SLAM + click-to-go navigation
nano-app       app_hub:     web_control + oled_display + behavior (the personality)
nano-map       map_bridge:  /dev/shm map blob -> /map OccupancyGrid (for remote RViz)
```

Packages under `src/`:

```
robot_msgs        custom interfaces                                       [ament_cmake]
robot_bringup     launch files + the single config/robot.yaml             [ament_python]
lds_driver_py     serial LDS02RR -> /scan + /dev/shm scan blob            [rclpy]
imu_driver        BWT901CL IMU -> /imu/data, /imu/euler, /imu/web         [rclpy]
wheel_odometry    /wheel_ticks (from ESP32) -> /odom + TF                 [rclpy]
sys_monitor       /diagnostics + fan curve + health log + vitals blob     [rclpy]
slam_nav          scan-matching SLAM, planner, pure-pursuit control       [rclpy]
oled_display      SSD1306 dashboard + animated-eyes faces                 [rclpy]
behavior          Sismic presence statechart + purpose/A-B "brain"        [rclpy]
web_control       static control page + the browser's telemetry/control
                  gateway (SSE /telemetry, POST /publish|/param|/drive),
                  TTS, camera/mic, the LLM cognition core + skill library [rclpy]
sensor_hub        single-process host for the four sensor nodes           [rclpy]
app_hub           single-process host for web+oled+behavior               [rclpy]
sim_hardware      dev-PC Gazebo stand-ins + the map bridge                [rclpy]
lds_driver        abandoned Rust/r2r LDS node (doesn't build; reference)  [Rust / r2r]
motor_control     retired (ESP32 owns /cmd_vel->motors; PCA9685 aux only) [rclpy]
```

Data flows over **two planes**: the typed ROS/zenoh graph carries the small control
messages (incl. the ESP32 via zenoh-pico), while heavy/browser data rides `/dev/shm`
blobs + HTTP — the scan and map blobs are polled by the page, one SSE stream
(`/telemetry`) carries every light readout, and `sys_monitor`'s vitals blob feeds the
OLED dashboard + the cognition body snapshot without any fast subscriptions.

```
 lds_driver_py ─/scan──> slam_nav ─/dev/shm map─┐            ┌─> browser
 wheel_odometry ─/odom─> slam_nav   scan blob ──┼─ web_control┤   (one origin:
        ▲                           vitals blob─┘  (HTTP+SSE) │    page + telemetry
        └/wheel_ticks── ESP32 <──/cmd_vel── teleop POST /drive┘    + media + control)
```

## 1. Prepare Armbian (enable the buses)

> **Reflash recovery / one-shot:** `sudo bash deploy/sbc-setup.sh` applies all the
> OS-level config automatically and idempotently — device-tree overlays
> (`i2c0 i2c1 i2c2 uart1 usbhost1 usbhost2`), the I2C udev rule, the `dialout` +
> `video` group memberships, the scoped passwordless sudoers rules (poweroff/reboot
> for the web UI's power buttons + start/stop/restart of `nano-robot.target` for
> `stack.sh`), and the per-process systemd units that auto-start the stack on boot.
> Then `sudo reboot` and continue at §2. The rest of this section is the manual
> equivalent / explanation.

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
over time. It knows the time of day (prompts carry it; during the configured quiet hours it
mutes its autonomous chatter and idles at a sleepier cadence — user-initiated speech still
works). It's entirely best-effort: no key / no network = silent, and it can never make the
robot unsafe. See **[docs/brain.md](docs/brain.md)** for the full picture, and test it
off-robot with `scripts/dev_webui.py --behavior`.

## 3d. (Optional) GPU vision

The webcam feed can be run through the H5's **Mali-450 GPU** (`gpu_vision_enable`,
default `true`) instead of a plain passthrough — motion detection ("PIR"), colour-blob
tracking, and a batch of cheap secondary signals (visual clutter, shiny-surface,
backlit, colour-cast, overhead-structure, focus/blur, motion-vs-target correlation),
all computed as GLES2 fragment shaders and reduced on-GPU so almost nothing crosses
back to the CPU. Needs the `lima` kernel driver loaded (`lsmod | grep lima`;
`deploy/sbc-setup.sh` handles this) — if missing, vision silently falls back to a
*software* GL rasterizer (a clear warning in the logs, not a crash) or the page's
📷 **Camera** tab has a **Manual mode** switch for a zero-cost raw passthrough, plus a
master **Camera enabled** switch to turn the whole thing off. Everything is
live-viewable and live-tunable from the web UI's **Sensors → Camera (GPU vision)**
card — hover any reading or slider there for an explanation (toggle off with
**💡 Show hints** if you don't want them). All of it is informational only today;
nothing in the robot's autonomous behavior acts on these signals yet.

## 4. Run

**On the robot** the stack runs as systemd units (installed by `deploy/sbc-setup.sh`,
auto-started on boot). `scripts/stack.sh` is the day-to-day wrapper:

```bash
bash scripts/stack.sh up|down|restart|status   # = systemctl ... nano-robot.target
journalctl -u nano-app -f                       # logs (also nano-sensors/nav/router/map)
```

Crashes restart via `Restart=on-failure`; hangs via the systemd watchdog (each hub
pets `WATCHDOG=1` from an executor timer, so a wedged callback gets the hub
restarted); a per-unit `MemoryMax` keeps a leak from taking down the 1 GB board.

Open **`http://<robot-ip>:8080`** (the OLED shows the IP). The page auto-connects to
the same-origin `/telemetry` stream, draws the live `/scan`, and teleops via the
on-screen pad or `WASD`/arrow keys — everything on one port, no rosbridge.

**Debug / dev alternatives** (same node graph, separate processes, via ros2 launch):

```bash
pixi run zenohd      # the rmw_zenoh router (own terminal), then:
pixi run bringup     # all nodes + the web gateway (real hardware)
pixi run sim         # dev PC: Gazebo stand-ins for lidar/IMU/ESP32 + RViz
pixi run visualize   # dev PC: RViz watching the REAL robot over zenoh
pixi run web         # just the control page/gateway (UI development)
pixi run smoke       # end-to-end gateway/whitelist/vitals/shutdown checks
pixi run shell       # a shell with everything sourced, for ad-hoc ros2 commands
python scripts/dev_webui.py --behavior   # the AI/personality layer with NO ROS at all
```

## Configuration

Everything SBC-side lives in **`src/robot_bringup/config/robot.yaml`**: serial ports,
I2C addresses, ticks/rev, wheel geometry, drive limits, publish rates, SLAM/nav
tuning, OLED size, web port, TTS/LLM/behaviour settings, quiet hours. Python packages
install editable and `robot.yaml`/`web/` are symlinked, so a source edit + a stack
restart applies it — no rebuild.

The ESP32 side (motor pins, encoder GPIOs, PID gains, watchdog timings) is `#define`d
at the top of `firmware/nanobot_coprocessor/src/main.cpp` and flashed from a dev PC
(`pio run -t upload`) — keep its diff-drive limits in sync with `robot.yaml`.

## Deploying to the board (from a dev PC)

```bash
scripts/deploy.sh [pkgs…]    # push src+scripts, colcon build on the board, restart
```

Credentials come from the environment / `.nano-deploy.env` (never committed). By
default it also pushes the dev-made "soul" (`memory/personality.json` + phrase bank)
over the robot's — set `DEPLOY_SOUL=0` to keep the robot's evolved personality. Run
`pixi run smoke` before deploying.

## Notes / gotchas

- **First boot debugging:** UART0 (`/dev/ttyS0`, PA4/5) stays the Armbian serial
  console — keep a USB-TTL adapter handy. Don't use it for the LDS.
- **Rust LDS node (abandoned):** `lds_driver/src/main.rs` targets r2r 0.9, which
  does not compile against this RoboStack Humble. `lds_driver_py` replaced it. Its
  build toolchain (rust/clang/llvm, ~1.6 GB) is deliberately excluded from
  `pixi.toml`; re-add those deps only if you revive the node.
- **Fully offline:** the page loads no external scripts (roslib/rosbridge are gone),
  so the UI works with no internet at all.
- **Run as a service:** `deploy/sbc-setup.sh` installs per-process systemd units under
  `nano-robot.target` (with `Restart=on-failure`), so the robot comes up headless on
  power-on; `scripts/stack.sh` wraps `systemctl {start|stop|restart}` of the target.

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

