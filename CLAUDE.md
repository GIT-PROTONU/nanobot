# CLAUDE.md

Guidance for working in this repo. See `README.md` for the human-facing setup.

## What this is
**Nano** — a mobile robot on a **NanoPi NEO Plus2 (Allwinner H5, aarch64, 1 GB RAM)**
running **Armbian**, with **ROS 2 Humble** installed as conda packages via
**pixi + RoboStack** (channel `robostack-staging`). Middleware is **`rmw_zenoh`**
(chosen for low RAM; needs `rmw_zenohd` running). The web UI is **rosbridge + a
static HTML page** (`web_control`), not Foxglove.

Hardware: Roborock **LDS02RR** lidar (UART1 `/dev/ttyS1`), quadrature **wheel
encoders** (GPIO), **PCA9685** PWM (I2C), **SSD1306** OLED (I2C), **BWT901CL** IMU
(WitMotion, USB-serial/CH340), **Logitech C270** webcam + mic (USB).

## Layout (`src/`)
- `robot_msgs` — custom interfaces (ament_cmake).
- `robot_bringup` — launch files + **the single config `config/robot.yaml`** (all
  ports/pins/rates live here).
- `lds_driver_py` — **the LDS driver in use** (rclpy, publishes `/scan`).
- `lds_driver` — **abandoned** Rust/r2r LDS node; does NOT build against this
  RoboStack. Kept for reference only. **Do not try to build it** (see below).
- `wheel_odometry`, `motor_control`, `oled_display`, `imu_driver`, `sys_monitor`,
  `web_control` — rclpy nodes.

## Build / run
- Build: `pixi run build` (colcon, msgs + all python pkgs). There is **no
  `build-lds`/`build-all`** — the Rust node and its toolchain are intentionally gone.
- Run the stack: **`scripts/stack.sh {up|down|restart|status}`** (run via
  `pixi run bash scripts/stack.sh ...`). It starts router → rosbridge → web → oled
  → imu → sys → lds in order, idempotent (pgrep-guarded), logs to `.run/*.log`.
  Nodes are launched by their installed executables directly (not `ros2 run`) to
  keep RAM down.
- On the board the stack **auto-starts on boot** via systemd `nano-stack.service`.
- OS-level setup (overlays, udev, groups, sudoers, systemd) is scripted in
  **`deploy/sbc-setup.sh`** (idempotent; run once after a reflash + reboot).

## Conventions / gotchas
- **NEVER add `rust`, `clang`, or `libclang` to `pixi.toml`.** They were build-only
  deps for the abandoned Rust LDS node and pulled a ~1.6 GB toolchain onto the 7 GB
  card. A note in `pixi.toml` guards this.
- **Python packages are installed editable (egg-link → src).** Editing a `.py`
  under `src/<pkg>/<pkg>/` + restarting the node picks it up — **no rebuild needed**.
  A *new* module file still imports fine via the egg-link. `config/robot.yaml` and
  `web/` are symlinked into `install/`, so pushing src updates them too.
- **`rmw_zenoh` ordering matters:** a node started before `rmw_zenohd` runs islanded
  (won't appear in the graph). `stack.sh up` handles this (router first, then waits).
- **`web_control` static server**: serves `web/index.html`; `/stream.mjpg` is a
  zero-dep V4L2 MJPEG passthrough (`mjpeg_camera.py`); `/audio.pcm` is the webcam
  mic as raw PCM via `arecord` (`mic_audio.py`). Both are ref-counted (only run
  while a client is connected) and the audio endpoint **must** be HTTP/1.1 chunked
  (browsers don't stream an HTTP/1.0 body to `fetch`).
- Tune live: `imu_driver`/`lds_driver_py` expose `publish_rate` as a settable param;
  the web UI sliders call `/<node>/set_parameters` over rosbridge. The IMU's device
  stream rate auto-follows `publish_rate` (`output_rate_hz: 0`).

## Deploying to the live board (from a dev host)
- One-shot deploy: **`scripts/deploy.sh [pkgs…]`** — copies `src/`+`scripts/`,
  colcon-builds (optionally `--packages-select`), then `stack.sh restart`. Creds via
  env (`NANO_PW`, `NANO_HOST`, `NANO_HOSTKEY`) — **never commit secrets**.
- From Windows, remote shell is PuTTY `plink`/`pscp`. **`plink -m <localfile>` sends
  the file's text as the remote shell's argv**, so any `pkill -f`/`pgrep -f` (or a
  `/proc/*/cmdline` scan) whose pattern appears in the script will match and kill the
  controlling shell. Fix: `pscp` the script and run it **by path** (`plink ... "bash
  /tmp/x.sh"`).
- **`stack.sh restart` is currently unreliable at killing the old node** (can leave a
  stale process holding the port / serving old code). If a change "doesn't take",
  check for a duplicate node and do a clean `down` → verify with a python `/proc`
  scan → `up`.
- The board has only ~1 GB RAM and a 7 GB rootfs — watch memory and disk. Don't run
  heavy compiles on it.
