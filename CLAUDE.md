# CLAUDE.md

Guidance for working in this repo. See `README.md` for the human-facing setup.

## What this is
**Nano** — a mobile robot on a **NanoPi NEO Plus2 (Allwinner H5, aarch64, 1 GB RAM)**
running **Armbian**, with **ROS 2 Humble** installed as conda packages via
**pixi + RoboStack** (channel `robostack-staging`). Middleware is **`rmw_zenoh`**
(chosen for low RAM; needs `rmw_zenohd` running). The web UI is **rosbridge + a
static HTML page** (`web_control`), not Foxglove.

Hardware: Roborock **LDS02RR** lidar (UART1 `/dev/ttyS1`), single-channel **wheel
encoders** + **motors** (now via an **ESP32-WROOM coprocessor**, see below),
**PCA9685** PWM (I2C, now unused by the stack), **SSD1306** OLED (I2C), **BWT901CL**
IMU (WitMotion, USB-serial/CH340), **Logitech C270** webcam + mic (USB).

## Layout (`src/`)
- `robot_msgs` — custom interfaces (ament_cmake).
- `robot_bringup` — launch files + **the single config `config/robot.yaml`** (all
  ports/pins/rates live here).
- `lds_driver_py` — **the LDS driver in use** (rclpy, publishes `/scan`).
- `lds_driver` — **abandoned** Rust/r2r LDS node; does NOT build against this
  RoboStack. Kept for reference only. **Do not try to build it** (see below).
- `wheel_odometry` — integrates `/wheel_ticks` (from the ESP32) into `/odom`+TF;
  no longer reads GPIO.
- `motor_control` — **retired** (PCA9685 path). The ESP32 owns `/cmd_vel`→motors;
  not launched by `stack.sh`/`robot.launch.py`. Kept for the optional PCA9685
  LDS-spin/aux channels only.
- `oled_display`, `imu_driver`, `sys_monitor`, `web_control` — rclpy nodes.

## ESP32 motor/encoder coprocessor (`firmware/nanobot_coprocessor/`)
- **Native zenoh-pico over a direct UART link** (PlatformIO + Arduino) — NO micro-ROS,
  NO Fast-DDS, no agent. It joins the SBC's `rmw_zenoh` graph directly, emitting
  rmw_zenoh's exact wire format + liveliness tokens (see the `src/main.cpp` header).
  Subscribes `/cmd_vel` (geometry_msgs/Twist → diff-drive → H-bridge LEDC PWM), `/led`
  (Bool, onboard-LED pipeline test), `/lds_target_rpm` (Float32 PID setpoint). Publishes
  `/wheel_ticks` (Int64MultiArray `[L,R]`) from **single-channel** rising-edge GPIO-
  interrupt counts (unsigned, **no direction**), `/left_wheel_suspended` +
  `/right_wheel_suspended` (Bool per-wheel off-ground microswitch, **published on change**
  for low latency + a 1 Hz heartbeat republish), `/esp32_temp` (Float32) + `/esp32_hall`
  (Int32) on-die telemetry, and `/esp32_heartbeat` (Int32). Also reads a **spin-lidar**
  (LDS02RR) → `/lds_rpm` (Float32, RPM only — scan data ignored; 0 when stale) + `/lds_hz`
  (valid-frame rate, 0 = not receiving), and closed-loop-controls its spin motor: a PID
  (**tune on hardware**) holds `/lds_target_rpm` by driving the motor PWM, output on
  `/lds_duty`. The LDS path is gated by `LDS_ENABLED` (currently 0). WiFi/BT kept off.
- **Tunables are `#define`s inline at the top of `src/main.cpp`** (there is no
  `include/config.h`). `include/zenoh_generic_config.h` only holds zenoh-pico feature
  flags (enables `Z_FEATURE_LINK_SERIAL`). Pins: encoders L=19 R=26, switches 18/27,
  motor STBY=23, LEFT_IN_REV=4, **UART2 = zenoh link (TX=17, RX=16)**, LDS data on
  GPIO35 (UART1 RX-only), LDS motor PWM=21. Keep diff-drive limits synced to `robot.yaml`.
- **The link needs a serial-capable `zenohd`** — the conda `libzenohc` is built without
  `transport_serial`, so stock `rmw_zenohd` can't open the UART. Build one with
  `firmware/nanobot_coprocessor/tools/build_zenohd_serial.sh {x86_64|aarch64}`; `stack.sh`
  runs it on the board so the ESP32 (serial) and the rmw_zenoh nodes (TCP) share a graph.
  See [[robostack-zenoh-no-serial]] and [[esp32-zenoh-pico-integration]].
- Build/flash from the dev PC: `cd firmware/nanobot_coprocessor && pio run -t upload`
  (pio lives in `~/pio-venv`). **Don't build the firmware on the board.**

## Build / run
- Build: `pixi run build` (colcon, msgs + all python pkgs). There is **no
  `build-lds`/`build-all`** — the Rust node and its toolchain are intentionally gone.
- Run the stack: **`scripts/stack.sh {up|down|restart|status}`** (run via
  `pixi run bash scripts/stack.sh ...`). It starts router → agent → rosbridge → web
  → oled → imu → sys → odom → lds in order, idempotent (pgrep-guarded), logs to
  `.run/*.log`. (The agent is skipped if `$AGENT_DEV` isn't plugged in.)
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
