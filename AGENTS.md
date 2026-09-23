# AGENTS.md — Nano robot

Open work items live in [`docs/TODO.md`](docs/TODO.md) (consolidated 2026-09-16).
Guidance for working in this repo. See `README.md` for the human-facing setup.

## What this is
**Nano** — a mobile robot on a **NanoPi NEO Plus2 (Allwinner H5, aarch64, 1 GB RAM)**
running **Armbian**, with **ROS 2 Humble** installed as conda packages via
**pixi + RoboStack** (channel `robostack-staging`). Middleware is **`rmw_zenoh`**
(chosen for low RAM; needs `rmw_zenohd` running). The web UI is **a static HTML page
served by `web_control`, which is also the browser's only gateway** — there is **NO
rosbridge** (removed 2026-07-06; it cost ~a full core with the UI open).

> **Two planes.** The typed ROS/zenoh graph is the *control plane* (small messages,
> few Hz; incl. the ESP32 via zenoh-pico through `zenohd-serial`). Heavy/browser data
> is the *data plane*: `/dev/shm` blobs + HTTP (`/scan.bin`, `/map`, camera, mic, TTS)
> and ONE Server-Sent-Events stream (`GET /telemetry`, `web_control/telemetry.py`)
> carrying every light readout as a ~5 Hz JSON frame built once and fanned out to all
> viewers. Writes from the page are whitelisted POSTs: `/drive` (teleop), `/publish`
> (topic pokes, clamped per topic), `/param` (live-tune sliders). The telemetry
> subscriptions are created only while a browser is connected, so a closed page costs
> nothing. Going zenoh-all-the-way to the browser (zenoh-ts) was considered and
> rejected — you'd hand-decode CDR in JS.

Hardware: Roborock **LDS02RR** lidar (scan on **UART2 `/dev/ttyS2`**; RPM also read by
the ESP32), single-channel **wheel
encoders** + **motors** (now via an **ESP32-WROOM coprocessor**, see below),
**PCA9685** PWM (I2C, now unused by the stack), **SSD1306** OLED (I2C), **BWT901CL**
IMU (WitMotion, USB-serial/CH340), **Logitech C270** webcam + mic (USB).

## Build & run

- Build: `pixi run build` (runs `scripts/build.sh` — colcon + explicit CMake Python hints for RoboStack; msgs + all python pkgs). There is **no `build-lds`/`build-all`** — the Rust node and its toolchain are intentionally gone. Python pkgs are `--symlink-install` (edit + restart, no rebuild).
- **Do NOT add `rust`/`clang`/`libclang` to `pixi.toml`.** The LDS is driven by the pure-Python `lds_driver_py`; the old Rust `lds_driver` node was abandoned and removed. The toolchain would pull ~1.6 GB onto the 7 GB board (a note in `pixi.toml` guards this).

- **`pixi run smoke`** (`scripts/smoke_test.py`) — the end-to-end contract check: boots
  the real router + sys_monitor + app_hub on the dev PC and asserts the /telemetry
  frame keys, the publish/param whitelists, the OLED-face echo, the vitals blob, and
  the SIGTERM shutdown path. **Run it before deploying** — the telemetry frame is a
  typed-nowhere contract between `telemetry.py` and the web page, and this is what catches
  a drift.

- Run the stack: **`scripts/stack.sh {up|down|restart|status}`** — now a thin wrapper
  over **systemd**. The stack is seven units under **`nano-robot.target`**:
  `nano-router` (zenohd-serial) → `nano-sensors` (sensor_hub = imu+sys+odom+lds) →
  `nano-nav` (ONE `rclcpp_components/component_container_isolated` hosting the Nav2
  servers + their lifecycle manager; components attached by the `nano-nav-loader`
  oneshot via `nav2.launch.py load_only:=true`) → `nano-tf` (the static
  `base_link→laser` TF with yaw π — its OWN unit: a never-exiting ExecStartPost would
  hold a Type=simple unit in "activating" forever) → `nano-slam` (slam_toolbox 2.6.10,
  own process) →
  `nano-app` (app_hub = web+oled+behavior).
  (The old `nano-ekf`/`nano-map` units died with the Nav2 migration — the EKF and the
  map blob bridge are gone.)
  Ordering (`After=nano-router.service` + the router unit's ExecStartPost probe that
  waits for :7447 to actually accept) encodes the rmw_zenoh island gotcha; nav/slam
  start after sensors (need /odom + /scan). **Crash recovery is `Restart=on-failure`**, and
  **hang recovery is the systemd watchdog**: app/sensors are `Type=notify` and pet
  `WATCHDOG=1`
  every 5 s from an *executor timer* (`_sd_notify` in each main), so an alive-but-wedged
  executor (a stuck callback) stops petting and gets restarted (`WatchdogSec=90`). The
  nav units are `Type=simple` (stock C++ binaries, no watchdog) guarded instead by
  `CPUAffinity=2 3` + `Nice=10` + `MemoryMax=400M` (Ceres's thread spike + pose-graph
  growth). Each unit also has a
  `MemoryMax` cap so a leak restarts that hub instead of waking the kernel OOM killer.
  (The old `nano-heal.timer` polling — and its heal-vs-restart duplicate-node race —
  is gone.)
  What each unit execs lives in ONE place: **`scripts/unit_exec.sh`** (pixi env
  activation via `pixi shell-hook`, then `exec` of the installed executable — no
  resident wrapper, no `ros2 run` RAM overhead). **Every ROS unit also runs as a
  zenoh CLIENT of the router** (`ZENOH_SESSION_CONFIG_URI=$NANO/.run/zenoh_client.json5`,
  2026-09-22 — cross-host discovery was one-sided-blind in peer mode; the router
  branch is excluded; `NANO_ZENOH_PEER=1` reverts. The client session config MUST
  disable zenoh shared memory — see the gotcha below). **Every unit also runs a bounded
  NTP clock-step wait at the top of `unit_exec.sh`** (up to 20 s, then starts anyway):
  the board has no battery RTC, so a power-on boots with a days-stale fake-hwclock and
  NTP steps the clock mid-run while the stack is already up — under a live SLAM session
  that step drops every scan (slam_toolbox message filter: "earlier than all the data in
  the transform cache", hit 2026-09-19). **Second live instance (2026-09-21 pm, worse —
  it wrecked NAV): after a mid-session board reboot the units started stale; NTP stepped
  the clock ~64 s later; map→base_link TF stamps were future-dated vs the stepped clock →
  pose lookups failed ("extrapolation into the past") → Nav2's RPP evaluated a garbage
  pose → "detected collision ahead" ×2 → patience exceeded → the BT's backup ALSO aborted
  ("Collision Ahead") → goal failed; the user saw the robot "spin around like crazy" (RPP
  rotate-to-heading in place while collision checking refused to translate). HEAL = stack
  bounce + wake the parked lidar (a fresh slam gets no scans on a quiet robot → /map stays
  empty until the LDS spins).** The 20 s wait is evidently not enough — a LATER NTP
  correction can always land mid-session; a sys_monitor clock-step watcher (restart
  slam+nav on a detected step) is the candidate fix (docs/TODO.md). All units are
  `After=nano-router`, so the router's wait orders the whole stack. Logs: `journalctl -u nano-app` etc.

- Auto-starts on boot via systemd `nano-robot.target`. `nano-stack.service` + `nano-heal.timer` are retired. Restart/recovery is systemd's (`Restart=on-failure`); `stack.sh down` → verify via `/proc` → `up` is still the clean cycle after a deploy (`stack.sh restart` can leave stale processes holding ports — see Gotchas).
- Zenoh needs a serial-capable `zenohd` binary (conda builds lack `transport_serial`). Build with `firmware/nanobot_coprocessor/tools/build_zenohd_serial.sh {x86_64|aarch64}`; the `nano-router` systemd unit (via `scripts/unit_exec.sh router`) runs it on the board so the ESP32 (serial) and the rmw_zenoh nodes (TCP) share a graph.

- OS-level setup (overlays, udev, groups, sudoers, systemd units) is scripted in
  **`deploy/sbc-setup.sh`** (idempotent; run once after a reflash + reboot).
  stack.sh's start/stop/restart go through scoped NOPASSWD sudoers rules it installs
  (deploy/sudoers).

- **Disk-lean timer (2026-09-23):** the pixi/rattler download cache
  (`~/.cache/rattler/cache`) regrows on every `pixi install`/resolve and once hit
  **3.7 G = 91% of the 7 GB rootfs** (755 stale package dirs from past resolves —
  the live env at `~/Nano/.pixi/envs/default` keeps its own copies, so clearing it
  is regenerable-by-definition; a manual run freed 605 MB → 82%). A daily
  **`nano-disk-cleanup.timer`** (`deploy/systemd/`, new sbc-setup step **6b**)
  runs **`scripts/disk_cleanup.sh`** at ~03:20 + `Persistent=true` (catches up after
  downtime), `Nice=15`/`IOSchedulingClass=idle`/`MemoryMax=50M`. Zero sudo, entirely
  user-owned regenerable caches (rattler pkgs/repodata/mapping/uv + pip) — safe
  mid-drive, the running stack never touches those dirs. Deliberately NOT part of
  `nano-robot.target` (runs whether or not the stack is up; enabled against
  `timers.target`). Logs freed-MB to `journalctl -u nano-disk-cleanup`. NOT touched:
  the live env, `build/`, `install/`, `brain/`, `~/.local/state/nanobot` (the soul),
  `/var`/`/usr`. Already installed + armed on the live board (2026-09-23).

- Dev PC offline testing: `scripts/dev_webui.py` serves the real web UI + cognition (no ROS).

## Tests

- **Nano repo ROS-free unit tests:** `pixi run test` (adds `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` — without it the RoboStack env's `launch_testing`/`launch_testing_ros` pytest entrypoints register unknown hooks and collection crashes; see pixi.toml). Repo-root + per-package `conftest.py` files put each `src/<pkg>` on `sys.path` so tests import without a colcon build — but pytest's rootdir jumps to a package dir when you run one directly (each package has a `setup.cfg`), which is why the per-package conftest exists in addition to the root one.
  - `src/wheel_odometry/test/test_odom_math.py` — the pure differential-drive integration (`odom_math.py`) behind `/odom` + the odom→base_link TF: midpoint vs exact arc geometry, heading wrap, quat. `odom_math.py` is stdlib-only so the test never touches rclpy/robot_msgs.
  - `src/imu_driver/test/test_imu_mount_math.py` — the mount-matrix/lever-arm math that builds chassis-frame `/imu/data` (SLAM heading): pins the yaw-90 mount's roll↔pitch swap (the per-angle-shortcut bug class), lever-arm centripetal/tangential signs, quat↔matrix cross-check.
  - `src/lds_driver_py/test/test_scan_blob.py` — the `/dev/shm/nano_scan.bin` wire format: header shape, float32/inf packing, atomicity (no torn reads).
  - `src/web_control/test/test_nav_telemetry.py` — the Nav2-facing web glue: `NAV_STATUS` map, ±12 m goal clamp, degenerate/truncated `/map` rejection, goal-mirror lifecycle (terminal states 4/5/6 clear it), rmw_zenoh bytes-status normalization.
  - Brain tests live separately in the **nanobot-brain** repo at `/home/ib/Desktop/nanobot-brain`:
    ```
    cd /home/ib/Desktop/nanobot-brain
    pixi run python -m pytest tests/
    ```
    All 94 tests are ROS-free (no rclpy, no network). The `nanobot-brain` package is a standalone dependency — no colcon overlay needed.

## Dependencies

- **nanobot-brain** (`/home/ib/Desktop/nanobot-brain`) — the standalone, ROS-free brain package containing:
  - `nanobot_brain.behavior` — Sismic statechart (presence), PurposeEngine, Planner, Bandit, Personality
  - `nanobot_brain.cognition` — CognitionCore (LLM personality), LlmClient, SkillLibrary, WorkshopState, PhraseBank
  - `nanobot_brain.orchestra.NanoBrain` — unified orchestrator tying behavior + cognition
  - `nanobot_brain.interfaces` — Protocol classes for platform adapters
  - `nanobot_brain.config` — Dataclass-based config (BrainConfig, BehaviorConfig, CognitionConfig)
  
  Install via `pip install -e /path/to/nanobot-brain` or add to `pixi.toml` as a pypi dependency.

## Dev/prod ROS parity + Gazebo sim
There are now **two dev paths**, not one, serving different purposes:
- **`scripts/dev_webui.py` / `dev_run.ps1`** (Windows, no ROS at all) — unchanged, still
  the fastest way to iterate on the LLM/personality/TTS layer (see the LLM/cognition
  section below). Doesn't run `web_control`'s rclpy node, `oled_display`,
  `behavior/mood_node`, `wheel_odometry`, or the nav stack — it's a ROS-free stand-in for
  just the AI/Speak/Brain cards.
- **`scripts/sim_run.sh`** (Ubuntu/Linux dev PC, real ROS 2 via the SAME `pixi.toml`
  RoboStack env the board uses — `linux-64` is already one of its `platforms`) — runs
  the **exact same node graph** as the robot: `web_control`, `oled_display`,
  `behavior/mood_node`, `sys_monitor`, `wheel_odometry`, the Nav2 container
  (`launch/nav2.launch.py`: planner/controller/bt_navigator/behaviors/lifecycle
  manager + slam_toolbox) are all real,
  unmodified nodes. Only the lowest hardware-transducer layer differs: **Gazebo
  Sim** (`ros_gz_sim`, the modern actively-maintained "Ignition"-lineage simulator —
  `robostack-staging` doesn't cleanly ship classic `gazebo_ros_pkgs` for Humble, but does
  ship `ros-gz-*`) plus `ros_gz_bridge` and the `sim_hardware` package stand in for
  the LDS02RR/BWT901CL/ESP32. `sim_hardware.sim_bridge_node` converts Gazebo's bridged
  wheel-joint angles into `/wheel_ticks` (so the **real** `wheel_odometry` node still
  does the integration — Gazebo's own diff-drive odometry is deliberately not used) and
  its bridged IMU into `/imu/euler`+`/imu/web` matching `imu_driver`'s exact contract.
  The webcam/mic aren't simulated at all — `mjpeg_camera.py`/`mic_audio.py` are
  V4L2/ALSA and just use the dev PC's real ones.
  - `robot_bringup/launch/bringup.launch.py` (replaces the previously-stale
    `robot.launch.py`, which still referenced the abandoned Rust LDS node +
    `micro_ros_agent`) is the single launch description for both: `sim:=false` (default)
    launches the real `lds_driver_py`/`imu_driver` — a `ros2 launch`-based **debug**
    alternative to the systemd units (which stay the production launcher, for their
    RAM-saving direct-executable approach; note the launch path runs the nodes as
    separate processes, not the hubs — same graph, more RAM, fine on a dev PC);
    `sim:=true` swaps those for Gazebo +
    `ros_gz_bridge` + `sim_hardware`. `rviz:=true` also opens RViz2
    (`robot_bringup/rviz/nano.rviz`: RobotModel/TF/LaserScan/Map/Odometry).
  - The Gazebo/RViz/`ros_gz_*`/`xacro`/`robot_state_publisher` deps live under
    `pixi.toml`'s **`[target.linux-64.dependencies]`**, not the top-level
    `[dependencies]` table, so none of it ever resolves onto the board
    (`linux-aarch64`) — same "don't bloat the 1 GB/7 GB board" discipline as the
    rust/clang ban below.
  - `pixi run sim` / `scripts/sim_run.sh` build + launch it (the script additionally
    resolves `OPENROUTER_API_KEY` and pre-warms the phrase bank, mirroring
    `dev_run.ps1`'s job for the ROS-free path).

### Remote RViz (the REAL robot, not a simulation)
A third option, orthogonal to the two dev paths above: watch the **physical robot live**
in RViz from the dev PC while it runs its own systemd stack unchanged — no Gazebo, no sim.
- `scripts/rviz_remote.sh` (optionally `--connect <robot-ip>`) / `pixi run visualize` runs
  `robot_bringup/launch/visualize.launch.py`, which starts **only**
  `robot_state_publisher` + `rviz2` — deliberately NOT `wheel_odometry`/the nav stack/
  `sensor_hub`/etc. a second time (the robot is already publishing all of that; a second
  copy on the dev PC would just be a redundant duplicate publisher on the same topics).
  `/scan`, `/odom`, `/imu/euler`, TF, `/map` all stream in over the shared `rmw_zenoh`
  graph.
- **`/map` is a real ROS topic now**: slam_toolbox publishes it transient-local, so a
  remote RViz simply subscribes over the zenoh graph (the old `/dev/shm` map blob +
  `nano-map` bridge unit are gone with slam_nav).
- **Cross-host zenoh discovery (TESTED end-to-end 2026-09-22)**: `ROS_DOMAIN_ID`/
  `RMW_IMPLEMENTATION` already match by construction (both machines activate the same
  `pixi.toml`). Same-LAN zenoh multicast scouting usually finds the robot's
  `zenohd-serial` router with no extra config; if not (blocked multicast / different
  subnet), `rviz_remote.sh --connect <ip>` writes a small session config pointing at
  `tcp/<ip>:7447` and sets `ZENOH_SESSION_CONFIG_URI`. The old one-sided blindness
  (dev-PC saw only the ESP32's topics, not the robot's ROS nodes) was a **peer-mode
  propagation failure** — fixed 2026-09-22 by running every robot-side unit as a zenoh
  CLIENT of the router (`unit_exec.sh`, see above): a dev-PC session pointed at the
  router now sees /scan /odom /tf /map /wheel_* /goal_pose /cmd_vel + costmaps and
  receives real data (`ros2 topic echo --once /diagnostics` verified). Two related
  gotchas when hunting the graph: the ros2 CLI's persistent daemon caches a stale
  graph (`ros2 daemon stop` first), and a bare ssh `ros2` runs under fastrtps (wrong
  RMW sees nothing) — export `RMW_IMPLEMENTATION=rmw_zenoh_cpp` first.

## Architecture

| Package | Role |
|---|---|
| `robot_msgs` | Custom ROS interfaces (ament_cmake) |
| `robot_bringup` | Launch files + single config `config/robot.yaml` + `config/nav2/nav2_params.yaml` |
| `lds_driver_py` | Active LDS driver (rclpy → `/scan` + `/dev/shm/nano_scan.bin`) |
| `sensor_hub` | **One process** for imu_driver + sys_monitor + wheel_odometry + lds_driver_py |
| `web_control` | HTTP+SSE gateway: static web page + TTS + vision + delegates to `nanobot_brain.cognition` |
| `behavior` | ROS glue layer: Sismic chart lifecycle, topic wiring — delegates to `nanobot_brain.behavior` |
| `app_hub` | **One process** hosting web_control + oled_display + behavior (unit `nano-app`) |
| `oled_display` | I2C SSD1306 dashboard |
| `wheel_odometry` | `/wheel_ticks` → `/odom` + TF (from ESP32, not GPIO) |
| `imu_driver` | BWT901CL over USB-serial |
| `sys_monitor` | CPU/RAM/temp → `/diagnostics` |
| `sim_hardware` | Dev-PC-only Gazebo hardware stand-in (`bringup.launch.py sim:=true`) |

Navigation/SLAM are stock C++ (not packages here): **Nav2 Humble servers** in one component container (unit `nano-nav`, loaded by `nano-nav-loader`) + **slam_toolbox 2.6.10** as its own process (unit `nano-slam`); the static `base_link→laser` TF is its own `nano-tf` unit. The custom `slam_nav` package and the robot_localization EKF were deleted 2026-09-14 (`docs/nav2-migration.md`).

### src/ layout in depth (`src/`)
- `robot_msgs` — custom interfaces (ament_cmake).
- `robot_bringup` — launch files + **the config `config/robot.yaml`** (all
  ports/pins/rates) + **`config/nav2/`** (`nav2_params.yaml` for the Nav2 Humble
  servers + slam_toolbox 2.6.10, and `recovery_bt.xml` — the minimal fail→clear
  costmaps→back up→spin→retry BT). **Navigation = Nav2 + slam_toolbox** (the custom
  `slam_nav` node and the robot_localization EKF were retired 2026-09-14 — see
  [`docs/nav2-migration.md`](docs/nav2-migration.md): slam_toolbox turns `/scan` +
  `/odom` into `/map` + the `map→odom` TF; the Nav2 servers plan/drive to
  `/goal_pose` and publish `/cmd_vel` straight to the ESP32 contract).
  `launch/bringup.launch.py` is the one node graph shared
  by the real robot and the Gazebo dev-sim (`sim:=true`/`rviz:=true` args) — see
  "Dev/prod ROS parity + Gazebo sim" below. Also holds `launch/nav2.launch.py`
  (the ONE component container for the Nav2 servers + the `load_only:=true`
  systemd pairing + the static `base_link→laser` TF with yaw π for this unit's
  back-facing sensor head), the URDF (`urdf/nano.urdf.xacro`),
  the Gazebo world (`worlds/nano_room.sdf`), the `ros_gz_bridge` topic map
  (`config/gz_bridge.yaml`) and the RViz config (`rviz/nano.rviz`).
- `lds_driver_py` — **the LDS driver in use** (rclpy, publishes `/scan`; also writes a
  compact scan blob to `/dev/shm/nano_scan.bin` for the web UI — see `web_control` below).
  The blob writer is `scan_blob.write_scan_blob`, shared with `sim_hardware` so the
  Gazebo dev-sim writes byte-identical blobs.
- `wheel_odometry` — integrates `/wheel_ticks` (from the ESP32, or from `sim_hardware` in
  Gazebo dev-sim) into `/odom`; **this node owns the `odom→base_link` TF now**
  (`publish_tf: true` — the EKF is gone). No longer reads GPIO.
- `oled_display`, `imu_driver`, `sys_monitor`, `web_control` — rclpy nodes.
  `imu_driver` also wires the **WitMotion accel/mag calibration** (2026-07-16, hw-
  verification tracked in `docs/TODO.md`: `/imu_calibrate` String cmds `accel|mag_start|mag_stop|save` executed
  in the reader thread → latched `/imu_calibrate_status`; web IMU card buttons + live
  mag xyz readout — no protocol readback exists, so verification is eyeballing
  |accel|≈9.8 + a smooth mag sweep).
- `sim_hardware` — **dev-PC-only**, not built/launched on the board (linux-aarch64). Used
  only by `bringup.launch.py sim:=true`: `sim_bridge_node` re-publishes
  Gazebo's bridged `/joint_states_sim` + `/imu` + `/scan` as the exact contracts the real
  lidar/IMU/ESP32 publish (`/wheel_ticks`, `/imu/euler`+`/imu/web`, the scan blob), plus
  synthetic ESP32 board telemetry. (The old `map_bridge_node` — the `/dev/shm` map blob →
  `/map` bridge — died with slam_nav; slam_toolbox publishes `/map` natively.)
- `behavior` — **behaviour layer (Sismic statechart)**. *Human-readable overview of the
  whole brain (statechart + LLM + traits/evolution + model caps + decision log):
  [`docs/brain.md`](docs/brain.md); the bullets below are the terse engineering summary.*
  `mood_node`: an idle "feel alive"
  presence supervisor that drives the OLED face (`/oled_face`) during true idle and stands
  down when another owner uses the panel (motion/goal, TTS, manual web mood, pick-up).
  **Expression-only — never publishes `/cmd_vel`.** The chart lives in `presence.py`
  (ROS-free, unit-tested offline: `pixi run python -m pytest src/behavior/test`); the node
  maps topics→events. No-op if sismic is missing or `behavior.enable:=false`.
  - **`mood_node` is thin ROS glue; ALL the ROS-free thinking is in `brain.py`** —
    mirroring how `nanobot_brain.cognition.CognitionCore` factored the LLM side. `brain.py` is the
    single behaviour-layer "brain" module: the **Purpose Engine** (objective + intrinsic-reward
    weights, deterministic reflection — `default/merge/reflect_purpose`), the **Pursuit** driver
    + A/B **bandit** (`OBJECTIVES`/`precond_ok`/`Pursuit`/`Bandit`), and the orchestration —
    `PurposeBrain` (beat-upgrade decisions, reflect, reward, reflection mode, persist) +
    `Personality` (chart-context traits/evolution: seed/evolve/heartbeat/persist). (The Purpose
    Engine + Pursuit used to be separate `purpose.py`/`planner.py`; folded into `brain.py` to
    keep the behaviour layer to three files — `brain.py` + `presence.py` + `mood_node.py`.) Both
    classes announce state through injected adapters and run identically on the dev harness
    (`scripts/dev_webui.py`) — one base, not a robot/dev copy. Unit-tested offline in
    `test/test_brain.py` (+ `test_purpose.py`/`test_planner.py`, which now import from
    `behavior.brain`). See [[llm-openrouter-personality]].
  - **The chart is also the single brain for autonomous LLM expression — and the idle mix is
    dynamic + self-learning.** Each idle cycle the chart enters ONE `performing` state that asks
    the injected `pick_beat()` (pure `choose_beat` in `presence.py`) to choose a beat by a
    **priority-weighted, novelty-aware, trait-gated lottery** over the *enabled* registry beats:
    `musing` (sensors), `looking` (camera), `wondering` (a deep-question musing), `listening`
    (reacts to the mic). Each beat's `priority` is its base weight and is **evolvable** (LLM
    reflection nudges it), an optional `trait` scales the weight by a live personality axis, and
    the most-recent beat is down-weighted (`HABITUATION`) so behaviour stays varied — so the
    robot *learns* which beats to favour and the mix shifts with mood/reward. (`look_every` is
    retired; the camera cadence is now `looking`'s learnable priority/trait.) On a beat the node
    shows the default face immediately AND (if `enrich_enable`) fires a **fire-and-forget**
    `/cognition/request` (JSON `{beat,state,prompt,camera,audio}`) that `web_control` executes
    asynchronously (LLM line + optional camera/mic + mood). A slow/absent LLM = a silent
    face-beat; the chart never waits. **Add a beat = one `BEATS` row + one `DEFAULT_REGISTRY`
    row** (face/camera/audio/prompt + priority/needs/trait); no chart surgery. Both the beat
    templates and the chart itself are also **hand-editable without touching code**: `BEATS` is
    layered with an optional `memory/beats.json` (`presence.merge_beats`, robot-side
    `beats_path` param) and the Sismic graph itself can be overridden with
    `memory/presence_chart.yaml` (`presence.load_chart_yaml`/`_build_statechart`, robot-side
    `chart_path` param) — either falls back to the bundled Python default if absent or broken,
    so an edit can never take the presence layer offline. `scripts/export_statechart_puml.py`
    renders whichever chart is active.
  - **Skill beat (capability library).** Every `skill_every`-th body (`musing`) beat is
    upgraded — like `pursuing` — into a **`skill` beat** (`mood_node._deliver_skill_beat`,
    gated by `skills_enable`): a fire-and-forget `{beat:"skill",state:"acting"}` request that
    `web_control` executes by **picking a capability** from the skill library and performing
    it. Goals (`pursuing`) take the `musing` slot first, then skills, else the chosen beat
    (musing/looking/wondering/listening). See the skill-library note under `web_control` and
    [[skill-library]].
  - **Parametric personality + evolution.** `traits` (curiosity/extraversion/caution/
    playfulness, 0..1) + a `registry` (per-beat priority/enable/needs/trait for
    musing/looking/wondering/listening) live as mutable
    dicts in the Sismic context, seeded from `personality.json` (made by
    `scripts/personality_creator.py`, persisted as they drift). Guards read them (curiosity
    gates the camera beat; extraversion scales the idle cadence; registry can demote beats),
    they're folded into the cognition prompt, and `mood_node` publishes them latched on
    `/cognition/traits` (expression-level influence only — the old slam_nav
    `caution`→stop_distance/max_lin mapping went away with slam_nav). Evolution
    is event-driven + smoothed: an `evolve` event (exponential smoothing, internal transition)
    from **fast rules** (pickup→caution, in mood_node) OR **slow LLM reflection**
    (`web_control`, pro model reads the decision log on `reflect_period` + on events →
    `/cognition/evolve`). A `brain_lost` heartbeat (`brain_timeout` with no evolve) reverts to
    the **seeded baseline** (not generic defaults). **INVARIANT: `brain_timeout` MUST stay well
    above `reflect_period`** — it's a process-death failsafe, and if it's shorter than the gap
    between reflections the chart reverts accumulated drift during normal quiet, so the robot can
    never "become its own" (this bit us once: 90 < 600). Reflexes (`greeting`/`resting`/`dormant`/
    pickup) are NOT in the registry, so the brain can never disable them. See the
    llm-openrouter-personality memory.
  - **LLM-steerable `drives` (new expressive axes + new chart states).** Beyond the 4 traits, a
    third Sismic-context dict `drives` gives the LLM *more kinds* of influence (not just weights):
    `energy`/`focus`/`introspection` (0..1) + a categorical `mood` face. They ride the **same
    `evolve` event** as traits (same guardrails: clamped, smoothed by `smoothing_alpha`, reverted
    on `brain_lost`, seeded from + persisted to `personality.json`) and are **expression-only**.
    Each drives NEW chart structure: `energy`→idle cadence + an *energetic burst* (`performing`
    self-loops to chain a 2nd beat); `focus`→a brief alert **`attending`** perk-up state before a
    beat (`attend_face`/`attend_secs`); `mood`→a **`feeling`** state that wears the face between
    beats (`feel_secs`); `introspection`→scales `reflect_auto_idle` in mood_node. **0.5 is the
    neutral "off" point** (`drive_prob`): at default the new states never fire, so behaviour is
    unchanged until the LLM pushes a drive >0.5. The post-beat / perk-up choice is decided ONCE on
    state entry (where the rng is rolled) so the competing eventless guards stay mutually exclusive
    (Sismic errors on simultaneously-enabled non-orthogonal transitions). `cognition.reflect` may
    propose `drives`; `mood_node`/`dev_webui` carry them through evolve; `robot.yaml` has
    `attend_face`/`attend_secs`/`feel_secs`. Regenerate the chart diagram with
    `scripts/export_statechart_puml.py` (→ `docs/presence.puml`).
  - **Time awareness.** The chart's idle cadence is multiplied by a live `tempo()`
    callable (injected by `mood_node._tempo`; re-read on every guard evaluation): inside
    the `behavior.quiet_start`/`quiet_end` window it returns `night_tempo` (2.0 = beats
    fire half as often), so the robot is naturally sleepier after hours — without touching
    the LLM-owned traits/drives. The matching SPEECH muting lives in the cognition core
    (web_control `quiet_start`/`quiet_end` — **keep the two yaml windows in sync**):
    autonomous speech (beats, skill beats, boot greeting, offline line, stats announcer,
    reflection bookends) is silenced and logged as `quiet-hours`; user-initiated speech
    (chat/say/observe/look, POST /tts, a manually invoked skill) always talks. Faces still
    animate at night — quiet, not dormant. `cognition.time_context()` ("It is Tuesday
    21:47, in the evening.") is folded into the beat/skill-pick/observe prompts so lines
    fit the moment. Helpers (`daypart`/`in_quiet_hours`) are pure + unit-tested
    (`test_time_awareness.py`, `test_tempo.py`).
- `sensor_hub` — **runs `imu_driver` + `sys_monitor` + `wheel_odometry` + `lds_driver_py`
  in ONE process** (one executor) to save ~100+ MB RAM on the 1 GB board. Same node
  names/topics/params/services — purely an packaging change. Trade-off: they no longer
  crash/restart independently.
- `app_hub` — the same move for the expression/cognition layer: **runs `web_control` +
  `oled_display` + `behavior` (mood_node) in ONE process**. The board now runs exactly
  **four fault domains** — `sensor_hub` (the body), the `nav2_container`
  (spatial/nav: planner + controller + bt_navigator + behaviors + lifecycle manager in
  ONE `component_container_isolated`), `slam_toolbox` (`nano-slam`, plain node, own
  process — `/map` + `map→odom` TF), `app_hub` (expression/web/brain) — plus the zenoh
  router and the one-shot `nano-nav-loader` that attaches the components.
  app_hub's main also preserves the OLED SIGTERM end-screen (restart/shutdown glyph).
  It also registers an in-process **SIGUSR1 faulthandler** (`_install_stackdump`,
  2026-09-21): `kill -USR1 <pid>` dumps EVERY thread's stack to stderr → journald,
  no ptrace/root needed — the executor-stall diagnosis (the persistent
  `nano-stall-trap.service` → `scripts/stall_trap.sh` signals it on a ≥5 s D-state;
  read `journalctl -u nano-app`). Verified live 2026-09-21. **The OLED's panel
  I2C runs on a dedicated worker thread** (`oled_display` bounded drop-oldest render
  queue, 2026-09-22): the executor side only submits, so a wedged I2C bus (the
  `mv64xxx` ≥5 s D-state that froze every executor callback under load, found live
  2026-09-22) costs a stale panel — never the executor. `shutdown_sequence` renders
  the end-screen inline after stopping that worker.

### ESP32 motor/encoder coprocessor (`firmware/nanobot_coprocessor/`)
- **Native zenoh-pico over a direct UART link** (PlatformIO + Arduino) — NO micro-ROS,
  NO Fast-DDS, no agent. It joins the SBC's `rmw_zenoh` graph directly, emitting
  rmw_zenoh's exact wire format + liveliness tokens (see the `src/main.cpp` header).
  Subscribes `/cmd_vel` (geometry_msgs/Twist → diff-drive → H-bridge LEDC PWM), `/led`
  (Bool, onboard-LED pipeline test), `/lds_target_rpm` (Float32 PID setpoint), `/fan_pwm`
  (Float32 0..1 → SBC cooling-fan LEDC PWM; published by `sys_monitor` from the CPU-temp
  curve, web-overridable), `/motor_trim` (Float32 manual straight-line trim set/reset —
  see below), `/motor_pid` (Float32MultiArray `[kp,ki,kd]` — LIVE wheel-PID gains, see
  the closed-loop note below; **the web Coprocessor card's PID sliders drive it**, so
   tuning needs no reflash). **All subscriptions live in ONE table** (`SUBS` in
   main.cpp), declared once at boot. The 45 s periodic undeclare+re-declare
   (`SUB_REDECLARE_MS`) built for the 2026-09-20 `/cmd_vel`-deaf bug is **DISABLED
   (`0` — compiled out; `subsRedeclare()` kept for manual reuse)**: it failed its
   2026-09-21 pm verify (the burst re-declared every 45 s over a half-dead session
   without reviving delivery) and load-correlated ESP drops kept landing ON redeclare
   moments. The reliable heal is a `nano-robot.target` bounce (the ESP's ping-watchdog
   reboots it → fresh session both ends); `/esp32_reset` (@1 Hz, `esp_reset_reason()`)
   triages any drop remotely. Reproduction/diagnosis: `scripts/cmd_vel_deaf_test.py`
   — see the gotcha below. **The fan is parked (0 duty) whenever the SBC link isn't alive** — boot race,
  a dropped link, or the SBC genuinely powered off — same `linkAlive()`-gated treatment as
  the LDS spin-motor park below; there's no SBC heat to move if the SBC isn't running, and
  it resumes the instant `sys_monitor` reconnects (2026-07-15 fix — it used to hold its last
  commanded duty forever on link loss, so the fan kept running after a clean SBC shutdown).
  Publishes
  `/wheel_ticks` (Int64MultiArray `[L,R]`) from **single-channel** rising-edge GPIO-
  interrupt counts (**signed by commanded direction** — the encoders have no 2nd channel,
  so the ISR signs each tick by the last `/cmd_vel` wheel direction),   `/left_wheel_suspended` +
  `/right_wheel_suspended` (Bool per-wheel off-ground microswitch, **published on change**
  for low latency + a 1 Hz heartbeat republish; **`true` = the wheel is UP / lifted, the
  robot is suspended — `SUSPEND_ACTIVE_HIGH` is `true` (2026-07-16 flip): the switch reads
  HIGH (INPUT_PULLUP) while lifted, LOW while on the ground.** The SBC consumers —
  mood_node pickup reflex, web_control snapshot — all honor a **latched
  `/pickup_override` test hook** (Int8: -1 auto, 0 force-grounded, 1 force-lifted; ESP32
  1 Hz heartbeat makes overriding at the source impossible), set from the web
  Coprocessor card, auto-cleared on page reload), `/esp32_temp` (Float32) + `/esp32_hall`
  (Int32) on-die telemetry, and `/esp32_heartbeat` (Int32). Also reads a **spin-lidar**
  (LDS02RR) → `/lds_rpm` (Float32, RPM only — scan data ignored; 0 when stale) + `/lds_hz`
  (valid-frame rate, 0 = not receiving),   and closed-loop-controls its spin motor: a PID
  (hardware tuning tracked in `docs/TODO.md`) holds `/lds_target_rpm` by driving the motor PWM, output on
  `/lds_duty`. The LDS path is gated by `LDS_ENABLED` (currently 1; UART1 is drained once
  per PID tick, not every loop, since only the RPM is needed). WiFi/BT kept off.
  **The setpoint is OWNED by web_control's idle controller** (see the LDS idle spin-down
  section) — the firmware is a clamp-holding follower (NaN-reject + 0..400 clamp on
  `/lds_target_rpm`, default 300 at boot, corrected within ≤30 s by the SBC re-assert).
  **Jam guard** (`ldsControl`, 2026-09-21): rpm < 0.4×target or stale UART1 tach for
  6 s while a target is set ⇒ latched motor park (`/lds_jam` Bool @5 Hz), cleared only
  on target ≤ 0 — a blocked rotor can't cook the motor; see the LDS idle spin-down section.
- **Line lasers**: subscribes `/laser_pwm`
  (`std_msgs/Int32MultiArray [v1,v2]`, each 0..255) and drives two line-laser PWM
  outputs on **GPIO 23/32** (`LASER1..2_PIN`). 10-bit duty =
  `value*1023/255`. The web "Line lasers" card's two sliders POST `/laser_pwm` via
  `telemetry.py`'s `_mk_laser` whitelist builder. Publish-only (like `/motor_accel`) —
  no read-back. Lasers park at 0 while the SBC link isn't alive (same
  `linkAlive()`-gated park as the fan/LDS) and zero the setpoints so they resume at 0,
  not the stale pre-drop value. **Laser 3 was removed 2026-08-18**: its GPIO13 stayed
  stuck full-on through every PWM peripheral tried (LEDC low-speed ch 8 stuck it high
  silently — every `ledcWrite()` incl. 0 drove the pin high; MCPWM couldn't sink it
  either), so the laser was hardware-controlled, not `/laser_pwm`-controlled, and it's
  gone from both firmware and the web UI. GPIO13 + LEDC ch 8 are now untouched.
- **Bad-encoder-signal diagnostic (2026-07-15, built; flashed 2026-09-20)**: a
  per-wheel `/wheel_stray_ticks` (Int64MultiArray `[L,R]`, same cadence as `/wheel_ticks`)
  counts ISR ticks that land while that wheel is commanded **and settled** (`STRAY_SETTLE_MS`
  = 300 ms coast-down grace period after duty→0) stopped — a real coast-down tick isn't
  noise, but anything after that settle window can only be electrical noise/ground-bounce
  on the encoder GPIO (relevant given the earlier [[esp32-hardware-fried-ground-fix]]
  ground-bounce failure). Cheap bool check in the ISR, no FPU. Web Coprocessor card shows
  it (red if nonzero) with a **🔁 Reset ticks** button (`/reset_ticks` Bool) that zeros
  both `/wheel_ticks` and `/wheel_stray_ticks` on the ESP32; `wheel_odometry` also watches
  `/reset_ticks` and re-seeds its prev-tick baseline (`_have_ticks=False`) so `/odom`
  doesn't see a fake huge jump when the raw counters reset. **2026-09-23: the counter has
  a BLIND SPOT — phantom ticks also occur DURING commanded motion** (measured live: the
  L channel raced up to 8.4× its command in sustained bursts gated by motor DRIVE, idle
  perfectly clean — PWM/ground coupling into the encoder ISR; see docs/TODO.md for the
  `frame_record.py` evidence and the debounce/excess-rate-guard candidates).
- **Straight-line trim (open-loop rebalance)**: the mismatched gearmotors are rebalanced
  by a single trim factor in `applyMotors` (`l*=(1-t)`, `r*=(1+t)`; **negative = robot was
  pulling left** — boost left / cut right — because the robot currently veers LEFT).
  **`TRIM_DEFAULT = -0.10`** (main.cpp, 2026-07-16) is the NVS fallback; **`TRIM_AUTOCAL` is
  re-enabled (`TRIM_AUTOCAL 1`, 2026-09-17)** — the 2026-07-16 wrong-way convergence ran
  under the old inverted-polarity gate (it only passed while the robot was LIFTED); with
  `SUSPEND_ACTIVE_HIGH true` verified, the gate means "both wheels on the ground" and the
  loop math is sound negative feedback. Adaptation result persists to ESP32 NVS (survives
  reboot/reflash; written only while stopped, rate-limited). Manual
  set/reset live via **`/motor_trim`** (Float32, 0 = reset) — the web Coprocessor card has
  a **Wheel trim** slider (`±0.30`) that POSTs it and re-seeds from the live `/wheel_trim`
  @1 Hz value; the slider's "Reset trim to 0" button clears it. Tunables `TRIM_*` in
  `main.cpp`; compiled out if `WHEEL_PID_ENABLED`.
- **Motor control is now CLOSED-LOOP: per-wheel velocity PID (2026-09-20, flashed +
  live on the robot).** `WHEEL_PID_ENABLED 1`: each
  wheel's commanded linear speed (m/s) is held by a feedforward+PI(D) on encoder-tick
  velocity at `WHEEL_PID_HZ` (50) — the standard ROS 2 control shape (`/cmd_vel` is a
  SETPOINT refresh, the fixed-rate loop owns the dynamics deterministically regardless
  of SBC load). This SUPERSEDES the whole open-loop band-aid stack: the stiction
  `MOTOR_MIN_DUTY` remap is gone (`writeSide` is linear now — only `MOTOR_DEADZONE`
  zeroes an intended stop), and the breakaway kick/push state machine is compiled out
  (both kept verbatim behind `#if !WHEEL_PID_ENABLED` as the legacy fallback — that
  path still compiles). Why closed-loop: the 2026-09-19/20 drive tests proved
  breakaway is physically unpredictable (wheels seized 1.4-2.4 s into every crawl at
  remapped duty 0.60-0.83; then the flashed 80 ms kick juddered 1-2 s before a lurch;
  a pulse can't sustain torque past static friction) — the I-term integrates through
  stiction instead of gambling on it. **Accel limiting moved to a SETPOINT slew**
  (`WHEEL_TGT_SLEW` m/s per s, applied to the per-wheel target before the PID):
  slewing the PID OUTPUT would add loop lag + integral windup, so the old duty slew +
  `/motor_accel` live knob are compiled out under the PID (the web Coprocessor card's
  dead "Accel ramp" slider was replaced by the live PID KP/KI/KD sliders,
  2026-09-20). **The gains are LIVE-TUNABLE — no reflash per iteration**: publish
  `/motor_pid` (Float32MultiArray `[kp,ki,kd]`, whitelisted via `telemetry.py`'s
  `_mk_motor_pid`; firmware clamps kp 0..20 / ki 0..100 / kd 0..5), the running PID
  picks them up instantly (integrators reset), the web Coprocessor card's PID sliders
  re-seed from the 1 Hz `/wheel_pid` readback (`f.esp.wheel_pid`), and gains persist
  to ESP32 NVS rate-limited **while parked**.
  **Tuned gains (2026-09-21, live sweep via `scripts/pid_tune.py` —
  see the "PID retune prep" note in docs/TODO.md): KP 5.0 / KI 60.0 / KD 0**, NVS-
  persisted (`kp`/`ki`/`kd` keys, rate-limited while parked — verify they survived the
  next reboot). The sweep walked the ladder 0.05/0.08/0.10/0.15 m/s ×3+ repeats per
  config over {baseline 1.1/46, KP 3-5 × KI 45-80}: baseline hunted everywhere
  (crawl p2p up to 0.05 m/s), KI 60 alone amplified the mid-speed limit cycle, KP is
  the damper. KP 5 + KI 60 won the aggregate (crawl p2p avg 0.009 m/s vs 0.017;
  0.08/0.10/0.15 rungs clean-to-marginal). Residual carpet-stiction limit-cycling is
  band- and run-dependent (single-rung verdicts flip run-to-run — judge aggregates;
  `pid_tune.py --repeat N` exists for exactly this). **GOTCHA — the tuning instrument
  itself lied until 2026-09-22: after a gateway POST stall the backlogged SSE frames
  arrive in one BURST, and pid_tune divided real motion by collapsed PARSE-time dt →
  speeds inflated ~6-10× → phantom HUNT / "spin overspeed" verdicts (measured live:
  spin legs "0.43 m/s vs 0.041 commanded", gone on re-run). The telemetry frame now
  carries a build stamp `"t"` (telemetry.py `_build`, additive key) and pid_tune
  scores against it (parse-dt floored at `BURST_MIN_DT` 0.15 s for stampless
  gateways). Corollary: a run's verdicts are only trustworthy when its frame gaps
  stayed small — a deadman/stall in the same run can also mean the numbers are burst
  garbage; re-run before concluding anything about gains. **SECOND phantom mechanism
  (2026-09-23): encoder-noise bursts DURING commanded motion inflate legs for real**
  (the L channel raced to 8.4× its command, sustained — idle clean, see the stray-tick
  bullet + docs/TODO.md), so a bad leg is either instrument burst OR plant noise:
  run `frame_record.py` alongside any decisive tuning session and check the recording
  before trusting HUNT/SAG verdicts.** (Same session's real
  findings: the live NVS had drifted to KP 0.7/KI 19.4/KFF 5.95 — the user's
  feedforward-only experiments, third drift occurrence — and restoring 5/60/0 +
  slew 1.5 + vhyst 0.15 + KFF auto re-verified clean: crawl p2p 0.005-0.029, mid
  0.019-0.028, spins 0.028-0.038 of 0.041 across 16 legs. KFF 5.95 saturates duty at
  ≥0.17 m/s — the loop can brake but not push; KI <30 can't break away. POST stalls
  still deadman'd 2 legs even with the scan-poll fix deployed — delivery, not gains.)
  **2026-09-21 smoothness pass
  (flashed + deployed same day)** — five structural fixes in the PID block, no gain
  changes: (1) **parked-at-zero integrator bleed** — the web keepalive re-asserts `{0,0}`
  forever, so the cmd never goes stale and the dead-man never resets the integrators; a
  stop left `integ` wound NEGATIVE (braking unwinds it below zero), holding a small
  REVERSE duty on the parked wheel (rollback nudge at every stop, asymmetric lurch on
  the next start); once the slewed setpoint is ~0 AND the wheel measured stopped, PID
  state drops to zero (flat-floor contract: on a slope the reset lets the robot creep
  until the I-term rebuilds); (2) **direction-flip reset** — single-channel ticks are
  signed by COMMANDED direction, so on a reverse command the still-forward-rolling wheel
  read as already moving backwards and `kp*err` drove the OLD direction at near-full
  duty until friction stalled the wheel (pause → lurch into reverse); a commanded sign
  flip now zeroes that wheel's PID state (rebuilds from feedforward); (3) **`WHEEL_VEL_FILT 1`**
  — the PID feedback averages the tick delta over TWO 20 ms windows (40 ms): one tick
  per window was a 0.042 m/s quantization step = 83% of a 0.05 m/s crawl setpoint, so
  kp injected ±0.1 duty ripple (stick-slip excitation); halved for ~20 ms extra lag;
  (4) the integral clamp is now **1/ki (live-gain aware)** — the fixed
  `WHEEL_INTEG_MAX=1.0` was sized for ki≈8 and let one wound integrator hold ±60 duty of
  authority at the tuned ki=60; (5) **fake-velocity guards**: a boot baseline seed (the
  first PID tick after boot used a ticks-from-0 delta) and a `/reset_ticks` jump guard
  (>3 m/s equivalent delta = counter reset, re-seed, no lurch). Also the
  `WHEEL_SEPARATION` define fallback is now 0.102 (was the 0.16 guess — NVS masked it;
  an erased flash would have booted the wrong geometry + wrong KFF). **Permanent
  hardware fact (2026-09-21, user-decided): the encoders are and will stay
  SINGLE-CHANNEL** — ticks signed by COMMANDED direction, blind on
  reverse-through-zero/stall/slip/being-pushed. There is no 2nd channel to wire and
  never will be — the software mitigations (commanded-direction signing, the
  direction-flip PID reset + ring zero, the stiction-aware I-term) are the FINAL
  design, not a stopgap. Also `MAX_ANGULAR_SPEED` synced 3.0 → 0.8 (robot.yaml `drive_max_ang`,
  SLAM rotation-smear budget — firmware backstop now matches). Trim: `TRIM_AUTOCAL`
  is compiled out under the PID (per-wheel control equalizes the wheels itself) — the
  manual `/motor_trim` offset still applies and the loop absorbs it. NOTE 2026-09-20:
  a wheel pressed against an obstacle holds full duty (integral clamped, no backoff
  anymore — the dead-man still cuts on command loss; DRV8871 `nFAULT` wiring remains
  the proper hardware fix), and manual-driving feel on this robot also depends on the
  web gateway's intermittent 1-9 s POST stalls (dead-man cuts mid-drive → stop →
  lurch on recovery) — the /proc-based stall trap is now a PERSISTENT unit
  (`nano-stall-trap.service` → `scripts/stall_trap.sh`, 5 s D-state → /proc snapshot
  + SIGUSR1), and since 2026-09-21 an in-process SIGUSR1 faulthandler dumps every
  app_hub thread's stack (see docs/TODO.md);
  the keepalive half of that problem is fixed (see the HTTP teleop note below).
  **2026-09-21 smoothness pass II (FLASHED + VERIFIED 2026-09-21 pm)** — two more
  structural changes in the PID block, plus a tuning-harness `outback` mode:
  (6) **ADAPTIVE velocity filter (`WHEEL_VEL_FILT_MAX 8` / `WHEEL_VEL_QUANT 0.25`)**
  replaces the fixed 2-window average: per-window tick deltas go into a ring and the
  velocity is the average of the last N windows, N re-picked EVERY tick so the 1-tick
  quantization step (1/(N·dt) m/s) stays ≤0.25× the wheel's moving setpoint — crawl
  (0.05 m/s) gets N=4 (step 0.010 vs the fixed-2 0.021), turn wheels (±w·0.051 m/s)
  get N=5-8, fast cruise keeps N=2 (lag still matters there). The ring is zeroed on the
  jump guard AND on a commanded-direction flip (its entries carry the OLD sign
  convention). This is the anti-chatter fix: kp injects duty ripple ∝ kp×step, and the
  step now scales WITH the setpoint instead of being a fixed 0.021 m/s floor.
  (7) **Stiction-aware I-term (`WHEEL_STUCK_FRAC 0.30` / `WHEEL_I_WIND_RATE 1.2
  duty/s`)**: while a wheel is commanded but |meas| < 0.30×|tgt_s| (fighting static
  friction), the I-CONTRIBUTION (ki·integ, duty units) may move at most 1.2 duty/s —
  the tuned ki=60 turns a 0.05 m/s crawl error into 3 duty/s of push, a hammer that
  breaks away violently, overshoots, re-sticks (the stick-slip limit cycle); a rate-
  capped ramp breaks away firmly instead. Integration is UNRESTRICTED while tracking or
  braking (stuck=false there — |meas| ≥ 0.30×|tgt|), so stopping/normal control are
  untouched; conditional integration + the 1/ki clamp still apply. No new state (the
  cap recomputes from ki·integ inside `wheelPid`, which gained a `stuck` arg).
  **`scripts/pid_tune.py outback`** — the in-place test mode (fwd leg → 1.2 s settle →
  reverse leg, optional alternating ±spin legs; the robot nets ~zero travel, walls stay
  far away): per-leg mean/p2p/stall + **breakaway seconds** (first 3 ticks after the leg
  command, ~0.2 s SSE resolution) + **distance** (fwd ≈ |rev| symmetry check) + a
  **freeze/deadman detector** (both-wheels-frozen ≥0.4 s while commanded = the 500 ms cmd
  watchdog's stop+re-breakaway signature — an input-delivery failure the controller can
  never tune away; controller-level stick-slip never fully freezes).
  **2026-09-21 trace diagnosis (ticktrace probe, live robot): the residual stutter is a
  PLANT-level stick-slip limit cycle, not dead-man resets and not gains.** A 15 s crawl
  at 0.05 m/s (and 8 s at 0.12) shows NO mid-leg freezes (serial/dead-man ruled out at
  the 3.3 Hz keepalive) but rhythmic speed dips (12 → 7-8 ticks per 0.2 s SSE frame) in
  0.4-0.6 s clusters every ~1 s, both wheels together, at BOTH speed bands — the wheel
  seizes between I-term surges as carpet static friction re-engages. A/B'ing KI 60 → 30
  live changed the shape, not the amplitude (and stretched breakaway past 1 s) — so
  gains are not the fix. **The pass-II answer at the plant level is a stiction DITHER**
  (`WHEEL_DITHER 0.05` duty, `WHEEL_DITHER_IN 0.02` / `WHEEL_DITHER_FADE 0.15` m/s
  gates, toggled every `WHEEL_DITHER_TICKS 2` PID windows ≈12.5 Hz): a small alternating
  duty keeps the gear mesh micro-moving so static friction never re-engages; added
  OUTSIDE the PID (the integrator never sees it), zero when parked or cmd-stale.
  Live-tunable via **/motor_params id 6** (0..0.2, 0 = off, NVS key "dith", readback in
  /wheel_params — so the amplitude is tunable from the web Coprocessor card's future
  slider or `pid_tune.py params --set 6=0.05` without a reflash). **VERDICT (2026-09-21
  pm A/B on hardware): the dither was COUNTERPRODUCTIVE — at dither 0.05 the crawl p2p
  read 0.016-0.018 (pure injected ripple) vs 0.007 with it OFF, and spins showed no
  benefit either; the setting is now id 6 = 0** (the mechanism stays compiled-in and
  live-tunable for future experiments; the adaptive filter + rate-limited I alone beat
  both the baseline and the dither).
  **PASS-II VERIFIED NUMBERS (in-place outback suite, dither off): crawl 0.05 m/s
  p2p 0.007 (~2× better than the pre-pass-II baseline 0.011-0.013), instant clean
  breakaway; 0.12 m/s p2p 0.018-0.037 (baseline's worst rung 0.060 gone); 10+ fwd→rev
  transitions with ZERO lunge events → the linear lunge guard was retired (maxlin
  restored 0.4; the flip-stale-ring zero held); no ESP drops across the whole suite.**
  REMAINING (flash pending): spin-band SAG — mean 0.024-0.034 vs 0.041 at ±0.8 rad/s =
  the rate-limited I recovering spin-band stick-slip slowly, so **`WHEEL_I_WIND_RATE`
  was raised 1.2 → 2.5 duty/s** (built; re-verify spins after the next flash).
  **2026-09-21 pm III — the 2.5 rate is FLASHED + VERIFIED: spin tracking tightened
  (0.65 rad/s p2p 0.027 → 0.015, mean 85 → 88%; 0.8 rad/s best leg 95% of target),
  aggregate spin mean 0.029-0.039 vs 0.024-0.034 at rate 1.2. The spin band remains
  partly stiction-bound (single-channel ticks + carpet at ±0.04 m/s per-wheel — a
  PERMANENT accepted limit: there is no 2nd encoder channel and never will be). The canned-turn speed is capped by
  move_ang_speed 0.8 (the smear budget); a 90° canned turn measured 90.2° in 3.56 s.**
  **2026-09-21 pm IV — the turn ceiling was raised 0.8 → 1.0 rad/s at the user's
  "still slow" report (firmware maxang id 4 LIVE via /motor_params + robot.yaml
  drive_max_ang/move_ang_speed + MOVE_ANG_RANGE (0.10, 1.00) + the Drive-card
  slider max). 90° canned turn: 4.09 s → 3.52 s (peak rate rides the 1.0 cap; the
  remaining time is wheel breakaway + the P-taper tail — stiction-bound). Smear
  trade: 1.0 rad/s = 11.5°/scan vs the 9.2 at 0.8 (slam_toolbox 2.6.10 has no
  deskew) — reversible via the same knobs; watch map quality after heavy spin
  use. NOTE the board's persisted ~/.local/state/nanobot/move.json WINS over
  robot.yaml defaults — the live bump went through POST /move/config.**
  **Web-side pressure fix (same session): the page's /scan.bin poll ran every 80 ms on
  the lidar hero view (the DEFAULT view while driving) = 12.5 fetches/s against ~5 Hz
  data — 60% no-op fetches churning the gateway. Now 200 ms (5 Hz = the data rate) with
  an overlap guard (`scanBusy`) so a slow fetch can't pile up concurrent fetches; the
  off-view header refresh stays ~1 Hz. This is a candidate contributor to the
  intermittent 1-9 s POST stalls felt as manual-drive stutter/lag.**
- **GOTCHA — the ESP32 NVS gains/params DRIFT (found 2026-09-21): the documented "tuned"
  values are only true if the LAST tuning session parked cleanly.** A `gains --set`/
  `params --set` writes NVS rate-limited while parked — an abandoned session leaves
  whatever it last set. The board was live on **KP 6.8 / KI 10.0 / maxlin 0.15 /
  maxang 0.3** (an old session) — i.e. the firmware was clamping every drive to 0.15 m/s
  and every turn to 0.3 rad/s, the spin legs wouldn't break away at all (KI 10 vs the
  needed 60), and "turning is slow" was mostly THIS, not the web slider. FIRST check the
  readback before diagnosing smoothness: `pid_tune.py state` (wheel_pid + wheel_params
  ids 3/4 must read 5/60/0 + 0.4/0.8). Restored live via `gains --set 5,60,0` +
  `params --set 3=0.4,4=0.8` (2026-09-21; the rate-limited while-parked save re-persists
  them). **Hit AGAIN 2026-09-22** (found during the KFF deploy): the board was live on
  **KP 1.0 / KI 0.0** (feedforward-only experiment leftovers) — restored 5/60/0 the same
  way; the check must be the FIRST step of every tuning session, no exceptions. The canned-turn default was also raised **`move_ang_speed` 0.5 → 0.8**
  (web_server.py + robot.yaml + the Drive-card slider placeholder — 0.8 = the accepted
  w·0.2 rad/scan smear ceiling, same as drive_max_ang, and doubles the turn wheels'
  per-wheel speed out of the stickiest PID regime); NOTE the board's persisted
  `~/.local/state/nanobot/move.json` WINS over robot.yaml defaults on boot — the live
  value was bumped to 0.8 via POST /move/config (persists there too). A 90° canned turn
  measured 90.2° physical in 3.56 s after the change (turntest probe).
- **The wheel-encoder scale was 5.7× WRONG (found + fixed 2026-09-20, live-verified
  by rollout).** The PID's first tuning session exposed it: "0.06 m/s" cruises with
  duty pinned at 1.0 are impossible for this drivetrain — a user-measured rollout
  (2235 ticks / 186 cm) gave **1202 ticks/m ⇒ `TICKS_PER_REV` 253, not 1440** (the
  1440 was a quad-vs-single-channel counting assumption; single-channel rising-edge
  counts ~253/rev). Physical full-duty cruise ≈ 0.37 m/s loaded vs the 0.464
  no-load figure — the drive hardware was healthy all along; `/odom`, the PID's
  velocity feedback and `WHEEL_KFF`'s "full scale" were all in fictional units, and
  every pre-fix commanded speed saturated into a ~0.37 m/s lurch. Corrected in
  `main.cpp` (`TICKS_PER_REV 253`) + `robot.yaml` (`wheel_odometry.ticks_per_rev`
  AND `sim_bridge.ticks_per_rev`, which MUST stay in lockstep). Consequences: the
  pre-fix SLAM maps were built on a 5.7×-scaled odom (garbage geometry — rebuilt);
  Nav2/SLAM real-unit parameters (max vel, costmap radii, minimum_travel_*) only
  NOW mean what they say. Straightness note: the big left-pull during tuning was
  the stale `wheel_trim` 0.106 (zeroed; under closed-loop the per-wheel loops
  equalize, so trim only matters at saturation) — residual ~3% L/R tick imbalance
  at saturation is expected motor variance and should vanish in unsaturated
  operation; if a consistent pull remains after the scale fix, suspect per-wheel
  encoder bias (candidate fix: a firmware straight-line differential assist, NOT
  yet implemented).
- **The wheel SEPARATION was ~1.57× WRONG (found + fixed 2026-09-21, lidar-verified).**
  Both configs carried `wheel_separation 0.16` — a chassis-width guess, never
  measured (same failure class as the 2026-09-20 scale error). Symptom: web canned
  rotations overshot 1.5-1.62× (115°→~180°, 60°→~90° as seen from the seat) while
  DISTANCE stayed exact — odom yaw = diff-travel/sep, so a too-large sep compresses
  yaw only. Ground truth: cross-correlating two `/scan.bin` range profiles before/
  after a canned 90° turn (the lidar is independent of wheels AND of the SLAM
  pose's odom prior) measured 146°/139°/144° physical ⇒ true track ≈ **0.102 m**;
  set in firmware (live `/motor_params` id 2, NVS) + `robot.yaml`
  (`wheel_odometry.wheel_separation`; the board's install config symlinks through to
  src — rsync the file + restart the stack). Post-fix verification, same lidar
  method: 90° request → **89°** physical, 60° → **57°**, drive 0.3 m → 0.302 m with
  ~2° drift. Bonus: `/odom` yaw is now in true units, so slam_toolbox's motion
  prior and Nav2's rotation handling no longer fight a compressed heading. The
  pre-fix "90.2°" canned-turn verification (2026-09-21 morning) was CIRCULAR — it
  divided the tick differential by the same wrong sep; only the lidar correlation
  is a valid rotation ruler.
- **Drivetrain geometry is LIVE-TUNABLE — `/motor_params`, no reflash (2026-09-20).**
  The scale lesson generalized: `TICKS_PER_REV`, `wheel_radius`, `wheel_separation`,
  `max_linear_speed`, `max_angular_speed` and `WHEEL_TGT_SLEW` are now NVS-backed
  runtime parameters in the firmware (defaults = the `#define`s; loaded in
  `setup()`, saved rate-limited while parked exactly like the gains). Wire format:
  POST `/publish {topic:"/motor_params", value:[id,val, id,val, …]}` — Float32MultiArray
  (id,value) pairs, ids `0 ticks_per_rev · 1 wheel_radius_m · 2 wheel_separation_m ·
  3 max_linear_ms · 4 max_angular_rads · 5 target_slew · 6 dither · 7 vel_hyst (0..0.5) ·
  8 kff_override (duty per m/s; 0 = derive from ids 3/4/2)`
  (whitelisted via
  `telemetry.py`'s `_mk_motor_params`, max 9 pairs). Any accepted change recomputes
  the derived ticks/meter + KFF full-scale map and resets the PID integrators
  (their error units just changed meaning). Readback on `/wheel_params` @1 Hz in
  the SAME (id,value) layout → `f.esp.wheel_params` re-seeds any future web sliders.
  A future recalibration is a POST, never a flash. `clampf` ranges: tpr 10..5000,
  radius 0.005..0.5, separation 0.05..1.0, maxlin 0.05..2, maxang 0.05..5, slew
  0.05..10, dither 0..0.2, vel_hyst 0..0.5, kff 0..10 (values <0.1 = auto-derive).
  **Vel hyst is now web-tunable (2026-09-22)**: the Drive tab's Coprocessor card has a
  "Vel hyst" slider (0..0.5) — release publishes `[/motor_params [7, v]]` (the same
  instant-apply pattern as the PID sliders) and the slider re-seeds from
  `f.esp.wheel_params` id 7 on first frame, so the page shows the NVS-persisted value
  the firmware is actually running. **Wheel-PID KFF is web-tunable too (2026-09-22, id 8 — flashed + deployed + live-verified
  same day)**:
  a "Feed fwd KFF" slider (0..10, 0 = auto) on the same card — the manual override for the
  derived feedforward duty-per-m/s gain (auto ≈ 2.24 with the current geometry); every
  accepted change resets the PID integrators and persists to NVS ("kff" key) while parked.
  **Tgt slew is web-tunable too (2026-09-22, id 5 — deployed same day, web-only: the
  flashed firmware already accepted id 5)**: a "Tgt slew" slider (0.05..10 m/s per s,
  default 1.50) on the same card — the `WHEEL_TGT_SLEW` setpoint accel limiter (each
  wheel's speed target is ramped at this rate before the PID sees it, so a /cmd_vel step
  is a soft start, not a lurch; at crawl speeds even 1.5 reaches full speed in <0.1 s, so
  it mostly matters at higher speeds). Same instant-apply/reseed-from-`f.esp.wheel_params`
  id 5 pattern; NVS key "slew".
  The other ids stay script-only
  (`pid_tune.py params --set id=val`).
- **Tunables are `#define`s inline at the top of `src/main.cpp`** (there is no
  `include/config.h`). `include/zenoh_generic_config.h` only holds zenoh-pico feature
  flags (enables `Z_FEATURE_LINK_SERIAL`). Pins (ESP32 GPIO): encoders L=19 R=5,
  off-ground switches L=4 R=21, DRV8871 IN L=26/27 R=25/33 (fwd/rev; one DRV8871 per
  motor, no STBY/enable pin),
  onboard LED=2, **UART2 = zenoh link (TX=17, RX=16) → SBC `/dev/ttyS1`**, **LDS data on
  UART1 RX=GPIO14 (TX=GPIO13 unused)**, LDS spin-motor PWM=18, cooling-fan PWM=22 (via a
  logic-level MOSFET — the ESP can't source fan current). (SBC side: ESP32 link on
  `/dev/ttyS1`/UART1-PG6/PG7, LDS scan on `/dev/ttyS2`/UART2-PA0/PA1, OLED on
  `/dev/i2c-0`/PA11-PA12 @400kHz.) Keep diff-drive limits synced to `robot.yaml`.
- **The link needs a serial-capable `zenohd`** — the conda `libzenohc` is built without
  `transport_serial`, so stock `rmw_zenohd` can't open the UART. Build one with
  `firmware/nanobot_coprocessor/tools/build_zenohd_serial.sh {x86_64|aarch64}`; the
  `nano-router` systemd unit (via `scripts/unit_exec.sh router`) runs it on the board
  so the ESP32 (serial) and the rmw_zenoh nodes (TCP) share a graph.
  See [[robostack-zenoh-no-serial]] and [[esp32-zenoh-pico-integration]].
- Build/flash from the dev PC: `cd firmware/nanobot_coprocessor && pio run -t upload`
  (pio lives in `~/pio-venv`). **Don't build the firmware on the board.**

### Brain architecture (nanobot-brain package)
All brain logic lives in `nanobot-brain` — a **ROS-free** Python package. The robot's ROS nodes (`mood_node`, `web_server`) import from it:

```
┌─────────────────────────────────────────────────────────┐
│  mood_node.py (ROS glue)                                │
│  ┌───────────────────────────────────────────────────┐  │
│  │  nanobot_brain.behavior (ROS-free)                │  │
│  │  ┌─────────────────────────────────────────────┐  │  │
│  │  │  presence.py (Sismic statechart)             │  │  │
│  │  │  brain.py (PurposeBrain + Personality)        │  │  │
│  │  └─────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  web_server.py (ROS glue)                               │
│  ┌───────────────────────────────────────────────────┐  │
│  │  nanobot_brain.cognition (ROS-free)               │  │
│  │  ┌─────────────────────────────────────────────┐  │  │
│  │  │  core.py (CognitionCore)                    │  │  │
│  │  │  llm.py (LlmClient — OpenRouter)            │  │  │
│  │  │  skills.py (SkillLibrary)                    │  │  │
│  │  │  skillsmith.py (WorkshopState)              │  │  │
│  │  │  phrasebank.py (PhraseBank)                 │  │  │
│  │  └─────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

Platform adapters (interfaces.py):
- BrainPlatform protocol: face, capture_frame, sensor_snapshot, publish_action, etc.
- TTS protocol: say, available
- LlmProvider protocol: generate, complete, available

### LLM cognition pattern (single base)
- `nanobot_brain.cognition.CognitionCore` = ALL LLM logic (ROS-free). Shared verbatim by `web_server.py` (robot) and `scripts/dev_webui.py` (dev).
- `behavior.mood_node` = thin ROS glue; imports from `nanobot_brain.behavior`.
- `LlmClient.generate()` is blocking stdlib `urllib` (no SDK). Free-first model fallback chain.
- Key: `llm_api_key` or `$OPENROUTER_API_KEY` — never commit.
- `cognition_log_path` default `~/.local/state/nanobot/cognition.log` (survives reboot).

### Skill library
- Skills are one `.md` each (YAML frontmatter + markdown body), living in the `nanobot-brain` repo under `skills/`. Drop a file, `POST /skills/reload`. No code change.
- Resolution (`nanobot_brain.cognition.skills.resolve_skills_dir`): `skills_dir` param → `$NANOBOT_SKILLS_DIR` → **the brain repo's root `skills/`** → installed `<share>/web_control/skills` fallback. The brain `skills/` dir must be rsynced to the board next to `brain/src` (see the deploy note below).
- Two tiers: narrative (`say`/`observe`/`look`) and gated action (`topic` — whitelisted, off by default).
- Workshop (reflection mode) synthesizes new skills via LLM → `workshop_dir` (default `~/.local/state/nanobot/skills`).

### Cognition, skills & runtime internals (detail)

- **Cognition core (`cognition.py`, ROS-free).** ALL the LLM-personality *logic* — generate +
  express, the say/chat/observe/look paths, the statechart beat executor, the skill library
  invocation, the phrase bank, the decision log, slow reflection, lifecycle speech — lives in
  ONE class, `CognitionCore`, shared verbatim by `web_server.py` (robot) and `dev_webui.py`
  (dev). Each side only injects a few **adapters** (face→`/oled_face` vs print, capture_frame→
  V4L2 vs webcam, sensors→`/proc`+IMU vs synthetic, the gated action tier→whitelisted
  publishers vs no-op, persist→`llm.json` vs none) plus its own HTTP handler + ROS/sim
  plumbing. So a new cognition feature is written **once**. The node/`DevState` keep only thin
  one-line delegators for the handler. See [[llm-openrouter-personality]].
- **LLM personality (OpenRouter)** — the *client* is `llm.py` (ROS-free); the orchestration is
  `cognition.py` (above). It
  offloads "say something" / chat lines **plus the matching OLED expression** to a model
  on OpenRouter. `LlmClient.generate()` is a blocking stdlib-`urllib` POST (no SDK) that
  returns `{"say","mood"}`; the **mood is constrained to the OLED's four faces** +
  `neutral` (coerced if the model strays). **Two text tiers, each FREE-FIRST:** the cheap
  tier (everything) and the smart tier (chat + reflection, `generate(smart=True)`) each try
  one or more **free** OpenRouter models (`llm_free_model` / `llm_free_smart_model`, comma-
  separated lists) and only fall back to the **paid DeepSeek** model (`llm_model` flash /
  `llm_smart_model` pro) when *all* the free ones are rate-limited. `_candidates(smart,image)`
  builds the ordered `(model,is_paid)` list; `_chat` tries each, **falling through only on a
  rate/daily-limit error** (429/402/limit-ish msg, incl. 200-with-error bodies) — other
  failures stop. `last_model` = the slug that answered (logged). **Hourly caps apply only to
  the PAID fallback** (`llm_smart_max_per_hour` 15 / `llm_vision_max_per_hour` 10, 0=off; free
  is never capped). Vision tier is already free (`llm_vision_model`); no DeepSeek vision
  fallback (set `llm_vision_fallback_model` for a paid one). Free `:free` slugs rotate +
  get throttled → swap via OpenRouter `/models` if a default stops working. pro/reasoning
  models narrate so `llm_max_tokens` is 1024 (too low → empty JSON = no-reply).
  `LlmClient.complete(system,user,smart=,json_object=)` is a general (non-`{say,mood}`) call.
  `scripts/personality_creator.py` (ROS-free) runs a short questionnaire through the smart
  model → writes `personality.json` ({name,persona,traits,registry}) + a robot.yaml snippet.
  **`POST /llm/observe`** is sensor-aware chatter: it builds a short plain-English snapshot
  of the robot's own body — CPU/RAM/temp (`/proc`), IMU motion+tilt (`/imu/web`+`/imu/euler`),
  pick-up (`/left|right_wheel_suspended`) — and has the model comment in character on how it
  "feels" (web "👁 Observe" button). **`POST /llm/look`** is vision: it grabs one JPEG from
  the webcam (`CameraStream.add_viewer→get_frame→remove_viewer`), base64-data-URIs it as an
  `image_url` part, and routes to the **vision** model (`llm_vision_model`, default the
  credit-free `nvidia/nemotron-nano-12b-v2-vl:free` — the text model can't see) so it
  comments on what it sees (web "📷 Look" button). `generate(image_jpeg=…)` skips
  `response_format` for image requests (some multimodal models reject it). Note: many
  OpenRouter `:free` vision slugs come and go (Llama-3.2-vision is paid-only now) — pick a
  current one via OpenRouter's `/models` API if the default stops working. Endpoints: `POST
  /llm/say` (one-shot), `POST /llm/chat` (rolling history), `GET|POST /llm/config`,
  `GET /llm/log`. The web "AI" card (AI tab) drives the on-demand ones. **Autonomous
  chatter is NOT here** — it's driven by the `behavior` statechart's beats via
  `/cognition/request`, which `web_control` executes (`_on_cog`→`_run_beat`: capture frame
  if asked, append the sensor snapshot, `_generate`). The old standalone idle-chatter timer
  was retired (one brain). Best-effort: **no key / no network = silent no-op**, never on
  the critical path. All config is in `robot.yaml` (`llm_*`, and the `behavior:` beat
  knobs); the **key is read from `llm_api_key` or, when blank, `$OPENROUTER_API_KEY`,
  or — winning over both — a key pasted into the web "AI" card**. To set it up: copy
  `memory/openrouter_key.example` to `memory/openrouter_key`, replace with your real
  OpenRouter key (one line, no quotes), and the key is picked up by **every entry point**
  (`scripts/dev_webui.py`, `scripts/sim_run.sh`, `scripts/unit_exec.sh` for the systemd
  units, and all `pixi run` tasks). The LLM
  **auto-enables** when a key is detected (`web_server.py` + `dev_webui.py` override
  `llm_enabled: false` to `true`), so no web UI toggle needed on first run. UI toggles
  (enable/model ids/persona) **and now the API key itself** persist to
  `~/.local/state/nanobot/llm.json` (outside git) so they survive a reboot; the key field
  is a write-only password input — `GET /llm/config` never echoes the saved secret back,
  only an `api_key_set` boolean (`LlmClient.has_key`) the page shows as "saved"/"not set".
  A key saved via the UI takes priority over `llm_api_key`/`$OPENROUTER_API_KEY` on the
  next load. Calls run off the ROS executor thread and are one-at-a-time guarded.
  - **Decision log** (`GET /llm/log`, web "🧠 Decision log" panel): every generation path
    (`say`/`chat`/`observe`/`look`/`beat:*`) records a `CognitionLog` entry (trigger,
    state, camera, model, status, say/mood, latency) — incl. skip reasons
    (`skipped-busy`/`llm-unavailable`/`no-frame`). Appended as JSON lines to
    `cognition_log_path` (default `~/.local/state/nanobot/cognition.log`) and seeded back
    into the ring buffer on start, so it survives reboots. **Both `web_server` (robot) and
    `scripts/dev_webui.py` (dev) write the same file/format**, so history is shared. See the
    llm-openrouter-personality memory.
  - **Trait trajectory** (`cognition.record_trait_snapshot`/`trait_trend_text`,
    `trait_history.json`): a durable log of `(timestamp, traits)` snapshots so the robot can reason
    about **how it has drifted over time**, not just react to the last few events. Sampled (≤ once
    per `trait_history_period`) during reflection; `trait_trend_text()` summarises the change over
    the trailing `trait_history_window` (e.g. `curiosity 0.50 -> 0.68 (rising)`) and is folded into
    the `reflect()` + `consolidate()` prompts, so the self-narrative grows from a real trajectory.
    Deploy-synced like the soul. Config: `trait_history_*` in robot.yaml; readout `get_trait_history`.
  - **Phrase bank** (`phrasebank.py`): the most frequent lines — the body-reaction beats
    (`musing`/`observe`) — are **pre-generated** instead of hitting the LLM every idle cycle.
    A batch of in-character lines per *situation* (picked_up/hot/busy/idle/… classified from
    the sensors), each with **placeholders** (`{name}{cpu}{mem}{temp}{tilt}`) filled with
    live values at speak time → instant, free, offline, still varied. Logged
    `status="bank"`. `pick()` prefers lines whose placeholders are all fillable. The bank
    (`~/.local/state/nanobot/phrases.json`) stores the persona+traits **signature** it was
    made with and **auto-regenerates in the background** when the soul drifts too far
    (`phrasebank_drift`) or the persona changes; `phrasebank_live_ratio` still sends a few
    beats live for freshness. **It also grows over time** (`PhraseBank.grow`/`maybe_grow`,
    `CognitionCore.bank_grow_check`): each reflection (`brain_reflect` entry) it *appends* a
    few BRAND-NEW LLM lines to the most under-filled offline situation (deduped, up to
    `phrasebank_grow_max`) — so the offline-triggerable lines keep gaining variety without
    discarding what's there. Growth only runs while the soul is stable (a drifted soul
    regenerates first) and is rate-limited by `phrasebank_grow_period`. **Growth is also an
    on-demand `phrases` meta skill** — `skills/grow-phrases.md` (`CognitionCore.grow_phrasebank`,
    parallel to `forge-skill`): invoke it any time to add lines now (bypasses the period gate,
    blocks on the LLM); excluded from autonomous skill-beat picks like the workshop. Force/
    inspect: `scripts/pregenerate_phrases.py [--show]`, `GET /llm/phrases`,
    `POST /llm/phrases/regenerate`. Config: `phrasebank_*` in robot.yaml.
- **Skill library** (`nanobot_brain.cognition.skills`, ROS-free + unit-tested; the nanobot-brain repo's `skills/*.md`):
  capabilities as **self-documenting markdown** (an OpenClaw-style "SKILL.md" port). Each
  `.md` = one capability — YAML frontmatter contract (`name`/`description`/`trigger`/`action`)
  + a Markdown body the brain reads as the "how". Drop a new file in (and `POST /skills/reload`)
  to add a capability — no code change. `SkillLibrary` loads + indexes them; `web_server`
  executes. **Two tiers:** *narrative* (`kind: say`/`observe`/`look` — speak a line steered by
  the body, optionally with the sensor snapshot or a `read-lidar`-style `/dev/shm` scan summary
  or a camera frame; routes through the same `_generate`/vision path) and a **gated *action*
  tier** (`kind: topic` — publishes a **whitelisted, clamped** ROS msg: `/led`, `/fan_pwm`,
  `/lds_target_rpm`, `/cmd_vel`). An action runs only when the skill sets `enabled: true` **AND**
  `skills_allow_actions` (web_control param, **off by default**); motion speeds are clamped in
  `web_server` itself (SKILL_MOTION_* caps), so a skill can never make the robot unsafe. **Two entry points:** autonomous (the
  chart's `skill` beat → `_run_skill_beat` asks the cheap model to PICK one from the offered
  catalogue → performs it) and on-demand (`GET /skills`, `POST /skills/invoke {name}`,
  `POST /skills/reload`; web "🛠 Skills" card). Every invocation logs to the decision log as
  `skill:<name>`. The dir resolves via `skills_dir` → share → source tree
  (`resolve_skills_dir`); `dev_webui.py` wires the same panel off-robot (topic actions no-op
  there, no ROS). See [[skill-library]].
- **Skill workshop** (`skillsmith.py`, ROS-free + unit-tested): **reflection mode** (formerly
  "meditation") is a **skill-synthesis loop**, not just consolidation. On reflection entry
  `CognitionCore.run_skill_workshop()` runs
  **suggest → check → rehearse → trial → adopt/retire**: the smart model mines the decision log
  (gaps / repeated `no-pick`/`stumped`) for ONE *new* or *adapted* capability, it's validated
  (`validate_candidate`: parse round-trip, kind whitelist, no name collision; action skills born
  `enabled:false`), **rehearsed once** + smart-model **critiqued**, then written to a writable
  **"learned" dir** (`workshop_dir`, default `~/.local/state/nanobot/skills`, loaded as
  `SkillLibrary(extra_dir=…)` — separate from the committed catalogue, deploy-synced like the
  soul/bank) and tracked in `workshop.json` (`WorkshopState`). A trial is a normal, immediately
  auto-eligible skill; the `gate()` **auto-adopts** it (permanent) after `min_runs` good runs +
  net-👍 + no errors, or **auto-retires** (deletes the file) on errors/net-👎. The contextual
  👍/👎 reward is forwarded to the trial that last ran (`reward_trial_skill`). Manual override:
  `GET /skills/workshop` + `POST /skills/workshop/{keep,kill}` (web "🛠 Skills" card, 🧪 trials).
  `deploy.sh` pushes `memory/skills/*.md` + `workshop.json` with the soul. Config: `workshop_*`
  in robot.yaml. Runs identically on the dev harness (mints into `memory/skills/`).
  **The workshop is also an on-demand skill** — `skills/forge-skill.md` (`action.kind: workshop`,
  a "meta" kind in `skills.py` that runs an internal routine, never a topic/narrative): invoke it
  any time (web "🛠 Skills" / `POST /skills/invoke {name:"forge-skill"}`) to forge a skill outside
  reflection mode. Meta skills are **excluded from autonomous skill-beat selection** (`offered()`),
  so they only run when deliberately invoked. (`grow-phrases` — `action.kind: phrases` — is the
  other meta skill: on-demand phrase-bank growth, see the phrase-bank note above.) See
  [[meditation-skill-workshop]].
  - **The autonomous skill beat (`run_skill_beat`) degrades gracefully when the LLM is down.**
    Picking normally asks the model (`llm.complete`) which offered capability best fits the
    moment; if the LLM is unavailable/rate-limited that call returns `None`, so the beat instead
    falls back to a **plain random pick among the currently offered `topic` (action) skills**
    (the only tier that needs zero model calls to execute) — a `blink-led`/`cool-down`-style
    reflex still fires instead of the beat going silent. Narrative (`say`/`observe`/`look`)
    skills (2026-07-16) now fall back too: `CognitionCore._invoke_skill` tries the generic
    sensor-classified phrase bank (`_bank.pick`, bypassing `bank_say`'s live-ratio "go live
    occasionally" skip since there's no live option right now) before requiring the LLM — so
    a named skill still says *something*, just not the skill-specific line, matching the
    generic idle "musing" beat's existing bank-first fallback.
  - **A fourth meta skill grows that offline-only fallback pool: `skills/expand-offline.md`**
    (`action.kind: offline`, `CognitionCore.expand_offline_skills`/`_do_offline_skill`). It reuses
    the exact same workshop pipeline (`run_skill_workshop(offline=True)` → `_suggest_skill
    (offline=True)`), constrained so the smart model MUST propose a pure `topic` capability — a
    reply that ignores the constraint is discarded, nothing is minted. No-op if
    `skills_allow_actions` is off (there'd be nothing useful to grow). Needs the LLM to invent
    the capability now, even though the point is to have something that runs later without it.
- **Reflection mode** (renamed from "meditation"; topic `/reflect`, web `POST /brain/reflect`,
  `🧘 Reflection mode` toggle, `PurposeBrain.set_reflecting`/`.reflecting`, chart state
  `reflecting` + event `reflect`/`wake`). It pauses beats and consolidates (purpose/A/B/bank +
  long-term self-narrative) **and** forges a skill (the workshop). **The robot enters it on its
  own** after a long idle: `behavior.mood_node._auto_reflect` publishes `/reflect_request` (Bool)
  on `reflect_auto_idle` s of continuous idle, runs `reflect_auto_secs`, then wakes (and exits
  early if activity resumes); `web_control` mediates that request through the same `brain_reflect`
  the web toggle uses (`_on_reflect_request`). Manual reflections (web toggle) are sticky and
  never auto-woken. The dev harness drives the same loop time-based in `run_behavior`.
- **Interaction fillers fire BEFORE the LLM call.** On a skill beat the instant "thinking"
  prelude is spoken before the (slow) skill-pick `complete()` call, not after (so TTS feels
  instant); the chosen skill then runs with `prelude=False` to avoid a double filler.
- **Heavy topics stay OFF the telemetry frame:** `/scan.bin` (compact lidar blob =
  JSON header + raw float32 ranges, written by `lds_driver_py`) is served
  same-origin from `/dev/shm` and polled by the page — the page controls the poll
  rate per view. (The old `/map` blob died with slam_nav; slam_toolbox publishes
  `/map` as a real topic that no longer crosses the web gateway.)
  Everything light rides the ONE `/telemetry` SSE frame (see the gateway note above).
  web_control also publishes `/esp32_ping` @1 Hz (ESP liveness, always on).
- **The vitals blob (`/dev/shm/nano_vitals.json`)**: sys_monitor writes ONE aggregated
  body snapshot per tick — CPU/RAM/temp/disk + IMU |a|/|g|/rate/tilt + LDS hz + ESP32
  liveness/temp, NaN-free, with per-source ages + a wall-clock `t` so readers add the
  file's own staleness. The slow consumers READ it instead of subscribing:
  `oled_display`'s dashboard (its telemetry topic subs are gone; local /proc fallback
  when the blob is stale) and `web_control` (cognition body snapshot + the frame's
  imu/eul sections). sys_monitor is now the only /imu/web + /imu/euler subscriber, and
  it's co-resident with imu_driver in sensor_hub — so IMU samples never cross a
  process boundary. **/dev/shm convention: one writer per `nano_*` file, atomic
  `os.replace`, JSON (or JSON-header+binary) payload.**
- Tune live: `imu_driver`/`lds_driver_py` expose `publish_rate` as a settable param;
  the web UI sliders POST `/param`, which calls `/<node>/set_parameters` (whitelisted).
  The IMU's device stream rate auto-follows `publish_rate` (`output_rate_hz: 0`).
  `sys_monitor.fan_temp_min`/`fan_min_duty`/`fan_smooth_alpha` (the Cooling fan card's "Fan
  starts at" / "Floor duty" / "Smoothing" sliders) are whitelisted the same way. The auto
  curve is fully OFF below `fan_temp_min` (50°C default), jumps straight to `fan_min_duty`
  (30% default — a floor above the fan's own stall/dead-band duty so it doesn't crawl too
  weakly to move air) right at that threshold, ramps linearly to `fan_max_duty` (100%) by
  `fan_temp_max`=70°C, and is EMA-smoothed (`fan_smooth_alpha`, default 0.15) so CPU-temp
  noise doesn't make it audibly hunt tick-to-tick. None of these `/param` values persist
  across a `sys_monitor` restart — `robot.yaml` is the durable source of truth.

### Brain health monitoring
Bidirectional heartbeat between the two brain layers:

| Topic | Type | Publisher | Rate | Fields |
|---|---|---|---|---|
| `/brain/behavior_health` | String JSON | `mood_node` | ~1 Hz | `alive`, `chart_states`, `cognition_alive`, `reflecting`, `purpose_enabled`, `traits` |
| `/brain/cognition_health` | String JSON | `web_server` | 1 Hz | `alive`, `llm_available`, `llm_fail_streak`, `llm_offline`, `reflecting`, `behavior_alive` |

Each node subscribes to the other's health topic. If cognition ping is >5s stale, `cognition_alive` → false. If behavior health is >10s stale, `behavior_alive` → false.

**HTTP endpoint:** `GET /brain/health` returns aggregated health:
```json
{"behavior":{...}, "cognition":{...}, "overall":{"behavior_alive":true,"cognition_alive":true,"all_healthy":true}}
```

**Web UI:** AI · Speak tab > AI & brain group > "Brain health" card shows behavior, cognition, LLM, purpose, chart status — green/alive or red/lost. Polled every 2s from `/brain/health`. If the endpoint itself fails, all indicators show amber `err`.

### Web gateway static page + media endpoints

- **`web_control` static server**: serves `web/` — `index.html` plus `style.css`. The
  page is **self-contained**: one big `"use strict"` inline block (`app.js`-derived: the
  SSE `/telemetry` EventSource + all control) with the OLED-mirror `oled.js` inlined
  right before it, then smaller self-contained IIFE blocks (chrome tabs,
  live odometry/IMU readouts, the 2026-09-15 Map/click-to-goal/Locations block) — all
  pure same-origin SSE/HTTP, no
  external scripts, no rosbridge/ROSLIB. The old split files (`app.js`, `map.js`,
  `oled.js`, `chrome.js`, `sim.js`, `devtools.js`, `logs.js`, `personality.js`) were
  DELETED from the repo (2026-09-16) — do not reintroduce external `<script src>`
  loading or wire a websocket. The in-browser Sim tab was removed with them (2026-09-16;
  dev-PC testing is `scripts/dev_webui.py`).
  The web **Map panel is back (rebuilt 2026-09-15 on top of Nav2)** — a canvas fed from
  slam_toolbox's `/map` via the `GET /map` HTTP route (NOT the SSE frame), click-to-goal,
  a goal-status chip from `/navigate_to_pose/_action/status`, `POST /nav/cancel`, an
  inflation bubble, and a rebuilt Locations card ("save spot" falls back to the TF pose).
  See the "Map view + click-to-goal REBUILT" block in AGENTS.md. `/scan.bin`
  still feeds the Lidar hero view; `/goal_pose` (locations, skills) still drives Nav2.
  `/stream.mjpg` is a zero-dep V4L2 MJPEG passthrough (`mjpeg_camera.py`);
  `/snapshot.jpg` is one still frame (📸 button); `/audio.pcm` is the webcam
  mic as raw PCM via `arecord` (`mic_audio.py`). Both streams are ref-counted (only
  run while a client is connected) and the audio endpoint **must** be HTTP/1.1 chunked
  (browsers don't stream an HTTP/1.0 body to `fetch`). `GET /health/log` serves the
  tail of sys_monitor's durable outage log for the web "Health events" card.
### GPU vision

- **GPU vision** (`gpu_vision.py`, `gpu_vision_enable` param, default `true`): runs the
  webcam through the H5's **Mali-450** instead of a plain passthrough — a headless
  EGL/GLES2 context (raw ctypes, no `moderngl`/OpenCV) captures continuous YUYV,
  converts to RGB in-shader, and runs everything else as GLSL ES 1.00 fragment shaders
  reduced via a box-filter downsample chain (`build_downsample_chain`/
  `run_downsample_chain`, ~1.9ms/pass measured on hardware) so only a handful of bytes
  ever cross back to the CPU — never a full frame. **Core** (hardware-verified,
  committed a881ddc): PIR motion-diff (`_DIFF_FS`) → `motion_score`/`motion_center`,
  and calibrated colour-blob tracking (`_THRESHOLD_FS` + largest-blob selection) →
  `target` (bearing/confidence), with live-tunable match tolerance + min/max blob-size
  gating. **Tier-B**: kinetic-intercept alert (blob-area growth rate), flashlight/dark
  reflex (opt-in, auto-`/led`), and the optical virtual bumper
  (commanded-but-not-moving → possible stall, in `telemetry.py`). (The old **manual
  mode** — `POST /vision/manual`, a live swap to the direct `CameraStream`
  passthrough — was REMOVED 2026-07-14 (01e64f9): the passthrough now engages only
  as the automatic fallback when GPU vision is off/unavailable. The same commit
  compensates the **upside-down camera mount** once at the YUYV→RGB source pass
  (`_VFLIP_VS`), so every consumer sees an upright frame.) Two on-demand debug MJPEG views mirror this same
  reduction machinery for human eyes: `/stream_mask.mjpg` (the colour-threshold hit
  mask) and `/stream_motion_mask.mjpg` (the PIR diff mask, reusing the same
  `_MASK_VIEW_FS` shader unmodified against a different source texture) — both
  viewer-gated (zero cost unwatched), toggled from the Camera tab.
  **Cheap-tier batch** (2026-07-12, live-verified on hardware): five more raw signals —
  `edge_density`/`overhead_edge_density` (a new 3-tap gradient shader, whole-frame and
  cropped to the top 30% as an overhead-clearance heuristic), `luma_max` (free —
  extends the existing dark-reflex luma readback with a max), `highlight_fraction`
  (reuses the blob-tracking shader with a fixed white target in a separate FBO so it
  never collides with the user's calibrated colour), and `motion_target_match` (pure
  CPU distance between the motion and blob centroids). All threshold/alert logic lives
  in `telemetry.py`'s `_vision_alerts` (NOT `gpu_vision.py`), mirroring the optical
  bumper's "read live ROS params, not fixed constants" pattern — **12 alerts total**
  (`obstructed`/`clutter`/`overhead_alert`/`focus_blur`/`backlit`/`shiny`/`looming`/
  `colorcast`/`motion_matches_target`, plus the 2026-07-13 batch's `novelty`/
  `camera_freeze`/`vibration`), each with its own `vision_*` param, live
  sliders under the Camera card's "▸ Vision alerts tuning". `vision_obstruction_var_max`
  was retuned from a wrong-by-~100x guess (15) to 400 after a live reading showed an
  ordinary scene reads `luma_variance` ~2700 — a reminder that these thresholds are
  real hardware quantities, not proportions, and need checking against actual readings
  before trusting a default. A software-measured **`gpu_duty`** ("pipeline load")
  tracks the fraction of each frame's period spent in the shader+readback block —
  measured 60-190%/tick on hardware with the full batch running, i.e. the vision loop
  can fall behind its configured fps under load (degrades gracefully, no crash). A
  **true hardware GPU-utilization reading was tried and removed**: a devfreq
  frequency-ratio estimate (`sys_monitor`) turned out to always read "n/a" on this
  board's kernel (no GPU devfreq node) and GPU temp just tracked CPU temp with no new
  information — both pulled from the UI; `gpu_duty` was kept since it's a different,
  demonstrably-useful number. Scalar readouts are exposed via **one
  `gpu_vision.snapshot()` atomic read** (single `_lock` acquisition for all ~20
  fields incl. frame_age/zero_motion_secs, added 2026-08-10): the 5 Hz telemetry
  build and the 10 Hz `_vision_state_tick` use it instead of ~20 per-property
  getters, each of which re-took the lock — saving ~60 guarded round-trips/s and
  removing cross-field reading skew. A **master camera switch** (`POST /vision/camera_enable`,
  the Camera tab's "📷 Camera enabled") fully stops BOTH `GpuVision` and the direct
  passthrough — for when the fuller pass set's cost isn't wanted. While off, the live-view `<img>` shows
  a real `#camWait` overlay message ("Camera disabled…") instead of a broken-image
  icon, and `app.js`'s `onVision` diffs `camera_enabled` tick-to-tick so re-enabling
  auto-resets the stream `src` itself — no manual page refresh needed to get the feed
  back. The Sensors tab's readouts AND every tunable slider have hover explanations
  (native `title` attributes keyed by element ID in `app.js`, not markup changes),
  toggleable/persisted via **💡 Show hints**.
  **2026-07-13 batch (code-complete + unit/smoke/GL-tested on the dev PC; hardware
  verification tracked in `docs/TODO.md`) — all built:**
  - **Named colour targets**: calibrations persist to `vision_targets.json` under a
    name (the "target name" box in the Camera view; default "default") and now
    **survive restarts** (re-applied on boot). One target is tracked at a time —
    selection, not simultaneous multi-target. `GET /vision/targets`,
    `POST /vision/target_select|target_delete`; blob-tune edits sync into the active
    entry; Sensors→Camera has the palette row.
  - **Novelty score** (`GpuVision.novelty`): mean diff of the already-read-back small
    colour buffer vs. a slow EMA background (`update_novelty`, `NOVELTY_EMA_ALPHA`
    ~22 s) — sustained scene change scores high then habituates. Zero extra GPU cost.
  - **Camera-freeze diagnostic** (`frame_age`/`zero_motion_secs` + the
    `camera_freeze` alert): capture stopped delivering, or delivers the identical
    buffer (exactly-zero diff) — "recover the camera", vs. the bumper's wheel-stall.
  - **Vibration diagnostic** (`vibration` alert, telemetry.py): edge-density far
    below the standing-still EMA baseline while driving = excess blur (loose screw /
    imbalance). Maintenance hint.
  - **Glare rejection** (`vision_glare_derate`, default 0=off): blob confidence is
    derated by `highlight_fraction` so a specular reflection can't hold a false lock.
  - **OLED mask mirror** (`POST /vision/oled_mask`, latched `/oled_mask` Bool +
    `/dev/shm/nano_oled_mask.bin`): gpu_vision rides the thresh chain to 160×120 then
    one more pass to exactly 128×64, re-binarizes (bytes.translate), writes the blob;
    `oled_display` renders it as a "mask" owner (below reflecting/words/shutdown,
    above the face). Auto-dropped when the camera master switch stops capture.
  - **Vision→behaviour plumbing**: web_control publishes a compact `/vision/state`
    JSON @2 Hz (approach/looming/clutter/novelty/warmth/motion, only while the
    pipeline is live — staleness IS the stand-down signal). mood_node consumes it:
    **anticipatory greeting** (motion growing + centred = someone walking up →
    greet-face + a `greeting` beat, rate-limited, idle-only), **looming/clutter →
    caution fast rules** in `brain.Personality` (looming = edge-triggered startle;
    clutter = hold caution ≥ `clutter_caution` while it lasts and RELEASE to the
    remembered pre-clutter value after — expression-level only now: the old
    caution→max_lin velocity throttle died with slam_nav's `trait_motion`),
    **ambient colour mood** (scene warmth R−B tints the
    chart's `feeling` face via the injected `ambient_mood` — the LLM's `drives.mood`
    always wins), and a **novelty boost** on the `looking` beat (transient `beat_boosts`
    multiplier in `choose_beat`, distinct from the LLM-evolvable registry priority).
  - **Visual diary** (`cognition.record_vision_snapshot`/`vision_trend_text`,
    `vision_diary.json`): scene scalars sampled every `vision_diary_period` (10 min),
    trend ("the room has got darker (60% → 15%), and calmer") folded into the
    `reflect()`/`consolidate()` prompts like the trait trajectory. `GET /llm/vision_diary`.
  Deferred/excluded (tracked in `docs/TODO.md`): the overhead-clearance
  camera-mount geometry check (needs the physical robot) and the docking/cliff
  items (explicitly excluded by the user).
### Stress test

- **Stress test mode** (`stress.py`, ROS-free; web "Stress test" card in System):
  `POST /stress/start {duration,workers?}` / `POST /stress/stop` / `GET /stress/status`.
  Deliberately loads every CPU core to validate the hardening tier (systemd watchdogs,
  MemoryMax, the fan curve) under real load — **without starving the web server that
  has to keep answering the browser during the test**. Workers are separate, NICED
  (19, the lowest scheduling priority) subprocesses running a tight busy loop; they
  aren't pinned away from any core, so an idle board gets genuinely pegged to 100% on
  every core, but the kernel's CFS scheduler always prefers a normal-priority process
  (this web server, the other ROS hubs) the instant it has work — same trick as
  `nice -19 stress --cpu N`, no core reservation needed. A background watchdog
  auto-stops the run at `stress_max_duration` (300 s default; a forgotten test can't
  run forever) regardless of the caller, and can abort early past `stress_temp_abort_c`
  (82°C default, 0 = off). CPU-only by design — no memory allocation, so there's no
  risk of tripping app_hub's own systemd `MemoryMax` and getting the web server's unit
  OOM-killed mid-test. Single-flight (one run at a time); `destroy_node` stops an active
  run on shutdown. Shared verbatim with `scripts/dev_webui.py` (same `StressTest` class).
### Browser telemetry+control gateway

- **Browser telemetry+control gateway (`telemetry.py`, replaced rosbridge)**:
  `GET /telemetry` is ONE SSE stream (browser `EventSource`, native auto-reconnect)
  of a compact JSON frame at `telemetry_rate` (5 Hz) with every light readout —
  odom, IMU, `/diagnostics`, ESP32 (hb/ticks/susp/temp/hall), LDS rpm/hz/duty, fan,
  plan (downsampled), latched brain strings (purpose/task/experiments), selftest,
   and the OLED-mirror inputs (face/word/brand/system). The frame carries a wall-clock
   BUILD stamp `"t"` (added 2026-09-22 after the pid_tune burst artifact — see the
   ESP32 PID gotcha; consumers must measure inter-frame dt from it, never from their
   parse time, or a post-stall burst inflates speeds ~6-10×). The frame is built ONCE per
  tick and fanned out; the underlying subscriptions are **lazy** (created on the
  first client — on the executor thread via the tick timer — dropped `SUB_LINGER`
  after the last), so idle cost is ~zero. Writes: `POST /publish {topic,value}`
  (whitelisted + clamped per topic: goal_pose, lds_target_rpm, motor_trim, motor_pid,
  motor_params, pickup_override, laser_pwm, reset_ticks, imu_calibrate, schedule_edit,
  selftest, go_home/save_map, oled_*) and `POST /param {node,name,value}`
  (whitelisted nodes/params via `/<node>/set_parameters`, fire-and-forget). The
  power buttons only POST `/system/*`; the server itself publishes `/oled_system`.
  **Zenoh gotcha:** `DiagnosticStatus.level` (from `sys_monitor`'s `/diagnostics`,
  carried as `_pipe_diag`) can arrive as a raw `bytes` (`b'\x00'|b'\x01'|b'\x02'`) under
  rmw_zenoh rather than a plain `int`. It is normalized to an `int` in
  `telemetry.py:_on_diag` — keep it that way, and treat every raw ROS field added to the
  frame as potentially-non-JSON-safe after the zenoh round-trip (an unhandled `bytes`
  in `json.dumps` in `_tick` kills the whole app hub → systemd respawn loop).
  **Any** unhandled exception inside a subscription callback runs on the executor
  thread and kills the hub the same way — e.g. `_on_slam_pose` reading
  `msg.pose.pose` (the Odometry layout) on the actually-`PoseStamped` `/slam_pose`
  was an `AttributeError` respawn loop (fixed 2026-08-10). New callbacks must
  match the real message type and be JSON-safe end-to-end after the zenoh round-trip.
### LDS idle spin-down + jam guard (2026-09-21)

The old slam_nav-era `_update_lds_idle` died with slam_nav (2026-09-14) and for a week
NOTHING owned `/lds_target_rpm` — the ESP32 held its last setpoint (boot default 300
rpm) forever, so the lidar spun whenever the robot sat idle ("the lds spindown isn't
reliable"). Rebuilt as THREE coordinated pieces:

- **Idle controller (`telemetry.py`, the OWNER of `/lds_target_rpm`)** — a 1 Hz
  ALWAYS-ON node timer (`_lds_ctrl_tick`, pure decision in `lds_idle_target`,
  unit-tested in `test_lds_idle.py`): spin at the user's target while the robot is
  **active** (commanded `/cmd_vel` above `vision_bumper_cmd_eps` within
  `lds_idle_secs`, OR a Nav2 goal in flight — `planning` counts, the planner needs
  fresh costmap scans BEFORE moving), park it (0) after that quiet stretch. A busy
  nav status is trusted only within `LDS_NAV_STALE` (90 s) of its last arrival — the
  status topic is event-driven, so a live goal is really held up by `/cmd_vel`, and
  the age bound stops a mid-navigation `nano-nav` death from freezing "navigating"
  and keeping the lidar awake on a parked robot forever.
  `lds_idle_enable=false` = always spin; `lds_manual_secs` (300) = how long a manual
  topic post (slider drag, skill action) holds the topic before the controller takes
  it back. Every 30 s (`LDS_REASSERT_SECS`) it re-publishes an unchanged setpoint —
  an ESP32 reboot resets its setpoint to the firmware default, so the re-assert
  corrects it with no user action. `note_lds_manual(rpm)` is the outside-publisher
  hook (browser `publish_json` sets the remembered spin-when-active target
  `_lds_user_rpm` too — the slider IS that value; skills only borrow the topic).
  `note_map_clear()` arms a **rebuild window** (one `lds_idle_secs` period treated
  as recent motion) so a POST `/map/clear` wakes a parked lidar — a wiped map can
  only rebuild if scans flow, and a quiet robot's lidar would otherwise stay parked
  while slam sits on no-map until someone drives.
  The controller's signals (`/cmd_vel` + `navigate_to_pose/_action/status`) are
  **always-on subscriptions moved OUT of the lazy browser-only set** (`__init__`,
  not `_make_subs`) — the lazy set drops `SUB_LINGER` after the last browser, which
  would park the lidar MID-NAVIGATION. They also feed the optical bumper + the web
  map's status chip (one sub, three consumers). Params: `lds_idle_enable`,
  `lds_idle_secs` (60), `lds_manual_secs` (300), `lds_active_rpm` (300) in
  robot.yaml `web_control`, all live-tunable via `/param` (web Lidar card).
  **PERSISTENT (2026-09-21): the Lidar card's spin-down settings survive a
  restart/reboot** — one `add_on_set_parameters_callback` in `web_server`
  (`_persist_lds_params`) snapshots the whole cluster
  (`lds_idle_enable`/`lds_idle_secs`/`lds_manual_secs`/`lds_active_rpm`) to
  `~/.local/state/nanobot/lds.json` on ANY setter (`POST /param` for the
  toggle+secs slider, the Spin slider's drag → `note_lds_manual(set_target=True)`
  → a param set), and boot re-applies the file over the robot.yaml defaults
  BEFORE `TelemetryHub` is constructed (its `_lds_user_rpm` seeds from the
  re-applied `lds_active_rpm`). Same "persisted UI wins" pattern as
  move.json/tts.json/vision.json (delete the file to return to robot.yaml);
  overridable via the `lds_settings_path` param. The frame's `f.lds` carries
  `enable`+`secs` so a freshly loaded page re-seeds its Lidar-card controls once
  on first arrival (the `f.esp.wheel_pid` readback pattern).
  **The page no longer re-publishes the Spin slider on SSE (re)connect** — that
  `syncLdsTgt()` was the 2026-07-14 force-wake bug (opening the page woke the parked
  lidar); the slider instead follows the live setpoint back from `f.lds.tgt`
  (skipped mid-drag). The IMU interference test calls `telemetry.lds_hold(True/False)`
  for its whole run so the controller never fights its own LDS phase.
- **Jam guard (firmware, `main.cpp` `ldsControl`)** — a physically blocked rotor
  (string wrapped around the turret, debris) can't reach speed, so the PID pins at
  full duty and the motor cooks. If rpm stays under `LDS_JAM_FRAC`*target (0.4 —
  scales with the setpoint so a low cruise target can't false-trip) OR UART1 is
  stale (no tach frames = driving blind = same risk) continuously for
  `LDS_JAM_MS` (6000) while a target is set, the firmware LATCHES a jam and cuts
  the PWM. The latch clears ONLY on target ≤ 0 (which the SBC idle timeout provides
  naturally at the next quiet stretch, or the slider's 0) — no periodic grind, each
  wake-from-idle retries at most once. Published on **`/lds_jam`** (Bool, @5 Hz
  with the other lds topics; PubDef tag/eid 14, `g_lv[]` bumped to 16) →
  `f.lds.jam` → the Lidar card shows **JAM** red with the remedy in the hint
  (clear the obstruction, set Spin 0, then back up). `ldstgt_cb` now also clamps
  to `LDS_RPM_MAX` 400 and rejects NaN.
- **Web UI (Lidar card)**: new readouts **`last move`** (the live seconds-since-last-
  commanded-motion timer, ticking every second between frames — this is the idle
  clock) and **`spin state`** (spinning/parked/manual/held/JAM, colour-coded), plus
  the **Idle spin-down** toggle and the **Spin down after N s** slider (both
  `/param` → web_control). The Map card's feeds-health strip paints the LDS dot
  **amber** when `f.lds.state == "park"` — an intentionally parked lidar is not a
  broken feed. `f.lds` is now `{rpm, hz, duty, jam, age, tgt, idle, state}`.

### HTTP teleop

- **HTTP teleop (`POST /drive`)**: the page POSTs `{v,w}` same-origin; `web_server`
  clamps (`drive_max_lin`/`drive_max_ang`) and arms a ~3.3 Hz keepalive that
  re-asserts the command while non-zero with a `drive_timeout` dead-man — so browser
  jank can't outlast the ESP32's 500 ms cmd watchdog and stutter the drive. The
  keepalive runs on a **dedicated thread, not the ROS executor**
  (`web_server._drive_loop`, 2026-09-20): executor callbacks slip 1-9 s under
  TTS/vision/LLM load, which starved the re-assert past the ESP32 watchdog and
  dead-manned the drive mid-motion (stop → lurch on recovery). rclpy publishers are
  thread-safe; the thread only touches the lock-protected (v,w) state + publish(),
  gets a best-effort `os.nice(-5)`, and is joined in `destroy_node`. The dev
  harness accepts it as a no-op. **Braked stop (2026-09-21): after the command goes
  zero (explicit `{0,0}` POST or the dead-man), the keepalive keeps publishing the
  zero for `BRAKE_GRACE` (1.0 s) before going idle — going idle immediately meant the
  ESP's 500 ms cmd watchdog cut DUTY (an unpowered coast): a 0.05 m/s crawl rolled
  ~1.4 s / ~7 cm past the stop (blip-test). With the cmd kept fresh at zero the
  firmware's PID actively brakes (kp on the negative error + integrator unwind +
  the parked-bleed reset at rest) — a firm ~0.3 s stop. Serial cost: 3-4 extra
  frames per stop.**
- **Canned moves (`POST /move` {"dist" m, "deg" deg, "cancel"?}, added 2026-09-21):**
  relative, /odom-feedback maneuvers (drive N metres — signed, then rotate N°) for
  the Drive card's two numeric fields beside the joystick. **No new /cmd_vel
  publisher**: the maneuver thread (`web_server._man_loop`/`_run_maneuver`, 10 Hz)
  only rewrites the keepalive's lock-protected (v,w) state — every byte on the wire
  still goes out at the 3.3 Hz keepalive rate (serial budget). Feedback is
  `telemetry._odom` (wheel-integrated, true units post-scale-fix); the drive phase
  measures progress as the **projection on the phase-start heading** (trim veer pads
  path length, not the target), the turn is a P law on wrapped yaw error toward the
  re-snapshotted post-drive yaw. Guards: refuses while Nav2 is
  planning/navigating/canceling (telemetry `_goal_status`), refuses without /odom,
  one maneuver at a time (a new request replaces the running one), hard-abort
  timeout = max(`move_timeout`, 1.5× duration estimate), and a **browser dead-man**
  (SSE clients gone > `MOVE_BROWSER_GRACE` 5 s = abort — a canned move must not
  outlive its page). ANY `/drive` POST (joystick, keyboard, STOP, tab-hide {0,0})
  cancels the running maneuver (`drive()` sets `_man_cancel` — the keepalive's own
  dead-man can't stop a maneuver because the maneuver keeps feeding `_drive_at`).
  Progress rides the SSE frame as **`f.move`** `{active, phase driving|turning,
  target, unit m|deg, progress, err}` + a terminal `{result: done|cancelled|failed,
  error?}` latch. `POST /move {cancel:true}` is the explicit stop; drive speed/rate
  are `move_lin_speed` (0.12 m/s) / `move_ang_speed` (0.5 rad/s — under the
  `w·0.2 rad/scan` SLAM smear budget), clamps `move_max_dist` ±5 m / `move_max_deg`
  ±720°. The two speeds are **live-tunable from the Drive card** ("Canned speed"/
  "Canned turn" sliders, `GET/POST /move/config`): a change is clamped to
  `MOVE_LIN_RANGE`/`MOVE_ANG_RANGE`, applied via `set_parameters`, and persisted to
  `~/.local/state/nanobot/move.json`, which is re-applied over the robot.yaml
  defaults at boot (the llm.json/tts.json "persisted UI wins" pattern). **The page
  warns past the saturation cliff (2026-09-22)**: an amber `#moveLinWarn` note under
  the Canned speed slider when the value exceeds 0.35 m/s (the measured saturation
  cliff — loaded full-duty ≈0.37 m/s, so above ~0.35 the loop has zero authority and
  the drive stutters; smooth band ≤0.15 m/s). The cliff is hardware, not a bug — the
  note re-seeds from `GET /move/config` and the hover hint on `moveLin` says the same.
  The pure step math is
  `_maneuver_step` (unit-tested in `test_maneuver.py`: projection/backward sign,
  phase re-snapshot, P clamp + low-kp floor, wrap, full-loop on a fake node, +
  `_clamp_move_cfg`); the
  dev harness stubs `/move` + `/move/config` as no-ops (no odom, no motors).
  **~3.3 Hz, NOT 10 Hz — the serial budget (2026-09-20):** `/cmd_vel` crosses the
  115200-baud UART to the coprocessor, and SUSTAINED 10 Hz flow decays on that link
  (zenoh-pico's tiny UART RX FIFO + a busy zenoh task lose bytes → deliveries thin
  out → the wheels stall mid-command; measured same-session: 3 Hz cruises clean,
  10 Hz decays to a crawl). So the keepalive publishes at 0.3 s periods and
  `drive()` NO LONGER publishes directly (the keepalive is the SOLE /cmd_vel
  publisher — 10 Hz POSTs + a 3.3 Hz keepalive would still overflow); worst-case
  command latency is one keepalive period (300 ms, fine at crawler speeds). Same
  budget set Nav2's `controller_frequency` to 4.0. Scripted teleop/tests must also
  stay ≤3-4 Hz. (If higher rates are ever needed: raise the serial baud on BOTH
  ends — router config `serial//dev/ttyS1#baudrate=…` + firmware UART2 — noted in
  docs/TODO.md.)
### Text-to-speech (TTS)

- **Text-to-speech** (`tts.py`): `POST /tts {text,voice?}` synthesises with
  `espeak-ng` (install via `deploy/install-espeakng.sh`; NOT on conda-forge so must be
  apt-installed on the board separately) to a `/dev/shm` WAV, prepends `LEAD_SILENCE`
  (0.35 s) so the H5 codec's power-up ramp can't swallow the first word (it wakes on
  PCM open; a back-to-back utterance was never clipped because it was still awake),
  plays it with `aplay`, and
  publishes the words one at a time on **`/oled_word`** timed to the clip duration
  (espeak emits no word marks, so timing is length-weighted). `oled_display` shows
  each word big+centred as it's spoken ("karaoke"); `""` returns to the dashboard.
  Both binaries run **only while speaking** (zero idle cost). The web "Speak" box
  reuses the old OLED-text field; it no longer publishes `/oled_text` (that brand
  override still works if published manually). HTTP POST on purpose (server owns audio+timing).
  - **`TtsEngine.wait(timeout=)`** blocks until the in-flight utterance's playback thread
    exits. `POST /system/{restart,reboot,shutdown}` (`web_server.py`) speaks the matching
    farewell/restart line (`system_announce`→`cognition.speak_lifecycle`) then calls this
    (10 s bound) before firing the detached systemctl/stack.sh command — **fixed 2026-07-15**:
    it used to fire after a flat 3 s sleep regardless of the line's actual length, so a longer
    line got cut off mid-sentence by the shutdown/reboot itself (deployed with the
    2026-07 deploys).
  - **Voice/volume/speed/pitch** are tuned in the UI and applied directly to
    espeak-ng's `-v`/`-a`/`-s` flags. They + the stats announcer are **persisted**
    to `~/.local/state/nanobot/tts.json` (override with
    the `tts_settings_path` param) and reloaded on node start, so they survive a
    reboot. `GET/POST /tts/config` read/update them; the page restores its controls
    from `GET /tts/config` on load.
  - **Spoken system stats**: a server-side 1 Hz tick (`_announce_tick`) speaks
    CPU%/RAM%/CPU-temp every `announce_interval` s when `announce` is on — it lives
    in the node, so it **keeps running after every browser closes** and resumes after
    a reboot. `POST /tts/announce` says it once now. CPU/RAM/temp come from the same
    cheap `/proc` + thermal reads the OLED uses; phrasing follows the selected voice.
  - **Cross-platform TTS for dev testing**: `tts.py` is ROS-free and auto-selects a
  backend — `espeak-ng` on Linux, Windows SAPI (via PowerShell `System.Speech`) or
  macOS `say`. So `scripts/dev_tts_test.py` (no ROS) speaks a line on a dev PC:
  `python scripts/dev_tts_test.py "hi"`, or `--llm "prompt"` to run the full
  OpenRouter→speech pipeline (needs `OPENROUTER_API_KEY`). espeak-ng supports
  volume, speed, and pitch natively. `scripts/dev_webui.py` serves the
  **real `web/index.html`** on a dev PC (ROS-free stand-in for `web_server`) and runs
    the **same `CognitionCore`** (so there's one base, not two — see below), wiring `/llm/*`,
    `/skills/*`, `/tts*` + the brain card, so the AI/Skills/Brain cards + Speak box can be
    tested in a browser locally (telemetry/joystick/map show offline — no /telemetry). Reads
    the persona/model from robot.yaml (PyYAML) and the key from `$OPENROUTER_API_KEY`, or —
    if unset — a one-line `memory/openrouter_key` file (gitignored; `llm.load_openrouter_key()`,
    the ONE shared loader called by `dev_webui.py`/`dev_tts_test.py`/`personality_creator.py`/
    `pregenerate_phrases.py`, falling back to the old `scripts/.openrouter_key` path).
### Nav2 migration (2026-09-14) — slam_nav/EKF REPLACED, sections below are historical
`docs/nav2-migration.md` was executed: the custom `slam_nav` node, the robot_localization **EKF** (`nano-ekf`), and `nano-map` are **DELETED**. The stack is now Nav2 Humble servers composed into ONE `rclcpp_components/component_container_isolated` (unit `nano-nav`; components attached by a `nano-nav-loader` oneshot via `nav2.launch.py load_only:=true`), the static `base_link→laser` TF (yaw π) its OWN `nano-tf` unit (a never-exiting ExecStartPost would hold a Type=simple unit in "activating" forever — that is why it is not on `nano-nav`), plus slam_toolbox 2.6.10 as its own `nano-slam` unit. Units are now `router app sensors nav tf slam nav-loader`. Key facts that differ from everything written below this point:

- **Pose chain:** `map→odom` = slam_toolbox, `odom→base_link` = wheel_odometry (`publish_tf: true` now — it owns the TF, the EKF is gone). No `/odometry/filtered`, no `/slam_pose`, no `/slam_nav/diag`, no `/plan`.
- **heading_flip** survived as `nav2.launch.py heading_flip:=true` (default): a static `base_link→laser` TF with yaw π (this unit's sensor head faces back). slam_toolbox + Nav2 then work in the drive frame directly.
- **slam_toolbox lifecycle:** the robostack build is **2.6.10, where SlamToolbox is a PLAIN `rclcpp::Node`** — its own executable main calls `configure()`; there are NO lifecycle services and **composing the registered component is inert** (nothing calls configure; verified against the installed binaries; robostack has no newer build for either platform — 2.6.x only). So slam_toolbox runs as its OWN process (`nano-slam.service`, `unit_exec.sh slam`, `async_slam_toolbox_node` + `config/nav2/nav2_params.yaml`), NOT in the container and NOT in any lifecycle manager. The `lifecycle_manager_slam`/`bond_timeout: 0.0` design in the original plan applied to slam_toolbox ≥2.7 only. Do NOT try to "compose slam_toolbox" until robostack ships a ≥2.7 build (then revisit: a second manager with bond_timeout 0.0 drives it).
- **Goals:** browser/skills publish `/goal_pose` (PoseStamped, frame `map`) exactly as before — bt_navigator consumes it. The minimal recovery BT (`config/nav2/recovery_bt.xml`): fail → clear costmaps → back up 0.15 m → spin 90° → retry once → abort.
- **Params:** Nav2 tunables live in `config/nav2/nav2_params.yaml` (NOT robot.yaml; restart `nano-nav` to change them) — EXCEPT the navigation pace: **speed + linear/angular accel/decel are LIVE-tunable since 2026-09-23 via the composed `nav2_velocity_smoother`** (see the block below). The web `/param` whitelist's `slam_nav` entries are GONE. RPP controller: `rotate_to_heading_angular_vel` is the angular cap (no `max_angular_vel` in Humble); **`controller_frequency` is 4.0 (2026-09-20) — the serial budget: `/cmd_vel` crosses the 115200-baud ESP32 link, which decays under sustained 10 Hz flow (see the HTTP teleop note)**; Ceres threads are hardcoded 50 upstream — the guards are `minimum_travel_distance/heading` pose-graph gating + `CPUAffinity=2 3`/`Nice=10`/`MemoryMax=400M` on `nano-nav.service` + `MALLOC_ARENA_MAX=2`.
- **Nav2 speed & acceleration — `nav2_velocity_smoother` (added 2026-09-23, live-tunable):** the Drive tab's "Navigation pace" card (4 sliders: Nav speed / Nav accel / Turn rate / Turn accel → `GET/POST /nav/config` in `web_server.py`, persisted to `~/.local/state/nanobot/nav.json` — the move.json "persisted UI wins" pattern) drives the smoother's DYNAMICALLY-reconfigurable `max_velocity`/`min_velocity`/`max_accel`/`max_decel` (3-element `[x, 0, θ]` arrays, y=0 diff drive) through a SetParameters client on `/velocity_smoother/set_parameters` — **live, no nano-nav restart** (unlike the RPP plugin params, which are cached at `configure()`). Routing: `controller_server` publishes `/cmd_vel_nav` (remapped), the smoother caps speed+accel and publishes the final `/cmd_vel`; ESP32, telemetry's `/cmd_vel` sub and the web keepalive are unchanged — teleop/canned moves stay DIRECT on `/cmd_vel` (firmware `WHEEL_TGT_SLEW` handles their accel; the smoother owns nav output only). `smoothing_frequency: 5.0` NOT the 20 Hz default — the smoother republishes on `/cmd_vel` at that rate and the 115200-baud ESP32 link decays under sustained high-rate flow (same serial budget that set `controller_frequency: 4.0`). `feedback: OPEN_LOOP`; `velocity_timeout: 0.5` = active zero-brake after the controller goes quiet. Sign rules are validated at the smoother's `configure()` (positive decel → crash): `/nav/config` derives `min_velocity = -max_velocity` and `max_decel = -max_accel` server-side (unit-tested in `test_nav_config.py`; smoke test covers GET/POST/clamps/persistence — the smoother push itself degrades to a graceful `"error"` when nano-nav is down, values still apply+persist). Boot re-apply is EVENT-DRIVEN (2026-09-23 board race found + fixed): the smoother's SetParameters service exists from node CONSTRUCTION, `on_configure` only DECLARES the params, and `on_activate` attaches the dynamic-params callback that updates the smoother's cached members — a push before activation is silently lost (the first board boot's push landed 3 s before `on_activate` and the cap stayed at the yaml default). So web_control subscribes **`/velocity_smoother/transition_event`** and re-applies the saved pace on EVERY `goal_state == "active"` transition (rate-limited 5 s) — covering both boot and any nano-nav restart — plus a bounded (60 s service + 90 s active-state) boot fallback via a `GetState` poll for the already-active-before-app case. GOTCHA: lifecycle `TransitionEvent` fields are `start_state`/`goal_state` (NOT `start`/`goal`) — a wrong attribute in an executor callback is the app-hub respawn loop. `ros-humble-nav2-velocity-smoother` added to pixi.toml (aarch64 + x86_64, ~195 KiB). Evaluated + rejected same day: `nav2_rotation_shim_controller` (RPP's `use_rotate_to_heading` already does the job), `nav2_collision_monitor` (deferred — costmap+optical bumper+dead-man already cover it), `nav2_denoise_layer` (no obstacle layer to denoise — the local costmap is static+inflation only).
- **Params-file delivery gotchas (found 2026-09-22 when the web costmap overlay shipped):**
  (1) **The `nav2_container` MUST get the full params file process-wide** (`unit_exec.sh nav` now execs it with `--ros-args --params-file "$NAV2_PARAMS"`; the launch's dev-path container got `parameters=[PARAMS]` for parity). launch_ros inlines ONLY the sections matching each component's own name into the load request, so the double-nested `local_costmap.local_costmap.*` / `global_costmap.global_costmap.*` sections NEVER reached the child costmap nodes the servers create at runtime — **both costmaps silently ran on Nav2 defaults for the entire Nav2-migration era** (inflation 0.55 not 0.25, robot_radius 0.1 not 0.16, `always_send_full_costmap` false). A process-wide `--params-file` applies by node FQN instead. Symptom signature: `ros2 param get /global_costmap/global_costmap robot_radius` reads the DEFAULT (0.1) instead of the yaml's 0.16.
  (2) **Costmap `width`/`height` must be INTEGERS in this Humble build** — Costmap2DROS declares them as int params; a yaml double (`width: 2.0`) throws `parameter 'height' has invalid type: {integer} → {double} not allowed` and the planner/controller component constructors ABORT (the loader logs `Failed to load node 'planner_server'` and the lifecycle manager waits on `planner_server/get_state` forever). Values are metres: `width: 2` / `height: 2` (local), `width: 24` / `height: 24` (global).

**SLAM rotation-smear tuning (2026-09-19, live-verified):** the first slam_toolbox map came out
as incoherent dust (852 scattered wall cells; a live scan correlated with it at only 33% at ANY
rigid offset — while the LDS itself repeated scans to 1-3 cm parked). Root causes were the drive
chain, not the matcher: **no deskew** (each scan covers a full ~0.2 s lidar revolution, so a turn
at `w` rad/s smears every scan by `w·0.2` rad — teleop's 3.0 rad/s clamp = 34°/scan) + the
low-duty motor stall (above) feeding jerk priors. Fixes, all in config and live on the board:
`drive_max_ang 3.0 → 0.8` (robot.yaml), `rotate_to_heading_angular_vel 1.5 → 0.5`,
slam `minimum_travel_heading 0.17 → 0.35` (fewer motion-blurred graph nodes; the matcher still
runs per scan for heading tracking) and `correlation_search_space_dimension 0.5 → 0.8` (±0.4 m
karto search vs the unverified wheel scale; ~2× correlation CPU, slam idled ~3.5% of a core
before). After the fix a fresh map scored **96-98% live-scan-to-map wall-hit at zero offset with
a sharp ±5° falloff** (was 15-33%, no coherent peak); after a stall-contaminated out-and-back it
held 0.84-0.86 (one bad-lock strip from the FWD jerk; the reverse run tracked map-pose-to-odometry
exactly). **There is no deskew in slam_toolbox 2.6.10 — keep every rotation rate (teleop clamp,
RPP cap, skill motion) tied to the `w·0.2 rad/scan` smear budget, and note the map resets on any
`nano-slam` restart (no `map_file_name` is configured — restart IS the map clear).**
- **Dead web-UI features** (scrapped per the plan): the old Map hero view + click-to-goal, wall guard + keep-away bubble, map buttons (Home/Save/Clear/Self-test), no-go brush, the Motion-chain and EKF cards, slam_nav/track_* sliders, LDS idle auto-spin-down (set Spin target 0 manually on the Lidar card instead). **The Map view + click-to-goal + Locations were REBUILT 2026-09-15 — see the block below.** (The LDS idle spin-down was itself REBUILT 2026-09-21 in `web_control/telemetry.py` — see the "LDS idle spin-down + jam guard" section.)

**Map view + click-to-goal REBUILT (2026-09-15, dev-verified):** the web Map hero view is back on top of Nav2, built resource-light (the 1 GB board budget):
- **`GET /map` HTTP route** (NOT the SSE frame): telemetry lazily subscribes to slam_toolbox's `/map` (OccupancyGrid, **transient-local QoS** → the current grid arrives the instant a browser connects), caches ONE copy (`telemetry._on_map` → `get_map_payload()`), and `_serve_map` serves it in the old slam_nav blob wire format: one JSON header line (`w,h,res,ox,oy,t`), `\n`, then raw int8 cells (-1 unknown / 0..100; row 0 = origin_y). The browser polls it at **1 Hz** while the Map view is on (slam_toolbox republishes at most every `map_update_interval: 5.0` s), so worst case ~230 KB/s per open browser; idle cost zero (subs drop with the other browser-only subs after `SUB_LINGER`).
- **`GET /local_costmap` + `/global_costmap` HTTP routes (2026-09-22):** Nav2's costmaps overlaid on the web map — the same `/map` plumbing (lazy browser-only OccupancyGrid subs in `_make_subs`, one cached copy each, `_serve_costmap` blob route). Differences that matter: (a) the subs are **VOLATILE on purpose** — Nav2's costmap publisher durability has varied across releases and a TRANSIENT_LOCAL request against a VOLATILE publisher is a silent QoS incompatibility (volatile is compatible with either; the page just waits ≤1 s for the next 1 Hz publish); (b) cells are Nav2 **COSTS** 0..255 (253 inscribed, 254 lethal, 255 unknown), not -1/0/100 occupancy, and the header carries `kind:"costmap"` so the page shades a yellow→red heat ramp (solid red = lethal) instead of the wall ramp; (c) the **local costmap lives in the odom frame**, so `_on_costmap` re-projects its origin into the map frame via TF `map→odom` (at the grid's stamp, `Time()` fallback) and the header carries `yaw` — the grid axes' rotation in the map frame — which the page applies as a rotated canvas transform (a TF miss keeps the previous cache: a mis-placed overlay is worse than a stale one); (d) the page's **Costmap toggle** (Map card, default off) picks local (2×2 m rolling window = live obstacle sensing) or global (static+inflation planning space) and polls at 1 Hz, lifecycle-tied to the Map view. Requires the container params-file fix above (`always_send_full_costmap: true` on BOTH costmaps — without it the global costmap switches to incremental `_updates` after the first grid and the full-grid poll would freeze).
- **SSE `f.nav` field (5 Hz, tiny)**: `{pose:[x,y,th]|null, goal:[x,y]|null, status:"idle|planning|navigating|canceling|arrived|failed", inflation:0.25, map_age, tf_laser}`. `pose` = a `tf2_ros` `Buffer`+`TransformListener` lookup `map→base_link` (lazy — created with the other browser-only subs, `unregister()`-ed in `_drop_subs`; None when the TF chain is down). `status` = the LAST entry of `/navigate_to_pose/_action/status` (GoalStatusArray → `NAV_STATUS` map; on terminal codes 4/5/6 the goal mirror clears so the ring doesn't resurrect). `goal` = the mirror of the last published `/goal_pose` — `publish_json` records it AND `WebServerNode._publish_skill_action`'s direct goal publisher calls `telemetry.note_goal(x,y)`, so browser clicks, Locations Go, and skill go-tos all light the chip. `map_age` = seconds since slam_toolbox last published `/map` (null = never arrived) and `tf_laser` = 0.0 when the static `base_link→laser` TF resolves / null when it doesn't (nano-tf down) — both feed the Map card's **feeds-health strip** (below). `f.lds` also gained an `age` field (monotonic seconds since the last `/lds_*` arrival; null = never) so a lingering `rpm>0` from a dead ESP32 link reads as stale instead of green.
- **Cancel = `POST /nav/cancel`**: a lightweight `CancelGoal` **service client** on `navigate_to_pose/_action/cancel_goal` (NOT a full ActionClient) with an empty `GoalInfo` = cancel-all per CancelGoal.srv; resets the goal mirror + chip immediately, fire-and-forget like `/param`. Client created before spin (thread-safe).
- **Page (index.html)**: `#view-map` hero canvas + a trimmed `#ctlMap` hero card (Motion toggle, status chip, ✕ Cancel, Sharp-walls toggle, **feeds-health strip**) + a Sensors "Map (Nav2)" readout card + a rebuilt **Locations card** ("Save spot" with NO x/y falls back server-side to the TF pose; Go publishes like a click). The map IIFE ports the deleted panel: zoom/pan/pinch, click-to-goal (`panMoved` suppression, y-flip), sharp/linear wall shading (≥70 = solid black), trail + robot + goal overlays, an **inflation bubble** (dashed circle, radius = `f.nav.inflation`), and a click-feedback ring.
- **Feeds-health strip (`#mapFeeds`, added 2026-09-17)**: five green/red dots in the Map card — `ESP32 · LDS · Odom · TF · SLAM` — one per link in the map-building chain, so a not-building map pinpoints the broken feed without shell access: **ESP32** = `f.esp.hb_age` < 3 s (heartbeat flowing); **LDS** = `f.lds.rpm` > 0 AND `f.lds.age` < 3 s (spin motor turning, `/lds_*` fresh — the age catches a dead ESP32 link whose last rpm lingers); **Odom** = `f.pipeline.feeds.odom` starts with "ok" (sys_monitor's pipeline watcher); **TF** = `f.nav.tf_laser` != null (the static `base_link→laser` TF resolves → nano-tf up; one extra tf2 lookup per tick, same lazy buffer as `_tf_pose`); **SLAM** = `f.nav.map_age` < 10 s (`/map` arriving — covers slam_toolbox itself AND the whole TF chain, since slam_toolbox drops scans on a missing TF and then publishes nothing). Diagnosis table: ESP32 red → power-cycle the coprocessor (documented wedge); LDS red + ESP32 green → set Spin target ≠ 0 on the Lidar card; Odom red → `/wheel_ticks` not flowing; TF red → `systemctl restart nano-tf`; SLAM red with all others green → `systemctl restart nano-slam`.
- **Motion toggle semantics (deliberate):** a map click with Motion OFF only marks the goal locally (browse mode — an accidental tap can't move the robot); Motion ON publishes `/goal_pose` and Nav2 drives. Locations **Go** is never gated (explicit intent).
- **Sim removed (2026-09-16):** the in-browser Sim tab/`sim.js` were deleted (the dev sim was the last consumer of the removed fetch overrides). Dev-PC testing is `scripts/dev_webui.py` (page + cognition, no ROS) — the Lidar/Map hero views render only on the robot now.
- **Deliberate omissions:** no server-side wall guard (Nav2's costmap enforces keep-away; the bubble is display-only, radius hardcoded `NAV_INFLATION_M=0.25` in telemetry.py to mirror `config/nav2/nav2_params.yaml` — if you tune `inflation_radius` there, change the constant: it's NOT read live, a startup get_parameters could race the nav lifecycle); no map Save buttons (map persistence is a `nano-slam` restart) — but the **✕ Clear-map button exists now** (added 2026-09-21): `POST /map/clear` (Drive tab's Map card) cancels any active goal, drops telemetry's cached grid + goal mirror (page shows mapWait), then runs `sudo -n systemctl restart nano-slam` **synchronously (20 s bound)** — 2.6.10 async mode has no clear service and no `map_file_name` is configured, so a restart IS the map clear. The sudo failure is REPORTED to the page (`{"ok":false,"error":...}` → the button alerts; fire-and-forget used to fail silently and slam kept republishing the old grid, which the 1 Hz poll pulled right back); the page's 503 `/map` poll now also re-shows mapWait + blanks the hero so a stale grid can't linger. It also arms `telemetry.note_map_clear()` — a lidar **rebuild window** (one `lds_idle_secs` period treated as recent motion) so a parked lidar wakes and the fresh map actually builds instead of slam sitting on no-map until someone drives. Needs the scoped sudoers rule (`deploy/sudoers/nano-power`, install via sbc-setup.sh or one `install -m 0440`); no no-go brush; no live Nav2 tuning (restart-only per the params note above). Nav2 doesn't expose a plan topic, so there is no plan polyline (the sim that drew one was removed 2026-09-16).
- **Verify after deploy:** Map view renders → click with Motion ON → `/goal_pose` published (app log `POST /publish /goal_pose`) → chip idle→navigating→arrived; ✕ mid-nav → chip back to idle; Locations Save (robot parked) → list shows the spot → Go → Nav2 drives; `/map` 503 while `nano-slam` is down (graceful placeholder in the hero); ✕ Wipe → `ok:true` (or a sudo/sudoers alert) → nano-slam restarts → fresh grid builds from the woken lidar (2026-09-21, after the silent-sudo-failure bug); **Costmap toggle** → local shows the 2×2 m window tracking the robot, global shows the full inflated planning space (2026-09-22 — remember the lidar parks on a quiet robot and slam then publishes nothing: wake it via a Spin-slider drag or `POST /publish /lds_target_rpm` before expecting grids).

## Deploying to the live board (from a dev host)
- One-shot deploy: **`scripts/deploy.sh [pkgs…]`** — `rsync`s `src/`+`scripts/` over the
  passwordless `ssh nano` key alias (`~/.ssh/config`, `Host nano`; override the target with
  `NANO_HOST`), colcon-builds on the board (optionally `--packages-select`), then
  `stack.sh restart`. No creds needed in the environment — key auth only.
  It also pushes the dev-made soul/bank (`memory/personality.json` + `phrases.json`, plus
  hand-edited `presence_chart.yaml`/`beats.json` if present)
  into the board's `~/.local/state/nanobot/` — **OFF by default**
  (`DEPLOY_SOUL=0`), so the robot keeps whatever personality it has evolved on its own.
  Set **`DEPLOY_SOUL=1`** to overwrite the board's persisted soul with `memory/` (discards
  accumulated trait drift).
- The dev host is native Ubuntu with the passwordless `ssh nano`/`scp nano:` alias above
  (`.nano-deploy.env`, gitignored, only still used by `.nano-askpass.sh` to bootstrap the
  key if it's ever missing). (Historical, from the old Windows/PuTTY deploy path: `plink -m
  <localfile>` sends the file's text as the remote shell's argv, so any `pkill -f`/`pgrep -f`
  pattern appearing in the script kills the controlling shell — `pscp` the script and run it
  by path instead. No longer relevant now that deploy is `ssh`/`rsync`-based.)
- `stack.sh restart` is now `systemctl restart nano-robot.target` — systemd owns
  stop/kill/verify, so the old "stale process serving old code" failure mode (and the
  heal-timer duplicate-node race) is gone by construction. If a change "doesn't take",
  check `journalctl -u nano-app` (etc.) and `systemctl status nano-robot.target`.
- The board has only ~1 GB RAM and a 7 GB rootfs — watch memory and disk. Don't run
  heavy compiles on it.

The rest of this Architecture section describes the OLD slam_nav/EKF pipeline — kept as tuning history; do not treat its params/topics as current.
**HTTP teleop gotcha (2026-09-11):** `POST /drive` parses the raw body with `json.loads()` and IGNORES `Content-Type` — any scripted POST with `{"v":..,"w":..}` works (curl included). BUT a single POST only pulses the motors ~0.5 s (web `drive_timeout: 0.6` + ESP32 `CMD_TIMEOUT_MS` 500): the joystick re-POSTs at 10 Hz while pushed. Scripted teleop must re-POST within ~0.5 s or the wheels just twitch. Also: the ESP32 can wedge into a no-motion stall after hard stop/turn sequences — heartbeat/LDS keep running while encoder counts sit frozen (pattern: ~1-2 s of motion then silence) — that's the documented physical power-cycle case, not a nav bug.
Sensor chain: ESP32 (`/wheel_ticks`, signed by commanded wheel direction in firmware) → `wheel_odometry/encoder_node.py` → `/odom` → robot_localization **EKF** (`src/robot_bringup/config/ekf.yaml`, fuses `/odom` + `/imu/data` → `/odometry/filtered` @15 Hz) → `slam_nav` (`odom_topic = odometry/filtered`, `imu_yaw_sign: -1` in `robot.yaml`).

IMU heading wiring (fixed 2026-08-10): `imu_driver` publishes `/imu/data` with **`frame_id: base_link`** (the driver pre-rotates via the mount matrix + lever-arm-corrects into the chassis frame, so `imu_link` gets dropped by the EKF — it had no TF and robot_localization silently ignored the absolute orientation). `robot.yaml imu_driver.yaw_sign: -1` aligns the BWT901CL heading with the wheels. `imu_driver/_configure_device()` forces the sensor's **range registers on every connect** (`0x29=0x03` → accel ±16 g, `0x2B=0x03` → gyro ±2000°/s — WitMotion codes are **inverted**: 0x00 = narrowest). If this unit boots at ±250°/s while the driver decodes ±2000, **every gyro axis and the device's fused heading come out 8× too big**, silently poisoning the EKF/SLAM heading (symptom: `/odom` yaw looks ~10× smaller than `/imu/euler` during a spin, on top of real tire slip).

Sensor-heading-vs-drive frame flip (`slam_nav heading_flip`, refined 2026-09-11): this unit's sensor head (LDS + IMU) is mounted **facing the robot's back**, so the SENSOR references (IMU yaw, lidar beam 0) sit 180° from the drive direction. The map is internally **consistent** (scan matcher keeps working, no lost-storms), but the global "which way is front" anchor was off: **autonomous nav drove the wrong way** (it rotated the physical back toward the goal then drove forward = away). Fix = rotate the SENSOR references only: `heading_flip: true` → `nav_node.py` adds π to `_on_euler` yaw + `_on_scan` beam angles. **The `/odom` WHEEL yaw is NOT rotated** — it comes from the wheel encoders and is the physical drive frame. (The flip was introduced as "one knob, three ingest flips" when the yaw source was the IMU-anchored EKF; the same-day retune that moved the pose yaw to raw `/odom` (`odom_topic: odom`, `use_imu_yaw: false`) left the π on the wheel yaw, so `pth` meant the physical **back**: the scan matcher locked the map's heading 180° from the drive direction and goals came out unreachable — the "map shifting / driving in circles" symptom, 2026-09-11.) Also `head_tol` (default 0.6 rad) bounds how far a scan match may correct the pose heading per scan vs the wheel prior, and `pos_tol` (0.35 m) + the odom-lock (below `recover_min_seen` the scan matcher may not move the pose at all *except on a genuine revisit*) do the same for position — a sparse/symmetric-map well can otherwise slowly WALK `pth`/the pose off the physical heading & position without any lost-storm. **Restart-only**; default `false` for the usual front-facing mount. The browser's **Lidar hero view** still draws raw `/scan.bin` beam 0 as "the nose" and was NOT flipped — if the head stays back-mounted it will look 180° off versus the SLAM map icon. *The 2026-09-11 "map still skews" sub-thread below is a slam_nav-era diagnosis record; its surviving validation items live in `docs/TODO.md`.*

What the page already surfaces for each stage:
- **Wheel ticks** — Coprocessor/ESP32 card: `/wheel_ticks` L/R counts, tick Hz, heartbeat, stray-tick diagnostic + reset.
- **Odom** — Odometry card: `/odom` x/y/θ + publish-rate slider (`/wheel_odometry/set_parameters`).
- **IMU** — IMU card: `/imu/web` rate ("lost" beacon when stale), `/imu/euler` display, 3D mount indicator, spin check, drift tool, interference self-test, mag-cal scatter, 6-axis toggle.
- **SLAM/feeds** — map panel + mapStats line: mode/explored/match score/loc + `⚠ feeds: odom · imu · lds` staleness, read from the `/map` JSON header written by `slam_nav/nav_node.py` (`meta["feeds"]`; `-1` = never received; re-aged at read time in `index.html:1948`). The header also carries `mcmd` = last published `/cmd_vel` `(v, w)` (`slam_nav`), so a stalled feed vs. a commanded one is distinguishable at a glance.
- **EKF** — NOT shown standalone: `/odometry/filtered` only enters the UI indirectly via the SLAM feed-staleness. Any new EKF view must compare raw `/odom` vs `/odometry/filtered` vs SLAM map pose / IMU yaw.

**Low-compute occupancy rewrite (2026-09-12, deployed):** `slam_nav/occupancy.py` was rebuilt around a **2-bit packed ternary grid** (00=Unknown, 01=Free, 10=Occupied; four cells per uint8) instead of float32 log-odds — the *permanent* 1200×1200 @ 2 cm map persists/serves in ~360 KB, and total RAM is ~6.2 MB (packed cells + decoded state + int8 bleach counter + int8 distance transform + no-go) vs ~8.6 MB before. The old continuous log-odds ray-cast scoring is replaced by a **pre-computed integer chamfer distance transform** of the occupied mask (pure numpy, int8, exact within the 0.15 m `SUPPORT_RADIUS_M` kernel, cached per `rev`): `score()` is a rapid DT gather — integer in the hot loop. The matcher's coarse-to-fine search is **integer-only**: beam angles + candidate headings quantise to a **4096-bin cos/sin LUT** (`ANG_NQ`) in fixed point (2^16), so a per-candidate heading is a table-index shift followed by one `r_cells·lut >> 16`; no fp trig inside the inner loop. Scores are normalised to ≈"beams resting on structure" (`÷SCORE_MAX`), so the existing absolute gates (`min_match_score`, `recover_exit_score`, `loop_score`) and the relative `min_improve` keep their meaning **unchanged** — no re-tuning of those numbers was needed.

New behaviours to know about:

- **Hessian-degeneracy lock** — the final refine pass measures score-surface variance along x / y / heading and *locks any flat axis entirely to the odometry prior*. A long corridor or symmetric pocket can no longer pull the pose along a direction the map doesn't constrain (this is one of the pose-walk mechanisms the 2026-09-11 TODO suspected — now removed at the source).
- **Gated rasterization** — `integrate(..., rotating=True)` (nav_node sets it when `|vel_ang| > 0.20` rad/s) **locks the grid out completely during spins**; the matcher may keep refining, but a rotating mirror can't smear walls into the map. Verified on the board 2026-09-12: a full spin left single-cell walls intact.
- **Scan-preparation pipeline** runs before matching: `reject_dynamic` (1-D range-jump clustering → drops moving-leg clusters 0.05–0.2 m wide), `deskew` (per-beam odometry interpolation across the continuous mirror sweep, identity when parked), `decimate_points` (keeps hits ~0.05 m apart in wall space, corners survive) and `GridMap.conflict_mask` (drops beams whose endpoint lands in CONFIRMED free space). Deskew + decimation always run; the two dynamic-obstacle stages sit behind the **opt-in `dynamic_reject` param (default `false`)** — a thin static pillar looks exactly like a leg to the range-jump test, so leave it off in cluttered rooms.
- **Velocity-scaled search** — `vel_scale()` / `search_window()` on the grid widen the match windows AND the `pos_tol`/`head_tol` authority gates with the wheel-velocity EMA (`_predict`, meters + `abs(rot)`/s): 1.0 when parked, up to 4×. Slow driving = tight trust; fast driving = proportionate forgiveness.
- **Bleach counter (integer)** restores the old log-odds "moved chair / opened door" responsiveness: an Occupied cell flips back to Free after `BLEACH_N` (16) beam passes through it.
- **`save()`/`load()` use the packed `cells`**, and old float32 log-odds `.npz` maps are **imported automatically** (seen & log>0 → Occupied, else Free) — a pre-rewrite save loads fine as long as the geometry still matches.
- Web export is byte-identical in meaning (−1 unknown / 0 free / 100 wall), so the Map panel, Sharp-walls posterize, and keep-away bubble all render as documented.

Tuning (occupancy.py): `SUPPORT_RADIUS_M` (0.15 = DT kernel width), `EXACT_B` (2 = extra points for an EXACT wall hit, keeping the peak sharp above the DT basin), `BLEACH_N`, `ANG_NQ`. nav_node: new `dynamic_reject` (whitelisted in telemetry.py), and `pos_tol`/`head_tol` are now velocity-scaled automatically. Validated live 2026-09-12: parked pose-vs-odom drift **0.000 m** over 90 s on a fresh map, zero lost-storms + crisp walls through a self-test forward/back/spin, and the rasterization lock kept the map clean during the spin. See the TODO just below for the still-open map-vs-room skew question — the rewrite removed the pose-walk *mechanisms* the old matcher was blamed for, so what remains points at the drive-chain / wheel-scale side (2026-09-12 live finding: the robot is exercised in a confined ~1×1.5 m area, so full-drive validation is still pending on open floor; surviving items live in `docs/TODO.md`).

**Map skew vs the room after manual driving (2026-09-11, slam_nav-era diagnosis record — the matcher retired 2026-09-14; surviving validation items live in `docs/TODO.md`):** after clearing a fresh map, driving one lap a few metres by hand, then parking, the map walls did NOT line up with the physical room *and* the SLAM pose did not sit back on the wheel chain. This was a **live, open problem** at the time — several candidate causes were found and fixed. **2026-09-12 live-driven diagnosis (real robot, dev-PC offline matcher):** the single biggest remaining cause found & fixed was the **boot-into-saved-map frame deadlock** (see the new block below); after that fix a loaded map self-matched to **1.9 cm / 1.8°** (was 22 cm / 58°), scans became trusted (`t 1`), and coverage grew. The residual manual-driving skew question now points at the wheel-scale/slip side (`/odom` scale, `wheel_trim`), NOT the matcher — the open-floor re-verification is tracked in `docs/TODO.md` (the last scripted lap was interrupted by an ESP32 no-motion stall; see the hardware note below). Verified facts and the trail:

- **scan↔map are internally consistent** (`overlap 0.82-0.98`, map `loc ok`, no lost-storms) — the map is a *valid-looking* room, just positioned/rotated wrong wrt reality.
- **`heading_flip: true` IS correct** (verified 2026-09-11: projecting the live scan with beam+π gives 31-36% wall hits vs ~5% without — the reversed-head frame is real).
- **Autocorrelation of the wall mask shows NO second shifted copy** (peak at the 2 cm single-cell thickness, no 5-15 cm companion peak) — so it is not "two overlapping masks", it is **one mask drawn at a pose that drifted off the wheels**.
- **The pose disagrees with `/odom` while parked** (observed several times): `map (0.05, -0.16, yaw -1.38)` vs `odom (0.01, -0.08, yaw -1.41)` (≈9 cm); earlier after a 0.5 m goal drive the gap was **0.35 m / 0.60 rad**. The map is drawn at `pth/src pose`, so a pose that doesn't return to the wheels = a map that doesn't match the room.
- **What was fixed & deployed** (all live on the board, all tensioned toward "converge on the first trace, never walk"):
  - sparse-map `_on_scan` wheel-authority gate now: `coverage ≥ recover_min_seen` → full `pos_tol`/`head_tol` (unchanged); `overlap ≥ min_overlap_ratio` re-visit below that → position snap capped at NEW `pos_tol_sparse` (0.12 m, was 0.35 = too generous, seeded the 35 cm walk) + heading capped at `head_tol_sparse` (0.15 rad, not 0.6 = seeded a 0.6 rad rotational smear); else pure odom-lock (unchanged). `pos_tol_sparse` whitelisted + live-tunable.
  - `_maybe_loop_close` now accepts a wide re-match that only *improves over the offset-free wheel prior* (was: absolute `loop_score ≥ 4`, which never fired on a fresh map). It fired once (19:26 drift 0.62 m) — but `loop_alpha: 0.1` bled only ~6 cm per event, so it cannot out-run the drift in a single drive.
- **Open items moved to `docs/TODO.md` (2026-09-16):** the surviving validation work — the `/odom` wheel-scale vs a measured rollout (`wheel_trim`/scale compensation) and the open-floor check that a clean second lap must not paint a shifted mask (park → pause: map pose back on wheel-integrated odom within a few cm, re-checked only on a cleanly-built fresh map). The slam_nav-specific knobs in the old list — `loop_alpha 0.1` → 0.3–0.5 + tighter `loop_apply_thresh`/more frequent `loop_probe_every`, the rotated-seed `rot_from`/`_seed_pth` pth-vs-odom check, `pos_tol_sparse`/`head_tol_sparse` retunes — retired with slam_nav on 2026-09-14. (The struck-through "instrument which scan moves the pose" item was DONE 2026-09-12: the per-scan `scan->` breadcrumb below, kept — the 9 cm gap was the boot-into-saved-map frame mis-load, not a residual walk.)

**Boot-into-saved-map FRAME deadlock — the big 2026-09-12 "map shifted+rotated" bug (FIXED, live-verified):** `occupancy.save/load` used to persist **only the occupancy cells** (no frame metadata). On boot the nav node loaded the grid but `rot_from`/`_seed_pth`/`_seed_odom` stayed at their init defaults (the seed path that anchors them only runs for FRESH maps). Since odom is continuous across a nav-only restart (the sensor hub keeps publishing), the loaded grid was then interpreted in the WRONG odom frame: `_predict` rotated odom deltas by `rot_from=0` while the walls were painted under `R(old_seed_yaw)`, so EVERY scan's best match sat a fixed ~0.3 rad / 0.2–0.4 m away from the wheel-anchored pose. The sparse trust gate (pos_tol_sparse 0.12 / head_tol_sparse 0.15) then rejected that correction forever (`t 0` in the breadcrumb), coverage froze at the loaded value, and because global coverage (≈0.3% for a small room) < `recover_min_seen` (0.25 = 144 m² of the 24 m grid!) the lost-counter could NEVER fire → no boot-locate → permanent mismatch. **Fix (all in `occupancy.py` + `nav_node.py`):**
  1. `save()` now also persists the frame anchors: `rot` (map-vs-odom yaw) + the seed tuple `s0-s5` = (odom x, odom y, odom yaw, seed_dx, seed_dy, seed_pth) — i.e. WHERE the robot was in both frames when the map was anchored. `load()` restores them (older npz without the keys → zeros → treated as "drawn in the current odom frame", which old saves effectively were).
  2. On boot-into-saved-map `_on_scan` re-anchors: `pose = (seed_dx, seed_dy) + R(rot_from)·(odom − seed_odom)`, `pth = seed_pth + (odom_yaw − seed_odom_yaw)`, then enters recovery (kidnap) so the matcher can still correct e.g. a carried robot. Live-verified: loaded map self-match residual **1.9 cm / 1.8°** (was 22 cm / 58°), scan trust restores (`t 1`), wall cells grow.
  3. A recovery/load path that found the map too EMPTY now checks **occupied-cell count** (`grid.occ_count()`, local structure) instead of global coverage fraction — a real 1.5 m²-room map reads 0.3 % coverage but has hundreds of wall cells and MUST be kept/matched; a truly empty grid (0 walls) is discarded → fresh seed.
  4. `_on_scan`/`_on_clear_map` gained a per-scan matcher breadcrumb (`scan->cov … | t … | pose | anch | cand | sc/imp/ov | dpos/dhead`) throttled to 1 Hz — the TODO's "instrument which scan moves the pose" ask; keep it, it makes the next diagnosis log-only.
  **Old saved maps without frame metadata cannot be repaired in place** — clear the map once after deploying this fix so the seed tuple is written fresh. *(The remaining `loop_alpha 0.1` loop-closer knob died with slam_nav, 2026-09-14 — see `docs/TODO.md`.)*

**Empty-map relocalize guard (`recover_min_seen`, added 2026-08-11):** when localization is lost on a near-empty grid there is nothing for the scan matcher to lock onto, so a persistent low score is *expected* (fresh map / just cleared) rather than evidence of drift — and the relocalize in-place spin (`recover_spin`) only smears the grid + drains the battery. `slam_nav` now suppresses the spin whenever map coverage (`grid.coverage()`) is below `recover_min_seen` (default **0.25**): it holds pose, logs `map too empty (seen X%) to relocalize — holding pose, not spinning` (throttled 5 s), and keeps running the recovery matching in `_on_scan` so the spin resumes automatically once the map fills past the threshold. Live-tunable via `/param` (`recover_min_seen`), whitelisted in `telemetry.py`. Verified 2026-08-11: pre-fix a goal-click on a just-cleared map stormed 155 `localization lost` cycles in ~100 s spinning at 0.6 rad/s; post-fix the same scenario logs the guard line and publishes **zero** cmd_vel. `0.10` was too low (a 0.106-covered smeared map still stormed) — hence `0.25`. Threshold choice is a map-density judgment; raise it for sparse rooms, lower for dense ones. NOTE 2026-09-12: the drop-to-remap branch now keys on `occ_count() < 4` (structure), because global coverage can never reach 0.25 for a small-room map.

**EKF heading is now pure gyro-z (fixed 2026-08-11):** the BWT901CL's *device-fused* yaw (0x53) develops a decaying bias transient for a minute+ after any real motion (measured: +450° in 76 s post-drive while raw gyro-z stayed ~0) — fusing it into the EKF made the fused heading, and thus SLAM's motion prior, drift for a minute after every drive. Fix: **`ekf.yaml imu0_config` yaw fusion is OFF** (only `vyaw` = gyro-z is fused for heading, `Ax/Ay` for tilt); heading = gyro-z integration from 0 at boot. Verified: rest 120 s EKF yaw delta +0.03°; drive EKF −178° vs scan −168° (gyro ~1.05× over-rotates; SLAM absorbs it); post-drive 70 s EKF holds −0.53° while device fused yaw runs +101° (now display-only, feeds the web 3D + IMU card, NOT navigation). The web drift tool reads EKF yaw (`telemetry.py _drift_yaw_deg()`), roll/pitch stay on `/imu/euler`. **Config drift warning (MOOT):** the EKF/robot.yaml changes were applied on the board directly — the sync concern ended with the 2026-09-14 migration that deleted `ekf.yaml`/`use_imu_yaw` entirely (no references remain in the dev repo). One knock-on tradeoff: nav_node's runtime slip/sign diagnostics now compare odom-vs-odom (IMU yaw no longer feeds nav_node), so those checks are inert.

**Boot-into-saved-map 180° heading flip (`recover_min_move`, fixed 2026-08-11):** saved maps carry **no heading metadata** (`occupancy.py save/load` stores only `log/seen/forb/n/res`), and on load `pth` stays at the default `0.0` — the IMU/odom seed only runs for fresh maps. With no absolute heading, the full-grid 16-heading `relocalize` is the only way to find the robot, and on a **symmetric or sparse** room the correct and the 180°-flipped heading score identically — so the `recover_confirm` gate re-scoring the SAME pose while the robot is still/spinning-in-place **self-confirms the wrong flip** ("robot thinks its front is its back"; the EKF/drift-tool/teleop are all still correct — only the SLAM map pose `pth` flips, and the web map arrow reads `pth` from the map JSON header). Only **translation** changes what the scan sees, so recovery candidates may now be adopted AND confirmed only after the robot has actually moved ≥ `recover_min_move` (default **0.10 m**) from where recovery started; a still robot holds the odom-derived (physically-correct) heading and stays "recovering" until nudged (or the 12 s timeout runs on the best estimate). Live-tunable via `/param` (`recover_min_move`), whitelisted in `telemetry.py`. A pure in-place recovery spin does NOT break the 180° ambiguity by itself.

**"Who told the robot to move/spin?" diagnosability logging (added 2026-08-11):** every driving decision is now in the logs so a mysterious spin can be traced without live-tracing:
- `slam_nav._send` throttle-logs the first non-zero `/cmd_vel` after an idle stretch with its mode: `cmd_vel -> v X w Y (mode recover|goal|other)` (2 s throttle); the last published command (incl. stops) is always in the map header as `mcmd`.
- `slam_nav._on_params` logs `enable_motion -> True/False (was …)` transitions (`cmd_vel now live/blocked`).
- `web_server` logs `POST /drive v X w Y (web teleop)` (2 s throttle) and skill actions (`skill action /cmd_vel lin=… ang=… for …s`, `goal_pose -> '<name>' (x, y) (skill)`).
- `telemetry.publish_json` logs every discrete map-button/goal publish (`POST /publish /goal_pose …`, `/slam_nav/go_home|save_map|clear_map`, `/selftest`, `/reset_ticks`); `set_param_json` logs `POST /param node/name = value`.

**Web UI tab/group layout (reorganized 2026-09-16, driving/SLAM cards moved Drive-ward 2026-09-21, all IDs unchanged — JS untouched):** the tab bar is **4 tabs**: `Drive` · `AI · Speak` · `Sensors` · `System` (the old separate Speak tab was merged into AI; `data-tab="ai"`/`panel-ai` kept its ids, `panel-speak` is gone). Cards are clustered under `.group` header dividers (styled in `style.css`): **Drive** = 🕹 Driving (the Drive card: joystick, canned moves) + 🗺️ SLAM & navigation (Map (Nav2, incl. the ✕ Clear-map button), Locations, Odometry) + ⚙️ Drive hardware (Coprocessor (ESP32) — wheel encoders/trim/suspension + the **live PID KP/KI/KD sliders**, moved out of Sensors 2026-09-21, so operating + tuning the drivetrain needs one tab); **Sensors** = 🗺️ IMU & orientation (IMU — sensor health/calibration, kept out of Drive deliberately; its own hint says "IMU health, not navigation") + 📡 Lidar (the Lidar (LDS) card — scan health + spin motor; bounced back to Sensors 2026-09-21) + 👁️ Vision (Camera (GPU vision)); **System** = Connection, **System health** (moved out of Sensors), **⚙️ Actuators** (Cooling fan, Line lasers — moved out of Sensors 2026-09-21, next to the CPU temp the fan tracks), View, Health events, Stress test, Power. When adding a card, put it in the right group and re-check the hover-hint cross-references that name other cards/tabs (e.g. the IMU interference hint points at the Sensors tab's Lidar card and the System tab's Cooling fan card, the stress-test hint at System health).

**Served-page state (RESOLVED 2026-08-10):** `src/web_control/web/index.html` is a **self-contained, pure-SSE** page — no rosbridge/roslib. The main inline script (`app.js`-derived) opens an `EventSource("/telemetry")` and drives everything off the SSE frame + `POST /publish|/param|/drive`; `oled.js` is inlined for the OLED mirror. Brain readouts (`purpose`/`task`/`experiments`) come from the SSE `f.*` fields, NOT `GET /purpose` etc. (those endpoints don't exist — an initial `404` from `pollBrainHttp` before the link is up is harmless). The old stale ROSLIB block and the SSE split files (`app.js`, `map.js`, `oled.js`, `chrome.js`, `sim.js`, …) were superseded — **do not** try to `script src` them or wire a websocket. When editing the page, keep it self-contained SSE and follow the pattern of the existing blocks (map/chrome/sim/motion-chain/EKF/slam-tuning).

**Vermap map buttons wired through the SSE gateway (fixed 2026-08-11):** the Map card's Clear/Home/Save/Test/Stop buttons use `pub()`/`sendDrive()` (NOT ROSLIB): `mapClear` → confirm → `pub("/slam_nav/clear_map", true)` + the page resets `mapGoal`/`mapPlan` locally (telemetry only emits `f.plan` while non-empty); `mapHome` → `pub("/slam_nav/go_home", true)`; `mapSave` → `pub("/slam_nav/save_map", true)`; `mapTest` → `pub("/selftest", true)`; `mapStop` → `sendDrive(0,0)`. All four ROS topics are **bare** names in `telemetry.py` (`go_home`/`save_map`/`clear_map` — see the web-publish-topic-namespace gotcha), whitelisted as `/slam_nav/…` keys for the browser POST. `_on_clear_map` (nav_node.py ~1309) sets `_nogo_dirty=True` so `/dev/shm/nano_nogo.bin` is rewritten to a `count:0` mask, drops `_goal`/`_goal_is_frontier`/`_path`, and publishes an empty plan — else the robot keeps driving toward a stale goal on the fresh map. The blob is a JSON header + a `w*h` bool mask (1440072 B at the 1200×1200 / 2 cm default grid; 230472 B at the old 480×480 / 5 cm); `/map/nogo` is served by `web_server` via `_serve_shm`.

**Map zoom/pan/fit (added 2026-08-11):** the Map hero view zooms/pans via a **CSS transform on the full-res canvas** — `#mapcv` keeps its native grid resolution and `draw()` stays cheap; `applyTf()` in the map panel IIFE (index.html ~3220) writes `translate(tx,ty) scale(zoom)` with `transform-origin: 0 0` and the canvas CSS no longer centres it (`top:0;left:0`). `zoom=0` means auto-fit: `fitMap()`/the ⤢ Fit button reset it, and the first frame + `window resize` re-fit. Zoom sources: `+`/`−` overlay buttons (`.map-zoom`), mouse wheel (anchored to cursor, `zoomAt(factor,mx,my)` keeps the map pixel under the cursor fixed), and two-finger pinch (pointer-map distance ratio). Pan = one-pointer drag; pointer capture is on **`cv`** (not `#mapWrap`) so the release-`click` still targets the canvas for click-to-goal — a `panMoved` flag (set when drag >4px) suppresses the stray goal after a pan. Click-to-goal coords still work because `getBoundingClientRect` reflects the scale and pan (`(e.clientX-rect.left)*cv.width/rect.width` = `(e.clientX-rect.left)/zoom`).

**Map grid + wall shading (2026-09-11):** the occupancy grid is now **2 cm/cell (1200×1200 @ 24 m)** — `robot.yaml slam_nav.map_resolution: 0.02` (was 5 cm), matching the raw 1° lidar point spacing (~1.7 cm @ 1 m) so the map renders with at least as much wall detail as the lidar hero view. RAM ~6.2 MB since the 2026-09-12 2-bit occupancy rewrite (was ~8.6 MB; see the Low-compute occupancy rewrite block above), blob 1.44 MB/write at map_write_rate. Because the planner + global relocalize walk **fixed metre spacings**, `plan_downsample` and `recover_global_step` bumped **4→10** to keep their coarse cells at the same **0.20 m** (planner/nav behaviour byte-identical to the old 5 cm layout — do NOT bump them back to small N "for more planning detail"; that changes pathing). A saved 5 cm `.npz` will not load on the 2 cm grid (`load()` rejects geometry mismatch) — the robot starts a fresh map. The board's `slam_nav up:` log confirms `1200x1200 grid @ 0.020 m`. Display: the map panel downscales with **nearest-neighbour at/under native res** (`imageSmoothingEnabled = z > 1`) so fit-view walls don't smear into a bilinear blur, and a **"Sharp walls" toggle** (`#mapShade`, Map card controls) switches wall shading between the default **hard posterize** and the **flat linear** ramp. Sharp mode snaps cells at/above a single lidar hit (`occupancy ≥ 70`) to **solid black**, with a thin 45-69 transition band and a pale ramp below — crisp `black wall / white floor / grey void` like the raw scan (the old √ gamma reached only grey ~42 at the 70 band, near-invisible against the 44-grey unknown void, so walls blended into the void on sparse fresh maps; fixed 2026-09-11). Toggling re-shades the last grid in place (`shadeMap()`) — no re-poll.

**Web-serving chain:** `web_server.py` serves the package's installed `web/` dir, symlinked `install/web_control/share/web_control/web/ → build/web_control/web/ → src/web_control/web/`. Edit `src/web_control/web/index.html`, restart the app — picked up live.

**Line lasers (updated 2026-08-18):** the Sensors tab → Actuators group's "Line lasers" card has **two** 0–255 sliders (Laser 1–2). They POST `/laser_pwm` `[v1,v2]` (Int32MultiArray) via `pub()` → telemetry whitelist (`telemetry.py _mk_laser`, clamps 0..255, 2-element) → bare ROS topic `laser_pwm` → ESP32 `laser_cb` parses the empty-layout CDR body (`hdr(4) | dim_len | data_offset | data_len | int32×2` — values at payload +16/+20, no pad; verified byte-for-byte against rclpy `serialize_message`; the `hdr(4)` matches the same `+4`-payload convention `cmd_cb` uses). Pins **GPIO 23/32** (`LASER1..2_PIN`), 10-bit duty = `value*1023/255`. Lasers ride **LEDC ch 6–7** (high-speed group). **Laser 3 was removed 2026-08-18**: its GPIO13 stayed stuck full-on through every PWM peripheral tried — LEDC low-speed ch 8 was a silent low-speed-channel failure (every `ledcWrite()` incl. 0 drove the pin high) and MCPWM couldn't sink it either (board-side test: GPIO13 pad itself toggles clean LOW/HIGH, so the module is powered/wired independently of firmware). It's gone from firmware + web UI; GPIO13/LEDC ch8 are untouched. Publish-only (like `/motor_accel`) — no read-back, so a page reload shows 0 until the next drag. Lasers park at 0 while the SBC link isn't `alive` (same `linkAlive()` gating as the fan/LDS) and resume at 0, not the stale pre-drop value. Verify the CDR parse in `laser_cb` matches the serialized layout if the message definition ever changes.

**Wall keep-away distance + manual-driving wall guard (added 2026-08-11):** the distance the robot keeps from walls is `slam_nav stop_distance` (default 0.25 m, already live-tunable). The Map card now has a **Keep-away** slider + a **Wall guard** toggle:
- The slider POSTs **both** `slam_nav/stop_distance` (autonomous reactive stop) and `web_control/wall_guard_distance` (manual guard) so one control drives both; the Sensors-tab "Stop distance" slider (`navStopDist`) also writes both now.
- The map **draws the keep-away bubble**: `nav_node._write_map` puts `wd` (= `stop_distance`, metres) in the map JSON header and the map panel draws a dashed circle of radius `wd/res` around the robot — grey = autonomous distance, blue = manual guard ON, orange = guard actively refusing a drive right now. `f.wall_guard` in the SSE frame (`web_server.wall_guard_state()`) drives the toggle/slider/bubble colour.
- **Wall guard** (`web_control` params `wall_guard_enable`/`wall_guard_distance`, whitelisted in `telemetry.py`): when ON, `web_server.drive()` AND its 10 Hz `_drive_tick` keepalive run `_wall_guard(v,w)` before publishing `/cmd_vel` — the linear axis is refused (published as 0) if the robot would push **toward** a wall closer than the distance; rotation always passes so the driver can turn away. It reads the SLAM pose (`telemetry._slam_pose`, must be < `WALL_GUARD_STALE`=1.5 s old) + the `/dev/shm/nano_map.bin` blob (cached on mtime/size, parsed via `numpy.frombuffer`; row 0 = origin_y, same orientation as the browser map), probes `WALL_GUARD_PROBE`=0.06 m along the direction of travel, and blocks when that probe's nearest-occupied distance < the bubble. Direction-aware on purpose: a robot next to a wall can still drive parallel/away (a corridor narrower than 2×dist stays drivable). Inert (never worse than a no-op) when the guard is off, pose is stale, or the blob is unreadable — the guard errs **permissive** when localization is lost, it can never strand the robot. Blocks log throttled as `wall guard: forward/reverse drive blocked (X.XX m to wall, keep-away Y.YY m)`. Head-on the probe adds a ~0.06 m margin beyond the drawn bubble (safe, conservative). Implementation is entirely in `web_server.py` (no new ROS wiring); the guard does NOT apply to skill/`cmd_vel` actions or autonomous nav (those already clamp via slam_nav).

## Gotchas

- **`stack.sh restart` is unreliable** — can leave stale processes holding ports. Clean `down` → verify via `/proc` → `up`.
- **A hung `nano-nav-loader` oneshot blocks the whole `nano-robot.target` start job** (seen 2026-09-21: an interrupted `systemctl restart` left the loader's `nav2.launch.py load_only` waiting on a mid-bounce router forever; the target's start job sat in "start waiting" behind it). The loader is a plain user process — `pkill -f nav2.launch.py` clears the block (CAREFUL: quote the pattern so the ssh shell's own cmdline doesn't match — the AGENTS plink gotcha applies over ssh too), then `sudo -n systemctl start nano-robot.target`. NOTE the sudoers rules are EXACT-command matches: `systemctl start nano-robot.target` is allowed, `--no-block` appended makes sudo ask for a password.
- **A missing `/dev/shm/nano_nogo.bin` right around a restart is teardown, not a bug** — during a `deploy.sh`/`stack.sh restart` window the old processes + the `user@1000` session teardown transiently remove blobs (even a hand-created dummy `/dev/shm/nano_nogo.bin` vanished within that window). Nothing in code deletes it (no `os.remove`/`unlink`/`os.replace` to that path anywhere; tmpfiles.d/crons/timers are clean). Re-check once the stack is quiet before hunting a "deleter" — the blob persists indefinitely after a clear once settled. See `web-map-clear-buttons-nogo` memory.
- **`brain_timeout` must stay well above `reflect_period`** (invariant: timeouts shorter than the reflection gap cause the chart to revert accumulated drift).
- **Heavy data paths bypass the ROS graph and the SSE frame:** `/scan.bin` (+ the other `nano_*` blobs) live in `/dev/shm` and are served over HTTP; `/map` is served by the `GET /map` HTTP route (telemetry holds a transient-local sub to slam_toolbox). (rosbridge was removed 2026-07-06 — there is no bridge at all.)
- **`rmw_zenoh` ordering:** a node started before `rmw_zenohd` runs islanded (won't appear in the graph).
- **Zenoh CLIENT sessions MUST disable shared memory** (2026-09-22, hit live): a
  client-mode session config without `transport: { shared_memory: { enabled: false } }`
  crash-loops at rmw_init on the board with `Failed to create POSIX SHM provider
  (OS error 12)` — app/slam/nav restart-looped ~55× each; only the first session of
  a boot (nano-sensors) survived. `.run/zenoh_client.json5` (written by `unit_exec.sh`)
  carries the disable flag; keep it if you regenerate the config. Cost of the fix: an
  extra copy on big loopback messages (/map, /scan) — negligible at their rates; the
  ESP32 serial link never used SHM anyway.
- **Python edits are live:** `--symlink-install` means edit `src/<pkg>/<pkg>/foo.py`, restart node = picked up. New modules import fine via egg-link.
- **nanobot-brain is pip-installed**: edit `src/nanobot_brain/` in the nanobot-brain repo, restart node = picked up (editable install).
- **Deleting a file under `src/web_control/web/` breaks the next colcon build** with `error: can't copy '...': doesn't exist` — colcon caches the setup.py `glob("web/*")` result in `build/web_control`. `rm -rf build/web_control install/web_control` once (the board needs the same after rsyncing such a deletion) and rebuild. Hit 2026-09-16 when the 8 dead split `.js` files were deleted.
- **`deploy.sh` does NOT push `nanobot-brain`** — the brain repo is a separate git checkout copied to the board at `/home/ibster/Nano/brain/src` (a `brain/src` PYTHONPATH entry, not pip-installed there). If you change the brain on the dev PC you MUST sync it to the board yourself: `rsync -az --exclude __pycache__ src/ nano:/home/ibster/Nano/brain/src/` **and** `rsync -az skills/ nano:/home/ibster/Nano/brain/skills/` (the skill catalogue — `resolve_skills_dir` looks for `<brain>/skills` next to `brain/src`). A stale brain silently breaks nodes at runtime with `TypeError: __init__() got an unexpected keyword argument ...` (e.g. `vision_diary_enable`, `nudge_looming_caution`, `chart_path`) — the glue (`mood_node`/`web_server`/`dev_webui`) and the brain must stay in lockstep.
- **`telemetry.py` `DiagnosticStatus.level` is `bytes` under rmw_zenoh** — `pipe.level` arrives as `b'\x00'|b'\x01'|b'\x02'`, which is NOT JSON-serializable and kills the entire app_hub (telemetry `_tick` runs on the executor, so an unhandled `TypeError` crashes the process → systemd respawn loop). Normalize to `int` at ingest (`_on_diag`). Any new raw ROS field put into the telemetry frame must be a JSON-safe type after passing through rmw_zenoh.
- **Callback exceptions on the executor = respawn loop** — any unhandled error inside a subscription callback (not just `DiagnosticStatus.level`) kills the whole hub process, so systemd `Restart=on-failure` looks like a boot loop. Real case (fixed 2026-08-10): `telemetry.py:_on_slam_pose` copied the Odometry layout (`msg.pose.pose`) onto the actually-`PoseStamped` `/slam_pose` → `AttributeError` every tick. Match the real message type (`/slam_pose` is `PoseStamped` = `msg.pose`). Same class: `nav_node.py:_on_scan` used a local `angles` bound only inside the lidar-geometry memoization branch → `UnboundLocalError` on the 2nd scan; the memoized alias is `self._ang_cache` — always use that.
- **Vision readouts are one atomic snapshot** — `gpu_vision` scalars are read via `snapshot()` (one `_lock` acquisition for all fields, added 2026-08-10). Don't revert to per-property getters in the 5 Hz telemetry build or the 10 Hz `_vision_state_tick`: that was ~20 lock round-trips/tick + cross-field reading skew.
- **BWT901CL gyro/accel range registers are inverted and NOT persisted** — WitMotion maps `0x00 = narrowest`, `0x03 = widest`: `0x29` accel (0=±2 g … 3=±16 g), `0x2B` gyro (0=±250 … 3=±2000 °/s). The device defaults to ±250°/s at power-on, but the driver decodes raw as ±2000 → **8× too big on every axis + the device's internal fused heading** (root cause of the 2026-08-10 SLAM lost-storms). `imu_driver/_configure_device()` re-writes `0x29=0x03, 0x2B=0x03` on every (re)connect exactly like RRATE — do NOT "simplify" them away, or the next power-cycle silently reintroduces a scaled heading.
- **A differential robot's body yaw rate is bounded by its wheel command** — if `/imu/euler` Δyaw vastly exceeds what the wheels could have rolled (`(ΔL+ΔR)/2·m_per_tick`, ARC of both wheels), suspect an IMU scale/sign error, not an encoder undercount. Wheel-odom translation is correctly scaled; expect only small tire-slip gaps on spins.
- **Low-duty turn commands can stall the wheels** — in-place turns map to tiny per-wheel speeds (±0.04 m/s at 0.5 rad/s) whose duty can stop the motors ~0.5-1 s in (wheels AND body both freeze mid-command; encoder counts + IMU plateau together). This is a drive-power issue, separate from sensing. The 2026-09-19 drive test sharpened it: with the flashed `MOTOR_MIN_DUTY 0.55` deadband the wheels still seized 1.4-2.4 s into EVERY command at 0.12 m/s, 0.25 m/s AND 0.5 rad/s spins (constant ~0.60-0.83 remapped duty), recovering only on a direction change — the firmware breakaway kick + 0.70 floor (above; flashed 2026-09-20) is the fix. Scripted teleop (`POST /drive`) still needs ~10 Hz re-POSTs; my 2026-09-19 test loop (~0.4 s period) kept `/cmd_vel` alive via web_server's 10 Hz re-assert, so freezes were real stalls, not cmd-timeouts.
- **`config/robot.yaml` is the single config source** — all ports, pins, rates, LLM params live there. Its `slam_nav:` block uses the ROS param layout (`slam_nav.ros__parameters.<name>`). Indentation must match sibling keys exactly: a block one space off parses as a *nested map* and the whole `slam_nav` section silently returns `None` to nav_node (only the in-code default saves you). Always sanity-check with `python3 -c "import yaml,sys; print(yaml.safe_load(open('src/robot_bringup/config/robot.yaml'))['slam_nav']['ros__parameters']['recover_min_seen'])"` after editing, and remember the running stack reads it via the `build/ → src/` symlink, not a copied install.
- **ESP32 link can wedge after a stack restart and needs a PHYSICAL power cycle** — after `stack.sh down/up` the coprocessor may never re-attach to the router's serial link (`/dev/ttyS1`): `esp32 DOWN: no heartbeat ever received`, `/wheel_ticks` silent, LDS motor dead (ESP32 drives its PID), scans stop. Service restarts, full `nano-robot.target` restarts, even a board `sudo systemctl reboot` do NOT reliably recover it — the firmware's auto-recovery watchdogs (`LINK_CONNECT_DEADLINE_MS`, `LINK_RX_TIMEOUT_MS` in `firmware/nanobot_coprocessor/src/main.cpp`) apparently can't re-sync a wedged UART. Symptom chain when it happens: `esp32 DOWN` → `lds DOWN: lidar not spinning` → `wheel_ticks SILENT` → map `feeds.scan: -1`. Diagnosis: `journalctl -u nano-sensors.service | grep -i esp32`, and confirm the router holds the fd (`ls -l /proc/$(pgrep -f zenohd-serial)/fd | grep ttyS1`). Fix = unplug/replug the ESP32's power. After a successful power cycle it comes back on its own (`esp32 UP after …`, `/wheel_ticks resumed`, `lds UP`), and `lds_idle_enable=false` via `/param` (or a Spin-slider drag) wakes the lidar if the idle controller has it parked. **2026-09-20 variant (open, see docs/TODO.md): a SNEAKIER partial wedge** — after a router restart the session re-attaches (heartbeat/ticks/LDS all flow) and SOME subscriptions still deliver (`/motor_pid` write→`/wheel_pid` readback flips), but **`/cmd_vel` specifically goes deaf** (observed with the web keepalive, `ros2 topic pub`, AND raw zenoh puts on the exact keyexpr) while a full ESP32 reboot restores it. **2026-09-21 firmware fix (FLASHED 2026-09-21 — deployed + robot live)**: the firmware now periodically (45 s, `SUB_REDECLARE_MS`) undeclares + re-declares ALL subscriptions (`SUBS` table + `subsRedeclare()` in main.cpp), refreshing the router's remote-sub table in place — the deaf window is bounded at ≤45 s and the workaround above dies once flashed. VERIFY after a few router restarts: /cmd_vel revives within ≤45 s if dropped (tracked in docs/TODO.md). Note for the next pico-API edit: zenoh-pico's Arduino build defines `ZENOH_C_STANDARD=99`, which compiles the `z_move`/`z_call` _Generic macros out — use the explicit generated functions (`z_subscriber_move()`, like the existing `z_config_move()`).
  **2026-09-21 pm VERIFICATION — the redeclare fix FAILED its verify; the reliable heal is the ping-watchdog reboot.** After the smoothness-pass-II flash + a `deploy.sh web_control` (full stack restart), the robot drove NOWHERE for minutes: ESP session half-attached (hb/ticks/LDS-flowing, `nano_esp32` visible as the /wheel_ticks publisher), but the router wasn't routing SBC→ESP at all — and the 45 s redeclare (which printed `subs re-declared (10)` on the console every period) did NOT revive delivery over that half-dead session. Diagnosis chain, all motion-free: gateway journal shows POST /drive landed → `ros2 topic info /cmd_vel -v` on the board shows only behavior+web_control subs → the console (USB) shows the ESP believes all is well. **Heal that worked (no power cycle): `sudo -n systemctl restart nano-robot.target` → pings stop → the ESP's own LINK_RX watchdog (8 s) esp_restart()s → fully fresh session BOTH ends → console shows clean boot + `zenoh CONNECTED` + `subs re-declared`, no declare failures.** Motion-free RX proof: POST `/motor_params [6, <current dither>]` (a physical no-op) → the console prints `drive params … saved to NVS` within ~10 s = the ESP received a put (telemetry's id gate had to widen to 0..6 first). LESSON: after any stack/router bounce, if the robot ignores /drive but hb/ticks flow, bounce the target once (a forced clean ESP re-handshake) before suspecting firmware or motors; the USB console + a no-op param echo discriminates without moving a wheel.
  **2026-09-21 pm II — redeclare DISABLED + drop triage is now remote.** The load-correlated ESP drops (deaf legs → ping-watchdog esp_restart, ~every test run) landed ON redeclare moments and the burst had already failed its one job, so `SUB_REDECLARE_MS` is now **0 (off)** — `subsRedeclare()` remains compiled for manual reuse. The firmware now publishes **`/esp32_reset` (Int32, 1 Hz, `esp_reset_reason()`)**: after ANY drop, `ros2 topic echo /esp32_reset` triages it — 9=brownout (power), 3=SW (the ping watchdog), 4/5/6=panic (code), 1/2=power-on/external — no console needed. (Context: the user reported "didn't have this problem before the PID update"; with battery+regs declared good and connections reseated, the open suspects are motor-noise coupling into the ESP feed/UART2 (the known ground-bounce family) vs the redeclare burst — the latter is now gone and `/esp32_reset` will attribute the next drop definitively. Same session: a full BOARD reboot also happened during power fiddling.)
- **`plink -m` on Windows:** the script text becomes the shell's argv. `pkill -f` patterns can kill the controlling shell. Fix: `pscp` script, run by path.
- **ESP32 firmware:** PlatformIO from dev PC (`pio run -t upload`). Don't build on the board. Tunables are `#define`s at top of `src/main.cpp`. **After ANY flash, expect the router's serial transport to be DESYNCED** (the ESP reboots mid-session and sends garbage; the router logs `Read error on Serial link ... Unexpected Init flag`) — the ESP then boot-loops on its 40 s connect deadline ("link not up within deadline") and only a `sudo -n systemctl restart nano-robot.target` (→ ping-watchdog ESP reboot → fresh handshake) clears it; the ESP's own retry loop cannot recover the half-dead router transport (hit 2026-09-21 pm twice). USB-tethering the ESP also means motors are unpowered in this setup (the ESP runs off USB) — flash first, then switch power, then test. **Also (2026-09-22): a raw console read (`cat /dev/ttyUSB0`) REBOOT-RESETS the tethered coprocessor** — opening the port asserts DTR/RTS and the dev board's auto-reset circuit pulls EN (board logs `esp32 DOWN 9s → UP`, ROM-banner garbage at 74880 baud shows on the console instead of the app). Never read the USB console raw while the robot is live; use `pio device monitor` (it manages DTR/RTS) or deassert dtr/rts explicitly if you must capture the boot banner.
- **Deploy soul overwrite:** `DEPLOY_SOUL=1` pushes `memory/` personality to the board, discarding evolved drift. Default is `DEPLOY_SOUL=0` (keep the robot's soul) — matching deploy.sh.
- **Board has ~1 GB RAM and 7 GB rootfs** — watch memory, don't run heavy compiles.