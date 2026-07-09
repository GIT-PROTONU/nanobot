# CLAUDE.md

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

## Layout (`src/`)
- `robot_msgs` — custom interfaces (ament_cmake).
- `robot_bringup` — launch files + **the single config `config/robot.yaml`** (all
  ports/pins/rates live here). `launch/bringup.launch.py` is the one node graph shared
  by the real robot and the Gazebo dev-sim (`sim:=true`/`rviz:=true` args) — see
  "Dev/prod ROS parity + Gazebo sim" below. Also holds the URDF (`urdf/nano.urdf.xacro`),
  the Gazebo world (`worlds/nano_room.sdf`), the `ros_gz_bridge` topic map
  (`config/gz_bridge.yaml`) and the RViz config (`rviz/nano.rviz`).
- `lds_driver_py` — **the LDS driver in use** (rclpy, publishes `/scan`; also writes a
  compact scan blob to `/dev/shm/nano_scan.bin` for the web UI — see `web_control` below).
  The blob writer is `scan_blob.write_scan_blob`, shared with `sim_hardware` so the
  Gazebo dev-sim writes byte-identical blobs.
- `lds_driver` — **abandoned** Rust/r2r LDS node; does NOT build against this
  RoboStack. Kept for reference only. **Do not try to build it** (see below).
- `wheel_odometry` — integrates `/wheel_ticks` (from the ESP32, or from `sim_hardware` in
  Gazebo dev-sim) into `/odom`+TF; no longer reads GPIO.
- `motor_control` — **retired** (PCA9685 path). The ESP32 owns `/cmd_vel`→motors;
  not launched by the systemd units/`bringup.launch.py`. Kept for the optional PCA9685
  LDS-spin/aux channels only.
- `oled_display`, `imu_driver`, `sys_monitor`, `web_control` — rclpy nodes.
- `sim_hardware` — **dev-PC-only**, not built/launched on the board (linux-aarch64). Two
  nodes used only by `bringup.launch.py sim:=true`: `sim_bridge_node` re-publishes
  Gazebo's bridged `/joint_states_sim` + `/imu` + `/scan` as the exact contracts the real
  lidar/IMU/ESP32 publish (`/wheel_ticks`, `/imu/euler`+`/imu/web`, the scan blob), plus
  synthetic ESP32 board telemetry; `map_bridge_node` republishes `slam_nav`'s
  `/dev/shm/nano_map.bin` blob as a real `nav_msgs/OccupancyGrid` on `/map` for RViz
  (useful on the real robot too, not sim-specific).
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
    mirroring how `web_control.cognition.CognitionCore` factored the LLM side. `brain.py` is the
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
    `/cognition/traits` (slam_nav maps `caution`→stop_distance/max_lin, **clamped reflexively
    in slam_nav** so the brain can't push motion unsafe — gated by `trait_motion`). Evolution
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
  **three node hubs matching the three fault domains** — `sensor_hub` (the body),
  `slam_nav` (spatial), `app_hub` (expression/web/brain) — plus the zenoh router.
  app_hub's main also preserves the OLED SIGTERM end-screen (restart/shutdown glyph).

## ESP32 motor/encoder coprocessor (`firmware/nanobot_coprocessor/`)
- **Native zenoh-pico over a direct UART link** (PlatformIO + Arduino) — NO micro-ROS,
  NO Fast-DDS, no agent. It joins the SBC's `rmw_zenoh` graph directly, emitting
  rmw_zenoh's exact wire format + liveliness tokens (see the `src/main.cpp` header).
  Subscribes `/cmd_vel` (geometry_msgs/Twist → diff-drive → H-bridge LEDC PWM), `/led`
  (Bool, onboard-LED pipeline test), `/lds_target_rpm` (Float32 PID setpoint), `/fan_pwm`
  (Float32 0..1 → SBC cooling-fan LEDC PWM; published by `sys_monitor` from the CPU-temp
  curve, web-overridable), `/motor_trim` (Float32 manual straight-line trim set/reset —
  see below). Publishes
  `/wheel_ticks` (Int64MultiArray `[L,R]`) from **single-channel** rising-edge GPIO-
  interrupt counts (**signed by commanded direction** — the encoders have no 2nd channel,
  so the ISR signs each tick by the last `/cmd_vel` wheel direction), `/left_wheel_suspended` +
  `/right_wheel_suspended` (Bool per-wheel off-ground microswitch, **published on change**
  for low latency + a 1 Hz heartbeat republish; the SBC consumers — slam_nav pickup
  freeze, mood_node pickup reflex, web_control snapshot — all honor a **latched
  `/pickup_override` test hook** (Int8: -1 auto, 0 force-grounded, 1 force-lifted; ESP32
  1 Hz heartbeat makes overriding at the source impossible), set from the web
  Coprocessor card, auto-cleared on page reload), `/esp32_temp` (Float32) + `/esp32_hall`
  (Int32) on-die telemetry, and `/esp32_heartbeat` (Int32). Also reads a **spin-lidar**
  (LDS02RR) → `/lds_rpm` (Float32, RPM only — scan data ignored; 0 when stale) + `/lds_hz`
  (valid-frame rate, 0 = not receiving), and closed-loop-controls its spin motor: a PID
  (**tune on hardware**) holds `/lds_target_rpm` by driving the motor PWM, output on
  `/lds_duty`. The LDS path is gated by `LDS_ENABLED` (currently 1; UART1 is drained once
  per PID tick, not every loop, since only the RPM is needed). WiFi/BT kept off.
- **Straight-line trim autocal**: the mismatched gearmotors are rebalanced by a single
  trim factor in `applyMotors` (`l*=(1-t)`, `r*=(1+t)`; positive = robot was pulling
  right). While a straight drive is commanded (equal duties, wheels on the ground, enough
  ticks) the encoder L/R tick-rate imbalance is folded into the trim each 200 ms window —
  **calibration = drive straight ahead for ~5 s**. Persisted to ESP32 NVS (survives
  reboot/reflash; written only while stopped, rate-limited). Manual set/reset via
  `/motor_trim` (Float32, 0 = reset); live value on `/wheel_trim` @1 Hz + the debug
  status line. Tunables `TRIM_*` in `main.cpp`; compiled out if `WHEEL_PID_ENABLED`.
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

## Build / run
- Build: `pixi run build` (colcon, msgs + all python pkgs). There is **no
  `build-lds`/`build-all`** — the Rust node and its toolchain are intentionally gone.
- **`pixi run smoke`** (`scripts/smoke_test.py`) — the end-to-end contract check: boots
  the real router + sys_monitor + app_hub on the dev PC and asserts the /telemetry
  frame keys, the publish/param whitelists, the OLED-face echo, the vitals blob, and
  the SIGTERM shutdown path. **Run it before deploying** — the telemetry frame is a
  typed-nowhere contract between `telemetry.py` and `app.js`, and this is what catches
  a drift.
- Run the stack: **`scripts/stack.sh {up|down|restart|status}`** — now a thin wrapper
  over **systemd**. The stack is five units under **`nano-robot.target`**:
  `nano-router` (zenohd-serial) → `nano-app` (app_hub = web+oled+behavior) /
  `nano-sensors` (sensor_hub = imu+sys+odom+lds) / `nano-nav` (slam_nav) /
  `nano-map` (map_bridge_node, `/dev/shm/nano_map.bin` → `/map` for a remote RViz).
  Ordering (`After=nano-router.service` + the router unit's ExecStartPost probe that
  waits for :7447 to actually accept) encodes the rmw_zenoh island gotcha; **crash
  recovery is `Restart=on-failure`**, and **hang recovery is the systemd watchdog**:
  app/sensors/nav are `Type=notify` and pet `WATCHDOG=1` every 5 s from an *executor
  timer* (`_sd_notify` in each main), so an alive-but-wedged executor (a stuck
  callback) stops petting and gets restarted (`WatchdogSec=90`). Each unit also has a
  `MemoryMax` cap so a leak restarts that hub instead of waking the kernel OOM killer.
  (The old `nano-heal.timer` polling — and its heal-vs-restart duplicate-node race —
  is gone.)
  What each unit execs lives in ONE place: **`scripts/unit_exec.sh`** (pixi env
  activation via `pixi shell-hook`, then `exec` of the installed executable — no
  resident wrapper, no `ros2 run` RAM overhead). Logs: `journalctl -u nano-app` etc.
- On the board the stack **auto-starts on boot** via `nano-robot.target`.
- OS-level setup (overlays, udev, groups, sudoers, systemd units) is scripted in
  **`deploy/sbc-setup.sh`** (idempotent; run once after a reflash + reboot, and
  **re-run once after deploying this systemd migration**). stack.sh's start/stop/
  restart go through scoped NOPASSWD sudoers rules it installs (deploy/sudoers).

## Dev/prod ROS parity + Gazebo sim
There are now **two dev paths**, not one, serving different purposes:
- **`scripts/dev_webui.py` / `dev_run.ps1`** (Windows, no ROS at all) — unchanged, still
  the fastest way to iterate on the LLM/personality/TTS layer (see the LLM/cognition
  section below). Doesn't run `web_control`'s rclpy node, `oled_display`,
  `behavior/mood_node`, `wheel_odometry`, or `slam_nav` — it's a ROS-free stand-in for
  just the AI/Speak/Brain cards.
- **`scripts/sim_run.sh`** (Ubuntu/Linux dev PC, real ROS 2 via the SAME `pixi.toml`
  RoboStack env the board uses — `linux-64` is already one of its `platforms`) — runs
  the **exact same node graph** as the robot: `web_control`, `oled_display`,
  `behavior/mood_node`, `sys_monitor`, `wheel_odometry`, `slam_nav` are all real,
  unmodified rclpy nodes. Only the lowest hardware-transducer layer differs: **Gazebo
  Sim** (`ros_gz_sim`, the modern actively-maintained "Ignition"-lineage simulator —
  `robostack-staging` doesn't cleanly ship classic `gazebo_ros_pkgs` for Humble, but does
  ship `ros-gz-*`) plus `ros_gz_bridge` and the new `sim_hardware` package stand in for
  the LDS02RR/BWT901CL/ESP32. `sim_hardware.sim_bridge_node` converts Gazebo's bridged
  wheel-joint angles into `/wheel_ticks` (so the **real** `wheel_odometry` node still
  does the integration — Gazebo's own diff-drive odometry is deliberately not used) and
  its bridged IMU into `/imu/euler`+`/imu/web` matching `imu_driver`'s exact contract;
  `sim_hardware.map_bridge_node` republishes `slam_nav`'s `/dev/shm/nano_map.bin` blob as
  a real `nav_msgs/OccupancyGrid` on `/map` for RViz (also usable on the real robot).
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
  `robot_state_publisher` + `rviz2` — deliberately NOT `wheel_odometry`/`slam_nav`/
  `sensor_hub`/etc. a second time (the robot is already publishing all of that; a second
  copy on the dev PC would just be a redundant duplicate publisher on the same topics).
  `/scan`, `/odom`, `/imu/euler`, TF, `/map` all stream in over the shared `rmw_zenoh`
  graph.
- **`/dev/shm` is per-machine RAM**, so `sim_hardware.map_bridge_node` (republishing
  `slam_nav`'s map blob as a real `nav_msgs/OccupancyGrid`) has to run **on the board**,
  not the dev PC, for a remote RViz to see `/map` — the `nano-map` unit runs it on the
  board (after `nav`; harmless/cheap, no Gazebo deps).
- **Cross-host zenoh discovery**: `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` already match by
  construction (both machines activate the same `pixi.toml`). Same-LAN zenoh multicast
  scouting usually finds the robot's `zenohd-serial` router with no extra config; if not
  (blocked multicast / different subnet), `rviz_remote.sh --connect <ip>` writes a small
  session config pointing at `tcp/<ip>:7447` and sets `ZENOH_SESSION_CONFIG_URI` (written
  without a way to test cross-host discovery end-to-end from here — if `ros2 topic list`
  on the dev PC doesn't show the robot's topics, check the installed `rmw_zenoh_cpp`
  version's docs for the current session-config env var/schema).

## Conventions / gotchas
- **NEVER add `rust`, `clang`, or `libclang` to `pixi.toml`.** They were build-only
  deps for the abandoned Rust LDS node and pulled a ~1.6 GB toolchain onto the 7 GB
  card. A note in `pixi.toml` guards this.
- **Python packages are installed editable (egg-link → src).** Editing a `.py`
  under `src/<pkg>/<pkg>/` + restarting the node picks it up — **no rebuild needed**.
  A *new* module file still imports fine via the egg-link. `config/robot.yaml` and
  `web/` are symlinked into `install/`, so pushing src updates them too.
- **`rmw_zenoh` ordering matters:** a node started before `rmw_zenohd` runs islanded
  (won't appear in the graph). The systemd units handle this (`After=nano-router.service`
  + the router unit's settle sleep).
- **`web_control` static server**: serves `web/` — `index.html` (markup only, NO
  external scripts — roslib is gone) plus `style.css` and one JS file per former
  inline block (`app.js` main page, `map.js` SLAM panel, `oled.js` OLED mirror,
  `chrome.js` tabs/joystick, `sim.js` in-browser sim, `devtools.js` dev-harness
  sensors; kept as separate `<script>`s on purpose — error isolation + per-file
  strict mode match the old inline blocks).
  `/stream.mjpg` is a zero-dep V4L2 MJPEG passthrough (`mjpeg_camera.py`);
  `/snapshot.jpg` is one still frame (📸 button); `/audio.pcm` is the webcam
  mic as raw PCM via `arecord` (`mic_audio.py`). Both streams are ref-counted (only
  run while a client is connected) and the audio endpoint **must** be HTTP/1.1 chunked
  (browsers don't stream an HTTP/1.0 body to `fetch`). `GET /health/log` serves the
  tail of sys_monitor's durable outage log for the web "Health events" card.
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
- **Browser telemetry+control gateway (`telemetry.py`, replaced rosbridge)**:
  `GET /telemetry` is ONE SSE stream (browser `EventSource`, native auto-reconnect)
  of a compact JSON frame at `telemetry_rate` (5 Hz) with every light readout —
  odom, IMU, `/diagnostics`, ESP32 (hb/ticks/susp/temp/hall), LDS rpm/hz/duty, fan,
  plan (downsampled), latched brain strings (purpose/task/experiments), selftest,
  and the OLED-mirror inputs (face/word/brand/system). The frame is built ONCE per
  tick and fanned out; the underlying subscriptions are **lazy** (created on the
  first client — on the executor thread via the tick timer — dropped `SUB_LINGER`
  after the last), so idle cost is ~zero. Writes: `POST /publish {topic,value}`
  (whitelisted + clamped per topic: goal_pose, lds_target_rpm, pickup_override,
  selftest, go_home/save_map, oled_*) and `POST /param {node,name,value}`
  (whitelisted nodes/params via `/<node>/set_parameters`, fire-and-forget). The
  power buttons only POST `/system/*`; the server itself publishes `/oled_system`.
- **HTTP teleop (`POST /drive`)**: the page POSTs `{v,w}` same-origin; `web_server`
  clamps (`drive_max_lin`/`drive_max_ang`), publishes `/cmd_vel` immediately, and
  re-asserts it at 10 Hz while non-zero with a `drive_timeout` dead-man — so browser
  jank can't outlast the ESP32's 500 ms cmd watchdog and stutter the drive. The dev
  harness accepts it as a no-op.
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
    if unset — a one-line `memory/openrouter_key` file (gitignored; `_load_openrouter_key()`
    in `dev_webui.py`/`dev_tts_test.py`/`personality_creator.py`/`pregenerate_phrases.py`,
    falling back to the old `scripts/.openrouter_key` path for back-compat).
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
- **Skill library** (`skills.py`, ROS-free + unit-tested; `src/web_control/skills/*.md`):
  capabilities as **self-documenting markdown** (an OpenClaw-style "SKILL.md" port). Each
  `.md` = one capability — YAML frontmatter contract (`name`/`description`/`trigger`/`action`)
  + a Markdown body the brain reads as the "how". Drop a new file in (and `POST /skills/reload`)
  to add a capability — no code change. `SkillLibrary` loads + indexes them; `web_server`
  executes. **Two tiers:** *narrative* (`kind: say`/`observe`/`look` — speak a line steered by
  the body, optionally with the sensor snapshot or a `read-lidar`-style `/dev/shm` scan summary
  or a camera frame; routes through the same `_generate`/vision path) and a **gated *action*
  tier** (`kind: topic` — publishes a **whitelisted, clamped** ROS msg: `/led`, `/fan_pwm`,
  `/lds_target_rpm`, `/cmd_vel`). An action runs only when the skill sets `enabled: true` **AND**
  `skills_allow_actions` (web_control param, **off by default**); motion stays clamped reflexively
  by slam_nav, so a skill can never make the robot unsafe. **Two entry points:** autonomous (the
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
    skills still require the LLM (no cached-line fallback for named skills yet — unlike the
    generic idle "musing" beat, which already tries the phrase bank first via `bank_say`).
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
- **Heavy topics stay OFF the telemetry frame:** the two biggest messages are served
  same-origin from `/dev/shm` and polled by the page: `/map` (occupancy grid, written
  by `slam_nav`) and `/scan.bin` (compact lidar blob = JSON header + raw float32
  ranges, written by `lds_driver_py`) — the page controls the poll rate per view.
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
  `sys_monitor.fan_temp_min` (the "Fan starts at" slider on the web Cooling fan card) is
  whitelisted the same way — the °C below which the auto curve idles the fan at 0% duty
  before ramping up to `fan_temp_max`=70°C.

## Deploying to the live board (from a dev host)
- One-shot deploy: **`scripts/deploy.sh [pkgs…]`** — copies `src/`+`scripts/`,
  colcon-builds (optionally `--packages-select`), then `stack.sh restart`. Creds via
  env (`NANO_PW`, `NANO_HOST`, `NANO_HOSTKEY`) — **never commit secrets**.
  It also pushes the dev-made soul/bank (`memory/personality.json` + `phrases.json`, plus
  hand-edited `presence_chart.yaml`/`beats.json` if present)
  into the board's `~/.local/state/nanobot/` — **ON by default for now**
  (`DEPLOY_SOUL=1`), which **overwrites** the robot's own persisted personality (discarding
  any evolved trait drift). Set **`DEPLOY_SOUL=0`** to skip and keep the robot's evolved soul.
- The dev host is native Ubuntu with a passwordless `ssh nano`/`scp nano:` alias
  (creds git-ignored in `.nano-deploy.env`). (Historical, if ever deploying from
  Windows again: **`plink -m <localfile>` sends the file's text as the remote shell's
  argv**, so any `pkill -f`/`pgrep -f` pattern appearing in the script kills the
  controlling shell — `pscp` the script and run it by path instead.)
- `stack.sh restart` is now `systemctl restart nano-robot.target` — systemd owns
  stop/kill/verify, so the old "stale process serving old code" failure mode (and the
  heal-timer duplicate-node race) is gone by construction. If a change "doesn't take",
  check `journalctl -u nano-app` (etc.) and `systemctl status nano-robot.target`.
- The board has only ~1 GB RAM and a 7 GB rootfs — watch memory and disk. Don't run
  heavy compiles on it.
