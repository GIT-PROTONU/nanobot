# CLAUDE.md

Guidance for working in this repo. See `README.md` for the human-facing setup.

## What this is
**Nano** — a mobile robot on a **NanoPi NEO Plus2 (Allwinner H5, aarch64, 1 GB RAM)**
running **Armbian**, with **ROS 2 Humble** installed as conda packages via
**pixi + RoboStack** (channel `robostack-staging`). Middleware is **`rmw_zenoh`**
(chosen for low RAM; needs `rmw_zenohd` running). The web UI is **rosbridge + a
static HTML page** (`web_control`), not Foxglove.

> **zenoh vs rosbridge are different layers, not competitors.** `rmw_zenoh` is the
> node-to-node RMW (incl. the ESP32 via zenoh-pico through `zenohd-serial`). A browser
> can't speak zenoh/DDS, so `rosbridge_websocket` (a Python rclpy node *on* the zenoh
> graph) bridges ROS↔browser over a WebSocket on `:9090` for roslib. Heavy topics
> (`/scan`, `/map`) bypass rosbridge entirely via `/dev/shm`+HTTP, so what's left on it
> is light. Going zenoh-all-the-way to the browser is possible (zenoh-ts + a router
> plugin) but not worth it — you'd lose ROS typing and hand-decode CDR in JS. See the
> rosbridge-vs-zenoh-transport memory.

Hardware: Roborock **LDS02RR** lidar (scan on **UART2 `/dev/ttyS2`**; RPM also read by
the ESP32), single-channel **wheel
encoders** + **motors** (now via an **ESP32-WROOM coprocessor**, see below),
**PCA9685** PWM (I2C, now unused by the stack), **SSD1306** OLED (I2C), **BWT901CL**
IMU (WitMotion, USB-serial/CH340), **Logitech C270** webcam + mic (USB).

## Layout (`src/`)
- `robot_msgs` — custom interfaces (ament_cmake).
- `robot_bringup` — launch files + **the single config `config/robot.yaml`** (all
  ports/pins/rates live here).
- `lds_driver_py` — **the LDS driver in use** (rclpy, publishes `/scan`; also writes a
  compact scan blob to `/dev/shm/nano_scan.bin` for the web UI — see `web_control` below).
- `lds_driver` — **abandoned** Rust/r2r LDS node; does NOT build against this
  RoboStack. Kept for reference only. **Do not try to build it** (see below).
- `wheel_odometry` — integrates `/wheel_ticks` (from the ESP32) into `/odom`+TF;
  no longer reads GPIO.
- `motor_control` — **retired** (PCA9685 path). The ESP32 owns `/cmd_vel`→motors;
  not launched by `stack.sh`/`robot.launch.py`. Kept for the optional PCA9685
  LDS-spin/aux channels only.
- `oled_display`, `imu_driver`, `sys_monitor`, `web_control` — rclpy nodes.
- `behavior` — **behaviour layer (Sismic statechart)**. *Human-readable overview of the
  whole brain (statechart + LLM + traits/evolution + model caps + decision log):
  [`docs/brain.md`](docs/brain.md); the bullets below are the terse engineering summary.*
  `mood_node`: an idle "feel alive"
  presence supervisor that drives the OLED face (`/oled_face`) during true idle and stands
  down when another owner uses the panel (motion/goal, TTS, manual web mood, pick-up).
  **Expression-only — never publishes `/cmd_vel`.** The chart lives in `presence.py`
  (ROS-free, unit-tested offline: `pixi run python -m pytest src/behavior/test`); the node
  maps topics→events. No-op if sismic is missing or `behavior.enable:=false`.
  - **The chart is also the single brain for autonomous LLM expression.** Its idle beats
    are *predefined states with offline default faces*: `musing` (sensors) every idle
    cycle, `looking` (camera) every `look_every`-th cycle (gated by `camera_beats`). On a
    beat the node shows the default face immediately AND (if `enrich_enable`) fires a
    **fire-and-forget** `/cognition/request` (JSON `{beat,state,prompt,camera}`) that
    `web_control` executes asynchronously (LLM line + optional camera + mood). A
    slow/absent LLM = a silent face-beat; the chart never waits. The per-beat convention
    (default face + prompt + camera flag) is the `BEATS` table in `presence.py`; add a beat
    = add a state calling `do_beat('name')` + a BEATS entry.
  - **Parametric personality + evolution.** `traits` (curiosity/extraversion/caution/
    playfulness, 0..1) + a `registry` (musing/looking priority/enable/gates) live as mutable
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
    the **seeded baseline** (not generic defaults). Reflexes (`greeting`/`resting`/`dormant`/
    pickup) are NOT in the registry, so the brain can never disable them. See the
    llm-openrouter-personality memory.
- `sensor_hub` — **runs `imu_driver` + `sys_monitor` + `wheel_odometry` + `lds_driver_py`
  in ONE process** (one executor) to save ~100+ MB RAM on the 1 GB board. Same node
  names/topics/params/services — purely an packaging change. `stack.sh` launches this
  instead of those four separately. Trade-off: they no longer crash/restart independently.

## ESP32 motor/encoder coprocessor (`firmware/nanobot_coprocessor/`)
- **Native zenoh-pico over a direct UART link** (PlatformIO + Arduino) — NO micro-ROS,
  NO Fast-DDS, no agent. It joins the SBC's `rmw_zenoh` graph directly, emitting
  rmw_zenoh's exact wire format + liveliness tokens (see the `src/main.cpp` header).
  Subscribes `/cmd_vel` (geometry_msgs/Twist → diff-drive → H-bridge LEDC PWM), `/led`
  (Bool, onboard-LED pipeline test), `/lds_target_rpm` (Float32 PID setpoint), `/fan_pwm`
  (Float32 0..1 → SBC cooling-fan LEDC PWM; published by `sys_monitor` from the CPU-temp
  curve, web-overridable). Publishes
  `/wheel_ticks` (Int64MultiArray `[L,R]`) from **single-channel** rising-edge GPIO-
  interrupt counts (**signed by commanded direction** — the encoders have no 2nd channel,
  so the ISR signs each tick by the last `/cmd_vel` wheel direction), `/left_wheel_suspended` +
  `/right_wheel_suspended` (Bool per-wheel off-ground microswitch, **published on change**
  for low latency + a 1 Hz heartbeat republish), `/esp32_temp` (Float32) + `/esp32_hall`
  (Int32) on-die telemetry, and `/esp32_heartbeat` (Int32). Also reads a **spin-lidar**
  (LDS02RR) → `/lds_rpm` (Float32, RPM only — scan data ignored; 0 when stale) + `/lds_hz`
  (valid-frame rate, 0 = not receiving), and closed-loop-controls its spin motor: a PID
  (**tune on hardware**) holds `/lds_target_rpm` by driving the motor PWM, output on
  `/lds_duty`. The LDS path is gated by `LDS_ENABLED` (currently 1; UART1 is drained once
  per PID tick, not every loop, since only the RPM is needed). WiFi/BT kept off.
- **Tunables are `#define`s inline at the top of `src/main.cpp`** (there is no
  `include/config.h`). `include/zenoh_generic_config.h` only holds zenoh-pico feature
  flags (enables `Z_FEATURE_LINK_SERIAL`). Pins (ESP32 GPIO): encoders L=19 R=26,
  off-ground switches L=18 R=27, motor STBY=23, H-bridge IN L=25/4 R=32/33 (fwd/rev),
  onboard LED=2, **UART2 = zenoh link (TX=17, RX=16) → SBC `/dev/ttyS1`**, **LDS data on
  UART1 RX=GPIO14 (TX=GPIO13 unused)**, LDS spin-motor PWM=21, cooling-fan PWM=22 (via a
  logic-level MOSFET — the ESP can't source fan current). (SBC side: ESP32 link on
  `/dev/ttyS1`/UART1-PG6/PG7, LDS scan on `/dev/ttyS2`/UART2-PA0/PA1, OLED on
  `/dev/i2c-0`/PA11-PA12 @400kHz.) Keep diff-drive limits synced to `robot.yaml`.
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
  → oled → **sensors** (one `sensor_hub` process = imu+sys+odom+lds) → nav in order,
  idempotent (pgrep-guarded), logs to `.run/*.log`. `down`/`restart` SIGTERM→wait→SIGKILL
  and verify (also sweeps pre-merge per-node stragglers). (The agent is skipped if
  `$AGENT_DEV` isn't plugged in.)
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
- **Text-to-speech** (`tts.py`): `POST /tts {text,voice?}` synthesises with
  `pico2wave` (SVOX Pico, **English + German only** — install via
  `deploy/install-picotts.sh`) to a `/dev/shm` WAV, plays it with `aplay`, and
  publishes the words one at a time on **`/oled_word`** timed to the clip duration
  (Pico emits no word marks, so timing is length-weighted). `oled_display` shows
  each word big+centred as it's spoken ("karaoke"); `""` returns to the dashboard.
  Both binaries run **only while speaking** (zero idle cost). The web "Speak" box
  reuses the old OLED-text field; it no longer publishes `/oled_text` (that brand
  override still works if published manually). Off rosbridge on purpose (HTTP POST).
  - **Voice/volume/speed/pitch** are tuned in the UI and applied as Pico's inline
    `<volume>/<speed>/<pitch>` level markup (no ALSA-mixer dep). They + the stats
    announcer are **persisted** to `~/.local/state/nanobot/tts.json` (override with
    the `tts_settings_path` param) and reloaded on node start, so they survive a
    reboot. `GET/POST /tts/config` read/update them; the page restores its controls
    from `GET /tts/config` on load.
  - **Spoken system stats**: a server-side 1 Hz tick (`_announce_tick`) speaks
    CPU%/RAM%/CPU-temp every `announce_interval` s when `announce` is on — it lives
    in the node, so it **keeps running after every browser closes** and resumes after
    a reboot. `POST /tts/announce` says it once now. CPU/RAM/temp come from the same
    cheap `/proc` + thermal reads the OLED uses; phrasing follows the selected voice.
  - **Cross-platform TTS for dev testing**: `tts.py` is ROS-free and auto-selects a
    backend — the robot's `pico2wave`+`aplay` if present, else **Windows SAPI** (via
    PowerShell `System.Speech`) or macOS `say`. So `scripts/dev_tts_test.py` (no ROS)
    speaks a line on a dev PC: `python scripts/dev_tts_test.py "hi"`, or
    `--llm "prompt"` to run the full OpenRouter→speech pipeline (needs
    `OPENROUTER_API_KEY`). Pico level markup + German lingware are pico-only; the
    fallbacks speak plain text with native rate/volume. `scripts/dev_webui.py` serves
    the **real `web/index.html`** on a dev PC (ROS-free stand-in for `web_server`) wiring
    only `/llm/*`+`/tts*`, so the "AI · OpenRouter" card + Speak box can be tested in a
    browser locally (telemetry/joystick/map show offline — no rosbridge). Reads the
    persona/model from robot.yaml (PyYAML) and the key from `$OPENROUTER_API_KEY`.
- **LLM personality (OpenRouter)** lives in `web_control` (`llm.py`, ROS-free) — it
  offloads "say something" / chat lines **plus the matching OLED expression** to a model
  on OpenRouter. `LlmClient.generate()` is a blocking stdlib-`urllib` POST (no SDK) that
  returns `{"say","mood"}`; the **mood is constrained to the OLED's four faces** +
  `neutral` (coerced if the model strays). **Two text tiers:** `llm_model`
  (`deepseek/deepseek-v4-flash`, cheap) for everything; `llm_smart_model`
  (`deepseek/deepseek-v4-pro`, reasoning) only for chat (`generate(smart=True)`).
  `LlmClient.model_for(smart,image)` resolves the slug; pro is a reasoning model so
  `llm_max_tokens` is 1024 (too low → empty JSON = no-reply). `LlmClient.complete(system,
  user,smart=,json_object=)` is a general (non-`{say,mood}`) call used by tools.
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
  `GET /llm/log`. The web "AI" card (Speak tab) drives the on-demand ones. **Autonomous
  chatter is NOT here** — it's driven by the `behavior` statechart's beats via
  `/cognition/request`, which `web_control` executes (`_on_cog`→`_run_beat`: capture frame
  if asked, append the sensor snapshot, `_generate`). The old standalone idle-chatter timer
  was retired (one brain). Best-effort: **no key / no network = silent no-op**, never on
  the critical path. All config is in `robot.yaml` (`llm_*`, and the `behavior:` beat
  knobs); the **key is read from `llm_api_key` or, when blank, `$OPENROUTER_API_KEY`** —
  keep real keys out of the committed yaml. UI toggles (enable/model/persona) persist to
  `~/.local/state/nanobot/llm.json` (never the key). Calls run off the ROS executor thread
  and are one-at-a-time guarded.
  - **Decision log** (`GET /llm/log`, web "🧠 Decision log" panel): every generation path
    (`say`/`chat`/`observe`/`look`/`beat:*`) records a `CognitionLog` entry (trigger,
    state, camera, model, status, say/mood, latency) — incl. skip reasons
    (`skipped-busy`/`llm-unavailable`/`no-frame`). Appended as JSON lines to
    `cognition_log_path` (default `~/.local/state/nanobot/cognition.log`) and seeded back
    into the ring buffer on start, so it survives reboots. **Both `web_server` (robot) and
    `scripts/dev_webui.py` (dev) write the same file/format**, so history is shared. See the
    llm-openrouter-personality memory.
  - **Phrase bank** (`phrasebank.py`): the most frequent lines — the body-reaction beats
    (`musing`/`observe`) — are **pre-generated** instead of hitting the LLM every idle cycle.
    A batch of in-character lines per *situation* (picked_up/hot/busy/idle/… classified from
    the sensors), each with **placeholders** (`{name}{cpu}{mem}{temp}{tilt}`) filled with
    live values at speak time → instant, free, offline, still varied. Logged
    `status="bank"`. `pick()` prefers lines whose placeholders are all fillable. The bank
    (`~/.local/state/nanobot/phrases.json`) stores the persona+traits **signature** it was
    made with and **auto-regenerates in the background** when the soul drifts too far
    (`phrasebank_drift`) or the persona changes; `phrasebank_live_ratio` still sends a few
    beats live for freshness. Force/inspect: `scripts/pregenerate_phrases.py [--show]`,
    `GET /llm/phrases`, `POST /llm/phrases/regenerate`. Config: `phrasebank_*` in robot.yaml.
- **Heavy topics go over HTTP, not rosbridge:** rosbridge's cost is rclpy building a
  Python msg per *incoming* sample (throttle_rate doesn't help — see [[sbc-cpu-profile]]),
  so the two biggest messages are served same-origin from `/dev/shm` and polled: `/map`
  (occupancy grid, written by `slam_nav`) and `/scan.bin` (compact lidar blob = JSON
  header + raw float32 ranges, written by `lds_driver_py`). The page polls these like
  files; `/scan` is **not** bridged. Also publishes `/esp32_ping` @1 Hz (ESP liveness).
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
