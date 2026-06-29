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
  - **`mood_node` is thin ROS glue; ALL the ROS-free thinking is in `brain.py`** —
    mirroring how `web_control.cognition.CognitionCore` factored the LLM side. `brain.py` is the
    single behaviour-layer "brain" module: the **Purpose Engine** (objective + intrinsic-reward
    weights, deterministic reflection — `default/merge/reflect_purpose`), the **Horizon Planner**
    + A/B **bandit** (`decompose`/`verify`/`Planner`/`Bandit`), and the orchestration —
    `PurposeBrain` (beat-upgrade decisions, reflect, reward, reflection mode, persist) +
    `Personality` (chart-context traits/evolution: seed/evolve/heartbeat/persist). (The Purpose
    Engine + Planner used to be separate `purpose.py`/`planner.py`; folded into `brain.py` to
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
    row** (face/camera/audio/prompt + priority/needs/trait); no chart surgery.
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
    the **real `web/index.html`** on a dev PC (ROS-free stand-in for `web_server`) and runs
    the **same `CognitionCore`** (so there's one base, not two — see below), wiring `/llm/*`,
    `/skills/*`, `/tts*` + the brain card, so the AI/Skills/Brain cards + Speak box can be
    tested in a browser locally (telemetry/joystick/map show offline — no rosbridge). Reads
    the persona/model from robot.yaml (PyYAML) and the key from `$OPENROUTER_API_KEY`.
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
  `deploy.sh` pushes `devstate/skills/*.md` + `workshop.json` with the soul. Config: `workshop_*`
  in robot.yaml. Runs identically on the dev harness (mints into `devstate/skills/`).
  **The workshop is also an on-demand skill** — `skills/forge-skill.md` (`action.kind: workshop`,
  a "meta" kind in `skills.py` that runs an internal routine, never a topic/narrative): invoke it
  any time (web "🛠 Skills" / `POST /skills/invoke {name:"forge-skill"}`) to forge a skill outside
  reflection mode. Meta skills are **excluded from autonomous skill-beat selection** (`offered()`),
  so they only run when deliberately invoked. See [[meditation-skill-workshop]].
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
  It also pushes the dev-made soul/bank (`devstate/personality.json` + `phrases.json`)
  into the board's `~/.local/state/nanobot/` — **ON by default for now**
  (`DEPLOY_SOUL=1`), which **overwrites** the robot's own persisted personality (discarding
  any evolved trait drift). Set **`DEPLOY_SOUL=0`** to skip and keep the robot's evolved soul.
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
