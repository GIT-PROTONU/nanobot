# AGENTS.md — Nano robot

## Build & run

- Build: `pixi run build` (runs `scripts/build.sh` — colcon + explicit CMake Python hints for RoboStack). Python pkgs are `--symlink-install` (edit + restart, no rebuild).
- **Do NOT add `rust`/`clang`/`libclang` to `pixi.toml`.** The Rust `lds_driver` is abandoned — `lds_driver_py` is the active driver. `src/lds_driver/` is reference only.
- Runtime on the board: `scripts/stack.sh {up|down|restart|heal|status}`. Nodes launched by direct executable path (not `ros2 run`) to save RAM. `rmw_zenoh` router must start first.
- Zenoh needs a serial-capable `zenohd` binary (conda builds lack `transport_serial`). Build with `firmware/nanobot_coprocessor/tools/build_zenohd_serial.sh {x86_64|aarch64}`.
- `stack.sh restart` can leave stale processes. Prefer `down` → verify — `up`.
- Auto-starts via systemd `nano-stack.service`.
- Dev PC offline testing: `scripts/dev_webui.py` serves the real web UI + cognition (no ROS).

## Tests

- Brain tests live in the **nanobot-brain** repo at `/home/ib/Desktop/nanobot-brain`:
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

## Architecture

| Package | Role |
|---|---|
| `robot_msgs` | Custom ROS interfaces (ament_cmake) |
| `robot_bringup` | Launch files + single config `config/robot.yaml` |
| `lds_driver_py` | Active LDS driver (rclpy → `/scan` + `/dev/shm/nano_scan.bin`) |
| `sensor_hub` | **One process** for imu_driver + sys_monitor + wheel_odometry + lds_driver_py |
| `slam_nav` | SLAM/mapping (writes `/dev/shm/nano_map.bin`) |
| `web_control` | ROS glue layer: rosbridge + static web page + TTS + delegates to `nanobot_brain.cognition` |
| `behavior` | ROS glue layer: Sismic chart lifecycle, topic wiring — delegates to `nanobot_brain.behavior` |
| `motor_control` | **Retired** (ESP32 owns motor path) |
| `oled_display` | I2C SSD1306 dashboard |
| `wheel_odometry` | `/wheel_ticks` → `/odom` + TF (from ESP32, not GPIO) |
| `imu_driver` | BWT901CL over USB-serial |
| `sys_monitor` | CPU/RAM/temp → `/diagnostics` |

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
- Skills live in the `nanobot-brain` repo under `skills/*.md` — YAML frontmatter + markdown body. Drop a file, `POST /skills/reload`. No code change.
- Two tiers: narrative (`say`/`observe`/`look`) and gated action (`topic` — whitelisted, off by default).
- Workshop (reflection mode) synthesizes new skills via LLM → `workshop_dir` (default `~/.local/state/nanobot/skills`).

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

**Web UI:** Sensors panel > "Brain health" card shows behavior, cognition, LLM, purpose, chart status — green/alive or red/lost. Polled every 2s from `/brain/health`. If the endpoint itself fails, all indicators show amber `err`.

### Localization troubleshooting in the web UI
Sensor chain: ESP32 (`/wheel_ticks`, signed by commanded wheel direction in firmware) → `wheel_odometry/encoder_node.py` → `/odom` → robot_localization **EKF** (`src/robot_bringup/config/ekf.yaml`, fuses `/odom` + `/imu/data` → `/odometry/filtered` @15 Hz) → `slam_nav` (`odom_topic = odometry/filtered`, `imu_yaw_sign: -1` in `robot.yaml`).

IMU heading wiring (fixed 2026-08-10): `imu_driver` publishes `/imu/data` with **`frame_id: base_link`** (the driver pre-rotates via the mount matrix + lever-arm-corrects into the chassis frame, so `imu_link` gets dropped by the EKF — it had no TF and robot_localization silently ignored the absolute orientation). `robot.yaml imu_driver.yaw_sign: -1` aligns the BWT901CL heading with the wheels. `imu_driver/_configure_device()` forces the sensor's **range registers on every connect** (`0x29=0x03` → accel ±16 g, `0x2B=0x03` → gyro ±2000°/s — WitMotion codes are **inverted**: 0x00 = narrowest). If this unit boots at ±250°/s while the driver decodes ±2000, **every gyro axis and the device's fused heading come out 8× too big**, silently poisoning the EKF/SLAM heading (symptom: `/odom` yaw looks ~10× smaller than `/imu/euler` during a spin, on top of real tire slip).

What the page already surfaces for each stage:
- **Wheel ticks** — Coprocessor/ESP32 card: `/wheel_ticks` L/R counts, tick Hz, heartbeat, stray-tick diagnostic + reset.
- **Odom** — Odometry card: `/odom` x/y/θ + publish-rate slider (`/wheel_odometry/set_parameters`).
- **IMU** — IMU card: `/imu/web` rate ("lost" beacon when stale), `/imu/euler` display, 3D mount indicator, spin check, drift tool, interference self-test, mag-cal scatter, 6-axis toggle.
- **SLAM/feeds** — map panel + mapStats line: mode/explored/match score/loc + `⚠ feeds: odom · imu · lds` staleness, read from the `/map` JSON header written by `slam_nav/nav_node.py` (`meta["feeds"]`; `-1` = never received; re-aged at read time in `index.html:1948`). The header also carries `mcmd` = last published `/cmd_vel` `(v, w)` (`slam_nav`), so a stalled feed vs. a commanded one is distinguishable at a glance.
- **EKF** — NOT shown standalone: `/odometry/filtered` only enters the UI indirectly via the SLAM feed-staleness. Any new EKF view must compare raw `/odom` vs `/odometry/filtered` vs SLAM map pose / IMU yaw.

**Empty-map relocalize guard (`recover_min_seen`, added 2026-08-11):** when localization is lost on a near-empty grid there is nothing for the scan matcher to lock onto, so a persistent low score is *expected* (fresh map / just cleared) rather than evidence of drift — and the relocalize in-place spin (`recover_spin`) only smears the grid + drains the battery. `slam_nav` now suppresses the spin whenever map coverage (`grid.coverage()`) is below `recover_min_seen` (default **0.25**): it holds pose, logs `map too empty (seen X%) to relocalize — holding pose, not spinning` (throttled 5 s), and keeps running the recovery matching in `_on_scan` so the spin resumes automatically once the map fills past the threshold. Live-tunable via `/param` (`recover_min_seen`), whitelisted in `telemetry.py`. Verified 2026-08-11: pre-fix a goal-click on a just-cleared map stormed 155 `localization lost` cycles in ~100 s spinning at 0.6 rad/s; post-fix the same scenario logs the guard line and publishes **zero** cmd_vel. `0.10` was too low (a 0.106-covered smeared map still stormed) — hence `0.25`. Threshold choice is a map-density judgment; raise it for sparse rooms, lower for dense ones.

**"Who told the robot to move/spin?" diagnosability logging (added 2026-08-11):** every driving decision is now in the logs so a mysterious spin can be traced without live-tracing:
- `slam_nav._send` throttle-logs the first non-zero `/cmd_vel` after an idle stretch with its mode: `cmd_vel -> v X w Y (mode recover|goal|other)` (2 s throttle); the last published command (incl. stops) is always in the map header as `mcmd`.
- `slam_nav._on_params` logs `enable_motion -> True/False (was …)` transitions (`cmd_vel now live/blocked`).
- `web_server` logs `POST /drive v X w Y (web teleop)` (2 s throttle) and skill actions (`skill action /cmd_vel lin=… ang=… for …s`, `goal_pose -> '<name>' (x, y) (skill)`).
- `telemetry.publish_json` logs every discrete map-button/goal publish (`POST /publish /goal_pose …`, `/slam_nav/go_home|save_map|clear_map`, `/selftest`, `/reset_ticks`); `set_param_json` logs `POST /param node/name = value`.

**Served-page state (RESOLVED 2026-08-10):** `src/web_control/web/index.html` is a **self-contained, pure-SSE** page — no rosbridge/roslib. The main inline script (`app.js`-derived) opens an `EventSource("/telemetry")` and drives everything off the SSE frame + `POST /publish|/param|/drive`; `oled.js` is inlined for the OLED mirror. Brain readouts (`purpose`/`task`/`experiments`) come from the SSE `f.*` fields, NOT `GET /purpose` etc. (those endpoints don't exist — an initial `404` from `pollBrainHttp` before the link is up is harmless). The old stale ROSLIB block and the SSE split files (`app.js`, `map.js`, `oled.js`, `chrome.js`, `sim.js`, …) were superseded — **do not** try to `script src` them or wire a websocket. When editing the page, keep it self-contained SSE and follow the pattern of the existing blocks (map/chrome/sim/motion-chain/EKF/slam-tuning).

**Vermap map buttons wired through the SSE gateway (fixed 2026-08-11):** the Map card's Clear/Home/Save/Test/Stop buttons use `pub()`/`sendDrive()` (NOT ROSLIB): `mapClear` → confirm → `pub("/slam_nav/clear_map", true)` + the page resets `mapGoal`/`mapPlan` locally (telemetry only emits `f.plan` while non-empty); `mapHome` → `pub("/slam_nav/go_home", true)`; `mapSave` → `pub("/slam_nav/save_map", true)`; `mapTest` → `pub("/selftest", true)`; `mapStop` → `sendDrive(0,0)`. All four ROS topics are **bare** names in `telemetry.py` (`go_home`/`save_map`/`clear_map` — see the web-publish-topic-namespace gotcha), whitelisted as `/slam_nav/…` keys for the browser POST. `_on_clear_map` (nav_node.py ~1309) sets `_nogo_dirty=True` so `/dev/shm/nano_nogo.bin` is rewritten to a `count:0` mask, drops `_goal`/`_goal_is_frontier`/`_path`, and publishes an empty plan — else the robot keeps driving toward a stale goal on the fresh map. The blob is 230472 B = JSON header + 480×480 bool mask; `/map/nogo` is served by `web_server` via `_serve_shm`.

**Web-serving chain:** `web_server.py` serves the package's installed `web/` dir, symlinked `install/web_control/share/web_control/web/ → build/web_control/web/ → src/web_control/web/`. Edit `src/web_control/web/index.html`, restart the app — picked up live.

## Gotchas

- **`stack.sh restart` is unreliable** — can leave stale processes holding ports. Clean `down` → verify via `/proc` → `up`.
- **A missing `/dev/shm/nano_nogo.bin` right around a restart is teardown, not a bug** — during a `deploy.sh`/`stack.sh restart` window the old processes + the `user@1000` session teardown transiently remove blobs (even a hand-created dummy `/dev/shm/nano_nogo.bin` vanished within that window). Nothing in code deletes it (no `os.remove`/`unlink`/`os.replace` to that path anywhere; tmpfiles.d/crons/timers are clean). Re-check once the stack is quiet before hunting a "deleter" — the blob persists indefinitely after a clear once settled. See `web-map-clear-buttons-nogo` memory.
- **`brain_timeout` must stay well above `reflect_period`** (invariant: timeouts shorter than the reflection gap cause the chart to revert accumulated drift).
- **Heavy topics bypass rosbridge:** `/map` and `/scan.bin` are served from `/dev/shm` via HTTP, not bridged.
- **`rmw_zenoh` ordering:** a node started before `rmw_zenohd` runs islanded (won't appear in the graph).
- **Python edits are live:** `--symlink-install` means edit `src/<pkg>/<pkg>/foo.py`, restart node = picked up. New modules import fine via egg-link.
- **nanobot-brain is pip-installed**: edit `src/nanobot_brain/` in the nanobot-brain repo, restart node = picked up (editable install).
- **`deploy.sh` does NOT push `nanobot-brain`** — the brain repo is a separate git checkout copied to the board at `/home/ibster/Nano/brain/src` (a `brain/src` PYTHONPATH entry, not pip-installed there). If you change the brain on the dev PC you MUST sync it to the board yourself: `rsync -az --exclude __pycache__ src/ nano:/home/ibster/Nano/brain/src/`. A stale brain silently breaks nodes at runtime with `TypeError: __init__() got an unexpected keyword argument ...` (e.g. `vision_diary_enable`, `nudge_looming_caution`, `chart_path`) — the glue (`mood_node`/`web_server`/`dev_webui`) and the brain must stay in lockstep.
- **`telemetry.py` `DiagnosticStatus.level` is `bytes` under rmw_zenoh** — `pipe.level` arrives as `b'\x00'|b'\x01'|b'\x02'`, which is NOT JSON-serializable and kills the entire app_hub (telemetry `_tick` runs on the executor, so an unhandled `TypeError` crashes the process → systemd respawn loop). Normalize to `int` at ingest (`_on_diag`). Any new raw ROS field put into the telemetry frame must be a JSON-safe type after passing through rmw_zenoh.
- **Callback exceptions on the executor = respawn loop** — any unhandled error inside a subscription callback (not just `DiagnosticStatus.level`) kills the whole hub process, so systemd `Restart=on-failure` looks like a boot loop. Real case (fixed 2026-08-10): `telemetry.py:_on_slam_pose` copied the Odometry layout (`msg.pose.pose`) onto the actually-`PoseStamped` `/slam_pose` → `AttributeError` every tick. Match the real message type (`/slam_pose` is `PoseStamped` = `msg.pose`). Same class: `nav_node.py:_on_scan` used a local `angles` bound only inside the lidar-geometry memoization branch → `UnboundLocalError` on the 2nd scan; the memoized alias is `self._ang_cache` — always use that.
- **Vision readouts are one atomic snapshot** — `gpu_vision` scalars are read via `snapshot()` (one `_lock` acquisition for all fields, added 2026-08-10). Don't revert to per-property getters in the 5 Hz telemetry build or the 10 Hz `_vision_state_tick`: that was ~20 lock round-trips/tick + cross-field reading skew.
- **BWT901CL gyro/accel range registers are inverted and NOT persisted** — WitMotion maps `0x00 = narrowest`, `0x03 = widest`: `0x29` accel (0=±2 g … 3=±16 g), `0x2B` gyro (0=±250 … 3=±2000 °/s). The device defaults to ±250°/s at power-on, but the driver decodes raw as ±2000 → **8× too big on every axis + the device's internal fused heading** (root cause of the 2026-08-10 SLAM lost-storms). `imu_driver/_configure_device()` re-writes `0x29=0x03, 0x2B=0x03` on every (re)connect exactly like RRATE — do NOT "simplify" them away, or the next power-cycle silently reintroduces a scaled heading.
- **A differential robot's body yaw rate is bounded by its wheel command** — if `/imu/euler` Δyaw vastly exceeds what the wheels could have rolled (`(ΔL+ΔR)/2·m_per_tick`, ARC of both wheels), suspect an IMU scale/sign error, not an encoder undercount. Wheel-odom translation is correctly scaled; expect only small tire-slip gaps on spins.
- **Low-duty turn commands can stall the wheels** — in-place turns map to tiny per-wheel speeds (±0.04 m/s at 0.5 rad/s) whose duty can stop the motors ~0.5-1 s in (wheels AND body both freeze mid-command; encoder counts + IMU plateau together). This is a drive-power issue, separate from sensing.
- **`config/robot.yaml` is the single config source** — all ports, pins, rates, LLM params live there. Its `slam_nav:` block uses the ROS param layout (`slam_nav.ros__parameters.<name>`). Indentation must match sibling keys exactly: a block one space off parses as a *nested map* and the whole `slam_nav` section silently returns `None` to nav_node (only the in-code default saves you). Always sanity-check with `python3 -c "import yaml,sys; print(yaml.safe_load(open('src/robot_bringup/config/robot.yaml'))['slam_nav']['ros__parameters']['recover_min_seen'])"` after editing, and remember the running stack reads it via the `build/ → src/` symlink, not a copied install.
- **ESP32 link can wedge after a stack restart and needs a PHYSICAL power cycle** — after `stack.sh down/up` the coprocessor may never re-attach to the router's serial link (`/dev/ttyS1`): `esp32 DOWN: no heartbeat ever received`, `/wheel_ticks` silent, LDS motor dead (ESP32 drives its PID), scans stop. Service restarts, full `nano-robot.target` restarts, even a board `sudo systemctl reboot` do NOT reliably recover it — the firmware's auto-recovery watchdogs (`LINK_CONNECT_DEADLINE_MS`, `LINK_RX_TIMEOUT_MS` in `firmware/nanobot_coprocessor/src/main.cpp`) apparently can't re-sync a wedged UART. Symptom chain when it happens: `esp32 DOWN` → `lds DOWN: lidar not spinning` → `wheel_ticks SILENT` → map `feeds.scan: -1`. Diagnosis: `journalctl -u nano-sensors.service | grep -i esp32`, and confirm the router holds the fd (`ls -l /proc/$(pgrep -f zenohd-serial)/fd | grep ttyS1`). Fix = unplug/replug the ESP32's power. After a successful power cycle it comes back on its own (`esp32 UP after …`, `/wheel_ticks resumed`, `lds UP`), and `lds_idle_enable=false` + `lds_active_rpm=300` via `/param` wakes the lidar if it's parked.
- **`plink -m` on Windows:** the script text becomes the shell's argv. `pkill -f` patterns can kill the controlling shell. Fix: `pscp` script, run by path.
- **ESP32 firmware:** PlatformIO from dev PC (`pio run -t upload`). Don't build on the board. Tunables are `#define`s at top of `src/main.cpp`.
- **Deploy soul overwrite:** `DEPLOY_SOUL=1` (default) pushes `devstate/` personality to board, discarding evolved drift. Set `DEPLOY_SOUL=0` to keep the robot's soul.
- **Board has ~1 GB RAM and 7 GB rootfs** — watch memory, don't run heavy compiles.