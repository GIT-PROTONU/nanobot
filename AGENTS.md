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
  All 93 tests are ROS-free (no rclpy, no network). The `nanobot-brain` package is a standalone dependency — no colcon overlay needed.

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

## Gotchas

- **`stack.sh restart` is unreliable** — can leave stale processes holding ports. Clean `down` → verify via `/proc` → `up`.
- **`brain_timeout` must stay well above `reflect_period`** (invariant: timeouts shorter than the reflection gap cause the chart to revert accumulated drift).
- **Heavy topics bypass rosbridge:** `/map` and `/scan.bin` are served from `/dev/shm` via HTTP, not bridged.
- **`rmw_zenoh` ordering:** a node started before `rmw_zenohd` runs islanded (won't appear in the graph).
- **Python edits are live:** `--symlink-install` means edit `src/<pkg>/<pkg>/foo.py`, restart node = picked up. New modules import fine via egg-link.
- **nanobot-brain is pip-installed**: edit `src/nanobot_brain/` in the nanobot-brain repo, restart node = picked up (editable install).
- **`config/robot.yaml` is the single config source** — all ports, pins, rates, LLM params live there.
- **`plink -m` on Windows:** the script text becomes the shell's argv. `pkill -f` patterns can kill the controlling shell. Fix: `pscp` script, run by path.
- **ESP32 firmware:** PlatformIO from dev PC (`pio run -t upload`). Don't build on the board. Tunables are `#define`s at top of `src/main.cpp`.
- **Deploy soul overwrite:** `DEPLOY_SOUL=1` (default) pushes `devstate/` personality to board, discarding evolved drift. Set `DEPLOY_SOUL=0` to keep the robot's soul.
- **Board has ~1 GB RAM and 7 GB rootfs** — watch memory, don't run heavy compiles.