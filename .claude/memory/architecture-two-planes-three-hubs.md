---
name: architecture-two-planes-three-hubs
description: "2026-07-06 architecture overhaul — rosbridge deleted (SSE /telemetry gateway in web_control), three node hubs (sensor/nav/app), per-unit systemd supervision"
metadata: 
  node_type: memory
  type: project
  originSessionId: cb34647d-a54a-4283-895b-3f5b2e1a7a22
---

Big 2026-07-06 restructure ("rethink the whole architecture", all three phases built the
same day):

**1. rosbridge is GONE.** It cost ~a full core with the web UI open ([[sbc-cpu-profile]]);
everything heavy had already been routed around it. `web_control/telemetry.py`
(`TelemetryHub`) now serves the browser: `GET /telemetry` = ONE SSE stream (browser
EventSource, native auto-reconnect) of a ~5 Hz JSON frame with every light readout, built
once per tick and fanned out to all viewers; `POST /publish` (per-topic whitelist+clamp) and
`POST /param` (node/param whitelist → set_parameters) are the write paths. The telemetry
subscriptions are **lazy** — created on the first client (on the executor thread via the
tick timer, to avoid create-while-spinning races), dropped `SUB_LINGER` (15 s) after the
last — so idle cost ≈ 0. roslib/the CDN script/the ws host input are removed from the page;
`ros-humble-rosbridge-suite` was removed from pixi.toml (disk win on the 7 GB card).
The power buttons now only POST `/system/*`; the SERVER publishes `/oled_system`.
Dev-harness fallback unchanged: no /telemetry → page shows disconnected, HTTP pollers
(brain card, /oled/state mirror) take over.

**Why: rosbridge's cost was per-incoming-message rclpy work + per-client JSON/ws framing;
the frame approach pays one deserialize per topic sample (only while a browser is open)
and one JSON dump per tick regardless of viewer count.**

**2. Three node hubs = three fault domains** (plus the zenoh router): `sensor_hub` (the
body: imu+sys+odom+lds, pre-existing), `slam_nav` (spatial), and NEW **`app_hub`**
(expression/cognition: web_control + oled_display + behavior/mood_node in one executor,
`src/app_hub/`, mirrors sensor_hub). app_hub's main preserves the OLED SIGTERM
end-screen (spin_once loop + shutdown_sequence). Saves ~2 rclpy interpreter baselines.
`bringup.launch.py` (dev/sim path) still launches the nodes separately — same graph.

**3. Supervision is systemd-native.** Five units under `nano-robot.target`
(`nano-router|app|sensors|nav|map`), `After=nano-router.service` + a 6 s router settle
sleep encodes the rmw_zenoh island gotcha, `Restart=on-failure` replaces
[[stack-autoheal]] (and its heal-vs-restart duplicate-node race). Each unit runs
**`scripts/unit_exec.sh <name>`** — the ONE command table: `pixi shell-hook` env
activation, sources install/setup.bash, resolves `OPENROUTER_API_KEY` from
`memory/openrouter_key`, then `exec`s the installed executable (no resident wrapper).
`scripts/stack.sh` is now a thin `sudo -n systemctl {start|stop|restart} nano-robot.target`
wrapper (scoped NOPASSWD rules in deploy/sudoers/nano-power — exact commands, no
wildcards: systemctl takes multiple unit args so a glob would be an escape hatch).
Logs moved from `.run/*.log` to journald (`journalctl -u nano-app`).

**Deploy note:** after deploying this, run `sudo bash deploy/sbc-setup.sh` ONCE on the
board (installs the new units, removes nano-stack/nano-heal) — until then the new
stack.sh refuses with a pointer. **Not yet deployed/verified on the live board as of
2026-07-06** (board was unreachable); verified on the dev PC end-to-end (app_hub hosting
3 nodes, SSE frames incl. latched purpose, whitelist rejects, /publish→OLED face flowed).

**Hardening tier (same day, second pass):**
- **systemd watchdog**: nano-app/sensors/nav are `Type=notify` + `WatchdogSec=90`; each
  main sends READY=1 then pets WATCHDOG=1 from a **5 s executor timer** (`_sd_notify`,
  deliberately duplicated ~10 lines per main — no cross-package util dep). A wedged
  executor (stuck callback) stops petting → systemd restarts the hub. This is the hang
  coverage `Restart=on-failure` lacks. Verified against a fake NOTIFY_SOCKET.
- **Router readiness**: nano-router's ExecStartPost is a bash /dev/tcp probe on :7447
  (up to 30 s) instead of a blind sleep — After= now means "router accepts".
- **MemoryMax** per unit (app 450M / sensors+nav 300M / router+map 150M): a leak
  restarts one hub instead of invoking the kernel OOM killer on the 1 GB board.
- **Vitals blob** `/dev/shm/nano_vitals.json`: sys_monitor writes ONE aggregated body
  snapshot per tick (cpu/mem/temp/disk + imu a/g/hz/tilt + lds hz + esp hb/temp,
  NaN→null because it re-enters browser JSON, per-source ages + wall-clock `t`).
  oled_display's 5 telemetry subs are GONE (dashboard reads the blob, /proc fallback
  when stale); web_control's imu/web+imu/euler subs are GONE (cognition snapshot +
  frame imu/eul read the blob). sys_monitor is the only IMU subscriber left and is
  co-resident with imu_driver → IMU samples never cross a process. Page staleness
  thresholds were loosened (imu age <3 s) since vitals tick at 1 Hz.
  Convention: one writer per /dev/shm nano_* file, atomic os.replace.
- **`pixi run smoke`** (`scripts/smoke_test.py`): boots router+sys_monitor+app_hub for
  real and asserts the frame keys, whitelists, face echo, cross-process diag, vitals,
  and clean SIGTERM. THE check for the untyped telemetry.py<->app.js frame contract —
  run before deploying. All 12 checks green on the dev PC 2026-07-06.

**How to apply:** any new browser-visible readout goes in `TelemetryHub._build` (+ lazy sub);
any new page write goes through the /publish or /param whitelist — never a new transport.
Slow consumers of fast topics read a /dev/shm blob instead of subscribing.
New processes = new fault domains only; otherwise join a hub.
