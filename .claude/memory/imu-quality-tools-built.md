---
name: imu-quality-tools-built
description: "2026-07-16: 6-axis mode toggle, automated interference self-test, mag-cal scatter view, bandwidth filter all built same-day as the backlog request; smoke/unit/manual-tested, NOT hardware-verified/deployed. Follow-up same day: mag scatter view REMOVED, replaced by mount offset/rotation 3D indicator + spin check — see [[imu-mount-rotation-fixed]]"
metadata: 
  node_type: memory
  type: project
  originSessionId: 86e220b7-6c44-4d8e-9467-f21fce9a03e4
---

**Follow-up, same day, later session — superseded/extended:**
- **Mag-cal scatter view (item 3 below) was REMOVED** at the user's request (decluttering
  the IMU card) once the actual mount problem was found and fixed — see
  [[imu-mount-rotation-fixed]]. The mag xyz/magnitude/noise numeric readouts stay; only
  the canvas scatter plot + its buttons/JS are gone.
- **New: a 3D mount-offset axis triad** in the existing CSS-3D attitude view (`imu3d-*` in
  `style.css`/`app.js`) — a small red/green/blue gizmo (X-forward/Y-left/Z-up) that sits
  on the body's centre dot at offset `0,0,0` and slides out live as the `offset_x/y/z_mm`
  fields change, so the numbers can be visually matched to where the sensor actually sits.
  Verified with matrix algebra against the rig's own documented rest pose before shipping
  (forward=+Z, right=+X, up=-Y).
- **New: a front/back arrow (▼) drawn on the 3D model's roof**, always visible from the
  fixed camera angle (unlike the coloured front face, which can rotate out of view) —
  added because the coloured-face-only indicator wasn't unambiguous enough on its own.
- **New: a "spin check"** — client-side-only, reuses the existing `|accel|`/`|gyro|`
  stream: while the robot is actually rotating (`|gyro|` > ~15°/s), a CORRECT mount
  offset should keep `|accel|` pinned near 9.81 (lever-arm correction cancels the
  rotation-induced term) just like when parked; this grades the residual live plus a
  peak-hold "worst spin residual" (mirrors the existing mag-noise worst-seen idiom).
  This tooling is what actually led to finding + fixing the real mount problem — see
  [[imu-mount-rotation-fixed]] for the resolution and the mount-ROTATION correction
  (`mount_roll/pitch/yaw_deg`) that came out of using it.
- **New: a "Clear map" button** (Map card) — unrelated to the IMU work but built same
  session: wipes `slam_nav`'s occupancy grid and re-seeds from the current pose, like a
  fresh boot. Surfaced a real pre-existing bug in the process — see
  [[web-publish-topic-namespace-gotcha]].

Built same session as [[software-features-todo]]'s "IMU quality tools" entry (user said
"add all to todo" then "execute these" in the same conversation). All four target the
open [[selftest-spin-imu-mismatch]] mag-interference suspicion and build on
[[imu-calibration-added]]'s WitMotion command channel. **Committed + smoke-tested +
198 unit tests pass + manually verified against real HTTP endpoints on the dev host
(no real BWT901CL attached) — NOT yet run against actual hardware or deployed.**

**1. 6-axis mode toggle (`imu_node.py`).** New `_CAL_CMDS` entries `axis6`/`axis9`
write WitMotion register `0x24` (ALG: 1=6-axis gyro-only yaw, 0=9-axis mag-fused yaw —
per WitMotion's documented table, not verified against this specific unit) via the
existing unlock/write/save pattern. `zero_yaw` writes CALSW (`0x01`)=4 then exits. Web
UI: 🧭/🧲/↺ buttons in the IMU card (`imuAxis6`/`imuAxis9`/`imuZeroYaw`), confirm()
dialogs, routed through the same `/imu_calibrate` whitelist
(`telemetry.py._mk_calibrate` extended).

**2. Bandwidth filter (`imu_node.py`).** New `bandwidth_hz` param (0=leave device
default, else nearest of 256/188/98/42/20/10/5 Hz programmed into register `0x1F` in
`_configure_device`). Whitelisted in `PARAM_WHITELIST["imu_driver"]`. Web UI: a select
next to the axis buttons. Declared in `robot.yaml` (default 0 = off).

**3. Mag-cal quality scatter (`app.js`, pure client-side).** A `<canvas id="imuMagScatter">`
in the IMU card; `onImuMag` appends (x,y) points to `magScatterPts` while
`magScatterOn` is true, toggled by the existing Start/Stop mag-cal buttons (`imuCalMagStart`
sets it + clears the plot, `imuCalMagStop` clears the flag). Auto-scaled scatter with a
crosshair — a circle centred on it = good hard-iron cal. Zero backend/ROS changes (mag
xyz already streamed).

**4. Automated interference self-test (new `web_control/imu_interference.py` +
`web_server.py` wiring).** `IMUInterferenceTest` class: single-flight background
thread, 4-5 phases (baseline / LDS spin / fan 100% / LED / optional motor wiggle),
~10Hz polling of `telemetry._mag`+`telemetry._eul` per phase, reports mag-noise
(raw + % of field) and yaw-wobble per phase, ranked worst-first in the web UI.
**Safety design (matches project conventions, not new machinery):**
  - Gated entirely on `skills_allow_actions` (reuses `node._skill_pubs`, the SAME
    LED/fan/LDS/cmd_vel publishers the skill action tier already uses — no second
    unguarded actuation path). Returns a clear error if off.
  - Fan phase goes through sys_monitor's `fan_override` PARAM (not a raw /fan_pwm
    publish), since sys_monitor is /fan_pwm's sole continuous owner — a competing
    publish would just get overwritten on its next tick.
  - Preconditions before starting: IMU mag/euler actually streaming, robot not
    picked up, not being driven (reuses `vision_bumper_cmd_eps`, same "commanded"
    test as the drift check/optical bumper).
  - Live re-check every ~0.1s during each phase; aborts + restores every actuator
    (fan auto, LED off, motors stopped) if the robot is picked up or starts being
    driven mid-run (except the motor phase itself, which deliberately skips the
    "being driven" check since it's the one driving).
  - LDS is NOT force-reset after its phase — matches the "slam_nav's own idle-park
    logic owns LDS spin-down" convention already used elsewhere.
  - New `robot.yaml` params: `imu_test_lds_rpm` (300, matches `slam_nav.lds_active_rpm`),
    `imu_test_motor_ang` (0.35 rad/s).
  - Endpoints: `POST /imu/interference/start {include_motor}`, `POST .../stop`,
    `GET .../status` — same poll-while-active pattern as `stress.py`/`stressPoll()`.
  - Web UI: ▶/■ buttons + "include motor wiggle" switch in the IMU card, results
    rendered as colour-graded rows (reuses the SLAM-margin thresholds from
    [[imu-calibration-added]]'s follow-up: red ≥6% mag noise, amber ≥2%).

**Verification done:** `pixi run build`, `pixi run smoke` (full pass), the existing
198 unit tests (`src/web_control/test` + `src/behavior/test`, all pass unchanged), plus
an ad hoc script booting sys_monitor+app_hub and hitting the new endpoints directly:
confirmed `bandwidth_hz` clears the param whitelist (fails later only on "imu_driver
not reachable" since imu_driver isn't part of this harness — same limitation
`pixi run smoke` already has for other nodes), the three new calibrate commands are
accepted and a bogus one is still rejected, `/imu/interference/status` returns a clean
idle status, `/imu/interference/start` correctly refuses without `skills_allow_actions`,
and — with that flag forced on — correctly refuses with "IMU mag/euler not streaming
yet" (no real IMU in the harness) instead of doing anything unsafe.

**Not yet done:** flashed/verified against a real BWT901CL (register `0x24`/`0x1F`
values are per WitMotion's documented table, not confirmed against this specific unit
— same caveat as [[imu-calibration-added]]), not deployed to the board. Next step:
deploy, then try 6-axis mode + a bandwidth drop and re-run the self-test SPIN check to
see if it measurably tightens the drift documented in [[selftest-spin-imu-mismatch]].
