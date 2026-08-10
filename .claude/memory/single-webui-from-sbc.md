---
name: single-webui-from-sbc
description: There is only ONE web UI now — the one served from the SBC (web_control); ignore any other/earlier web UI
metadata: 
  node_type: memory
  type: project
  originSessionId: 5d13d972-dd72-4442-a232-c24364a49aa9
---

As of June 2026 there is **only one web UI**: the one served from the SBC by `web_control`
(static `web/index.html` + the SSE `/telemetry` gateway + MJPEG/mic passthrough — rosbridge
was removed 2026-07-06). Any reference to a second/alternate web UI (e.g. a dev-host-served
page) is obsolete — treat the SBC-served one as the single source of truth for the operator
UI. (`scripts/dev_webui.py` serves the same page off-robot for AI/TTS dev — a harness, not
a second UI.)

This is what shows live robot data (wheel_ticks, lds_rpm, suspension, camera, etc.), so it's
the real end-to-end test that the ESP32→[[esp32-zenoh-pico-integration]] link is healthy.

## Page state (RESOLVED 2026-08-10) — self-contained SSE, no split files

`web/index.html` is a **single self-contained SSE page**: one big `"use strict"` inline
block (`app.js`-derived EventSource on `/telemetry` + all control) with `oled.js`
inlined before it, then self-contained IIFE blocks (map/chrome/sim/motion-chain/EKF/
slam-tuning). There is **no rosbridge/ROSLIB** and **no `<script src>` loading** of the
old split files (`app.js`/`map.js`/`oled.js`/`chrome.js`/`sim.js`/`devtools.js`/`logs.js`/
`personality.js` — committed but now ORPHANED). Do not re-add external script loading;
keep new features as inline SSE blocks following the existing pattern.

**Diagnosing the "disconnected" indicator:** the reporter's JS console errors were
`ROSLIB is not defined` (a stale build) + `Invalid destructuring assignment target`
(a browser-parse-breaking line, fixed by destructuring the `bindSlider` opts in the
body) + `404 /purpose|/task_current|/experiments` (harmless — those GETs only fire in
`pollBrainHttp` while disconnected; brain readouts come from the SSE `f.*` fields).
Verify with a headless Chromium against the board IP: `#conn` should read "connected"
and `#dot` get class `ok`.

**Telemetry JSON gotcha:** `DiagnosticStatus.level` arrives as raw `bytes` under
rmw_zenoh (`b'\x00'|b'\x01'|b'\x02'`) — non-JSON-serializable → unhandled `TypeError`
in `telemetry._tick` kills the whole app hub (systemd respawn loop). Normalize to `int`
at ingest and keep every new frame field JSON-safe after the zenoh round-trip.
