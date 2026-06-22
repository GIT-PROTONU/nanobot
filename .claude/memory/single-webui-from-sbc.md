---
name: single-webui-from-sbc
description: There is only ONE web UI now — the one served from the SBC (web_control); ignore any other/earlier web UI
metadata: 
  node_type: memory
  type: project
  originSessionId: 5d13d972-dd72-4442-a232-c24364a49aa9
---

As of June 2026 there is **only one web UI**: the one served from the SBC by `web_control`
(rosbridge + static `web/index.html`, plus the MJPEG + mic passthrough). Any reference to a
second/alternate web UI (e.g. a dev-host-served page) is obsolete — treat the SBC-served one
as the single source of truth for the operator UI.

This is what shows live robot data (wheel_ticks, lds_rpm, suspension, camera, etc.), so it's
the real end-to-end test that the ESP32→[[esp32-zenoh-pico-integration]] link is healthy.
