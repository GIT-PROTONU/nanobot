---
name: gpu-vision-camera-reconnect
description: "2026-07-16: live incident — C270 webcam dropped off USB mid-session (dmesg confirmed), gpu_vision.py had zero reconnect logic and stayed dead until a manual restart; fixed with auto-reconnect + fresh device rediscovery, deployed+verified"
metadata: 
  node_type: memory
  type: project
  originSessionId: c5961d6a-8b34-4313-9d6d-8ead3f62d4e5
---

Real hardware incident, not a hypothetical: mid-session the Logitech C270 physically
dropped off USB (`dmesg`: `usb 4-1: USB disconnect, device number 2` /
`uvcvideo 4-1:1.1: Failed to resubmit video URB (-19)`), then re-enumerated ~17s later
as a NEW device number, landing on a **different** `/dev/videoN` path (`/dev/video0` →
`/dev/video1`). Happened during a run of several `stack.sh restart`s — correlation, not
confirmed causation; could be coincidental flaky-hardware timing or could be related to
the known power/ground marginality tracked in
[[esp32-hardware-fried-ground-fix]] (a brief voltage dip during a restart resetting the
USB hub). Worth watching if it recurs.

**The bug**: `gpu_vision.py`'s capture loop (`GpuVision._loop`) had a retry-on-*initial*
open (tolerating a transient "device busy" race with the direct-passthrough fallback),
but ANY read error once already streaming (`cam.read()` raising, e.g. the `ENODEV` from
this disconnect) set `self._run = False` and permanently ended the thread — the
elaborate GL context/shaders/textures just got silently abandoned. Nothing supervises
or restarts this thread; it needed a full `nano-app` restart to recover, and nothing
in the telemetry frame screamed "camera is dead" beyond the vision fields quietly
flatlining at zero (`frame_age` stuck, `luma`/`edge_density`/`motion` all 0).

**The fix**: extracted `_open_camera_with_retries()` (used for both the initial open
and reconnects) and changed the per-frame read-error handler to close the stale handle
and retry (2s backoff) instead of giving up — critically, **re-running
`mjpeg_camera.find_camera()` fresh each time** rather than reusing the cached device
path, since the path itself can shift across a real re-enumeration (confirmed: 0→1
here). The GL context/shaders/FBOs are NOT torn down across a reconnect — only the
camera handle is reopened — so recovery is fast and cheap. Only bails permanently
(logs + stops cleanly) if the reconnected camera renegotiates a different resolution
than the textures were built for, which should never happen for the same physical
hardware reconnecting.

**Verified live on hardware**: triggered the real failure (`journalctl` showed
`capture read error: [Errno 19] No such device` → `gpu_vision: stopped`), deployed the
fix, and confirmed a subsequent restart recovered cleanly (`GL context up` →
`capturing /dev/video1`) with live telemetry data flowing again. The actual auto-
reconnect path (mid-session drop → automatic recovery without any restart) has NOT yet
been hardware-triggered again to directly confirm the reconnect code path itself runs
end-to-end on real hardware — logically sound and code-reviewed, but next real USB
blip is the first live test of it.
