---
name: gpu-vision-implemented
description: "GPU vision (PIR + blob-tracking + largest-blob selection + tunable blob-size gating + 4 Tier-B extensions + manual/direct mode + tunable optical bumper + colour tracking-mask debug view) BUILT, hardware-verified, and COMMITTED (a881ddc, 2026-07-12); lima boot-load bug + 3 manual-mode bugs found AND fixed. Plus 2026-07-12: motion-mask debug view + 9 live-tunable alerts (obstruction var_max FIXED to 400 from real data), GPU utilization tried then REMOVED (devfreq useless on this board; pipeline-load KEPT), master camera on/off switch + camWait UX fix, hover hints -- ALL DEPLOYED + LIVE-VERIFIED on real hardware multiple times, COMMITTED+PUSHED (96f4913, 45a09e0), README.md/CLAUDE.md now document the whole subsystem"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97322c83-aa6a-4fd1-89af-d8c3f90dd86f
---

Status update, supersedes the "not started" framing in [[gpu-vision-features-todo]]: the core
GPU vision pipeline was **built and verified working end-to-end on the live robot** on
2026-07-11, in the same session as [[gpu-vision-phase0-verified]] and
[[gpu-vision-camera-architecture]]. Not a prototype/spike — this is the real feature module,
wired into `web_server.py`, gated by `gpu_vision_enable` (default `true` as of commit 7060bde,
2026-07-12 — see below).

**Committed 2026-07-12 (commit a881ddc, pushed to main)** — the user explicitly asked for
commit+push after confirming the crosshair fix, largest-blob tracking, and a duplicated
manual-mode toggle all worked live. Fixed same session: `TARGET_LOCK_MIN` (a hardcoded 8%
frontend confidence floor, disconnected from the blob-size tuning sliders) was removed —
"locked"/crosshair now just follow whether the backend reported a target at all, since that
already passed the tuned min/max gate; a real ball settles around ~4.9% confidence, well under
the old 8% floor, so the crosshair/lock had never actually been reachable for a normal target.
Also added connected-component "largest blob" selection (`largest_blob_sums` in
`gpu_vision.py`) so multiple matching regions no longer blend into one wrong averaged centroid.
**Gotcha hit during this session, now RESOLVED**: a full `scripts/deploy.sh` (no package
filter) rsyncs `robot_bringup/config/robot.yaml` verbatim, silently reverting any hand-edit
made directly on the board (like the live `gpu_vision_enable: true` test flip, back when the
repo default was `false`) — this happened mid-session and looked like "the manual-mode button
vanished" until traced to `gv is None` on the board. **Fixed by flipping the repo's own
default to `gpu_vision_enable: true`** (commit 7060bde, 2026-07-12 — the feature is now
hardware-verified enough to ship on by default), so a full deploy no longer disables it. The
general class of gotcha (a full deploy always overwrites board-local hand-edits to
robot.yaml) still applies to any OTHER param someone hand-tweaks directly on the board without
committing it — not fixed generically, just for this one setting.

**What was built** (originally in the working tree pre-commit — code commits need explicit
ask per the user's standing git preference; now committed, see above):
- `src/web_control/web_control/gpu_vision.py` (new) — the full module: headless EGL/GLES2
  context (raw ctypes, no `moderngl` needed — same zero-dependency philosophy as
  `mjpeg_camera.py`), continuous YUYV capture via `mjpeg_camera.MjpegCamera(fourcc=YUYV)`,
  YUYV→RGB conversion shader, PIR motion-diff shader + GPU reduction chain, colour-threshold
  blob-tracking shader + centroid reduction, and a JPEG-encode tee (via a `libturbojpeg0`
  ctypes binding) for the browser live view, gated by viewer ref-count.
- `src/web_control/web_control/mjpeg_camera.py` — `MjpegCamera.__init__` gained a `fourcc=`
  param (default MJPG, zero change for the existing browser-stream caller); exports
  `FOURCC_MJPG`/`FOURCC_YUYV`.
- `src/web_control/web_control/web_server.py` — `gpu_vision_enable` param; when true,
  `self._cam` points at a `GpuVision` instance instead of `CameraStream` (drop-in: `GpuVision`
  exposes the same `add_viewer`/`get_frame`/`remove_viewer`/`running` shape), so
  `/stream.mjpg`/`/snapshot.jpg`/`_capture_frame` needed ZERO other changes. `GpuVision.stop()`
  wired into `destroy_node`.
- `src/web_control/web_control/telemetry.py` — `/telemetry` SSE frame gained a `vision` key
  (`{motion, target}`) read directly from `GpuVision`'s thread-safe properties (no ROS topic —
  it's the same process, a plain Python read is cheaper and simpler).
- `src/web_control/web/{index.html,style.css,app.js}` — a small "GPU vision" badge overlaid on
  the camera view, showing live motion % (and target confidence % once a colour is calibrated —
  calibration UI itself is NOT built yet, `set_target_color()` exists but nothing calls it from
  the web UI).
- `src/robot_bringup/config/robot.yaml` — `gpu_vision_enable: false` (committed default; the
  LIVE robot currently has this hand-flipped to `true` via direct file edit for testing/demo —
  not committed, so a future `git`-sourced redeploy of `robot.yaml` will reset it to `false`
  unless deliberately kept enabled).
- `libturbojpeg0` apt-installed on the robot (542KB, confirmed available in Phase 0 research).

**Debugging findings worth keeping (real bugs hit + fixed on real hardware, useful if this code
is ever touched again):**
1. **`GL_LUMINANCE_ALPHA` upload was mishandled by `lima`** (produced periodic horizontal
   banding with wrong colours) — switched to uploading the raw YUYV bytes directly as an
   `GL_RGBA` texture at half-width (4 YUYV bytes = `Y0 U Y1 V` maps exactly onto one RGBA
   texel's R/G/B/A, zero CPU repacking) and unpacking Y0/Y1 in-shader via `mix()`. RGBA8 is the
   one format every GLES2 impl is guaranteed to get right — prefer it over LUMINANCE_ALPHA on
   this driver for any future shader work.
2. **Classic `glUniform*` gotcha**: uniform calls apply to whichever program is *currently
   bound* via `glUseProgram`, not the program whose location you looked up. Setting a uniform
   before calling `glUseProgram(prog)` silently no-ops (or worse, corrupts a different program's
   uniform state) — always `glUseProgram` first.
3. **Orientation**: empirically determined (via a manual pure-Python YUYV decode as ground
   truth, then a mask-visualization pixel-alignment check) that the GPU readback comes back with
   rows already correct top-to-bottom, but columns mirrored left-right — NOT the "GL is
   bottom-up, flip vertically" assumption a textbook GL pipeline would suggest. A dedicated
   `_FLIP_VS` GPU pass (mirrors `u`) corrects this for the browser-facing output only; internal
   passes (diff/threshold) use the unflipped mapping and apply a `1.0 - raw_u` correction
   directly when computing the reported target centroid.
4. **A real GPU-memory leak, caught by an actual OOM-kill on hardware** (`systemd`:
   `status=killed, status=9/KILL`, `MemoryCurrent` hit the 450MB `nano-app` cap ~4 minutes after
   start): `downsample_chain()` originally called `gl.make_fbo(...)` — allocating brand-new
   GL textures/FBOs — **on every single frame**, for both the PIR-diff and blob-tracking
   reduction chains, and nothing ever freed the old ones. Fixed by splitting into
   `plan_downsample_stages()` (pure size math) + `build_downsample_chain()` (allocates the FBO
   chain ONCE, called during per-thread setup) + `run_downsample_chain()` (zero-allocation,
   reuses the pre-built chain every frame). Verified flat memory (~153MB RSS, ~217-219MB cgroup)
   across 3 checks spanning 2.5+ minutes post-fix, vs. unbounded growth before. **Any future GL
   code in this codebase must pre-allocate all textures/FBOs during setup, never inside the
   per-frame loop** — this is the one hard rule this debugging session established.

**Measured live**: `nano-app` RSS ~153MB / cgroup ~217-219MB with GPU vision active (well within
the 450MB `MemoryMax`), matching the Phase-0 spike's standalone ~70MB context cost plus the
existing app_hub baseline. `/snapshot.jpg` ~40KB (in line with the original hardware-MJPEG
baseline ~38KB). `/stream.mjpg` streamed continuously without issue in a multi-second real pull.

**2026-07-11, same-day follow-up session — all four Tier-B extensions + calibration UI added:**
- **Click-to-calibrate UI, built.** `POST /vision/calibrate {r,g,b,threshold}` or `{clear:true}`
  (`WebServerNode.vision_calibrate`, routed via `POST_JSON`). The browser samples the pixel
  colour itself (canvas `drawImage`+`getImageData` on the live `<img>`, same-origin so no CORS
  taint) and posts the raw RGB — no server-side coordinate mapping needed. Web UI: "🎯 Pick
  target colour" arms click-to-pick on the camera view (`app.js` `onCamPickClick`/
  `camDisplayRect` — the latter correctly accounts for `object-fit:contain` letterboxing), "✕
  Clear target" disables tracking, a crosshair overlay (`#visionCrosshair`) shows the live lock
  position once confidence clears `TARGET_LOCK_MIN` (0.08).
- **Motion-saliency bounding center** — `_DIFF_FS` extended to pack `(magnitude, magnitude*x,
  magnitude*y)` into R/G/B (same trick as the blob-tracker's `_THRESHOLD_FS`), so the existing
  PIR reduction pass gives both the plain motion scalar AND a weighted centroid "for free," no
  extra shader dispatch. `GpuVision.motion_center` property.
- **Kinetic intercept alert** — pure Python, no new shader: a 5-sample ring buffer of
  `(timestamp, target_confidence)` in `_loop`, growth rate = `(confidence_now - confidence_oldest)
  / dt` once ≥3 samples exist. `GpuVision.intercept_rate` property; only meaningful while a
  target colour is set (0 otherwise).
- **Flashlight/dark reflex** — a new `_LUMA_FS` shader (standard perceptual-weight luminance) +
  its own persistent downsample chain, running **unconditionally every frame** (unlike
  diff/threshold, which are conditional on `have_prev`/a target being set) since brightness has
  no such precondition. `GpuVision.luma` property (0..1). The *auto-LED* half is a genuinely new,
  independently-gated opt-in feature: `vision_dark_reflex_enable`/`vision_dark_threshold`/
  `vision_dark_recover` params (hysteresis — recover must stay above threshold), a dedicated
  `/led` publisher + 1Hz timer in `web_server.py` (`_dark_reflex_tick`), deliberately NOT reusing
  the skill-action-tier's `/led` publisher (that's gated by `skills_allow_actions`, a different,
  unrelated opt-in) so the two features stay decoupled.
- **Optical virtual bumper** — lives in `telemetry.py` (not `gpu_vision.py`, which is
  deliberately ROS-free): a lazy `/cmd_vel` subscription (`Twist`) + `_optical_bumper(now,
  motion_score)`, flags "commanded to move but GPU motion score has stayed under the noise floor
  for `BUMPER_CONFIRM_SECS`" (0.6s) as a likely wheel-stall/slip signal. Like everything else in
  `telemetry.py` it only evaluates while a browser is connected (lazy-sub philosophy) — so on a
  fresh reconnect mid-stall there's up to a 0.6s reporting delay before the alert shows; accepted
  as a minor cosmetic limitation, not fixed (informational-only feature, not a real safety path).
- All five new signals surfaced in **both** places: the existing camera-view badge (motion +
  target only, to stay uncluttered) AND a new **"Camera (GPU vision)" card in the Sensors tab**
  (`#panel-sensors`) showing all six readouts (motion, motion center, target lock, intercept
  rate, brightness, optical bumper) **plus the dark-reflex config controls** (enable toggle +
  on/off threshold sliders, wired to `/param` via a new `PARAM_WHITELIST["web_control"]` entry) —
  this card works **whether or not the Camera tab / video stream is open**, since `GpuVision`
  computes continuously regardless of viewers and `/telemetry` is a lightweight JSON poll, not
  the video stream itself. This was an explicit user ask (see the conversation this memory comes
  from) distinct from the camera-view badge.

**A real bug found and fixed during a self-requested "efficiency + correctness" review pass**
(the robot was offline for this session, so this review was code-only, leaning on patterns
already hardware-verified earlier the same day — no live re-test yet, see below): the dark-reflex
timer/publisher were only created if `vision_dark_reflex_enable` was `true` at **node startup**
— toggling it on later via the web UI (the whole point of the `PARAM_WHITELIST` entry) would
silently no-op because the timer calling `_dark_reflex_tick` was never scheduled. Fixed: the
timer/publisher are now always created whenever `gpu_vision_enable` is on (cheap — one param read
+ one property read per second), and `_dark_reflex_tick` reads `vision_dark_reflex_enable` LIVE
each tick rather than only at construction, including turning the LED back off if it was on when
the feature gets disabled mid-flight.

**Efficiency fixes made in the same pass** (concrete, not speculative — found by re-reading the
hot per-frame path with fresh eyes):
1. **Eliminated a double-copy in the JPEG encoder.** `JpegEncoder.encode` was doing
   `bytes(rgba_bytes)` (copy #1, converting the ctypes readback array to a Python `bytes`) then
   `ctypes.create_string_buffer(...)` (copy #2) before ever handing the data to `tjCompress2`. Now
   `tjCompress2.argtypes`'s `srcBuf` param is `c_void_p` (not `c_char_p`), so the pre-allocated
   ctypes readback buffer is passed straight through with zero copies. At ~15fps while a browser
   viewer is watching, this was ~2.4MB/frame of avoidable memcpy (~36MB/s).
2. **Pre-allocated all per-frame readback buffers once**, mirroring the FBO-leak fix's lesson:
   `readback_into()`/`make_readback_buffer()` replace the naive `readback()` (which allocated a
   fresh ctypes array every call) for all four hot-path reads (diff/threshold/luma small buffers +
   the full-resolution JPEG-tee buffer) — the old `readback()` is kept only for the one-off
   `__main__` test/debug helpers, where per-call allocation is fine.
3. **Removed a redundant `create_string_buffer` wrap on the YUYV texture upload** (both in the
   hot loop and the two test helpers) — `buf` from `cam.read()` is already a plain Python `bytes`
   object, and `glTexImage2D` has no explicit `argtypes` declared, so ctypes' default per-call
   argument conversion already turns a `bytes` object into a pointer with zero extra copy; wrapping
   it first just copied the whole frame a second time for no reason.

**Compute budget re-confirmed still generous** even with the luma pass added unconditionally:
baseline continuous cost (no target set, no viewer) is now YUYV-convert + diff-reduce (~1.9ms) +
luma-reduce (~1.9ms) ≈ under 5ms/frame against a 66ms (15fps) frame period — ~13x headroom,
consistent with the Phase-0 measured 1.89ms-per-reduction-chain number. Adding the threshold pass
(target set) or the JPEG tee (viewer watching) each add roughly one more reduction-chain's worth
of cost, still comfortably inside budget.

**Still not built**: the `chase-target.md` skill + `auto_bearing` motion consumer (Feature 1's
actual physical reaction to a tracked target), and PIR/bumper/intercept wiring into
`mood_node._camera_beats_ok()` or any other actual behavior-layer reflex — all five GPU signals
remain informational-only in the web UI; nothing in the robot's autonomous behavior reads them
yet. The dark reflex is the ONE signal with a real actuator response (the LED), and even that is
off by default and fully opt-in.

**2026-07-11, robot reconnected same day — Tier-B extensions now LIVE-VERIFIED, including the
CPU/RAM test.** Deploying surfaced two real, previously-unknown issues, both since fixed:

1. **`systemd-modules-load.service` does NOT reliably load `lima` at boot**, despite
   `/etc/modules-load.d/lima.conf` being correctly in place (confirmed present + correct
   content) — after this reboot `/dev/dri` was entirely absent and `lsmod` showed no `lima`. A
   manual `sudo modprobe lima` worked instantly (exit 0, `/dev/dri/renderD128` appeared), so the
   module itself is fine; the failure is an **ordering problem** — `systemd-modules-load.service`
   runs very early in boot, likely before whatever the Mali GPU platform device depends on
   (display-engine/DRM core) is ready to bind. **Consequence if unnoticed: `gpu_vision.py`
   silently falls back to Mesa's `llvmpipe` SOFTWARE rasterizer** (confirmed via the startup log
   line `renderer=llvmpipe (LLVM 19.1.7, 128 bits)` instead of `renderer=Mali450`) rather than
   erroring — `EGL_PLATFORM=surfaceless` happily hands you a software context if no DRM render
   node exists, defeating the entire point of offloading to the GPU without any obvious failure
   signal. **RESOLVED same day, immediate follow-up session — see [[lima-boot-load-bug]] for the
   full fix + verification.** Short version: `gpu_vision.py` now warns loudly on a non-Mali
   renderer, and `deploy/systemd/nano-app.service` retries the module load right before app_hub
   starts (`ExecStartPre=-+/usr/sbin/modprobe lima` — the `+` was the actual missing piece,
   ExecStartPre otherwise runs as the unprivileged `ibster` user and can't load kernel modules).
   Verified fixed across a genuine reboot, not just a warm restart.
2. **A genuine test-only bug surfaced during dark-reflex verification, NOT a production bug**:
   directly POSTing `/param` with `vision_dark_threshold > vision_dark_recover` (an inverted
   hysteresis band, bypassing the web UI's client-side guard) causes real 1Hz on/off oscillation
   — confirmed via `ros2 topic echo /led` showing rapid `true`/`false` alternation. Added a
   server-side defensive clamp in `_dark_reflex_tick` (`if high <= low: high = low + 0.05`) so
   this can't happen even via direct API misuse, not just relying on the UI's JS guard. The
   underlying live-param-toggle fix (the bug found in the earlier code-review pass) is confirmed
   working — real toggling was observed in direct response to `/param` POSTs.

**Camera device path shifted after reboot**: `/dev/video1` this time (was `/dev/video2` before) —
expected/harmless, USB enumeration order isn't guaranteed stable across reboots and
`mjpeg_camera.find_camera()` already auto-detects via `VIDIOC_QUERYCAP` rather than hardcoding a
path, so this needed no fix, just confirms the auto-detect path works correctly in practice.

**CPU/RAM test results (the one the user explicitly asked for), measured via `/proc/<pid>/stat`
utime+stime deltas over 5s windows, `renderer=Mali450` confirmed active for all of it:**

| Scenario | RSS | CPU (% of one core) |
|---|---|---|
| GPU vision OFF (baseline: web+OLED+behavior only) | 81.3 MB | 50% |
| GPU vision ON, idle (continuous YUYV+PIR+luma, no target, no viewer) | 155.7 MB (+74.4MB) | 70% (+20pp) |
| + blob-tracking target set (threshold pass + intercept calc) | 155.7 MB (+0) | 70% (+0, immeasurable) |
| + browser viewer streaming (`/stream.mjpg` actively pulled) | 157.9 MB (+2.2MB) | 120% (+50pp) |

Key takeaways: the **+74.4MB idle RSS delta matches the Phase-0 spike's standalone ~70MB estimate
almost exactly** — strong cross-session consistency. Blob tracking is genuinely free (buffers are
pre-allocated regardless of whether a target is set, per the leak-fix design). **The browser-tee
JPEG path is the one real, deliberate cost** (+50 percentage-points of one core, i.e. roughly
12.5% of a quad-core board's total capacity) — but only while a viewer is actively watching, and
it drops straight back to 70% the instant they disconnect (confirmed). All of this sits
comfortably under `nano-app`'s 450MB `MemoryMax`.

**Extended stability re-confirmed for the NEW Tier-B code specifically** (the original leak-fix
was only proven for the PIR/threshold reduction chains; the new luma chain + intercept ring
buffer + JPEG double-copy fix hadn't been soak-tested until now): flat RSS across a ~4-minute
mixed-activity soak (idle + periodic snapshot pulls + a streaming burst) — 155.7MB → 158.0MB, a
~2.3MB drift over ~4 minutes that reads as ordinary allocator noise, not a leak (compare to the
original bug's ~85MB/minute unbounded growth). `/snapshot.jpg` and `/stream.mjpg` both confirmed
working through the real production HTTP path throughout. Zero GPU-vision-related errors in the
logs across the whole test session (the one log hit was an unrelated OpenRouter free-tier
rate-limit message, expected/known behavior per the LLM fallback design).

**Final deployed state after this test session**: `gpu_vision_enable: true` (left on so the user
can see it live), `vision_dark_reflex_enable: false` + defaults restored (0.15/0.25), no target
colour calibrated (cleared). Committed git default remains `gpu_vision_enable: false`.

Nothing in this session has been committed to git — only memory writes, per the user's standing
preference. All code (original core + this session's Tier-B extensions) sits in the working tree.

## Manual mode (2026-07-11, same day, third follow-up session)

**Built**: a live, no-restart toggle to bypass GPU vision entirely and get the original
zero-CPU/GPU hardware-MJPEG passthrough back on demand. `POST /vision/manual {enabled: bool}`
(`WebServerNode.vision_manual`) + a "🎥 Manual mode" switch in the web UI's Camera tab.

**Architecture**: `WebServerNode` now ALWAYS constructs `self._cam_direct` (a `CameraStream`,
cheap/idle until viewed, same as before GPU vision existed) regardless of `gpu_vision_enable`.
`self._cam` became a `@property`: returns `self._cam_direct` when `self._gpu_vision is None` OR
`self._manual_mode` is true, else `self._gpu_vision`. Every existing consumer
(`_capture_frame`/`snapshot()`, and `_Handler._stream_mjpeg` after being changed to resolve
`self._node._cam` instead of a construction-time-bound `self._stream`) needed zero further
changes — both backends already share the same `add_viewer`/`get_frame`/`remove_viewer`/
`running` shape. `_stream_mjpeg` pins the backend ONCE per streaming session (at connection
start) rather than re-resolving per call, so a mid-stream toggle affects only the NEXT
connection, not an already-open one — avoids split-brain viewer-count accounting between two
different backend objects. `vision_manual(True)` calls `self._gpu_vision.stop()` (releases the
V4L2 device) before flipping the flag; `vision_manual(False)` flips the flag then calls `.start()`
(reacquires + resumes PIR/blob/luma/dark-reflex). `telemetry.py`'s `vision.manual` field lets the
UI show frozen/stale readouts correctly instead of implying they're still live while paused.

**Testing this surfaced THREE real bugs, all found and fixed the same session** (this is turning
into a pattern for this codebase's GPU work — build it, then find what breaks under an actual
stop/start cycle, not just "does it work once"):

1. **`GpuVision.stop()` never freed the EGL context's ~70MB.** Toggling manual mode dropped CPU
   back to true baseline (confirmed: 70%→50%, exactly matching the "GPU off" baseline) but RSS
   barely moved (~3MB of a possible ~74MB). Root cause: `eglDestroyContext`/`eglTerminate` were
   never called — letting the Python wrapper object get garbage-collected does nothing, ctypes
   has no idea those are cleanup functions. Fixed: `GLContext.close()` now does the full EGL
   teardown sequence, called from all three `_loop()` exit paths (normal stop, capture-read-error,
   and the two early camera-open-failure returns).
2. **Repeated toggling leaked ~3-4MB per cycle even after fix #1**, confirmed via 5 back-to-back
   toggle cycles (155.8→159.8→163.7→166.6→169.3→171.4MB, not plateauing). `eglDestroyContext`
   alone is *supposed* to implicitly free every GL object per spec, but isn't fully honored by
   `lima` (a reverse-engineered driver — consistent with the earlier `GL_LUMINANCE_ALPHA` bug
   found the same day). Two further fixes stacked:
   - Explicit `glDeleteTextures`/`glDeleteFramebuffers`/`glDeleteProgram`/`glDeleteShader` for
     every GL object, tracked automatically via `GLContext`-level lists appended to inside
     `make_texture`/`make_fbo`/`compile_shader`/`program` — this alone cut the per-cycle growth
     from ~3-4MB to ~1MB (a ~75% reduction) but didn't fully eliminate it.
   - **The bigger single fix**: `JpegEncoder` called `tjInitCompress()` fresh every `_loop()` run
     but never `tjDestroy()` — a genuine leak in project code, not the driver. A fresh
     `JpegEncoder` is created every GpuVision restart (manual-mode toggle), so each cycle leaked
     one compressor's internal buffers/tables forever. Fixed: `JpegEncoder.close()` added,
     called from `_loop()`'s cleanup alongside `cam.close()`/`gl.close()`.
   - Net result: per-cycle growth reduced from ~3-4MB to ~1MB residual (likely the remaining
     driver-level imperfection in `lima`, not something fixable from the Python side without
     patching Mesa). At that rate, ~300+ toggles would be needed to threaten the 450MB budget —
     acceptable for the interactive/occasional use manual mode is designed for, NOT
     safe for high-frequency automated toggling in a tight loop.
3. **A real race condition, causing intermittent `503`s on `/snapshot.jpg` after switching manual
   mode off**: `vision_manual(False)` calls `self._gpu_vision.start()` immediately, but
   `self._cam_direct` (`CameraStream`) releases the V4L2 device ASYNCHRONOUSLY in its own
   background thread (`remove_viewer()` just sets a flag; the actual `close()` happens once that
   thread notices) — confirmed on hardware via the exact log line
   `gpu_vision: camera open (YUYV) failed: [Errno 16] Device or resource busy`. Worse, the
   failure path didn't reset `self._run`, so `GpuVision.running()` kept lying "True" forever after
   the thread had actually died — `_capture_frame()`'s frame-wait loop had no way to detect the
   dead thread and just timed out every single call, producing the 503. Fixed: (a) the camera-open
   step now retries up to 6 times with a 0.3s backoff (the busy condition is transient, clearing
   once the other backend's thread catches up), aborting early if `stop()` is called mid-retry;
   (b) EVERY early-return failure path in `_loop()` now explicitly sets `self._run = False` so
   `running()` is accurate. Verified fixed via the exact failing sequence repeated 3x (all `200`)
   plus a more aggressive rapid-toggle-with-concurrent-snapshot test (4x, all `200`, no busy/retry
   even needed that time — but the retry logic is now a permanent safety net for when the race
   does occur, which is real and reproducible, just timing-dependent).

**Verified working end-to-end**: manual-on gives a valid, correctly-oriented direct-passthrough
JPEG (visually confirmed) at a different size/quality than the GPU-tee path (15-42KB vs ~38-42KB,
different encoder settings — cosmetic, not a bug); `/stream.mjpg` streams correctly in both modes;
CPU cost of GPU vision processing is fully eliminated while manual (confirmed back to the true
"off" baseline); telemetry's `manual` flag correctly reflects state and the readouts freeze
rather than silently going stale-but-labeled-live; resuming reliably restores fresh, live
(non-frozen) motion/luma readings.

One elevated-CPU false alarm during testing, resolved by verification rather than more code
changes: a sustained 120% CPU reading that didn't settle during a "quiet" test window turned out
to be three genuinely active browser connections on port 8080 (confirmed via `ss -tn`) — i.e. a
real person/browser watching the live feed the whole time, matching the ALREADY-documented
"idle 70% + streaming 50% = 120%" cost from the original CPU/RAM test matrix. Not a bug; a good
reminder to check for real concurrent viewers before chasing a phantom regression.

## UI polish + optical bumper diagnostics (2026-07-11, same day, fourth follow-up)

**On-video overlay trimmed per user preference**: the "GPU vision X%" badge (`#visionBadge`) was
removed from the camera video view entirely — all six readouts (motion, motion center, target
lock, intercept rate, brightness, optical bumper) live ONLY in the Sensors tab's "Camera (GPU
vision)" card now, which already worked with the video view closed. The target-lock **crosshair**
(`#visionCrosshair`) was removed in the same pass, then explicitly restored per follow-up
request — it's a spatial indicator overlaid on the video (where the tracked colour actually is),
not a text status label, so it stays on the video itself while the numeric badge doesn't.

**Optical bumper made properly usable** (was previously "always clear" with zero way to tell why
or tune it): its three thresholds (`vision_bumper_cmd_eps` 0.03 m/s-or-rad/s,
`vision_bumper_motion_floor` 0.01 gpu-motion-score fraction, `vision_bumper_confirm_secs` 0.6s)
were fixed Python module constants in `telemetry.py`, now real `web_control` ROS params
(declared in `web_server.py`, added to `PARAM_WHITELIST`, in `robot.yaml`) with live sliders in
a collapsible "▸ Optical bumper tuning" section of the Sensors card. `telemetry.py`'s
`_optical_bumper` now returns a diagnostic dict (`{alert, commanded, cmd_vel, low_motion_secs}`)
instead of a bare bool — the `vision.bumper_alert` telemetry key became `vision.bumper` (a
breaking shape change, both `telemetry.py`'s `_build()` and `app.js`'s `onVision` updated
together). **This directly explains the "always clear" symptom**: it was correct behavior, not a
bug — the bumper only evaluates anything while `commanded` is true (robot actually being driven
above `cmd_eps`), and the robot mostly just sits idle. The new `commanded`/`cmd_vel` readout
(labelled "commanded /cmd_vel" in the UI) makes this visible instead of a single opaque flag — a
new hint in the card tells the user how to actually trigger it: drive from the Drive tab while a
wheel is blocked (or pick the robot up), watch `commanded /cmd_vel` go non-zero while `motion`
stays near zero, and after `confirm_secs` held (shown live as "stall held for") it should alert.
Not personally triggered/verified live this session (driving the physical robot wasn't done
without the user explicitly present/expecting it) — the telemetry shape, param-tuning round trip,
and the underlying threshold logic were all verified working; the actual stall-detection trigger
itself is logic already covered by `_optical_bumper`'s existing design, unchanged by this pass.

**Recurring gotcha hit 3 times this session, worth flagging for next time**: pushing the local
repo's `src/robot_bringup/config/robot.yaml` to the robot resets `gpu_vision_enable` back to its
committed default (`false`), silently undoing the live demo override — deploying ANY other
robot.yaml change requires re-flipping it back to `true` afterward (`sed -i` + restart) if the
demo state is meant to persist. Consider: either don't touch the committed default's value casually
between sessions, or remember this step is now a standing part of "deploy robot.yaml + verify
gpu vision still shows `renderer=Mali450` in the log" for this robot specifically.

## Tracking-mask debug view (2026-07-11, same day, fifth follow-up)

**Built**: a "🎭 Show tracking mask" button in the Camera tab that swaps the video feed to a
live black/white view of exactly which pixels currently match the calibrated blob-tracking
colour — the same kind of visualization used earlier this session to debug the threshold shader
itself (`gpu_vision_mask.png`), now a permanent, on-demand feature instead of a one-off test
script. Click again ("🎥 Show normal feed") to swap back.

**Architecture**: mirrors the existing JPEG-tee design exactly, as a fully parallel second
pipeline rather than complicating the first: `GpuVision` gained `_mask_viewers`/`_mask_jpeg`/
`_mask_jpeg_seq`/`_mask_jpeg_cond` (same shape as the normal `_viewers`/`_jpeg`/...) +
`add_mask_viewer`/`remove_mask_viewer`/`get_mask_frame` (same shape as `add_viewer`/
`remove_viewer`/`get_frame`). A new `_MASK_VIEW_FS` shader (reads just the R channel of
`thresh_tex` — the existing threshold pass's 0/1 hit value, ignoring G/B which encode centroid
math for the tracker itself, not for display — and replicates it to grayscale) combined with the
already-existing `_FLIP_VS` (same mirror-correction as the normal live view) renders into a new
`mask_flip_fbo`, read back into a new pre-allocated `mask_flip_buf`, and JPEG-encoded via the
SAME `jpeg_enc` instance as the main tee (two sequential encode calls per tick when both are
active — safe, since everything in `_loop()` is single-threaded, no concurrency between them).
Only computed when `_mask_viewers > 0` AND a target colour is set (mirrors `want_jpeg`'s
viewer-gating) — zero cost when the mask view isn't open, same idle-cost philosophy as
everything else in this module.

New route `GET /stream_mask.mjpg` (`WebServerNode._Handler._stream_mask_mjpeg`, mirrors
`_stream_mjpeg`) fails fast with a clear `503` reason — "gpu vision not active" or "no target
colour set" — instead of hanging forever waiting for a mask frame that will never be computed;
needed a new `GpuVision.has_target_color` property to distinguish "no colour calibrated at all"
from "`target` is None because nothing currently matches," which look identical from `target`
alone. The web UI's `#cam` `<img>` just points at a different URL depending on a `visionMaskOn`
JS flag (`camStreamUrl()` picks `/stream.mjpg` or `/stream_mask.mjpg`); the existing `camOn`
show/hide toggle, viewer ref-counting, and error styling all work unchanged since it's the same
`<img>` element just pointed elsewhere.

**Verified working end-to-end on hardware**, including a spatial correctness check: calibrated a
light blue-grey colour, pulled a mask frame and a contemporaneous normal snapshot, and confirmed
the mask's white region lines up exactly with the light wall visible in the real scene (matching
the earlier session's manual debug-script validation, now proven through the real HTTP
endpoints/UI path instead of a one-off script). Also confirmed the fail-fast 503 fires correctly
with no target colour set, avoiding an indefinitely-hanging stream.

## Blob-tracking tuning (2026-07-11, same day, sixth follow-up)

**Built**: three live-tunable knobs for the colour-blob tracker, adjustable WITHOUT re-picking
the target colour — colour-match tolerance (the existing `_target_thresh`, now exposed instead of
fixed at calibration time) plus new min/max blob-size gating (`_blob_min_confidence`/
`_blob_max_confidence`, fractions of the frame that must match for `target` to be reported at
all — min rejects noise/speckle, max rejects "the whole frame matched" false locks).

**Architecture**: `GpuVision.set_blob_tuning(threshold=, min_confidence=, max_confidence=)` +
`blob_tuning` property (reads `(threshold, min, max)` under the existing lock) — same idiom as
`set_target_color`/`target`. Gating is applied in `_loop()` right where `target` is computed:
`confidence` (matched-fraction) is checked against `[blob_min, blob_max]` before `target` is set,
else `target = None` — same semantics as "nothing currently matches," reusing the existing
`has_target_color` vs `target` distinction (mask view / UI still know tracking is *armed* even
while gated out). Deliberately, `confidence` itself is computed ungated and still feeds the
kinetic-intercept ring buffer regardless of min/max — so intercept-rate trend tracking doesn't
blink on/off at the gate boundary. New `POST /vision/blob_tune {threshold?, min_confidence?,
max_confidence?}` (`WebServerNode.vision_blob_tune`) — all three fields optional/independent,
server clamps each to sane ranges and forces `max >= min`. `set_target_color()` resets tuning to
`(existing threshold, 0.0, 1.0)` (no filtering) on every fresh colour pick, specifically so a new
calibration is never silently invisible because of a leftover min/max from tuning a *previous*
target.

**UI**: collapsible "▸ Blob tracking tuning" section in the Sensors card (3 sliders: tolerance
5-60%, min size 0-50%, max size 1-100%), synced once from telemetry's new `blob_tuning` array the
first time a target colour is present (`blobTuneSynced` flag, mirrors the manual-mode sync
pattern) so the sliders reflect server state rather than always resetting to their HTML defaults.
Reset to defaults explicitly on every fresh colour pick/clear (`resetBlobTuneUI()`), matching the
server-side reset. `telemetry.py`'s `vision` key gained `blob_tuning: [threshold, min, max]`.

**Verified live end-to-end**: calibrated a target (confidence settled at ~4.9% of frame), then
`min_confidence=0.5` correctly filtered `target` to `null` (4.9% < 50%) while `blob_tuning`
telemetry still reflected the updated value — confirmed the gate suppresses reporting without
disabling tracking. Symmetrically, `max_confidence=0.01` also correctly filtered to `null`
(4.9% > 1%), confirming both directions of the range gate work. Reset to clean defaults
(`threshold=0.22, min=0.0, max=1.0`, target cleared) after testing.

## Motion-mask debug view (2026-07-12, dev-host session — CODE WRITTEN, NOT hardware-verified)

**Built**: a second "🎭"-style debug view, `/stream_motion_mask.mjpg` + a "👣 Show motion mask"
button in the Camera tab, showing a live grayscale heatmap of the PIR/motion-diff signal
(brighter = more change since the last frame) — "where is something actually moving," as opposed
to the existing colour-mask view's "where is the tracked colour." Prompted by the user asking for
exactly this after a design discussion about mirroring GPU masks to the OLED (see
[[gpu-vision-features-todo]]'s OLED mask-mirroring section — a related but separate idea; this
build is the browser-stream version, not the OLED one).

**Architecture: an almost-exact copy of the existing colour tracking-mask view**, reusing far more
than expected once the code was actually read: `_MASK_VIEW_FS` (reads the R channel of a texture,
outputs grayscale) already works unmodified on `diff_tex` because `_DIFF_FS` packs its magnitude
into R the same way `_THRESHOLD_FS` packs its hit value (both are "weighted-centroid" shaders by
design, per the comment at `_DIFF_FS`'s definition) — **zero new shader code**. Added: a parallel
ref-counted viewer/JPEG state on `GpuVision` (`_motion_mask_viewers`/`_motion_mask_jpeg`/
`_motion_mask_jpeg_seq`/`_motion_mask_jpeg_cond` + `add_motion_mask_viewer`/
`remove_motion_mask_viewer`/`get_motion_mask_frame`, same shape as the colour-mask trio), one more
persistent flip FBO + readback buffer (`motion_mask_flip_fbo`/`motion_mask_flip_buf`, allocated
once at setup like everything else in this module), and one more viewer-gated render block inside
`_loop()`'s existing `if have_prev:` section (right after the motion score/center are computed) —
draws `mask_view_prog` against `diff_tex` into the new FBO, reads back full W×H, JPEG-encodes,
bumps the sequence counter. **Unlike the colour mask, no "target must be configured" gate is
needed** — motion diff runs unconditionally once a second frame exists, so
`get_motion_mask_frame` only needs to check `running()` on timeout, not an equivalent of
`has_target_color`.

New route `GET /stream_motion_mask.mjpg` (`WebServerNode._Handler._stream_motion_mask_mjpeg`,
mirrors `_stream_mask_mjpeg` minus the target-colour check). Web UI: `app.js`'s `visionMaskOn`
boolean was generalized to a three-way `camMode` (`"normal"`/`"color"`/`"motion"`) with a new
`setCamMode(mode)` toggling whichever button was clicked back to normal if already active —
`camStreamUrl()` now picks between `/stream.mjpg`, `/stream_mask.mjpg`, and
`/stream_motion_mask.mjpg`. New button `#visionMotionMaskToggle` ("👣 Show motion mask") added
next to the existing `#visionMaskToggle` in `index.html`.

**Cost, per the existing measured numbers**: same order as the already-proven colour-mask view —
zero when no viewer is connected, one more full-res pass+flip+readback+JPEG-encode while watched
(the colour mask's equivalent add was already measured as part of the existing viewer-streaming
cost in the CPU/RAM table above; this is a second instance of the same shape, not a new cost
category).

**Verification status — honest gap**: `pixi run build` and `pixi run smoke` both pass (this dev
host has no Mali GPU/camera, so `gpu_vision_enable` never actually engages here — smoke doesn't
exercise this code path). `py_compile` + a `symtable` scope check both pass on the edited files.
**Not yet run on the real robot** — needs a deploy + a live check (open the Camera tab, click "👣
Show motion mask", wave a hand in frame, confirm the stream brightens where the motion is and
stays dark on a static scene) before this can be called verified, following this module's
established "build then hardware-verify" pattern (see the several real bugs found only on
hardware earlier in this file).

**Files touched**: `gpu_vision.py`, `web_server.py`, `web/index.html`, `web/app.js`. Not committed
yet — per the user's standing git preference, code commits need an explicit ask.

## Three more cheap signals (2026-07-12, same dev-host session — CODE WRITTEN, NOT hardware-verified)

User asked for 3 more features from [[gpu-vision-features-todo]] that are cheap AND don't need
the robot to drive to test (a real constraint: several backlog items, like the clutter throttle
or looming brake, need actual driving to observe their effect). Picked the three that are both
cheapest to build and verifiable just by pointing the stationary camera at things:

1. **Motion-intercept rate** (`GpuVision.motion_intercept_rate`) — the exact same ring-buffer
   growth-rate trick as the existing `intercept_rate` (kinetic intercept alert), but keyed on the
   raw PIR `motion` score instead of a calibrated colour target's confidence. **Zero new GPU
   cost** — pure Python over `score`, already computed every tick. Resolves the open item from the
   user's earlier motion-parallax pitch (properly formalized as tau/looming-rate this time, not
   the rejected `Δy_pixel` formula) and the "looming corroboration signal" backlog entry — this
   IS that signal, now built. Test without driving: wave a hand toward the lens, watch the rate
   spike; hold still, watch it settle near zero.
2. **Camera-obstruction / lens-covered flag** (`GpuVision.luma_variance`/`.obstructed`) — variance
   computed over the SAME small downsampled luma buffer already read back every tick for the
   flashlight/dark reflex — **zero new GPU cost**, pure CPU stats on numbers already transferred.
   Flags `variance < OBSTRUCTION_VAR_MAX (15.0) AND luma < OBSTRUCTION_DARK_MAX (0.15)` —
   deliberately requires BOTH (flat AND dark) so a genuinely blank bright wall (also low-variance)
   isn't misreported as an obstruction. **Both thresholds are initial guesses, explicitly not yet
   hardware-tuned** — a real board test (cover the lens with a hand, check it flips; point at a
   blank bright wall, check it doesn't) is needed before trusting the exact numbers, though the
   logic shape itself needs no new infrastructure to test. Test without driving: cover the lens.
3. **Colour-cast (white-balance drift) signal** (`GpuVision.color_cast`) — one new persistent
   downsample-chain instance (`wb_chain`/`wb_buf`), reusing the **already-compiled** `copy_prog`/
   `_COPY_FS` verbatim, run straight on `cur_tex` (the RGB frame) instead of a luminance/threshold
   derivative, so R/G/B stay separable into an `(r,g,b)` average instead of collapsing to one grey
   value like the existing luma pass. **No new shader source** — same cost as any other reduction
   chain instance (~1.9ms per the existing measured numbers), runs unconditionally every frame
   like luma. Test without driving: point the camera at different coloured/lit surfaces, watch the
   R/G/B split shift.

**Wiring**: all three surfaced in `telemetry.py`'s `vision` dict (`motion_intercept_rate`,
`obstructed`, `color_cast`) and the Sensors "Camera (GPU vision)" card (`app.js`'s `onVision` +
new `index.html` rows: "motion intercept", "lens obstructed", "colour cast") — same pattern as
every other Tier-B signal, all informational-only, nothing yet acts on them autonomously.

**Verification status**: `pixi run build` + `pixi run smoke` both pass; `py_compile` clean.
**Not yet hardware-verified** — same honest gap as the motion-mask viewer above, this dev host has
no Mali GPU/camera. Needs a board deploy + manually covering the lens / waving a hand / pointing
at different-coloured surfaces while watching the Sensors card, before the obstruction thresholds
in particular can be trusted. Not committed to git yet.

## Batch: 5 more raw signals + full live-tunable UI for all 9 alerts (2026-07-12, same session)

User asked to add MOST of the remaining [[gpu-vision-features-todo]] items that don't need
driving the robot to test, kept simple, with EVERY result viewable and every threshold tunable
via the web UI, plus real tests run. Delivered:

**New raw GpuVision signals** (all unconditional every tick, none need driving to verify — point
the stationary camera at things):
- `edge_density` — NEW shader `_EDGE_FS` (cheap 3-tap gradient: centre + right + down neighbour,
  not a full 8-tap Sobel), reduced through the existing chain machinery. "Visual interest"/
  clutter, a static complement to PIR's "how much just changed."
- `overhead_edge_density` — the SAME edge shader's output, cropped to the top 30% of frame via a
  second new tiny shader (`_CROP_TOP_FS`, remaps v_uv.y) before reducing. Overhead-clearance
  heuristic (lidar's 2D plane blind spot) — still just a heuristic, camera-mount-vs-lidar-tower
  geometry is unverified, as flagged in the backlog.
- `luma_max` — genuinely free: the existing dark-reflex luma downsample already reads back a
  small multi-cell buffer (chain stops at `min_size=24`, not 1×1), so tracking a max alongside
  the existing sum needed zero new GPU work. `luma_max - luma` is the backlit/dynamic-range
  signal from the backlog.
- `highlight_fraction` — reuses the ALREADY-COMPILED `thresh_prog` with a FIXED near-white
  target/threshold in a separate FBO/chain from the user's calibrated blob target (same program
  object, different uniforms set immediately before each draw — verified safe, matches the
  existing double-use-per-tick pattern the mask-view code already established). Shiny/wet-surface
  signal.
- `motion_target_match` — zero new GPU cost, pure CPU distance between the already-computed
  motion centroid and target centroid. Resolves the "motion-matches-target correlation" backlog
  item.

**Architecture decision: moved ALL threshold/alert logic OUT of gpu_vision.py and into
telemetry.py** (`TelemetryHub._vision_alerts`), mirroring the already-established `_optical_bumper`
pattern exactly — GpuVision only ever exposes raw scalars, alerts are computed live from
`self._node.get_parameter(...)` each tick. This is a deliberate REWORK of last round's
obstruction flag: it was built with hardcoded `OBSTRUCTION_VAR_MAX`/`DARK_MAX` module constants
and no way to tune them from the UI; retrofitted into the same live-param pattern as everything
else in this batch, for consistency and actual tunability. 9 alerts total: `obstructed`,
`clutter`, `overhead_alert`, `focus_blur`, `backlit`, `shiny`, `looming`, `colorcast`,
`motion_matches_target` — each pairs one raw scalar with ONE tunable param (secondary constants
like the obstruction-darkness floor's luma-lower-bound companion were kept as fixed values to
avoid excessive slider clutter, per "keep them simple").

**10 new `web_control` ROS params** (declared in `web_server.py`, whitelisted in `telemetry.py`'s
`PARAM_WHITELIST`, defaulted in `robot.yaml`): `vision_obstruction_var_max/dark_max`,
`vision_clutter_alert`, `vision_overhead_alert`, `vision_focus_blur_max`,
`vision_backlit_delta_min`, `vision_highlight_alert`, `vision_looming_alert`,
`vision_colorcast_alert`, `vision_motiontarget_match_max`. ALL are initial guesses, explicitly
not hardware-tuned — that's exactly what the new live sliders are for.

**Web UI**: every raw signal + alert surfaced as a new row in the Sensors "Camera (GPU vision)"
card (motion↔target, visual interest, overhead structure, shiny surface, backlit, focus/blur —
each shows the raw % AND an inline "⚠" when its alert fires), plus a new collapsible "▸ Vision
alerts tuning" section with all 10 sliders, following the exact same collapsible-toggle +
`setParam` idiom as the existing bumper/blob-tuning sections. Updated the hint paragraph to
explain what to physically do to trigger each one (cover the lens partially, point at a mirror,
hold something up high, etc.) — literally instructions for testing without driving.

**Tests actually run this session** (dev host has no Mali GPU/camera, so these verify the
*code paths that don't need hardware*, not the shaders themselves):
1. `pixi run build` + `pixi run smoke` — both pass, including two NEW smoke assertions added to
   `scripts/smoke_test.py` (a permanent addition, not throwaway): the `/telemetry` frame's
   `vision` dict has all 14 expected keys including the new `alerts` sub-dict with all 9 alert
   keys (verified on a REAL running app_hub process, not just a static check — `gpu_vision_enable`
   defaults true, so `GpuVision` really does construct and its idle-default properties really do
   flow through telemetry.py end-to-end even with no camera attached), and a real `POST /param`
   for one new whitelisted param (`vision_clutter_alert`) is accepted.
2. A throwaway pure-Python unit check (`/tmp/.../test_vision_alerts.py`, not committed) mocking
   `get_parameter`/`GpuVision` to exercise `_vision_alerts`'s 9 formulas against 15 hand-picked
   scenarios, specifically probing the AND-conditions' edge cases: dark+flat (obstructed=True) vs.
   bright+flat (a blank wall, obstructed must stay False) vs. lit+flat (focus_blur=True instead);
   dim+bright-spot (backlit=True) vs. bright+bright-spot (backlit=False); `motion_target_match=
   None` doesn't crash the None-safety check. **All 15/15 passed** on the first logic pass (one
   test-fixture bug caught and fixed along the way — an unrelated default value in the mock, not
   the alert logic itself).

**Deliberately excluded from this batch** (per "keep simple" + "don't need driving," but these
don't fit either constraint): second/named colour target (needs real skill/UI design work, not
just a GPU signal); camera-freeze/vibration diagnostics (need commanded motion to test
meaningfully); charge-LED confirmation/doorway-bias (speculative, unconfirmed hardware/behavior);
personality hooks (anticipatory greeting, mood→face accent, novelty score) — these touch
`behavior`/`mood_node`/cognition, a different subsystem with higher integration risk, not a
"simple first pass"; OLED mask-mirroring — needs cross-node transport design (`gpu_vision.py`
lives in `web_control`, the mask would need to reach `oled_display`), a separate architecture
question from "add a signal + a slider."

**Not committed to git yet** — per the user's standing preference, all of this session's code
(motion-mask viewer + this batch) sits in the working tree pending an explicit commit ask.

## Viewability/tunability audit (2026-07-12, same session)

User asked to confirm every result is viewable and every threshold tunable in the web UI —
prompted a systematic cross-check rather than trusting the earlier build. Method: enumerated all
15 `GpuVision` raw `@property` names via a script, confirmed all 15 appear in `telemetry.py`'s
`vision` dict (they did), then read `app.js`'s actual `onVision()` body line-by-line against that
list (not just skimmed) to find any key that's sent but never rendered to a DOM element.

**Found 2 real gaps**: `luma_variance` and `luma_max` were both in the telemetry frame but ONLY
ever consumed to compute their alert booleans (`obstructed`/`backlit`) — the raw numbers
themselves had no visible row, so a user watching the "lens obstructed"/"backlit" rows could see
the flag flip but never see the actual variance/delta value driving it, or how close it was to
tripping. **Fixed**: `visObstructed2` now reads `"var {luma_variance} clear/⚠ covered/dark"`,
`visBacklit2` now reads `"Δ{(luma_max-luma)*100}% clear/⚠ backlit"` — both raw + alert in one row,
matching the pattern every other row (edge_density, overhead_edge_density, highlight_fraction,
motion_target_match) already used correctly the first time.

**Confirmed correct** (all 10 params, not just the one smoke already checked): wrote a throwaway
script that POSTs `/param` for all 10 new `vision_*` names against a live app_hub instance — all
10 return `{"status":"sent",...}` with the right name echoed back, confirming the whole
`PARAM_WHITELIST` list is correct (a typo in either the whitelist set or a slider's JS `param`
string would have silently 403'd just that one). Cross-checked all 10 slider element IDs between
`index.html` and `app.js`'s `VISION_ALERT_SLIDERS` array textually too (no typos). Re-ran
`pixi run build` + `pixi run smoke` after the fix — still green.

## GPU utilization, "if possible" (2026-07-12, same session)

User asked for GPU utilization next to CPU/RAM in the web UI. Investigated what's actually
available rather than assuming: a TRUE hardware busy-time reading (the modern generic mechanism
is DRM fdinfo, `/proc/<pid>/fdinfo/<fd>` `drm-engine-*` lines, deltas over time — what tools like
`nvtop` use) needs the raw DRI device file descriptor, which Mesa's `EGL_PLATFORM=surfaceless`
opens INTERNALLY inside `eglGetPlatformDisplay` — this code doesn't control that fd today, and
whether `lima` even supports fdinfo on this board's specific kernel version is unverified from
here. Rather than guess, built two DIFFERENT, clearly-distinguished-in-the-UI numbers that don't
need that fd, each honestly labeled for what it actually is:

1. **`gpu_percent`** (System health card, next to CPU/RAM as asked) — a devfreq frequency-ratio
   ESTIMATE (`cur_freq/max_freq * 100`), sysfs-only, zero root/debugfs needed, same discovery-once
   pattern as the existing `_thermal_zones()` (`monitor_node._find_gpu_devfreq()` scans
   `/sys/class/devfreq/*` for a name containing "gpu"). Correlates with load under an
   "ondemand"-style governor but is NOT true busy-time (a GPU pinned at max freq by a different
   governor would misreport as 100%) — labeled as an estimate in the UI hint, not oversold.
   Also added `gpu_temp_c` to the same UI row group since it already existed as an unused
   diagnostics field (`_temp("gpu-thermal")`, defined since before this session) — free to surface
   alongside utilization while touching this exact card.
2. **`gpu_duty`** (Camera/GPU-vision card, "pipeline load") — a SOFTWARE proxy: wall-clock time
   spent in `gpu_vision.py`'s shader-submit+readback block (measured from after the camera frame
   is captured to after `glFinish()`) divided by the frame period. Always available with zero
   sysfs/hardware dependency once GPU vision is running, but measures "how loaded is the vision
   pipeline," not GPU-core occupancy — explicitly documented as a DIFFERENT number from
   `gpu_percent` that "won't necessarily match," in both the property docstring and the UI hint,
   so the two are never confused for the same thing.

**On this dev host** (no Mali, no `gpu-thermal` zone, no GPU devfreq node), both `gpu_percent`
and `gpu_temp_c` correctly resolve to NaN → the string `"nan"` in the diagnostics KeyValue → the
UI shows "n/a" (added `Number.isNaN` guards in `app.js`, since the existing `cpu_temp_c` pattern
would have shown a literal confusing "nan°C" otherwise). `gpu_duty` DOES compute on this host
once `GpuVision` is running (it's pure Python timing, no hardware dependency), so it's real, not
a stub. Verified via `pixi run build` + `pixi run smoke` (added one more permanent assertion:
`/diagnostics` has `gpu_percent`/`gpu_temp_c` keys, and `gpu_duty` is in the vision frame's key
list) — all green. **Not yet known whether `gpu_percent` will read a real number or "n/a" on the
actual board** — depends on whether this specific Armbian kernel build has a devfreq node for the
Mali GPU, genuinely unverified until deployed.

## LIVE HARDWARE VERIFICATION (2026-07-12, deployed via scripts/deploy.sh, real robot)

Deployed the whole session's working tree (motion-mask viewer, all 9 alerts, GPU utilization) via
a full `scripts/deploy.sh` (no package filter — multiple packages touched: `web_control`,
`sys_monitor`, `robot_bringup`). Build succeeded on-board (2min 1s), stack restarted clean, all
5 systemd units UP. **This is the real verification pass the earlier entries were pending.**

**Confirmed working, no errors:**
- `renderer=Mali450` in the app_hub log (real GPU, not the llvmpipe software fallback) — all the
  new shader passes (`_EDGE_FS`, `_CROP_TOP_FS`, the reused `thresh_prog`/`mask_view_prog`
  instances) compiled and ran with zero GL errors/tracebacks across a 5-minute log window.
- `/telemetry` served real, sane numbers for every new field: `motion_center`, `luma`,
  `color_cast` (~neutral grey, correctly no `colorcast` alert), `edge_density`/
  `overhead_edge_density` (low, correctly no `clutter`/`overhead_alert`), `highlight_fraction`
  (0.0, correctly no `shiny`), all 9 `alerts` keys present and behaving sensibly for the scene.
- `POST /param vision_clutter_alert` → real `"sent"` ack on the live node (not just the smoke
  test's mock). `GET /stream_motion_mask.mjpg` → real `200 OK` with multipart headers (not 503).
- `sys_monitor` log confirms `gpu-thermal` IS a real thermal zone on this board — `gpu_temp_c`
  reads a real number (~54°C, tracking `cpu_temp_c` closely, as expected for an SoC with no
  separate GPU die). `gpu_percent` correctly reads `"nan"` → UI shows "n/a" — **confirmed this
  kernel has NO GPU devfreq node** (`sys_monitor` logged "gpu devfreq: not found" explicitly) —
  the honest-degradation design worked exactly as intended, this wasn't a guess that happened to
  be wrong, it's a confirmed real fact about this board's kernel config now.

**Two real, actionable findings from live numbers — not caught by any of the pre-deploy
testing, exactly why "not yet hardware-verified" was the right caveat to keep repeating:**
1. **`vision_obstruction_var_max`'s default guess (15.0) is off by ~2 orders of magnitude.**
   Real `luma_variance` on an ordinary indoor scene reads ~2700-2770 — the obstruction alert
   (`variance < 15`) can never fire at that default; it needs to be re-tuned via the live slider
   (or the default bumped substantially, e.g. into the hundreds-to-low-thousands range) before
   it's useful. Concrete proof the "not yet hardware-tuned" caveat on every alert threshold in
   this batch was not just boilerplate hedging.
2. **`gpu_duty` (the software pipeline-load proxy) reads 60%-190%+ per tick in practice**, not
   the "well within budget" headroom the earlier ~1.9ms-per-chain estimates implied — the 4 new
   unconditional reduction-chain passes added this session (edge, overhead-crop, colour-cast,
   highlight) apparently push real per-tick cost close to or over the 66ms (15fps) frame period.
   The existing `next_t`/`dt` scheduling logic degrades gracefully (no crash, just a lower
   effective analysis rate — confirmed `app_hub` CPU at ~76.7% of one core / 160MB RSS, both only
   modestly above the previously-documented ~70% / ~155-158MB Tier-B baseline, so the board isn't
   struggling, just not hitting nominal fps). Worth knowing before adding more unconditional
   passes on top of this batch, and a candidate for a future optimization pass (not all 6 chains
   strictly need to run every single tick) if more headroom is ever needed.

Nothing crashed, nothing hung, no viewer-gated path (mask views) fired spuriously with zero
viewers connected (confirmed 0 mask-JPEG log lines during a no-viewer window). Code remains
uncommitted to git per the user's standing preference — this was a live functional verification,
not a commit.

## Follow-up fixes from the live-verification findings (2026-07-12, same session, deployed)

User acted on both findings immediately:

1. **`vision_obstruction_var_max` fixed**: 15.0 → **400.0** (`robot.yaml` + `web_server.py`
   default, plus the UI slider range widened from `1-60` to `10-2000` step 10 — the old range
   couldn't even REACH a value near the real ~2700 baseline). 400 leaves comfortable margin
   under a normal scene's ~2700-2770 reading while still (in principle) catching a genuinely
   flat/covered frame — the exact right cutoff for an ACTUALLY covered lens is still unverified
   (never physically tested covering the camera), but the default is no longer structurally
   broken. **Re-deployed and confirmed live**: a fresh reading showed `luma_variance: 2208.22`
   (a slightly dimmer moment than the first check, still normal-scene range) and `alerts.
   obstructed: false` — correctly NOT flagging a normal scene now that the threshold sits below
   it instead of absurdly above it.
2. **GPU utilization + GPU temp REMOVED from the UI** per explicit user request ("remove gpu
   utilization and temp, it['s] useless") — the devfreq-based `gpu_percent` was confirmed
   real-hardware-useless (this board's kernel has no GPU devfreq node, permanently reads "n/a"),
   and GPU temp just tracked CPU temp closely with no distinct information. Removed: `sys_
   monitor.py`'s `_find_gpu_devfreq()`/`_gpu_percent()`/the `gpu_devfreq` attribute/the
   `gpu_percent` field entirely (new code, fully deleted); the System health card's "GPU"/"GPU
   temp" rows and their `app.js` wiring (also fully deleted); the smoke-test assertion that
   checked for `gpu_percent`. **`gpu_temp_c` itself was NOT deleted from the backend** — it
   pre-dated this session (an existing unused diagnostics field), only its NEW UI exposure was
   removed, so this is a value-neutral revert of this session's addition, not new deletion of
   old code. **`gpu_duty`" (the Camera card's "pipeline load," a different, software-measured,
   demonstrably-varying metric) was KEPT** — not what the user called useless, and it already
   proved informative (the 60-190%/tick finding above). Updated the Camera card's hint text to
   drop the now-invalid "compare to the System card's GPU reading" cross-reference. Re-deployed,
   re-verified live: `diag` frame confirmed `gpu_percent` key is gone, `gpu_temp_c` still present
   in the backend (67.8°C this check) but simply not surfaced in the UI anymore.

## Camera-disabled UX fix (2026-07-12, same session, deployed+verified)

User reported the master camera-disable toggle left the live-view `<img>` showing a bare red
broken-image square (the old `onerror` handler just set `background:#3d1418`) and that
re-enabling required a manual page refresh to get the feed back. Fixed both:
- New `#camWait` overlay div (same centered-message pattern as the map view's `#mapWait`) shows
  a real message instead of a broken image: "📷 Camera disabled — enable it in Sensors → Camera
  (GPU vision)" when the master switch is off, vs. "⚠ Camera unavailable — check the connection"
  for any other stream failure — `lastCameraEnabled` (tracked from telemetry's `camera_enabled`
  key) picks which text to show.
- **Auto-retry on re-enable**: `onVision` now diffs `camera_enabled` tick-to-tick; the instant it
  flips false→true while the camera checkbox is on, it resets `<img src>` itself (no user action
  needed) — this directly fixes the "I have to refresh the page" complaint. It also reacts
  proactively on the disable side too (shows the message the moment telemetry reports
  `camera_enabled:false`), not only when the stream connection itself eventually errors — matters
  because `_cam` is resolved once per TCP connection (see `web_server.py`'s comment on `_cam`), so
  an already-open stream wouldn't otherwise notice a mid-stream disable until reconnecting.
- Verified live: served page confirmed to include `id="camWait"`; disable/re-enable round-tripped
  cleanly via `/vision/camera_enable` + `/telemetry` with no errors in the logs.

Committed (see git log) and pushed alongside the rest of this session's GPU vision work.

**Unrelated finding surfaced in the post-fix log check, NOT caused by anything in this session's
diff** (worth a note for a future session, not chased further here): a single
`AttributeError: 'WebServerNode' object has no attribute '_cog'` from a request arriving at
11:02:46, ~2-3 minutes after the restart. `self._cog = CognitionCore(...)` is assigned early and
unconditionally in `__init__` (line ~415) — nothing in this session touched that path — so this
reads like a narrow HTTP-server-accepting-connections-before-`__init__`-finishes startup race,
most likely triggered by opening/refreshing the browser tab right as the restart was still
settling. Not reproduced/investigated further; flagging in case it recurs.
