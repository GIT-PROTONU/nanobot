---
name: gpu-vision-implemented
description: "GPU vision (PIR + blob-tracking + 4 Tier-B extensions + manual/direct mode) BUILT and fully hardware-verified 2026-07-11, incl. CPU/RAM numbers; lima boot-load bug + 3 manual-mode bugs found AND fixed same day"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97322c83-aa6a-4fd1-89af-d8c3f90dd86f
---

Status update, supersedes the "not started" framing in [[gpu-vision-features-todo]]: the core
GPU vision pipeline was **built and verified working end-to-end on the live robot** on
2026-07-11, in the same session as [[gpu-vision-phase0-verified]] and
[[gpu-vision-camera-architecture]]. Not a prototype/spike — this is the real feature module,
wired into `web_server.py`, gated by `gpu_vision_enable` (default `false`).

**What was built** (all in the working tree, NOT committed to git — code commits need explicit
ask per the user's standing git preference; only this memory write is committed):
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
