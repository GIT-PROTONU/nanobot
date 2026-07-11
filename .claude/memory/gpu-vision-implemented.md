---
name: gpu-vision-implemented
description: "GPU vision (color-blob tracking + PIR motion diff) BUILT and verified working end-to-end on hardware 2026-07-11 — camera capture, shaders, JPEG tee, web UI all wired; a real GPU-memory leak was found and fixed"
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

**Verification status of this session's additions: CODE-REVIEWED ONLY, NOT LIVE-TESTED.** The
user disconnected the robot partway through ("ill disconnect the robot for now just make the
code ready") before the Tier-B extensions could be pushed/verified on hardware, unlike the
original PIR/blob-tracking/JPEG-tee core (which WAS fully hardware-verified earlier the same
day — see above). All Python files pass `py_compile`; HTML/CSS/JS were checked for brace/paren
balance and cross-referenced ID matching, but none of it has run on the actual Mali-450/lima
driver yet. **A live re-test (including the CPU/RAM impact test the user explicitly asked for) is
still outstanding and must happen next time the robot is reachable** — treat the four Tier-B
extensions and the calibration UI as "should work, same patterns as the hardware-verified core,
but unproven" until then, distinct from the core PIR/blob/JPEG-tee pipeline's "proven on
hardware" status.

Nothing in this session has been committed to git — only memory writes, per the user's standing
preference. All code (original core + this session's Tier-B extensions) sits in the working tree.
