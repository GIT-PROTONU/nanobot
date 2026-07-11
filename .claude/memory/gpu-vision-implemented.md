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

**Not yet built** (explicitly out of scope for this pass — the user asked for "webview shows
normal webcam + GPU is visibly analyzing", not the full backlog):
- Click-to-calibrate UI for blob-tracking's target colour (`GpuVision.set_target_color()` exists,
  nothing calls it yet — blob tracking is dormant/no-op until a target is set).
- The `chase-target.md` skill + `auto_bearing` motion consumer (Feature 1's actual physical
  reaction to a tracked target).
- PIR wiring into `mood_node._camera_beats_ok()` (the `motion` telemetry field exists and is
  live, but nothing in the behaviour layer reads it yet — it's purely informational in the web
  UI so far).
- The four approved Tier-B extensions (motion-saliency center, optical bumper, kinetic
  intercept, flashlight reflex) from [[gpu-vision-features-todo]].
- Nothing has been committed to git — only this memory write. The code sits in the working tree,
  deployed + tested on the robot but not version-controlled yet.
