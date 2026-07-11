---
name: gpu-vision-features-todo
description: GPU (Mali-450/GLES2) vision backlog/design history — core features built (see gpu-vision-implemented); this file now also holds an unbuilt "cheap-tier" brainstorm (white-balance drift, obstruction flag, 2nd colour target, edge-density, backlit detector, motion/target correlation) plus rejected/hardware-blocked ideas
metadata: 
  node_type: memory
  type: project
  originSessionId: 0511c092-56af-4656-9941-b2e947bb7aaf
---

User approved 2026-07-10 (session `0511c092`): two vision features to build using the Mali-450 GPU (GLES2 fragment shaders, not OpenCV — see [[h5-gpu-only-webcam-use]]). A GPU blacklist was briefly added the same session then reverted once these features were approved, so the `lima` driver is untouched — no re-enable step needed, `deploy/sbc-setup.sh` never shipped the blacklist.

**1. Color-threshold blob tracking → a bearing.** Shader thresholds camera frame by hue, downsample-reduces the mask to a centroid on-GPU, reads back only `(x, y, confidence)` — a few bytes, not an image. Convert x-position to a bearing via the camera FOV, publish on something like `/target`. A small CPU-side proportional controller turns/drives toward it. Unlocks: chase-a-colored-ball play skill, camera-based dock approach (put a colored marker at the charging dock — currently there's no vision-based return-to-dock, only lidar/SLAM pose), orient-before-commenting for the "looking" beat.

**It's color-based, not shape-based** (a deliberate fit for ES2's no-compute-shader limits, not a shortcut) — it tracks "anything this color," not specifically a ball, so false positives (skin tone, wood floor, warm lighting near the target hue) are a real risk, mitigated by calibrating on the actual object under the actual room lighting rather than a hardcoded color preset. The GPU computes a full-res binary mask internally but that image never leaves the GPU — only the fully-reduced tiny result crosses back. If shape/circularity discrimination is ever wanted (CPU-side, cheap NumPy over the already-thresholded mask), that needs reading back a small **downscaled mask image** (e.g. 64×64, a few KB) instead of just the reduced scalar — a different, slightly pricier readback than pure bearing-tracking; not part of the current plan.

**Calibration ("teach it what color to track"):** planned as click-to-calibrate in the web UI — grab a still frame (`/snapshot.jpg` already exists), click the target in a `<canvas>`, browser reads the pixel's HSV and POSTs a small range (hue ± tolerance, sat/val floors) the same way the Schedule card posts edits, persisted to a small JSON file the shader reads its thresholds from. Optional add-on: a "learn-target" skill that uses the existing `/llm/look` vision model to describe/estimate the color as a one-time setup convenience — must stay a one-shot setup action, never per-frame (defeats the point of a cheap GPU reflex). Open design question, not yet decided: single target color vs. a **named list** of targets (e.g. "ball" vs. "dock marker") so skills/schedule can reference one by name — cheap to support, same JSON file just as a list.

**Speed/feasibility (worked through with a yellow-ball example):** the GPU pipeline is NOT the bottleneck — per-frame GPU work + readback is single-digit ms against the camera's own ~66 ms frame period (15 fps), so analysis can run every frame with room to spare. The real speed ceiling is **the robot itself**: `slam_nav`'s 10 Hz control loop + the deliberately gentle `max_lin` default (0.15 m/s) put end-to-end reaction latency (ball moves → GPU detects → bearing published → motor commanded) around **100–200 ms**, and the robot physically cannot catch anything moving faster than its own ~0.15–0.4 m/s cap regardless of vision speed. Good fit: a slowly-rolled ball or a stationary marker (the dock-approach case). Not a fit: anything thrown or fast play. Also need a "lost track, spin to reacquire" fallback since a fast/erratic target can leave the C270's modest FOV between polls.

**2. Motion/frame-diff wake trigger — named "PIR" by the user (2026-07-10), for the functional resemblance to a passive-IR motion sensor.** Shader diffs current vs. previous camera frame, downsample-reduces to one "how much changed" scalar. Near-zero for a while = skip the autonomous "looking" beat's LLM call (nothing to comment on). A spike = something entered the frame → *that's* the trigger to actually wake up and narrate. Turns "looking" from blind-timer-driven into an actual presence/motion reflex, same shape as the existing pickup-detection reflex in `mood_node`.

**PIR is meant to run continuously** (always-on background reflex, not gated/throttled to only certain states) — user confirmed this explicitly. Needs a **trigger threshold** param (how big the aggregate change scalar must get to fire, separate from a small fixed per-pixel noise floor baked into the shader) — live-tunable via `/param` + a web UI slider, same shape as `stop_distance`/`lds_idle_timeout`. Should also decide a rate-limit/cooldown, though the existing beat system's `enrich_min_interval` may already cover it for free since PIR just feeds the "looking" beat's existing cadence.

**Continuous-operation cost — MEASURED ON HARDWARE 2026-07-11 (see the Phase-0 spike results below), superseding this 2026-07-10 estimate:**
- RAM: **actual +70 MB** for a live EGL/GLES2 context (12→82 MB RSS in the standalone spike), higher than the 15-45 MB estimate — BUT confirmed affordable: `nano-app`'s systemd `MemoryMax` is 450 MB and it was sitting at only 44-81 MB RSS at measurement time, so there's ~300+ MB of headroom even after adding this. Not a blocker, just a bigger number than guessed.
- CPU/GPU: **the realistic pipeline (5 downsample passes 640x480→...→1x1 + one tiny `glReadPixels`) measured 1.89 ms average** — confirms the original "single-digit ms" estimate once measured correctly. (A naive full-frame `glReadPixels` alone measured ~58 ms — that's an artifact of reading back a whole frame, which the real reduce-to-a-few-bytes design never does; don't be misled by that number if re-deriving this.)
- See `gpu-vision-phase0-verified` for the full writeup (extensions, exact numbers, procedure).

**PREFERRED capture design (decided 2026-07-10, supersedes plain-JPEG-decode as the default plan):** capture the camera as **raw YUYV at 640×480/30fps** (not MJPEG) for the vision path — bandwidth math checks out (640×480 YUYV @30fps ≈ 18.4 MB/s / ~147 Mbps, comfortably inside USB 2.0's ~280-320 Mbps real throughput; 720p YUYV would NOT fit, hence MJPEG is used at that resolution today). No JPEG anywhere in this path, so **no CPU decode step at all** — the shader does the trivial YUV→RGB conversion itself. Go further with **zero-copy DMA-buf import**: export the V4L2 YUYV capture buffer (`VIDIOC_EXPBUF`) and import it directly into a GLES texture via `EGL_EXT_image_dma_buf_import` + `GL_OES_EGL_image_external` (`samplerExternalOES` in GLSL) — camera driver to GPU texture with **zero CPU copy**, not just zero decode.

**Both former open items are now RESOLVED (2026-07-11, on real hardware — see `gpu-vision-phase0-verified`):**
1. **Concurrent MJPG+YUYV: confirmed NOT possible as two independent V4L2 sessions** — the C270 exposes exactly one true capture-capable V4L2 node (`/dev/video2`; `/dev/video3` is metadata-only, verified via a live ioctl probe). The camera-ownership architecture that resolves this (superseding an earlier "arbitration/pause" idea) is its own memory — see [[gpu-vision-camera-architecture]]: GPU vision becomes the sole continuous camera owner, the browser's live view is a downstream tee off the same frames (CPU JPEG-encode only while a viewer is connected), so neither consumer ever pauses.
2. **`lima` extension support: confirmed YES** — both `EGL_EXT_image_dma_buf_import` and `GL_OES_EGL_image_external` are present (spike script output on the live board). The zero-copy DMA-buf import path is viable; no `glTexImage2D`-upload fallback needed.

**Implementation stack (worked out 2026-07-10, nothing built yet):** ordinary embedded-Linux headless-GPU-rendering tech, not anything GPU-compute-specific (no Vulkan/OpenCL/CUDA-equivalent) — `lima` (kernel DRM driver, already present) + Mesa's `libEGL`/`libGLESv2` (userspace) → an **EGL context via the GBM platform** (`EGL_KHR_platform_gbm`, the standard way to render off-screen against `/dev/dri/renderD128` with no display attached) → **OpenGL ES 2.0**, where the actual logic is a **GLSL ES 1.00 fragment shader** (the threshold-compare for feature 1, the texel-subtract for PIR) run over a full-screen quad into an **FBO** (off-screen render target), chained through a few downsample passes, finished with one `glReadPixels` call. From Python: **moderngl** preferred over raw **PyOpenGL** (lower per-call overhead, better fit for this per-frame repeated pattern) to drive it from inside `app_hub`.

**Design constraints from the cost discussion (see this session for full reasoning):**
- Both must fold into an **existing hub** (`app_hub`, which already owns the camera via `web_control`) — a standalone new rclpy process would cost ~80-150 MB just to exist, dwarfing everything else. This is the single highest-leverage decision, learned the hard way already in this codebase (it's why `sensor_hub`/`app_hub` exist at all).
- Poll/analyze at 5-10 Hz, well below the camera's ~15 fps cap — neither feature needs full frame rate, and it bounds worst-case CPU (~5-15% of one core estimated, unmeasured).
- Biggest unmeasured unknown: the Mesa/lima GLES driver's RAM footprint once loaded into a process (textures themselves are trivial, a few hundred KB-2.4 MB). Recommended first step before building either feature: a small standalone GLES2 spike script on the board measuring actual RSS before/after context creation + one threshold pass, to replace the estimate with a real number.

**Phase 0 (hardware bring-up + verification) is DONE as of 2026-07-11** — see
`gpu-vision-phase0-verified` for the full record. Feature code itself (the `gpu_vision.py`
module, the camera-ownership change — see [[gpu-vision-camera-architecture]] — the
`chase-target.md` skill, the PIR wiring into `mood_node._camera_beats_ok()`) is still not
started — that's the next chunk of work, and it's a real feature build (new GL shader code,
camera architecture change, motion-adjacent skill file) worth confirming with the user before
diving in, not something to just start unprompted.

**3-6. BUILT 2026-07-11 (same day, follow-up session) — see [[gpu-vision-implemented]] for the
full writeup, code locations, and verification status (code-reviewed only, NOT yet live-tested —
robot went offline mid-session).** Original backlog description kept below for the design
rationale/consumer ideas, several of which (the caution-trait nudge, the `looking`-beat orient
consumer) are still NOT wired up — only the raw GPU signals + telemetry/UI surfacing are built:
- **Motion-saliency bounding center**: the exact same frame-diff shader as PIR (feature 2), but
  keep the bounding-box extent instead of collapsing to one scalar → `(target_x, target_y)`.
  Near-zero extra cost once PIR exists; reasonable to bundle into the same PR as PIR rather than
  build separately. Use: orient toward movement (a person/pet entering frame) before the
  `looking` beat snaps a photo, or before the LLM comments.
- **Optical virtual bumper**: global frame-to-frame delta (same diff shader again) correlated
  CPU-side against the currently *commanded* `/cmd_vel` — large delta while commanded-stopped =
  something moved through frame (expected, ignore); near-zero delta while commanded-moving =
  wheel slip/stall (the interesting case). The correlation logic is cheap CPU, not a new shader.
  Consumer: a caution-trait nudge via the same fast-rule mechanism as the pickup reflex
  (`Personality.tick_events` in `brain.py`), or a `slam_nav` reactive-stop input.
- **Kinetic intercept alert**: reuses feature 1's color-mask/blob machinery — track the mask's
  bounding-box area over a short (3-frame) history and flag rapid growth as "something's
  approaching the lens fast." Needs a small ring-buffer texture (trivial RAM). Complementary to
  lidar's distance-based stop, not redundant with it — lidar says "something is close," this
  says "something is closing fast," which lidar's snapshot-style range reads don't distinguish
  well frame-to-frame.
- **Flashlight/dark reflex**: global average luminance (one more reduction pass) → trigger the
  ESP32's existing `/led` topic via the `blink-led.md` skill pattern. Trivially cheap once any
  reduction-pass plumbing exists from features 1/2 — good candidate for "first extra thing to
  ship" after the core two.

**Explicitly considered and NOT added to the backlog (2026-07-11 analysis)**, with reasons, so a
future session doesn't have to re-litigate: LED beacon tracking / flicker-locked IR tag detection
(no beacon/IR marker hardware exists — would become a legitimate Tier-B item on top of feature 1
IF a beacon is ever added at the dock); dynamic cliff detector (needs the camera remounted
downward-facing, conflicts with its forward-facing role for `looking`/`/llm/look` — needs a
second camera, not just software); chromatic-aberration proximity ("visual focal-proximity
cushion" — not physically sound on a fixed consumer lens, don't prototype); visual hydrophone
floor classifier (the IMU accelerometer is a strictly better, more direct vibration sensor);
horizon drift index / 4-quadrant visual-odometry cross-check (IMU + wheel encoders already do
tilt/roll/odometry more reliably; no stated consumer for a noisy camera-derived cross-check);
dynamic vignette exposure bias (blocked on missing V4L2 exposure-control plumbing in
`mjpeg_camera.py`, not hardware — revisit if that plumbing ever gets built for other reasons).

**Hardware wishlist (not software-buildable today, but worth knowing about if a small BOM
addition is ever considered)**: a **structured-light depth plane** (a cheap $2-5 laser line
projector + isolating its Y-position across 3 forward windows) would give the robot actual
*depth* sensing, which nothing today provides (lidar is 2D-planar only) — this is the single
highest-value hardware addition on the original 21-item brainstorm list if ever pursued; the
matching **laser stripe gap/hole finder** (row-wise continuity check on the same laser line, for
trench/gap negative-obstacle detection) would come essentially free alongside it, same shader
infra. Also: the **AC-hum flicker analyzer** idea is physically real (rolling-shutter banding
under mains-frequency lighting is well documented) but should be narrowed if ever built — "flicker
detected → lock exposure/frame-rate to de-band the live view," not the original brainstorm's
"detect industrial space" framing, which overreaches what a bare flicker-frequency signal
supports.

## Cheap-tier brainstorm, 2026-07-12 (post-commit a881ddc/7060bde, nothing built yet)

Prompted by "what new functions would be cheap" — these all reuse the reduction-pass plumbing
(shader → `build_downsample_chain`/`run_downsample_chain` → tiny `glReadPixels`) that already
exists and costs ~1.9ms/pass measured on hardware, or cost literally **zero** extra GPU time by
combining signals `_loop()` already computes. Not yet approved/scoped — brainstorm only, same
status as the rest of this file until a user picks one to build.

**Reuses an existing pass (near-zero marginal GPU cost):**
- **Auto white-balance / colour-cast drift signal.** `_LUMA_FS` already reads
  `texture2D(tex,uv).rgb` before collapsing to one grayscale scalar via the luminance dot
  product — summing R/G/B separately instead (same packing trick `_DIFF_FS`/`_THRESHOLD_FS`
  already use: 3 values in one RGBA readout) gives an average-scene-colour signal for free off
  the pass that already runs unconditionally every frame. Use: detect when ambient light has
  shifted (evening incandescent vs. daytime) and auto-nudge blob-tracking's calibrated colour
  threshold, instead of a stale calibration silently degrading through the day.
- **Camera-obstruction / lens-covered flag.** Also off the always-on luma pass: near-zero
  variance (needs one more cheap stat, see "dynamic range" below) combined with very-dark or
  very-uniform average luma = "something is covering or has fogged the lens" (a hand, dust, the
  robot wedged face-first into something). Pure CPU threshold logic on numbers already being
  computed — zero new shader.
- **Second / named colour target.** Directly the open design question already on record above
  ("single target vs. a named list"). Running the threshold+reduce pass twice (or packing two
  hues into separate RGBA channels of one pass) is ~another 1.9ms — still trivial against the
  ~66ms frame budget. GPU cost is basically free; the real cost is the same as always, the UI/
  skill work to let something *name and pick* a target ("track the ball" vs. "track the dock
  marker").

**New shader, still cheap (one more reduction-chain instance):**
- **Visual "interest"/edge-density scalar.** A Sobel-ish (or even a cheap 4-tap gradient)
  fragment shader reduced the same way as PIR, giving "how much texture/contrast is in frame"
  as a static complement to PIR's "how much just *changed*." Use: let the `looking` beat also
  fire on "camera is pointed at something visually busy" (a cluttered shelf, a person standing
  still) not just on motion — PIR alone misses a stationary subject entirely.
- **Dynamic range / backlit detector.** The downsample chain's `_COPY_FS` passthrough relies on
  `GL_LINEAR` box-filter averaging for free box-blur; swapping in a MAX-blend variant for one
  chain (GLES2 doesn't do this via blend state the way desktop GL might, but a tiny fragment
  shader that samples the 4 texels a `GL_NEAREST` upstream stage would've blurred and takes
  their `max()` gets the same effect) gives frame max-luma; combined with the existing average
  from `_LUMA_FS`, `max - avg` is a crude "how backlit/blown-out is this scene" signal — the
  plain dark reflex (average brightness only) can't distinguish "dim room" from "bright window
  behind a dark subject," which matters because dark-reflex's LED response is the wrong fix for
  the second case (more light won't help a silhouette).

**Zero GPU cost — pure CPU correlation of signals already computed:**
- **Motion-matches-target correlation.** `motion_center` (PIR's weighted centroid) and `target`
  (the colour-blob centroid) are both already read every tick in `_loop()` — comparing their
  (x,y) distance costs nothing. Answers "is the thing that's moving actually my tracked ball,
  or is something else moving elsewhere in frame" — meaningfully sharper than either signal
  alone, and the intercept-rate/bumper logic could both benefit from knowing this.
- **Camera-freeze / stuck-capture diagnostic.** The PIR diff score already goes near-zero when
  nothing changes; correlating that against *commanded* `/cmd_vel` (already read for the
  optical bumper) the same way the bumper does, but framed as "the whole picture should be
  sliding past while driving, and it isn't" — a `mjpeg_camera`/V4L2-level hang (camera USB
  wedge, not a wheel stall) would look identical to the optical bumper's existing check but
  means something different downstream (recover the camera, not flag a stall). Might just be a
  second interpretation of the same number rather than a genuinely separate feature — worth
  deciding if it needs its own signal or just a second consumer of the existing one.

**Not cheap, listed for contrast (don't build under a "cheap" ask):** the AC-hum flicker
analyzer above needs multi-frame temporal analysis (not a single reduction pass), and per-target
shape/circularity discrimination (see the earlier answer in this session) needs a real
downscaled-mask readback (KBs, not bytes) — both are real, previously-considered ideas but a
different cost tier than everything in this section.
