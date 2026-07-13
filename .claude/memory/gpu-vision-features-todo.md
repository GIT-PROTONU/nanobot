---
name: gpu-vision-features-todo
description: GPU (Mali-450/GLES2) vision backlog/design history — NOW ESSENTIALLY CLEARED. 2026-07-13 batch built EVERYTHING remaining except docking aids + cliff detection (user-excluded) — named targets, clutter velocity-throttle (via caution clamp), anticipatory greeting, ambient colour mood, visual diary, novelty score, vibration + camera-freeze diagnostics, glare rejection, OLED mask mirror (see gpu-vision-implemented for status: code-complete, not hardware-verified). Still open — overhead-clearance camera-mount geometry check (needs hardware). Kept for design rationale + rejected/hardware-blocked ideas
metadata: 
  node_type: memory
  type: project
  originSessionId: 0511c092-56af-4656-9941-b2e947bb7aaf
---

**STATUS 2026-07-13: this backlog is essentially cleared.** The user asked to "do all except the
docking and cliff detection" and everything below that was still open got BUILT in that one
dev-host session (named colour targets, clutter velocity-throttle via the caution clamp,
anticipatory greeting, ambient colour mood, visual diary, novelty score, vibration +
camera-freeze diagnostics, glare rejection, OLED mask mirroring) — see the 2026-07-13 section of
[[gpu-vision-implemented]] for what/where/verification status (code-complete + unit/smoke/GL
tested; NOT hardware-verified, NOT committed, NOT deployed). Still genuinely open: the
overhead-clearance camera-mount geometry check (needs the physical robot); permanently excluded:
docking aids + cliff detection. Everything below is kept as design rationale/history.

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
- **Auto white-balance / colour-cast drift signal — BUILT 2026-07-12** (see
  [[gpu-vision-implemented]], code written/not yet hardware-verified). Built as a NEW reduction
  chain on `cur_tex` directly (reusing `copy_prog`/`_COPY_FS` verbatim) rather than modifying
  `_LUMA_FS` in place — kept the existing dark-reflex luma calc byte-for-byte unchanged instead of
  risking it. `GpuVision.color_cast` property, surfaced in telemetry + the Sensors card. The
  auto-nudge-blob-threshold consumer described below is NOT built — only the raw signal exists.
- **Camera-obstruction / lens-covered flag — BUILT 2026-07-12** (see [[gpu-vision-implemented]],
  code written/not yet hardware-verified). `GpuVision.luma_variance`/`.obstructed`, exactly the
  zero-new-shader design described here (variance over the already-read-back luma buffer). Flags
  `variance < 15.0 AND luma < 0.15` — both thresholds are initial guesses, explicitly flagged as
  not yet hardware-tuned.
- **Second / named colour target.** Directly the open design question already on record above
  ("single target vs. a named list"). Running the threshold+reduce pass twice (or packing two
  hues into separate RGBA channels of one pass) is ~another 1.9ms — still trivial against the
  ~66ms frame budget. GPU cost is basically free; the real cost is the same as always, the UI/
  skill work to let something *name and pick* a target ("track the ball" vs. "track the dock
  marker").

**New shader, still cheap (one more reduction-chain instance):**
- **Visual "interest"/edge-density scalar — BUILT 2026-07-12** (see [[gpu-vision-implemented]],
  code written/not yet hardware-verified). Built as `GpuVision.edge_density` via a NEW `_EDGE_FS`
  shader — a cheap 3-tap gradient (centre+right+down neighbour), not a full 8-tap Sobel, still
  matches the "even a cheap 4-tap gradient" framing here. The `looking`-beat consumer described
  below is NOT built — only the raw signal + a live-tunable `clutter` alert exist so far.
- **Dynamic range / backlit detector — BUILT 2026-07-12** (see [[gpu-vision-implemented]], code
  written/not yet hardware-verified), and CHEAPER than this entry assumed: no MAX-blend shader
  needed. The existing luma downsample chain already reads back a small MULTI-CELL buffer (the
  chain stops at `min_size=24`, not 1×1) for the dark reflex — tracking a max alongside the
  existing sum in that already-read-back buffer was genuinely free. `GpuVision.luma_max`, alert
  = `(luma_max - luma) > threshold AND luma < 0.5` (both live-tunable), same "distinguish dim room
  from backlit silhouette" reasoning as originally proposed here.

**Zero GPU cost — pure CPU correlation of signals already computed:**
- **Motion-matches-target correlation — BUILT 2026-07-12** (see [[gpu-vision-implemented]], code
  written/not yet hardware-verified). `GpuVision.motion_target_match` — exactly as described here,
  a plain distance between the already-computed centroids. Surfaced with a live-tunable
  "motion_matches_target" alert (distance below threshold = match). The intercept-rate/bumper
  cross-consumer idea is NOT wired up — only the raw signal + alert exist.
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

## Reworked safety-adjacent trio, 2026-07-12 (user-pitched, reworked after cost/architecture critique)

User pitched three ideas framed as safety features; reworked below to correct overstated claims
(monocular RGB isn't depth, the vision loop isn't faster than slam_nav's 10Hz loop, a global
scalar isn't a costmap) while keeping the genuinely useful core of each. None built.

- **Overhead-clearance signal** (was "under-furniture wedge sentinel") — **the raw signal is
  BUILT 2026-07-12** as `GpuVision.overhead_edge_density` (see [[gpu-vision-implemented]], code
  written/not yet hardware-verified): a second new tiny shader (`_CROP_TOP_FS`) crops the
  edge-density shader's output to the top 30% of frame before reducing, plus a live-tunable
  `overhead_alert` threshold. Real gap this addresses: lidar is strictly 2D-planar, so an
  overhang entirely above the scan plane (couch arm, bed skirt, chair armrest) is invisible to
  it, and the lidar tower sticks up past the scan plane — the robot can fit its body under
  something while shearing the tower off on the overhang. **Still NOT done: the prerequisite
  camera-mount-geometry check** (is the top 30% of frame actually where the tower's collision
  zone projects to, given this camera's real height/tilt vs. the lidar tower's height) — this
  was built as a heuristic signal on spec, not verified against the real mount geometry, and
  the "trend while driving forward" / caution-trait consumer described below is also not wired
  up, only the raw scalar + a generic alert threshold exist.
- **Looming corroboration signal** (was "global looming brake") — **the raw signal is BUILT
  2026-07-12** as `GpuVision.motion_intercept_rate` (see [[gpu-vision-implemented]], code
  written/not yet hardware-verified): same ring-buffer growth-rate trick as the already-built
  colour-based kinetic intercept alert, applied to the raw PIR motion score instead of edge-
  density (edge-density itself is still unbuilt — this used the simpler, already-existing motion
  score instead, which covers the same "is anything looming" question without needing a new
  shader at all). Catches a pet/kid running at the robot, a door swinging open, etc. — anything
  looming, not just the calibrated tracked colour. **Correction from the original pitch:** drop
  the "bypasses the 10Hz slam_nav loop" framing — the vision pipeline is designed to poll at
  5-10Hz, the same order as slam_nav, not faster, and monocular RGB motion-growth is a noisier
  looming cue than lidar range for the glass/thin-wire cases cited (both sensors are weak there,
  not just lidar). **Still NOT built**: routing this into the caution-trait fast-rule path
  (`Personality.tick_events`) — the signal exists and is surfaced in telemetry/UI, but nothing
  consumes it autonomously yet, same as every other Tier-B signal.
- **Clutter-aware velocity throttle** (was "dynamic clutter costmap penalty") — **the raw signal
  + alert are BUILT 2026-07-12** as `GpuVision.edge_density` + the live-tunable `clutter` alert
  (see the edge-density entry above, and [[gpu-vision-implemented]]). Real gap: lidar only sees
  obstacles at its own mounted height, so cables, a shoe, or a dropped toy read as clear floor
  even though they can snag a caster or tangle a wheel — a global clutter score catches that.
  **Still NOT built: the actual velocity-throttle ACTION** (writing `max_lin` via the
  `caution`→`max_lin` clamp mechanism) — deliberately left informational-only like everything
  else in this batch, since wiring an actual speed change needs driving to verify safely, which
  was explicitly out of scope for this pass. **Correction from the original pitch:** this is one
  global scalar per frame, not a spatial map — real per-cell costmap inflation needs floor-plane
  camera calibration, a separate, non-cheap project; framed as a coarse global throttle it's cheap
  and buildable now, framed as "costmap penalty" it overpromises.

**Combined cost, all three:** at most 2 new reduction-chain instances (top-band edge-density +
whole-frame edge-density, shareable between the looming signal and the clutter throttle) — an
estimated +2-4ms of GPU time on top of the existing ~1.9ms PIR/diff chain, trivial against the
66ms frame period, with the analysis loop still only running at 5-10Hz. Zero new RAM (same EGL
context/FBOs). Only real prerequisite work: the camera-geometry check for the overhead-clearance
signal, and wiring the other two through the existing caution-trait clamp rather than a new
motor-authority path.

## General brainstorm — personality/diagnostic/docking, 2026-07-12

Prompted by "what other GPU features would be great and innovative" — broader than the cheap-tier
safety brainstorm above, covering personality/expression hooks, self-diagnostics, and docking
aids. Same status as the rest of this file: brainstorm only, nothing built/approved.

**Personality/expression hooks (turns raw signals into character, not just safety):**
- **Anticipatory-approach greeting.** The motion-saliency bounding box (already built in the 3-6
  batch) growing rapidly + centered = someone walking up — trigger the `greeting` reflex before
  pickup, not just on contact. Cost: ~zero, pure CPU correlation of an existing signal.
- **Ambient colour mood → face accent.** The colour-cast scalar already brainstormed (sum R/G/B
  off the luma pass) could bias which face/mood the `feeling` state leans toward (warm evening
  light vs. cold daytime fluorescents) — a cheap "the room's vibe rubs off on me" touch. Cost:
  zero marginal, same pass, new consumer.
- **Visual diary / continuity for reflection.** Log the already-computed per-frame scalars (luma,
  variance, motion) over time, same mechanism as `trait_trend_text` — gives the reflection prompt
  real sensory continuity ("the room got darker and quieter through the evening"). Cost: zero
  GPU, host-side logging only.
- **Novelty/boredom score for curiosity.** Maintain one small persistent low-res texture (e.g.
  16×16) as a slow exponential-moving-average "background" of the room, diff the current frame
  against *that* instead of just the previous frame (PIR already does previous-frame diff) — a
  sustained delta vs. the long-run background is a much better novelty signal for the curiosity
  trait / `looking` beat priority than raw motion. Cost: ~2 extra passes (blend-into-background +
  diff), still sub-2ms each.

**Self-diagnostic / maintenance:**
- **Shiny-floor / wet-spot detector — BUILT 2026-07-12** (see [[gpu-vision-implemented]], code
  written/not yet hardware-verified). `GpuVision.highlight_fraction`, exactly as described here —
  reuses the already-compiled threshold shader/program with a FIXED near-white target in a
  SEPARATE FBO/chain from the user's calibrated blob target, so the two can never collide. A
  live-tunable `shiny` alert exists; the "throttle speed" consumer is NOT wired up, informational
  only so far like everything else in this batch.
- **Vibration/looseness diagnostic.** Correlate the edge-density scalar against commanded speed —
  an image blurrier than the current drive speed should produce indicates excess chassis
  vibration (loose screw, wheel imbalance, worn caster), a maintenance flag not a stop. Cost:
  zero marginal if edge-density exists, pure CPU correlation.
- **Camera-freeze diagnostic** (already on the backlog above) fits the same cluster.

**Docking / navigation aids:**
- **Charge-LED confirmation.** If the dock has a status LED, crop to a small fixed ROI once
  roughly docked and sample average luma over time to detect a blink pattern — cheap vision-side
  corroboration of "am I actually charging." Cost: trivial, a handful of pixel reads, no
  reduction chain needed. **Caveat: speculative** — depends on the dock having a visible LED the
  camera can see while docked, unverified.
- **Doorway/open-space bias for exploration.** A vertical-uniformity heuristic (open space tends
  to look flat/uniform vs. cluttered) could bias autonomous wandering toward doorways/open rooms.
  **Caveat:** only useful if the robot does open-ended roaming rather than fixed pursuits —
  unverified whether that behavior exists today.

**Safety-adjacent, framed honestly:**
- **Reflection/glare rejection for blob tracking.** A hard specular highlight can spoof the
  hue-threshold tracker (a shiny reflection matching the tracked hue). The signal this would
  consume (`highlight_fraction`) is now BUILT (see the shiny-floor entry above), but the
  actual suppression/derating of blob-tracking confidence is NOT wired up — deliberately not
  done yet since it would change existing blob-tracking behaviour, a bigger step than adding an
  informational signal.

**Overall cost picture:** almost everything above reuses a pass already planned elsewhere in this
file (colour-cast, edge-density, threshold, motion bounding box) or costs literally nothing extra
(CPU correlation of numbers already computed each tick). The only two with real new GPU work are
the shiny-floor threshold pass and the novelty background-blend, both still sub-2ms.

## Distance-estimation pair, 2026-07-12 (user-pitched, reworked)

User pitched motion parallax and focus-blur as distance proxies. Both reworked below — the
literal formulas as pitched don't hold, but a corrected version of each survives and is cheap.

- **Motion parallax / "vertical descent" distance.** As pitched (`d ≈ 1/Δy_pixel` on an arbitrary
  tracked edge) conflates two different real techniques and the formula itself isn't right for
  either: (1) **inverse perspective mapping** — if a point is known to sit on the floor, its image
  row maps to a real-world distance from ONE frame + a one-time camera height/tilt calibration, no
  motion needed at all (cheaper than parallax, but only valid for floor-touching objects — useless
  for the overhang/glass cases this backlog cares about); (2) **time-to-contact ("tau") via optical
  expansion** — `τ ≈ size / (rate of size growth)`, scale- and distance-invariant, which is what
  the pitch is actually reaching for. **Tau is already on this backlog** as the "kinetic intercept
  alert" and the reworked "looming corroboration signal" above (bounding-box area growth over a
  short frame history) — this pitch should be merged into those, not built as a third variant, and
  formalized with the correct tau math rather than ad hoc `Δy`.
  **Feasibility boundary:** tracking an *arbitrary* edge needs real per-pixel optical flow /
  frame-to-frame correspondence — a genuine vision workload that doesn't fit the cheap
  global-reduction-pass toolkit this whole backlog runs on. Restricted to a feature ALREADY
  tracked with a returned centroid/box (the colour blob, the motion-saliency bounding box — both
  built/planned) it's free: the `(x,y)`/box history is already read back every tick, the only
  missing piece is a few lines of host-side tau math.
- **Focus-blur near-proximity proxy.** Reuses the edge-density shader (free marginal cost once
  built) as a sharpness scalar. **Correction:** the C270 is fixed-focus and consumer webcam
  fixed-focus lenses are deliberately set near the hyperfocal distance to maximize depth of field
  — so the "far away = blurry too" half of the pitch is likely too weak to detect on this class of
  lens; drop it. The **near/macro end survives** — genuine defocus blur does spike within a few
  cm-tens of cm (the lens's minimum focus distance), which is conveniently exactly the "about to
  touch a glass pane" case wanted. **Confound to guard against:** near-zero edge-density is
  indistinguishable from "looking at a blank wall/foggy window" — texture-free scenes read
  identically to genuine defocus. Don't trust the scalar alone; fuse with lidar-no-return + high
  average luma (not a dark void) + the already-backlogged wheel-stall-vs-commanded-motion check
  (robot commanded to move but not actually moving) for a real "touching something transparent"
  signal.

## Nice-to-have: mirror the tracking mask to the OLED, 2026-07-12

Prompted by "would it take a lot of CPU to draw the mask to the OLED" — answer: the OLED draw
itself is free (the `np.packbits` fast-display path already flushes full 128×64 frames at up to
20Hz for face animation, ~0.6ms CPU, ~79% of flush wall-time is I2C bus wait not CPU — see
`oled-display-perf`); the real new work is upstream, getting an actual mask image off the GPU
instead of the current design's few-byte scalar-only readback.

**How, cheaply:** the existing colour-threshold/PIR downsample chain already box-filter-halves
the frame in passes (~1.9ms full chain to 1×1, hardware-measured) — branch/stop that chain early
and read back an intermediate stage instead of continuing to 1×1. **Can't land on the OLED's
exact 128×64 in one resize pass from the 640×480 source** — GLES `GL_LINEAR` minification only
blends a 2×2 texel neighbourhood regardless of scale ratio, so a single ~5×/7.5× jump would alias;
that's precisely why the existing chain halves repeatedly instead of resizing once. Correct
shape: ride the existing chain down to something close (640×480 → 320×240 → 160×120), then one
more modest-ratio pass (160×120 → 128×64, ~1.25×/1.875×) to land exactly on the panel's
resolution — one extra pass, still sub-2ms, same shader machinery already built. Box-filtering a
binary mask through those passes turns it greyscale ("coverage fraction" per enlarged texel), so
re-threshold back to true black/white right before `np.packbits` — one more cheap GPU pass or a
trivial CPU-side numpy threshold.

**Cost summary:** readback grows from a few bytes to ~8-32KB (128×64, a handful of bytes/pixel),
still sub-ms on this hardware; one extra reduction-chain pass beyond what's already planned for
blob tracking. Genuinely cheap, just not the literal zero-new-code of the scalar-only design.

**Not just a CPU question — an ownership one:** the OLED is a single shared panel (face/
dashboard/karaoke words) and `mood_node` already has an arbitration pattern for exactly this
("stand down when another owner uses the panel" — TTS/manual override/pickup). Showing the mask
should fit that same model — e.g. only during an active chase-target skill, standing the face
down for that window — rather than fighting the face for the panel simultaneously.
