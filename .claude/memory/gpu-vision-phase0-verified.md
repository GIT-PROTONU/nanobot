---
name: gpu-vision-phase0-verified
description: "Phase-0 GPU vision bring-up done + measured on the live robot 2026-07-11 — lima loaded, Mesa/EGL installed, real RSS/timing numbers, both zero-copy extensions confirmed present"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97322c83-aa6a-4fd1-89af-d8c3f90dd86f
---

Phase 0 of the GPU-vision backlog plan ([[gpu-vision-features-todo]], [[h5-gpu-only-webcam-use]])
was executed and verified live on the robot on 2026-07-11 (user supplied the sudo password
interactively for this session only — not stored anywhere). This replaces every "unmeasured"/
"unverified" caveat in the older memories with real numbers.

**What was done, in order:**
1. `sudo modprobe lima` — worked immediately; `/dev/dri/renderD128` appeared (group `render`,
   mode 0660). Confirmed the module was NOT auto-loading before this (this Armbian image doesn't
   auto-probe it despite `CONFIG_DRM_LIMA=m` + the enabled `gpu@1e80000` device-tree node).
2. `apt-get install -y --no-install-recommends libegl1 libgles2 libgbm1 libgl1-mesa-dri` — pulled
   `mesa-libgallium` + LLVM (`libllvm19`, needed by Mesa's Gallium infra even for the non-LLVM
   `lima` driver) + Vulkan loader as transitive deps. **Actual disk cost ~200 MB** (3.5→3.7 GB
   used out of 7 GB card), higher than the earlier ~40-50 MB estimate (which missed the LLVM
   pull-in) but still trivial against the 3.0 GB that remained free.
3. Persisted both: `/etc/modules-load.d/lima.conf` (contains `lima`) and confirmed `ibster` was
   already in the `render` group (Armbian default, not something sbc-setup.sh had done). Both
   folded into `deploy/sbc-setup.sh` as a new idempotent step `5/6` (renumbered from `X/5` to
   `X/6`), so a fresh reflash reproduces this automatically — no manual step needed again.
4. Wrote **`scripts/gpu_vision_spike.py`** (new, committed) — a zero-dependency raw-ctypes
   probe (same style as `mjpeg_camera.py`'s raw V4L2 ioctls, no `moderngl`/`PyOpenGL` needed even
   for this): creates a headless EGL context via Mesa's `EGL_PLATFORM=surfaceless` (no GBM device
   handle needed, no X/Wayland), queries extensions, and times both a naive full-frame readback
   and a realistic multi-stage downsample-reduce chain. Run via `python3 scripts/gpu_vision_spike.py`
   on the board.

**Results (the numbers that matter for the design decision):**
- `EGL_EXT_image_dma_buf_import`: **True**. `GL_OES_EGL_image_external`: **True**. Both required
  zero-copy-DMA-buf extensions are present on this `lima`/Mesa 25.0.7 build — the zero-copy
  capture path from the original plan is viable, no fallback to plain `glTexImage2D` upload
  needed. `GL_RENDERER=Mali450`, `GL_VERSION=OpenGL ES 2.0 Mesa 25.0.7-2` (confirms hardware
  path, not a software/llvmpipe fallback).
- **RAM: EGL context creation costs ~+57 MB RSS** (12.3→69.0 MB), settling at **~70 MB total**
  once a texture/FBO/shader program exist (82 MB). This is a real, fixed, one-time cost paid once
  the feature is active at all — doesn't scale with frame rate. Higher than the earlier 15-45 MB
  guess, but confirmed **affordable**: `app_hub`'s systemd `MemoryMax=450 MB`, measured
  `RSS≈81 MB` at the time (plenty of other headroom already in use for ROS/web/OLED/behavior), so
  there's ~300+ MB of margin even after adding this in-process.
- **Timing: DO NOT trust a naive "read back the whole frame" number.** A naive
  `glDrawArrays`+`glReadPixels(640,480)`+`glFinish` pass measured **~58 ms steady-state** — this
  looked alarming (exceeds the camera's own 66 ms/frame period at 15fps) until isolated: the draw
  call alone is 1.15 ms, a *tiny* 4x4 readback is 0.05 ms, and **the full realistic pipeline (a
  5-stage downsample chain 640x480→160x120→40x30→10x8→1x1 + one tiny final `glReadPixels`)
  measured 1.89 ms average (1.75-2.08 ms range)**. The ~58 ms number was purely an artifact of
  reading back a full-resolution frame, which the real design (reduce to a few bytes on-GPU, read
  back only that) never does. **This confirms the original "single-digit ms" estimate from the
  2026-07-10 planning session was correct — the earlier 2026-07-11 in-session scare number was a
  measurement-methodology mistake, caught and corrected within the same session.** Anyone
  re-deriving this: always benchmark the actual reduce-to-small-buffer design, never a full-frame
  readback, or you'll get a wildly pessimistic number.

**Camera capture-path decision made this session** (not just measured, actually decided): since
the C270 has exactly one true capture-capable V4L2 node (`/dev/video2`; `/dev/video3` is
metadata-only — confirmed via a live `VIDIOC_ENUM_FMT`/`ENUM_FRAMESIZES`/`ENUM_FRAMEINTERVALS`
probe, also YUYV 640x480 confirmed up to 30fps), a dedicated raw-YUYV vision capture cannot run
concurrently with the browser's MJPG `/stream.mjpg`. Resolved design: extend `CameraStream`
(`mjpeg_camera.py`) with vision-viewer arbitration — YUYV vision gets exclusive access only when
the MJPG (browser) viewer count is 0; a browser viewer connecting tears down vision's session and
reopens MJPG, vision resumes when the browser disconnects. Full detail in the plan document (see
below) and in `gpu-vision-features-todo`'s updated "resolved" section.

**Net verdict: Phase 0 is a clean GO.** All three of the plan's open unknowns (extension support,
RAM cost, per-frame timing) are now real measured numbers, not guesses, and none of them block
the feature. What's NOT done yet: the actual feature code (`gpu_vision.py`, the `CameraStream`
arbitration change, `chase-target.md` skill, the PIR→`mood_node._camera_beats_ok()` wiring) — see
[[gpu-vision-features-todo]] for that scope. The full implementation plan (file-level, with the
above numbers folded in) is saved at
`/home/ib/.claude/plans/well-plan-the-webcam-proud-rossum.md`.
