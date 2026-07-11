---
name: gpu-vision-camera-architecture
description: "Camera-ownership design for GPU vision + the browser live view running simultaneously — GPU owns the camera continuously, browser view is a downstream JPEG-encoded tee, not a second V4L2 session"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97322c83-aa6a-4fd1-89af-d8c3f90dd86f
---

Resolves a design question raised 2026-07-11 in the same session as [[gpu-vision-phase0-verified]]:
how can the GPU vision pipeline (PIR / blob-tracking, see [[gpu-vision-features-todo]]) use the
camera stream continuously **and** the web UI keep a live copy, given the confirmed hardware
constraint that the C270 exposes exactly one true capture-capable V4L2 node (`/dev/video2`) —
i.e. only one exclusive streaming session can exist at a time.

**Superseded idea (do not build): "CameraStream arbitration" — vision pauses while a browser
viewer is connected.** This was the original answer worked out during Phase-0 planning
(`add_vision_viewer()`/`add_viewer()` priority, YUYV vision session only when MJPG viewer count
is 0). It works, but it means PIR — which is explicitly meant to run continuously, per the
original 2026-07-10 approval — actually stops the moment someone opens the web UI's camera view.
Superseded by the design below, which gives both consumers genuinely simultaneous live access.

**Chosen design: flip camera ownership. GPU vision is the sole, continuous, always-on camera
owner; the browser's live view is a downstream tee off the same captured frames, not a second
V4L2 session.**

1. `gpu_vision.py`'s background thread is the *only* thing that ever opens the physical camera,
   always in raw YUYV, using the already-verified zero-copy DMA-buf import path (`lima` confirmed
   to support both `EGL_EXT_image_dma_buf_import` + `GL_OES_EGL_image_external` — see
   `gpu-vision-phase0-verified`). This is genuinely zero CPU cost per frame for vision itself —
   no JPEG anywhere in this leg.
2. Every captured frame is imported into a GLES2 texture and run through whichever vision shaders
   are active (PIR diff, blob-tracking threshold, the Tier-B extensions) — unchanged from the
   core feature plan.
3. **New**: when ≥1 browser viewer is registered (same ref-counting idiom `CameraStream` already
   uses for `/stream.mjpg` today), the *same* per-frame pass also runs a cheap YUYV→RGB
   conversion shader (trivial — needed for nothing else, purely for this path) into one more FBO,
   then does a **full-frame `glReadPixels`** — this is the one place in the whole design where
   the expensive full-frame readback (measured ~58ms at 640x480 in the Phase-0 spike, see
   `gpu-vision-phase0-verified`) is actually paid, and it's correctly scoped: only while someone
   is actually watching, and it runs on `gpu_vision.py`'s own background thread — never on the
   ROS executor thread, so it can't stall telemetry/OLED/behavior regardless of its cost.
   (Downscaling before readback, e.g. to 320x240, would cut this roughly proportionally if 58ms
   proves too coarse a cadence in practice — not yet measured at reduced resolution.)
4. The RGB buffer is JPEG-encoded **CPU-side** via a small ctypes binding to `libturbojpeg0`
   (`tjCompress2` from libjpeg-turbo) — confirmed available in the Armbian trixie apt repo,
   **542 KB installed size**, not currently installed. This matches the codebase's established
   "raw ctypes over one focused native library" pattern (`mjpeg_camera.py`'s V4L2 ioctls,
   `scripts/gpu_vision_spike.py`'s EGL/GLES ctypes) rather than pulling in a heavier wheel like
   Pillow/OpenCV.
5. The resulting JPEG bytes feed the **existing** `/stream.mjpg` and `/snapshot.jpg` endpoints
   completely unchanged from the frontend's perspective — `CameraStream`'s viewer-facing API
   (`add_viewer`/`get_frame`/`remove_viewer`) keeps the same shape; only where the frames
   originate changes (from `gpu_vision.py`'s pipeline instead of a second, independent
   `MjpegCamera` MJPG session).

**A real, deliberate trade being made**: today's `/stream.mjpg` is a genuinely zero-cost hardware
MJPEG passthrough (the C270 does JPEG encoding in its own onboard ASIC — no CPU/GPU work at all).
This design trades that away in exchange for continuous, truly-uninterrupted vision — the browser
view now costs a CPU JPEG encode (via libturbojpeg, expected fast — single-digit ms typical for
this codec on an A53-class core, not yet measured on this board) *only while a viewer is
connected*, which matches the existing "closed page costs nothing" philosophy exactly, just
shifted from "zero cost period" to "zero cost when idle, small cost when watched."

**Gating / backward compatibility**: this entire flip only activates when `gpu_vision_enable:
true`. With it off (the default), `MjpegCamera`'s existing hardware-MJPEG capture path in
`mjpeg_camera.py` is completely untouched — zero risk to current behavior until the feature is
deliberately turned on.

**Investigated and ruled out — Cedrus hardware MJPEG decode (2026-07-11):** the H5's Cedrus VPU
(`/dev/video0`, driver `cedrus`, kernel module **built-in** — `CONFIG_VIDEO_SUNXI_CEDRUS=y`, not
`=m`, so unlike `lima` it's always active, no `modprobe` needed) was checked as a possible way to
unify capture around the camera's existing hardware MJPEG stream (decode via VPU instead of CPU,
feeding both the GPU vision path and the browser from ONE MJPG capture with no format-switching
at all — would have been simpler than the design above). **Confirmed NOT viable**: a live
`VIDIOC_ENUM_FMT` probe on both queues shows Cedrus is a decode-only M2M device supporting
`MG2S`/`S264`/`S265`/`VP8F` (MPEG-2/H.264/H.265/VP8 *parsed slice data* — the stateless V4L2 API,
requiring userspace-side bitstream parsing, not a simple "feed compressed bytes" API) on its
OUTPUT queue, producing `NV12`/`NV21`/`YU12`/`YV12` on CAPTURE. **No JPEG/MJPEG support at all,
in either direction.** This path is closed for good — don't re-investigate it in a future
session; if a bigger unification is ever wanted, the earlier `h5-gpu-only-webcam-use` note about
"Cedrus MJPEG-decode → shader → H.264-encode" for bandwidth reduction is *also* now known to be
wrong on the decode side (Cedrus can't decode the camera's MJPEG) — encode-only trust in Cedrus
would need re-deriving from H.264 encode capability, which mainline Allwinner VPU support
generally does not have (Cedrus is decode-only in mainline Linux).
