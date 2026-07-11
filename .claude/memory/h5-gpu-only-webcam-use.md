---
name: h5-gpu-only-webcam-use
description: "H5 Mali-450 GPU is unused by the stack today; the only worthwhile use is webcam/vision processing, and it's now actively planned (not blacklisted)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0511c092-56af-4656-9941-b2e947bb7aaf
---

Nothing in the stack uses the H5's **Mali-450 MP4 GPU** today — camera is C270 hardware-MJPEG passthrough (no decode/encode), SLAM scan-match + occupancy are NumPy on CPU, everything else is rclpy. The only "gpu" reference is `sys_monitor` reading `gpu-thermal` for the dashboard.

Hard constraint: Mali-450 is **OpenGL ES 2.0 only** (Lima/Mesa) — **no OpenCL, no compute shaders**. GPGPU is only possible via fragment-shader-into-FBO tricks, and ES2 (no int ops/atomics) + readback latency kills most uses; NEON on the A53 usually wins for small data (e.g. ~360-point lidar scans → GPU scan-match not worth it).

**Decision (user agreed 2026-06-25):** the only real use for the GPU is **webcam/vision processing** — per-pixel, parallel, small output is exactly what it's for (colour-convert/downscale/undistort, threshold+blob for visual servoing toward a target/dock, line detection, motion diff).

**2026-07-10 update: the GPU is going to be used, so it stays on.** Earlier the same day a blacklist (`/etc/modprobe.d/blacklist-mali-gpu.conf`, `blacklist lima`) was added to `deploy/sbc-setup.sh` to power-gate the idle GPU — but the user then approved two concrete vision features to build (see [[gpu-vision-features-todo]]: a colour-threshold blob-tracking bearing + a motion-diff wake trigger), so the blacklist was **reverted the same session before ever being deployed** (`deploy/sbc-setup.sh` is back to its original 5-step form, no GPU-disabling code). The `lima` driver is untouched/available on the live board — no re-enable step needed later.

When building vision: start with one GLES2 fragment shader doing colour-threshold blob tracking on camera frames, publish a `/target` bearing for the ESP32 to chase. Fold the work into `app_hub` (which already owns the camera via `web_control`) rather than a new process — see [[gpu-vision-features-todo]] for the full cost/architecture reasoning. The bigger-ROI combined path (not part of the current todo) is Cedrus VPU + Mali (MJPEG-decode → shader → **H.264-encode**), which also cuts WiFi bandwidth ~5–10× vs MJPEG passthrough — but that leans on the VPU more than the GPU.

**2026-07-11: the GPU is now actually active, not just theoretically available.** `sudo modprobe lima` + the Mesa/EGL/GBM apt packages were installed and verified live (previously the `lima` module had never been loaded despite being in the kernel build — see [[gpu-vision-phase0-verified]] for the full bring-up + measured numbers). `/dev/dri/renderD128` now exists and persists across reboots via `/etc/modules-load.d/lima.conf` (also folded into `deploy/sbc-setup.sh`).
