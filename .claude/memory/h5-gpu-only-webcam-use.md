---
name: h5-gpu-only-webcam-use
description: H5 Mali-450 GPU is unused by the stack and now actively blacklisted (RAM/power); the only worthwhile future use is webcam/vision processing
metadata: 
  node_type: memory
  type: project
  originSessionId: 0511c092-56af-4656-9941-b2e947bb7aaf
---

Nothing in the stack uses the H5's **Mali-450 MP4 GPU** today — camera is C270 hardware-MJPEG passthrough (no decode/encode), SLAM scan-match + occupancy are NumPy on CPU, everything else is rclpy. The only "gpu" reference is `sys_monitor` reading `gpu-thermal` for the dashboard.

Hard constraint: Mali-450 is **OpenGL ES 2.0 only** (Lima/Mesa) — **no OpenCL, no compute shaders**. GPGPU is only possible via fragment-shader-into-FBO tricks, and ES2 (no int ops/atomics) + readback latency kills most uses; NEON on the A53 usually wins for small data (e.g. ~360-point lidar scans → GPU scan-match not worth it).

**Decision (user agreed 2026-06-25):** the only real use for the GPU is **webcam/vision processing** — per-pixel, parallel, small output is exactly what it's for (colour-convert/downscale/undistort, threshold+blob for visual servoing toward a target/dock, line detection, motion diff). Until an onboard-vision feature is actually wanted, leave the GPU idle/power-gated (better for thermals — see [[cooling-fan-control]]).

**Implemented 2026-07-10:** `deploy/sbc-setup.sh` step 5/6 now actually enforces this — it installs `/etc/modprobe.d/blacklist-mali-gpu.conf` (`blacklist lima`, the mainline DRM driver for the Mali-400/450 "Utgard" family on this board's mainline-tracking kernel) and best-effort `modprobe -r lima`s the current session too. Idempotent, part of the normal setup/re-run flow. Does NOT affect HDMI/fbcon (separate sun4i-drm display-controller driver) — headless console/serial debugging still works. **To re-enable when building the vision feature:** `sudo rm /etc/modprobe.d/blacklist-mali-gpu.conf && sudo reboot` (or re-run sbc-setup.sh after removing that step). Not yet re-run on the live board (192.168.178.141) — see [[deployment-state]] for the re-run backlog.

When building vision: start with one GLES2 fragment shader doing colour-threshold blob tracking on camera frames, publish a `/target` bearing for the ESP32 to chase. The bigger-ROI combined path is Cedrus VPU + Mali (MJPEG-decode → shader → **H.264-encode**), which also cuts WiFi bandwidth ~5–10× vs MJPEG passthrough — but that leans on the VPU more than the GPU.
