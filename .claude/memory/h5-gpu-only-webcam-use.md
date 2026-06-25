---
name: h5-gpu-only-webcam-use
description: H5 Mali-450 GPU is idle in the stack; the only worthwhile future use is webcam/vision processing
metadata:
  type: project
---

Nothing in the stack uses the H5's **Mali-450 MP4 GPU** today — camera is C270 hardware-MJPEG passthrough (no decode/encode), SLAM scan-match + occupancy are NumPy on CPU, everything else is rclpy. The only "gpu" reference is `sys_monitor` reading `gpu-thermal` for the dashboard.

Hard constraint: Mali-450 is **OpenGL ES 2.0 only** (Lima/Mesa) — **no OpenCL, no compute shaders**. GPGPU is only possible via fragment-shader-into-FBO tricks, and ES2 (no int ops/atomics) + readback latency kills most uses; NEON on the A53 usually wins for small data (e.g. ~360-point lidar scans → GPU scan-match not worth it).

**Decision (user agreed 2026-06-25):** the only real use for the GPU is **webcam/vision processing** — per-pixel, parallel, small output is exactly what it's for (colour-convert/downscale/undistort, threshold+blob for visual servoing toward a target/dock, line detection, motion diff). Until an onboard-vision feature is actually wanted, **leave the GPU idle/power-gated** (better for thermals — see [[cooling-fan-control]]). When building vision: start with one GLES2 fragment shader doing colour-threshold blob tracking on camera frames, publish a `/target` bearing for the ESP32 to chase. The bigger-ROI combined path is Cedrus VPU + Mali (MJPEG-decode → shader → **H.264-encode**), which also cuts WiFi bandwidth ~5–10× vs MJPEG passthrough — but that leans on the VPU more than the GPU.
