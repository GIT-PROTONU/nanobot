---
name: lima-boot-load-bug
description: "OPEN BUG — the lima GPU kernel module doesn't reliably auto-load at boot despite correct /etc/modules-load.d/lima.conf; gpu_vision.py silently falls back to llvmpipe software rendering with no obvious error"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97322c83-aa6a-4fd1-89af-d8c3f90dd86f
---

Discovered 2026-07-11 during a GPU-vision CPU/RAM test session, after a genuine board reboot.

**Symptom**: after rebooting the robot, `/dev/dri` was entirely absent and `lsmod | grep lima`
showed nothing — despite `/etc/modules-load.d/lima.conf` being present and containing exactly
`lima` (confirmed via `cat`), which is supposed to make `systemd-modules-load.service` load it
automatically at boot. `systemctl status systemd-modules-load.service` reported
`Active: active (exited)` with `status=0/SUCCESS` — the service itself didn't fail, it just
silently didn't bind the GPU.

**Consequence — the dangerous part**: `gpu_vision.py`'s EGL context creation
(`EGL_PLATFORM=surfaceless`) does NOT error when there's no DRM render node — Mesa happily hands
back a context backed by `llvmpipe`, its universal software rasterizer, instead. The startup log
line (`gpu_vision: GL context up, renderer=...`) is the ONLY signal, and it's logged at plain
`INFO` level, easy to miss. Confirmed this actually happened: after this reboot the log read
`renderer=llvmpipe (LLVM 19.1.7, 128 bits)` instead of the expected `renderer=Mali450`. Running
GPU vision (a design specifically built to offload work FROM the weak Cortex-A53 CPU) on
`llvmpipe` means it's silently running back on the CPU — defeating the entire point, with no
crash, no error, nothing that would surface the problem short of manually reading the renderer
string in the log.

**Root cause (best guess, not confirmed with kernel-level tracing)**: `modprobe lima` run
manually AFTER boot completes works instantly (exit 0, `/dev/dri/renderD128` appears) — so the
module itself is fine, it's not a build/compatibility issue. This points to an **ordering
problem**: `systemd-modules-load.service` runs very early in the boot sequence (in/near
`sysinit.target`), likely before whatever the Mali GPU platform device depends on (the
display-engine or DRM core subsystem) has finished probing. `modprobe` succeeding just means the
`.ko` loaded into the kernel; it doesn't guarantee the driver successfully bound to the actual
hardware device at that early point if a dependency isn't ready yet.

**NOT fixed yet** — this was found during a testing session focused on the GPU vision feature
itself, and fixing systemd unit ordering was explicitly out of scope for that pass. Two candidate
fixes for whoever picks this up:
1. Give `systemd-modules-load.service` explicit ordering against whatever unit represents display
   engine / DRM readiness (`After=`), if such a unit/target exists on this Armbian image — may
   need a custom drop-in.
2. Switch to a udev-rule-triggered load instead of (or in addition to) `modules-load.d` — udev
   rules fire in response to actual device appearance, sidestepping the early-boot ordering
   problem entirely.
3. At minimum, regardless of the above: make `gpu_vision.py` log a LOUD warning (not just INFO)
   when `renderer` doesn't contain `"Mali"` — silent CPU fallback should never look identical to
   the intended GPU path in the logs.

**Workaround until fixed**: `sudo modprobe lima` manually after every reboot, before relying on
GPU vision being GPU-accelerated. **Always check `journalctl -u nano-app | grep renderer` after a
reboot** — `renderer=Mali450` = correct, `renderer=llvmpipe` = the bug hit again, GPU vision is
running on the CPU.

See [[gpu-vision-implemented]] and [[gpu-vision-phase0-verified]] for the broader GPU vision
feature this bug was found within.
