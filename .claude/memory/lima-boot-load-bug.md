---
name: lima-boot-load-bug
description: "RESOLVED 2026-07-11 — lima GPU kernel module wasn't reliably loaded at boot; fixed via ExecStartPre=-+/usr/sbin/modprobe lima in nano-app.service (verified across a real reboot)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 97322c83-aa6a-4fd1-89af-d8c3f90dd86f
---

Discovered AND fixed 2026-07-11, same day, across two sessions (found during a GPU-vision
CPU/RAM test, fixed + verified in the immediate follow-up).

**Symptom**: after rebooting the robot, `/dev/dri` was entirely absent and `lsmod | grep lima`
showed nothing — despite `/etc/modules-load.d/lima.conf` being present and correct (which is
supposed to make `systemd-modules-load.service` load it automatically at boot).
`gpu_vision.py`'s EGL context creation (`EGL_PLATFORM=surfaceless`) does **not** error when
there's no DRM render node — Mesa silently falls back to `llvmpipe` (software rendering)
instead, so "GPU" vision was silently running on the CPU with the only symptom being a
`renderer=llvmpipe` string in an INFO-level log line. See [[gpu-vision-implemented]] for how
this was first caught.

**Fix, in two parts:**

1. **`gpu_vision.py` now logs a loud, impossible-to-miss warning** whenever the renderer string
   doesn't contain `"mali"` (case-insensitive) — no longer just an easy-to-miss INFO line. Check
   `journalctl -u nano-app | grep renderer` after any reboot as a sanity habit regardless.
2. **`deploy/systemd/nano-app.service` retries the module load right before app_hub actually
   needs it**: `ExecStartPre=-+/usr/sbin/modprobe lima`. Both prefix characters matter and were
   each individually verified necessary by testing on hardware:
   - `+` — runs *this one command* as root, ignoring the unit's `User=ibster`. **This was the
     real, previously-undiagnosed root cause of the fix's first attempt failing**: loading a
     kernel module needs `CAP_SYS_MODULE`, which the unprivileged `ibster` user doesn't have.
     Without `+`, `modprobe` failed with `could not insert 'lima': Operation not permitted` —
     confirmed by testing (stop nano-app, `rmmod lima`, restart nano-app, watched the journal).
   - `-` — non-fatal: if `lima` genuinely can't load (hardware missing, driver removed), app_hub
     still starts and `gpu_vision.py` degrades gracefully (falls back, now warns loudly) rather
     than the whole hub failing to start over an optional feature.
   - The original theory (a pure early-boot *ordering* race in `systemd-modules-load.service`,
     which runs very early, before whatever the Mali platform device depends on is ready) may
     still be part of why `modules-load.d` itself is unreliable — that mechanism is left in place
     unchanged (it's harmless and occasionally may just work) — but the ACTUAL fix that matters is
     the privileged retry late in boot, at the one point that actually needs the module.

**Verified fixed across a genuine reboot** (not just a warm service restart — the real test for a
boot-ordering-shaped bug): rebooted the robot (`sudo systemctl reboot`), waited for it to come
back (`uptime` confirmed `up 0 min`), and the very first `nano-app` boot log showed
`renderer=Mali450` with no warning — `lima` loaded, `/dev/dri/renderD128` present, zero
`modprobe` errors in the boot journal, `nano-app` healthy (RSS ~155MB, matching the established
GPU-vision-idle baseline exactly). Also separately verified the mechanism itself (not just the
end-to-end reboot) via a controlled test: `systemctl stop nano-app` → `rmmod lima` → confirmed
unloaded → `systemctl start nano-app` → confirmed `ExecStartPre` reloaded it before `gpu_vision.py`
opened its EGL context.

**Deploy note**: this only fixed the LIVE robot's installed unit file (`/etc/systemd/system/
nano-app.service`, hand-installed + `daemon-reload`d during the fix) and `deploy/systemd/
nano-app.service` + `deploy/sbc-setup.sh`'s comment in the repo. Like the rest of this session's
GPU vision work, **none of this is committed to git yet** — a future `deploy/sbc-setup.sh` re-run
(which reinstalls all systemd units from `deploy/systemd/`) will correctly pick up the fix once
the repo changes are committed and deployed.
