---
name: dev-host-is-ubuntu
description: "Dev host is now native Ubuntu 24.04, not Windows/WSL — repo's Windows docs are stale"
metadata: 
  node_type: memory
  type: project
  originSessionId: e378609d-bfa0-4599-9931-d35cfe36c672
---

The dev PC used to build/flash and deploy to the board is now **native Ubuntu 24.04**
(switched 2026-06-18), no longer Windows + WSL.

**Why it matters:** `README.md` and `CLAUDE.md` still describe a Windows workflow that
no longer applies — COM ports (`COM10`), WSL2 + usbipd-win to reach USB serial, and
PuTTY `plink`/`pscp` for deploying to the board. On Ubuntu the ESP32/IMU appear directly
at `/dev/ttyUSB*` and deploy can use plain `ssh`/`scp` instead of plink/pscp.

**How to apply:** ignore the WSL/usbipd and COM-port instructions; use `/dev/ttyUSB*`
directly (see [[esp32-flash-setup-ubuntu]]). `scripts/deploy.sh` is **Windows-only**
(hardcodes `plink.exe`/`pscp.exe`) — don't run it as-is on Ubuntu; drive the deploy with
plain `ssh`/`scp` instead. Board: **`ibster@192.168.178.141`** (the `NANO_HOST` default),
home `~/Nano`, build/run via `~/.pixi/bin/pixi run`. `ibster` has **password sudo** (no
NOPASSWD) — pipe with `echo <pw> | sudo -S …`.

`sshpass` is NOT installed; for non-interactive auth use OpenSSH's askpass (no install):
write the password to a `chmod 700` temp file, then
`SSH_ASKPASS=<file> SSH_ASKPASS_REQUIRE=force setsid -w ssh …` (delete the file after,
never commit it). Network commands need the Bash sandbox disabled.

Manual deploy recipe that worked (June 2026, LDS→ttyS2 change): `scp -r src scripts deploy
ibster@…:~/Nano/` → `pixi run colcon build --symlink-install --packages-select <pkgs>` →
edit `/boot/armbianEnv.txt` overlays + reboot for device-tree changes. rmw_zenoh gotcha:
a throwaway `pixi run ros2 topic list/hz` often shows an EMPTY graph (CLI doesn't attach to
the router) — verify topics another way (node logs, raw serial read) rather than trusting it.
