---
name: stack-autoheal
description: RETIRED 2026-07-06 — heal-timer supervision replaced by per-unit systemd (nano-robot.target, Restart=on-failure); this file keeps the old failure history
metadata:
  node_type: memory
  type: project
  originSessionId: ce9593fa-fa1a-494e-a534-d9fded658468
---

**RETIRED 2026-07-06.** The whole heal-timer design (nano-stack.service oneshot + detached
setsid nodes + nano-heal.timer pgrep polling) was replaced by **per-process systemd units
under `nano-robot.target`** with `Restart=on-failure` — see [[architecture-two-planes-three-hubs]].
That removes by construction the failure modes this memory documented:

- the **heal-vs-restart race** (2026-07-05: heal tick's pgrep guard fired inside restart's
  down→up window → duplicate mood_node, both PPID 1);
- the **KillMode cgroup reap** (2026-07-04: heal-relaunched nodes SIGKILLed ~4 s in, zero
  log output, because the oneshot's cgroup was reaped — autoheal had never actually revived
  anything until `KillMode=process` was added);
- "heal not installed on the live board" drift (2026-07-04).

Still true and worth keeping: **`Restart=on-failure` catches crashes, NOT hangs.** A node
alive-but-wedged keeps running; a data-flow liveness check (e.g. is `/scan` flowing) remains
the not-yet-built option. Especially relevant because the hubs concentrate failure domains:
sensor_hub = imu+sys+odom+lds, app_hub = web+oled+behavior — one crash takes the whole hub
down, but systemd restarts the whole hub in seconds.

Diagnostic patterns that stay useful:
- a node that runs fine in a foreground shell but dies when service-launched → the killer is
  environmental (cgroup/env), not the code;
- `pgrep -fc` run via `ssh nano "bash -c ..."` counts the remote shell itself (its cmdline
  contains the pattern) — use `pgrep -a` and read the list;
- now first stop: `systemctl status nano-*` + `journalctl -u nano-app` (etc.).
