---
name: stack-autoheal
description: Stack autoheal = nano-heal.timer runs `stack.sh heal` every 20s to relaunch crashed nodes; catches crashes not hangs; gated on nano-stack.service
metadata:
  node_type: memory
  type: project
  originSessionId: ce9593fa-fa1a-494e-a534-d9fded658468
---

The stack had **no autoheal** before 2026-06-24: `nano-stack.service` is `Type=oneshot` +
`RemainAfterExit=yes` + `KillMode=process` with detached `setsid` children, so systemd never
tracked individual node PIDs and a crashed node stayed dead until a manual
`stack.sh up`/`restart` or reboot. (Only narrow self-heal: the zenoh router runs
`exit_on_failure:false`; and a wedged ESP link self-reboots — see [[esp32-zenoh-pico-integration]].)

**Added a liveness-timer autoheal** (user picked this over per-node `Restart=` units, to keep
the RAM-saving detached design):
- **`scripts/stack.sh heal`** — new subcommand = `do_up` with no trailing status/sleep.
  `do_up`'s `pgrep` guards make it a NO-OP when healthy (silent), relaunching only dead
  nodes. Also moved the zenohd router-config `python` generation INSIDE the "zenohd down"
  guard so a healthy heal tick spawns no python (cheap on the 1 GB H5).
- **`deploy/systemd/nano-heal.timer` + `nano-heal.service`** — timer fires `stack.sh heal`
  every 20s (`OnBootSec=90`, `AccuracySec=2s`). Both gated `Requisite=nano-stack.service`,
  so `systemctl stop nano-stack` also stops healing (no zombie relaunch).
- `deploy/sbc-setup.sh` installs all three units + `enable nano-heal.timer`.

**Catches crashes, NOT hangs**: a node alive-but-wedged keeps its PID so `pgrep` skips it.
A "liveness + data-flow check" (e.g. is `/scan` actually flowing) was the option NOT taken;
the `heal` subcommand is the hook to add it later. Especially relevant because sensor_hub
runs imu+sys+odom+lds in one executor — one crash takes all four down, now auto-revived
within ~20s.

**2026-07-04: found NOT installed on the live board, now FIXED** — `nano-heal.timer`/
`nano-heal.service` didn't exist (a killed map_bridge stayed dead >10 min; behavior/mood_node
was also found dead — that one was a startup crash, see [[behavior-layer-plan]]: mood_node
overwrote rclpy Node's private `_clock` with sismic's SimulatedClock, breaking create_timer;
renamed to `_chart_clock`). Installed both units from `deploy/systemd/` and enabled the timer
the same day (nano-stack.service was already present, so sbc-setup.sh had simply last run
before the heal units existed).
