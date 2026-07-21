#!/usr/bin/env bash
# Nano robot runtime stack manager — now a thin wrapper over systemd.
#
#   bash scripts/stack.sh {up|down|restart|status}     (no pixi env needed)
#
# The stack runs as six systemd units (installed by deploy/sbc-setup.sh):
#   nano-router    serial-capable zenohd (rmw_zenoh graph + the ESP32 UART link)
#   nano-app       app_hub: web_control + oled_display + behavior in ONE process
#   nano-sensors   sensor_hub: imu + sys_monitor + wheel_odometry + lds in ONE process
#   nano-ekf       robot_localization EKF: /odom + /imu/data -> /odometry/filtered
#   nano-nav       slam_nav
#   nano-map       map_bridge_node (/dev/shm map blob -> /map for remote RViz)
# grouped under nano-robot.target. What each unit execs lives in ONE place:
# scripts/unit_exec.sh (env activation + the installed-executable command table).
#
# systemd replaced the old hand-rolled pgrep supervision AND nano-heal.timer:
# ordering is After=nano-router.service (the rmw_zenoh island gotcha), crash
# recovery is Restart=on-failure (no heal-vs-restart duplicate-node race), and
# stop/kill/verify is systemd's. Logs: journalctl -u nano-app (etc.) — the old
# .run/*.log files are no more, except the router config still generated there.
#
# The scoped NOPASSWD sudoers rules for exactly these systemctl verbs are installed
# by deploy/sbc-setup.sh (deploy/sudoers/nano-power).
set -u

TARGET="nano-robot.target"
UNITS=(nano-router nano-app nano-sensors nano-ekf nano-nav nano-map)
SYSTEMCTL="/usr/bin/systemctl"

installed() { "$SYSTEMCTL" list-unit-files "$TARGET" --no-legend 2>/dev/null | grep -q nano-robot; }

need_units() {
  installed && return 0
  echo "nano-robot.target is not installed. Run once:  sudo bash deploy/sbc-setup.sh" >&2
  exit 1
}

ctl() {  # ctl <verb> — root runs it directly; the stack user goes through sudo -n
  if [ "$(id -u)" -eq 0 ]; then "$SYSTEMCTL" "$1" "$TARGET"
  else
    sudo -n "$SYSTEMCTL" "$1" "$TARGET" || {
      echo "sudo denied — re-run deploy/sbc-setup.sh to install the nano-power sudoers rules" >&2
      exit 1
    }
  fi
}

status() {
  for u in "${UNITS[@]}"; do
    if [ "$("$SYSTEMCTL" is-active "$u" 2>/dev/null)" = "active" ]; then
      echo "  $u: UP"
    else
      echo "  $u: down"
    fi
  done
}

case "${1:-status}" in
  up)      need_units; echo "stack up…";      ctl start;   status ;;
  down)    need_units; echo "stack down…";    ctl stop;    status ;;
  restart) need_units; echo "stack restart…"; ctl restart; sleep 2; status ;;
  status)  status ;;
  heal)    ;;  # retired: systemd Restart=on-failure does this natively. No-op so a
               # stale nano-heal.timer tick during an upgrade window can't error.
  *) echo "usage: $0 {up|down|restart|status}"; exit 2 ;;
esac
