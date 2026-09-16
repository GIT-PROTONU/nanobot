#!/usr/bin/env bash
# Nano robot runtime stack manager — now a thin wrapper over systemd.
#
#   bash scripts/stack.sh {up|down|restart|status}     (no pixi env needed)
#
# The stack runs as seven systemd units (installed by deploy/sbc-setup.sh):
#   nano-router    serial-capable zenohd (rmw_zenoh graph + the ESP32 UART link)
#   nano-app       app_hub: web_control + oled_display + behavior in ONE process
#   nano-sensors   sensor_hub: imu + sys_monitor + wheel_odometry + lds in ONE process
#   nano-nav       ONE rclcpp component container (Nav2 servers + their
#                  lifecycle manager); components attached by the loader unit
#                  below
#   nano-tf        static base_link->laser TF (own unit — a never-exiting
#                  ExecStartPost on nano-nav would hold that unit in
#                  "activating" forever and block the whole target start)
#   nano-slam      slam_toolbox 2.6.10 (plain node, self-configuring; /map +
#                  map->odom TF)
#   nano-nav-loader oneshot: `nav2.launch.py load_only:=true` against nano-nav
#                  (the old nano-ekf/nano-map units died with the slam_nav
#                  migration — see docs/nav2-migration.md)
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
UNITS=(nano-router nano-app nano-sensors nano-nav nano-tf nano-slam nano-nav-loader)
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
  # One systemctl call for all seven units (it prints one line per unit, in order)
  # instead of seven separate subprocess forks — status() runs after every up/down/restart.
  local i=0 st line
  while read -r st; do
    printf '  %s: %s\n' "${UNITS[$i]}" "$([ "$st" = "active" ] && echo UP || echo down)"
    i=$((i + 1))
  done < <("$SYSTEMCTL" is-active "${UNITS[@]}" 2>/dev/null)
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
