#!/usr/bin/env bash
# Nano robot runtime stack manager. Run ON THE BOARD via:
#   ~/.pixi/bin/pixi run bash scripts/stack.sh {up|down|restart|status}
#
# Two hard-won lessons are baked in:
#  * rmw_zenoh ordering: a node started BEFORE `rmw_zenohd` checks for a router
#    once, then runs islanded (never appears in the graph). So `up` starts the
#    ROUTER first, waits, then the web stack, then the nodes.
#  * pkill/pgrep -f are SAFE here because this runs as a script *file* (the
#    shell's argv is just the path). That's the opposite of `plink -m`, where the
#    script text becomes the remote shell's argv, so a `pkill -f <node>` pattern
#    matches — and kills — the running shell itself (seen as plink exit 128).
set -u

NANO="${NANO:-$HOME/Nano}"
LOG="$NANO/.run"; mkdir -p "$LOG"
PARAMS="$NANO/install/robot_bringup/share/robot_bringup/config/robot.yaml"

# The ROS overlay's setup scripts reference unset vars (e.g. COLCON_TRACE);
# relax nounset just around the source so `set -u` can stay on for our logic.
if [ -f "$NANO/install/setup.bash" ]; then
  set +u; source "$NANO/install/setup.bash"; set -u
fi
cd "$NANO" || exit 1

# launch NAME "command…"  — detached, own session, logged to .run/NAME.log
launch() {
  setsid bash -c "exec $2" >"$LOG/$1.log" 2>&1 </dev/null &
  echo "  $1: started -> $LOG/$1.log"
}

do_up() {
  pgrep -f 'rmw_zenohd' >/dev/null \
    || { launch zenohd "ros2 run rmw_zenoh_cpp rmw_zenohd"; sleep 6; }
  pgrep -f 'rosbridge_websocket' >/dev/null \
    || launch web "ros2 launch web_control web.launch.py"
  pgrep -f 'oled_display/lib/oled_display' >/dev/null \
    || launch oled "ros2 run oled_display display_node --ros-args --params-file $PARAMS"
}

do_down() {
  for p in 'oled_display/lib/oled_display' 'ros2 run oled_display' \
           'web_control/lib/web_control' 'rosbridge_websocket' 'rosapi_node' \
           'ros2 launch web_control' 'rmw_zenohd'; do
    pkill -f "$p" 2>/dev/null
  done
}

status() {
  for s in "zenohd:rmw_zenohd" "rosbridge:rosbridge_websocket" \
           "web:web_control/lib/web_control" "oled:oled_display/lib/oled_display"; do
    if pgrep -f "${s#*:}" >/dev/null; then echo "  ${s%%:*}: UP"; else echo "  ${s%%:*}: down"; fi
  done
}

case "${1:-status}" in
  up)      echo "stack up…";      do_up;   sleep 5; status ;;
  down)    echo "stack down…";    do_down; sleep 1; status ;;
  restart) echo "stack restart…"; do_down; sleep 2; do_up; sleep 5; status ;;
  status)  status ;;
  *) echo "usage: $0 {up|down|restart|status}"; exit 2 ;;
esac
