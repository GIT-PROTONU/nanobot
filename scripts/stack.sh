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
  # Launch every node DIRECTLY by its installed executable, not via `ros2 run` /
  # `ros2 launch`: each of those leaves a ~27-40 MB Python CLI wrapper resident
  # for the lifetime of the node. On the 970 MB board that overhead added up to
  # ~175 MB across the stack. rosapi is also dropped — the web page talks to known
  # topics and never enumerates, so it isn't needed (~65 MB more).
  local ros="$CONDA_PREFIX/lib"     # ROS package libexec dirs in the pixi env
  local own="$NANO/install"         # our colcon packages
  pgrep -f 'rmw_zenohd' >/dev/null \
    || { launch zenohd "$ros/rmw_zenoh_cpp/rmw_zenohd"; sleep 6; }
  pgrep -f 'rosbridge_websocket' >/dev/null \
    || launch rosbridge "$ros/rosbridge_server/rosbridge_websocket --ros-args -p port:=9090"
  pgrep -f 'web_control/lib/web_control' >/dev/null \
    || launch web "$own/web_control/lib/web_control/web_server --ros-args -p web_port:=8080 -p rosbridge_port:=9090"
  pgrep -f 'oled_display/lib/oled_display' >/dev/null \
    || launch oled "$own/oled_display/lib/oled_display/display_node --ros-args --params-file $PARAMS"
  pgrep -f 'imu_driver/lib/imu_driver' >/dev/null \
    || launch imu "$own/imu_driver/lib/imu_driver/imu_node --ros-args --params-file $PARAMS"
  pgrep -f 'sys_monitor/lib/sys_monitor' >/dev/null \
    || launch sys "$own/sys_monitor/lib/sys_monitor/monitor_node --ros-args --params-file $PARAMS"
  # LDS: Python driver (the Rust lds_driver doesn't build against this RoboStack).
  pgrep -f 'lds_driver_py/lib/lds_driver_py' >/dev/null \
    || launch lds "$own/lds_driver_py/lib/lds_driver_py/lds_node --ros-args --params-file $PARAMS"
}

do_down() {
  # Node path substrings match whether launched directly or via ros2 run/launch.
  # rosapi_node + ros2cli.daemon sweep up anything left by older launches or by
  # `ros2 ...` CLI probes (each spawns a ~60 MB daemon).
  for p in 'lds_driver_py/lib/lds_driver_py' \
           'sys_monitor/lib/sys_monitor' \
           'imu_driver/lib/imu_driver' \
           'oled_display/lib/oled_display' \
           'web_control/lib/web_control' \
           'rosbridge_websocket' 'rosapi_node' 'ros2cli.daemon' \
           'rmw_zenohd'; do
    pkill -f "$p" 2>/dev/null
  done
}

status() {
  for s in "zenohd:rmw_zenohd" "rosbridge:rosbridge_websocket" \
           "web:web_control/lib/web_control" "oled:oled_display/lib/oled_display" \
           "imu:imu_driver/lib/imu_driver" "sys:sys_monitor/lib/sys_monitor" \
           "lds:lds_driver_py/lib/lds_driver_py"; do
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
