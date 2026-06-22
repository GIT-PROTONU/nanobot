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

# glibc gives each thread its own malloc arena (up to ~8*cores*64 MB of address space,
# with real RSS creep). The threaded Python nodes (imu/lds/web/nav serial+HTTP readers)
# are exactly that pattern, so cap the arenas — a cheap RSS win on the 1 GB board. Every
# node is launched as a child of this script, so the export is inherited by all of them.
export MALLOC_ARENA_MAX="${MALLOC_ARENA_MAX:-2}"

NANO="${NANO:-$HOME/Nano}"
LOG="$NANO/.run"; mkdir -p "$LOG"
PARAMS="$NANO/install/robot_bringup/share/robot_bringup/config/robot.yaml"
# ESP32 coprocessor: runs zenoh-pico over a direct UART (NO micro-ROS agent). The
# serial-capable zenohd LISTENs on this UART so the ESP32 joins the zenoh graph
# directly (/cmd_vel,/led,/lds_target_rpm in; /wheel_ticks,/lds_*,/esp32_* etc out).
# Build the binary on a dev host: firmware/nanobot_coprocessor/tools/build_zenohd_serial.sh aarch64
ESP32_UART="${ESP32_UART:-/dev/ttyS1}"
ESP32_BAUD="${ESP32_BAUD:-115200}"
ZENOHD_SERIAL="${ZENOHD_SERIAL:-$NANO/bin/zenohd-serial}"

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
  # ROUTER: the serial-capable zenohd (built with --features transport_serial; the
  # conda libzenohc has NO serial support). It LISTENs on TCP for the rmw_zenoh stack
  # AND on the ESP32's UART, so the ESP32 (running zenoh-pico, NO micro-ROS agent, NO
  # DDS) joins the graph directly. Replaces conda rmw_zenohd + micro_ros_agent.
  #
  # It MUST run with rmw_zenoh's own ROUTER config (not zenohd defaults): the default
  # routing lets the ROS peers gossip into a direct mesh that bypasses delivery of the
  # ESP32 (a zenoh CLIENT) data to them. We generate that config + add the serial listen
  # endpoint, and set exit_on_failure:false so a transient serial desync can't kill the
  # router. (The ESP32 firmware also disables its LDS UART so it can keep the zenoh
  # serial link fed under the rmw config's tighter transport timings.)
  local rcfg="$LOG/router_serial.json5"
  python - "$rcfg" "$ESP32_UART" "$ESP32_BAUD" <<'PY'
import sys, os
out, uart, baud = sys.argv[1], sys.argv[2], sys.argv[3]
src = f"{os.environ['CONDA_PREFIX']}/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"
t = open(src).read()
old = '    endpoints: [\n      "tcp/[::]:7447"\n    ],'
new = f'    endpoints: [\n      "tcp/[::]:7447",\n      "serial/{uart}#baudrate={baud}"\n    ],'
assert t.count(old) == 1, "router config listen-endpoints block not found as expected"
open(out, "w").write(t.replace(old, new).replace("exit_on_failure: true", "exit_on_failure: false"))
PY
  pgrep -x 'zenohd-serial' >/dev/null \
    || { launch zenohd "$ZENOHD_SERIAL -c $rcfg"; sleep 6; }
  pgrep -f 'rosbridge_websocket' >/dev/null \
    || launch rosbridge "$ros/rosbridge_server/rosbridge_websocket --ros-args -p port:=9090"
  pgrep -f 'web_control/lib/web_control' >/dev/null \
    || launch web "$own/web_control/lib/web_control/web_server --ros-args -p web_port:=8080 -p rosbridge_port:=9090"
  pgrep -f 'oled_display/lib/oled_display' >/dev/null \
    || launch oled "$own/oled_display/lib/oled_display/display_node --ros-args --params-file $PARAMS"
  # Sensor hub: imu_driver + sys_monitor + wheel_odometry + lds_driver_py in ONE process
  # (one executor) to save ~100+ MB of RAM vs four separate interpreters on the 1 GB board.
  # Same node names/topics/params, so /odom, /diagnostics, /scan, /imu/*, the live-retune
  # services and the LDS scan blob (/dev/shm) are all unchanged. The LDS data wire fans out
  # to the ESP32 (UART1 RX=GPIO14, RPM->spin PID) AND the SBC's UART2 (/dev/ttyS2, full
  # scan); UART2 needs the `uart2` overlay (deploy/sbc-setup.sh) + a reboot to exist.
  pgrep -f 'sensor_hub/lib/sensor_hub' >/dev/null \
    || launch sensors "$own/sensor_hub/lib/sensor_hub/sensor_hub --ros-args --params-file $PARAMS"
  # SLAM/mapping: builds an occupancy grid from /scan + /odom + /imu/euler and writes
  # /dev/shm/nano_map.bin for the web map panel. Started last (needs scan + odom flowing).
  pgrep -f 'slam_nav/lib/slam_nav' >/dev/null \
    || launch nav "$own/slam_nav/lib/slam_nav/nav_node --ros-args --params-file $PARAMS"
}

# Node path substrings match whether launched directly or via ros2 run/launch.
# rosapi_node + ros2cli.daemon sweep up anything left by older launches or by
# `ros2 ...` CLI probes (each spawns a ~60 MB daemon).
# sensor_hub now hosts imu/sys/odom/lds in one process, but the old per-node patterns are
# kept here too so `down`/`restart` also sweeps up stragglers from a pre-merge deploy.
NODE_PATS=(
  'slam_nav/lib/slam_nav'
  'sensor_hub/lib/sensor_hub'
  'lds_driver_py/lib/lds_driver_py'
  'wheel_odometry/lib/wheel_odometry'
  'sys_monitor/lib/sys_monitor'
  'imu_driver/lib/imu_driver'
  'oled_display/lib/oled_display'
  'web_control/lib/web_control'
  'rosbridge_websocket' 'rosapi_node' 'ros2cli.daemon'
)

# true while ANY managed process is still alive (nodes by -f pattern, router by exact name)
any_alive() {
  local p
  for p in "${NODE_PATS[@]}"; do pgrep -f "$p" >/dev/null && return 0; done
  pgrep -x 'zenohd-serial' >/dev/null && return 0
  return 1
}

do_down() {
  # The old version sent ONE SIGTERM then slept a fixed 1-2 s. A node slow to exit
  # survived, and then do_up's pgrep guard saw it still running and SKIPPED relaunch —
  # leaving a stale process holding the port / serving old code (the "change didn't
  # take" foot-gun in CLAUDE.md). Now: SIGTERM all, WAIT for actual exit, then SIGKILL
  # any straggler and verify.
  local p
  for p in "${NODE_PATS[@]}"; do pkill -f "$p" 2>/dev/null; done
  pkill -x 'zenohd-serial' 2>/dev/null    # router holds the ESP32 UART; kill by exact name

  # Poll up to ~6 s, returning as soon as everything is gone.
  local i
  for i in $(seq 1 30); do any_alive || break; sleep 0.2; done

  # SIGKILL whatever ignored SIGTERM, then report anything still standing.
  for p in "${NODE_PATS[@]}"; do
    pgrep -f "$p" >/dev/null && { echo "  forcing kill: $p"; pkill -9 -f "$p" 2>/dev/null; }
  done
  pgrep -x 'zenohd-serial' >/dev/null && { echo "  forcing kill: zenohd-serial"; pkill -9 -x 'zenohd-serial' 2>/dev/null; }
  sleep 0.3
  any_alive && echo "  WARNING: a managed process is still alive after SIGKILL"
}

status() {
  for s in "zenohd:zenohd-serial" "rosbridge:rosbridge_websocket" \
           "web:web_control/lib/web_control" "oled:oled_display/lib/oled_display" \
           "sensors:sensor_hub/lib/sensor_hub" \
           "nav:slam_nav/lib/slam_nav"; do
    if pgrep -f "${s#*:}" >/dev/null; then echo "  ${s%%:*}: UP"; else echo "  ${s%%:*}: down"; fi
  done
}

case "${1:-status}" in
  up)      echo "stack up…";      do_up;   sleep 5; status ;;
  down)    echo "stack down…";    do_down; status ;;          # do_down now blocks until gone
  restart) echo "stack restart…"; do_down; do_up; sleep 5; status ;;
  status)  status ;;
  *) echo "usage: $0 {up|down|restart|status}"; exit 2 ;;
esac
