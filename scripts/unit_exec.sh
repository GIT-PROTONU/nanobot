#!/usr/bin/env bash
# Activate the pixi/RoboStack env, then EXEC one stack process — the single source of
# truth for what each nano-*.service actually runs. Because we `exec`, the systemd unit
# supervises the node process itself (no resident bash/pixi wrapper): Restart=on-failure
# relaunches a crashed node natively, which replaced the old nano-heal.timer polling
# (and its heal-vs-restart duplicate-node race).
#
#   scripts/unit_exec.sh {router|app|sensors|ekf|nav|map}
#
# Notes baked in from stack.sh's era:
#  * Nodes are launched by their INSTALLED EXECUTABLES, not `ros2 run`/`ros2 launch` —
#    each of those leaves a ~27-40 MB Python CLI wrapper resident per node.
#  * rmw_zenoh ordering: a node started before the router runs islanded. The units
#    encode that with After=nano-router.service (+ the router's start-up settle sleep).
set -u

NANO="${NANO:-$HOME/Nano}"
# glibc gives each thread its own malloc arena (real RSS creep on the threaded nodes);
# cap the arenas — a cheap RSS win on the 1 GB board.
export MALLOC_ARENA_MAX="${MALLOC_ARENA_MAX:-2}"
cd "$NANO" || exit 1

# Activate the pixi env (conda + ROS underlay). `pixi shell-hook` prints the activation
# script and exits, so nothing pixi stays resident after the exec below. The hook sources
# conda activate.d scripts that reference unset vars (ros-workspace: $CONDA_BUILD), so
# relax nounset around the eval, same as install/setup.bash below.
set +u
eval "$("$HOME/.pixi/bin/pixi" shell-hook --manifest-path "$NANO/pixi.toml")" || exit 1
set -u
# The ROS overlay's setup scripts reference unset vars; relax nounset around the source.
if [ -f "$NANO/install/setup.bash" ]; then
  set +u; source "$NANO/install/setup.bash"; set -u
fi

PARAMS="$NANO/install/robot_bringup/share/robot_bringup/config/robot.yaml"
EKF_PARAMS="$NANO/install/robot_bringup/share/robot_bringup/config/ekf.yaml"
OWN="$NANO/install"
LOGDIR="$NANO/.run"; mkdir -p "$LOGDIR"

# LLM key for the app hub: $OPENROUTER_API_KEY wins; else the gitignored one-line
# memory/openrouter_key file (same convention as dev_webui.py / dev_run.ps1).
if [ -z "${OPENROUTER_API_KEY:-}" ] && [ -f "$NANO/memory/openrouter_key" ]; then
  OPENROUTER_API_KEY="$(head -n1 "$NANO/memory/openrouter_key" | tr -d '[:space:]')"
  export OPENROUTER_API_KEY
fi

# ESP32 coprocessor link: the serial-capable zenohd LISTENs on this UART so the ESP32
# (zenoh-pico, no micro-ROS agent) joins the graph directly.
ESP32_UART="${ESP32_UART:-/dev/ttyS1}"
ESP32_BAUD="${ESP32_BAUD:-115200}"
ZENOHD_SERIAL="${ZENOHD_SERIAL:-$NANO/bin/zenohd-serial}"

case "${1:-}" in
  router)
    # The router MUST run with rmw_zenoh's own ROUTER config (not zenohd defaults):
    # default routing lets the ROS peers gossip into a direct mesh that bypasses
    # delivery of the ESP32 (a zenoh CLIENT) data. Generate that config + add the
    # serial listen endpoint; exit_on_failure:false so a transient serial desync
    # can't kill the router.
    rcfg="$LOGDIR/router_serial.json5"
    python - "$rcfg" "$ESP32_UART" "$ESP32_BAUD" <<'PY' || exit 1
import sys, os
out, uart, baud = sys.argv[1], sys.argv[2], sys.argv[3]
src = f"{os.environ['CONDA_PREFIX']}/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"
t = open(src).read()
old = '    endpoints: [\n      "tcp/[::]:7447"\n    ],'
new = f'    endpoints: [\n      "tcp/[::]:7447",\n      "serial/{uart}#baudrate={baud}"\n    ],'
assert t.count(old) == 1, "router config listen-endpoints block not found as expected"
open(out, "w").write(t.replace(old, new).replace("exit_on_failure: true", "exit_on_failure: false"))
PY
    exec "$ZENOHD_SERIAL" -c "$rcfg"
    ;;
  app)      # web_control + oled_display + behavior in ONE process (see app_hub)
    exec "$OWN/app_hub/lib/app_hub/app_hub" --ros-args --params-file "$PARAMS"
    ;;
  sensors)  # imu + sys_monitor + wheel_odometry + lds in ONE process (see sensor_hub)
    exec "$OWN/sensor_hub/lib/sensor_hub/sensor_hub" --ros-args --params-file "$PARAMS"
    ;;
  ekf)      # robot_localization EKF: fuses /odom (wheel encoders) + /imu/data (IMU)
            # into a single filtered pose on /odometry/filtered + odom->base_link TF.
            # Node name must match ekf.yaml's top-level key (ekf_node).
    exec "$CONDA_PREFIX/lib/robot_localization/ekf_node" \
      --ros-args -r __node:=ekf_node --params-file "$EKF_PARAMS"
    ;;
  nav)
    exec "$OWN/slam_nav/lib/slam_nav/nav_node" --ros-args --params-file "$PARAMS"
    ;;
  map)      # republishes /dev/shm/nano_map.bin as /map for a remote RViz
    exec "$OWN/sim_hardware/bin/map_bridge_node" --ros-args --params-file "$PARAMS"
    ;;
  *)
    echo "usage: $0 {router|app|sensors|ekf|nav|map}" >&2
    exit 2
    ;;
esac
