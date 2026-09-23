#!/usr/bin/env bash
# Activate the pixi/RoboStack env, then EXEC one stack process — the single source of
# truth for what each nano-*.service actually runs. Because we `exec`, the systemd unit
# supervises the node process itself (no resident bash/pixi wrapper): Restart=on-failure
# relaunches a crashed node natively, which replaced the old nano-heal.timer polling
# (and its heal-vs-restart duplicate-node race).
#
#   scripts/unit_exec.sh {router|app|sensors|nav|nav-loader|slam|tf}
#
# Notes baked in from stack.sh's era:
#  * Nodes are launched by their INSTALLED EXECUTABLES, not `ros2 run`/`ros2 launch` —
#    each of those leaves a ~27-40 MB Python CLI wrapper resident per node.
#  * rmw_zenoh ordering: a node started before the router runs islanded. The units
#    encode that with After=nano-router.service (+ the router's start-up settle sleep).
#  * nav = the ONE heavy process: an rclcpp_components/component_container_isolated
#    hosting the Nav2 servers + their lifecycle manager (see
#    robot_bringup/launch/nav2.launch.py). The nano-nav-loader unit (After=/
#    Requisite=nano-nav) attaches the six components to that container via
#    `nav2.launch.py load_only:=true` — the launch's LoadComposableNodes retries
#    the container's load-node service every 1 s until it appears. Fallback if
#    the loader path ever misbehaves: `ros2 launch robot_bringup nav2.launch.py`
#    (spawns container + components + slam_toolbox + TF itself).
#  * slam = slam_toolbox 2.6.10 (robostack's only build): a PLAIN rclcpp::Node
#    whose executable main calls configure() itself — no lifecycle services, so
#    it must run as its own process (nano-slam unit); composing it is inert.
#  * tf = the static base_link -> laser transform (yaw π: this unit's sensor
#    head faces back — slam_nav's old heading_flip, now expressed in TF world).
#    nano-nav.service runs it as ExecStartPost so both die/restart together.
set -u

# Clock-step guard: the board has no battery-backed RTC, so every power-on boots with a
# days-stale fake-hwclock and NTP STEPS the clock forward ~a minute after boot — mid-run,
# while the stack is already up. A step under a live SLAM session destroys it (every scan
# stamp jumps ~44 h; slam_toolbox's message filter drops them all: "earlier than all the
# data in the transform cache", 2026-09-19 12:00:53). Wait up to 20 s for NTP before
# exec'ing anything, then proceed anyway so an offline robot still boots its stack
# (bounded, never a permanent block). Every unit runs this; after the router's wait the
# rest see a synced clock instantly. Keep it here in ONE place —
# deploy/systemd/nano-router.service deliberately has no ExecStartPre twin.
for i in $(seq 1 40); do
  [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = yes ] && break
  sleep 0.5
  [ "$i" = 40 ] && echo "unit_exec: clock not NTP-synced after 20s — starting anyway (stale-clock risk)" >&2
done

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
NAV2_PARAMS="$NANO/install/robot_bringup/share/robot_bringup/config/nav2/nav2_params.yaml"
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

# Cross-host discovery fix (2026-09-22): the ROS units ran as zenoh PEERs (loopback
# gossip). Peer declarations do NOT propagate through the router to later joiners, so
# a dev-PC session pointed at tcp/<board>:7447 saw ONLY the ESP32's topics (it is a
# zenoh CLIENT) — /scan /odom /tf /map stayed invisible (one-sided blindness,
# 2026-09-21 test). Client sessions demonstrably propagate (the ESP32's do), so run
# every ROS unit as a CLIENT of the router — the same declaration path as the ESP32.
# GOTCHA (found + fixed live 2026-09-22): with plain client configs every rmw_zenoh
# node except the FIRST after boot dies at rmw_init with "Failed to create POSIX SHM
# provider (OS error 12)" (nano-app/slam/nav crash-looped 50+ times; sensors — first
# session — survived). Disabling zenoh's shared-memory transport in the session config
# fixes init entirely; the cost is an extra copy on big loopback messages (/map, /scan)
# — negligible at their rates, and the ESP32 serial link never used SHM anyway.
# The router branch (raw zenohd with its own -c config) must NOT get this env.
# NANO_ZENOH_PEER=1 reverts to the old peer behaviour.
if [ "${1:-}" != "router" ] && [ -z "${NANO_ZENOH_PEER:-}" ]; then
  ZCFG="$LOGDIR/zenoh_client.json5"
  cat > "$ZCFG" <<'EOF'
{
  mode: "client",
  connect: { endpoints: ["tcp/localhost:7447"] },
  transport: { shared_memory: { enabled: false } },
}
EOF
  export ZENOH_SESSION_CONFIG_URI="$ZCFG"
fi

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
  nav)      # ONE heavy C++ process: the rclcpp component container (isolated
            # executor, upstream nav2's Humble choice) hosting planner_server +
            # controller_server + velocity_smoother + bt_navigator +
            # behavior_server + their lifecycle manager (see nav2.launch.py).
            # Components are attached by the nano-nav-loader unit
            # (load_only:=true). The container
            # itself gets the FULL params file: launch_ros inlines only the
            # sections matching each component's own name into the load
            # request, so the DOUBLE-NESTED costmap sections
            # (local_costmap.local_costmap.*) never reached the child costmap
            # nodes those servers create at runtime — both costmaps silently
            # ran on Nav2 defaults (inflation 0.55, robot_radius 0.1,
            # always_send_full_costmap false; found 2026-09-22 when the web
            # costmap overlay shipped). A process-wide --params-file applies
            # by node FQN instead, so the child nodes match their sections.
    exec "$CONDA_PREFIX/lib/rclcpp_components/component_container_isolated" \
      --ros-args -r __node:=nav2_container --params-file "$NAV2_PARAMS"
    ;;
  nav-loader)
    # Wait for the container (up to ~30 s at 0.5 s steps) — the launch's
    # LoadComposableNodes retries its service for ever, but a bounded poll first
    # means a dead container surfaces as THIS unit failing (visible in the journal
    # + systemctl) instead of a silently looping loader.
    for i in $(seq 1 60); do
      if ros2 node list --no-daemon 2>/dev/null | grep -q "nav2_container"; then break; fi
      if [ "$i" = 60 ]; then
        echo "nav-loader: nav2_container never appeared — is nano-nav.service up?" >&2
        exit 1
      fi
      sleep 0.5
    done
    exec "$CONDA_PREFIX/bin/ros2" launch robot_bringup nav2.launch.py load_only:=true
    ;;
  slam)     # slam_toolbox 2.6.10 (the only robostack build): PLAIN rclcpp::Node,
            # self-configuring executable. Provides /map + the map->odom TF that
            # Nav2's costmaps/navigator build on.
    exec "$CONDA_PREFIX/lib/slam_toolbox/async_slam_toolbox_node" \
      --ros-args --params-file "$NAV2_PARAMS"
    ;;
  tf)       # static base_link -> laser, yaw pi (sensor head mounted facing back —
            # the TF-world replacement for slam_nav's heading_flip; see
            # nav2.launch.py's heading_flip arg for the launch-side equivalent).
    exec "$CONDA_PREFIX/lib/tf2_ros/static_transform_publisher" \
      --x 0 --y 0 --z 0.065 --yaw 3.14159265 \
      --frame-id base_link --child-frame-id laser
    ;;
  *)
    echo "usage: $0 {router|app|sensors|nav|nav-loader|slam|tf}" >&2
    exit 2
    ;;
esac
