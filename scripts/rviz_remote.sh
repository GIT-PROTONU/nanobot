#!/usr/bin/env bash
# Watch the REAL robot live in RViz from an Ubuntu dev PC -- NOT a simulation (that's
# scripts/sim_run.sh / Gazebo). The robot keeps running its own `stack.sh` unchanged;
# this just joins the same rmw_zenoh graph and opens robot_state_publisher + rviz2
# (robot_bringup/launch/visualize.launch.py).
#
# Prerequisites:
#  - the robot is up: `stack.sh up` on the board (now also runs sim_hardware's
#    map_bridge_node, so /map exists as a real topic -- see CLAUDE.md "Remote RViz").
#  - same ROS_DOMAIN_ID / RMW_IMPLEMENTATION -- already guaranteed since both machines
#    activate the SAME pixi.toml (its [activation.env] sets both).
#  - the dev PC's zenoh session can reach the robot's zenohd-serial router. Same-LAN
#    multicast scouting usually finds it with no extra config -- just run:
#        scripts/rviz_remote.sh
#    If that doesn't discover the robot (blocked multicast / different subnet / a
#    router that isolates clients), point the session at it explicitly:
#        scripts/rviz_remote.sh --connect 192.168.1.42
#
# NOTE: ZENOH_SESSION_CONFIG_URI is rmw_zenoh_cpp's documented env var for the SESSION
# (client) side config, as opposed to ZENOH_ROUTER_CONFIG_URI for the router (rmw_zenohd)
# -- distinct from stack.sh's router config, which is instead passed as a `-c` CLI arg to
# the custom zenohd-serial binary. Written without a way to test the exact discovery
# behaviour end-to-end from here; if `ros2 topic list` doesn't show the robot's topics
# after this, check the installed rmw_zenoh_cpp version's docs for the current env var
# name/session-config schema.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [ "${1:-}" = "--connect" ] && [ -n "${2:-}" ]; then
  ROBOT_IP="$2"
  CFG="$(mktemp -t nano_zenoh_session.XXXXXX.json5)"
  cat > "$CFG" <<EOF
{
  mode: "peer",
  connect: { endpoints: ["tcp/${ROBOT_IP}:7447"] },
}
EOF
  export ZENOH_SESSION_CONFIG_URI="$CFG"
  echo "Connecting explicitly to the robot's zenoh router at tcp/${ROBOT_IP}:7447"
  echo "  (session config: $CFG)"
else
  echo "Relying on same-LAN zenoh multicast scouting to find the robot."
  echo "  (pass --connect <robot-ip> if the robot's topics don't show up)"
fi

pixi run build
pixi run visualize
