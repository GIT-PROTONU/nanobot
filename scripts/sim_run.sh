#!/usr/bin/env bash
# Run the FULL stack on an Ubuntu dev PC with real ROS 2 (via pixi/RoboStack) + Gazebo Sim
# standing in for the lidar/IMU/encoders/ESP32, so dev and the real robot run the exact
# same node graph (see robot_bringup/launch/bringup.launch.py). RViz2 opens alongside it.
#
# This is the ROS-based dev path (needs pixi install on Linux, linux-64). It's a SECOND
# dev path, not a replacement: scripts/dev_webui.py + dev_run.ps1 (Windows, no ROS) still
# work for quick LLM/personality/TTS iteration without installing ROS.
#
#   scripts/sim_run.sh              # build + launch Gazebo + RViz + the whole stack
#   scripts/sim_run.sh --rviz false # skip RViz (still logs to the terminal)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# --- 1. Resolve the OpenRouter key (env first, then memory/openrouter_key, then the old
#        scripts/.openrouter_key location for back-compat) -------------------------------
if [ -z "${OPENROUTER_API_KEY:-}" ] && [ -f memory/openrouter_key ]; then
  export OPENROUTER_API_KEY="$(tr -d '[:space:]' < memory/openrouter_key)"
elif [ -z "${OPENROUTER_API_KEY:-}" ] && [ -f scripts/.openrouter_key ]; then
  export OPENROUTER_API_KEY="$(tr -d '[:space:]' < scripts/.openrouter_key)"
fi
if [ -n "${OPENROUTER_API_KEY:-}" ]; then
  echo "OpenRouter key loaded (...${OPENROUTER_API_KEY: -4})"
else
  echo "No OPENROUTER_API_KEY set (env or memory/openrouter_key) -- the AI card/beats"
  echo "will run offline (phrase bank + reflexes only, no live LLM lines)."
fi

# --- 2. Make sure the phrase bank exists so the first idle beats don't stall on a live
#        LLM call (same rationale as dev_run.ps1; harmless/no-op if already current). ----
pixi run python scripts/pregenerate_phrases.py --if-needed \
  || echo "(phrase-bank pre-build skipped/failed -- continuing; runtime will retry)"

# --- 3. Build + launch: Gazebo Sim + ros_gz_bridge + sim_hardware + the real behaviour/
#        web_control/oled/slam_nav/wheel_odometry stack + (by default) RViz2. `pixi run
#        sim` is the plain equivalent of this without the key/phrase-bank setup above. --
RVIZ="true"
if [ "${1:-}" = "--rviz" ] && [ "${2:-}" = "false" ]; then RVIZ="false"; fi
pixi run build
pixi run bash -c "source install/setup.bash && ros2 launch robot_bringup bringup.launch.py sim:=true rviz:=$RVIZ"
# (equivalent to `pixi run sim` when RVIZ=true; kept explicit here to honour --rviz false)
