#!/usr/bin/env bash
# Build the rmw-aware micro_ros_agent from source into ~/uros_ws, inside the pixi
# Humble env, so it bridges the ESP32 into the graph via whatever RMW is set
# (rmw_zenoh_cpp here = identical to the SBC). RoboStack no longer ships the agent
# as a package, hence the source build. Run via: pixi run bash <this>.
#
# fmt-12 problem: conda ships fmt 12 / spdlog 1.17, but the pinned
# Micro-XRCE-DDS-Agent v2.4.2 predates fmt 9's formatting changes and logs many
# types via operator<< (endpoints, plus Fast-DDS RTPS types like EntityId_t /
# GuidPrefix_t in TRACE logs) that fmt 9+ rejects as "type_is_unformattable_for".
# Chasing per-type fmt::ostream_formatter shims is whack-a-mole. Instead we build
# the agent with UAGENT_LOGGER_PROFILE=OFF, which compiles every UXR_AGENT_LOG_*
# macro to void(0) and drops the spdlog dependency entirely (Logger.hpp) — killing
# the whole class of fmt-12 errors at the root, and also the stale-bundled-spdlog
# headers that otherwise broke micro_ros_agent's own graph_manager.cpp.
#
# Trade-off: the agent runs without its diagnostic stdout logging. That logging is
# optional instrumentation; connection state is observed via the ROS graph (the
# webtest "ESP32 link" indicator), not agent stdout. To get verbose logs back,
# drop the LOGGER_PROFILE injection and instead add fmt::ostream_formatter
# specializations for every type the agent logs via operator<< (endpoints + the
# Fast-DDS RTPS types EntityId_t/GuidPrefix_t, etc.) under UAGENT_USE_SYSTEM_LOGGER=ON.
#
# The superbuild only forwards a fixed set of -D vars to the inner xrceagent
# ExternalProject, and UAGENT_LOGGER_PROFILE isn't one of them — so we inject it
# straight into that ExternalProject's CMAKE_CACHE_ARGS in SuperBuild.cmake.
set -euo pipefail

WS="$HOME/uros_ws"
mkdir -p "$WS/src"
cd "$WS"

clone() {  # repo-url  dest
  if [ -d "src/$2" ]; then echo "have src/$2"; else
    git clone --depth 1 -b humble "$1" "src/$2"
  fi
}
clone https://github.com/micro-ROS/micro_ros_msgs.git   micro_ros_msgs
clone https://github.com/micro-ROS/micro-ROS-Agent.git  micro-ROS-Agent

# Deterministically inject UAGENT_LOGGER_PROFILE:BOOL=OFF into the xrceagent
# ExternalProject. Restore the file from git first so re-runs are idempotent and
# don't stack injections (or carry over an earlier experiment's edits).
SUPERBUILD="src/micro-ROS-Agent/micro_ros_agent/cmake/SuperBuild.cmake"
git -C src/micro-ROS-Agent checkout -- micro_ros_agent/cmake/SuperBuild.cmake
python3 - "$SUPERBUILD" <<'PY'
import sys, pathlib
sb = pathlib.Path(sys.argv[1])
text = sb.read_text()
anchor = "                -DUAGENT_CED_PROFILE:BOOL=OFF\n"
inject = "                -DUAGENT_LOGGER_PROFILE:BOOL=OFF\n"
assert anchor in text, "couldn't find xrceagent CMAKE_CACHE_ARGS anchor in SuperBuild.cmake"
text = text.replace(anchor, inject + anchor, 1)
sb.write_text(text)
print("injected -DUAGENT_LOGGER_PROFILE:BOOL=OFF into the xrceagent ExternalProject")
PY

echo "=== clean prior build artifacts (stale bundled-spdlog headers, old cache) ==="
rm -rf "$WS/build" "$WS/install" "$WS/log"

echo "=== colcon build (micro_ros_msgs + micro-ROS-Agent) ==="
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

echo "=== DONE. agent at: $WS/install ==="
ls "$WS/install/micro_ros_agent/lib/micro_ros_agent/" 2>/dev/null || true
