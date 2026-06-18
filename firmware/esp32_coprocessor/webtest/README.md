# ESP32 coprocessor — browser test rig

A no-CLI way to exercise the flashed **micro-ROS** firmware end to end from a
browser: toggle the onboard LED (`/led`), watch `/wheel_ticks` stream live, see the
wheel-suspension switch (`/wheel_suspended`), and (with an explicit safety opt-in)
nudge the motors (`/cmd_vel`).

```
Browser ──HTTP──▶ esp32_webtest.py (rclpy + native micro_ros_agent) ──micro-ROS──▶ ESP32
```

One command brings up the agent, an rclpy bridge, and the web UI:

```sh
pixi run python firmware/esp32_coprocessor/webtest/esp32_webtest.py
# open http://localhost:8088
```

## The agent (build it once)
The micro-ROS firmware speaks XRCE-DDS over serial, so it needs a `micro_ros_agent`
to join the ROS 2 graph. RoboStack no longer ships one for any platform, so build it
from source (no Docker):

```sh
pixi run bash firmware/esp32_coprocessor/webtest/build_agent.sh   # -> ~/uros_ws
```

`esp32_webtest.py` then finds it automatically: a `micro_ros_agent` on PATH (e.g. on
the board), else the `~/uros_ws` overlay (sourced + `ros2 run`).

> **The agent is Fast-DDS only.** It links `librmw_fastrtps_shared_cpp` + `libfastrtps`,
> so it always bridges the ESP32 onto a Fast-DDS/RTPS graph regardless of
> `RMW_IMPLEMENTATION`. A pure `rmw_zenoh_cpp` graph speaks a different wire protocol
> and **cannot see the agent's topics** (the agent + ESP32 connect, but no data flows).
> Hence the default `--rmw rmw_fastrtps_cpp`. Getting the ESP32 onto the robot's zenoh
> graph needs a zenoh↔DDS bridge — a separate, unresolved decision.

## Options
| flag | default | note |
|------|---------|------|
| `--dev` | `/dev/ttyUSB0` | ESP32 serial port |
| `--baud` | `115200` | must match the firmware |
| `--port` | `8088` | web UI port |
| `--rmw` | `rmw_fastrtps_cpp` | the only RMW the agent bridges over (see above) |
| `--agent-overlay` | `~/uros_ws/install` | colcon overlay with the source-built agent |
| `--no-agent` | off | reuse an already-running agent instead of spawning one |
| `--no-router` | off | don't auto-start `rmw_zenohd` (only relevant under `--rmw rmw_zenoh_cpp`) |

## What the indicators mean
- **ESP32 link** — the agent created the ESP32's `wheel_ticks` publisher (XRCE session
  established). Detected via `count_publishers`, not a node name — micro-ROS over the
  agent doesn't register a discoverable node name.
- **wheel_ticks Hz** — live publish rate (~30 Hz). Spin the wheel and the left count
  climbs (single-channel encoder, unsigned counts).
- **wheel** — green "down" / red "UP (suspended)" from the D18 microswitch.
- **web link** — the browser ↔ server poll is alive.

The LED toggle and motor nudge exercise the host → ESP32 (subscribe) direction.

## Serial sanity check
The firmware is silent on the wire until an agent connects (pure XRCE-DDS), so there's
nothing human-readable to `cat`. Use `--no-agent` only if you've started your own agent.
