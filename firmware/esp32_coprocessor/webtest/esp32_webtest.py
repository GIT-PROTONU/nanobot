#!/usr/bin/env python3
"""Browser test rig for the ESP32 micro-ROS coprocessor — no ROS CLI needed.

One command brings up the whole pipeline and a web UI:
  * spawns `micro_ros_agent serial` on the ESP32's USB port (bridges its
    micro-ROS link into the ROS 2 graph),
  * runs an rclpy node that publishes `/led` (std_msgs/Bool), subscribes
    `/wheel_ticks` (std_msgs/Int64MultiArray, best-effort to match the firmware),
    and can nudge `/cmd_vel` (geometry_msgs/Twist),
  * serves a small page: toggle the onboard LED, watch wheel_ticks stream live,
    and (optionally) drive the motors.

Run it from the pixi ROS env, e.g.:
    pixi run python firmware/esp32_coprocessor/webtest/esp32_webtest.py
then open http://localhost:8088.

It launches the native micro_ros_agent (built into ~/uros_ws by build_agent.sh,
or found on PATH on the board) and an rclpy bridge node.

RMW: defaults to rmw_fastrtps_cpp. This is NOT cosmetic — micro_ros_agent is
hardwired to Fast-DDS (it links librmw_fastrtps_shared_cpp + libfastrtps; see
`ldd`), so it always bridges the ESP32 onto a Fast-DDS/RTPS graph regardless of
RMW_IMPLEMENTATION. A pure rmw_zenoh_cpp graph speaks a different wire protocol
and simply cannot see the agent's topics (verified: ticks flow under Fast-DDS,
nothing under zenoh). So the robot's ESP32 path needs Fast-DDS (or a zenoh<->DDS
bridge) — a separate decision from this test tool. Passing `--rmw rmw_zenoh_cpp`
here will start a router but you'll see no ESP32 data; it's left in only for that
future bridge work.
"""
import argparse
import json
import os
import shlex
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent


class State:
    """Shared snapshot between the rclpy node and the HTTP threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.ticks = [0, 0]
        self.msg_count = 0          # monotonic count of wheel_ticks received
        self.last_rx = 0.0          # monotonic time of last wheel_ticks
        self.led = False            # last commanded LED state
        self.node_present = False   # is the esp32_coprocessor node in the graph?
        self.suspended = None       # wheel off the ground? None until first msg
        self.rmw = ""
        self.dev = ""

    def snapshot(self):
        with self.lock:
            age = (time.monotonic() - self.last_rx) if self.last_rx else None
            return {
                "ticks": list(self.ticks),
                "count": self.msg_count,
                "age": age,
                "led": self.led,
                "node": self.node_present,
                "suspended": self.suspended,
                "rmw": self.rmw,
                "dev": self.dev,
            }


state = State()
pub = {}  # publish callbacks, filled in once the node is up


def agent_command(args):
    """argv to launch the native micro_ros_agent over serial, or None to skip.

    Prefers a `micro_ros_agent` already on PATH (the board, or any sourced ROS
    env). Otherwise falls back to the source-built overlay in ~/uros_ws (created
    by build_agent.sh on a linux-64 dev PC, where RoboStack ships no agent): the
    overlay is sourced over the current pixi env and the agent run via `ros2 run`.
    Whatever RMW is in the environment (rmw_zenoh_cpp by default) is inherited, so
    the agent bridges the ESP32 into the graph exactly like it does on the SBC.
    """
    if args.no_agent:
        return None
    serial = ["serial", "--dev", args.dev, "-b", str(args.baud)]

    if shutil.which("micro_ros_agent"):
        return ["micro_ros_agent"] + serial

    setup = Path(args.agent_overlay).expanduser() / "setup.bash"
    if setup.is_file():
        inner = "ros2 run micro_ros_agent micro_ros_agent " + " ".join(serial)
        return ["bash", "-c", f"source {shlex.quote(str(setup))} && exec {inner}"]

    raise SystemExit(
        "[webtest] no micro_ros_agent found. Build it once with:\n"
        "    pixi run bash firmware/esp32_coprocessor/webtest/build_agent.sh\n"
        f"(installs into {args.agent_overlay}), or run where it's on PATH (the board).")


def ensure_router():
    """Start a zenoh router (rmw_zenohd) if one isn't already running.

    rmw_zenoh needs a router; a node started before it runs islanded (won't appear
    in the graph) — same ordering stack.sh enforces on the SBC. Returns the Popen
    we started (to clean up later), or None if one was already up."""
    if subprocess.run(["pgrep", "-x", "rmw_zenohd"],
                      stdout=subprocess.DEVNULL).returncode == 0:
        print("[webtest] zenoh router already running")
        return None
    print("[webtest] starting zenoh router: ros2 run rmw_zenoh_cpp rmw_zenohd")
    return subprocess.Popen(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"])


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):  # quiet; this is a test tool
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = (HERE / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/state":
                self._json(state.snapshot())
            else:
                self.send_error(404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(n) if n else b"{}"
            try:
                data = json.loads(raw or b"{}")
            except json.JSONDecodeError:
                self._json({"error": "bad json"}, 400)
                return
            if self.path == "/led":
                on = bool(data.get("on"))
                pub["led"](on)
                self._json({"ok": True, "led": on})
            elif self.path == "/cmd_vel":
                pub["cmd_vel"](float(data.get("lin", 0.0)), float(data.get("ang", 0.0)))
                self._json({"ok": True})
            else:
                self.send_error(404)

    return Handler


def main():
    ap = argparse.ArgumentParser(
        description="Browser test rig for the ESP32 micro-ROS coprocessor.")
    ap.add_argument("--dev", default="/dev/ttyUSB0", help="ESP32 serial device")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--port", type=int, default=8088, help="web UI port")
    # Fast-DDS is the only RMW the agent actually bridges over (see module docstring);
    # rmw_zenoh_cpp is accepted but won't surface ESP32 topics without a DDS bridge.
    ap.add_argument("--rmw", default="rmw_fastrtps_cpp")
    ap.add_argument("--agent-overlay", default="~/uros_ws/install",
                    help="colcon overlay holding the source-built micro_ros_agent")
    ap.add_argument("--no-agent", action="store_true",
                    help="don't spawn an agent (reuse an already-running one)")
    ap.add_argument("--no-router", action="store_true",
                    help="don't auto-start rmw_zenohd (reuse an existing router)")
    args = ap.parse_args()

    # RMW must be chosen before rclpy initialises and before the agent/router start;
    # they all inherit it from the environment.
    os.environ["RMW_IMPLEMENTATION"] = args.rmw

    # zenoh: the router must be up before any node/agent, or they run islanded.
    router = None
    if args.rmw == "rmw_zenoh_cpp" and not args.no_router:
        router = ensure_router()
        if router is not None:
            time.sleep(2.0)  # let the router bind before peers connect

    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Int64MultiArray
    from geometry_msgs.msg import Twist

    cmd = agent_command(args)
    agent = None
    if cmd:
        print(f"[webtest] starting agent: {' '.join(cmd)}")
        agent = subprocess.Popen(cmd)

    rclpy.init()
    node = rclpy.create_node("esp32_webtest")
    state.rmw, state.dev = args.rmw, args.dev

    led_pub = node.create_publisher(Bool, "led", 10)
    cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)
    # wheel_ticks is published best-effort by the firmware — the subscriber must
    # match or it sees nothing.
    best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

    def on_ticks(msg):
        d = list(msg.data)
        with state.lock:
            state.ticks = (d + [0, 0])[:2]
            state.msg_count += 1
            state.last_rx = time.monotonic()

    node.create_subscription(Int64MultiArray, "wheel_ticks", on_ticks, best_effort)

    # /wheel_suspended is published reliably (state topic) — default QoS matches.
    def on_susp(msg):
        with state.lock:
            state.suspended = bool(msg.data)

    node.create_subscription(Bool, "wheel_suspended", on_susp, 10)

    def publish_led(on):
        led_pub.publish(Bool(data=bool(on)))
        with state.lock:
            state.led = bool(on)

    def publish_cmd(lin, ang):
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        cmd_pub.publish(m)

    pub["led"], pub["cmd_vel"] = publish_led, publish_cmd

    def poll_presence():
        # micro-ROS over the agent doesn't reliably register a discoverable node
        # *name*, so get_node_names() is unreliable here. The solid signal that the
        # agent established the ESP32's XRCE session is the wheel_ticks publisher it
        # creates on the ESP32's behalf (we only subscribe, never publish it).
        present = node.count_publishers("wheel_ticks") > 0
        with state.lock:
            state.node_present = present

    node.create_timer(1.0, poll_presence)

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler())
    httpd.daemon_threads = True
    print(f"[webtest] open http://localhost:{args.port}  "
          f"(RMW={args.rmw}, dev={args.dev})")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[webtest] shutting down")
    finally:
        httpd.shutdown()
        try:
            publish_cmd(0.0, 0.0)  # coast motors on exit
        except Exception:
            pass
        rclpy.shutdown()
        for proc in (agent, router):  # agent first, then the router we started
            if not proc:
                continue
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    main()
