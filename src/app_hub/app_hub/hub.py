"""Single-process host for the expression/cognition layer (the "app" fault domain).

Runs web_control's web_server, oled_display's display_node and behavior's mood_node in
ONE process under a single executor — the same packaging move sensor_hub made for the
sensor layer. On the 1 GB H5 each separate rclpy process carries a full interpreter +
rmw baseline (tens of MB each); merging the three reclaims that RAM with NO change to
node names, topics, params or behaviour (params still load per-name from
`--ros-args --params-file robot.yaml`). The stack is now three fault domains =
three hubs: sensor_hub (the body), slam_nav (spatial), app_hub (expression/web/brain).

Trade-off (same as sensor_hub, accepted): the three no longer crash/restart
independently. In practice web_server dying always took the face's *content* with it
anyway (cognition, TTS karaoke, web moods), and systemd's Restart=on-failure is the
real safety net — see deploy/systemd/nano-app.service.

Shutdown mirrors oled_display's standalone main: SIGTERM (systemctl stop / poweroff)
trips a flag, the spin loop exits, and the display node draws its end-screen
(restart/shutdown glyph) before the process leaves — so the panel behaviour on a stack
restart or board shutdown is unchanged.
"""
import os
import signal
import socket
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

from oled_display.display_node import DisplayNode
from web_control.web_server import WebServerNode
from behavior.mood_node import MoodNode

# DisplayNode first so its /oled_face subscription exists before MoodNode's chart
# publishes the boot greeting face during construction.
NODE_CLASSES = (DisplayNode, WebServerNode, MoodNode)


def _sd_notify(msg):
    """Best-effort systemd notification (Type=notify units): READY on start, then
    WATCHDOG pets from an executor timer — if any callback wedges the executor the
    pets stop and systemd restarts the hub (WatchdogSec in nano-app.service). No-op
    outside systemd. (Deliberately duplicated in each hub main: ~10 dependency-free
    lines beat a cross-package util import.)"""
    path = os.environ.get("NOTIFY_SOCKET")
    if not path:
        return
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            s.sendto(msg.encode(), "\0" + path[1:] if path.startswith("@") else path)
        finally:
            s.close()
    except OSError:
        pass


def _disable_default_qos_event_callbacks():
    """rclpy Humble attaches a default incompatible-QoS warning waitable to EVERY
    publisher/subscription (PublisherEventCallbacks/SubscriptionEventCallbacks
    default to use_default_callbacks=True) — one more entity the executor's
    per-spin wait-set rebuild has to iterate, across ~200 pubs+subs stack-wide.
    This stack is single-vendor with fixed QoS profiles, so the warning is never
    actionable; force it off process-wide by patching the (keyword-only)
    constructors before any node is built. (Deliberately duplicated in each hub
    main, same rationale as _sd_notify.)"""
    from rclpy import qos_event

    def _patch(cls):
        orig_init = cls.__init__

        def _init(self, *, use_default_callbacks=True, **kwargs):
            orig_init(self, use_default_callbacks=False, **kwargs)

        cls.__init__ = _init

    _patch(qos_event.SubscriptionEventCallbacks)
    _patch(qos_event.PublisherEventCallbacks)


def main():
    rclpy.init()
    _disable_default_qos_event_callbacks()
    nodes = []
    for cls in NODE_CLASSES:
        try:
            nodes.append(cls())
        except Exception as exc:   # one node failing to construct shouldn't sink the rest
            print(f"[app_hub] {cls.__name__} init failed: {exc}", flush=True)

    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    print(f"[app_hub] hosting {len(nodes)} nodes: "
          f"{', '.join(n.get_name() for n in nodes)}", flush=True)
    _sd_notify("READY=1")
    if nodes:                       # executor-liveness watchdog pet (see _sd_notify)
        nodes[0].create_timer(5.0, lambda: _sd_notify("WATCHDOG=1"))

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            ex.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            try:
                if isinstance(n, DisplayNode):
                    n.shutdown_sequence()   # end-screen (restart/shutdown glyph)
                n.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
