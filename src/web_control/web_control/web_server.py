"""Tiny static-file HTTP server for the control page, plus an MJPEG webcam stream.

Kept as a ROS node so it starts/stops with the rest of the launch and shows up
in `ros2 node list`. It serves the package's installed `web/` directory (which
contains index.html). The page talks to ROS over the rosbridge websocket, not to
this server — this only delivers the HTML/JS and the camera stream.

`/stream.mjpg` serves the USB webcam as multipart/x-mixed-replace, fed by a
zero-dependency V4L2 MJPEG passthrough (see mjpeg_camera). The camera is opened
only while a client is connected, so it costs nothing when nobody is watching.
"""
import functools
import http.server
import os
import subprocess
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .mjpeg_camera import CameraStream


class WebServerNode(Node):
    def __init__(self):
        super().__init__("web_control")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("rosbridge_port", 9090)
        self.declare_parameter("cam_device", "")      # "" = auto-detect the UVC cam
        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 480)
        self.declare_parameter("cam_fps", 15)
        g = self.get_parameter
        port = g("web_port").value

        self._cam = CameraStream(
            dev=g("cam_device").value or None,
            width=g("cam_width").value, height=g("cam_height").value,
            fps=g("cam_fps").value, logger=self.get_logger().info)

        web_dir = os.path.join(get_package_share_directory("web_control"), "web")
        handler = functools.partial(_Handler, directory=web_dir, stream=self._cam)
        self._httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"control page at http://0.0.0.0:{port}  (serving {web_dir})")

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


class _Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, stream=None, **kwargs):
        self._stream = stream
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/stream.mjpg":
            return self._stream_mjpeg()
        return super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/system/restart":
            # Restart the whole ROS stack. Detached + new session so it survives
            # do_down killing this very web server, then do_up brings it back.
            self._run_detached(
                'cd "$HOME/Nano" && "$HOME/.pixi/bin/pixi" run bash scripts/stack.sh restart')
            self._respond(200, "restarting stack")
        elif path == "/system/shutdown":
            # Power off the SBC (needs the scoped NOPASSWD sudo rule for systemctl).
            self._run_detached("sudo -n /usr/bin/systemctl poweroff")
            self._respond(200, "shutting down")
        else:
            self.send_error(404)

    @staticmethod
    def _run_detached(cmd):
        # 1 s delay lets the HTTP response flush before the action runs.
        subprocess.Popen(["bash", "-lc", "sleep 1; " + cmd],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)

    def _respond(self, code, msg):
        body = msg.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream_mjpeg(self):
        if self._stream is None:
            self.send_error(503, "no camera")
            return
        self._stream.add_viewer()
        try:
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            seq = 0
            while True:
                seq, jpeg = self._stream.get_frame(seq, timeout=5.0)
                if jpeg is None:
                    if not self._stream.running():
                        break          # camera failed / no device
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass                       # client closed the stream
        finally:
            self._stream.remove_viewer()

    def log_message(self, *args):      # silence per-request stderr spam
        pass


def main():
    rclpy.init()
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
