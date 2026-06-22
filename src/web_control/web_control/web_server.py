"""Tiny static-file HTTP server for the control page, plus an MJPEG webcam stream.

Kept as a ROS node so it starts/stops with the rest of the launch and shows up
in `ros2 node list`. It serves the package's installed `web/` directory (which
contains index.html). The page talks to ROS over the rosbridge websocket, not to
this server — this only delivers the HTML/JS and the camera stream.

`/stream.mjpg` serves the USB webcam as multipart/x-mixed-replace, fed by a
zero-dependency V4L2 MJPEG passthrough (see mjpeg_camera). `/audio.pcm` streams
the webcam mic as raw PCM via arecord (see mic_audio). Both the camera and the
mic are started only while a client is connected, so they cost nothing idle.
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
from .mic_audio import AudioStream


class WebServerNode(Node):
    def __init__(self):
        super().__init__("web_control")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("rosbridge_port", 9090)
        self.declare_parameter("cam_device", "")      # "" = auto-detect the UVC cam
        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 480)
        self.declare_parameter("cam_fps", 15)
        self.declare_parameter("mic_device", "")       # "" = auto-detect USB mic
        self.declare_parameter("mic_rate", 16000)      # Hz; 16k mono = 32 KB/s
        g = self.get_parameter
        port = g("web_port").value

        self._cam = CameraStream(
            dev=g("cam_device").value or None,
            width=g("cam_width").value, height=g("cam_height").value,
            fps=g("cam_fps").value, logger=self.get_logger().info)

        self._mic = AudioStream(
            device=g("mic_device").value or None,
            rate=g("mic_rate").value, channels=1, logger=self.get_logger().info)

        web_dir = os.path.join(get_package_share_directory("web_control"), "web")
        handler = functools.partial(_Handler, directory=web_dir,
                                    stream=self._cam, audio=self._mic)
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
    def __init__(self, *args, stream=None, audio=None, **kwargs):
        self._stream = stream
        self._audio = audio
        super().__init__(*args, **kwargs)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/stream.mjpg":
            return self._stream_mjpeg()
        if path == "/audio.pcm":
            return self._stream_audio()
        if path == "/map":
            return self._serve_map()
        return super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/system/restart":
            # Restart the whole ROS stack. Detached + new session so it survives
            # do_down killing this very web server, then do_up brings it back.
            self._set_oled_action("restart")   # tells the OLED to show "Restarting"
            self._run_detached(
                'cd "$HOME/Nano" && "$HOME/.pixi/bin/pixi" run bash scripts/stack.sh restart')
            self._respond(200, "restarting stack")
        elif path == "/system/shutdown":
            # Power off the SBC (needs the scoped NOPASSWD sudo rule for systemctl).
            self._set_oled_action("shutdown")  # tells the OLED to show "Shutting down" + go dark
            self._run_detached("sudo -n /usr/bin/systemctl poweroff")
            self._respond(200, "shutting down")
        else:
            self.send_error(404)

    @staticmethod
    def _set_oled_action(action):
        # Hint the OLED node (read in its SIGTERM shutdown sequence) which end-screen to
        # show. Written synchronously here so it exists before the (delayed) stop runs.
        try:
            with open("/dev/shm/nano_oled_action", "w") as f:
                f.write(action)
        except Exception:
            pass

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

    def _stream_audio(self):
        if self._audio is None:
            self.send_error(503, "no microphone")
            return
        q = self._audio.add_listener()
        try:
            # Stream as HTTP/1.1 chunked. Browsers buffer an HTTP/1.0 (close-
            # delimited) streaming body and never hand it to fetch()'s reader until
            # the connection closes — which for a live mic is never — so without
            # chunked the page would receive nothing. Chunked is surfaced live.
            self.protocol_version = "HTTP/1.1"
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            # raw signed 16-bit little-endian PCM; rate/channels in headers so the
            # browser can configure the Web Audio decoder without hardcoding.
            self.send_header("Content-Type", "audio/L16;rate=%d;channels=%d"
                             % (self._audio.rate, self._audio.channels))
            self.send_header("X-Sample-Rate", str(self._audio.rate))
            self.send_header("X-Channels", str(self._audio.channels))
            self.send_header("Transfer-Encoding", "chunked")
            self.end_headers()
            import queue as _q
            while True:
                try:
                    data = q.get(timeout=5.0)
                except _q.Empty:
                    if not self._audio.running():
                        break          # mic failed / no device
                    continue
                # one HTTP chunk: <hex len>CRLF <data> CRLF, flushed immediately
                self.wfile.write(b"%X\r\n" % len(data))
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            self.wfile.write(b"0\r\n\r\n")
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass                       # client stopped listening
        finally:
            self._audio.remove_listener(q)

    def _serve_map(self):
        # The slam_nav node writes the live occupancy map to a RAM file (/dev/shm);
        # we just hand the bytes over same-origin so the page's map canvas can render
        # them. No ROS subscription / OccupancyGrid serialization in this process.
        try:
            with open("/dev/shm/nano_map.bin", "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(503, "no map yet")
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

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
