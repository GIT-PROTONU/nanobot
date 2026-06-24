"""Tiny static-file HTTP server for the control page, plus an MJPEG webcam stream.

Kept as a ROS node so it starts/stops with the rest of the launch and shows up
in `ros2 node list`. It serves the package's installed `web/` directory (which
contains index.html). The page talks to ROS over the rosbridge websocket, not to
this server — this only delivers the HTML/JS and the camera stream.

`/stream.mjpg` serves the USB webcam as multipart/x-mixed-replace, fed by a
zero-dependency V4L2 MJPEG passthrough (see mjpeg_camera). `/audio.pcm` streams
the webcam mic as raw PCM via arecord (see mic_audio). Both the camera and the
mic are started only while a client is connected, so they cost nothing idle.

`POST /tts` ({"voice","text"}) speaks a line via SVOX Pico (see tts) and streams
its words to the OLED on /oled_word so they appear/disappear as they're said.
"""
import functools
import http.server
import json
import os
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Int32, String

from .mjpeg_camera import CameraStream
from .mic_audio import AudioStream
from .tts import TtsEngine, VOICES, clamp

# Persisted, web-tunable TTS settings (merged over the file on disk). `voice` is
# seeded from the tts_default_voice param at load time.
SETTINGS_DEFAULTS = {
    "voice": "en-US",
    "volume": 100,            # Pico level %, 100 = normal
    "speed": 100,
    "pitch": 100,
    "announce": False,        # speak CPU/RAM/temp every `announce_interval` s
    "announce_interval": 30,  # seconds (clamped to >= ANNOUNCE_MIN)
}
ANNOUNCE_MIN = 5              # don't let the announcer spam faster than this (s)
ANNOUNCE_MAX = 3600

# SBC vitals for the spoken stats (same cheap /proc + thermal reads the OLED uses).
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"
STAT_PATH = "/proc/stat"
MEMINFO_PATH = "/proc/meminfo"


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
        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_pico_bin", "pico2wave")  # from PATH (deploy/install-picotts.sh)
        self.declare_parameter("tts_device", "")       # aplay -D target; "" = ALSA default
        self.declare_parameter("tts_default_voice", "en-US")
        # Where the live TTS settings (voice/volume/speed/pitch + stats-announcer) are
        # persisted so they survive an SBC reboot. "" = XDG state dir under $HOME.
        self.declare_parameter("tts_settings_path", "")
        g = self.get_parameter
        port = g("web_port").value

        self._cam = CameraStream(
            dev=g("cam_device").value or None,
            width=g("cam_width").value, height=g("cam_height").value,
            fps=g("cam_fps").value, logger=self.get_logger().info)

        self._mic = AudioStream(
            device=g("mic_device").value or None,
            rate=g("mic_rate").value, channels=1, logger=self.get_logger().info)

        # TTS publishes one word at a time to the OLED (it shows them karaoke-style),
        # blanking with "" at the end so the panel returns to the dashboard.
        self._word_pub = self.create_publisher(String, "oled_word", 10)
        self._tts = TtsEngine(
            pico_bin=g("tts_pico_bin").value, device=g("tts_device").value or None,
            default_voice=g("tts_default_voice").value, enabled=g("tts_enabled").value,
            on_word=lambda w: self._word_pub.publish(String(data=w)),
            logger=self.get_logger().info)

        # Persisted live settings (voice/volume/speed/pitch + periodic stats announcer).
        # Loaded from disk so they survive a reboot, then pushed into the engine. The
        # announce schedule is driven off the existing 1 Hz ping timer (see
        # _publish_ping) — no extra timer/wakeup — and a disabled announcer is a single
        # dict lookup, so it keeps working with the node even after every browser closes.
        self._cpu_prev = None                          # (idle, total) jiffies for CPU%
        self._settings = self._load_settings()
        self._apply_engine()
        self._cpu_prev = self._cpu_sample()
        self._announce_next = time.monotonic() + float(self._settings["announce_interval"])

        web_dir = os.path.join(get_package_share_directory("web_control"), "web")
        handler = functools.partial(_Handler, directory=web_dir, stream=self._cam,
                                    audio=self._mic, tts=self._tts, node=self)
        self._httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"control page at http://0.0.0.0:{port}  (serving {web_dir})")

        # Liveness ping for the ESP32 coprocessor. The ESP joins the zenoh graph over a raw
        # UART that can't detect the peer vanishing, so if the router/SBC restarts it would
        # keep publishing into the void until a manual reset. This always-on 1 Hz heartbeat
        # lets the firmware reboot itself when the pings stop (see LINK_RX_TIMEOUT_MS in
        # firmware/nanobot_coprocessor/src/main.cpp). Independent of any browser being open.
        self._ping_pub = self.create_publisher(Int32, "esp32_ping", 10)
        self._ping_seq = 0
        self.create_timer(1.0, self._publish_ping)

    def _publish_ping(self):
        self._ping_seq = (self._ping_seq + 1) & 0x7FFFFFFF
        self._ping_pub.publish(Int32(data=self._ping_seq))
        self._announce_tick()                          # piggy-backs on this 1 Hz tick

    # ---- persisted TTS settings ---------------------------------------------
    def _settings_file(self):
        p = self.get_parameter("tts_settings_path").value
        return p or os.path.expanduser("~/.local/state/nanobot/tts.json")

    def _load_settings(self):
        s = dict(SETTINGS_DEFAULTS)
        s["voice"] = self.get_parameter("tts_default_voice").value or "en-US"
        try:
            with open(self._settings_file()) as f:
                saved = json.load(f)
            s.update({k: v for k, v in saved.items() if k in SETTINGS_DEFAULTS})
        except Exception:
            pass                                       # no/invalid file -> defaults
        return _sanitize_settings(s)

    def _save_settings(self):
        try:
            path = self._settings_file()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self._settings, f)
            os.replace(tmp, path)                      # atomic; never a half-written file
        except Exception as exc:
            self.get_logger().warning(f"tts: could not persist settings ({exc})")

    def _apply_engine(self):
        """Push the current voice + markup levels into the TTS engine."""
        s = self._settings
        self._tts.configure(voice=s["voice"], volume=s["volume"],
                            speed=s["speed"], pitch=s["pitch"])

    def get_settings(self):
        return dict(self._settings)

    def update_settings(self, data):
        """Merge a partial settings dict from the web UI, persist, and apply."""
        old = self._settings
        s = dict(old)
        for k in SETTINGS_DEFAULTS:
            if k in data:
                s[k] = data[k]
        self._settings = _sanitize_settings(s)
        self._save_settings()
        self._apply_engine()
        # (Re)arm the announcer ONLY when it was just enabled or its interval changed,
        # so adjusting an unrelated slider can't keep postponing the next announcement.
        if self._settings["announce"] and (
                not old["announce"]
                or self._settings["announce_interval"] != old["announce_interval"]):
            self._cpu_prev = self._cpu_sample()
            self._announce_next = time.monotonic() + float(self._settings["announce_interval"])
        return self._settings

    # ---- periodic spoken system stats ---------------------------------------
    def _announce_tick(self):
        # Cheapest checks first (this runs every second): a disabled announcer is one
        # dict lookup. available() (and the /proc reads) only happen at announce time.
        if not self._settings["announce"]:
            return
        now = time.monotonic()
        if now < self._announce_next:
            return
        self._announce_next = now + float(self._settings["announce_interval"])
        self.announce_now()

    def announce_now(self):
        if not self._tts.available():
            return
        text = self._compose_stats(self._cpu_percent(), self._mem_percent(),
                                   self._cpu_temp())
        if text:
            self._tts.say(text)

    def _compose_stats(self, cpu, mem, temp):
        de = self._settings["voice"].startswith("de")
        parts = []
        if cpu == cpu:                                 # not NaN
            parts.append(f"Prozessor {cpu:.0f} Prozent" if de else f"C P U {cpu:.0f} percent")
        if mem == mem:
            parts.append(f"Arbeitsspeicher {mem:.0f} Prozent" if de else f"RAM {mem:.0f} percent")
        if temp == temp:
            parts.append(f"Temperatur {temp:.0f} Grad" if de else f"Temperature {temp:.0f} degrees")
        if not parts:
            return "Keine Daten" if de else "No data"
        return ". ".join(parts)

    def _cpu_sample(self):
        try:
            with open(STAT_PATH) as f:
                parts = [int(x) for x in f.readline().split()[1:]]
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)  # idle + iowait
            return idle, sum(parts)
        except Exception:
            return None

    def _cpu_percent(self):
        """Busy % since the previous sample (the gap between announcements)."""
        cur = self._cpu_sample()
        pct = float("nan")
        if cur and self._cpu_prev:
            di, dt = cur[0] - self._cpu_prev[0], cur[1] - self._cpu_prev[1]
            if dt > 0:
                pct = 100.0 * (1.0 - di / dt)
        self._cpu_prev = cur
        return pct

    def _mem_percent(self):
        try:
            tot = avail = 0
            with open(MEMINFO_PATH) as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        tot = int(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        avail = int(line.split()[1])
                    if tot and avail:
                        break
            return 100.0 * (tot - avail) / tot if tot else float("nan")
        except Exception:
            return float("nan")

    def _cpu_temp(self):
        try:
            with open(THERMAL_PATH) as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            return float("nan")

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def _sanitize_settings(s):
    """Coerce/clamp a settings dict to safe types + ranges (UI is untrusted)."""
    out = dict(SETTINGS_DEFAULTS)
    out.update(s)
    out["voice"] = out["voice"] if out["voice"] in VOICES else SETTINGS_DEFAULTS["voice"]
    out["volume"] = clamp(out["volume"], 0, 500)
    out["speed"] = clamp(out["speed"], 20, 500)
    out["pitch"] = clamp(out["pitch"], 50, 200)
    out["announce"] = bool(out["announce"])
    out["announce_interval"] = clamp(out["announce_interval"], ANNOUNCE_MIN, ANNOUNCE_MAX)
    return out


class _Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, stream=None, audio=None, tts=None, node=None, **kwargs):
        self._stream = stream
        self._audio = audio
        self._tts = tts
        self._node = node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/tts/config":
            # Current persisted settings, so the page can restore its controls on load.
            return self._respond_json(self._node.get_settings() if self._node else {})
        if path == "/stream.mjpg":
            return self._stream_mjpeg()
        if path == "/audio.pcm":
            return self._stream_audio()
        if path == "/map":
            return self._serve_map()
        if path == "/scan.bin":
            return self._serve_scan()
        return super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/tts":
            # Speak a line and karaoke its words to the OLED. Body: {"text","voice"?}.
            data = self._read_json()
            text = (data.get("text") or "").strip()
            voice = (data.get("voice") or "").strip() or None
            if self._tts is None or not self._tts.available():
                self._respond(503, "tts unavailable")
            elif not text:
                self._respond(400, "empty text")
            else:
                self._tts.say(text, voice=voice)
                self._respond(200, "speaking")
        elif path == "/tts/config":
            # Update + persist live settings (voice/volume/speed/pitch/announce/interval).
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.update_settings(self._read_json()))
        elif path == "/tts/announce":
            # Speak the system stats once, right now (independent of the periodic toggle).
            if self._node is None or self._tts is None or not self._tts.available():
                self._respond(503, "tts unavailable")
            else:
                self._node.announce_now()
                self._respond(200, "announcing")
        elif path == "/tts/stop":
            if self._tts is not None:
                self._tts.stop()
            self._respond(200, "stopped")
        elif path == "/system/restart":
            # Restart the whole ROS stack. Detached + new session so it survives
            # do_down killing this very web server, then do_up brings it back.
            self._set_oled_action("restart")   # tells the OLED to show "Restarting stack"
            self._run_detached(
                'cd "$HOME/Nano" && "$HOME/.pixi/bin/pixi" run bash scripts/stack.sh restart')
            self._respond(200, "restarting stack")
        elif path == "/system/reboot":
            # Reboot the whole SBC (needs the scoped NOPASSWD sudo rule for systemctl).
            self._set_oled_action("reboot")    # tells the OLED to show "Restarting"
            self._run_detached("sudo -n /usr/bin/systemctl reboot")
            self._respond(200, "rebooting")
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

    def _read_json(self):
        """Parse the request body as JSON; {} on any problem (length-bounded)."""
        try:
            n = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(min(n, 8192)) if n > 0 else b""
            return json.loads(raw or b"{}")
        except Exception:
            return {}

    def _respond(self, code, msg):
        body = msg.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _respond_json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
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
        self._serve_shm("/dev/shm/nano_map.bin", "no map yet")

    def _serve_scan(self):
        # The lidar driver writes each scan as a compact blob to /dev/shm (JSON header +
        # raw float32 ranges); the page polls it here instead of bridging the heavy
        # /scan LaserScan over rosbridge. Same idea as the map — keeps rosbridge light.
        self._serve_shm("/dev/shm/nano_scan.bin", "no scan yet")

    def _serve_shm(self, path, missing_msg):
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(503, missing_msg)
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
