"""Browser gateway: ONE Server-Sent-Events telemetry stream + whitelisted POST control.

This replaces rosbridge entirely. rosbridge cost ~a full core with the web UI open
(rclpy builds a Python message per *incoming* sample, per topic, plus per-client JSON +
websocket framing — see the sbc-cpu-profile memory), and everything heavy had already
been moved off it (/scan + /map via /dev/shm, teleop via POST /drive, TTS/LLM via HTTP).
What was left were ~35 light topics. This module serves those from web_server itself:

  * `GET /telemetry` — an SSE stream of one compact JSON frame at `telemetry_rate` Hz
    (default 5). The frame is built ONCE per tick and fanned out to every connected
    browser, so N viewers cost one JSON dump. The browser's native EventSource
    auto-reconnects across stack restarts.
  * `POST /publish {topic, value…}` — publish on a WHITELISTED topic with a hard
    clamp/validation per topic (goal, LDS setpoint, pickup override, OLED owners, …).
    Same philosophy as the skills action tier: the page can never publish anything
    the whitelist doesn't spell out.
  * `POST /param {node, name, value}` — set a WHITELISTED parameter on a whitelisted
    node via its /<node>/set_parameters service (the web tuning sliders). Fire-and-
    forget like the old roslib call (the page never used the response).

Idle cost is ~zero: the topic subscriptions are created only while a browser is
connected (and torn down after `SUB_LINGER` with none), and the frame builder early-outs
when there are no clients. Subscription create/destroy happens on the executor thread
(inside the tick timer) so it never races the spin loop.
"""
import json
import math
import threading
import time

from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Bool, Int8, Int32, Float32, Int64MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from diagnostic_msgs.msg import DiagnosticArray

SUB_LINGER = 15.0        # s to keep the browser-only subscriptions after the last client
# Optical virtual bumper (GPU vision Tier-B extension): commanded-to-move but the GPU's
# frame-diff score stays under a floor for a confirm window -> likely a wheel stall/slip
# (expected optical flow from ego-motion isn't happening). Informational only for now --
# nothing yet acts on this, it's surfaced in /telemetry + the web UI. Its three
# thresholds are live web_control PARAMS now (vision_bumper_cmd_eps/motion_floor/
# confirm_secs, declared in web_server.py, live-tunable via the Sensors panel), not
# fixed constants -- see _optical_bumper below.
# Cheap GPU-vision alert signals (2026-07-12 batch): each pairs one raw GpuVision
# scalar with a live web_control PARAM threshold, computed in _vision_alerts (same
# "read params live, not fixed constants" pattern as the optical bumper) -- so every
# alert's threshold is a web UI slider from day one, not a hardcoded guess. All are
# informational only so far; nothing autonomous acts on them yet.
PLAN_MAX_PTS = 64        # planned-path polyline is downsampled to at most this many points
LDS_RPM_MAX = 400.0      # clamp on the /lds_target_rpm setpoint a browser may publish
SCHEDULE_MAX_ENTRIES = 20  # cap on the scheduled-routines list a browser may set
STALE = -1e9

# ---- POST /param whitelist: node -> settable parameter names -------------------
PARAM_WHITELIST = {
    "imu_driver": {"publish_rate"},
    "lds_driver": {"publish_rate"},
    "wheel_odometry": {"publish_rate"},
    "slam_nav": {"enable_motion", "auto_explore", "max_lin", "max_ang", "stop_distance",
                 "robot_radius", "stuck_timeout", "relocalize", "pickup_pause",
                 "lds_idle_enable", "lds_idle_timeout", "lds_idle_rpm", "lds_active_rpm"},
    "sys_monitor": {"fan_override", "fan_temp_min"},
    "web_control": {"vision_dark_reflex_enable", "vision_dark_threshold", "vision_dark_recover",
                    "vision_bumper_cmd_eps", "vision_bumper_motion_floor", "vision_bumper_confirm_secs",
                    "vision_obstruction_var_max", "vision_obstruction_dark_max",
                    "vision_clutter_alert", "vision_overhead_alert", "vision_focus_blur_max",
                    "vision_backlit_delta_min", "vision_highlight_alert", "vision_looming_alert",
                    "vision_colorcast_alert", "vision_motiontarget_match_max",
                    "vision_novelty_alert", "vision_camera_stall_secs",
                    "vision_vibration_ratio", "vision_vibration_confirm_secs",
                    "vision_glare_derate", "vision_approach_rate", "vision_approach_band"},
}


class TelemetryHub:
    """Owns the browser-facing ROS surface of web_server: lazy telemetry subscriptions,
    the per-tick SSE frame, and the whitelisted publish/param endpoints."""

    def __init__(self, node, rate=5.0):
        self._node = node
        self._period = 1.0 / max(0.5, float(rate))
        self._cond = threading.Condition()
        self._clients = 0
        self._last_client_at = STALE
        self._seq = 0
        self._frame = b"{}"
        self._subs = []               # browser-only subscriptions (live only with clients)

        # --- latest-value stores written by the lazy subscriptions -------------
        self._odom = None             # (x, y, yaw_rad)
        self._plan = []               # [[x, y], ...] downsampled
        self._diag = ({}, STALE)      # ({key: value}, arrival monotonic)
        self._ticks = None            # (l, r)
        self._tick_cnt = 0            # wheel_ticks messages seen (for the rate readout)
        self._tick_win = (0, time.monotonic())
        self._tick_hz = 0.0
        self._hb = (None, STALE)      # (/esp32_heartbeat counter, arrival)
        self._esp_temp = (None, STALE)
        self._hall = None
        self._lds = {}                # rpm / hz / duty
        self._fan = None
        self._selftest = ""
        self._purpose = self._task = self._experiments = self._schedule = ""  # latched JSON
        self._oled = {"face": "", "word": "", "brand": "", "system": ""}
        self._cmd_vel = (0.0, 0.0)     # (linear.x, angular.z), for the optical bumper
        self._low_motion_since = None  # monotonic ts the stall condition started, or None
        # Vibration/looseness diagnostic state: a slow EMA of edge_density while NOT
        # commanded to move (the scene's "how sharp does this room normally look"
        # baseline), and when the driving-but-much-blurrier condition started.
        self._edge_still_ema = None
        self._vibration_since = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._latched_qos = latched
        # --- POST /publish whitelist: topic -> (publisher, msg builder) --------
        # Publishers are created up front (before spin starts — thread-safe) and are
        # cheap; the OLED face publisher is shared with the node's cognition path.
        pub = node.create_publisher
        self._pubs = {
            "/goal_pose": (pub(PoseStamped, "goal_pose", 5), self._mk_goal),
            "/lds_target_rpm": (pub(Float32, "lds_target_rpm", 5), self._mk_lds_rpm),
            "/pickup_override": (pub(Int8, "pickup_override", latched), self._mk_pickup),
            "/selftest": (pub(Bool, "selftest", 5), self._mk_bool),
            "/slam_nav/go_home": (pub(Bool, "slam_nav/go_home", 5), self._mk_bool),
            "/slam_nav/save_map": (pub(Bool, "slam_nav/save_map", 5), self._mk_bool),
            "/oled_face": (node._face_pub, self._mk_face),
            "/oled_text": (pub(String, "oled_text", 5), self._mk_text),
            "/oled_dashboard": (pub(Bool, "oled_dashboard", 5), self._mk_bool),
            "/oled_show_words": (pub(Bool, "oled_show_words", 5), self._mk_bool),
            # Scheduled routines: replace the whole schedule (mood_node validates/parses the
            # HH:MM + skill entries, persists them, and echoes the normalized result back on
            # the latched /schedule topic below — see behavior.brain.Schedule).
            "/schedule_edit": (pub(String, "schedule_edit", 5), self._mk_schedule),
        }
        # --- POST /param: one SetParameters client per whitelisted node --------
        self._param_clients = {
            n: node.create_client(SetParameters, f"/{n}/set_parameters")
            for n in PARAM_WHITELIST
        }
        # One always-on timer: builds/notifies frames while clients exist, manages the
        # lazy subscriptions, and is a single cheap early-out when nobody's watching.
        node.create_timer(self._period, self._tick)

    # ---- client lifecycle (called from HTTP handler threads) -------------------
    def add_client(self):
        with self._cond:
            self._clients += 1
            self._last_client_at = time.monotonic()

    def remove_client(self):
        with self._cond:
            self._clients = max(0, self._clients - 1)
            self._last_client_at = time.monotonic()

    def wait_frame(self, last_seq, timeout=5.0):
        """Block until a frame newer than last_seq exists (or timeout). Returns
        (seq, frame_bytes); an unchanged seq means 'send a keepalive comment'."""
        with self._cond:
            if self._seq == last_seq:
                self._cond.wait(timeout)
            return self._seq, self._frame

    # ---- the per-tick frame (executor thread) ----------------------------------
    def _tick(self):
        with self._cond:
            clients = self._clients
        if clients <= 0:
            if self._subs and (time.monotonic() - self._last_client_at) > SUB_LINGER:
                self._drop_subs()
            return
        if not self._subs:
            self._make_subs()
        frame = json.dumps(self._build(), separators=(",", ":")).encode()
        with self._cond:
            self._seq += 1
            self._frame = frame
            self._cond.notify_all()

    def _optical_bumper(self, now, motion_score):
        """Optical virtual bumper (GPU vision Tier-B): commanded to move, but the GPU's
        frame-diff score has stayed under the noise floor for `vision_bumper_confirm_secs`
        -> likely a wheel stall/slip, since ego-motion should otherwise produce visible
        optical flow. Purely informational -- nothing acts on this yet, it's surfaced in
        /telemetry + the web UI only. Reads its three thresholds LIVE from web_control's
        params (not fixed constants) so the web UI's sliders actually take effect --
        same pattern as _dark_reflex_tick's vision_dark_* params. Returns a dict (not
        just the alert bool) so the UI can show WHY it's clear -- "always clear" usually
        just means "not currently commanded to move," not that the reflex is broken;
        without visibility into the commanded /cmd_vel there was no way to tell the two
        apart, which is the whole reason this got richer."""
        g = self._node.get_parameter
        cmd_eps = g("vision_bumper_cmd_eps").value
        motion_floor = g("vision_bumper_motion_floor").value
        confirm_secs = g("vision_bumper_confirm_secs").value
        lin, ang = self._cmd_vel
        commanded = abs(lin) > cmd_eps or abs(ang) > cmd_eps
        if not commanded or motion_score >= motion_floor:
            self._low_motion_since = None
            return {"alert": False, "commanded": commanded,
                    "cmd_vel": [round(lin, 3), round(ang, 3)], "low_motion_secs": 0.0}
        if self._low_motion_since is None:
            self._low_motion_since = now
        held = now - self._low_motion_since
        return {"alert": held >= confirm_secs, "commanded": commanded,
                "cmd_vel": [round(lin, 3), round(ang, 3)], "low_motion_secs": round(held, 2)}

    def _vibration_alert(self, now, gv):
        """Vibration/looseness diagnostic: while driving, the image should stay roughly
        as sharp as the room normally looks -- excess motion blur (edge_density far
        below the standing-still baseline, held for a confirm window) indicates chassis
        vibration (loose screw, wheel imbalance, worn caster). A maintenance flag, not
        a stop. The baseline is a slow EMA sampled only while NOT commanded to move, so
        it tracks lighting/scene changes without the drive itself polluting it."""
        g = self._node.get_parameter
        lin, ang = self._cmd_vel
        moving = abs(lin) > g("vision_bumper_cmd_eps").value or \
            abs(ang) > g("vision_bumper_cmd_eps").value
        ed = gv.edge_density
        if not moving:
            self._edge_still_ema = (ed if self._edge_still_ema is None
                                    else self._edge_still_ema + 0.05 * (ed - self._edge_still_ema))
            self._vibration_since = None
            return False
        base = self._edge_still_ema
        # No trustworthy baseline (never stood still yet, or a blank-wall scene with no
        # texture to lose) -> can't tell blur from nothing-to-see; stay quiet.
        if base is None or base < 0.02 or ed >= base * g("vision_vibration_ratio").value:
            self._vibration_since = None
            return False
        if self._vibration_since is None:
            self._vibration_since = now
        return (now - self._vibration_since) >= g("vision_vibration_confirm_secs").value

    def _vision_alerts(self, gv, now=None, frozen=False):
        """Turn GpuVision's raw scalar properties into ALERT booleans against LIVE
        web_control params (not fixed constants), same pattern as _optical_bumper --
        the web UI's sliders actually take effect immediately, no restart needed. Kept
        in telemetry.py rather than gpu_vision.py so tuning never touches the GL thread.
        `frozen` (camera master switch off) suppresses the stateful, time-based
        alerts -- a deliberately stopped capture thread would otherwise read as a
        "frozen camera", and a stale edge_density as vibration."""
        g = self._node.get_parameter
        now = time.monotonic() if now is None else now
        luma = gv.luma
        obstructed = (gv.luma_variance < g("vision_obstruction_var_max").value
                      and luma < g("vision_obstruction_dark_max").value)
        clutter = gv.edge_density > g("vision_clutter_alert").value
        overhead = gv.overhead_edge_density > g("vision_overhead_alert").value
        # focus_blur additionally requires decent light -- otherwise it's redundant
        # with `obstructed` (a dark, low-edge-density frame is already covered there).
        focus_blur = (gv.edge_density < g("vision_focus_blur_max").value and luma > 0.1)
        # backlit additionally requires a dim-ish overall scene -- a bright highlight in
        # an already-bright frame isn't "backlit," it's just a normally lit room.
        backlit = ((gv.luma_max - luma) > g("vision_backlit_delta_min").value and luma < 0.5)
        shiny = gv.highlight_fraction > g("vision_highlight_alert").value
        looming = gv.motion_intercept_rate > g("vision_looming_alert").value
        cast = gv.color_cast
        colorcast = bool(cast) and (max(cast) - min(cast)) > g("vision_colorcast_alert").value
        match = gv.motion_target_match
        motion_matches_target = match is not None and match < g("vision_motiontarget_match_max").value
        novel = gv.novelty > g("vision_novelty_alert").value
        # Camera-freeze diagnostic: reads still "succeed" but the device stopped
        # delivering (frame_age growing) OR keeps handing back the identical buffer
        # (an exactly-zero diff for a while -- see GpuVision.zero_motion_secs). Means
        # "recover the camera", where the optical bumper's low-but-nonzero-motion case
        # means "the wheels stalled" -- same-looking numbers, different consumer.
        if frozen:
            camera_freeze = vibration = False
            self._vibration_since = None
        else:
            stall = g("vision_camera_stall_secs").value
            age = gv.frame_age
            camera_freeze = ((age is not None and age > stall)
                             or gv.zero_motion_secs > stall)
            vibration = self._vibration_alert(now, gv)
        return {
            "obstructed": obstructed, "clutter": clutter, "overhead_alert": overhead,
            "focus_blur": focus_blur, "backlit": backlit, "shiny": shiny, "looming": looming,
            "colorcast": colorcast, "motion_matches_target": motion_matches_target,
            "novelty": novel, "camera_freeze": camera_freeze, "vibration": vibration,
        }

    def _build(self):
        n = self._node
        now = time.monotonic()
        # wheel-tick message rate over a ~1 s window (the page's "ticks Hz" readout)
        wc, wt = self._tick_win
        if now - wt >= 1.0:
            self._tick_hz = (self._tick_cnt - wc) / (now - wt)
            self._tick_win = (self._tick_cnt, now)
        diag, diag_at = self._diag
        hb, hb_at = self._hb
        esp_temp, esp_temp_at = self._esp_temp
        vitals = n.vitals()               # IMU motion/tilt/rate from the /dev/shm blob
        f = {
            "susp": [n._susp_l, n._susp_r],
            "pickup_override": n._susp_override,
            "esp": {"hb": hb, "hb_age": round(now - hb_at, 2),
                    "temp": esp_temp, "temp_age": round(now - esp_temp_at, 2),
                    "hall": self._hall, "ticks": self._ticks,
                    "tick_hz": round(self._tick_hz, 1)},
            "lds": self._lds,
            "oled": self._oled,
        }
        # IMU summary + tilt ride the vitals blob (same keys the page always used);
        # omitted entirely when sys_monitor isn't writing — the page shows "lost".
        for k in ("imu", "eul"):
            sec = vitals.get(k)
            if isinstance(sec, dict) and sec.get("age") is not None:
                f[k] = sec
        if self._odom:
            f["odom"] = [round(v, 3) for v in self._odom]
        if self._plan:
            f["plan"] = self._plan
        if diag:
            f["diag"] = diag
            f["diag_age"] = round(now - diag_at, 2)
        if self._fan is not None:
            f["fan"] = self._fan
        if self._selftest:
            f["selftest"] = self._selftest
        gv = getattr(n, "_gpu_vision", None)
        camera_enabled = not bool(getattr(n, "_camera_disabled", False))
        if gv is not None:
            # Plain thread-safe Python properties, not a ROS topic -- no subscription
            # needed, just read on each tick (gpu_vision.py runs continuously regardless
            # of telemetry clients, so this is never stale) -- UNLESS the master
            # camera-disable switch has stopped GpuVision's capture thread entirely
            # (see WebServerNode.set_camera_enable), in which case these properties are
            # frozen at their last value before the stop, not live. Still report them
            # (harmless, and lets the UI show "last known" state) but `camera_enabled`
            # lets the page grey them out / label them stale instead of implying
            # they're updating.
            frozen = not camera_enabled
            target = gv.target
            motion_center = gv.motion_center
            motion_score = gv.motion_score
            bumper = ({"alert": False, "commanded": False, "cmd_vel": [0.0, 0.0], "low_motion_secs": 0.0}
                      if frozen else self._optical_bumper(now, motion_score))
            blob_threshold, blob_min, blob_max = gv.blob_tuning
            match = gv.motion_target_match
            frame_age = gv.frame_age
            f["vision"] = {
                "camera_enabled": camera_enabled,
                "target_name": getattr(n, "_vision_target_active", None),
                "approach": bool(getattr(n, "_vision_approach", False)),
                "oled_mask": bool(getattr(n, "_oled_mask_on", False)),
                "novelty": round(gv.novelty, 3),
                "frame_age": round(frame_age, 2) if frame_age is not None else None,
                "motion": round(motion_score, 3),
                "motion_center": [round(v, 3) for v in motion_center] if motion_center else None,
                "target": [round(v, 3) for v in target] if target else None,
                "has_target_color": gv.has_target_color,
                "blob_tuning": [round(blob_threshold, 3), round(blob_min, 3), round(blob_max, 3)],
                "intercept_rate": round(gv.intercept_rate, 3),
                "motion_intercept_rate": round(gv.motion_intercept_rate, 3),
                "motion_target_match": round(match, 3) if match is not None else None,
                "luma": round(gv.luma, 3),
                "luma_variance": round(gv.luma_variance, 2),
                "luma_max": round(gv.luma_max, 3),
                "color_cast": [round(v, 3) for v in gv.color_cast] if gv.color_cast else None,
                "edge_density": round(gv.edge_density, 3),
                "overhead_edge_density": round(gv.overhead_edge_density, 3),
                "highlight_fraction": round(gv.highlight_fraction, 3),
                "gpu_duty": round(gv.gpu_duty, 3),
                "alerts": self._vision_alerts(gv, now=now, frozen=frozen),
                "bumper": bumper,
            }
        # latched brain readouts, passed through as the raw JSON strings the page parses
        for k, v in (("purpose", self._purpose), ("task", self._task),
                     ("experiments", self._experiments), ("schedule", self._schedule)):
            if v:
                f[k] = v
        return f

    # ---- lazy browser-only subscriptions (created/destroyed on the executor) ---
    def _make_subs(self):
        n, s = self._node, self._subs.append
        sub = n.create_subscription
        s(sub(Odometry, "odom", self._on_odom, 5))
        s(sub(Path, "plan", self._on_plan, 2))
        s(sub(DiagnosticArray, "diagnostics", self._on_diag, 2))
        s(sub(Int64MultiArray, "wheel_ticks", self._on_ticks, 5))
        s(sub(Int32, "esp32_heartbeat", self._on_hb, 2))
        s(sub(Float32, "esp32_temp", self._on_esp_temp, 2))
        s(sub(Int32, "esp32_hall", self._on_hall, 2))
        s(sub(Float32, "lds_rpm", self._mk_lds("rpm"), 2))
        s(sub(Float32, "lds_hz", self._mk_lds("hz"), 2))
        s(sub(Float32, "lds_duty", self._mk_lds("duty"), 2))
        s(sub(Float32, "fan_pwm", self._on_fan, 2))
        s(sub(String, "selftest_result", self._on_selftest, 2))
        s(sub(Twist, "cmd_vel", self._on_cmd_vel, 5))   # optical virtual bumper correlation
        # OLED mirror inputs (the page renders a client-side copy of the panel)
        s(sub(String, "oled_face", self._mk_oled("face"), 5))
        s(sub(String, "oled_word", self._mk_oled("word"), 5))
        s(sub(String, "oled_text", self._mk_oled("brand"), 5))
        s(sub(String, "oled_system", self._mk_oled("system"), 5))
        # latched brain readouts — the latch is re-delivered on (re)subscribe
        s(sub(String, "purpose", self._mk_str("_purpose"), self._latched_qos))
        s(sub(String, "task_current", self._mk_str("_task"), self._latched_qos))
        s(sub(String, "experiments", self._mk_str("_experiments"), self._latched_qos))
        s(sub(String, "schedule", self._mk_str("_schedule"), self._latched_qos))
        self._node.get_logger().info("telemetry: browser connected — subscriptions up")

    def _drop_subs(self):
        for sub in self._subs:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._subs = []
        self._node.get_logger().info("telemetry: no browsers — subscriptions dropped")

    # ---- subscription callbacks (store the latest value, nothing else) ---------
    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self._odom = (p.x, p.y, yaw)

    def _on_plan(self, msg):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
               for p in msg.poses]
        if len(pts) > PLAN_MAX_PTS:                      # keep ends, thin the middle
            step = (len(pts) - 1) / (PLAN_MAX_PTS - 1)
            pts = [pts[int(i * step)] for i in range(PLAN_MAX_PTS - 1)] + [pts[-1]]
        self._plan = pts

    def _on_diag(self, msg):
        st = next((s for s in msg.status if s.name == "system"), None)
        if st is not None:
            self._diag = ({p.key: p.value for p in st.values}, time.monotonic())

    def _on_ticks(self, msg):
        d = list(msg.data)
        if len(d) >= 2:
            self._ticks = (d[0], d[1])
        self._tick_cnt += 1

    def _on_hb(self, msg):
        self._hb = (msg.data, time.monotonic())

    def _on_esp_temp(self, msg):
        self._esp_temp = (round(msg.data, 1), time.monotonic())

    def _on_hall(self, msg):
        self._hall = msg.data

    def _mk_lds(self, key):
        def cb(msg):
            self._lds[key] = round(msg.data, 3)
        return cb

    def _on_fan(self, msg):
        self._fan = round(msg.data, 3)

    def _on_selftest(self, msg):
        self._selftest = msg.data

    def _on_cmd_vel(self, msg):
        self._cmd_vel = (msg.linear.x, msg.angular.z)

    def _mk_oled(self, key):
        def cb(msg):
            self._oled[key] = msg.data
        return cb

    def _mk_str(self, attr):
        def cb(msg):
            setattr(self, attr, msg.data)
        return cb

    # ---- POST /publish ----------------------------------------------------------
    def publish_json(self, data):
        """Publish `value` on the whitelisted `topic`. Every topic has its own
        validator/clamp; anything else is refused."""
        topic = str((data or {}).get("topic") or "").strip()
        entry = self._pubs.get(topic)
        if entry is None:
            return {"error": "topic not whitelisted: " + (topic or "(none)")}
        pub, build = entry
        try:
            msg = build(data.get("value"))
        except (TypeError, ValueError, KeyError) as exc:
            return {"error": f"bad value: {exc}"}
        if msg is None:
            return {"error": "bad value"}
        pub.publish(msg)
        return {"status": "ok", "topic": topic}

    @staticmethod
    def _mk_goal(v):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x = float(v["x"])
        m.pose.position.y = float(v["y"])
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _mk_lds_rpm(v):
        return Float32(data=min(LDS_RPM_MAX, max(0.0, float(v))))

    @staticmethod
    def _mk_pickup(v):
        v = int(v)
        return Int8(data=v) if v in (-1, 0, 1) else None

    @staticmethod
    def _mk_bool(v):
        return Bool(data=bool(v))

    @staticmethod
    def _mk_face(v):
        s = str(v or "")[:40]
        return String(data=s)

    @staticmethod
    def _mk_text(v):
        return String(data=str(v or "")[:32])

    @staticmethod
    def _mk_schedule(v):
        """Light shape-check only — the real HH:MM/skill-name parsing (and dropping
        malformed entries) is mood_node's Schedule, which echoes the normalized result
        back on the latched /schedule topic."""
        if not isinstance(v, list) or len(v) > SCHEDULE_MAX_ENTRIES:
            raise ValueError(f"expected a list of at most {SCHEDULE_MAX_ENTRIES} entries")
        entries = []
        for e in v:
            if not isinstance(e, dict):
                raise ValueError("each entry must be an object")
            entries.append({"time": str(e.get("time") or "")[:8],
                            "skill": str(e.get("skill") or "")[:64]})
        return String(data=json.dumps(entries))

    # ---- POST /param --------------------------------------------------------------
    def set_param_json(self, data):
        """Set one whitelisted parameter on a whitelisted node. Fire-and-forget: the
        service call completes on the executor; the page never consumed the reply."""
        node = str((data or {}).get("node") or "").strip()
        name = str((data or {}).get("name") or "").strip()
        allowed = PARAM_WHITELIST.get(node)
        if not allowed or name not in allowed:
            return {"error": f"param not whitelisted: {node}/{name}"}
        value = data.get("value")
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        else:
            try:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(value)
            except (TypeError, ValueError):
                return {"error": "bad value"}
        client = self._param_clients[node]
        if not client.service_is_ready():
            return {"error": f"{node} not reachable"}
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=pv)]
        client.call_async(req)
        return {"status": "sent", "node": node, "name": name}
