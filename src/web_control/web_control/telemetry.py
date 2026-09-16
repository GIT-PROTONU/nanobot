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
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Bool, Int8, Int32, Float32, Int32MultiArray, Int64MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, OccupancyGrid
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import MagneticField
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import Buffer as TfBuffer, TransformListener

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
LDS_RPM_MAX = 400.0      # clamp on the /lds_target_rpm setpoint a browser may publish
MOTOR_ACCEL_MIN = 0.3    # clamp on the /motor_accel ramp rate (duty/s) -- matches the
MOTOR_ACCEL_MAX = 8.0    # ESP32 firmware's own MOTOR_SLEW_MIN/MAX clamp (main.cpp)
TRIM_MAX = 0.30          # ESP32 firmware's TRIM_MAX -- |wheel_trim| rebalance range (main.cpp)
GOAL_MAX_ABS_M = 12.0    # clamp on /goal_pose x/y -- Nav2's global costmap is
                         # 24x24 m; a goal outside it would just fail to plan
# Keep-away bubble drawn around the robot on the web map (metres) -- mirrors
# local_costmap/global_costmap inflation_radius in config/nav2/nav2_params.yaml.
# Deliberately hardcoded (NOT a startup get_parameters call): that service could
# race the Nav2 lifecycle at boot, and the value is restart-only anyway. If you
# tune inflation_radius in nav2_params.yaml, change this to match.
NAV_INFLATION_M = 0.25
# action_msgs/GoalStatus code -> the web map's status chip word. The status
# sub is the LAST entry of /navigate_to_pose/_action/status (one entry per goal
# bt_navigator knows, appended chronologically).
NAV_STATUS = {
    0: "idle",        # STATUS_UNKNOWN
    1: "planning",    # STATUS_ACCEPTED
    2: "navigating",  # STATUS_EXECUTING
    3: "canceling",   # STATUS_CANCELING
    4: "arrived",     # STATUS_SUCCEEDED
    5: "idle",        # STATUS_CANCELED (goal gone)
    6: "failed",      # STATUS_ABORTED (bt recovery gave up)
}
SCHEDULE_MAX_ENTRIES = 20  # cap on the scheduled-routines list a browser may set
STALE = -1e9

# ---- POST /param whitelist: node -> settable parameter names -------------------
PARAM_WHITELIST = {
    "imu_driver": {"publish_rate", "euler_rate", "offset_x_mm", "offset_y_mm", "offset_z_mm",
                   "mount_roll_deg", "mount_pitch_deg", "mount_yaw_deg", "bandwidth_hz"},
    "lds_driver": {"publish_rate"},
    "wheel_odometry": {"publish_rate"},
    "sys_monitor": {"fan_override", "fan_temp_min", "fan_min_duty", "fan_smooth_alpha"},
    "web_control": {"vision_dark_reflex_enable", "vision_dark_threshold", "vision_dark_recover",
                    "vision_bumper_cmd_eps", "vision_bumper_motion_floor", "vision_bumper_confirm_secs",
                    "vision_obstruction_var_max", "vision_obstruction_dark_max",
                    "vision_clutter_alert", "vision_overhead_alert", "vision_focus_blur_max",
                    "vision_backlit_delta_min", "vision_highlight_alert", "vision_looming_alert",
                    "vision_colorcast_alert", "vision_motiontarget_match_max",
                    "vision_novelty_alert", "vision_camera_stall_secs",
                    "vision_vibration_ratio", "vision_vibration_confirm_secs",
                    "vision_glare_derate", "vision_approach_rate", "vision_approach_band",
                    "imu_drift_min_secs"},
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
        self._diag = ({}, STALE)      # ({key: value}, arrival monotonic)
        self._pipe_diag = None    # (feed dict, arrival, level, message) or None
        self._ticks = None            # (l, r)
        self._stray = None            # (l, r) ticks seen while stopped -- bad-encoder-signal diagnostic
        self._tick_cnt = 0            # wheel_ticks messages seen (for the rate readout)
        self._tick_win = (0, time.monotonic())
        self._tick_hz = 0.0
        self._hb = (None, STALE)      # (/esp32_heartbeat counter, arrival)
        self._esp_temp = (None, STALE)
        self._hall = None
        self._wheel_trim = None     # live straight-line trim from the ESP32 (/wheel_trim)
        self._lds = {}                # rpm / hz / duty
        self._fan = None
        self._mag = None              # (x, y, z) raw counts, for eyeballing IMU cal quality
        self._eul = None              # (roll, pitch, yaw deg, arrival monotonic) -- direct
                                       # /imu/euler sub, NOT the 1 Hz vitals blob (see _on_eul)
        self._drift_base = None       # (r0, p0, y0, start monotonic) while stationary, else None
        self._drift_last = ""         # latched summary of the last completed still period
        self._imu_cal_status = ""     # latched, from imu_driver's calibration routine
        self._imu_mount_settings = ""  # latched JSON, from imu_driver's effective mount offset/rotation
        self._purpose = self._task = self._experiments = self._schedule = ""  # latched JSON
        self._oled = {"face": "", "word": "", "brand": "", "system": ""}
        self._cmd_vel = (0.0, 0.0)     # (linear.x, angular.z), for the optical bumper
        self._low_motion_since = None  # monotonic ts the stall condition started, or None
        # Vibration/looseness diagnostic state: a slow EMA of edge_density while NOT
        # commanded to move (the scene's "how sharp does this room normally look"
        # baseline), and when the driving-but-much-blurrier condition started.
        self._edge_still_ema = None
        self._vibration_since = None
        # --- Nav2 map view state (web map panel; see index.html's map IIFE) -----
        # Latest /map OccupancyGrid: the grid is polled by the browser over the
        # /map HTTP route (NOT the SSE frame — ~230 KB would dwarf the rest of
        # the frame). slam_toolbox publishes every map_update_interval (5 s), so
        # one cached copy is all a 1 Hz poll can consume.
        self._map_payload = None       # atomic (meta{w,h,res,ox,oy,t}, raw int8 cells);
                                       #   one tuple so /map can never pair a new header
                                       #   with the previous grid
        self._map_arrival = STALE      # monotonic, for staleness surfacing
        # map-frame pose via TF (map->odom from slam_toolbox + odom->base_link
        # from wheel_odometry). Listener is lazy — created with the other
        # browser-only subs, unregistered in _drop_subs.
        self._tf_buf = None
        self._tf_listener = None
        # Goal mirror + Nav2 action status for the web chip. _goal tracks the
        # last published /goal_pose (browser clicks, locations, skills — all go
        # through publish_json); _goal_status comes from the action status sub.
        self._goal = None              # [x, y] in the map frame, or None
        self._goal_status = "idle"

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
            "/reset_ticks": (pub(Bool, "reset_ticks", 5), self._mk_bool),
            # ESP32 motor accel-ramp rate (duty/s) -- see main.cpp's MOTOR_SLEW_DEFAULT.
            "/motor_accel": (pub(Float32, "motor_accel", 5), self._mk_motor_accel),
            # ESP32 straight-line wheel trim (-0.3..0.3): negative = boost left / cut right
            # (robot veers left), positive = the opposite. Live-tunes the open-loop trim that
            # rebalances the mismatched gearmotors in main.cpp's applyMotors(); persisted to NVS.
            "/motor_trim": (pub(Float32, "motor_trim", 5), self._mk_motor_trim),
            # ESP32 line lasers 1-2 (GPIO 23/32): [v1,v2] PWM 0..255 each.
            "/laser_pwm": (pub(Int32MultiArray, "laser_pwm", 5), self._mk_laser),
            "/oled_face": (node._face_pub, self._mk_face),
            "/oled_text": (pub(String, "oled_text", 5), self._mk_text),
            "/oled_dashboard": (pub(Bool, "oled_dashboard", 5), self._mk_bool),
            "/oled_show_words": (pub(Bool, "oled_show_words", 5), self._mk_bool),
            # WitMotion 5-byte hex calibration protocol, executed in imu_driver's
            # reader thread — see ImuNode._do_calibrate.
            "/imu_calibrate": (pub(String, "imu_calibrate", 5), self._mk_calibrate),
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

    def _vibration_alert(self, now, edge_density):
        """Vibration/looseness diagnostic: while driving, the image should stay roughly
        as sharp as the room normally looks -- excess motion blur (edge_density far
        below the standing-still baseline, held for a confirm window) indicates chassis
        vibration (loose screw, wheel imbalance, worn caster). A maintenance flag, not
        a stop. The baseline is a slow EMA sampled only while NOT commanded to move, so
        it tracks lighting/scene changes without the drive itself polluting it."""
        g = self._node.get_parameter
        lin, ang = self._cmd_vel
        eps = g("vision_bumper_cmd_eps").value
        moving = abs(lin) > eps or abs(ang) > eps
        ed = edge_density
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

    def _drift_yaw_deg(self):
        """Heading source for the yaw-drift numbers: the raw /imu/euler yaw
        (degrees). The robot_localization EKF this used to prefer is GONE with
        the slam_nav migration (docs/nav2-migration.md), so the device's own
        fused yaw is the only heading source again — remember (2026-08-11
        finding) that it develops a decaying bias transient for a minute+ after
        any real motion, so a fresh post-drive "drift" reading can be the
        transient, not real drift. Wait a minute after moving before trusting
        the numbers below."""
        return self._eul[2]

    def _imu_drift_tick(self, now):
        """IMU drift check: while the robot is provably stationary (not commanded to
        move, same eps as the bumper/vibration checks above, AND both wheels
        grounded -- not mid pick-up), the reported roll/pitch/yaw shouldn't change at
        all. Any change IS the drift -- gyro bias or magnetometer interference (see
        the selftest-spin-imu-mismatch investigation), not real motion. Purely
        observational, like the other alerts here: nothing acts on it, it just
        answers "is my IMU trustworthy at rest" from the web UI.
        Roll/pitch come from the raw /imu/euler (accel-corrected, no transient);
        yaw comes from the EKF heading (see _drift_yaw_deg) so the post-drive
        device-fused-yaw transient can't light this up red.
        Returns the live in-progress reading (zeros while moving) plus `last`, a
        latched one-line summary of the most recently completed still period long
        enough to mean anything (>= imu_drift_min_secs)."""
        n = self._node
        g = n.get_parameter
        lin, ang = self._cmd_vel
        eps = g("vision_bumper_cmd_eps").value
        commanded = abs(lin) > eps or abs(ang) > eps
        grounded = not any(n._susp_eff())
        still = grounded and not commanded and self._eul is not None
        if not still:
            if self._drift_base is not None:
                self._latch_drift(now, g("imu_drift_min_secs").value)
            self._drift_base = None
            return {"still_s": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                    "yaw_per_min": 0.0, "last": self._drift_last}
        r, p, _, _ = self._eul
        y = self._drift_yaw_deg()
        if self._drift_base is None:
            self._drift_base = (r, p, y, now)
        dr, dp, dy, dur = self._drift_since(r, p, y, now)
        return {"still_s": round(dur, 1), "roll": round(dr, 2), "pitch": round(dp, 2),
                "yaw": round(dy, 2),
                "yaw_per_min": round(dy / dur * 60.0, 2) if dur > 0.5 else 0.0,
                "last": self._drift_last}

    def _drift_since(self, r, p, y, now):
        r0, p0, y0, t0 = self._drift_base
        dy = ((y - y0 + 180.0) % 360.0) - 180.0     # wrap-aware (yaw is -180..180)
        return r - r0, p - p0, dy, now - t0

    def _latch_drift(self, now, min_secs):
        r, p, _, _ = self._eul
        y = self._drift_yaw_deg()
        dr, dp, dy, dur = self._drift_since(r, p, y, now)
        if dur < min_secs:
            return                # too brief to mean anything -- don't overwrite the last real reading
        rate = dy / dur * 60.0 if dur > 0.5 else 0.0
        self._drift_last = (f"{dur:.0f}s stationary: yaw {dy:+.2f}° ({rate:+.2f}°/min), "
                             f"roll {dr:+.2f}°, pitch {dp:+.2f}°")

    def _vision_alerts(self, sc, now=None, frozen=False):
        """Turn GpuVision's raw scalar properties into ALERT booleans against LIVE
        web_control params (not fixed constants), same pattern as _optical_bumper --
        the web UI's sliders actually take effect immediately, no restart needed. Kept
        in telemetry.py rather than gpu_vision.py so tuning never touches the GL thread.
        `sc` is the one-per-tick snapshot dict of GpuVision scalars pre-read in _build,
        so this alert pass doesn't re-acquire the GpuVision lock ~13x per frame.
        `frozen` (camera master switch off) suppresses the stateful, time-based
        alerts -- a deliberately stopped capture thread would otherwise read as a
        "frozen camera", and a stale edge_density as vibration."""
        g = self._node.get_parameter
        now = time.monotonic() if now is None else now
        luma = sc["luma"]
        obstructed = (sc["luma_variance"] < g("vision_obstruction_var_max").value
                      and luma < g("vision_obstruction_dark_max").value)
        clutter = sc["edge_density"] > g("vision_clutter_alert").value
        overhead = sc["overhead_edge_density"] > g("vision_overhead_alert").value
        # focus_blur additionally requires decent light -- otherwise it's redundant
        # with `obstructed` (a dark, low-edge-density frame is already covered there).
        focus_blur = (sc["edge_density"] < g("vision_focus_blur_max").value and luma > 0.1)
        # backlit additionally requires a dim-ish overall scene -- a bright highlight in
        # an already-bright frame isn't "backlit," it's just a normally lit room.
        backlit = ((sc["luma_max"] - luma) > g("vision_backlit_delta_min").value and luma < 0.5)
        shiny = sc["highlight_fraction"] > g("vision_highlight_alert").value
        looming = sc["motion_intercept_rate"] > g("vision_looming_alert").value
        cast = sc["color_cast"]
        colorcast = bool(cast) and (max(cast) - min(cast)) > g("vision_colorcast_alert").value
        match = sc["motion_target_match"]
        motion_matches_target = match is not None and match < g("vision_motiontarget_match_max").value
        novel = sc["novelty"] > g("vision_novelty_alert").value
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
            age = sc["frame_age"]
            camera_freeze = ((age is not None and age > stall)
                             or sc["zero_motion_secs"] > stall)
            vibration = self._vibration_alert(now, sc["edge_density"])
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
            "esp": {"hb": hb, "hb_age": round(now - hb_at, 2) if hb is not None else None,
                    "temp": esp_temp, "temp_age": round(now - esp_temp_at, 2)
                    if esp_temp is not None else None,
                    "hall": self._hall, "ticks": self._ticks,
                    "stray": self._stray,
                    "tick_hz": round(self._tick_hz, 1),
                    "wheel_trim": self._wheel_trim},
            "lds": self._lds,
            "oled": self._oled,
        }
        # IMU |accel|/|gyro| summary rides the vitals blob (numeric labels only, 1 Hz
        # is plenty); omitted entirely when sys_monitor isn't writing — the page shows
        # "lost". eul is its OWN direct sub (self._eul, not the blob) — the 3D
        # orientation view needs it fresh every frame, and the blob only updates once
        # a second regardless of imu_driver's rate (see _on_eul).
        sec = vitals.get("imu")
        if isinstance(sec, dict) and sec.get("age") is not None:
            f["imu"] = sec
        if self._eul is not None:
            r, p, y, at = self._eul
            f["eul"] = {"r": r, "p": p, "y": y, "age": round(now - at, 2)}
        f["imu_drift"] = self._imu_drift_tick(now)
        if self._odom:
            f["odom"] = [round(v, 3) for v in self._odom]
        if diag:
            f["diag"] = diag
            f["diag_age"] = round(now - diag_at, 2)
        if self._pipe_diag is not None:
            pd, pd_at, lvl, msg = self._pipe_diag
            if now - pd_at < 10.0:           # stale pipeline diag = stale source
                f["pipeline"] = {"feeds": pd, "level": lvl, "msg": msg,
                                 "age": round(now - pd_at, 2)}
        if self._fan is not None:
            f["fan"] = self._fan
        if self._mag is not None:
            f["imuMag"] = list(self._mag)
        if self._imu_cal_status:
            f["imuCalStatus"] = self._imu_cal_status
        if self._imu_mount_settings:
            f["imuMountSettings"] = self._imu_mount_settings
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
            sc = gv.snapshot()         # one lock acquisition for all ~20 readouts
            target = sc["target"]
            motion_center = sc["motion_center"]
            motion_score = sc["motion_score"]
            bumper = ({"alert": False, "commanded": False, "cmd_vel": [0.0, 0.0], "low_motion_secs": 0.0}
                      if frozen else self._optical_bumper(now, motion_score))
            blob_threshold, blob_min, blob_max = sc["blob_tuning"]
            match = sc["motion_target_match"]
            frame_age = sc["frame_age"]
            novelty = sc["novelty"]
            intercept_rate = sc["intercept_rate"]
            motion_intercept_rate = sc["motion_intercept_rate"]
            luma = sc["luma"]
            luma_variance = sc["luma_variance"]
            luma_max = sc["luma_max"]
            color_cast = sc["color_cast"]
            edge_density = sc["edge_density"]
            overhead_edge_density = sc["overhead_edge_density"]
            highlight_fraction = sc["highlight_fraction"]
            has_target_color = sc["has_target_color"]
            gpu_duty = sc["gpu_duty"]
            f["vision"] = {
                "camera_enabled": camera_enabled,
                "target_name": getattr(n, "_vision_target_active", None),
                "approach": bool(getattr(n, "_vision_approach", False)),
                "oled_mask": bool(getattr(n, "_oled_mask_on", False)),
                "novelty": round(novelty, 3),
                "frame_age": round(frame_age, 2) if frame_age is not None else None,
                "motion": round(motion_score, 3),
                "motion_center": [round(v, 3) for v in motion_center] if motion_center else None,
                "target": [round(v, 3) for v in target] if target else None,
                "has_target_color": has_target_color,
                "blob_tuning": [round(blob_threshold, 3), round(blob_min, 3), round(blob_max, 3)],
                "intercept_rate": round(intercept_rate, 3),
                "motion_intercept_rate": round(motion_intercept_rate, 3),
                "motion_target_match": round(match, 3) if match is not None else None,
                "luma": round(luma, 3),
                "luma_variance": round(luma_variance, 2),
                "luma_max": round(luma_max, 3),
                "color_cast": [round(v, 3) for v in color_cast] if color_cast else None,
                "edge_density": round(edge_density, 3),
                "overhead_edge_density": round(overhead_edge_density, 3),
                "highlight_fraction": round(highlight_fraction, 3),
                "gpu_duty": round(gpu_duty, 3),
                "alerts": self._vision_alerts(
                    {"luma": luma, "luma_variance": luma_variance, "luma_max": luma_max,
                     "edge_density": edge_density, "overhead_edge_density": overhead_edge_density,
                     "highlight_fraction": highlight_fraction,
                     "motion_intercept_rate": motion_intercept_rate, "color_cast": color_cast,
                     "motion_target_match": match, "novelty": novelty, "frame_age": frame_age,
                     "zero_motion_secs": sc["zero_motion_secs"]},
                     now=now, frozen=frozen),
                "bumper": bumper,
            }
        # Nav2 map view: pose (TF map->base_link), goal mirror + action status +
        # the inflation bubble radius. All tiny; the heavy map grid itself is
        # served by the /map HTTP route, never this frame.
        pose = self._tf_pose()
        f["nav"] = {
            "pose": [round(v, 3) for v in pose] if pose else None,
            "goal": self._goal,
            "status": self._goal_status,
            "inflation": NAV_INFLATION_M,
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
        s(sub(DiagnosticArray, "diagnostics", self._on_diag, 2))
        s(sub(Int64MultiArray, "wheel_ticks", self._on_ticks, 5))
        s(sub(Int64MultiArray, "wheel_stray_ticks", self._on_stray, 5))
        s(sub(Int32, "esp32_heartbeat", self._on_hb, 2))
        s(sub(Float32, "esp32_temp", self._on_esp_temp, 2))
        s(sub(Int32, "esp32_hall", self._on_hall, 2))
        s(sub(Float32, "wheel_trim", self._on_wheel_trim, 2))
        s(sub(Float32, "lds_rpm", self._mk_lds("rpm"), 2))
        s(sub(Float32, "lds_hz", self._mk_lds("hz"), 2))
        s(sub(Float32, "lds_duty", self._mk_lds("duty"), 2))
        s(sub(Float32, "fan_pwm", self._on_fan, 2))
        s(sub(MagneticField, "imu/mag", self._on_mag, 2))
        # Direct sub (not the 1 Hz vitals blob) -- the 3D orientation view needs eul
        # fresh every frame, and sys_monitor's own /imu/euler sub only feeds its once-
        # a-second blob write, so routing through it silently capped the browser to
        # ~1 Hz updates no matter how fast imu_driver itself published.
        s(sub(Vector3Stamped, "imu/euler", self._on_eul, 5))
        s(sub(String, "imu_calibrate_status", self._mk_str("_imu_cal_status"), self._latched_qos))
        s(sub(String, "imu_mount_settings", self._mk_str("_imu_mount_settings"), self._latched_qos))
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
        # --- Nav2 map view ----------------------------------------------------
        # slam_toolbox's latched map: transient-local so a browser that opens the
        # Map view gets the current grid immediately (instead of waiting up to
        # map_update_interval). The OccupancyGrid is cached once (see _on_map);
        # the browser polls it over the /map HTTP route.
        s(sub(OccupancyGrid, "map", self._on_map, self._latched_qos))
        # bt_navigator's goal status: the state machine behind the web chip
        # (idle/planning/navigating/arrived/failed). Tiny messages, and empty
        # between goals.
        s(sub(GoalStatusArray, "navigate_to_pose/_action/status", self._on_goal_status, 5))
        # map->base_link TF (map->odom: slam_toolbox @10 Hz; odom->base_link:
        # wheel_odometry) — the pose dot + "save current spot". tf subs are kept
        # OUT of self._subs (TransformListener.unregister handles teardown).
        if self._tf_listener is None:
            self._tf_buf = TfBuffer()
            self._tf_listener = TransformListener(self._tf_buf, n)
        self._node.get_logger().info("telemetry: browser connected — subscriptions up")

    def _drop_subs(self):
        for sub in self._subs:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._subs = []
        if self._tf_listener is not None:
            try:
                self._tf_listener.unregister()   # destroys its /tf + /tf_static subs
            except Exception:
                pass
            self._tf_listener = None
            self._tf_buf = None
        self._node.get_logger().info("telemetry: no browsers — subscriptions dropped")

    # ---- subscription callbacks (store the latest value, nothing else) ---------
    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self._odom = (p.x, p.y, yaw)

    def _on_diag(self, msg):
        st = next((s for s in msg.status if s.name == "system"), None)
        if st is not None:
            self._diag = ({p.key: p.value for p in st.values}, time.monotonic())
        # localization-pipeline feed status (odom/EKF/imu) from sys_monitor
        pipe = next((s for s in msg.status if s.name == "pipeline"), None)
        if pipe is not None:
            # DiagnosticStatus.level can arrive as a raw byte under rmw_zenoh
            # (b'\x00'|b'\x01'|b'\x02'...) rather than a plain int -- normalize it
            # here so it survives json.dumps in the telemetry frame.
            lvl = pipe.level
            if isinstance(lvl, bytes):
                lvl = int.from_bytes(lvl, byteorder="little") if lvl else 0
            self._pipe_diag = (
                {p.key: p.value for p in pipe.values}, time.monotonic(),
                lvl, pipe.message)

    def _on_ticks(self, msg):
        d = list(msg.data)
        if len(d) >= 2:
            self._ticks = (d[0], d[1])
        self._tick_cnt += 1

    def _on_stray(self, msg):
        d = list(msg.data)
        if len(d) >= 2:
            self._stray = (d[0], d[1])

    def _on_hb(self, msg):
        self._hb = (msg.data, time.monotonic())

    def _on_esp_temp(self, msg):
        self._esp_temp = (round(msg.data, 1), time.monotonic())

    def _on_hall(self, msg):
        self._hall = msg.data

    def _on_wheel_trim(self, msg):
        self._wheel_trim = round(float(msg.data), 3)

    def _mk_lds(self, key):
        def cb(msg):
            self._lds[key] = round(msg.data, 3)
        return cb

    def _on_fan(self, msg):
        self._fan = round(msg.data, 3)

    def _on_mag(self, msg):
        m = msg.magnetic_field
        self._mag = (round(m.x, 1), round(m.y, 1), round(m.z, 1))

    def _on_eul(self, msg):            # x=roll, y=pitch, z=yaw (deg) -- imu_driver's /imu/euler
        v = msg.vector
        self._eul = (v.x, v.y, v.z, time.monotonic())

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

    # ---- Nav2 map view (see the map IIFE in index.html) -------------------------
    def _on_map(self, msg):
        """Cache the latest /map OccupancyGrid for the /map HTTP route. One copy,
        one memcpy per update (every map_update_interval = 5 s at most); no
        per-tick work. A degenerate (0-sized or truncated) grid is skipped so a
        half-written publish can't poison the browser's renderer."""
        info = msg.info
        data = bytes(msg.data)               # int8[] -> raw bytes (-1..100 mod 256)
        if info.width <= 0 or info.height <= 0 or len(data) != info.width * info.height:
            return
        self._map_payload = ({          # single atomic assignment (see __init__)
            "w": info.width, "h": info.height, "res": round(info.resolution, 6),
            "ox": info.origin.position.x, "oy": info.origin.position.y,
            "t": time.time(),
        }, data)
        self._map_arrival = time.monotonic()

    def _on_goal_status(self, msg):
        """Track bt_navigator's action status (the web chip). status_list gains an
        entry per goal state transition; the LAST entry is the current goal. On a
        terminal state the goal mirror is dropped too, so the browser's goal ring
        doesn't resurrect from every subsequent frame."""
        if msg.status_list:
            code = msg.status_list[-1].status
            if isinstance(code, bytes):      # rmw_zenoh int8 paranoia (see _on_diag)
                code = code[0]
            code = int(code)
            self._goal_status = NAV_STATUS.get(code, "idle")
            if code in (4, 5, 6):            # SUCCEEDED / CANCELED / ABORTED
                self._goal = None

    def _tf_pose(self):
        """map-frame pose (x, y, yaw_rad) from TF, or None (slam_toolbox or
        wheel_odometry not up yet / TF stale). lookup_transform with a zero time
        = latest available transform."""
        if self._tf_buf is None:
            return None
        try:
            t = self._tf_buf.lookup_transform("map", "base_link", Time())
            tr, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
            return (tr.x, tr.y, yaw)
        except Exception:                    # lookup/connectivity/extrapolation — all "no pose"
            return None

    def get_map_payload(self):
        """The /map HTTP route body: (meta_dict, int8_bytes), or (None, None)
        until slam_toolbox has published a grid. Atomic: meta and cells always
        come from the same /map message."""
        return self._map_payload or (None, None)

    def clear_goal(self):
        """Drop the goal mirror + chip state (POST /nav/cancel). Nav2's own status
        topic will corroborate with CANCELED/UNKNOWN on the next tick."""
        self._goal = None
        self._goal_status = "idle"

    # ---- POST /publish ----------------------------------------------------------
    def publish_json(self, data):
        """Publish `value` on the whitelisted `topic`. Every topic has its own
        validator/clamp; anything else is refused."""
        topic = str((data or {}).get("topic") or "").strip()
        if topic and not topic.startswith("/"):
            topic = "/" + topic      # tolerate the bare form ("goal_pose") — location_go
                                     # has sent it since aede005 and never matched the key
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
        if topic == "/goal_pose":
            # Goal mirror for the web map (f["nav"].goal). Nav2 will corroborate
            # via the action status topic within a tick or two; set "planning"
            # here so the chip reacts to the click immediately.
            self.note_goal(msg.pose.position.x, msg.pose.position.y)
        # Diagnosability: log map-click goals + LDS rpm etc. so "who told the robot to
        # go there / spin" is in the app log. Throttle the chatty spin-down? No — these
        # are discrete user actions, not a hot loop; every one is a meaningful event.
        if topic in ("/goal_pose", "/reset_ticks", "/laser_pwm"):
            self._node.get_logger().info(f"POST /publish {topic} value={data.get('value')!r}")
        return {"status": "ok", "topic": topic}

    def note_goal(self, x, y):
        """Record a goal published outside POST /publish (skill actions) so the
        web map's goal ring + status chip stay in sync with those too."""
        self._goal = [round(float(x), 3), round(float(y), 3)]
        self._goal_status = "planning"

    @staticmethod
    def _mk_goal(v):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x = min(GOAL_MAX_ABS_M, max(-GOAL_MAX_ABS_M, float(v["x"])))
        m.pose.position.y = min(GOAL_MAX_ABS_M, max(-GOAL_MAX_ABS_M, float(v["y"])))
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _mk_lds_rpm(v):
        return Float32(data=min(LDS_RPM_MAX, max(0.0, float(v))))

    @staticmethod
    def _mk_motor_accel(v):
        return Float32(data=min(MOTOR_ACCEL_MAX, max(MOTOR_ACCEL_MIN, float(v))))

    @staticmethod
    def _mk_motor_trim(v):
        t = float(v)
        if not (-TRIM_MAX <= t <= TRIM_MAX):
            return None          # out of the rebalance range; ignore rather than clamp silently
        return Float32(data=t)

    @staticmethod
    def _mk_pickup(v):
        v = int(v)
        return Int8(data=v) if v in (-1, 0, 1) else None

    @staticmethod
    def _mk_bool(v):
        return Bool(data=bool(v))

    @staticmethod
    def _mk_laser(v):
        if not isinstance(v, (list, tuple)) or len(v) != 2:
            raise ValueError("expected a 2-element list [v1, v2]")
        return Int32MultiArray(data=[min(255, max(0, int(x))) for x in v])

    @staticmethod
    def _mk_face(v):
        s = str(v or "")[:40]
        return String(data=s)

    @staticmethod
    def _mk_text(v):
        return String(data=str(v or "")[:32])

    @staticmethod
    def _mk_calibrate(v):
        v = str(v or "").strip()
        if v not in ("accel", "mag_start", "mag_stop", "save", "axis6", "axis9", "zero_yaw"):
            raise ValueError(f"unknown imu calibrate command: {v}")
        return String(data=v)

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
        # Diagnosability: who/what changed a param (esp. enable_motion) must be in the
        # app log — nav_node's _on_params only logs the transition it receives, so this
        # records the browser-facing side too.
        self._node.get_logger().info(
            f"POST /param {node}/{name} = {value!r} (source: web UI)")
        return {"status": "sent", "node": node, "name": name}
