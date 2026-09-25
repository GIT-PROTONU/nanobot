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
import collections
import json
import math
import threading
import time

from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.parameter import Parameter as RclpyParameter
from std_msgs.msg import Bool, Int8, Int32, Float32, Int32MultiArray, Int64MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
try:
    from nav2_msgs.msg import CostmapFilterInfo
except Exception:                                    # keep telemetry importable without nav2_msgs
    CostmapFilterInfo = None
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import MagneticField
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import Buffer as TfBuffer, TransformListener

try:
    from nav2_msgs.action import NavigateThroughPoses
    NavThroughPosesFeedback = NavigateThroughPoses.Feedback
except Exception:                                    # board without nav2_msgs? never
    NavThroughPosesFeedback = None

SUB_LINGER = 15.0        # s to keep the browser-only subscriptions after the last client
PLAN_MAX_POINTS = 200    # /plan downsample cap (GET /plan polyline stays bounded)
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
# LDS idle spin-down controller (2026-09-21; the old slam_nav-era _update_lds_idle died
# with slam_nav and NOTHING owned /lds_target_rpm afterwards — the ESP32 just held its
# last setpoint, default 300, forever). telemetry.py now owns the topic: spin at the
# user's target while the robot is active (recent commanded /cmd_vel motion or a Nav2
# goal in flight), park it after LDS_IDLE_SECS_DEFAULT of quiet. Runs on an always-on
# 1 Hz timer so it works with the page CLOSED — which is the whole point, and why the
# page no longer re-publishes the slider value on SSE (re)connect (that reconnect
# re-assert used to force-woke the lidar; see the 2026-07-14 memory).
LDS_DEFAULT_RPM = 300.0   # boot spin-when-active target (slider + firmware default)
LDS_IDLE_SECS_DEFAULT = 60.0   # quiet stretch before the spin-down (lds_idle_secs)
LDS_MANUAL_SECS_DEFAULT = 300.0  # a manual topic post holds the topic this long
LDS_IDLE_TICK = 1.0       # controller period (s)
LDS_REASSERT_SECS = 30.0  # re-publish an unchanged setpoint after this long — an ESP32
                          # reboot resets its setpoint to the firmware default, so a
                          # periodic re-assert corrects it without any user action
# Nav2 goal states that count as "the lidar must stay up" for the idle controller.
# "planning" matters: Nav2 needs fresh scans for its costmap BEFORE it starts moving.
NAV_BUSY = ("planning", "navigating", "canceling")
# A busy status is trusted only this long since its last ARRIVAL: the status topic is
# event-driven (bt_navigator publishes on transitions, not periodically), so a long
# goal rides on /cmd_vel keeping the motion clock alive instead. The age bound exists
# for the failure mode where nano-nav dies mid-navigation and the last "navigating"
# status would otherwise freeze the lidar awake forever on a parked robot. During a
# genuinely active goal, cmd_vel refreshes the clock, so this never parks a moving
# robot; a goal silently stuck >90 s with no motion loses only the ~2 s spin-up.
LDS_NAV_STALE = 90.0
# "planning" gets a MUCH longer trust window. Planning is exactly the state where
# the planner needs fresh scans (slam's map→odom TF) BEFORE it can even compute a
# path, and there is NO /cmd_vel during planning to refresh the motion clock —
# so the 90 s window above would park the lidar out from under a slow-waking
# planner (the 2026-09-23 live deadlock: click → "planning" → lidar parked → no
# scans → Nav2's Time(0) map→base_link lookup keeps resolving to the frozen
# map→odom stamp → "extrapolation into the past" → never finishes planning →
# robot never moves). A genuinely dead nano-nav/slam now costs at most 5 min of
# lidar spin instead of silently deadlocking every goal.
LDS_PLANNING_STALE = 300.0
# Goal-click pre-wake (2026-09-23): a goal clicked while the idle controller has the
# lidar parked used to FAIL — the goal went out first, Nav2 started planning against
# slam's frozen map→odom TF, and bt recovery aborted the FIRST goal before the ~2 s
# spin-up finished (a re-click then worked because the lidar was already up). The goal
# publish now fires the spin setpoint immediately and holds (bounded) until /lds_hz
# shows the lidar actually delivering frames, so planning starts against a live TF.
LDS_READY_MIN_HZ = 2.0    # /lds_hz valid-frame rate that counts as "scans flowing"
LDS_READY_AGE = 1.5       # /lds_* readouts must be fresher than this (s) to be trusted
LDS_WAKE_WAIT = 10.0      # max seconds a goal is held while the parked lidar spins up
LDS_WAKE_POLL = 0.25      # readiness poll period while holding
# --- SLAM/Nav event log (the Drive tab's "Nav log" card; GET /nav/log) ------
NAVLOG_MAX = 400          # ring size (oldest dropped)
NAVLOG_WARN_PERIOD = 15.0  # min gap between repeated planning-stuck warnings
PLANNING_WARN_SECS = 10.0  # planning older than this starts warning in the log
MAP_AGE_FRESH = 10.0       # /map younger than this counts as a live slam feed
MOTOR_ACCEL_MIN = 0.3    # clamp on the /motor_accel ramp rate (duty/s) -- matches the
MOTOR_ACCEL_MAX = 8.0    # ESP32 firmware's own MOTOR_SLEW_MIN/MAX clamp (main.cpp)
TRIM_MAX = 0.30          # ESP32 firmware's TRIM_MAX -- |wheel_trim| rebalance range (main.cpp)
GOAL_MAX_ABS_M = 12.0    # clamp on /goal_pose x/y -- Nav2's global costmap is
                         # 24x24 m; a goal outside it would just fail to plan
NAV_WAYPOINT_MAX = 12    # cap on POST /nav/waypoints stops (a crawl-speed tour
                         # of more would outlive any sane watchdog)
KEEPOUT_MAX_ZONES = 16   # cap on persisted keepout rectangles (a mask rasterizes
                         # in O(zones×cells); 16 painted rects is far past the map)
# Fallback for the keep-away bubble drawn around the robot on the web map
# (metres) when web_control's nav_inflation_m param can't be read (used to be
# a hardcoded mirror of local_costmap/global_costmap inflation_radius in
# config/nav2/nav2_params.yaml — now that value is LIVE-tunable from the
# Navigation pace card via /nav/config, and the frame carries the web node's
# current setting each tick).
NAV_INFLATION_M = 0.25
NAV_ROBOT_DIAM_M = 0.32   # fallback robot-circle ⌀ (2× nav2_params.yaml robot_radius 0.16)
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
                    "imu_drift_min_secs",
                    # LDS idle spin-down controller (telemetry.py's _lds_ctrl_tick)
                    "lds_idle_enable", "lds_idle_secs", "lds_manual_secs",
                },
    # velocity_smoother (nav2_container): the nav speed/accel caps. The SCALAR
    # params here are settable via POST /param; the 3-element array params
    # (max_velocity/min_velocity/max_accel/max_decel) need the dedicated
    # GET/POST /nav/config endpoint in web_server (set_param_json sends
    # scalars only). Both routes hit the same dynamically-reconfigurable
    # params — no nano-nav restart needed for either.
    "velocity_smoother": {"smoothing_frequency", "velocity_timeout"},
}


def lds_idle_target(now, last_move_at, idle_secs, idle_enable, nav_busy,
                    manual_active, active_rpm):
    """The /lds_target_rpm setpoint the idle controller wants on the wire right now.

    None = hands off (another owner — the browser slider or a skill — holds the
    topic via the manual latch). A Nav2 goal in flight, `idle_enable` off, or
    commanded motion within the last `idle_secs` keeps the lidar at `active_rpm`;
    a quiet stretch past it parks the motor (0.0 — the firmware's target<=0 branch).
    Pure + unit-tested; see test_lds_idle.py. Note `last_move_at=None` (no commanded
    motion since boot) counts as idle, so a freshly booted idle robot parks the
    lidar instead of leaving the firmware's boot default spinning."""
    if manual_active:
        return None
    if not idle_enable or nav_busy or (
            last_move_at is not None and now - last_move_at < idle_secs):
        return max(0.0, float(active_rpm))
    return 0.0


def lds_nav_busy(status, status_age):
    """Is a Nav2 goal in flight, trusted PER-STATUS? The action status topic is
    event-driven (arrivals only on transitions), so a busy status is trusted only
    within its arrival window: `navigating`/`canceling` for LDS_NAV_STALE (a live
    goal keeps /cmd_vel flowing, which refreshes the motion clock anyway; the age
    bound stops a dead nano-nav from freezing "navigating" forever), but
    "planning" for the much longer LDS_PLANNING_STALE — planning emits no
    follow-up status and no /cmd_vel, yet the planner NEEDS fresh scans (the
    map→odom TF) before it can compute a path. Parking the lidar during planning
    is the "keeps planning, never moves" deadlock. Pure + unit-tested."""
    if status not in NAV_BUSY:
        return False
    return status_age < (LDS_PLANNING_STALE if status == "planning" else LDS_NAV_STALE)


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
        # Cached map->odom (x, y, yaw_rad) — see _tf_pose's "extrap" fallback
        # (persistent web-map pose while the lidar is parked). DISPLAY-ONLY.
        self._map_odom = None
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
        self._wheel_pid = None      # live wheel-PID gains [kp,ki,kd] from the ESP32 (/wheel_pid)
        self._wheel_params = None   # live drivetrain params (id,value) pairs (/wheel_params)
        self._lds = {}                # rpm / hz / duty
        self._lds_at = None           # monotonic ts of the last /lds_* arrival
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
        # Latest Nav2 costmaps, same shape as _map_payload and served the same
        # way (GET /local_costmap + /global_costmap). The local costmap lives in
        # the odom frame, so its origin is re-projected into the map frame at
        # arrival time (see _on_costmap) — the page overlays both with plain
        # map-frame coords. Cells are Nav2 COSTS (0 free .. 254 lethal, 255
        # unknown), not -1/0/100 occupancy — the header carries kind:"costmap"
        # + the grid yaw so the page shades/rotates accordingly.
        self._local_costmap_payload = None
        self._global_costmap_payload = None
        # Latest planned path (planner_server publishes nav_msgs/Path on /plan
        # each replan — 1 Hz with the RateController-wrapped ComputePathToPose).
        # Served by GET /plan (NOT the SSE frame — a plan can be hundreds of
        # poses); downsampled to PLAN_MAX_POINTS at arrival so the browser
        # polyline is bounded regardless of path length.
        self._plan_payload = None      # atomic ({n, t}, [x0, y0, x1, y1, ...])
        self._plan_arrival = STALE
        # Drawable keep-away zones (web Map card → the global costmap's
        # KeepoutFilter): rectangles in map-frame metres, owned by web_server
        # (persisted to keepout.json) and mirrored here for mask rasterization.
        self._keepout_zones = []
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
        self._goal_status_at = STALE   # monotonic ts of the last status arrival — the
                                       # busy-state trust window (LDS_NAV_STALE)
        self._goal_status_since = None  # monotonic ts the CURRENT status value began
                                        # (transition durations for the Nav log)
        self._goal_published_at = None  # monotonic ts of the last goal publish
        # Multi-waypoint progress (NavigateThroughPoses action feedback): the
        # waypoint index the action is currently driving toward + the total.
        # None on a single-goal NavigateToPose (and reset on terminal states).
        self._wp_index = None
        self._wp_total = None
        self._moving = False           # last /cmd_vel above the motion eps — the
                                       # Nav log's motion start/stop transitions
        self._navlog_warn_at = 0.0     # monotonic ts of the last planning-stuck warning
        # --- SLAM/Nav event log (Drive tab "Nav log" card; GET /nav/log) -------
        # A bounded ring of nav-chain events: goal publishes/cancels, action-status
        # transitions with durations, the lidar idle controller's wake/park
        # decisions, motion start/stop, planning-stuck warnings. Served via
        # GET /nav/log?since=<id> (NOT the SSE frame — the frame is a typed
        # contract, and a log is pull-friendly). Every entry is also mirrored to
        # the app log (journald) so a diagnosis never needs the browser.
        self._navlog_seq = 0
        self._navlog = collections.deque(maxlen=NAVLOG_MAX)

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
            # ESP32 wheel-PID live gains [kp, ki, kd] (duty units) — tune the closed-loop
            # wheel velocity controller without reflashing. Firmware clamps to sane ranges,
            # resets its integrators on change, and persists to NVS. Readback on /wheel_pid
            # (f.esp.wheel_pid) re-seeds the web sliders.
            "/motor_pid": (pub(Float32MultiArray, "motor_pid", 5), self._mk_motor_pid),
            # Live drivetrain parameters (id,value) pairs — scale/geometry, NO reflash:
            # 0 ticks_per_rev, 1 wheel_radius_m, 2 wheel_separation_m, 3 max_linear_ms,
            # 4 max_angular_rads, 5 target_slew, 6 dither, 7 vel_hyst.
            # Readback on /wheel_params (f.esp.wheel_params).
            "/motor_params": (pub(Float32MultiArray, "motor_params", 5), self._mk_motor_params),
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
            # Drawable keep-away/no-go zones (web Map card): the keepout mask is
            # an OccupancyGrid in the MAP frame at slam's resolution, rasterized
            # from the persisted rectangles — the global costmap's KeepoutFilter
            # (nav2_params.yaml) reads it and the planner treats painted cells
            # as lethal. Both topics LATCHED (transient-local): the costmap
            # filter subscribes whenever the costmap (re)starts and must get
            # the current mask immediately, browser or no browser.
            "/keepout_mask": (pub(OccupancyGrid, "keepout_mask", latched),
                              self._mk_keepout_mask),
            "/keepout_filter_info": (pub(CostmapFilterInfo, "keepout_filter_info", latched),
                                     self._mk_keepout_info),
        }
        # --- POST /param: one SetParameters client per whitelisted node --------
        self._param_clients = {
            n: node.create_client(SetParameters, f"/{n}/set_parameters")
            for n in PARAM_WHITELIST
        }
        # --- LDS idle spin-down controller state (see lds_idle_target) ---------
        # The remembered spin-when-active rpm: the browser's Spin slider IS this
        # value (each drag publishes the topic AND updates it); it boots from the
        # lds_active_rpm param. _lds_sent tracks the last setpoint published by ANY
        # owner so the SSE frame's tgt/state stay truthful.
        try:
            self._lds_user_rpm = float(node.get_parameter("lds_active_rpm").value)
        except Exception:
            self._lds_user_rpm = LDS_DEFAULT_RPM
        self._lds_manual_until = 0.0   # monotonic until a manual owner holds the topic
        self._lds_rebuild_until = 0.0  # monotonic until: map-clear rebuild window
        self._lds_sent = None          # last setpoint published (any owner), or None
        self._lds_sent_at = STALE
        self._lds_hold = 0             # >0 while the IMU interference test owns the spin motor
        self._last_move_at = None      # monotonic ts of the last commanded motion (/cmd_vel)
        # One always-on timer: builds/notifies frames while clients exist, manages the
        # lazy subscriptions, and is a single cheap early-out when nobody's watching.
        node.create_timer(self._period, self._tick)
        # ...plus the idle controller's own 1 Hz tick (runs regardless of browsers —
        # the spin-down must work with the page closed).
        node.create_timer(LDS_IDLE_TICK, self._lds_ctrl_tick)
        # The controller's ALWAYS-ON subscriptions: /cmd_vel (commanded motion — the
        # teleop keepalive, Nav2's controller, canned moves and skill actions all
        # publish it) + bt_navigator's goal status. Both are tiny and MUST be seen
        # with no browser connected (the lazy browser-only subs vanish after
        # SUB_LINGER, which would make the controller park the lidar mid-navigation).
        # They also feed the optical bumper and the web map's status chip — one sub,
        # three consumers.
        self._ctrl_subs = [
            node.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 5),
            node.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status",
                                     self._on_goal_status, 5),
            # NavigateThroughPoses status rides the SAME shape (GoalStatusArray)
            # on the sibling action namespace — one more always-on sub so the
            # waypoint chip + LDS busy window track a multi-waypoint goal with
            # the page closed, exactly like a single goal.
            node.create_subscription(GoalStatusArray,
                                     "navigate_through_poses/_action/status",
                                     self._on_goal_status, 5),
            # Waypoint progress (f.nav.wp_index): the action's 1-2 Hz feedback
            # carries number_of_poses_remaining — enough to light the current
            # waypoint on the map. Always-on like the status sub above.
            node.create_subscription(NavThroughPosesFeedback,
                                     "navigate_through_poses/_action/feedback",
                                     self._on_wp_feedback, 2),
        ]

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
            # Wall-clock BUILD stamp: consumers measuring inter-frame dt (pid_tune's
            # tick-speed scoring) must divide motion by BUILD time, not parse time —
            # after a gateway stall the backlogged frames arrive in one burst and
            # parse-time dt collapses, inflating speeds ~6x (the 2026-09-22 "spin
            # overspeed" that wasn't). Additive key; the page ignores it.
            "t": round(time.time(), 3),
            "susp": [n._susp_l, n._susp_r],
            "pickup_override": n._susp_override,
            "esp": {"hb": hb, "hb_age": round(now - hb_at, 2) if hb is not None else None,
                    "temp": esp_temp, "temp_age": round(now - esp_temp_at, 2)
                    if esp_temp is not None else None,
                    "hall": self._hall, "ticks": self._ticks,
                    "stray": self._stray,
                    "tick_hz": round(self._tick_hz, 1),
                    "wheel_trim": self._wheel_trim,
                    "wheel_pid": self._wheel_pid,
                    "wheel_params": self._wheel_params},
            "lds": dict(self._lds, **self._lds_ctrl_state(now)),
            "oled": self._oled,
        }
        # Canned-move (POST /move) progress — web_server's maneuver state, a plain
        # JSON-safe dict it replaces atomically (getattr: the fake dev node has none).
        mv = getattr(n, "_maneuver_state", None)
        if mv is not None:
            f["move"] = mv
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
        # the inflation bubble + robot-circle radii. All tiny; the heavy map grid
        # itself is served by the /map HTTP route, never this frame. The two radii
        # read web_control's own params (the /nav/config sliders' source of truth —
        # the same values the costmap pushes carry) each tick, with the module
        # fallbacks if the params are somehow missing.
        try:
            infl = float(self._node.get_parameter("nav_inflation_m").value)
        except Exception:
            infl = NAV_INFLATION_M
        try:
            diam = float(self._node.get_parameter("nav_robot_diam_m").value)
        except Exception:
            diam = NAV_ROBOT_DIAM_M
        pose = self._tf_pose()
        map_age = (now - self._map_arrival) if self._map_arrival != STALE else None
        f["nav"] = {
            "pose": [round(v, 3) for v in pose[:3]] if pose else None,
            # Pose source: "tf" = live map->base_link lookup; "extrap" =
            # display-only dead-reckon while the lidar is parked (amber dot on
            # the page). Additive key — older pages ignore it.
            "pose_src": pose[3] if pose else None,
            "goal": self._goal,
            "status": self._goal_status,
            "inflation": round(infl, 3),
            "robot_radius": round(diam / 2.0, 3),
            # Feeds-health strip (Map card): seconds since slam_toolbox last
            # published /map (None = never arrived) + whether the static
            # base_link->laser TF exists (None = nano-tf down). Both cheap:
            # _map_arrival is already tracked, _tf_laser_age is one TF lookup.
            "map_age": round(map_age, 1) if map_age is not None else None,
            "tf_laser": self._tf_laser_age(),
            # Waypoint progress (multi-waypoint goals, f.nav.wp_*): None on a
            # single-goal NavigateToPose; index/total while NavigateThroughPoses
            # runs. Additive keys — older pages ignore them.
            "wp_index": self._wp_index,
            "wp_total": self._wp_total,
            "plan_age": round(now - self._plan_arrival, 1)
                        if self._plan_arrival != STALE else None,
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
        s(sub(Float32MultiArray, "wheel_pid", self._on_wheel_pid, 2))
        s(sub(Float32MultiArray, "wheel_params", self._on_wheel_params, 2))
        s(sub(Float32, "lds_rpm", self._mk_lds("rpm"), 2))
        s(sub(Float32, "lds_hz", self._mk_lds("hz"), 2))
        s(sub(Float32, "lds_duty", self._mk_lds("duty"), 2))
        # ESP32 jam guard (main.cpp ldsControl): latched true when the spin motor is
        # commanded but can't reach speed (physically blocked / no tach frames) — the
        # firmware parks the motor so it can't overheat; surfaced in f.lds.jam.
        s(sub(Bool, "lds_jam", self._on_lds_jam, 2))
        s(sub(Float32, "fan_pwm", self._on_fan, 2))
        s(sub(MagneticField, "imu/mag", self._on_mag, 2))
        # Direct sub (not the 1 Hz vitals blob) -- the 3D orientation view needs eul
        # fresh every frame, and sys_monitor's own /imu/euler sub only feeds its once-
        # a-second blob write, so routing through it silently capped the browser to
        # ~1 Hz updates no matter how fast imu_driver itself published.
        s(sub(Vector3Stamped, "imu/euler", self._on_eul, 5))
        s(sub(String, "imu_calibrate_status", self._mk_str("_imu_cal_status"), self._latched_qos))
        s(sub(String, "imu_mount_settings", self._mk_str("_imu_mount_settings"), self._latched_qos))
        # NOTE: /cmd_vel + /navigate_to_pose/_action/status are NOT here — they moved
        # to __init__ as ALWAYS-ON subscriptions (the LDS idle controller must see
        # them with no browser connected; they also feed the optical bumper + the
        # web map's status chip). See __init__.
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
        # Nav2's costmaps (owned by controller_server = local, planner_server =
        # global; published at publish_frequency 1 Hz). Subscribed VOLATILE on
        # purpose: Nav2 Humble's costmap publisher durability has varied across
        # releases, and a TRANSIENT_LOCAL request against a VOLATILE publisher
        # is a silent QoS incompatibility (no data, ever). A volatile sub is
        # compatible with either — we just wait <=1 s for the next publish,
        # which the 1 Hz page poll consumes anyway.
        s(sub(OccupancyGrid, "local_costmap/costmap",
              lambda m: self._on_costmap("_local_costmap_payload", "odom", m), 1))
        s(sub(OccupancyGrid, "global_costmap/costmap",
              lambda m: self._on_costmap("_global_costmap_payload", "map", m), 1))
        # planner_server's computed path (published on /plan each replan — 1 Hz
        # with the RateController-wrapped ComputePathToPose). VOLATILE depth 1,
        # same reasoning as the costmaps; cached downsampled (see _on_plan),
        # served by the GET /plan route as the map view's polyline.
        s(sub(Path, "plan", self._on_plan, 1))
        # NOTE: bt_navigator's goal status sub moved to __init__ (always-on) — see the
        # cmd_vel note above.
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

    def _on_plan(self, msg):
        """Cache planner_server's latest /plan (nav_msgs/Path), downsampled to
        PLAN_MAX_POINTS poses. Empty paths are skipped so a planner hiccup
        can't blank the polyline; the copy is one atomic assignment."""
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            return
        if len(pts) > PLAN_MAX_POINTS:
            step = (len(pts) - 1) / (PLAN_MAX_POINTS - 1)
            pts = [pts[round(i * step)] for i in range(PLAN_MAX_POINTS)]
        flat = []
        for x, y in pts:
            flat.extend((round(x, 4), round(y, 4)))
        self._plan_payload = ({"n": len(pts), "t": time.time()}, flat)
        self._plan_arrival = time.monotonic()

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

    def _on_wheel_pid(self, msg):
        # Float32MultiArray [kp, ki, kd] readback from the ESP32's live PID gains
        d = list(msg.data) if msg.data else []
        self._wheel_pid = [round(float(x), 4) for x in d[:3]] if len(d) >= 3 else None

    def _on_wheel_params(self, msg):
        # Float32MultiArray (id,value)-pair readback of the ESP32's live drivetrain
        # parameters — ids: 0 ticks_per_rev, 1 wheel_radius, 2 wheel_separation,
        # 3 max_linear, 4 max_angular, 5 target_slew, 6 stiction dither amplitude,
        # 7 vel_hyst (adaptive-filter N Schmitt band), 8 wheel-PID feedforward KFF
        # override (0 = auto-derive from ids 3/4/2; see firmware set_param).
        # Keep in step with the firmware's pair count.
        d = list(msg.data) if msg.data else []
        self._wheel_params = ([round(float(x), 5) for x in d[:18]]
                              if len(d) >= 2 and len(d) % 2 == 0 else None)

    @staticmethod
    def _mk_motor_params(v):
        # (id,value) pairs for /motor_params: flat even-length list, at most 9 pairs,
        # ids 0..8. Values are clamped firmware-side; here we only sanity-gate the shape.
        if not isinstance(v, (list, tuple)) or not v or len(v) % 2 or len(v) > 18:
            raise ValueError("expected a flat (id,value) pair list, max 9 pairs")
        out = []
        for i in range(0, len(v), 2):
            pid, val = int(v[i]), float(v[i + 1])
            if not 0 <= pid <= 8:
                raise ValueError(f"param id {pid} out of range 0..8")
            out.extend([float(pid), val])
        return Float32MultiArray(data=out)

    def _mk_lds(self, key):
        def cb(msg):
            self._lds[key] = round(msg.data, 3)
            self._lds_at = time.monotonic()   # staleness for the feeds strip
        return cb

    def _on_lds_jam(self, msg):
        # bool(msg.data) also normalizes the rmw_zenoh bytes paranoia class
        # (b'\x00'/b'\x01' are falsy/truthy exactly like False/True).
        self._lds["jam"] = bool(msg.data)

    def _on_fan(self, msg):
        self._fan = round(msg.data, 3)

    def _on_mag(self, msg):
        m = msg.magnetic_field
        self._mag = (round(m.x, 1), round(m.y, 1), round(m.z, 1))

    def _on_eul(self, msg):            # x=roll, y=pitch, z=yaw (deg) -- imu_driver's /imu/euler
        v = msg.vector
        self._eul = (v.x, v.y, v.z, time.monotonic())

    def _on_cmd_vel(self, msg):
        lin, ang = msg.linear.x, msg.angular.z
        self._cmd_vel = (lin, ang)
        # "Last commanded motion" — the LDS idle controller's spin-down clock (and
        # the web UI's live last-move timer). Uses the same commanded floor as the
        # optical bumper, so there's ONE definition of "being driven": the keepalive's
        # steady {0,0} re-asserts and Nav2's between-goal silence never reset it.
        try:
            eps = float(self._node.get_parameter("vision_bumper_cmd_eps").value)
        except Exception:
            eps = 0.03
        if abs(lin) > eps or abs(ang) > eps:
            self._last_move_at = time.monotonic()
            if not self._moving:
                self._moving = True
                self._navlog_add(f"moving (v={lin:+.2f} w={ang:+.2f})")
        elif self._moving:
            self._moving = False
            self._navlog_add("stopped (cmd_vel quiet)")

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
        # A rebuilt map may have new dims/origin — re-rasterize + re-latch the
        # keepout mask so the KeepoutFilter's cells stay aligned with the new
        # grid (no-op when no zones are set).
        self.publish_keepout()

    def _on_costmap(self, attr, frame, msg):
        """Cache the latest Nav2 costmap for the /local_costmap + /global_costmap
        HTTP routes (same blob shape as /map). Nav2 cells are COSTS, not
        occupancy: 0 free, 1..252 inflation gradient, 253 inscribed, 254 lethal,
        255 unknown — served raw so the page shades the ramp. The local costmap
        is a rolling window in the ODOM frame, so its origin is re-projected into
        the map frame here (TF map->odom at the grid's stamp) and the header
        carries `yaw` — the grid axes' rotation in the map frame (0 for the
        global costmap, which IS the map frame). A TF miss keeps the previous
        cache: a mis-placed overlay is worse than a stale one."""
        info = msg.info
        data = bytes(msg.data)               # int8[] cost bytes (0..255 unsigned)
        if info.width <= 0 or info.height <= 0 or len(data) != info.width * info.height:
            return
        ox, oy, yaw = info.origin.position.x, info.origin.position.y, 0.0
        if frame == "odom":
            t = None
            if self._tf_buf is not None:
                for stamp in (msg.header.stamp, None):   # exact ts first, latest as fallback
                    try:
                        t = self._tf_buf.lookup_transform(
                            "map", "odom", Time.from_msg(stamp) if stamp else Time())
                        break
                    except Exception:            # extrapolation/lookup — try the next
                        pass
            if t is None:
                return
            tr, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
            c, s = math.cos(yaw), math.sin(yaw)
            ox, oy = c * ox - s * oy + tr.x, s * ox + c * oy + tr.y
        if not (math.isfinite(ox) and math.isfinite(oy) and math.isfinite(yaw)):
            return
        setattr(self, attr, ({            # single atomic assignment (see __init__)
            "w": info.width, "h": info.height, "res": round(info.resolution, 6),
            "ox": round(ox, 6), "oy": round(oy, 6), "yaw": round(yaw, 6),
            "t": time.time(), "kind": "costmap",
        }, data))

    def _navlog_add(self, text, level="info"):
        """One line in the Drive tab's Nav log (GET /nav/log), mirrored to the
        app log (journald) so "why didn't it move" is diagnosable without the
        browser too. `text` must be a plain JSON-safe string (these ride HTTP)."""
        self._navlog_seq += 1
        self._navlog.append({"id": self._navlog_seq, "t": time.time(),
                             "lvl": level, "msg": str(text)})
        log = self._node.get_logger()
        msg = f"navlog: {text}"
        if level == "error":
            log.error(msg)
        elif level == "warn":
            log.warning(msg)
        else:
            log.info(msg)

    def get_navlog(self, since=0):
        """GET /nav/log body: entries newer than `since` (sequential ids), oldest
        first — the page polls incrementally and prepends newest-first."""
        since = max(0, int(since or 0))
        return [dict(e) for e in self._navlog if e["id"] > since]

    def _on_wp_feedback(self, msg):
        """NavigateThroughPoses action feedback: number_of_poses_remaining →
        f.nav.wp_index (0-based index of the waypoint being driven toward).
        Guarded so a malformed/garbage feedback can never kill the hub."""
        try:
            total = self._wp_total
            remaining = int(msg.number_of_poses_remaining)
            if total is None:
                return                      # feedback for some other client's goal
            idx = max(0, total - 1 - remaining)
            if idx != self._wp_index and idx < total:
                self._wp_index = idx
                self._navlog_add(f"waypoint {idx + 1}/{total} — heading to the next stop")
        except Exception:
            pass                            # rmw_zenoh oddity / foreign message — never fatal

    def _on_goal_status(self, msg):
        """Track bt_navigator's action status (the web chip + the LDS idle
        controller's busy signal). status_list gains an entry per goal state
        transition; the LAST entry is the current goal. On a terminal state the
        goal mirror is dropped too, so the browser's goal ring doesn't resurrect
        from every subsequent frame. Every transition also lands in the Nav log
        with its duration."""
        now = time.monotonic()
        self._goal_status_at = now   # arrival time (busy-state trust windows)
        if not msg.status_list:
            return
        code = msg.status_list[-1].status
        if isinstance(code, bytes):      # rmw_zenoh int8 paranoia (see _on_diag)
            code = code[0]
        code = int(code)
        new = NAV_STATUS.get(code, "idle")
        prev = self._goal_status
        if new != prev:
            since = self._goal_status_since
            dur = f" (was {prev} for {now - since:.1f}s)" if since is not None else ""
            if new == "failed":
                self._navlog_add(f"nav → FAILED{dur} — bt recovery exhausted", "warn")
            elif new == "planning":
                self._navlog_add(f"nav → planning{dur} — planner accepted the goal")
            elif new == "navigating":
                self._navlog_add(f"nav → navigating{dur} — plan ready, driving")
            elif new == "arrived":
                self._navlog_add(f"nav → arrived{dur}")
            elif code == 5:
                self._navlog_add(f"nav → canceled{dur}")
            else:
                self._navlog_add(f"nav → {new}{dur}")
            self._goal_status_since = now
        if code in (4, 5, 6):            # SUCCEEDED / CANCELED / ABORTED
            pub_at = self._goal_published_at
            if pub_at is not None:
                verdict = {4: "reached", 5: "canceled", 6: "FAILED"}[code]
                self._navlog_add(f"goal {verdict} in {now - pub_at:.1f}s",
                                 "info" if code in (4, 5) else "warn")
                self._goal_published_at = None
            self._goal = None
            self._wp_index = None            # waypoint progress is done either way
            self._wp_total = None
        self._goal_status = new

    def _tf_laser_age(self):
        """base_link→laser static TF existence check for the Map card's feeds
        strip: 0.0 when the TF resolves (nano-tf unit up — static transforms
        are latched so 'exists' is the meaningful state, not age), None when
        it doesn't (nano-tf down → slam_toolbox can't resolve the scan frame
        and silently drops every scan). One tf2 lookup per tick, same lazy
        buffer as _tf_pose."""
        if self._tf_buf is None:
            return None
        try:
            self._tf_buf.lookup_transform("base_link", "laser", Time())
            return 0.0
        except Exception:                    # missing TF / connectivity — nano-tf down
            return None

    def _tf_pose(self):
        """Map-frame pose (x, y, yaw_rad) + a source tag, or None.

        Source "tf": a live map->base_link lookup at Time(0) (= latest available
        transform). On success the map->odom half is also cached.

        Source "extrap": DISPLAY-ONLY dead-reckoned fallback (2026-09-24 TODO).
        While the lidar is parked, slam_toolbox processes no scans, so its
        map->odom stays stamped at the LAST processed scan; the composed
        map->base_link lookup then fails ("extrapolation into the past") and the
        web map's dot used to just stop. The wheels keep knowing where the robot
        is (odom->base_link flows continuously), so we compose the CACHED
        map->odom with the live /odom pose instead. Accuracy caveat: while
        parked-and-pushed (or on carpet slip) the dead-reckoned marker drifts vs
        reality until the lidar wakes and slam re-anchors — the page shows the
        amber pose dot for it. Nothing may CONSUME the extrapolated pose as if
        it were slam-verified (Locations Save uses its own lookup and never
        sees this fallback)."""
        if self._tf_buf is None:
            return None
        try:
            t = self._tf_buf.lookup_transform("map", "base_link", Time())
            tr, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
            try:
                mo = self._tf_buf.lookup_transform("map", "odom", Time())
                self._map_odom = (
                    mo.transform.translation.x, mo.transform.translation.y,
                    math.atan2(2.0 * mo.transform.rotation.w * mo.transform.rotation.z,
                               1.0 - 2.0 * mo.transform.rotation.z ** 2))
            except Exception:
                pass                        # keep the previous cache
            return (tr.x, tr.y, yaw, "tf")
        except Exception:
            # map->base_link failed. Dead-reckon: cached map->odom ∘ live /odom.
            mo, od = self._map_odom, self._odom
            if mo is None or od is None:
                return None
            c, s = math.cos(mo[2]), math.sin(mo[2])
            return (c * od[0] - s * od[1] + mo[0],
                    s * od[0] + c * od[1] + mo[1],
                    mo[2] + od[2], "extrap")

    def get_map_payload(self):
        """The /map HTTP route body: (meta_dict, int8_bytes), or (None, None)
        until slam_toolbox has published a grid. Atomic: meta and cells always
        come from the same /map message."""
        return self._map_payload or (None, None)

    def get_local_costmap_payload(self):
        """GET /local_costmap body, same shape as get_map_payload (meta carries
        kind:"costmap" + yaw; cells are Nav2 costs 0..255). None until the
        controller_server has published a grid."""
        return self._local_costmap_payload or (None, None)

    def get_global_costmap_payload(self):
        """GET /global_costmap body, same shape as get_local_costmap_payload."""
        return self._global_costmap_payload or (None, None)

    def get_plan_payload(self):
        """GET /plan body: ({n, t}, [x0, y0, x1, y1, ...]) — the latest planned
        path, downsampled. (None, None) until planner_server has published."""
        return self._plan_payload or (None, None)

    def clear_goal(self):
        """Drop the goal mirror + chip state (POST /nav/cancel). Nav2's own status
        topic will corroborate with CANCELED/UNKNOWN on the next tick."""
        if self._goal is not None or self._goal_status not in ("idle",):
            self._navlog_add("goal cancelled (POST /nav/cancel) — lidar may idle-park now")
        self._goal = None
        self._goal_status = "idle"
        self._goal_status_at = time.monotonic()
        self._goal_published_at = None

    def clear_map(self):
        """Drop the cached /map grid + goal mirror (POST /map/clear). The /map
        route serves (None, None) until the fresh post-restart grid arrives
        (slam_toolbox republishes transient-local the instant it's up), and
        map_age reads null so the page's feeds strip shows SLAM as down meanwhile.
        The goal mirror resets too — the map frame it points into is being reset."""
        self._map_payload = None
        self._map_arrival = STALE
        self._goal = None
        self._goal_status = "idle"
        self._goal_published_at = None
        self._navlog_add("map cleared — nano-slam restart, goal dropped, "
                         "lidar rebuild window armed")

    def note_map_clear(self):
        """POST /map/clear companion: a wiped map can only REBUILD if scans flow,
        but the idle controller may have just parked the lidar (quiet robot) — it
        would then sit on no-map until the user drives. Arm a rebuild window of
        one idle period (lds_idle_secs) that the 1 Hz controller treats as recent
        motion: the lidar wakes at the user's spin-when-active rpm, and the normal
        quiet-park resumes when the window lapses (or real motion keeps it up).
        A manual owner (slider/skill latch) or lds_hold still outranks this via
        the controller's usual paths; /lds_jam + the firmware clamp still bound it."""
        self._lds_rebuild_until = time.monotonic() + max(
            5.0, float(self._lds_param("lds_idle_secs", LDS_IDLE_SECS_DEFAULT)))

    # ---- LDS idle spin-down controller (2026-09-21) ------------------------------
    def _lds_param(self, name, default):
        """Live param read with a safe fallback (fake/dev nodes may not declare)."""
        try:
            return self._node.get_parameter(name).value
        except Exception:
            return default

    def _lds_manual_secs(self):
        try:
            return float(self._lds_param("lds_manual_secs", LDS_MANUAL_SECS_DEFAULT))
        except (TypeError, ValueError):
            return LDS_MANUAL_SECS_DEFAULT

    def _lds_ready(self):
        """Is the lidar actually delivering valid frames right now? Trusts the ESP32's
        /lds_hz (valid-frame rate, 0 = not receiving) only while the /lds_* readouts
        are fresh — a dead ESP32 link leaves the last hz lingering on the dict, and
        the age check is what reads that as NOT ready."""
        if self._lds_at is None or (time.monotonic() - self._lds_at) > LDS_READY_AGE:
            return False
        return float(self._lds.get("hz") or 0.0) >= LDS_READY_MIN_HZ

    def wake_lidar(self, reason="goal", wait=True):
        """Make sure the lidar is spinning + delivering frames BEFORE a Nav2 goal goes
        out. Fires the spin-when-active setpoint immediately if /lds_hz says no valid
        frames are flowing (the 1 Hz idle controller alone is too slow — the planner
        starts failing before its next tick), then optionally holds the CALLER (an
        HTTP/skill worker thread — never the executor) until frames actually arrive,
        bounded by LDS_WAKE_WAIT; on timeout the goal proceeds anyway (no worse than
        the old behaviour) with a Nav-log warning. Returns the seconds held (0.0 when
        the lidar was already ready). A remembered spin target of 0 (slider parked) is
        overridden for the goal — navigation is impossible without scans — falling
        back to LDS_DEFAULT_RPM; the IMU interference test's lds_hold is never fought."""
        now = time.monotonic()
        if self._lds_ready():
            return 0.0
        held = 0.0
        rpm = min(LDS_RPM_MAX, max(0.0, float(self._lds_user_rpm or 0.0)
                                   or LDS_DEFAULT_RPM))
        if not self._lds_hold:
            self._pubs["/lds_target_rpm"][0].publish(Float32(data=rpm))
            self._lds_sent = rpm
            self._lds_sent_at = now
            # Treat this as recent motion so _lds_ctrl_tick holds the wake (and
            # re-asserts it) until the goal's own nav_busy takes over.
            self._lds_rebuild_until = max(self._lds_rebuild_until, now + max(
                5.0, float(self._lds_param("lds_idle_secs", LDS_IDLE_SECS_DEFAULT))))
        self._navlog_add(f"lidar not delivering frames — spinning up to {rpm:.0f} rpm "
                         f"before {reason}")
        if wait:
            deadline = now + LDS_WAKE_WAIT
            while not self._lds_ready() and time.monotonic() < deadline:
                time.sleep(LDS_WAKE_POLL)
            held = min(time.monotonic() - now, LDS_WAKE_WAIT)
            if self._lds_ready():
                self._navlog_add(f"lidar ready after {held:.1f}s — {reason} "
                                 "going out now")
            else:
                self._navlog_add(
                    f"lidar STILL not delivering frames after {LDS_WAKE_WAIT:.0f}s — "
                    f"{reason} sent anyway; expect planning to fail (ESP32 link / "
                    "nano-sensors down?)", "warn")
        return held

    def _lds_ctrl_tick(self):
        """Own /lds_target_rpm when nobody else does: spin at the user's target while
        the robot is active (recent commanded motion or a Nav2 goal in flight), park
        it after `lds_idle_secs` of quiet, and re-assert periodically so an ESP32
        reboot (which resets its setpoint to the firmware default) is corrected
        within a tick. Runs on an always-on 1 Hz timer — the spin-down must work
        with the page closed. Manual owners (browser slider / skill action latch,
        IMU interference test hold) are never fought; the controller resumes after
        their window. Every wake/park lands in the Nav log with its reason, and a
        plan that sits unresolved (usually a stale map→odom TF — the parked-lidar
        deadlock) warns there too."""
        now = time.monotonic()
        # A busy nav status is trusted only within its arrival window, PER STATUS
        # (lds_nav_busy): navigating/canceling for LDS_NAV_STALE (a live goal keeps
        # /cmd_vel flowing), planning for the much longer LDS_PLANNING_STALE — the
        # planner needs scans BEFORE it can compute a path and there is no cmd_vel
        # during planning to refresh the motion clock. Nano-nav dying mid-goal is
        # still bounded (dead "navigating" ages out at 90 s; dead "planning" at 300).
        nav_busy = lds_nav_busy(self._goal_status, now - self._goal_status_at)
        last_move = self._last_move_at
        # A map clear (POST /map/clear) explicitly asks for a FRESH map, which can
        # only build if scans flow — a parked lidar would leave slam stuck at no-map
        # until the user drives. Treat the rebuild window as recent motion so the
        # controller spins at the user's spin-when-active rpm; it lapses back to the
        # normal quiet-park after ~lds_idle_secs (or real motion keeps it up).
        if now < self._lds_rebuild_until:
            last_move = now
        idle_enable = bool(self._lds_param("lds_idle_enable", True))
        idle_secs = float(self._lds_param("lds_idle_secs", LDS_IDLE_SECS_DEFAULT))
        rpm = lds_idle_target(now, last_move, idle_secs, idle_enable, nav_busy,
                              now < self._lds_manual_until, self._lds_user_rpm)
        # Planning watchdog (Nav log): a plan unresolved past PLANNING_WARN_SECS
        # almost always means slam's map→odom TF is stale — Nav2's Time(0)
        # map→base_link lookups resolve to the frozen map→odom stamp and fail with
        # "extrapolation into the past" forever until scans flow again. Warn
        # (rate-limited) while it lasts instead of failing silently.
        stuck = None
        if (self._goal_status == "planning" and self._goal_status_since is not None
                and now - self._goal_status_since > PLANNING_WARN_SECS):
            stuck = now - self._goal_status_since
            if now - self._navlog_warn_at > NAVLOG_WARN_PERIOD:
                self._navlog_warn_at = now
                map_age = ((now - self._map_arrival)
                           if self._map_arrival != STALE else None)
                if map_age is not None and map_age < MAP_AGE_FRESH:
                    # /map IS arriving (scans flow, slam lives) but the goal still
                    # hasn't reached EXECUTING — usually bt_navigator/lifecycle is
                    # not accepting goals (nano-nav still activating after a
                    # restart: "Managed nodes are active" never logged) or the
                    # goal was dropped mid-restart. Re-click once the nav
                    # container is up.
                    self._navlog_add(
                        f"planning stuck {int(stuck)}s but /map is fresh "
                        f"({map_age:.0f}s) — the goal is not being processed; "
                        "nano-nav may still be activating or was restarted "
                        "(check journalctl -u nano-nav, then re-send the goal)",
                        "warn")
                else:
                    self._navlog_add(
                        f"planning stuck {int(stuck)}s — /map age "
                        f"{'never arrived' if map_age is None else f'{map_age:.0f}s'}, "
                        f"lidar {self._lds_sent} rpm: slam needs fresh scans for the "
                        "map→odom TF (wake the lidar / check nano-slam)", "warn")
        if rpm is None or self._lds_hold:
            return
        changed = rpm != self._lds_sent
        if changed or (now - self._lds_sent_at) > LDS_REASSERT_SECS:
            self._pubs["/lds_target_rpm"][0].publish(Float32(data=rpm))
            self._lds_sent = rpm
            self._lds_sent_at = now
            if changed:
                self._navlog_add(
                    f"lidar spin {'wake →' if rpm > 0 else 'park →'} {rpm:.0f} rpm "
                    f"({self._lds_reason(nav_busy, idle_enable, idle_secs, last_move, now)})")

    def _lds_reason(self, nav_busy, idle_enable, idle_secs, last_move, now):
        """The Nav log's why for a wake/park transition (best-effort, never fatal)."""
        if nav_busy:
            return "nav " + self._goal_status
        if now < self._lds_rebuild_until:
            return "map rebuild window"
        if not idle_enable:
            return "idle spin-down disabled"
        if last_move is not None and now - last_move < idle_secs:
            return f"motion {now - last_move:.0f}s ago"
        if self._goal_status == "planning":
            return "planning trust window expired"
        if last_move is None:
            return "no motion since boot"
        return f"idle >{idle_secs:.0f}s"

    def note_lds_manual(self, rpm, set_target=False):
        """Record a /lds_target_rpm published OUTSIDE the controller (browser slider
        via POST /publish, or a skill action): latch the manual window so the
        controller hands the topic over for `lds_manual_secs`, and track the value
        for the frame. `set_target` (browser slider only) also makes it the
        remembered spin-when-active rpm; skills merely borrow the topic. A
        set_target also writes the lds_active_rpm param — web_control's on-set
        callback persists it to lds.json, so the chosen spin speed survives a
        restart (same flow as the toggle/secs sliders' POST /param)."""
        rpm = max(0.0, float(rpm))
        self._lds_manual_until = time.monotonic() + self._lds_manual_secs()
        self._lds_sent = rpm
        self._lds_sent_at = time.monotonic()
        if set_target:
            self._lds_user_rpm = rpm
            try:
                pv = ParameterValue()
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = rpm
                self._node.set_parameters(
                    [RclpyParameter("lds_active_rpm", value=pv)])
            except Exception:
                pass
        return rpm

    def lds_hold(self, on):
        """Suspend/resume the idle controller (reference-counted). The IMU
        interference test drives the spin motor itself and must not be fought."""
        self._lds_hold = max(0, self._lds_hold + (1 if on else -1))

    def _lds_ctrl_state(self, now):
        """The controller's view of the world, folded into f.lds: `age` (moved here
        so the whole section is built in one place), `tgt` (last setpoint published
        by any owner), `idle` (s since the last commanded motion — the web UI's live
        last-move timer; null = nothing commanded since boot), `state`, and the
        persisted controller config (`enable`/`secs` — the page re-seeds its
        Lidar-card controls from them once on first arrival, like f.esp.wheel_pid
        does for the PID sliders)."""
        age = round(now - self._lds_at, 1) if self._lds_at is not None else None
        idle = (now - self._last_move_at) if self._last_move_at is not None else None
        if self._lds.get("jam"):
            state = "jam"         # firmware latched a blocked rotor; motor is parked
        elif now < self._lds_manual_until:
            state = "manual"      # browser slider / skill holds the topic
        elif self._lds_hold:
            state = "hold"        # IMU interference test owns the spin motor
        else:
            state = "spin" if (self._lds_sent or 0.0) > 0.0 else "park"
        try:
            enable = bool(self._lds_param("lds_idle_enable", True))
            secs = float(self._lds_param("lds_idle_secs", LDS_IDLE_SECS_DEFAULT))
        except (TypeError, ValueError):
            enable, secs = True, LDS_IDLE_SECS_DEFAULT
        return {"age": age, "state": state, "tgt": self._lds_sent,
                "idle": round(idle, 1) if idle is not None else None,
                "enable": enable, "secs": secs}

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
        held = 0.0
        if topic == "/goal_pose":
            # Pre-wake: never let Nav2 plan against slam's frozen map→odom TF — hold
            # the goal (bounded) until the parked lidar is actually delivering frames.
            held = self.wake_lidar(reason="the goal")
        pub.publish(msg)
        if topic == "/goal_pose":
            # Goal mirror for the web map (f["nav"].goal). Nav2 will corroborate
            # via the action status topic within a tick or two; set "planning"
            # here so the chip reacts to the click immediately.
            self.note_goal(msg.pose.position.x, msg.pose.position.y)
        if topic == "/lds_target_rpm":
            # The browser's Spin slider: remember it as the spin-when-active target
            # AND latch the manual window so the idle controller doesn't fight the
            # user for lds_manual_secs.
            self._lds_user_rpm = self.note_lds_manual(msg.data, set_target=True)
            # Bookkeep the setpoint here too, so wake_lidar sees a slider-parked 0
            # as parked (and a slider spin-up as already commanded).
            self._lds_sent = float(msg.data)
            self._lds_sent_at = time.monotonic()
        # Diagnosability: log map-click goals + LDS rpm etc. so "who told the robot to
        # go there / spin" is in the app log. Throttle the chatty spin-down? No — these
        # are discrete user actions, not a hot loop; every one is a meaningful event.
        if topic in ("/goal_pose", "/reset_ticks", "/laser_pwm", "/motor_pid",
                     "/lds_target_rpm"):
            self._node.get_logger().info(f"POST /publish {topic} value={data.get('value')!r}")
        out = {"status": "ok", "topic": topic}
        if held:
            out["lidar_wait"] = round(held, 1)
        return out

    def note_goal(self, x, y, source=""):
        """Record a goal published outside POST /publish (skill actions) so the
        web map's goal ring + status chip stay in sync with those too. `source`
        names the publisher in the Nav log ("skill go-to 'kitchen'")."""
        self._goal = [round(float(x), 3), round(float(y), 3)]
        self._goal_status = "planning"
        now = time.monotonic()
        self._goal_status_at = now
        self._goal_status_since = now
        self._goal_published_at = now
        via = f" via {source}" if source else ""
        self._navlog_add(f"goal ({self._goal[0]:.2f}, {self._goal[1]:.2f}){via} — "
                         "planning; lidar must spin for the planner's map→odom TF")

    def note_waypoints(self, poses, source=""):
        """Mirror a multi-waypoint goal (POST /nav/waypoints) for the web map:
        the goal ring sits on the FIRST waypoint, the chip reads planning, and
        f.nav gains wp_total (the action's feedback refines wp_index as it
        drives). `poses` is the [(x, y), ...] list that was sent."""
        self.note_goal(poses[0][0], poses[0][1], source=source)
        self._wp_total = len(poses)
        self._wp_index = 0
        self._navlog_add(f"waypoints: {len(poses)} stops — "
                         + " → ".join(f"({x:.2f},{y:.2f})" for x, y in poses))

    @staticmethod
    def _mk_goal(v):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x = min(GOAL_MAX_ABS_M, max(-GOAL_MAX_ABS_M, float(v["x"])))
        m.pose.position.y = min(GOAL_MAX_ABS_M, max(-GOAL_MAX_ABS_M, float(v["y"])))
        m.pose.orientation.w = 1.0
        return m

    # ---- keepout zones (web-drawn no-go rectangles → the global costmap) -------
    def set_keepout_zones(self, zones):
        """Take the persisted keepout rectangles (map-frame metres,
        [{x1,y1,x2,y2}, ...]) from web_server and (re)publish the mask. Called
        on save/delete/clear AND on boot re-apply."""
        self._keepout_zones = [
            (float(z["x1"]), float(z["y1"]), float(z["x2"]), float(z["y2"]))
            for z in (zones or [])
            if all(k in z for k in ("x1", "y1", "x2", "y2"))
        ]
        self.publish_keepout()

    def publish_keepout(self):
        """Rasterize the keepout rectangles into the CURRENT /map grid geometry
        and latch the mask + its filter-info topic. No map yet (boot) or no
        zones → an empty mask is still published once so a deleted zone can
        clear a previously latched one; nothing at all before the first /map."""
        if CostmapFilterInfo is None:
            return
        meta = self._map_payload[0] if self._map_payload else None
        if meta is None:
            return                          # can't rasterize without grid geometry
        w, h, res = int(meta["w"]), int(meta["h"]), float(meta["res"])
        ox, oy = float(meta["ox"]), float(meta["oy"])
        cells = bytearray(w * h)            # 0 = free everywhere
        for x1, y1, x2, y2 in self._keepout_zones:
            lo_x, hi_x = sorted((x1, x2))
            lo_y, hi_y = sorted((y1, y2))
            c0 = max(0, int((lo_x - ox) / res))
            c1 = min(w - 1, int(math.ceil((hi_x - ox) / res)) - 1)
            r0 = max(0, int((lo_y - oy) / res))
            r1 = min(h - 1, int(math.ceil((hi_y - oy) / res)) - 1)
            for r in range(r0, r1 + 1):
                base = r * w
                for c in range(c0, c1 + 1):
                    cells[base + c] = 100   # occupied = keepout
        mask = OccupancyGrid()
        mask.header.frame_id = "map"
        mask.header.stamp = self._node.get_clock().now().to_msg()
        mask.info.width = w
        mask.info.height = h
        mask.info.resolution = res
        mask.info.origin.position.x = ox
        mask.info.origin.position.y = oy
        mask.data = list(cells)
        info = CostmapFilterInfo()
        info.type = 0                       # KEEPOUT filter
        info.filter_mask_topic = "keepout_mask"
        self._pubs["/keepout_mask"][0].publish(mask)
        self._pubs["/keepout_filter_info"][0].publish(info)

    @staticmethod
    def _mk_keepout_mask(v):
        """POST /publish /keepout_mask is whitelisted but UNUSED by the page —
        the mask is server-owned (rasterized from the persisted zones). Accept
        nothing: a browser can't hand-craft grid cells."""
        raise ValueError("keepout mask is server-owned (draw zones on the map)")

    @staticmethod
    def _mk_keepout_info(v):
        raise ValueError("keepout filter info is server-owned")

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
    def _mk_motor_pid(v):
        if not isinstance(v, (list, tuple)) or len(v) != 3:
            raise ValueError("expected a 3-element list [kp, ki, kd]")
        kp, ki, kd = (float(x) for x in v)
        return Float32MultiArray(data=[min(20.0, max(0.0, kp)),
                                       min(100.0, max(0.0, ki)),
                                       min(5.0, max(0.0, kd))])

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
        if node == "web_control" and name == "lds_idle_enable":
            # Toggling Idle spin-down (off or on) is the user taking the topic BACK
            # from a manual owner (a slider drag latches `lds_manual_secs` — 300 s
            # by default). Without this the controller keeps handing off for the
            # whole manual window after re-enabling: the Lidar card stays "manual"
            # and spin-down never resumes. Same clearing the slider path does via
            # _lds_sent bookkeeping.
            self._lds_manual_until = 0.0
        # Diagnosability: who/what changed a param (esp. enable_motion) must be in the
        # app log — nav_node's _on_params only logs the transition it receives, so this
        # records the browser-facing side too.
        self._node.get_logger().info(
            f"POST /param {node}/{name} = {value!r} (source: web UI)")
        return {"status": "sent", "node": node, "name": name}
