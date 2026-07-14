"""Super-light 2D SLAM for the Nano robot (Stage 1 of 3).

Estimates the robot pose by fusing wheel-odometry translation (/odom) with IMU yaw
(/imu/euler) as the motion prior, then refines it each scan with a correlative
scan-to-map matcher (see occupancy.py). Builds an occupancy grid from /scan and dumps
it — plus the live pose — to a RAM file (/dev/shm/nano_map.bin) that web_control serves
to the browser map panel. Also republishes the corrected pose on /slam_pose for debug.

No motion is commanded here. Stage 2 adds the planner + click-to-goal; Stage 3 adds the
pure-pursuit controller (gated behind enable_motion).

Map file format (atomic via os.replace): one JSON metadata line, '\n', then the raw
int8 occupancy bytes (row-major, row 0 = origin_y). The browser parses the header and
draws the rest straight into an ImageData.
"""
import collections
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Int8, Int64MultiArray, Float32

from .occupancy import GridMap

MAP_FILE = "/dev/shm/nano_map.bin"

# --- self-test / calibration sequence (drives known motions to check IMU + encoders) ---
TEST_LIN = 0.12      # m/s forward/back speed (capped by max_lin)
TEST_ANG = 0.6       # rad/s in-place spin speed (capped by max_ang)
TEST_DIST = 0.35     # m to drive forward, then back
TEST_TURNS = 1.0     # full in-place rotations for the IMU-vs-odom cross-check
TEST_SETTLE = 1.2    # s to settle between motion legs


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _sd_notify(msg):
    """Best-effort systemd notification (Type=notify units): READY on start, then
    WATCHDOG pets from an executor timer — if a callback wedges the executor the pets
    stop and systemd restarts the node (WatchdogSec in nano-nav.service). No-op outside
    systemd. (Deliberately duplicated in each supervised main: ~10 dependency-free
    lines beat a cross-package util import.)"""
    path = os.environ.get("NOTIFY_SOCKET")
    if not path:
        return
    try:
        import socket
        s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            s.sendto(msg.encode(), "\0" + path[1:] if path.startswith("@") else path)
        finally:
            s.close()
    except OSError:
        pass


class NavNode(Node):
    def __init__(self):
        super().__init__("slam_nav")
        self.declare_parameters("", [
            ("scan_topic", "scan"),
            ("odom_topic", "odom"),
            ("euler_topic", "imu/euler"),
            ("map_size_m", 24.0),
            ("map_resolution", 0.05),
            ("range_min", 0.12),
            ("range_max", 6.0),
            ("match_lin", 0.10),         # scan-match search half-window, metres
            ("match_ang", 0.12),         # scan-match search half-window, radians
            ("match_points", 90),        # scan points used for matching (decimated)
            ("min_match_score", 1.0),    # below this, keep the prior (no good overlap)
            ("use_imu_yaw", True),       # IMU yaw delta for rotation (else wheel odom)
            ("still_skip", True),        # parked (odom+IMU unchanged) -> skip match+integrate
            ("still_lin", 0.005),        # m translation since last processed scan = "moved"
            ("still_ang", 0.005),        # rad (~0.3 deg) yaw delta = "moved" (fires on pure rotation)
            ("map_write_rate", 2.0),     # Hz to (re)write the /dev/shm map file
            # --- navigation (Stages 2/3) ---
            ("enable_motion", False),    # SAFETY: when false, plan+show path but DON'T drive
            ("robot_radius", 0.16),      # obstacle inflation for the planner (m)
            ("plan_downsample", 4),      # plan on a 1/N grid (CPU/RAM); 4 -> 0.20 m cells
            ("allow_unknown", True),     # let the global plan cross unmapped cells
            ("control_rate", 10.0),      # Hz controller / pursuit loop
            ("replan_period", 1.0),      # s between global replans while a goal is active
            ("max_lin", 0.15),           # m/s pure-pursuit speed cap
            ("max_ang", 1.0),            # rad/s turn rate cap
            ("lookahead", 0.30),         # m pure-pursuit lookahead
            ("goal_tol", 0.12),          # m: within this of the goal = arrived
            ("stop_distance", 0.25),     # m: obstacle closer than this ahead = stop+replan
            ("front_angle", 0.6),        # rad: half-width of the reactive front cone
            # --- extras (all independently toggleable; the heavy ones default OFF) ---
            ("map_store", ""),           # path to persist the map (.npz). "" = disabled.
            ("autosave_period", 0.0),    # s between background map autosaves; 0 = off
            ("auto_explore", False),     # drive to frontiers when idle (needs enable_motion)
            ("explore_period", 4.0),     # s between frontier picks while exploring
            ("trail_max", 400),          # breadcrumb ring-buffer length; 0 = off
            ("stuck_timeout", 0.0),      # s commanded-but-not-moving before abort; 0 = off
            ("stuck_eps", 0.04),         # m: movement below this counts as "not moving"
            # --- LDS idle spin-down (power/wear/noise: park the spin motor when it's not
            #     earning its keep — stationary + no reason to expect fresh scans soon) ---
            ("lds_idle_enable", True),    # master on/off for the spin-down behaviour
            ("lds_idle_timeout", 60.0),   # s of no motion/goal/exploring before spin-down; 0=off
            ("lds_idle_rpm", 0.0),        # target rpm while parked (0 = fully stop)
            ("lds_active_rpm", 300.0),    # target rpm to resume (match firmware LDS_TARGET_RPM)
            # --- pick-up awareness + lost-robot relocalization (Tier-1 autonomy) ---
            ("pickup_pause", True),       # both wheels off-ground -> halt + freeze SLAM
            ("pickup_face", "focused"),   # OLED mood while lifted ("" = don't touch the OLED)
            ("relocalize", True),         # auto-recover localization when the scan stops matching
            ("recover_patience", 5),      # consecutive unmatched scans before declaring "lost"
            ("recover_min_beams", 40),    # need this many in-range beams to trust a "mismatch"
            ("recover_exit_score", 4.0),  # match score that ends recovery (= relocalized)
            ("recover_lin", 0.5),         # recovery scan-match search half-window (m)
            ("recover_ang", 1.0),         # recovery scan-match search half-window (rad)
            ("recover_half", 6),          # recovery candidates per axis (2*half+1)
            ("recover_refine", 3),        # recovery coarse-to-fine passes
            ("recover_spin", 0.6),        # rad/s in-place spin while relocalizing (needs motion)
            ("recover_timeout", 12.0),    # s before giving up the active relocalize search
            # --- personality -> motion (the behaviour layer's `caution` trait, clamped
            #     REFLEXIVELY here so the cognitive layer can never push motion unsafe) ---
            ("trait_motion", False),       # opt-in: let `caution` nudge stop_distance/max_lin
            ("stop_distance_max", 0.45),   # m: the cautious extreme of stop_distance (caution=1)
            ("max_lin_min", 0.08),         # m/s: the cautious extreme of max_lin (caution=1)
        ])
        g = self.get_parameter
        self.grid = GridMap(
            size_m=float(g("map_size_m").value), res=float(g("map_resolution").value),
            rmin=float(g("range_min").value), rmax=float(g("range_max").value))
        self.match_lin = float(g("match_lin").value)
        self.match_ang = float(g("match_ang").value)
        self.match_pts = int(g("match_points").value)
        self.min_score = float(g("min_match_score").value)
        self.use_imu = bool(g("use_imu_yaw").value)
        self.still_skip = bool(g("still_skip").value)
        self.still_lin = float(g("still_lin").value)
        self.still_ang = float(g("still_ang").value)
        self._write_period = 1.0 / max(0.2, float(g("map_write_rate").value))

        # navigation params
        self.enable_motion = bool(g("enable_motion").value)
        self.robot_radius = float(g("robot_radius").value)
        self.plan_downsample = int(g("plan_downsample").value)
        self.allow_unknown = bool(g("allow_unknown").value)
        self.replan_period = float(g("replan_period").value)
        self.max_lin = float(g("max_lin").value)
        self.max_ang = float(g("max_ang").value)
        self.lookahead = float(g("lookahead").value)
        self.goal_tol = float(g("goal_tol").value)
        self.stop_distance = float(g("stop_distance").value)
        self.front_angle = float(g("front_angle").value)
        # personality -> motion: keep the params as the relaxed (caution=0) end; caution
        # eases stop_distance up + max_lin down toward the configured cautious extremes.
        self._trait_motion = bool(g("trait_motion").value)
        self._base_stop = self.stop_distance
        self._base_max_lin = self.max_lin
        self._stop_max = float(g("stop_distance_max").value)
        self._max_lin_min = float(g("max_lin_min").value)

        # extras
        self.map_store = str(g("map_store").value)
        self._autosave_period = float(g("autosave_period").value)
        self.auto_explore = bool(g("auto_explore").value)
        self.explore_period = float(g("explore_period").value)
        self._trail_max = int(g("trail_max").value)
        self.stuck_timeout = float(g("stuck_timeout").value)
        self.stuck_eps = float(g("stuck_eps").value)
        self.lds_idle_enable = bool(g("lds_idle_enable").value)
        self.lds_idle_timeout = float(g("lds_idle_timeout").value)
        self.lds_idle_rpm = float(g("lds_idle_rpm").value)
        self.lds_active_rpm = float(g("lds_active_rpm").value)

        # pick-up awareness + lost-robot relocalization
        self.pickup_pause = bool(g("pickup_pause").value)
        self.pickup_face = str(g("pickup_face").value)
        self.relocalize = bool(g("relocalize").value)
        self.recover_patience = int(g("recover_patience").value)
        self.recover_min_beams = int(g("recover_min_beams").value)
        self.recover_exit_score = float(g("recover_exit_score").value)
        self.recover_lin = float(g("recover_lin").value)
        self.recover_ang = float(g("recover_ang").value)
        self.recover_half = int(g("recover_half").value)
        self.recover_refine = int(g("recover_refine").value)
        self.recover_spin = float(g("recover_spin").value)
        self.recover_timeout = float(g("recover_timeout").value)

        # SLAM pose in the map frame.
        self.px = self.py = self.pth = 0.0
        self.home = (0.0, 0.0)       # where the robot booted = map origin; "go home" target
        self._have_map = False
        # Motion-prior trackers (last odom pose + last IMU yaw consumed by a scan).
        self._odom = None
        self._imu_yaw = None
        self._prev_odom = None
        self._prev_imu = None
        self._last_write = 0.0
        # _write_map export cache: recomputing occupancy_int8 (a full-grid np.exp) +
        # coverage at the write rate cost ~40% of a core even while SLAM was paused.
        # grid.rev only moves when the grid content does, so cache the derived bytes.
        self._occ_rev = -1
        self._occ_bytes = b""
        self._cov_cache = (0.0, 0.0, 0.0)

        # navigation state (all callbacks + the control timer run on the one spin
        # thread, so plain attributes are safe — no locks needed).
        self._goal = None            # (x, y) world, or None
        self._goal_is_frontier = False  # current goal came from auto-explore
        self._path = []              # [(x, y)] world waypoints
        self._last_scan = None       # (angles, ranges) for the reactive layer
        self._next_replan = 0.0
        self._next_explore = 0.0
        self._next_autosave = 0.0
        self._last_score = 0.0       # last accepted scan-match score (localization health)
        self._trail = (collections.deque(maxlen=self._trail_max)
                       if self._trail_max > 0 else None)
        # stuck detector
        self._stuck_ref = None       # (x, y) pose when the current move started
        self._stuck_since = 0.0
        # LDS idle spin-down: tracks the last (x, y) considered "moving" + when
        self._lds_idle_ref = None    # (x, y); None until the first tick seeds it
        self._lds_idle_since = 0.0
        self._lds_active = True      # current commanded state; only republish on change
        # pick-up + relocalization state
        self._susp_l = self._susp_r = False   # per-wheel off-ground switches (from the ESP)
        self._susp_override = -1              # /pickup_override: -1 auto, 0 grounded, 1 lifted
        self._picked_up = False
        self._recovering = False     # actively re-searching for the pose (lost / set down)
        self._recover_until = 0.0
        self._lost_count = 0         # consecutive scans the match has failed
        # self-test / calibration state
        self._test_active = False
        self._test_seq = []
        self._test_phase = 0
        self._test_phase_t0 = 0.0
        self._test_entered = False
        self._ticks = None           # latest raw /wheel_ticks [L,R] (lazy sub, test only)
        self._imu_accel = self._imu_gyro = self._imu_hz = 0.0
        self._ticks_sub = self._imuweb_sub = None

        # Optionally reload a previously-saved map (relocalize into it from the origin).
        if self.map_store and self.grid.load(self.map_store):
            self._have_map = True
            self.get_logger().info(f"loaded saved map from {self.map_store}")

        self.pose_pub = self.create_publisher(PoseStamped, "slam_pose", 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.lds_rpm_pub = self.create_publisher(Float32, "lds_target_rpm", 10)
        self.path_pub = self.create_publisher(Path, "plan", 5)
        self.face_pub = self.create_publisher(String, "oled_face", 10)   # pick-up reaction
        self.text_pub = self.create_publisher(String, "oled_text", 10)   # self-test status -> OLED
        self.test_pub = self.create_publisher(String, "selftest_result", 1)
        self.create_subscription(Odometry, g("odom_topic").value, self._on_odom, 20)
        self.create_subscription(Vector3Stamped, g("euler_topic").value, self._on_euler, 10)
        self.create_subscription(
            LaserScan, g("scan_topic").value, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "goal_pose", self._on_goal, 5)
        self.create_subscription(Bool, "go_home", self._on_go_home, 5)
        self.create_subscription(Bool, "save_map", self._on_save_map, 5)
        # per-wheel off-ground switches from the ESP32 (pick-up detection)
        self.create_subscription(Bool, "left_wheel_suspended", self._on_susp_l, 10)
        self.create_subscription(Bool, "right_wheel_suspended", self._on_susp_r, 10)
        # test override for the switches (latched so a node restarted mid-test still sees it)
        self.create_subscription(
            Int8, "pickup_override", self._on_pickup_override,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self._trait_motion:                  # personality -> motion thresholds (clamped)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, "cognition/traits", self._on_traits, latched)
        self.create_subscription(Bool, "selftest", self._on_selftest, 1)  # calibration drive
        self.create_timer(1.0 / max(1.0, float(g("control_rate").value)), self._control)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            f"slam_nav up: {self.grid.n}x{self.grid.n} grid @ {self.grid.res:.3f} m "
            f"({self.grid.n * self.grid.res:.1f} m square), match {self.match_pts} pts; "
            f"motion {'ENABLED' if self.enable_motion else 'disabled (view/plan only)'}")

    def _on_params(self, params):
        # let the web UI flip enable_motion / retune speeds live via set_parameters
        for p in params:
            if p.name == "enable_motion":
                self.enable_motion = bool(p.value)
                if not self.enable_motion:
                    self._send(0.0, 0.0)     # drop to a stop the moment it's disabled
            elif p.name == "max_lin":
                # also re-baseline the caution=0 end so a manual UI tune sticks even
                # with trait_motion on (else the next /cognition/traits tick would
                # recompute max_lin from the stale _base_max_lin and stomp this).
                self.max_lin = self._base_max_lin = float(p.value)
            elif p.name == "max_ang":
                self.max_ang = float(p.value)
            elif p.name == "stop_distance":
                self.stop_distance = self._base_stop = float(p.value)
            elif p.name == "robot_radius":
                self.robot_radius = float(p.value)
            elif p.name == "auto_explore":
                self.auto_explore = bool(p.value)
                if not self.auto_explore and self._goal_is_frontier:
                    self._goal, self._path = None, []   # drop the exploration goal
                    self._goal_is_frontier = False
            elif p.name == "stuck_timeout":
                self.stuck_timeout = float(p.value)
            elif p.name == "relocalize":
                self.relocalize = bool(p.value)
                if not self.relocalize:
                    self._recovering = False
            elif p.name == "pickup_pause":
                self.pickup_pause = bool(p.value)
            elif p.name == "lds_idle_enable":
                self.lds_idle_enable = bool(p.value)
                if self.lds_idle_enable:
                    # fresh countdown on re-enable, so it doesn't immediately spin
                    # down using a stale (possibly long-elapsed) idle clock
                    self._lds_idle_ref = (self.px, self.py)
                    self._lds_idle_since = time.monotonic()
                elif not self._lds_active:
                    # was parked -> wake it back up immediately, disabling means "always on"
                    self._lds_active = True
                    self.lds_rpm_pub.publish(Float32(data=self.lds_active_rpm))
                    self.get_logger().info("LDS idle spin-down disabled — spinning up")
            elif p.name == "lds_idle_timeout":
                self.lds_idle_timeout = float(p.value)
            elif p.name == "lds_idle_rpm":
                self.lds_idle_rpm = float(p.value)
            elif p.name == "lds_active_rpm":
                self.lds_active_rpm = float(p.value)
                if self._lds_active:
                    # currently spinning -> apply the new setpoint now, not just at the
                    # next idle<->active transition, so the web slider feels live
                    self.lds_rpm_pub.publish(Float32(data=self.lds_active_rpm))
        return SetParametersResult(successful=True)

    # --- motion-prior inputs -------------------------------------------------
    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        th = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, th)

    def _on_euler(self, msg):
        self._imu_yaw = math.radians(msg.vector.z)   # /imu/euler vector.z = yaw (deg)

    def _on_susp_l(self, msg):
        self._susp_l = bool(msg.data)

    def _on_susp_r(self, msg):
        self._susp_r = bool(msg.data)

    def _on_pickup_override(self, msg):
        v = int(msg.data)
        self._susp_override = v if v in (0, 1) else -1
        self.get_logger().warning(
            "pickup override: " + {-1: "auto (real switches)", 0: "FORCED grounded",
                                   1: "FORCED lifted"}[self._susp_override])

    def _susp_eff(self):
        """Effective off-ground switch pair, honoring the /pickup_override test hook."""
        if self._susp_override == 0:
            return False, False
        if self._susp_override == 1:
            return True, True
        return self._susp_l, self._susp_r

    def _on_traits(self, msg):
        """Map the behaviour layer's `caution` trait (0..1) onto the reactive motion
        thresholds, CLAMPED to safe bounds here in the reflex node — so the cognitive
        layer influences but can never override motion safety. Higher caution => stops
        earlier + drives slower."""
        try:
            caution = float((json.loads(msg.data).get("traits") or {}).get("caution"))
        except (ValueError, TypeError, AttributeError, json.JSONDecodeError):
            return
        c = max(0.0, min(1.0, caution))
        self.stop_distance = min(max(self._base_stop + c * (self._stop_max - self._base_stop),
                                     self._base_stop), self._stop_max)
        self.max_lin = max(min(self._base_max_lin - c * (self._base_max_lin - self._max_lin_min),
                               self._base_max_lin), self._max_lin_min)

    def _on_selftest(self, msg):
        if msg.data:
            self._start_selftest()

    def _on_ticks(self, msg):       # raw cumulative encoder counts [L, R] (test only)
        if len(msg.data) >= 2:
            self._ticks = (int(msg.data[0]), int(msg.data[1]))

    def _on_imuweb(self, msg):      # /imu/web: x=|accel|, y=|gyro|, z=measured /imu/data Hz
        self._imu_accel = float(msg.vector.x)
        self._imu_gyro = float(msg.vector.y)
        self._imu_hz = float(msg.vector.z)

    # --- the SLAM step (per scan) -------------------------------------------
    def _on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        self._last_scan = (angles, ranges)        # for the reactive front-stop layer

        if not self._have_map:
            # Seed: drop the first scan straight in at the origin and prime trackers.
            self.grid.integrate((0.0, 0.0, 0.0), angles, ranges)
            self._have_map = True
            self._prev_odom, self._prev_imu = self._odom, self._imu_yaw
            self._write_map()
            return

        # Pick-up freeze: while lifted off the ground, scans are garbage (being carried),
        # so don't predict / match / integrate. Just keep the web map status fresh so it's
        # visibly "picked up"; relocalization is armed for set-down (see _control).
        if self._picked_up:
            now = time.monotonic()
            if now - self._last_write >= self._write_period:
                self._last_write = now
                self._write_map()
            return

        # Stationary skip: when odom AND IMU agree we haven't moved since the last
        # PROCESSED scan, the pose+map can't change — skip the match+integrate (the
        # bulk of parked idle CPU). The _prev_* trackers are deliberately NOT updated
        # here, so motion (or slow drift) keeps accumulating against the last processed
        # scan and full SLAM resumes on the very first scan past the thresholds — a
        # pure rotation fires via the yaw delta. Never skips while seeding (above),
        # recovering (pose uncertain), or self-testing; pose + map telemetry are still
        # re-published at the map-write cadence so TF/web readouts stay fresh.
        if (self.still_skip and not self._recovering and not self._test_active
                and self._odom is not None and self._prev_odom is not None):
            ox, oy, oth = self._odom
            pox, poy, poth = self._prev_odom
            if self.use_imu and self._imu_yaw is not None and self._prev_imu is not None:
                dth = _wrap(self._imu_yaw - self._prev_imu)
            else:
                dth = _wrap(oth - poth)
            if math.hypot(ox - pox, oy - poy) < self.still_lin and abs(dth) < self.still_ang:
                now = time.monotonic()
                if now - self._last_write >= self._write_period:
                    self._last_write = now
                    self._publish_pose()
                    self._write_map()
                return

        px, py, pth = self._predict(self.px, self.py, self.pth)

        # Refine against the map with a decimated set of valid beams.
        v = (np.isfinite(ranges) & (ranges >= self.grid.rmin) & (ranges <= self.grid.rmax))
        va, vr = angles[v], ranges[v]
        if len(vr) > self.match_pts:
            idx = np.linspace(0, len(vr) - 1, self.match_pts).astype(int)
            va, vr = va[idx], vr[idx]

        if self._recovering:
            # Lost / kidnapped: search a much WIDER window around the prior (and the control
            # loop spins us in place to vary the geometry) until a strong match snaps back.
            if len(vr) > 10:
                cand = self.grid.match((px, py, pth), va, vr, lin=self.recover_lin,
                                       ang=self.recover_ang, half=self.recover_half,
                                       refine=self.recover_refine)
                score = self.grid.score(cand, va, vr)
                self._last_score = score
                if score >= self.min_score:
                    px, py, pth = cand                 # keep snapping toward the map
                if score >= self.recover_exit_score:
                    self._recovering = False
                    self._lost_count = 0
                    self.get_logger().info(f"relocalized (score {score:.1f})")
        elif len(vr) > 10:
            cand = self.grid.match((px, py, pth), va, vr,
                                   lin=self.match_lin, ang=self.match_ang)
            # Reject a match with no real overlap (e.g. wide-open space) — trust the prior.
            score = self.grid.score(cand, va, vr)
            self._last_score = score
            if score >= self.min_score:
                px, py, pth = cand
                self._lost_count = 0
            elif (self.relocalize and not self._test_active
                  and len(vr) >= self.recover_min_beams):
                # plenty of structure in view but it doesn't match the map -> we're drifting
                self._lost_count += 1
                if self._lost_count >= self.recover_patience:
                    self._recovering = True
                    self._recover_until = time.monotonic() + self.recover_timeout
                    self.get_logger().warning(
                        f"localization lost (score {score:.1f}) — relocalizing")

        self.px, self.py, self.pth = px, py, _wrap(pth)
        # Don't fold the scan into the map while the pose is uncertain (recovering) — a
        # wrong pose would smear obstacles across the map.
        if not self._recovering:
            self.grid.integrate((self.px, self.py, self.pth), angles, ranges)
        self._publish_pose()

        # breadcrumb trail: append only when the robot has actually moved a bit (keeps the
        # ring buffer meaningful and the JSON header small).
        if self._trail is not None and (
                not self._trail or math.hypot(self.px - self._trail[-1][0],
                                              self.py - self._trail[-1][1]) > 0.05):
            self._trail.append((round(self.px, 2), round(self.py, 2)))

        now = time.monotonic()
        if now - self._last_write >= self._write_period:
            self._last_write = now
            self._write_map()
        if self._autosave_period > 0 and self.map_store and now >= self._next_autosave:
            self._next_autosave = now + self._autosave_period
            self._save_map_file(quiet=True)

    def _predict(self, px, py, pth):
        """Apply the odom/IMU motion since the last scan as the scan-match prior."""
        if self._odom is None or self._prev_odom is None:
            self._prev_odom, self._prev_imu = self._odom, self._imu_yaw
            return px, py, pth
        ox, oy, oth = self._odom
        pox, poy, poth = self._prev_odom
        # forward distance travelled in the odom frame (projected on its heading)
        ds = (ox - pox) * math.cos(poth) + (oy - poy) * math.sin(poth)
        if self.use_imu and self._imu_yaw is not None and self._prev_imu is not None:
            dth = _wrap(self._imu_yaw - self._prev_imu)
        else:
            dth = _wrap(oth - poth)
        self._prev_odom, self._prev_imu = self._odom, self._imu_yaw
        pth = _wrap(pth + dth)
        return px + ds * math.cos(pth), py + ds * math.sin(pth), pth

    # --- outputs -------------------------------------------------------------
    def _publish_pose(self):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = self.px
        ps.pose.position.y = self.py
        ps.pose.orientation.z = math.sin(self.pth * 0.5)
        ps.pose.orientation.w = math.cos(self.pth * 0.5)
        self.pose_pub.publish(ps)

    # --- navigation (Stages 2/3) --------------------------------------------
    def _on_goal(self, msg):
        self._goal = (msg.pose.position.x, msg.pose.position.y)
        self._goal_is_frontier = False           # a human/explicit goal wins over exploring
        self._next_replan = 0.0                  # plan immediately on the next tick
        self.get_logger().info(f"goal set: ({self._goal[0]:.2f}, {self._goal[1]:.2f})")

    def _on_go_home(self, _msg):
        """Return-to-origin: target the pose the robot booted at (map (0,0))."""
        self._goal = self.home
        self._goal_is_frontier = False
        self._next_replan = 0.0
        self.get_logger().info("go home: heading to map origin")

    def _on_save_map(self, _msg):
        self._save_map_file()

    def _save_map_file(self, quiet=False):
        if not self.map_store:
            if not quiet:
                self.get_logger().warning("save_map ignored: map_store param is empty")
            return
        try:
            self.grid.save(self.map_store)
            if not quiet:
                self.get_logger().info(f"map saved to {self.map_store}")
        except OSError as exc:
            self.get_logger().warning(f"map save failed: {exc}", throttle_duration_sec=10.0)

    def _control(self):
        if not self._have_map:
            return
        now = time.monotonic()

        # Pick-up + self-test + relocalization take priority over navigation.
        self._update_pickup(now)
        self._update_lds_idle(now)
        if self._picked_up:
            if self._test_active:
                self._abort_selftest("picked up")
            self._send(0.0, 0.0)                       # halt while lifted
            return
        if self._test_active:
            if not self.enable_motion:
                self._abort_selftest("motion disabled")
            else:
                self._test_step(now)
            return
        if self._recovering:
            if now > self._recover_until:
                self._recovering = False               # give up the active search...
                self._lost_count = 0                   # ...and run on the best estimate
                self._send(0.0, 0.0)
                self.get_logger().warning("relocalize timed out; using best estimate")
            else:
                self._send(0.0, self.recover_spin)     # slow in-place spin (only if motion on)
            return

        # auto-explore: when there's no goal, periodically adopt the nearest reachable
        # frontier as one (only drives if enable_motion; otherwise just shows the plan).
        if self._goal is None:
            if self.auto_explore and now >= self._next_explore:
                self._next_explore = now + self.explore_period
                self._explore_step()
            if self._goal is None:
                return

        if now >= self._next_replan:
            self._next_replan = now + self.replan_period
            path = self.grid.plan(
                (self.px, self.py), self._goal, radius_m=self.robot_radius,
                downsample=self.plan_downsample, allow_unknown=self.allow_unknown)
            self._path = path or []
            self._publish_path()                 # always publish so the UI shows the plan
            if not self._path:
                self.get_logger().warning("no path to goal", throttle_duration_sec=3.0)

        gx, gy = self._goal
        if math.hypot(gx - self.px, gy - self.py) < self.goal_tol:
            self.get_logger().info("goal reached")
            self._goal, self._path, self._goal_is_frontier = None, [], False
            self._publish_path()
            self._send(0.0, 0.0)
            return

        # reactive safety: an obstacle in the forward cone -> stop and replan around it
        if self._front_blocked():
            self._send(0.0, 0.0)
            self._next_replan = min(self._next_replan, now + 0.2)
            return

        v, w = self._pursuit()
        if self._check_stuck(now, v):
            self._send(0.0, 0.0)
            self._goal, self._path, self._goal_is_frontier = None, [], False
            self._publish_path()
            return
        self._send(v, w)

    def _update_pickup(self, now):
        """Detect lift/drop from the per-wheel off-ground switches. BOTH wheels off the
        ground = picked up -> halt + freeze SLAM (the freeze itself is in _on_scan). On
        set-down, arm relocalization so the robot re-finds itself instead of driving on a
        stale pose."""
        susp_l, susp_r = self._susp_eff()
        picked = self.pickup_pause and susp_l and susp_r
        if picked == self._picked_up:
            return
        self._picked_up = picked
        if picked:
            self._send(0.0, 0.0)
            self._set_face(self.pickup_face)
            self.get_logger().info("picked up — pausing SLAM")
        else:
            self._set_face("")                         # back to the normal dashboard
            if self.relocalize:
                self._recovering = True
                self._recover_until = now + self.recover_timeout
                self._lost_count = 0
            self.get_logger().info("set down — relocalizing")

    def _update_lds_idle(self, now):
        """Park the LDS spin motor (/lds_target_rpm -> lds_idle_rpm, usually 0) after
        lds_idle_timeout s with no reason to expect fresh scans: no active/pending goal,
        not exploring, not picked up, not actively relocalizing, and the pose hasn't
        actually moved by more than stuck_eps. Wakes it back up (lds_active_rpm) the
        instant any of those become true again. Only publishes on a state CHANGE, so a
        manual override via the web UI's LDS slider isn't fought every tick — only at
        the next genuine idle<->active transition."""
        if not self.lds_idle_enable or self.lds_idle_timeout <= 0:
            return
        moved = (self._lds_idle_ref is None or
                 math.hypot(self.px - self._lds_idle_ref[0],
                           self.py - self._lds_idle_ref[1]) > self.stuck_eps)
        needs_scans = (self._goal is not None or self.auto_explore or
                      self._picked_up or self._recovering)
        if moved or needs_scans:
            self._lds_idle_ref = (self.px, self.py)
            self._lds_idle_since = now
        want_active = needs_scans or (now - self._lds_idle_since) < self.lds_idle_timeout
        if want_active == self._lds_active:
            return
        self._lds_active = want_active
        rpm = self.lds_active_rpm if want_active else self.lds_idle_rpm
        self.lds_rpm_pub.publish(Float32(data=rpm))
        self.get_logger().info(
            f"LDS {'spinning up' if want_active else 'idle spin-down'} ({rpm:.0f} rpm)")

    def _set_face(self, mood):
        """Drive the OLED face (an alert stare while carried). Skipped entirely when
        pickup_face is empty, so it never fights the web UI's own face control."""
        if not self.pickup_face:
            return
        self.face_pub.publish(String(data=mood))

    # --- self-test / calibration (drive known motions, cross-check IMU vs encoders) ----
    def _start_selftest(self):
        """Kick off a scripted drive (forward, back, spin) that measures what the IMU and
        encoders report vs what was commanded. Needs enable_motion ON (it deliberately
        moves). Subscribes raw /wheel_ticks + /imu/web only for the duration."""
        if self._test_active:
            return
        if not self.enable_motion:
            self._publish_test("self-test aborted: enable motion first (safety)")
            self.get_logger().warning("self-test: enable motion first")
            return
        lin = min(TEST_LIN, self.max_lin)
        ang = min(TEST_ANG, self.max_ang)
        fwd = TEST_DIST / lin if lin > 0 else 2.0
        rot = TEST_TURNS * 2.0 * math.pi / ang if ang > 0 else 6.0
        self._test_seq = [
            ("still",   TEST_SETTLE, 0.0,  0.0),   # IMU at rest: gravity + zero-gyro check
            ("forward", fwd,         lin,  0.0),   # encoders: both count +, balanced; odom dist
            ("settle",  TEST_SETTLE, 0.0,  0.0),
            ("back",    fwd,        -lin,  0.0),   # encoders: signs go negative on reverse
            ("settle",  TEST_SETTLE, 0.0,  0.0),
            ("rotate",  rot,         0.0,  ang),   # IMU yaw vs odom yaw vs commanded
            ("done",    0.4,         0.0,  0.0),
        ]
        self._test_ang, self._test_rot_dur = ang, rot
        self._test_phase, self._test_entered = 0, False
        self._test_phase_t0 = time.monotonic()
        self._test_report, self._test_warn, self._test_fail = [], 0, 0
        self._m_fwd = (0, 0)
        self._recovering = False        # don't let recovery fight the test
        self._lost_count = 0
        self._ticks = None
        self._ticks_sub = self.create_subscription(
            Int64MultiArray, "wheel_ticks", self._on_ticks, 10)
        self._imuweb_sub = self.create_subscription(
            Vector3Stamped, "imu/web", self._on_imuweb, 10)
        self._test_active = True
        self._set_face("focused")
        self._oled("Self-test...")
        self.get_logger().info("self-test: started (forward/back + in-place spin)")

    def _snapshot(self):
        od = self._odom or (0.0, 0.0, 0.0)
        tk = self._ticks or (0, 0)
        yaw = self._imu_yaw if self._imu_yaw is not None else 0.0
        return {"yaw": yaw, "x": od[0], "y": od[1], "L": tk[0], "R": tk[1]}

    def _accum_rotation(self):
        """Sum wrapped per-tick yaw deltas during the spin so a full turn isn't lost to the
        +/-pi wrap — for both the IMU and the wheel-odometry heading."""
        if self._imu_yaw is None or self._odom is None:
            return
        iy, oy = self._imu_yaw, self._odom[2]
        if self._rot_prev is None:
            self._rot_prev = (iy, oy)
            return
        piy, poy = self._rot_prev
        self._rot_imu += _wrap(iy - piy)
        self._rot_odom += _wrap(oy - poy)
        self._rot_prev = (iy, oy)

    def _test_step(self, now):
        seq = self._test_seq
        if self._test_phase >= len(seq):
            self._finish_selftest()
            return
        name, dur, v, w = seq[self._test_phase]
        if not self._test_entered:                  # entering this phase: snapshot baseline
            self._test_entered = True
            self._snap = self._snapshot()
            self._rot_imu = self._rot_odom = 0.0
            self._rot_prev = None
        if name == "rotate":
            self._accum_rotation()
        if now - self._test_phase_t0 < dur:
            self._send(v, w)
            return
        self._send(0.0, 0.0)                         # leg done: measure + advance
        self._measure(name)
        self._test_phase += 1
        self._test_phase_t0 = now
        self._test_entered = False

    def _measure(self, name):
        cur = self._snapshot()
        s = self._snap
        R = self._test_report
        if name == "still":
            a, g, hz = self._imu_accel, self._imu_gyro, self._imu_hz
            if hz <= 0:
                R.append("IMU: not publishing (/imu/web silent) -> FAIL")
                self._test_fail += 1
            else:
                bad = []
                if not (9.0 <= a <= 10.6):
                    bad.append("accel != ~9.81"); self._test_warn += 1
                if g > 0.08:
                    bad.append("gyro != ~0"); self._test_warn += 1
                R.append(f"IMU still: |a|={a:.2f} m/s2, |w|={g:.3f} rad/s, {hz:.0f} Hz -> "
                         + (", ".join(bad) if bad else "OK"))
        elif name == "forward":
            dist = math.hypot(cur["x"] - s["x"], cur["y"] - s["y"])
            dL, dR = cur["L"] - s["L"], cur["R"] - s["R"]
            self._m_fwd = (dL, dR)
            line = f"FWD: odom {dist:.2f} m (cmd ~{TEST_DIST:.2f}); ticks L={dL:+d} R={dR:+d}"
            if dL == 0 or dR == 0:
                line += " -> ENCODER DEAD / no tick data"; self._test_fail += 1
            elif dL < 0 or dR < 0:
                line += " -> SIGN WRONG (forward should be +)"; self._test_warn += 1
            else:
                bal = dR / dL
                if 0.7 <= bal <= 1.4:
                    line += f" -> OK (R/L={bal:.2f})"
                else:
                    line += f" -> WHEEL IMBALANCE (R/L={bal:.2f})"; self._test_warn += 1
            R.append(line)
        elif name == "back":
            dL, dR = cur["L"] - s["L"], cur["R"] - s["R"]
            line = f"REV: ticks L={dL:+d} R={dR:+d}"
            if dL >= 0 or dR >= 0:
                line += " -> SIGN WRONG (reverse should be -)"; self._test_warn += 1
            else:
                fL = self._m_fwd[0]
                sym = abs(dL) / fL if fL else 0.0
                line += f" -> direction OK (|rev/fwd|={sym:.2f})"
            R.append(line)
        elif name == "rotate":
            imu_d = math.degrees(self._rot_imu)
            odo_d = math.degrees(self._rot_odom)
            cmd_d = math.degrees(self._test_ang * self._test_rot_dur)
            line = f"SPIN: cmd {cmd_d:+.0f}deg, IMU {imu_d:+.0f}, odom {odo_d:+.0f}"
            if abs(imu_d) < 0.4 * abs(cmd_d):
                line += " -> IMU YAW NOT TRACKING"; self._test_fail += 1
            elif abs(odo_d) > 5.0:
                ratio = imu_d / odo_d
                if 0.85 <= ratio <= 1.18:
                    line += f" -> OK (IMU/odom={ratio:.2f})"
                else:
                    line += (f" -> MISMATCH (IMU/odom={ratio:.2f}; "
                             f"set wheel_separation*{ratio:.2f} to match IMU)")
                    self._test_warn += 1
            R.append(line)
        # "settle" / "done" legs: nothing to measure

    def _finish_selftest(self):
        self._send(0.0, 0.0)
        self._test_active = False
        self._test_entered = False
        for sub in (self._ticks_sub, self._imuweb_sub):
            if sub is not None:
                self.destroy_subscription(sub)
        self._ticks_sub = self._imuweb_sub = None
        verdict = "FAIL" if self._test_fail else ("WARN" if self._test_warn else "PASS")
        self._test_report.append(
            f"=== {verdict} (fail {self._test_fail}, warn {self._test_warn}) ===")
        for ln in self._test_report:
            self.get_logger().info("selftest: " + ln)
        self._publish_test("\n".join(self._test_report))
        self._oled(f"Self-test: {verdict}")
        self._set_face("")

    def _abort_selftest(self, reason):
        if not self._test_active:
            return
        self._send(0.0, 0.0)
        self._test_report.append(f"ABORTED: {reason}")
        self._test_fail += 1
        self._finish_selftest()

    def _publish_test(self, text):
        self.test_pub.publish(String(data=text))

    def _oled(self, text):
        self.text_pub.publish(String(data=text))

    def _explore_step(self):
        """Adopt the nearest *reachable* frontier as the goal (auto-exploration). Tries the
        nearest-first candidate list and keeps the first one the planner can actually reach,
        so a frontier tucked behind a wall is skipped rather than stalling exploration."""
        for fx, fy in self.grid.frontiers((self.px, self.py), radius_m=self.robot_radius,
                                          downsample=self.plan_downsample):
            path = self.grid.plan((self.px, self.py), (fx, fy), radius_m=self.robot_radius,
                                  downsample=self.plan_downsample,
                                  allow_unknown=self.allow_unknown)
            if path:
                self._goal, self._path, self._goal_is_frontier = (fx, fy), path, True
                self._next_replan = time.monotonic() + self.replan_period
                self._publish_path()
                self.get_logger().info(f"explore: frontier ({fx:.2f}, {fy:.2f})",
                                       throttle_duration_sec=2.0)
                return
        self.get_logger().info("explore: no reachable frontier (map complete?)",
                               throttle_duration_sec=10.0)

    def _check_stuck(self, now, v):
        """True if we've been commanding forward motion but the pose hasn't advanced for
        `stuck_timeout` s — a cheap watchdog for a wedged wheel / unsensed collision."""
        if self.stuck_timeout <= 0 or not self.enable_motion or v <= 0.02:
            self._stuck_ref = None
            return False
        if self._stuck_ref is None:
            self._stuck_ref, self._stuck_since = (self.px, self.py), now
            return False
        if math.hypot(self.px - self._stuck_ref[0],
                      self.py - self._stuck_ref[1]) > self.stuck_eps:
            self._stuck_ref, self._stuck_since = (self.px, self.py), now   # made progress
            return False
        if now - self._stuck_since > self.stuck_timeout:
            self.get_logger().warning("stuck: commanded motion but pose not advancing — "
                                      "aborting goal")
            self._stuck_ref = None
            return True
        return False

    def _front_blocked(self):
        if self._last_scan is None:
            return False
        ang, rng = self._last_scan
        fwd = np.abs(np.arctan2(np.sin(ang), np.cos(ang))) < self.front_angle
        r = rng[fwd]
        r = r[np.isfinite(r) & (r > 0.05)]
        return r.size > 0 and float(r.min()) < self.stop_distance

    def _pursuit(self):
        if not self._path:
            return 0.0, 0.0
        # lookahead target = first waypoint at least `lookahead` away (else the last)
        tx, ty = self._path[-1]
        for (x, y) in self._path:
            if math.hypot(x - self.px, y - self.py) >= self.lookahead:
                tx, ty = x, y
                break
        err = _wrap(math.atan2(ty - self.py, tx - self.px) - self.pth)
        dgoal = math.hypot(self._goal[0] - self.px, self._goal[1] - self.py)
        v = 0.0 if abs(err) > 0.6 else self.max_lin    # rotate in place if facing away
        v = min(v, self.max_lin * max(0.25, dgoal / 0.5))   # ease off near the goal
        w = max(-self.max_ang, min(self.max_ang, 1.5 * err))
        return v, w

    def _send(self, v, w):
        if not self.enable_motion:               # view/plan-only mode: never drive
            return
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def _publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for (x, y) in self._path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _write_map(self):
        if self._occ_rev != self.grid.rev:
            self._occ_bytes = self.grid.occupancy_int8().tobytes()
            self._cov_cache = self.grid.coverage()
            self._occ_rev = self.grid.rev
        seen_frac, free_m2, _occ_m2 = self._cov_cache
        mode = ("explore" if self._goal_is_frontier else
                "goal" if self._goal is not None else "idle")
        loc = ("picked up" if self._picked_up else
               "relocalizing" if self._recovering else
               "lost" if self._lost_count else "ok")
        meta = {
            "w": self.grid.n, "h": self.grid.n, "res": self.grid.res,
            "ox": self.grid.origin, "oy": self.grid.origin,
            "px": self.px, "py": self.py, "pth": self.pth,
            # --- telemetry the web map panel renders (all cheap to compute) ---
            "hx": self.home[0], "hy": self.home[1],          # home marker
            "seen": round(seen_frac, 3),                     # fraction of grid observed
            "free_m2": round(free_m2, 1),                    # mapped free area
            "score": round(self._last_score, 1),             # scan-match quality
            "mode": mode, "loc": loc, "motion": self.enable_motion,
            "trail": list(self._trail) if self._trail else [],
        }
        header = (json.dumps(meta) + "\n").encode()
        tmp = MAP_FILE + ".tmp"
        try:
            with open(tmp, "wb") as f:
                f.write(header)
                f.write(self._occ_bytes)
            os.replace(tmp, MAP_FILE)        # atomic: the server never reads a torn file
        except OSError as exc:
            self.get_logger().warning(f"map write failed: {exc}", throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = NavNode()
    _sd_notify("READY=1")
    node.create_timer(5.0, lambda: _sd_notify("WATCHDOG=1"))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
