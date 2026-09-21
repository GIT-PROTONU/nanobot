"""Offline tests (ROS-free) for the LDS idle spin-down controller (2026-09-21).

The old slam_nav-era _update_lds_idle died with slam_nav and NOTHING owned
/lds_target_rpm afterwards — the ESP32 held its last setpoint (boot default 300
rpm) forever. The web gateway (telemetry.py) owns the topic again, on an
always-on 1 Hz timer so the spin-down works with the page closed. What is ours
and is pinned here:

    * lds_idle_target: the pure setpoint decision — recent commanded motion
      (/cmd_vel) or a Nav2 goal in flight keeps the lidar up, a quiet stretch
      parks it, `idle_enable` off always spins, a manual latch hands the topic off
    * _on_cmd_vel: the "last commanded motion" clock (the web UI's live last-move
      timer) — the keepalive's steady {0,0} re-asserts must NOT reset it
    * _lds_ctrl_tick: publishes the decision on the ALWAYS-ON subscription signals,
      re-asserts after LDS_REASSERT_SECS (ESP32 reboot recovery), never fights a
      manual owner (slider/skill latch, IMU-test hold)
    * note_lds_manual / lds_hold: the two "another owner" mechanisms
    * note_map_clear: the POST /map/clear rebuild window — the parked lidar wakes
      for one idle period so the wiped map can actually rebuild (2026-09-21)
    * _lds_ctrl_state: the f.lds frame section (age/tgt/idle/state), jam wins

    pixi run test
"""
import time
from types import SimpleNamespace

from geometry_msgs.msg import Twist

from web_control.telemetry import (
    LDS_DEFAULT_RPM, LDS_IDLE_SECS_DEFAULT, LDS_MANUAL_SECS_DEFAULT,
    TelemetryHub, lds_idle_target,
)


class _FakeLog:
    def info(self, *a, **k):
        pass

    warning = error = info


class _FakePub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeClient:
    def service_is_ready(self):
        return False

    def call_async(self, req):
        pass


class _FakeNode:
    """Records publishers (so ticks can be observed) and holds a param dict."""

    _face_pub = _FakePub()

    def __init__(self, params=None):
        self.params = dict(params or {})
        self.pubs = {}
        self.timers = []
        self.subs = []

    def get_logger(self):
        return _FakeLog()

    def get_parameter(self, name):
        if name not in self.params:
            raise KeyError(name)
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, type_, name, qos):
        return self.pubs.setdefault(name, _FakePub())

    def create_client(self, srv, name):
        return _FakeClient()

    def create_subscription(self, type_, name, cb, qos):
        self.subs.append(name)
        return object()

    def create_timer(self, period, cb):
        self.timers.append((period, cb))


def _hub(params=None):
    return TelemetryHub(_FakeNode(params))


def _twist(lin=0.0, ang=0.0):
    t = Twist()
    t.linear.x = lin
    t.angular.z = ang
    return t


# ---- the pure setpoint decision -----------------------------------------------

def test_pure_manual_latch_hands_off():
    assert lds_idle_target(1000.0, None, 60.0, True, False, True, 300.0) is None


def test_pure_recent_motion_keeps_target():
    assert lds_idle_target(1000.0, 990.0, 60.0, True, False, False, 300.0) == 300.0


def test_pure_quiet_stretch_parks():
    assert lds_idle_target(1000.0, 900.0, 60.0, True, False, False, 300.0) == 0.0


def test_pure_no_motion_since_boot_is_idle():
    # last_move_at=None: a freshly booted, never-driven robot parks the lidar
    # instead of leaving the firmware's boot default spinning forever.
    assert lds_idle_target(1000.0, None, 60.0, True, False, False, 300.0) == 0.0


def test_pure_nav_busy_keeps_target():
    # planning/navigating both count — Nav2 needs the costmap fresh BEFORE it moves
    assert lds_idle_target(1000.0, 900.0, 60.0, True, True, False, 300.0) == 300.0


def test_pure_disabled_always_spins():
    assert lds_idle_target(1000.0, 900.0, 60.0, False, False, False, 300.0) == 300.0


def test_pure_window_boundary_exactly_idle_secs_parks():
    # strictly-less-than window: now-last == idle_secs is already idle
    assert lds_idle_target(1060.0, 1000.0, 60.0, True, False, False, 300.0) == 0.0


def test_pure_zero_active_target_stays_parked_when_active():
    # Spin slider at 0 = the user's explicit "never spin"
    assert lds_idle_target(1000.0, 990.0, 60.0, True, False, False, 0.0) == 0.0


# ---- the always-on subscriptions ----------------------------------------------

def test_controller_subs_are_always_on():
    hub = _hub()
    assert "cmd_vel" in hub._node.subs
    assert "navigate_to_pose/_action/status" in hub._node.subs


def test_controller_timer_created():
    hub = _hub()
    periods = [p for p, _ in hub._node.timers]
    assert any(abs(p - 1.0) < 1e-9 for p in periods)   # the 1 Hz controller tick


# ---- _on_cmd_vel: the last-motion clock ----------------------------------------

def test_cmd_vel_above_eps_sets_clock():
    hub = _hub()
    hub._on_cmd_vel(_twist(0.2, 0.0))
    assert hub._last_move_at is not None


def test_cmd_vel_zero_reassert_does_not_reset_clock():
    # the teleop keepalive re-publishes {0,0} after every stop — that must not
    # look like movement and keep the lidar awake forever
    hub = _hub()
    hub._on_cmd_vel(_twist(0.2, 0.0))
    first = hub._last_move_at
    hub._on_cmd_vel(_twist(0.0, 0.0))
    assert hub._last_move_at == first


def test_cmd_vel_below_eps_ignored():
    hub = _hub()      # fake node has no vision_bumper_cmd_eps -> 0.03 fallback
    hub._on_cmd_vel(_twist(0.01, 0.0))
    assert hub._last_move_at is None


def test_cmd_vel_ang_only_motion_counts():
    hub = _hub()
    hub._on_cmd_vel(_twist(0.0, 0.5))
    assert hub._last_move_at is not None


# ---- the 1 Hz tick --------------------------------------------------------------

def test_tick_parks_when_idle():
    hub = _hub({"lds_idle_secs": 60.0})
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 0.0
    assert hub._lds_sent == 0.0


def test_tick_spins_when_motion_recent():
    hub = _hub({"lds_idle_secs": 60.0})
    hub._last_move_at = time.monotonic() - 5.0
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 300.0


def test_tick_spins_when_nav_busy():
    hub = _hub({"lds_idle_secs": 60.0})
    hub._goal_status = "navigating"
    hub._goal_status_at = time.monotonic()   # fresh arrival
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 300.0


def test_stale_nav_busy_does_not_hold_the_lidar():
    # nano-nav died mid-goal: the last "navigating" status arrived ages ago and
    # nothing moves — the lidar must still park instead of spinning forever.
    hub = _hub({"lds_idle_secs": 60.0})
    hub._goal_status = "navigating"
    hub._goal_status_at = time.monotonic() - 120.0    # beyond LDS_NAV_STALE
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 0.0


def test_note_goal_refreshes_the_status_arrival():
    hub = _hub({"lds_idle_secs": 60.0})
    hub._goal_status_at = time.monotonic() - 120.0
    hub.note_goal(1.0, 2.0)          # planning, fresh arrival
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 300.0


def test_tick_no_duplicate_publish_while_unchanged():
    hub = _hub({"lds_idle_secs": 60.0})
    hub._lds_ctrl_tick()
    hub._lds_ctrl_tick()
    assert len(hub._node.pubs["lds_target_rpm"].published) == 1


def test_tick_reasserts_after_the_reassert_window():
    # an ESP32 reboot resets its setpoint to the firmware default — the periodic
    # re-assert must republish the SAME value after LDS_REASSERT_SECS
    hub = _hub({"lds_idle_secs": 60.0})
    hub._lds_ctrl_tick()
    hub._lds_sent_at = time.monotonic() - 60.0
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 2 and pubs[1].data == 0.0


def test_tick_hands_off_during_manual_latch():
    hub = _hub({"lds_idle_secs": 60.0})
    hub.note_lds_manual(200.0, set_target=True)
    hub._lds_ctrl_tick()
    # one publish exists: note_lds_manual's own tracked value isn't published by
    # the tick — but the hub never published it here, so still zero
    assert hub._node.pubs["lds_target_rpm"].published == []


def test_manual_expiry_returns_control_to_the_controller():
    hub = _hub({"lds_idle_secs": 60.0, "lds_manual_secs": 300.0})
    hub.note_lds_manual(200.0, set_target=True)
    assert hub._lds_user_rpm == 200.0
    hub._lds_manual_until = time.monotonic() - 1.0    # force the window closed
    hub._lds_ctrl_tick()                              # robot idle -> park
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 0.0


def test_hold_suspends_controller_and_refcount_floors_at_zero():
    hub = _hub({"lds_idle_secs": 60.0})
    hub.lds_hold(True)
    hub._lds_ctrl_tick()
    assert hub._node.pubs["lds_target_rpm"].published == []
    hub.lds_hold(True)
    hub.lds_hold(False)
    hub.lds_hold(False)     # extra release must not go negative
    assert hub._lds_hold == 0
    hub._lds_ctrl_tick()
    assert len(hub._node.pubs["lds_target_rpm"].published) == 1


# ---- note_lds_manual -------------------------------------------------------------

def test_manual_latch_tracks_value_for_frame():
    hub = _hub()
    assert hub.note_lds_manual(150.0) == 150.0
    assert hub._lds_sent == 150.0


def test_manual_latch_set_target_updates_active_rpm():
    hub = _hub()
    hub.note_lds_manual(150.0, set_target=True)
    assert hub._lds_user_rpm == 150.0


def test_manual_latch_without_set_target_keeps_active_rpm():
    # skill actions borrow the topic without redefining the user's spin target
    hub = _hub()
    hub._lds_user_rpm = 300.0
    hub.note_lds_manual(120.0)
    assert hub._lds_user_rpm == 300.0


def test_manual_latch_expiry_uses_the_param():
    hub = _hub({"lds_manual_secs": 12.0})
    hub.note_lds_manual(100.0)
    assert hub._lds_manual_until - time.monotonic() <= 12.0


# ---- POST /publish hook -----------------------------------------------------------

def test_publish_json_lds_latches_manual_and_target():
    hub = _hub({"lds_idle_secs": 60.0})
    r = hub.publish_json({"topic": "/lds_target_rpm", "value": 250})
    assert r["status"] == "ok"
    assert hub._lds_user_rpm == 250.0
    hub._lds_ctrl_tick()   # manual latch -> the controller adds nothing
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 250.0


def test_publish_json_bare_topic_form_latches_too():
    hub = _hub()
    r = hub.publish_json({"topic": "lds_target_rpm", "value": 0})
    assert r["status"] == "ok"
    assert hub._lds_user_rpm == 0.0


def test_publish_json_lds_clamps_to_max():
    hub = _hub()
    hub.publish_json({"topic": "/lds_target_rpm", "value": 9999})
    assert hub._lds_user_rpm == 400.0


# ---- the f.lds frame section -------------------------------------------------------

def test_ctrl_state_park_before_anything():
    hub = _hub()
    hub._lds_ctrl_tick()
    st = hub._lds_ctrl_state(time.monotonic())
    assert st["state"] == "park" and st["tgt"] == 0.0
    assert st["idle"] is None            # never commanded to move


def test_ctrl_state_spin_with_live_idle_clock():
    hub = _hub()
    hub._on_cmd_vel(_twist(0.2, 0.0))
    hub._lds_ctrl_tick()
    st = hub._lds_ctrl_state(time.monotonic())
    assert st["state"] == "spin" and st["tgt"] == 300.0
    assert 0.0 <= st["idle"] < 5.0


def test_ctrl_state_jam_wins_over_everything():
    hub = _hub()
    hub.note_lds_manual(150.0)
    hub._lds["jam"] = True
    st = hub._lds_ctrl_state(time.monotonic())
    assert st["state"] == "jam"


def test_ctrl_state_manual():
    hub = _hub()
    hub.note_lds_manual(150.0)
    st = hub._lds_ctrl_state(time.monotonic())
    assert st["state"] == "manual" and st["tgt"] == 150.0


def test_ctrl_state_hold():
    hub = _hub()
    hub.lds_hold(True)
    st = hub._lds_ctrl_state(time.monotonic())
    assert st["state"] == "hold"


def test_ctrl_state_age_tracks_arrival():
    hub = _hub()
    st = hub._lds_ctrl_state(time.monotonic())
    assert st["age"] is None             # no /lds_* ever arrived
    hub._lds_at = time.monotonic() - 2.0
    st = hub._lds_ctrl_state(time.monotonic())
    assert 1.0 < st["age"] < 3.0


# ---- boot defaults -------------------------------------------------------------------

def test_boot_uses_default_target_without_the_param():
    hub = _hub()
    assert hub._lds_user_rpm == 300.0


def test_boot_reads_lds_active_rpm_param():
    hub = _hub({"lds_active_rpm": 250.0})
    assert hub._lds_user_rpm == 250.0


# ---- the map-clear rebuild window ------------------------------------------------------

def test_map_clear_wakes_the_lidar_without_motion():
    # POST /map/clear: the fresh map can only build if scans flow, but a quiet
    # robot's lidar is parked — the rebuild window must count as recent motion.
    hub = _hub({"lds_idle_secs": 60.0})
    hub.note_map_clear()
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 300.0


def test_map_clear_window_lapses_back_to_park():
    hub = _hub({"lds_idle_secs": 60.0})
    hub.note_map_clear()
    hub._lds_rebuild_until = time.monotonic() - 1.0   # window over, still quiet
    hub._lds_ctrl_tick()
    pubs = hub._node.pubs["lds_target_rpm"].published
    assert len(pubs) == 1 and pubs[0].data == 0.0


def test_map_clear_does_not_override_a_manual_owner():
    hub = _hub({"lds_idle_secs": 60.0})
    hub.note_lds_manual(150.0, set_target=True)       # slider/skill holds the topic
    hub.note_map_clear()
    hub._lds_ctrl_tick()
    assert hub._node.pubs["lds_target_rpm"].published == []


def test_map_clear_window_uses_the_idle_secs_param():
    hub = _hub({"lds_idle_secs": 600.0})
    hub.note_map_clear()
    assert hub._lds_rebuild_until > time.monotonic() + 500.0