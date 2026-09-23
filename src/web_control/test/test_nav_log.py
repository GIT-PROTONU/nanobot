"""Offline tests (ROS-free) for the SLAM/Nav event log (2026-09-23).

The "click a goal on the map → status stuck on planning → robot never moves"
incident needed two things: the fix itself (per-status LDS trust windows — see
test_lds_idle.py) and VISIBILITY. TelemetryHub now keeps a bounded ring of
nav-chain events — goal publishes/cancels, action-status transitions with
durations, the lidar idle controller's wake/park decisions, motion start/stop,
planning-stuck warnings — served via GET /nav/log and mirrored to the app log
(journald). What is pinned here:

    * _navlog_add / get_navlog: sequential ids, since-filtering, ring cap
    * note_goal / publish_json: every goal publish is logged with a timestamp
    * _on_goal_status: idle→planning→navigating→terminal transitions with
      durations, terminal "goal reached/FAILED in Xs", mirror cleared
    * _on_cmd_vel: moving/stopped transitions logged ONCE (not per message)
    * _lds_ctrl_tick: wake/park transitions logged with their reason; the
      planning-stuck watchdog warns (rate-limited) on a stuck plan

    pixi run test
"""
import time

from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus, GoalStatusArray

from web_control.telemetry import NAVLOG_MAX, TelemetryHub


class _FakeLog:
    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


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
    _face_pub = _FakePub()

    def __init__(self, params=None):
        self.params = dict(params or {})

    def get_logger(self):
        return _FakeLog()

    def get_parameter(self, name):
        if name not in self.params:
            raise KeyError(name)
        from types import SimpleNamespace
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, type_, name, qos):
        return _FakePub()

    def create_client(self, srv, name):
        return _FakeClient()

    def create_subscription(self, type_, name, cb, qos):
        return object()

    def create_timer(self, period, cb):
        pass


def _hub(params=None):
    return TelemetryHub(_FakeNode(params or {}))


def _twist(lin=0.0, ang=0.0):
    t = Twist()
    t.linear.x = lin
    t.angular.z = ang
    return t


def _status_arr(code):
    """One-entry GoalStatusArray like bt_navigator appends per transition."""
    a = GoalStatusArray()
    gs = GoalStatus()
    gs.status = code
    a.status_list = [gs]
    return a


def _msgs(hub):
    """The log texts (newest last)."""
    return [e["msg"] for e in hub.get_navlog()]


def _levels(hub):
    return [e["lvl"] for e in hub.get_navlog()]


# ---- the ring: ids, since-filter, cap ------------------------------------------

def test_navlog_sequential_ids_and_since_filter():
    hub = _hub()
    hub._navlog_add("one")
    hub._navlog_add("two", "warn")
    hub._navlog_add("three")
    entries = hub.get_navlog()
    assert [e["id"] for e in entries] == [1, 2, 3]
    assert [e["msg"] for e in entries] == ["one", "two", "three"]
    assert entries[1]["lvl"] == "warn"
    later = hub.get_navlog(2)
    assert [e["msg"] for e in later] == ["three"]
    assert hub.get_navlog(99) == []


def test_navlog_entries_are_json_safe():
    # the entries ride GET /nav/log as JSON — every field must be a plain type
    hub = _hub()
    hub._navlog_add("x", "warn")
    e = hub.get_navlog()[0]
    assert isinstance(e["id"], int) and isinstance(e["t"], float)
    assert isinstance(e["lvl"], str) and isinstance(e["msg"], str)


def test_navlog_ring_is_bounded():
    hub = _hub()
    for i in range(NAVLOG_MAX + 50):
        hub._navlog_add(f"e{i}")
    entries = hub.get_navlog()
    assert len(entries) == NAVLOG_MAX
    assert entries[-1]["msg"] == f"e{NAVLOG_MAX + 49}"     # newest kept
    assert entries[0]["id"] == 51                          # oldest dropped


# ---- goal publishes --------------------------------------------------------------

def test_note_goal_logs_a_goal_entry():
    hub = _hub()
    hub.note_goal(1.234, -2.5)
    msgs = _msgs(hub)
    assert len(msgs) == 1
    assert "(1.23, -2.50)" in msgs[0] and "planning" in msgs[0]
    assert hub._goal_published_at is not None


def test_note_goal_skill_source_is_named():
    hub = _hub()
    hub.note_goal(0.0, 0.0, source="skill go-to 'kitchen'")
    assert "skill go-to 'kitchen'" in _msgs(hub)[0]


def test_publish_json_goal_logs_through_note_goal():
    hub = _hub()
    r = hub.publish_json({"topic": "/goal_pose", "value": {"x": 2.0, "y": 3.0}})
    assert r["status"] == "ok"
    assert len(_msgs(hub)) == 1 and "goal" in _msgs(hub)[0]


def test_clear_goal_logs_only_when_a_goal_existed():
    hub = _hub()
    hub.clear_goal()                       # nothing pending — no entry
    assert _msgs(hub) == []
    hub.note_goal(1.0, 1.0)
    hub.clear_goal()
    msgs = _msgs(hub)
    assert len(msgs) == 2 and "cancelled" in msgs[-1]
    assert hub._goal is None and hub._goal_published_at is None


def test_clear_map_logs():
    hub = _hub()
    hub.clear_map()
    assert len(_msgs(hub)) == 1 and "map cleared" in _msgs(hub)[0]


# ---- action-status transitions ----------------------------------------------------

def test_status_transitions_logged_with_durations():
    hub = _hub()
    hub._on_goal_status(_status_arr(1))    # planning
    time.sleep(0.02)
    hub._on_goal_status(_status_arr(2))    # navigating
    time.sleep(0.02)
    hub._on_goal_status(_status_arr(4))    # arrived
    msgs = _msgs(hub)
    assert len(msgs) == 3
    assert "nav → planning" in msgs[0]
    assert "was planning for" in msgs[1]
    assert "nav → arrived" in msgs[2] and "was navigating for" in msgs[2]


def test_terminal_goal_logs_total_duration_and_clears_mirror():
    hub = _hub()
    hub.note_goal(1.0, 2.0)
    time.sleep(0.02)
    hub._on_goal_status(_status_arr(4))
    msgs = _msgs(hub)
    assert "goal reached in" in msgs[-1]
    assert hub._goal is None and hub._goal_published_at is None


def test_failed_goal_is_a_warning():
    hub = _hub()
    hub.note_goal(1.0, 2.0)
    hub._on_goal_status(_status_arr(6))
    assert _levels(hub)[-1] == "warn"
    assert "FAILED" in _msgs(hub)[-1]


def test_repeated_same_status_is_not_logged_again():
    # the status topic re-arrives on every goal while a plan churns — only
    # VALUE transitions may log, or the ring fills with duplicates
    hub = _hub()
    hub._on_goal_status(_status_arr(1))
    hub._on_goal_status(_status_arr(1))
    hub._on_goal_status(_status_arr(1))
    assert len(_msgs(hub)) == 1


def test_goal_canceled_status():
    hub = _hub()
    hub.note_goal(1.0, 2.0)
    hub._on_goal_status(_status_arr(5))
    msgs = _msgs(hub)
    assert any("canceled" in m for m in msgs)
    assert hub._goal is None


# ---- motion transitions -------------------------------------------------------------

def test_cmd_vel_motion_transition_logged_once():
    hub = _hub()
    for _ in range(5):                     # 5 Hz nav stream — one "moving" line only
        hub._on_cmd_vel(_twist(0.18, 0.0))
    msgs = _msgs(hub)
    assert len(msgs) == 1 and "moving" in msgs[0]
    for _ in range(5):                     # keepalive {0,0} re-asserts — one "stopped"
        hub._on_cmd_vel(_twist(0.0, 0.0))
    msgs = _msgs(hub)
    assert len(msgs) == 2 and "stopped" in msgs[1]
    hub._on_cmd_vel(_twist(0.0, 0.0))      # still quiet — nothing new
    assert len(_msgs(hub)) == 2


# ---- lidar wake/park reasons + planning watchdog -------------------------------------

def test_tick_wake_park_logged_with_reason():
    hub = _hub({"lds_idle_secs": 60.0})
    hub._lds_ctrl_tick()                   # boot → park
    msgs = _msgs(hub)
    assert len(msgs) == 1 and "park" in msgs[0] and "no motion since boot" in msgs[0]
    hub._last_move_at = time.monotonic() - 2.0
    hub._lds_ctrl_tick()                   # wake on recent motion
    msgs = _msgs(hub)
    assert len(msgs) == 2 and "wake" in msgs[1] and "motion" in msgs[1]


def test_planning_watchdog_warns_on_stuck_plan():
    hub = _hub({"lds_idle_secs": 60.0})
    hub.note_goal(1.0, 2.0)
    hub._goal_status_since = time.monotonic() - 45.0    # stuck a while
    hub._navlog_warn_at = 0.0
    hub._lds_ctrl_tick()
    warns = [e for e in hub.get_navlog() if e["lvl"] == "warn"]
    assert len(warns) == 1 and "planning stuck" in warns[0]["msg"]


def test_planning_watchdog_is_rate_limited():
    hub = _hub({"lds_idle_secs": 60.0})
    hub.note_goal(1.0, 2.0)
    hub._goal_status_since = time.monotonic() - 45.0
    hub._navlog_warn_at = 0.0
    hub._lds_ctrl_tick()
    hub._lds_ctrl_tick()                   # 1 s later — still inside NAVLOG_WARN_PERIOD
    warns = [e for e in hub.get_navlog() if e["lvl"] == "warn"]
    assert len(warns) == 1


def test_no_watchdog_when_not_planning():
    hub = _hub({"lds_idle_secs": 60.0})
    hub._lds_ctrl_tick()                   # idle park, no goal — park log only
    assert not [e for e in hub.get_navlog() if e["lvl"] == "warn"]


def test_watchdog_names_the_fresh_map_case():
    # A goal published while nano-nav was still activating: bt_navigator never
    # accepts, the status stays "planning" locally — but /map IS fresh, so the
    # "slam needs scans" message would be misleading. Point at nav instead.
    hub = _hub({"lds_idle_secs": 60.0})
    hub.note_goal(1.0, 2.0)
    hub._goal_status_since = time.monotonic() - 45.0
    hub._map_arrival = time.monotonic() - 3.0          # live slam feed
    hub._navlog_warn_at = 0.0
    hub._lds_ctrl_tick()
    warns = [e for e in hub.get_navlog() if e["lvl"] == "warn"]
    assert len(warns) == 1 and "not being processed" in warns[0]["msg"]
    assert "nano-slam" not in warns[0]["msg"]