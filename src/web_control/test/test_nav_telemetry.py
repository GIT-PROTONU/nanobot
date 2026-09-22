"""Offline tests (ROS-free) for the web gateway's Nav2-facing telemetry — the goal
mirror + status chip the web Map view rides on, the /map HTTP payload slam_toolbox's
grid reaches the browser through, and the Nav2 costmap overlays (/local_costmap +
/global_costmap) the Costmap toggle rides on. The SLAM/Nav2 stack is stock C++
upstream; what is ours is exactly this glue:

    * NAV_STATUS: action GoalStatus code -> the web chip word
    * _mk_goal: browser clicks are clamped to Nav2's 24x24 m global costmap
    * _on_map: the ONE cached OccupancyGrid copy (a degenerate/truncated grid must
      be skipped so a half-written publish can't poison the browser renderer)
    * _on_costmap: the local/global costmap caches — degenerate grids skipped,
      and the ODOM-frame local costmap re-projected into the map frame (origin
      composed with TF map->odom at the grid's stamp, yaw carried for the page)
    * note_goal / clear_goal / _on_goal_status: goal mirror + chip state; terminal
      states drop the mirror so the goal ring can't resurrect every frame
    * bytes-status paranoia: rmw_zenoh can hand int8 fields over as bytes (the
      DiagnosticStatus.level bug class) — must normalize to int, not crash

    pixi run test
"""
import math

import pytest

from geometry_msgs.msg import PoseStamped, Transform, TransformStamped, Vector3, Quaternion
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatus, GoalStatusArray

from web_control.telemetry import GOAL_MAX_ABS_M, NAV_STATUS, STALE, TelemetryHub


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
    """The sliver of rclpy Node surface TelemetryHub.__init__ touches. No spin,
    no executor, no graph — the timers it creates never fire here."""

    _face_pub = _FakePub()

    def get_logger(self):
        return _FakeLog()

    def get_parameter(self, name):
        raise KeyError(name)     # the hub falls back to its module-level defaults

    def create_publisher(self, type_, name, qos):
        return _FakePub()

    def create_client(self, srv, name):
        return _FakeClient()

    def create_subscription(self, type_, name, cb, qos):
        return object()          # the always-on controller subs (cmd_vel, goal status)

    def create_timer(self, period, cb):
        pass


def _hub():
    return TelemetryHub(_FakeNode())


def _grid(w, h, cells=None, res=0.05):
    g = OccupancyGrid()
    g.info.width = w
    g.info.height = h
    g.info.resolution = res
    g.info.origin.position.x = -2.0
    g.info.origin.position.y = -3.0
    g.data = list(cells) if cells is not None else [0] * (w * h)
    return g


def _status_arr(code):
    """One-entry GoalStatusArray like bt_navigator appends per transition."""
    a = GoalStatusArray()
    gs = GoalStatus()
    gs.status = code
    a.status_list = [gs]
    return a


class _BytesGoalStatus:
    """Stands in for a deserialized GoalStatus whose int8 `status` arrived as
    bytes (rmw_zenoh) — rclpy's message validation refuses to BUILD that shape,
    but the wire can deliver it (the DiagnosticStatus.level bug class)."""

    def __init__(self, raw):
        self.status = raw


def _fake_arr(*entries):
    class _Arr:
        pass
    a = _Arr()
    a.status_list = list(entries)
    return a


# ---- NAV_STATUS mapping ---------------------------------------------------------
def test_nav_status_covers_every_goal_status_code():
    assert NAV_STATUS[0] == "idle"          # STATUS_UNKNOWN
    assert NAV_STATUS[1] == "planning"      # STATUS_ACCEPTED
    assert NAV_STATUS[2] == "navigating"    # STATUS_EXECUTING
    assert NAV_STATUS[3] == "canceling"     # STATUS_CANCELING
    assert NAV_STATUS[4] == "arrived"       # STATUS_SUCCEEDED
    assert NAV_STATUS[5] == "idle"          # STATUS_CANCELED -> goal gone -> idle chip
    assert NAV_STATUS[6] == "failed"        # STATUS_ABORTED


def test_nav_status_terminal_codes_match_goal_clearing_set():
    # the codes that clear the goal mirror must be exactly the terminal ones
    terminal = {c for c, word in NAV_STATUS.items() if c >= 4}
    assert terminal == {4, 5, 6}


# ---- _mk_goal clamping ----------------------------------------------------------
def test_mk_goal_inside_costmap_passes_through():
    m = TelemetryHub._mk_goal({"x": 3.25, "y": -4.5})
    assert isinstance(m, PoseStamped)
    assert m.header.frame_id == "map"
    assert m.pose.position.x == pytest.approx(3.25)
    assert m.pose.position.y == pytest.approx(-4.5)
    assert m.pose.orientation.w == pytest.approx(1.0)   # identity orientation


def test_mk_goal_clamps_to_global_costmap():
    # a goal outside the 24x24 m costmap can never be planned — clamp, don't fail
    m = TelemetryHub._mk_goal({"x": 999.0, "y": -999.0})
    assert m.pose.position.x == pytest.approx(GOAL_MAX_ABS_M)
    assert m.pose.position.y == pytest.approx(-GOAL_MAX_ABS_M)
    assert GOAL_MAX_ABS_M == 12.0                       # half the costmap width


def test_mk_goal_rejects_non_numeric():
    with pytest.raises((TypeError, ValueError)):
        TelemetryHub._mk_goal({"x": "north", "y": 0})
    with pytest.raises(KeyError):
        TelemetryHub._mk_goal({"y": 1.0})


# ---- /map payload cache ---------------------------------------------------------
def test_get_map_payload_is_none_before_first_map():
    assert _hub().get_map_payload() == (None, None)


def test_on_map_caches_grid_atomically():
    h = _hub()
    h._on_map(_grid(2, 3, cells=[0, 100, -1, 0, 50, 100], res=0.05))
    meta, cells = h.get_map_payload()
    assert meta["w"] == 2 and meta["h"] == 3
    assert meta["res"] == pytest.approx(0.05)
    assert meta["ox"] == pytest.approx(-2.0) and meta["oy"] == pytest.approx(-3.0)
    assert isinstance(meta["t"], float) and meta["t"] > 0
    assert len(cells) == 6
    assert cells[1] == 100                              # occupied
    assert cells[2] == 255                              # int8 -1 (unknown) mod 256
    assert cells[4] == 50


def test_on_map_skips_degenerate_grid():
    h = _hub()
    h._on_map(_grid(0, 0))                              # 0-sized
    assert h.get_map_payload() == (None, None)
    h._on_map(_grid(3, 3, cells=[0] * 4))               # truncated (len != w*h)
    assert h.get_map_payload() == (None, None)


def test_on_map_skips_truncated_but_keeps_last_good():
    h = _hub()
    h._on_map(_grid(2, 2, cells=[0, 0, 0, 100]))
    meta_good, _ = h.get_map_payload()
    h._on_map(_grid(2, 2, cells=[0, 0, 0]))             # torn publish
    meta_after, _ = h.get_map_payload()
    assert meta_after is meta_good                      # last good copy survives


def test_on_map_replaces_whole_payload():
    h = _hub()
    h._on_map(_grid(2, 2))
    meta1, _ = h.get_map_payload()
    h._on_map(_grid(4, 4))
    meta2, cells = h.get_map_payload()
    assert meta2["w"] == 4 and len(cells) == 16
    assert meta2 is not meta1


# ---- costmap payload caches (/local_costmap + /global_costmap) -------------------
class _FakeTf:
    """lookup_transform stand-in returning one fixed map->odom transform, or
    failing (the TF-chain-down case the callback must survive)."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, fail=False):
        self.x, self.y, self.yaw, self.fail = x, y, yaw, fail
        self.calls = []

    def lookup_transform(self, target, source, stamp):
        self.calls.append((target, source, stamp))
        if self.fail:
            raise Exception("no tf")
        inner = Transform()
        inner.translation = Vector3(x=self.x, y=self.y, z=0.0)
        q = Quaternion()
        q.w = math.cos(self.yaw / 2.0)
        q.z = math.sin(self.yaw / 2.0)
        inner.rotation = q
        ts = TransformStamped()
        ts.transform = inner
        return ts


def _costmap(w, h, cells=None, res=0.05):
    return _grid(w, h, cells=cells, res=res)


def test_costmap_payloads_are_none_before_first_grid():
    h = _hub()
    assert h.get_local_costmap_payload() == (None, None)
    assert h.get_global_costmap_payload() == (None, None)


def test_on_costmap_global_passes_origin_through():
    # the global costmap IS the map frame: origin as published, yaw 0
    h = _hub()
    h._on_costmap("_global_costmap_payload", "map",
                  _costmap(2, 2, cells=[0, -2, -1, -128]))
    meta, cells = h.get_global_costmap_payload()
    assert meta["kind"] == "costmap"
    assert meta["w"] == 2 and meta["h"] == 2 and meta["res"] == pytest.approx(0.05)
    assert meta["ox"] == pytest.approx(-2.0) and meta["oy"] == pytest.approx(-3.0)
    assert meta["yaw"] == pytest.approx(0.0)            # map frame needs no rotation
    # Nav2 COST bytes served RAW (0 free .. 254 lethal, 255 unknown) — the int8
    # wire values wrap mod 256 (254 = -2, 255 = -1) exactly as on the real wire
    assert list(cells) == [0, 254, 255, 128]


def test_on_costmap_local_reprojects_origin_into_map_frame():
    # local costmap lives in odom; its origin must be composed with TF map->odom
    # and the grid yaw reported, so the page can overlay it with plain
    # map-frame coords. origin (-2,-3) rotated 90 deg = (3,-2), +(10,20) = (13,18).
    h = _hub()
    h._tf_buf = _FakeTf(x=10.0, y=20.0, yaw=math.pi / 2)
    h._on_costmap("_local_costmap_payload", "odom", _costmap(2, 2))
    meta, _ = h.get_local_costmap_payload()
    assert meta["ox"] == pytest.approx(13.0)
    assert meta["oy"] == pytest.approx(18.0)
    assert meta["yaw"] == pytest.approx(math.pi / 2)
    assert meta["kind"] == "costmap"
    # the lookup targeted the map->odom edge
    assert h._tf_buf.calls[0][:2] == ("map", "odom")


def test_on_costmap_local_without_tf_is_not_cached():
    # no browser ever connected -> no TF buffer -> cannot place the grid; a
    # mis-placed overlay is worse than none, so the update is dropped
    h = _hub()
    h._on_costmap("_local_costmap_payload", "odom", _costmap(2, 2))
    assert h.get_local_costmap_payload() == (None, None)


def test_on_costmap_local_tf_failure_keeps_last_good():
    h = _hub()
    h._tf_buf = _FakeTf(x=1.0, y=2.0)
    h._on_costmap("_local_costmap_payload", "odom", _costmap(2, 2))
    meta_good, _ = h.get_local_costmap_payload()
    h._tf_buf = _FakeTf(fail=True)                      # TF chain drops
    h._on_costmap("_local_costmap_payload", "odom", _costmap(3, 3))
    meta_after, _ = h.get_local_costmap_payload()
    assert meta_after is meta_good                      # stale-but-placed beats mis-placed


def test_on_costmap_skips_degenerate_grid():
    h = _hub()
    h._on_costmap("_global_costmap_payload", "map", _costmap(0, 0))
    assert h.get_global_costmap_payload() == (None, None)
    h._on_costmap("_global_costmap_payload", "map", _costmap(3, 3, cells=[0] * 4))
    assert h.get_global_costmap_payload() == (None, None)


# ---- feeds-health strip (Map card) ----------------------------------------------
def test_map_arrival_starts_stale_then_fresh():
    """map_age: None (never arrived) before the first /map, ~0 right after —
    the SLAM dot's contract (fresh = slam_toolbox publishing)."""
    h = _hub()
    assert h._map_arrival == STALE
    h._on_map(_grid(2, 2))
    assert h._map_arrival != STALE


def test_tf_laser_none_without_buffer():
    """No TF listener (browser never connected) → the TF dot reads down/not-
    yet rather than crashing the frame build."""
    assert _hub()._tf_laser_age() is None


def test_lds_age_tracks_arrival():
    """lds.age: null until the first /lds_* arrives, then set — the LDS dot's
    staleness signal (a dead ESP32 link leaves rpm>0 but ages the timestamp)."""
    from std_msgs.msg import Float32
    h = _hub()
    assert h._lds_at is None
    h._mk_lds("rpm")(Float32(data=299.5))
    assert h._lds_at is not None
    assert h._lds["rpm"] == pytest.approx(299.5)


# ---- goal mirror + chip ----------------------------------------------------------
def test_note_goal_records_mirror_and_planning():
    h = _hub()
    h.note_goal(1.23456, -2.34567)
    assert h._goal == [1.235, -2.346]                   # rounded for the wire
    assert h._goal_status == "planning"


def test_clear_goal_resets_mirror_and_chip():
    h = _hub()
    h.note_goal(1.0, 2.0)
    h.clear_goal()
    assert h._goal is None
    assert h._goal_status == "idle"


def test_goal_status_non_terminal_keeps_mirror():
    h = _hub()
    h.note_goal(1.0, 2.0)
    h._on_goal_status(_status_arr(2))                   # EXECUTING
    assert h._goal_status == "navigating"
    assert h._goal == [1.0, 2.0]


def test_goal_status_terminal_clears_mirror():
    h = _hub()
    for code, expected in ((4, "arrived"), (5, "idle"), (6, "failed")):
        h.note_goal(1.0, 2.0)
        h._on_goal_status(_status_arr(code))
        assert h._goal_status == expected
        assert h._goal is None                          # ring must not resurrect


def test_goal_status_empty_list_changes_nothing():
    h = _hub()
    h._on_goal_status(GoalStatusArray())                # no entries yet
    assert h._goal_status == "idle"


def test_goal_status_bytes_code_normalized():
    h = _hub()
    h.note_goal(1.0, 2.0)
    h._on_goal_status(_fake_arr(_BytesGoalStatus(b"\x06")))   # ABORTED as raw bytes
    assert h._goal_status == "failed"
    assert h._goal is None


def test_goal_status_unknown_code_falls_back_to_idle():
    h = _hub()
    h._on_goal_status(_status_arr(99))
    assert h._goal_status == "idle"


def test_status_uses_last_entry_of_the_list():
    # status_list appends chronologically; the LAST entry is the current state
    h = _hub()
    arr = GoalStatusArray()
    arr.status_list = [_status_arr(1).status_list[0], _status_arr(2).status_list[0]]
    h._on_goal_status(arr)
    assert h._goal_status == "navigating"
