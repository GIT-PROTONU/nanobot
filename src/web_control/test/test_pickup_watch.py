"""Offline tests (ROS-free) for the pickup watch (lift-stop + release turn,
2026-09-28): both wheels up for pickup_stop_secs -> latch a STOP (web_server
zeroes the drive, cancels maneuvers + Nav2 goals, refuses new motion); both
wheels grounded again for the confirm window -> SPIN (a ~180° re-orientation
turn via the canned-move machinery).

Two contracts pinned here:

  * `PickupWatch` (web_control/pickup_watch.py, pure) — the debounce/latch
    machine: STOP once per latch, SPIN once per release, one-wheel samples
    reset both timers, the /pickup_override hook forces the pair, `up_for`
    feeds the frame.
  * `WebServerNode._persist_pickup_params` (the lds.json pattern) — the
    Coprocessor card's toggle + slider survive a restart: only the cluster's
    params write pickup.json, proposed values overlay the snapshot, the
    callback never vetoes/raises.

    pixi run test
"""
import json
import time
from types import SimpleNamespace

from web_control.telemetry import PICKUP_SPIN_GIVEUP, TelemetryHub
import web_control.web_server as ws
from web_control.pickup_watch import PickupWatch, STOP, SPIN
from web_control.web_server import PICKUP_PERSIST_KEYS


def _events(watch, samples, stop_secs=5.0, release_secs=1.0):
    """Run [(l, r, override, now), ...] through the watch; return every event."""
    out = []
    for l, r, ov, now in samples:
        out += watch.update(l, r, ov, now, stop_secs, release_secs)
    return out


def test_no_latch_before_stop_secs():
    w = PickupWatch()
    assert _events(w, [(True, True, -1, t) for t in range(0, 5)]) == []
    assert w.latched is False


def test_latches_at_stop_secs_once():
    w = PickupWatch()
    # 1 Hz heartbeat-style samples (on-change + 1 Hz re-publish, like the ESP32)
    samples = [(True, True, -1, 0.0 + 0.5 * i) for i in range(12)]
    events = _events(w, samples)
    assert events == [STOP]
    assert w.latched is True
    assert _events(w, samples, stop_secs=5.0) == []      # idempotent while held


def test_one_wheel_down_resets_the_timer():
    w = PickupWatch()
    samples = ([(True, True, -1, 0.5 * i) for i in range(8)]      # 3.5 s up
               + [(True, False, -1, 4.0)]                          # one wheel down
               + [(True, True, -1, 4.5 + 0.5 * i) for i in range(6)])
    assert _events(w, samples) == []
    assert w.latched is False


def test_partial_up_never_latches():
    w = PickupWatch()
    samples = [(True, False, -1, 0.5 * i) for i in range(40)]     # 20 s, one wheel
    assert _events(w, samples) == []


def test_release_needs_confirm_then_spins_once():
    w = PickupWatch()
    assert _events(w, [(True, True, -1, 0.5 * i) for i in range(12)]) == [STOP]
    # grounded for less than the confirm window: nothing yet
    assert _events(w, [(False, False, -1, 6.0), (False, False, -1, 6.5)]) == []
    assert w.latched is True
    # confirm reached: exactly one SPIN, latch cleared
    assert _events(w, [(False, False, -1, 7.2)]) == [SPIN]
    assert w.latched is False
    assert _events(w, [(False, False, -1, 8.0)]) == []            # no re-fire


def test_one_wheel_down_during_release_holds_the_latch():
    w = PickupWatch()
    _events(w, [(True, True, -1, 0.5 * i) for i in range(12)])
    assert _events(w, [(False, False, -1, 6.0), (False, True, -1, 6.5),
                       (False, False, -1, 30.0)]) == []
    assert w.latched is True                                       # still held


def test_override_forces_the_pair():
    w = PickupWatch()
    # override 1 = force lifted even though the real switches read grounded
    assert _events(w, [(False, False, 1, 0.5 * i) for i in range(12)]) == [STOP]
    assert w.latched is True
    # override 0 = force grounded -> release fires through the confirm window
    assert _events(w, [(True, True, 0, 6.0 + 0.5 * i) for i in range(4)]) == [SPIN]
    assert w.latched is False


def test_override_half_states_reset_both_timers():
    w = PickupWatch()
    # override -1 with one real wheel up (carried tilted): neither up nor down
    assert _events(w, [(True, False, -1, 0.5 * i) for i in range(40)]) == []
    assert w.latched is False


def test_re_pickup_after_release_works_again():
    w = PickupWatch()
    assert _events(w, [(True, True, -1, 0.5 * i) for i in range(12)]) == [STOP]
    assert _events(w, [(False, False, -1, 6.5 + 0.5 * i) for i in range(3)]) == [SPIN]
    assert _events(w, [(True, True, -1, 8.0 + 0.5 * i) for i in range(12)]) == [STOP]
    assert _events(w, [(False, False, -1, 14.5 + 0.5 * i) for i in range(3)]) == [SPIN]


def test_up_for_tracks_the_timer():
    w = PickupWatch()
    assert w.up_for(10.0) == 0.0
    w.update(False, False, -1, 10.0, 5.0)
    assert w.up_for(10.0) == 0.0
    w.update(True, True, -1, 10.5, 5.0)
    assert w.up_for(12.0) == 1.5
    w.update(True, False, -1, 12.5, 5.0)          # one wheel down -> 0 again
    assert w.up_for(13.0) == 0.0


def test_reset_stands_down_quietly():
    w = PickupWatch()
    _events(w, [(True, True, -1, 0.5 * i) for i in range(12)])
    assert w.latched is True
    w.reset()
    assert w.latched is False
    assert _events(w, [(False, False, -1, 7.0), (False, False, -1, 8.0)]) == []


# ---- persistence callback (the lds.json pattern) -------------------------------
class _Param:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class _FakeLog:
    def __init__(self):
        self.warnings = []

    def warning(self, msg, *a, **k):
        self.warnings.append(msg)


def _node(tmp_path, values):
    n = ws.WebServerNode.__new__(ws.WebServerNode)
    n._pickup_path = str(tmp_path / "pickup.json")
    n._pickup_settings_file = lambda: n._pickup_path
    n.get_parameter = lambda name: _Param(name, dict(values)[name])
    n.get_logger = lambda: _LOG
    return n


_LOG = _FakeLog()


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def test_unrelated_params_write_nothing(tmp_path):
    n = _node(tmp_path, {"pickup_stop_enable": True, "pickup_stop_secs": 5.0,
                         "pickup_spin_deg": 180.0})
    assert n._persist_pickup_params([_Param("lds_idle_secs", 61.0)]).successful
    assert not (tmp_path / "pickup.json").exists()


def test_cluster_write_snapshots_whole_cluster(tmp_path):
    n = _node(tmp_path, {"pickup_stop_enable": True, "pickup_stop_secs": 5.0,
                         "pickup_spin_deg": 180.0})
    res = n._persist_pickup_params([_Param("pickup_stop_secs", 7.0)])
    assert res.successful
    snap = _read(tmp_path / "pickup.json")
    assert snap == {"pickup_stop_enable": True, "pickup_stop_secs": 7.0,
                    "pickup_spin_deg": 180.0}


def test_never_vetoes_never_raises(tmp_path):
    n = ws.WebServerNode.__new__(ws.WebServerNode)
    n._pickup_settings_file = lambda: str(tmp_path / "nope" / "x.json")
    n.get_parameter = lambda name: (_ for _ in ()).throw(RuntimeError("dead"))
    n.get_logger = lambda: _LOG
    res = n._persist_pickup_params([_Param("pickup_stop_secs", 9.0)])
    assert res.successful
    assert not (tmp_path / "nope" / "x.json").exists()


def test_keys_match_the_cluster():
    assert PICKUP_PERSIST_KEYS == ("pickup_stop_enable", "pickup_stop_secs",
                                   "pickup_spin_deg")


# ---- telemetry hub wiring (the decision + the two actions + the gates) ---------
class _FakePub:
    def publish(self, msg):
        pass


class _FakeLog:
    def warning(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class _PickupNode:
    """The sliver of web_server's surface the hub's pickup paths touch, plus
    recording stubs for the two actions telemetry drives."""

    _face_pub = _FakePub()

    def __init__(self, params=None):
        self.params = dict(params or {})
        self.stops = 0
        self.releases = []
        self.giveups = 0
        self.release_ok = True
        self._susp_l = self._susp_r = False
        self._susp_override = -1

    def get_logger(self):
        return _FakeLog()

    def get_parameter(self, name):
        if name not in self.params:
            raise KeyError(name)
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, type_, name, qos):
        return _FakePub()

    def create_client(self, srv, name):
        return SimpleNamespace(service_is_ready=lambda: False,
                               call_async=lambda req: None)

    def create_subscription(self, type_, name, cb, qos):
        return object()

    def create_timer(self, period, cb):
        pass

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

    def on_pickup_stop(self):
        self.stops += 1

    def on_pickup_release(self, deg):
        self.releases.append(deg)
        return self.release_ok

    def pickup_release_giveup(self):
        self.giveups += 1


def _pnode(**params):
    defaults = {"pickup_stop_enable": True, "pickup_stop_secs": 5.0,
                "pickup_spin_deg": 180.0, "lds_idle_secs": 60.0}
    defaults.update(params)
    return _PickupNode(defaults)


def _navlog(h):
    return " | ".join(e["msg"] for e in h._navlog)


def test_hub_latches_stop_after_stop_secs():
    n = _pnode()
    h = TelemetryHub(n)
    for i in range(10):                       # 0.5 s samples: no latch before 5 s
        h._pickup_eval(100.0 + 0.5 * i, True, True, -1)
        assert n.stops == 0
    h._pickup_eval(105.0, True, True, -1)
    assert n.stops == 1
    assert h._pickup_latched is True
    assert "picked up" in _navlog(h)
    h._pickup_eval(106.0, True, True, -1)     # idempotent
    assert n.stops == 1


def test_hub_release_spin_starts_when_base_free():
    n = _pnode()
    h = TelemetryHub(n)
    for i in range(11):
        h._pickup_eval(100.0 + 0.5 * i, True, True, -1)
    assert n.stops == 1
    for i in range(3):                        # grounded confirm (1.0 s)
        h._pickup_eval(106.0 + 0.5 * i, False, False, -1)
    assert h._pickup_pending is not None
    h._pickup_ctrl_tick()
    assert h._pickup_pending is None
    assert n.releases == [180.0]
    assert "re-orientation turn started" in _navlog(h)


def test_hub_release_retries_then_gives_up():
    n = _pnode()
    n.release_ok = False
    h = TelemetryHub(n)
    for i in range(11):
        h._pickup_eval(100.0 + 0.5 * i, True, True, -1)
    for i in range(3):
        h._pickup_eval(106.0 + 0.5 * i, False, False, -1)
    assert h._pickup_pending is not None
    h._pickup_pending = time.monotonic()      # align with the tick's real clock
    h._pickup_ctrl_tick()                     # refused -> retried, not started
    assert h._pickup_pending is not None
    assert "re-orientation turn started" not in _navlog(h)
    h._pickup_pending = time.monotonic() - (PICKUP_SPIN_GIVEUP + 1.0)
    h._pickup_ctrl_tick()
    assert h._pickup_pending is None
    assert n.giveups == 1
    assert h._pickup_latched is False
    assert "gave up" in _navlog(h)


def test_hub_disabled_clears_the_latch():
    n = _pnode(pickup_stop_enable=False)
    h = TelemetryHub(n)
    h._pickup.latched = True
    h._pickup_latched = True
    h._pickup_eval(100.0, True, True, -1)
    assert h._pickup_latched is False
    assert n.stops == 0
    h._pickup_eval(101.0, True, True, -1)     # stays quiet
    assert n.stops == 0


def test_hub_goal_refused_while_latched():
    n = _pnode()
    h = TelemetryHub(n)
    h._pickup_latched = True
    out = h.publish_json({"topic": "/goal_pose", "value": {"x": 1.0, "y": 1.0}})
    assert "picked up" in out.get("error", "")
    h._pickup_latched = False
    # un-latched: the goal path proceeds past the gate (into wake_lidar's hold,
    # which needs no ROS graph on the fake)


def test_hub_cancel_pickup_spin():
    n = _pnode()
    h = TelemetryHub(n)
    h._pickup_pending = 100.0
    h.cancel_pickup_spin("user drive took over")
    assert h._pickup_pending is None
    assert "release turn cancelled" in _navlog(h)
    h.cancel_pickup_spin("again")             # no-op when nothing pending
    assert "again" not in _navlog(h)
