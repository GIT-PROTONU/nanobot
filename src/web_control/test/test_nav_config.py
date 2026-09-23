"""Offline tests (ROS-free) for web_control's Nav2 navigation-pace config — the
GET/POST /nav/config endpoint behind the Drive card's "Navigation pace" sliders.

The requested speed/accel caps live on web_control as nav_* params and are
pushed to nav2's velocity_smoother (composed in nav2_container) as 3-element
[x, 0, theta] DOUBLE_ARRAY SetParameters requests. The smoother's
dynamicParametersCallback accepts them live (no nano-nav restart). Pinned here:

    * _clamp_nav_cfg bounds: each of the four scalars lands inside its range
      (0.35 m/s saturation cliff, 1.0 rad/s SLAM smear budget, accel ranges)
    * _nav_param_msg: raw rcl_interfaces/msg/Parameter wire message
      (NOT rclpy's wrapper — telemetry.py's set_param_json convention),
      PARAMETER_DOUBLE_ARRAY type, 3 values, name carried
    * _nav_push composition on a fake node: four params in one request,
      min_velocity = -max_velocity, max_decel = -max_accel (decel NEGATIVE —
      the smoother's configure() throws on positive decel), y = 0 everywhere,
      and the not-reachable / bad-service paths stay graceful

    pixi run test
"""
import threading
import time
import types

import web_control.web_server as ws
from web_control.web_server import (_clamp_nav_cfg, NAV_LIN_RANGE,
                                    NAV_ANG_RANGE, NAV_LIN_ACC_RANGE,
                                    NAV_ANG_ACC_RANGE, NAV_SMOOTHER_NODE,
                                    NAV_PARAM_KEYS)
from rcl_interfaces.msg import ParameterType


# ---- clamps -----------------------------------------------------------------------
def test_clamp_nav_cfg_pass_through():
    lin, ang, la, aa = _clamp_nav_cfg(0.18, 0.8, 0.5, 1.6)
    assert (lin, ang, la, aa) == (0.18, 0.8, 0.5, 1.6)


def test_clamp_nav_cfg_bounds():
    # each axis clamps independently, high and low
    assert _clamp_nav_cfg(0.99, 9.0, 99.0, 99.0) == (
        NAV_LIN_RANGE[1], NAV_ANG_RANGE[1], NAV_LIN_ACC_RANGE[1],
        NAV_ANG_ACC_RANGE[1])
    assert _clamp_nav_cfg(0.001, 0.001, 0.001, 0.001) == (
        NAV_LIN_RANGE[0], NAV_ANG_RANGE[0], NAV_LIN_ACC_RANGE[0],
        NAV_ANG_ACC_RANGE[0])
    # the cliff: 0.35 m/s is the linear ceiling (loaded full-duty ~0.37)
    assert NAV_LIN_RANGE[1] == 0.35
    # the smear budget: 1.0 rad/s = 11.5 deg/scan, same ceiling as drive_max_ang
    assert NAV_ANG_RANGE[1] == 1.00


def test_clamp_nav_cfg_rejects_nothing_numeric():
    # floats-as-strings are the caller's problem (update_nav_config catches
    # ValueError); pure floats must never raise
    _clamp_nav_cfg(-5.0, -5.0, -5.0, -5.0)   # just floors


# ---- raw param message ------------------------------------------------------------
def test_nav_param_msg_shape():
    # the method uses no self; call it unbound with None
    p = ws.WebServerNode._nav_param_msg(None, "max_velocity", [0.2, 0.0, 0.5])
    # RAW rcl_interfaces/msg/Parameter (the SetParameters wire format), not
    # rclpy's Parameter wrapper
    assert type(p).__name__ == "Parameter"
    assert p.name == "max_velocity"
    assert p.value.type == ParameterType.PARAMETER_DOUBLE_ARRAY
    assert list(p.value.double_array_value) == [0.2, 0.0, 0.5]


# ---- push composition (fake node) ---------------------------------------------------
class _FakeClient:
    def __init__(self, ready=True):
        self.requests = []
        self.ready = ready

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        self.requests.append(req)


def _fake_node(ready=True):
    return types.SimpleNamespace(
        _nav_cfg_lock=threading.Lock(), _nav_client=_FakeClient(ready),
        _nav_param_msg=ws.WebServerNode._nav_param_msg.__get__(
            types.SimpleNamespace()))


def _by_name(req):
    return {p.name: list(p.value.double_array_value) for p in req.parameters}


def test_nav_push_composes_all_four_params():
    node = _fake_node()
    cfg = {"nav_lin_speed": 0.2, "nav_ang_speed": 0.5,
           "nav_lin_accel": 0.4, "nav_ang_accel": 1.2}
    err = ws.WebServerNode._nav_push(node, cfg)
    assert err is None
    (req,) = node._nav_client.requests
    params = _by_name(req)
    # y is 0 everywhere (differential drive); theta carries the angular value
    assert params["max_velocity"] == [0.2, 0.0, 0.5]
    assert params["min_velocity"] == [-0.2, 0.0, -0.5]
    assert params["max_accel"] == [0.4, 0.0, 1.2]
    # decel NEGATIVE (the smoother's configure() throws on positive decel),
    # magnitude = the linear accel setting => symmetric braking
    assert params["max_decel"] == [-0.4, 0.0, -1.2]


def test_nav_push_not_reachable_is_graceful():
    node = _fake_node(ready=False)
    err = ws.WebServerNode._nav_push(
        node, {"nav_lin_speed": 0.2, "nav_ang_speed": 0.5,
               "nav_lin_accel": 0.4, "nav_ang_accel": 1.2})
    assert err and NAV_SMOOTHER_NODE in err
    assert node._nav_client.requests == []


def test_nav_param_keys_are_the_four_sliders():
    assert NAV_PARAM_KEYS == ("nav_lin_speed", "nav_ang_speed",
                              "nav_lin_accel", "nav_ang_accel")


# ---- activation re-push (the 2026-09-23 board race) --------------------------------
def _fake_node_with_push(ready=True):
    node = _fake_node(ready)
    node._nav_last_push = 0.0
    node.nav_config = lambda: {"nav_lin_speed": 0.18, "nav_ang_speed": 0.8,
                               "nav_lin_accel": 0.5, "nav_ang_accel": 1.6}
    node._nav_push = ws.WebServerNode._nav_push.__get__(node)
    node.get_logger = lambda: types.SimpleNamespace(
        warning=lambda *a, **k: None, info=lambda *a, **k: None)
    return node


def _event(label):
    return types.SimpleNamespace(goal_state=types.SimpleNamespace(label=label))


def test_nav_transition_repushes_on_activate_only():
    node = _fake_node_with_push()
    # cleanup/deconfigure transitions are ignored
    ws.WebServerNode._on_nav_transition(node, _event("finalized"))
    assert node._nav_client.requests == []
    # the active transition re-applies the saved pace
    ws.WebServerNode._on_nav_transition(node, _event("active"))
    (req,) = node._nav_client.requests
    assert list(req.parameters[0].value.double_array_value) == [0.18, 0.0, 0.8]
    # rate-limited: an immediate second activation does not double-push
    ws.WebServerNode._on_nav_transition(node, _event("active"))
    assert len(node._nav_client.requests) == 1


def test_nav_transition_rate_limit_expires(monkeypatch):
    node = _fake_node_with_push()
    fake = types.SimpleNamespace(t=time.monotonic())
    monkeypatch.setattr(ws.time, "monotonic", lambda: fake.t)
    ws.WebServerNode._on_nav_transition(node, _event("active"))
    assert len(node._nav_client.requests) == 1
    # 6 s later the rate limit has expired -> a new activation pushes again
    fake.t += 6.0
    ws.WebServerNode._on_nav_transition(node, _event("active"))
    assert len(node._nav_client.requests) == 2
