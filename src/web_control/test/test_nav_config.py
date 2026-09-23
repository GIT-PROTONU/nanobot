"""Offline tests (ROS-free) for web_control's Nav2 navigation-pace config — the
GET/POST /nav/config endpoint behind the Drive card's "Navigation pace" sliders
(speed/accel caps) + "Keep-away zone"/"Robot size" sliders (costmap geometry).

The requested speed/accel caps live on web_control as nav_* params and are
pushed to nav2's velocity_smoother (composed in nav2_container) as 3-element
[x, 0, theta] DOUBLE_ARRAY SetParameters requests. The smoother's
dynamicParametersCallback accepts them live (no nano-nav restart). The zone
geometry rides the same endpoint: nav_inflation_m + nav_robot_diam_m are pushed
to BOTH costmap components as SCALAR doubles — robot_radius (= ⌀/2) and
"<plugin>.inflation_radius" — which Humble's Costmap2DROS / InflationLayer
dynamicParametersCallbacks accept live. Pinned here:

    * _clamp_nav_cfg bounds: each of the four scalars lands inside its range
      (0.35 m/s saturation cliff, 1.0 rad/s SLAM smear budget, accel ranges)
    * _clamp_nav_geom bounds: keep-away 0..0.6 m, robot ⌀ 0.1..1.0 m
    * _nav_param_msg: raw rcl_interfaces/msg/Parameter wire message
      (NOT rclpy's wrapper — telemetry.py's set_param_json convention),
      PARAMETER_DOUBLE_ARRAY type, 3 values, name carried
    * _scalar_param_msg: same wire message, PARAMETER_DOUBLE type
    * _nav_push composition on a fake node: four params in one request,
      min_velocity = -max_velocity, max_decel = -max_accel (decel NEGATIVE —
      the smoother's configure() throws on positive decel), y = 0 everywhere,
      and the not-reachable / bad-service paths stay graceful
    * _costmap_push composition: robot_radius = ⌀/2 (the web UI works in
      diameter, the costmap wants the radius) + the namespaced inflation param,
      on BOTH costmap nodes, graceful when unreachable

    pixi run test
"""
import threading
import time
import types

import web_control.web_server as ws
from web_control.web_server import (_clamp_nav_cfg, _clamp_nav_geom,
                                    NAV_LIN_RANGE, NAV_ANG_RANGE,
                                    NAV_LIN_ACC_RANGE, NAV_ANG_ACC_RANGE,
                                    NAV_INFL_RANGE, NAV_DIAM_RANGE,
                                    NAV_SMOOTHER_NODE, NAV_COSTMAP_NODES,
                                    NAV_INFLATION_LAYER, NAV_PARAM_KEYS)
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


# ---- zone-geometry clamps ---------------------------------------------------------
def test_clamp_nav_geom_pass_through():
    assert _clamp_nav_geom(0.25, 0.32) == (0.25, 0.32)


def test_clamp_nav_geom_bounds():
    # keep-away can be 0 (no inflation zone) but not negative; both clamp
    # independently, high and low
    assert _clamp_nav_geom(99.0, 99.0) == (NAV_INFL_RANGE[1], NAV_DIAM_RANGE[1])
    assert _clamp_nav_geom(-1.0, 0.02) == (NAV_INFL_RANGE[0], NAV_DIAM_RANGE[0])
    assert NAV_INFL_RANGE == (0.0, 0.6)
    assert NAV_DIAM_RANGE == (0.1, 1.0)


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
                              "nav_lin_accel", "nav_ang_accel",
                              "nav_inflation_m", "nav_robot_diam_m")


# ---- raw scalar param message (costmap geometry) -----------------------------------
def test_scalar_param_msg_shape():
    # the method uses no self; call it unbound with None
    p = ws.WebServerNode._scalar_param_msg(None, "robot_radius", 0.16)
    assert type(p).__name__ == "Parameter"
    assert p.name == "robot_radius"
    assert p.value.type == ParameterType.PARAMETER_DOUBLE
    assert p.value.double_value == 0.16


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


# ---- costmap geometry push (keep-away + robot size) --------------------------------
def test_costmap_targets_and_layer_name():
    # must match config/nav2/nav2_params.yaml: both costmaps expose an
    # inflation_layer plugin, and the nodes are the container's children
    assert NAV_COSTMAP_NODES == ("local_costmap/local_costmap",
                                 "global_costmap/global_costmap")
    assert NAV_INFLATION_LAYER == "inflation_layer"


def _fake_cm_node(ready=True):
    return types.SimpleNamespace(
        _nav_cfg_lock=threading.Lock(),
        _cm_clients={n: _FakeClient(ready) for n in NAV_COSTMAP_NODES},
        _scalar_param_msg=ws.WebServerNode._scalar_param_msg.__get__(
            types.SimpleNamespace()))


def _cm_by_name(req):
    return {p.name: p.value.double_value for p in req.parameters}


def test_costmap_push_composes_radius_and_inflation():
    node = _fake_cm_node()
    cfg = {"nav_inflation_m": 0.3, "nav_robot_diam_m": 0.4}
    for n in NAV_COSTMAP_NODES:
        assert ws.WebServerNode._costmap_push(node, cfg, n) is None
    for n in NAV_COSTMAP_NODES:
        (req,) = node._cm_clients[n].requests
        params = _cm_by_name(req)
        # the web UI works in DIAMETRE; the costmap wants the RADIUS (⌀/2)
        assert params["robot_radius"] == 0.2
        # the inflation param is namespaced with the plugin name
        assert params[f"{NAV_INFLATION_LAYER}.inflation_radius"] == 0.3
    # and each costmap gets its OWN request (two nodes, two calls)
    assert len(node._cm_clients[NAV_COSTMAP_NODES[0]].requests) == 1
    assert len(node._cm_clients[NAV_COSTMAP_NODES[1]].requests) == 1


def test_costmap_push_not_reachable_is_graceful():
    node = _fake_cm_node(ready=False)
    for n in NAV_COSTMAP_NODES:
        err = ws.WebServerNode._costmap_push(
            node, {"nav_inflation_m": 0.25, "nav_robot_diam_m": 0.32}, n)
        assert err and n in err
        assert node._cm_clients[n].requests == []


def _fake_cm_with_push(ready=True):
    node = _fake_cm_node(ready)
    node._cm_last_push = {n: 0.0 for n in NAV_COSTMAP_NODES}
    node.nav_config = lambda: {"nav_inflation_m": 0.25, "nav_robot_diam_m": 0.32}
    node._costmap_push = ws.WebServerNode._costmap_push.__get__(node)
    node._cm_push_saved = ws.WebServerNode._cm_push_saved.__get__(node)
    node.get_logger = lambda: types.SimpleNamespace(
        warning=lambda *a, **k: None, info=lambda *a, **k: None)
    return node


def test_cm_transition_repushes_on_activate_only():
    node = _fake_cm_with_push()
    n1, n2 = NAV_COSTMAP_NODES
    # cleanup/deconfigure transitions are ignored
    ws.WebServerNode._on_cm_transition(node, _event("finalized"), n1)
    assert node._cm_clients[n1].requests == []
    # the active transition re-applies the saved geometry on THAT costmap
    ws.WebServerNode._on_cm_transition(node, _event("active"), n1)
    (req,) = node._cm_clients[n1].requests
    assert _cm_by_name(req)["robot_radius"] == 0.16
    # the OTHER costmap's activation is independent (no shared rate limit —
    # smoother/local/global activate at staggered moments of one nav startup)
    assert node._cm_clients[n2].requests == []
    ws.WebServerNode._on_cm_transition(node, _event("active"), n2)
    assert len(node._cm_clients[n2].requests) == 1
    # per-target rate limiting: an immediate second activation does not re-push
    ws.WebServerNode._on_cm_transition(node, _event("active"), n1)
    ws.WebServerNode._on_cm_transition(node, _event("active"), n2)
    assert len(node._cm_clients[n1].requests) == 1
    assert len(node._cm_clients[n2].requests) == 1
