"""Offline tests (ROS-free) for multi-waypoint navigation glue — the POST
/nav/waypoints handler's validation (clamps, cap, bad points, unreachable
server) and telemetry's waypoint mirror (note_waypoints + the action-feedback
progress refinement + terminal-state clearing).

    pixi run test
"""
import math

import pytest

from web_control.telemetry import GOAL_MAX_ABS_M, NAV_WAYPOINT_MAX, TelemetryHub

from test_nav_telemetry import _FakeLog, _FakePub, _FakeClient, _FakeNode, \
    _status_arr


class _NoServerClient:
    def wait_for_server(self, timeout_sec):
        return False


class _OkClient:
    def __init__(self):
        self.goals = []

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)


def _waypoint_node(client):
    """A WebServerNode stand-in exposing just what nav_waypoints touches. The
    method itself is borrowed unbound from the real WebServerNode class (which
    can't be constructed offline — it needs rclpy)."""
    from test_nav_telemetry import _FakeNode as _N
    from web_control.web_server import WebServerNode

    class _N2(_FakeNode):
        _nav_poses_client = client

        def __init__(self, c):
            self._nav_poses_client = c
            self.telemetry = TelemetryHub(_FakeNode())
            self._log = _FakeLog()

        def get_logger(self):
            return self._log

        def get_clock(self):
            class _C:
                def now(self):
                    from rclpy.time import Time
                    return Time()

            return _C()

        nav_waypoints = WebServerNode.nav_waypoints
        restart_waypoint_loop = WebServerNode.restart_waypoint_loop
        _build_poses_goal = WebServerNode._build_poses_goal

    return _N2(client)


# ---- POST /nav/waypoints validation ---------------------------------------------
def test_waypoints_rejects_empty_and_bad_shape():
    n = _waypoint_node(_OkClient())
    assert "error" in n.nav_waypoints({})
    assert "error" in n.nav_waypoints({"points": []})
    assert "error" in n.nav_waypoints({"points": "nope"})
    assert "error" in n.nav_waypoints({"points": [{"x": 1.0}]})      # missing y
    assert "error" in n.nav_waypoints({"points": [{"x": "a", "y": 2}]})


def test_waypoints_caps_the_list():
    n = _waypoint_node(_OkClient())
    pts = [{"x": float(i), "y": 0.0} for i in range(NAV_WAYPOINT_MAX + 1)]
    out = n.nav_waypoints({"points": pts})
    assert str(NAV_WAYPOINT_MAX) in out["error"]


def test_waypoints_clamps_to_goal_bounds():
    n = _waypoint_node(_OkClient())
    out = n.nav_waypoints({"points": [{"x": 999.0, "y": -999.0}, {"x": 1.0, "y": 2.0}]})
    assert out == {"ok": True, "n": 2, "loop": False}
    goal = n._nav_poses_client.goals[-1]
    assert goal.poses[0].pose.position.x == pytest.approx(GOAL_MAX_ABS_M)
    assert goal.poses[0].pose.position.y == pytest.approx(-GOAL_MAX_ABS_M)
    assert goal.poses[1].pose.position.x == pytest.approx(1.0)
    assert goal.poses[0].header.frame_id == "map"
    assert len(goal.poses) == 2


def test_waypoints_refused_when_server_down():
    n = _waypoint_node(_NoServerClient())
    out = n.nav_waypoints({"points": [{"x": 1.0, "y": 2.0}]})
    assert "error" in out and "bt_navigator" in out["error"]


# ---- telemetry mirror ------------------------------------------------------------
def test_note_waypoints_mirrors_first_stop_and_total():
    h = TelemetryHub(_FakeNode())
    h.note_waypoints([(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)], source="web waypoints")
    assert h._goal == [1.0, 2.0]                    # ring on the FIRST stop
    assert h._goal_status == "planning"
    assert h._wp_total == 3 and h._wp_index == 0


def test_wp_feedback_advances_index():
    """number_of_poses_remaining 3→1 refines wp_index 0→2 of 4 (0-based)."""

    class _Fb:
        number_of_poses_remaining = 3

    h = TelemetryHub(_FakeNode())
    h.note_waypoints([(0, 0), (1, 1), (2, 2), (3, 3)])
    h._on_wp_feedback(_Fb())
    assert h._wp_index == 0                         # 4 total, 3 remaining → stop 1
    _Fb.number_of_poses_remaining = 1
    h._on_wp_feedback(_Fb())
    assert h._wp_index == 2                         # 1 remaining → driving toward stop 3


def test_wp_feedback_ignored_without_an_active_tour():
    h = TelemetryHub(_FakeNode())

    class _Fb:
        number_of_poses_remaining = 0

    h._on_wp_feedback(_Fb())                        # no note_waypoints first
    assert h._wp_index is None and h._wp_total is None


def test_wp_feedback_never_raises_on_garbage():
    h = TelemetryHub(_FakeNode())
    h.note_waypoints([(0, 0), (1, 1)])

    class _Fb:
        number_of_poses_remaining = "garbage"

    h._on_wp_feedback(_Fb())                        # must not raise
    assert h._wp_index == 0


def test_terminal_status_clears_waypoint_progress():
    h = TelemetryHub(_FakeNode())
    h.note_waypoints([(0, 0), (1, 1)])
    h._on_goal_status(_status_arr(4))               # SUCCEEDED
    assert h._wp_index is None and h._wp_total is None
    assert h._goal is None


# ---- waypoint loop mode ----------------------------------------------------------
def _loop_hub():
    """A hub whose node records restart_waypoint_loop calls (telemetry triggers
    the restart through getattr on its node, mirroring arrival_heading_rotate)."""
    node = _FakeNode()
    node.vitals = lambda: {}        # _build() reads the vitals blob via the node
    node._susp_l = node._susp_r = False     # _build()'s suspension keys
    node._susp_override = -1
    hub = TelemetryHub(node)
    node.restarts = []

    def _rec(poses):
        node.restarts.append(list(poses))

    node.restart_waypoint_loop = _rec
    return hub


def test_loop_flag_stored_and_in_frame():
    h = _loop_hub()
    h.note_waypoints([(0, 0), (1, 1)], loop=True)
    assert h._wp_loop is True
    assert h._wp_list == [(0.0, 0.0), (1.0, 1.0)]
    h2 = _loop_hub()
    h2.note_waypoints([(0, 0), (1, 1)])             # no loop → flag off
    assert h2._wp_loop is False
    assert h2._wp_list == [(0.0, 0.0), (1.0, 1.0)]  # list kept either way


def test_loop_restarts_on_success():
    h = _loop_hub()
    h.note_waypoints([(0, 0), (1, 1), (2, 2)], loop=True)
    h._on_goal_status(_status_arr(4))               # lap succeeded → re-send
    assert h._node.restarts == [[(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]]
    assert h._wp_loop is False and h._wp_list is None   # handed off to the restart


def test_loop_not_rearmed_without_loop_flag():
    h = _loop_hub()
    h.note_waypoints([(0, 0), (1, 1)])              # plain tour
    h._on_goal_status(_status_arr(4))
    assert h._node.restarts == []


def test_loop_stops_on_cancel_and_fail():
    h = _loop_hub()
    h.note_waypoints([(0, 0), (1, 1)], loop=True)
    h._on_goal_status(_status_arr(6))               # ABORTED
    assert h._node.restarts == [] and h._wp_loop is False and h._wp_list is None
    assert any("loop stopped — tour failed" in e["msg"] for e in h._navlog)
    h2 = _loop_hub()
    h2.note_waypoints([(0, 0), (1, 1)], loop=True)
    h2._on_goal_status(_status_arr(5))              # CANCELED
    assert h2._node.restarts == [] and h2._wp_loop is False
    assert any("loop stopped — tour canceled" in e["msg"] for e in h2._navlog)


def test_clear_goal_stops_loop():
    h = _loop_hub()
    h.note_waypoints([(0, 0), (1, 1)], loop=True)
    h.clear_goal()
    assert h._wp_loop is False and h._wp_list is None
    assert any("loop stopped — cancelled" in e["msg"] for e in h._navlog)


def test_single_goal_replaces_tour_loop():
    """A single-goal publish (map click / Locations / skill) while a looped
    tour runs must kill the loop — otherwise the single goal's own SUCCEEDED
    would resurrect the tour."""
    h = _loop_hub()
    h.note_waypoints([(0, 0), (1, 1)], loop=True)
    h.note_goal(5.0, 5.0, source="skill go-to 'kitchen'")
    assert h._wp_loop is False and h._wp_list is None
    assert h._wp_index is None and h._wp_total is None
    h._on_goal_status(_status_arr(4))
    assert h._node.restarts == []


def test_loop_survives_waypoint_feedback_only_when_armed():
    h = _loop_hub()
    h.note_waypoints([(0, 0), (1, 1)], loop=True)
    h.note_goal(2.0, 2.0)                           # replace with a single goal
    assert h._wp_loop is False


def test_restart_waypoint_loop_resends_and_rearms():
    n = _waypoint_node(_OkClient())
    poses = [(1.0, 2.0), (3.0, 4.0)]
    n.restart_waypoint_loop(poses)
    assert len(n._nav_poses_client.goals) == 1
    g = n._nav_poses_client.goals[0]
    assert [(p.pose.position.x, p.pose.position.y) for p in g.poses] == poses
    assert all(p.pose.orientation.w == 1.0 for p in g.poses)
    assert n.telemetry._wp_loop is True and n.telemetry._wp_total == 2
    assert n.telemetry._goal_status == "planning"


def test_restart_waypoint_loop_refused_when_picked_up():
    n = _waypoint_node(_OkClient())
    n.telemetry._pickup_latched = True
    n.restart_waypoint_loop([(1.0, 2.0)])
    assert n._nav_poses_client.goals == []
    assert any("loop stopped" in e["msg"] for e in n.telemetry._navlog)
