"""Offline tests (ROS-free) for the GET /plan polyline plumbing — telemetry's
_plan cache (downsample cap, empty-path skip, atomic tuple) and the wire shape
web_server's _serve_plan rides on. The planner itself is stock Nav2; what is
ours is exactly this glue.

    pixi run test
"""
import math
import struct

import pytest

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from web_control.telemetry import PLAN_MAX_POINTS, TelemetryHub


def _path(n, spacing=0.1):
    p = Path()
    for i in range(n):
        ps = PoseStamped()
        ps.pose.position.x = i * spacing
        ps.pose.position.y = -i * spacing
        p.poses.append(ps)
    return p


def test_plan_payload_none_before_first_plan():
    assert TelemetryHub(_FakeNode()).get_plan_payload() == (None, None)


class _FakeLog:
    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass


class _FakePub:
    def publish(self, msg):
        pass


class _FakeClient:
    def service_is_ready(self):
        return False


class _FakePub:
    def publish(self, msg):
        pass


class _FakeNode:
    _face_pub = _FakePub()

    def get_logger(self):
        return _FakeLog()

    def get_parameter(self, name):
        raise KeyError(name)

    def create_publisher(self, type_, name, qos):
        return _FakePub()

    def create_client(self, srv, name):
        return _FakeClient()

    def create_subscription(self, type_, name, cb, qos):
        return object()

    def create_timer(self, period, cb):
        pass


def test_on_plan_caches_downsampled_flat_points():
    h = TelemetryHub(_FakeNode())
    h._on_plan(_path(500))
    meta, pts = h.get_plan_payload()
    assert meta["n"] == PLAN_MAX_POINTS
    assert len(pts) == PLAN_MAX_POINTS * 2
    # endpoints preserved (downsample keeps first + last)
    assert pts[0] == pytest.approx(0.0) and pts[1] == pytest.approx(0.0)
    assert pts[-2] == pytest.approx(499 * 0.1, abs=0.2)
    assert pts[-1] == pytest.approx(-499 * 0.1, abs=0.2)
    # arrival stamp set for plan_age
    assert h._plan_arrival is not None


def test_on_plan_small_path_kept_verbatim():
    h = TelemetryHub(_FakeNode())
    h._on_plan(_path(5))
    meta, pts = h.get_plan_payload()
    assert meta["n"] == 5
    assert pts == pytest.approx([0.0, 0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3,
                                 0.4, -0.4])


def test_on_plan_skips_empty_and_single_pose_paths():
    """A planner hiccup (empty path) must not blank the last good polyline."""
    h = TelemetryHub(_FakeNode())
    h._on_plan(_path(10))
    good, _ = h.get_plan_payload()
    h._on_plan(Path())                       # empty
    h._on_plan(_path(1))                     # single pose — not drawable
    meta, _ = h.get_plan_payload()
    assert meta is good                      # last good copy survives


def test_plan_wire_shape_is_header_plus_float32():
    """_serve_plan writes JSON(header)+\\n+float32 — verify the exact byte
    shape the page's DataView parses (n*2 little-endian float32)."""
    h = TelemetryHub(_FakeNode())
    h._on_plan(_path(4))
    meta, pts = h.get_plan_payload()
    body = (str(meta).encode() + b"\n"
            + struct.pack("<%df" % len(pts), *pts))
    nl = body.index(b"\n")
    f = struct.unpack("<%df" % ((len(body) - nl - 1) // 4), body[nl + 1:])
    assert len(f) == 8
    assert f[2] == pytest.approx(0.1)
    assert f[3] == pytest.approx(-0.1)
