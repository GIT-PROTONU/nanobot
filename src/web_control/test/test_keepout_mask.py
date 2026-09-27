"""Offline tests (ROS-free) for the drawable keep-out zones — the keepout
handler set (save/edit/delete/clear/persist-shape, cap, legacy conversion) and
telemetry's mask rasterizer (ROTATED rects → cells on the live /map geometry,
alignment with the grid origin/resolution, re-emit on a rebuilt map, empty-mask
clear).

    pixi run test
"""
import math

import pytest

from nav_msgs.msg import OccupancyGrid

from web_control.telemetry import KEEPOUT_MAX_ZONES, TelemetryHub

from test_nav_telemetry import _FakeLog, _FakePub, _FakeClient, _FakeNode, _grid


def _painted(m):
    """(col, row) pairs of the nonzero cells of a mask message."""
    w = m.info.width
    return [(i % w, i // w) for i, v in enumerate(m.data) if v != 0]


def test_keepout_mask_none_without_a_map():
    """No /map geometry yet (boot) → nothing published, no crash."""
    h = TelemetryHub(_FakeNode())
    h.set_keepout_zones([{"x": 0, "y": 0, "w": 1, "h": 1}])
    assert h._keepout_zones


def test_keepout_mask_rasterizes_rects_on_map_geometry():
    """A 2x2 m map @0.05 m = 40x40 cells, origin (-2,-3) (the _grid helper).
    One rect from (-1,-2) to (-0.5,-1.5) must paint exactly the cells whose
    centres fall inside it: columns/rows 20..29 inclusive (legacy corner
    shape)."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h._pubs["/keepout_mask"][0].published.clear()
    h.set_keepout_zones([{"x1": -1.0, "y1": -2.0, "x2": -0.5, "y2": -1.5}])
    msgs = h._pubs["/keepout_mask"][0].published
    assert len(msgs) == 1
    m = msgs[0]
    assert m.info.width == 40 and m.info.height == 40
    assert m.info.resolution == pytest.approx(0.05)
    assert m.header.frame_id == "map"
    data = list(m.data)
    assert sorted(_painted(m)) == sorted(
        [(c, r) for r in range(20, 30) for c in range(20, 30)])
    assert all(data[i] == 100 for i, v in enumerate(data) if v != 0)
    # the filter-info topic rides along, pointing at the ABSOLUTE mask topic
    # (the KeepoutFilter subscribes from the costmap node — a relative name
    # would resolve to /global_costmap/keepout_mask and never match; found
    # live 2026-09-27). Two infos: the initial _on_map latch with no zones +
    # the set_keepout_zones re-emit.
    infos = h._pubs["/keepout_filter_info"][0].published
    assert len(infos) == 2
    assert infos[-1].filter_mask_topic == "/keepout_mask"
    assert infos[-1].type == 0


def test_keepout_mask_center_shape_matches_legacy():
    """The normalized {x,y,w,h,rot} shape rasterizes identically to the legacy
    corner shape at rot 0."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h.set_keepout_zones([{"x1": -1.0, "y1": -2.0, "x2": -0.5, "y2": -1.5}])
    legacy = list(h._pubs["/keepout_mask"][0].published[-1].data)
    h.set_keepout_zones([{"x": -0.75, "y": -1.75, "w": 0.5, "h": 0.5, "rot": 0.0}])
    center = list(h._pubs["/keepout_mask"][0].published[-1].data)
    assert legacy == center


def test_keepout_mask_rotated_rect_rasterization():
    """A 1.02x0.05 m bar rotated 90° becomes a vertical strip: exactly column
    20, rows 10..30 (the bar's x-extent shrinks to one cell, y grows to 21;
    side padded off the cell-centre grid so no cell sits exactly on the
    boundary)."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h._pubs["/keepout_mask"][0].published.clear()
    # centre at cell (20,20)'s centre: (-2+20.5*0.05, -3+20.5*0.05)
    h.set_keepout_zones([{"x": -0.975, "y": -1.975, "w": 1.02, "h": 0.05,
                          "rot": math.pi / 2}])
    m = h._pubs["/keepout_mask"][0].published[-1]
    assert sorted(_painted(m)) == [(20, r) for r in range(10, 31)]


def test_keepout_mask_rotated_excludes_aabb_corners():
    """A 45°-rotated 0.2x0.2 m square on one cell paints a diamond of 13
    cells — NOT the full 5x5 AABB: cells outside the rotated rect (its AABB
    corners) must stay free."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h._pubs["/keepout_mask"][0].published.clear()
    h.set_keepout_zones([{"x": -0.975, "y": -1.975, "w": 0.2, "h": 0.2,
                          "rot": math.pi / 4}])
    got = set(_painted(h._pubs["/keepout_mask"][0].published[-1]))
    # every cell with |i|,|j|<=2 and |i+j|<=2 and |j-i|<=2 around (20,20)
    expect = set()
    for i in range(-2, 3):
        for j in range(-2, 3):
            if abs(i + j) <= 2 and abs(j - i) <= 2:
                expect.add((20 + i, 20 + j))
    assert got == expect
    assert (22, 22) not in got and (22, 18) not in got   # AABB corners excluded


def test_keepout_mask_quarter_turn_invariance():
    """A rect rotated 90° paints the same cells as the w/h-swapped axis-aligned
    one (rotation maths is symmetric under quarter turns on this grid; side
    padded off the cell-centre grid so no cell sits exactly on the boundary)."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h.set_keepout_zones([{"x": -0.975, "y": -1.975, "w": 1.02, "h": 0.2,
                          "rot": math.pi / 2}])
    a = sorted(_painted(h._pubs["/keepout_mask"][0].published[-1]))
    h.set_keepout_zones([{"x": -0.975, "y": -1.975, "w": 0.2, "h": 1.02,
                          "rot": 0.0}])
    b = sorted(_painted(h._pubs["/keepout_mask"][0].published[-1]))
    assert a == b


def test_keepout_mask_covers_full_grid_and_outside():
    """A rect larger than the map clamps to the grid; one entirely outside
    paints nothing (but the publish still happens). A huge rotated rect also
    covers everything."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h._pubs["/keepout_mask"][0].published.clear()
    h.set_keepout_zones([{"x1": -99, "y1": -99, "x2": 99, "y2": 99},
                         {"x1": 50, "y1": 50, "x2": 60, "y2": 60}])
    m = h._pubs["/keepout_mask"][0].published[-1]
    data = list(m.data)
    assert sum(1 for v in data if v != 0) == 40 * 40
    assert all(v == 100 for v in data if v != 0)
    h.set_keepout_zones([{"x": 0, "y": 0, "w": 99, "h": 99, "rot": 0.3}])
    m = h._pubs["/keepout_mask"][0].published[-1]
    assert sum(1 for v in m.data if v != 0) == 40 * 40


def test_keepout_reemit_on_map_change():
    """A rebuilt /map (new dims/origin) re-rasterizes + re-latches the mask so
    the KeepoutFilter's cells stay aligned with the new grid."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    n_after_first = len(h._pubs["/keepout_mask"][0].published)
    h.set_keepout_zones([{"x": 0.25, "y": 0.25, "w": 0.5, "h": 0.5}])
    n_zones = len(h._pubs["/keepout_mask"][0].published)
    h._on_map(_grid(80, 80, res=0.05))       # slam restarted, finer grid
    assert len(h._pubs["/keepout_mask"][0].published) == n_zones + 1
    m = h._pubs["/keepout_mask"][0].published[-1]
    assert m.info.width == 80 and m.info.height == 80
    assert n_after_first >= 1                # initial empty-mask latch happened


def test_keepout_clear_publishes_empty_mask():
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h.set_keepout_zones([{"x": 0.5, "y": 0.5, "w": 1, "h": 1}])
    h._pubs["/keepout_mask"][0].published.clear()
    h.set_keepout_zones([])
    m = h._pubs["/keepout_mask"][0].published[-1]
    assert all(v == 0 for v in m.data)       # the lethal latch clears


def test_keepout_zone_tuple_accepts_both_shapes():
    """_zone_tuple: normalized rows pass through; legacy corners convert
    (centre/size, rot 0); garbage rows are dropped, not crashed on."""
    assert TelemetryHub._zone_tuple({"x": 1, "y": 2, "w": 3, "h": 4, "rot": 0.5}) \
        == (1.0, 2.0, 3.0, 4.0, 0.5)
    assert TelemetryHub._zone_tuple({"x1": 0, "y1": 0, "x2": 2, "y2": -2}) \
        == (1.0, -1.0, 2.0, 2.0, 0.0)
    assert TelemetryHub._zone_tuple({"x": 1}) is None
    assert TelemetryHub._zone_tuple({"x1": "a", "y1": 0, "x2": 0, "y2": 0}) is None
    assert TelemetryHub._zone_tuple("nope") is None


# ---- the web_server handler set (borrowed unbound like the waypoints tests) ----
def _keepout_node(tmp_path):
    from web_control.web_server import WebServerNode

    class _N2(_FakeNode):
        def __init__(self):
            self.telemetry = TelemetryHub(_FakeNode())
            self.telemetry._on_map(_grid(40, 40, res=0.05))
            self._keepout_zones = []
            self._keepout_path = str(tmp_path / "keepout.json")
            self._log = _FakeLog()

        def get_logger(self):
            return self._log

        _load_keepout = WebServerNode._load_keepout
        _save_keepout = WebServerNode._save_keepout
        _norm_zone = WebServerNode._norm_zone
        get_keepout = WebServerNode.get_keepout
        keepout_save = WebServerNode.keepout_save
        keepout_delete = WebServerNode.keepout_delete
        keepout_clear = WebServerNode.keepout_clear

    return _N2()


def test_keepout_save_roundtrip_and_persist(tmp_path):
    n = _keepout_node(tmp_path)
    out = n.keepout_save({"x": 0.25, "y": 0.25, "w": 0.5, "h": 0.5, "rot": 0.0})
    assert out["ok"] and len(out["zones"]) == 1
    assert n.get_keepout()["zones"] == [
        {"x": 0.25, "y": 0.25, "w": 0.5, "h": 0.5, "rot": 0.0}]
    # telemetry mirror took it (normalized to tuples)
    assert n.telemetry._keepout_zones == [(0.25, 0.25, 0.5, 0.5, 0.0)]
    # reload from disk restores the zone
    n2 = _keepout_node(tmp_path)
    n2._load_keepout()
    assert n2._keepout_zones == [{"x": 0.25, "y": 0.25, "w": 0.5, "h": 0.5, "rot": 0.0}]


def test_keepout_save_converts_legacy_and_edits_in_place(tmp_path):
    n = _keepout_node(tmp_path)
    # legacy corner POST → normalized centre/size shape
    out = n.keepout_save({"x1": 0.0, "y1": 0.0, "x2": 0.5, "y2": 0.5})
    assert out["zones"] == [{"x": 0.25, "y": 0.25, "w": 0.5, "h": 0.5, "rot": 0.0}]
    n.keepout_save({"x": 2, "y": 2, "w": 1, "h": 1})
    # index edit replaces in place (the page's drag/resize/rotate path)
    out = n.keepout_save({"x": 9, "y": 9, "w": 2, "h": 3, "rot": 1.5, "index": 0})
    assert out["ok"]
    assert out["zones"][0] == {"x": 9.0, "y": 9.0, "w": 2.0, "h": 3.0, "rot": 1.5}
    assert out["zones"][1] == {"x": 2.0, "y": 2.0, "w": 1.0, "h": 1.0, "rot": 0.0}
    assert len(out["zones"]) == 2
    assert "error" in n.keepout_save({"x": 0, "y": 0, "w": 1, "h": 1, "index": 9})
    assert "error" in n.keepout_save({"x": 0, "y": 0, "w": 1, "h": 1, "index": "x"})


def test_keepout_save_rejects_garbage(tmp_path):
    n = _keepout_node(tmp_path)
    assert "error" in n.keepout_save({"x": 0.0})
    assert "error" in n.keepout_save({"x": "a", "y": 0, "w": 0, "h": 0})
    assert "error" in n.keepout_save({"x1": None, "y1": 0, "x2": 0, "y2": 0})
    assert n._keepout_zones == []


def test_keepout_save_clamps(tmp_path):
    """Centres clamp to the grid neighbourhood, sides to a sane range, rot
    wraps into (-pi, pi]."""
    n = _keepout_node(tmp_path)
    out = n.keepout_save({"x": 99, "y": -99, "w": 50, "h": 0.001, "rot": 3 * math.pi})
    assert out["zones"] == [{"x": 20.0, "y": -20.0,
                             "w": 12.0, "h": 0.05,
                             "rot": pytest.approx(math.pi)}]


def test_keepout_delete_and_clear(tmp_path):
    n = _keepout_node(tmp_path)
    n.keepout_save({"x": 0, "y": 0, "w": 1, "h": 1})
    n.keepout_save({"x": 2, "y": 2, "w": 1, "h": 1})
    assert "error" in n.keepout_delete({"index": 9})
    out = n.keepout_delete({"index": 0})
    assert out["ok"] and len(n._keepout_zones) == 1
    assert n.keepout_clear()["ok"]
    assert n._keepout_zones == []
    assert n.telemetry._keepout_zones == []


def test_keepout_zone_cap(tmp_path):
    n = _keepout_node(tmp_path)
    for i in range(KEEPOUT_MAX_ZONES):
        assert n.keepout_save({"x": i, "y": i, "w": 1, "h": 1})["ok"]
    assert "error" in n.keepout_save({"x": 99, "y": 99, "w": 1, "h": 1})


def test_keepout_load_migrates_legacy_json(tmp_path):
    """An old keepout.json (corner rows) loads into the normalized shape."""
    import json
    p = tmp_path / "keepout.json"
    p.write_text(json.dumps({"zones": [{"x1": -1, "y1": -2, "x2": 0, "y2": -1}]}))
    n = _keepout_node(tmp_path)
    n._keepout_path = str(p)
    n._load_keepout()
    assert n._keepout_zones == [{"x": -0.5, "y": -1.5, "w": 1.0, "h": 1.0, "rot": 0.0}]
    assert n.telemetry._keepout_zones == [(-0.5, -1.5, 1.0, 1.0, 0.0)]
