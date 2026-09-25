"""Offline tests (ROS-free) for the drawable keep-out zones — the keepout
handler set (save/delete/clear/persist-shape, cap) and telemetry's mask
rasterizer (rects → cells on the live /map geometry, alignment with the grid
origin/resolution, re-emit on a rebuilt map, empty-mask clear).

    pixi run test
"""
import pytest

from nav_msgs.msg import OccupancyGrid

from web_control.telemetry import KEEPOUT_MAX_ZONES, TelemetryHub

from test_nav_telemetry import _FakeLog, _FakePub, _FakeClient, _FakeNode, _grid


def test_keepout_mask_none_without_a_map():
    """No /map geometry yet (boot) → nothing published, no crash."""
    h = TelemetryHub(_FakeNode())
    h.set_keepout_zones([{"x1": 0, "y1": 0, "x2": 1, "y2": 1}])
    assert h._keepout_zones


def test_keepout_mask_rasterizes_rects_on_map_geometry():
    """A 2x2 m map @0.05 m = 40x40 cells, origin (-2,-3) (the _grid helper).
    One rect from (-1,-2) to (-0.5,-1.5) must paint exactly the cells covering
    that box: columns/rows 20..29 inclusive."""
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
    painted = [(i % 40, i // 40) for i, v in enumerate(data) if v != 0]
    expect = [(c, r) for r in range(20, 30) for c in range(20, 30)]
    assert sorted(painted) == sorted(expect)
    assert all(data[i] == 100 for i, v in enumerate(data) if v != 0)
    # the filter-info topic rides along, pointing at the mask topic (two: the
    # initial _on_map latch with no zones + the set_keepout_zones re-emit)
    infos = h._pubs["/keepout_filter_info"][0].published
    assert len(infos) == 2
    assert infos[-1].filter_mask_topic == "keepout_mask"
    assert infos[-1].type == 0


def test_keepout_mask_covers_full_grid_and_outside():
    """A rect larger than the map clamps to the grid; one entirely outside
    paints nothing (but the publish still happens)."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h._pubs["/keepout_mask"][0].published.clear()
    h.set_keepout_zones([{"x1": -99, "y1": -99, "x2": 99, "y2": 99},
                         {"x1": 50, "y1": 50, "x2": 60, "y2": 60}])
    m = h._pubs["/keepout_mask"][0].published[-1]
    data = list(m.data)
    assert sum(1 for v in data if v != 0) == 40 * 40
    assert all(v == 100 for v in data if v != 0)


def test_keepout_reemit_on_map_change():
    """A rebuilt /map (new dims/origin) re-rasterizes + re-latches the mask so
    the KeepoutFilter's cells stay aligned with the new grid."""
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    n_after_first = len(h._pubs["/keepout_mask"][0].published)
    h.set_keepout_zones([{"x1": 0, "y1": 0, "x2": 0.5, "y2": 0.5}])
    n_zones = len(h._pubs["/keepout_mask"][0].published)
    h._on_map(_grid(80, 80, res=0.05))       # slam restarted, finer grid
    assert len(h._pubs["/keepout_mask"][0].published) == n_zones + 1
    m = h._pubs["/keepout_mask"][0].published[-1]
    assert m.info.width == 80 and m.info.height == 80
    assert n_after_first >= 1                # initial empty-mask latch happened


def test_keepout_clear_publishes_empty_mask():
    h = TelemetryHub(_FakeNode())
    h._on_map(_grid(40, 40, res=0.05))
    h.set_keepout_zones([{"x1": 0, "y1": 0, "x2": 1, "y2": 1}])
    h._pubs["/keepout_mask"][0].published.clear()
    h.set_keepout_zones([])
    m = h._pubs["/keepout_mask"][0].published[-1]
    assert all(v == 0 for v in m.data)       # the lethal latch clears


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
        get_keepout = WebServerNode.get_keepout
        keepout_save = WebServerNode.keepout_save
        keepout_delete = WebServerNode.keepout_delete
        keepout_clear = WebServerNode.keepout_clear

    return _N2()


def test_keepout_save_roundtrip_and_persist(tmp_path):
    n = _keepout_node(tmp_path)
    out = n.keepout_save({"x1": 0.0, "y1": 0.0, "x2": 0.5, "y2": 0.5})
    assert out["ok"] and len(out["zones"]) == 1
    assert n.get_keepout()["zones"] == [{"x1": 0.0, "y1": 0.0, "x2": 0.5, "y2": 0.5}]
    # telemetry mirror took it (normalized to tuples)
    assert n.telemetry._keepout_zones == [(0.0, 0.0, 0.5, 0.5)]
    # reload from disk restores the zone
    n2 = _keepout_node(tmp_path)
    n2._load_keepout()
    assert n2._keepout_zones == [{"x1": 0.0, "y1": 0.0, "x2": 0.5, "y2": 0.5}]


def test_keepout_save_rejects_garbage(tmp_path):
    n = _keepout_node(tmp_path)
    assert "error" in n.keepout_save({"x1": 0.0})
    assert "error" in n.keepout_save({"x1": "a", "y1": 0, "x2": 0, "y2": 0})
    assert n._keepout_zones == []


def test_keepout_delete_and_clear(tmp_path):
    n = _keepout_node(tmp_path)
    n.keepout_save({"x1": 0, "y1": 0, "x2": 1, "y2": 1})
    n.keepout_save({"x1": 2, "y1": 2, "x2": 3, "y2": 3})
    assert "error" in n.keepout_delete({"index": 9})
    out = n.keepout_delete({"index": 0})
    assert out["ok"] and len(n._keepout_zones) == 1
    assert n.keepout_clear()["ok"]
    assert n._keepout_zones == []
    assert n.telemetry._keepout_zones == []


def test_keepout_zone_cap(tmp_path):
    n = _keepout_node(tmp_path)
    for i in range(KEEPOUT_MAX_ZONES):
        assert n.keepout_save({"x1": i, "y1": i, "x2": i + 1, "y2": i + 1})["ok"]
    assert "error" in n.keepout_save({"x1": 99, "y1": 99, "x2": 100, "y2": 100})
