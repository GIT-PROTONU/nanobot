"""Offline tests (ROS-free) for web_control's LDS spin-down persistence — the
Lidar card's Idle spin-down toggle / Spin-down-after slider / Spin target must
survive a restart. The contract lives in `WebServerNode._persist_lds_params`, the
one on-set-parameters callback that fires for EVERY param write on the node:

    * a batch with none of the LDS cluster's params writes nothing (the vision
      sliders must not touch lds.json)
    * an LDS param write snapshots the WHOLE cluster — proposed values overlay the
      current ones (the callback runs before the values are applied), so the file
      holds exactly what will be in effect
    * unrelated keys in the same batch are ignored
    * the callback never vetoes and never raises (a broken get_parameter must not
      fail the param set — rclpy applies the callback's result to every parameter)

The boot re-apply (declare -> lds.json wins over robot.yaml) is exercised live by
scripts/smoke_test.py; the pure decision logic is what is pinned here.

    pixi run test
"""
import json

import web_control.web_server as ws
from web_control.web_server import LDS_PERSIST_KEYS


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
    """A WebServerNode shell (skip __init__) with stubbed param reads + a private
    lds.json under tmp_path (instance attributes shadow the bound methods)."""
    n = ws.WebServerNode.__new__(ws.WebServerNode)
    n._lds_path = str(tmp_path / "lds.json")
    n._lds_settings_file = lambda: n._lds_path
    n.get_parameter = lambda name: _Param(name, dict(values)[name])
    n.get_logger = lambda: _LOG
    return n


_LOG = _FakeLog()


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def test_unrelated_params_write_nothing(tmp_path):
    n = _node(tmp_path, {"lds_idle_enable": True, "lds_idle_secs": 60.0,
                         "lds_manual_secs": 300.0, "lds_active_rpm": 300.0})
    r = n._persist_lds_params([_Param("vision_novelty_alert", 0.4),
                               _Param("vision_dark_threshold", 30.0)])
    assert r.successful is True
    import os
    assert not os.path.exists(n._lds_path)


def test_lds_write_snapshots_whole_cluster(tmp_path):
    n = _node(tmp_path, {"lds_idle_enable": True, "lds_idle_secs": 60.0,
                         "lds_manual_secs": 300.0, "lds_active_rpm": 300.0})
    n._persist_lds_params([_Param("lds_idle_secs", 123.0)])
    snap = _read(n._lds_path)
    # the proposed value lands; the untouched keys keep their current values
    assert snap["lds_idle_secs"] == 123.0
    assert snap["lds_idle_enable"] is True
    assert snap["lds_manual_secs"] == 300.0
    assert snap["lds_active_rpm"] == 300.0
    assert set(snap) == set(LDS_PERSIST_KEYS)


def test_mixed_batch_proposed_values_win(tmp_path):
    n = _node(tmp_path, {"lds_idle_enable": True, "lds_idle_secs": 60.0,
                         "lds_manual_secs": 300.0, "lds_active_rpm": 300.0})
    # a Spin-slider drag rides in with a toggle change: both proposed values win,
    # the vision param is ignored, the rest keep current
    n._persist_lds_params([_Param("lds_idle_enable", False),
                           _Param("vision_looming_alert", 0.5),
                           _Param("lds_active_rpm", 200.0)])
    snap = _read(n._lds_path)
    assert snap["lds_idle_enable"] is False
    assert snap["lds_active_rpm"] == 200.0
    assert snap["lds_idle_secs"] == 60.0
    assert snap["lds_manual_secs"] == 300.0


def test_never_vetoes_or_raises_on_broken_node(tmp_path):
    n = _node(tmp_path, {})
    n.get_parameter = lambda name: (_ for _ in ()).throw(RuntimeError("gone"))
    r = n._persist_lds_params([_Param("lds_idle_secs", 45.0)])
    assert r.successful is True          # the param set must still be accepted
    assert n.get_logger().warnings      # and the failure logged, not swallowed silent


def test_result_covers_every_parameter(tmp_path):
    # rclpy applies one SetParametersResult to the whole batch — a rejection would
    # fail ALL parameters in the request, so the callback must always say yes
    n = _node(tmp_path, {"lds_idle_enable": True, "lds_idle_secs": 60.0,
                         "lds_manual_secs": 300.0, "lds_active_rpm": 300.0})
    for batch in ([_Param("unrelated", 1)], [_Param("lds_idle_secs", 5.0)]):
        assert n._persist_lds_params(batch).successful is True
