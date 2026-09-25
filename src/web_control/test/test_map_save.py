"""Offline tests (ROS-free) for slam map persistence glue — the POST /map/save
handler (path resolution, disabled-flag, service call shape) and POST /map/clear's
map-file retirement (the .posegraph/.data pair renamed so a slam boot with
map_file_name set re-clears instead of re-loading).

    pixi run test
"""
import os

import pytest


class _Log:
    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


def _node(tmp_path, map_file, telemetry):
    """A WebServerNode stand-in for save_map/clear_map's map-file handling.
    Real methods borrowed unbound; the actual systemctl/sudo call is stubbed."""
    from web_control.web_server import WebServerNode

    class _N2:
        def __init__(self):
            self.telemetry = telemetry
            self._map_file = map_file
            self._log = _Log()
            self.commands = []

        def get_logger(self):
            return self._log

        def get_parameter_or(self, name, default):
            return default if map_file is None else type(default)(
                name, type(default).Type.STRING, map_file)

        def get_parameter(self, name):
            raise KeyError(name)

        def create_client(self, srv, name):
            raise AssertionError("patched per-test")

        def cancel_goal(self):
            pass

        clear_map = WebServerNode.clear_map
        save_map = WebServerNode.save_map

    return _N2()


def _telemetry():
    from test_nav_telemetry import _FakeNode, _hub
    h = _hub()
    return h


def test_save_map_disabled_when_no_file(tmp_path):
    n = _node(tmp_path, None, _telemetry())
    # get_parameter_or returns the default param whose value is "" here
    out = n.save_map()
    assert "error" in out


def test_save_map_service_call_shape(tmp_path, monkeypatch):
    """The handler builds a SerializePoseGraph.Request(filename=...) and fires
    it on the slam service — verify the request + the returned path expand."""
    from web_control.web_server import WebServerNode

    captured = {}

    class _Client:
        def service_is_ready(self):
            return True

        def call_async(self, req):
            captured["req"] = req

    n = _node(tmp_path, str(tmp_path / "nano_map.posegraph"), _telemetry())

    def _fake_create_client(self, srv, name):
        captured["client"] = (srv, name)
        return _Client()

    monkeypatch.setattr(n.__class__, "create_client", _fake_create_client)
    out = n.save_map()
    assert out["ok"] and captured["client"][1] == "serialize_pose_graph"
    assert captured["req"].filename == str(tmp_path / "nano_map.posegraph")


def test_clear_map_retires_saved_map_file(tmp_path, monkeypatch):
    """With a saved .posegraph + .posegraph.data pair on disk, /map/clear
    renames both (to .bak) BEFORE the restart so a map_file_name-boot re-clears."""
    from web_control.web_server import WebServerNode

    pg = tmp_path / "nano_map.posegraph"
    data = tmp_path / "nano_map.posegraph.data"
    pg.write_text("posegraph")
    data.write_text("data")
    n = _node(tmp_path, str(pg), _telemetry())

    def fake_run(cmd, **k):
        n.commands.append(cmd)
        class _R:
            returncode = 0
            stderr = ""
        return _R()

    monkeypatch.setattr("web_control.web_server.subprocess.run", fake_run)
    # skip the systemctl prefix check by letting it run through fake sudo
    out = n.clear_map()
    assert out["ok"]
    assert not pg.exists() and not data.exists()
    assert os.path.exists(str(pg) + ".bak")
    assert os.path.exists(str(data) + ".bak")

    assert any("nano-slam" in " ".join(c) for c in n.commands)


def test_clear_map_without_file_still_restarts(tmp_path, monkeypatch):
    n = _node(tmp_path, None, _telemetry())

    def fake_run(cmd, **k):
        n.commands.append(cmd)
        class _R:
            returncode = 0
            stderr = ""
        return _R()

    monkeypatch.setattr("web_control.web_server.subprocess.run", fake_run)
    out = n.clear_map()
    assert out["ok"]
    assert any("nano-slam" in " ".join(c) for c in n.commands)


def test_clear_map_sudo_failure_reported(tmp_path, monkeypatch):
    n = _node(tmp_path, None, _telemetry())

    def fake_run(cmd, **k):
        class _R:
            returncode = 1
            stderr = "a password is required"
        return _R()

    monkeypatch.setattr("web_control.web_server.subprocess.run", fake_run)
    out = n.clear_map()
    assert not out["ok"] and "sudoers" in out["error"]
