"""Offline tests for the /dev/shm scan-blob writer shared by the LDS driver and the
(dev) Gazebo bridge — the byte stream the web UI's lidar panel (and, historically,
the SLAM map panel) parses. The web never reads the ROS topic, only this file, so
its layout is a de-facto wire format:

    one JSON header line ({"seq","amin","ainc","n"} + extras), '\\n',
    then the raw float32 ranges. Atomic via .tmp + os.replace.

    pixi run test
"""
import array
import json
import math
import os

import pytest

from lds_driver_py.scan_blob import SCAN_FILE, write_scan_blob


def _read_blob(path):
    with open(path, "rb") as f:
        blob = f.read()
    header_line, _, rest = blob.partition(b"\n")
    return json.loads(header_line.decode()), rest


def test_header_shape_and_float32_payload(tmp_path):
    p = str(tmp_path / "scan.bin")
    ranges = [0.5, 1.25, 2.0, math.inf, 0.0]
    write_scan_blob(7, -math.pi, math.pi / 180, ranges, path=p)
    header, rest = _read_blob(p)
    assert header["seq"] == 7
    assert header["amin"] == pytest.approx(-math.pi)
    assert header["ainc"] == pytest.approx(math.pi / 180)
    assert header["n"] == 5
    assert len(rest) == 5 * 4                       # float32 per beam
    vals = array.array("f")
    vals.frombytes(rest)
    assert list(vals) == pytest.approx([0.5, 1.25, 2.0, math.inf, 0.0])


def test_inf_packs_as_float32_inf(tmp_path):
    # "no hit" must survive as float32 inf (the browser treats inf as no point);
    # a NaN here would silently black-hole beams in the UI.
    p = str(tmp_path / "scan.bin")
    write_scan_blob(1, 0.0, 0.01, [math.inf, 1.0, math.inf], path=p)
    _, rest = _read_blob(p)
    vals = array.array("f")
    vals.frombytes(rest)
    assert math.isinf(vals[0]) and vals[0] > 0
    assert vals[1] == pytest.approx(1.0)
    assert math.isinf(vals[2])


def test_extra_header_fields_merge(tmp_path):
    p = str(tmp_path / "scan.bin")
    write_scan_blob(3, 0.0, 0.01, [1.0], path=p, extra={"lost": 2, "err": 0})
    header, _ = _read_blob(p)
    assert header["lost"] == 2
    assert header["err"] == 0
    assert header["n"] == 1                         # base keys untouched


def test_extra_can_override_base_keys(tmp_path):
    p = str(tmp_path / "scan.bin")
    write_scan_blob(3, 0.0, 0.01, [1.0], path=p, extra={"amin": -2.0})
    header, _ = _read_blob(p)
    assert header["amin"] == -2.0                   # documented: extras win


def test_write_is_atomic_no_tmp_left_behind(tmp_path):
    p = str(tmp_path / "scan.bin")
    write_scan_blob(1, 0.0, 0.01, [1.0, 2.0], path=p)
    assert os.path.exists(p)
    assert not os.path.exists(p + ".tmp")           # os.replace consumed it
    write_scan_blob(2, 0.0, 0.01, [3.0], path=p)    # overwrite in place
    header, rest = _read_blob(p)
    assert header["seq"] == 2
    assert len(rest) == 4


def test_oserror_is_swallowed_not_raised(tmp_path):
    # a bad directory must not take down the publisher thread (best-effort blob)
    bad = str(tmp_path / "no-such-dir" / "scan.bin")
    write_scan_blob(1, 0.0, 0.01, [1.0], path=bad)  # must not raise
    assert not os.path.exists(bad)


def test_default_path_is_the_devshm_blob():
    assert SCAN_FILE == "/dev/shm/nano_scan.bin"


def test_reader_never_sees_a_torn_file(tmp_path):
    # simulate a reader polling while the writer runs: every observed state must be
    # a complete old or new blob (single header line + n*4 payload bytes), never a mix.
    p = str(tmp_path / "scan.bin")
    write_scan_blob(0, 0.0, 0.01, [0.0] * 10, path=p)
    seen_valid = 0
    for seq in range(1, 40):
        n = 10 + (seq % 7)
        write_scan_blob(seq, 0.0, 0.01, [0.0] * n, path=p)
        header, rest = _read_blob(p)                # no torn read allowed
        assert header["n"] == len(rest) // 4
        assert len(rest) % 4 == 0
        seen_valid += 1
    assert seen_valid == 39
