"""Offline unit tests for the stress-test manager (ROS-free):

    pixi run python -m pytest src/web_control/test

Uses short (1-2s), single-worker runs so the suite stays fast; still exercises real
subprocesses (this module IS subprocess orchestration, so that's what's worth testing).
"""
import time

from web_control.stress import StressTest


def test_start_reports_active_status():
    st = StressTest()
    res = st.start(duration=2.0, workers=1)
    try:
        assert res["active"] is True
        assert res["cpu_workers"] == 1
        s = st.status()
        assert s["active"] is True
        assert s["remaining"] <= 2.0
    finally:
        st.stop()


def test_duration_clamped_to_max_duration():
    st = StressTest(max_duration=1.0)
    res = st.start(duration=100.0, workers=1)
    try:
        assert res["duration"] == 1.0                # clamped, not the requested 100
    finally:
        st.stop()


def test_duration_clamped_to_minimum():
    st = StressTest(max_duration=300.0)
    res = st.start(duration=0.0, workers=1)
    try:
        assert res["duration"] >= 1.0                 # never a zero/negative-length run
    finally:
        st.stop()


def test_workers_clamped_to_cpu_count():
    st = StressTest()
    res = st.start(duration=1.0, workers=999999)
    try:
        import os
        assert res["cpu_workers"] <= (os.cpu_count() or 1)
    finally:
        st.stop()


def test_single_flight_guard():
    st = StressTest()
    st.start(duration=2.0, workers=1)
    try:
        res = st.start(duration=2.0, workers=1)
        assert "error" in res                         # can't start a second run concurrently
    finally:
        st.stop()


def test_stop_ends_the_run_and_kills_workers():
    st = StressTest()
    st.start(duration=30.0, workers=1)
    procs = list(st._procs)
    res = st.stop()
    assert res["active"] is False
    for p in procs:
        assert p.wait(timeout=2.0) is not None        # actually terminated, not left running
    assert st.status()["active"] is False


def test_stop_when_not_running_is_a_harmless_noop():
    st = StressTest()
    res = st.stop()
    assert res["active"] is False


def test_status_reflects_natural_expiry():
    st = StressTest()
    st.start(duration=1.0, workers=1)
    time.sleep(1.6)                                    # past the worker's own deadline
    assert st.status()["active"] is False


def test_thermal_abort_stops_the_run_early():
    st = StressTest(abort_temp_c=50.0, read_temp=lambda: 60.0)
    st.start(duration=10.0, workers=1)
    try:
        time.sleep(1.5)                                # watchdog checks the temp every ~1s
        assert st.status()["active"] is False
    finally:
        st.stop()


def test_thermal_abort_disabled_by_default_keeps_running():
    st = StressTest(read_temp=lambda: 999.0)            # abort_temp_c defaults to 0 (disabled)
    st.start(duration=2.0, workers=1)
    try:
        time.sleep(1.0)
        assert st.status()["active"] is True
    finally:
        st.stop()
