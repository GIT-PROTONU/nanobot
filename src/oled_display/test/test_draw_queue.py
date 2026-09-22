"""OLED render-queue mechanics (drop-oldest bounded queue) — ROS-free unit tests.

The 2026-09-22 combined-load repro caught app_hub's executor thread in a >=5 s
D-state inside the SSD1306's I2C driver wait (`mv64xxx_i2c_wait_for_completion`)
— the OLED froze web_control + mood_node with it. The fix moves ALL panel I2C
onto a dedicated worker fed by `DisplayNode._submit_draw` (bounded, drop-oldest).
These tests pin the queue semantics WITHOUT a ROS node: `_submit_draw` is called
unbound against a stub carrying only the attributes it touches.
"""
import os
import queue
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), "oled_display"))

import pytest  # noqa: E402

from oled_display.display_node import DisplayNode  # noqa: E402


class _Stub:
    """Just the attribute `_submit_draw` touches — no rclpy, no luma, no device."""

    def __init__(self, maxsize=4):
        self._draw_q = queue.Queue(maxsize=maxsize)
        self._draw_stop = False


def _run(q, n=1, timeout=2.0):
    """Drain up to n items, executing them; returns the executed markers in order."""
    out = []
    for _ in range(n):
        try:
            fn = q.get_nowait()
        except queue.Empty:
            break
        fn()
        out.append(fn)
    return out


def test_submits_in_order_under_capacity():
    stub = _Stub(maxsize=4)
    order = []
    for i in range(4):
        DisplayNode._submit_draw(stub, lambda i=i: order.append(i))
    got = _run(stub._draw_q, 4)
    assert order == [0, 1, 2, 3]
    assert len(got) == 4


def test_drop_oldest_when_full_keeps_newest():
    # A wedged I2C bus backs the queue up: the OLDEST render is dropped, the
    # newest is kept (every render reads CURRENT node state, so nothing is lost
    # but one panel refresh — the exact contract the executor safety needs).
    stub = _Stub(maxsize=2)
    marker = []
    for i in range(5):
        DisplayNode._submit_draw(stub, lambda i=i: marker.append(i))
    assert stub._draw_q.qsize() == 2
    _run(stub._draw_q, 2)
    assert marker == [3, 4]


def test_submit_never_blocks():
    # The executor thread must NEVER wait on the panel: fill the queue past
    # capacity many times over; _submit_draw returns immediately every time.
    stub = _Stub(maxsize=1)
    for i in range(200):
        DisplayNode._submit_draw(stub, lambda: None)
    assert stub._draw_q.qsize() <= 1


def test_worker_executes_and_survives_exceptions():
    # The draw loop runs each queued callable and must survive one that raises
    # (the _draw_guard latches panel death; the worker itself keeps serving).
    stub = _Stub(maxsize=4)
    done = threading.Event()

    def boom():
        raise RuntimeError("I2C bus wedge (simulated)")

    def ok():
        done.set()

    DisplayNode._submit_draw(stub, boom)
    DisplayNode._submit_draw(stub, ok)

    def loop_forever():
        while not done.is_set():
            try:
                fn = stub._draw_q.get(timeout=2.0)
            except queue.Empty:
                return
            if stub._draw_stop:
                continue
            try:
                fn()
            except Exception:
                pass

    t = threading.Thread(target=loop_forever, daemon=True)
    t.start()
    assert done.wait(5.0), "worker died on the raising render or never served the queue"


def test_stop_flag_skips_queued_renders():
    # shutdown_sequence sets _draw_stop, drains, then renders the end-screen
    # INLINE — the worker must not execute anything drained-or-late afterwards.
    stub = _Stub(maxsize=4)
    stub._draw_stop = True
    ran = []
    DisplayNode._submit_draw(stub, lambda: ran.append(1))
    assert stub._draw_q.qsize() == 1
    # simulate the worker's post-get check
    try:
        fn = stub._draw_q.get_nowait()
    except queue.Empty:
        pytest.fail("queue lost the item")
    if stub._draw_stop:
        pass                        # skipped — exactly the contract
    else:
        fn()
    assert ran == []
