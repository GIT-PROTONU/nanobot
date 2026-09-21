"""Offline tests (ROS-free) for web_control's canned-move controller — the
POST /move {"dist" m, "deg" deg} maneuver the Drive card rides on. All the
math lives in the pure `_maneuver_step` (web_server.py module level), so the
sim here integrates a noise-free differential-drive rollout against it:

    * drive phase: progress along the phase-start heading, decel band, backward
      sign, veer (off-axis motion must NOT pad the projected distance)
    * phase handover: after the drive, the turn re-snapshots its start yaw
    * turn phase: P-law clamp to w_max, floor for tiny errors, wrap-around
      target (+/- beyond pi), finish band
    * turn-only maneuvers skip the drive phase entirely
    * the full _run_maneuver loop on a fake node: odom feedback -> shared (v,w)
      state -> stop + terminal SSE state (no /cmd_vel published by the loop)

    pixi run test
"""
import math
import threading

import pytest

import web_control.web_server as ws
from web_control.web_server import (_maneuver_step, _wrap_angle,
                                    MOVE_DIST_TOL, MOVE_TURN_MIN_W, MOVE_TURN_TOL)

V_MAX = 0.12      # m/s
W_MAX = 0.5       # rad/s
KP = 2.5          # 1/s
DT = 0.1          # s sim tick


def _st(dist=0.0, deg=0.0):
    return {"phase": "drive", "dist": dist, "deg": deg,
            "x0": None, "y0": None, "th0": None, "e0": 0.0,
            "v_max": V_MAX, "w_max": W_MAX, "turn_kp": KP}


def _run(dist=0.0, deg=0.0, start=(0.0, 0.0, 0.0), tick=None):
    """Integrate a perfect rollout of the maneuver; returns (ticks, final pose,
    progress of last tick). `tick(pose, v, w) -> pose` may inject veer/faults."""
    pose = start
    st = _st(dist, deg)
    n = 0
    prog = 0.0
    while n < 6000:                                   # ~10 min cap
        v, w, done, prog, _err = _maneuver_step(st, pose)
        if done:
            return n, pose, prog
        y = pose[2]
        pose = (pose[0] + v * math.cos(y) * DT,
                pose[1] + v * math.sin(y) * DT,
                _wrap_angle(y + w * DT))
        if tick:
            pose = tick(pose, v, w)
        n += 1
    raise AssertionError("maneuver never finished")


# ---- drive phase -----------------------------------------------------------------
def test_drive_forward_reaches_target():
    n, pose, prog = _run(dist=0.5)
    assert abs(pose[0] - 0.5) < 2 * MOVE_DIST_TOL
    assert prog >= 1.0 - 1e-9
    assert n * DT < 0.5 / V_MAX + 5.0               # ~target time, not a crawl


def test_drive_backward_sign():
    _n, pose, _p = _run(dist=-0.3)
    assert pose[0] < -0.3 + 2 * MOVE_DIST_TOL
    assert pose[1] == 0.0                            # never strafed


def test_drive_projection_ignores_veer():
    """The target distance is the projection on the phase-start HEADING, not path
    length: a drive along an arbitrary heading must land exactly N metres out
    along that heading (trim veer can pad path length, not the projection)."""
    _n, pose, _p = _run(dist=0.4, start=(0.0, 0.0, math.pi / 2))
    assert abs(pose[1] - 0.4) < 2 * MOVE_DIST_TOL    # drove along +y (the heading)
    assert abs(pose[0]) < 1e-9


# ---- phase handover --------------------------------------------------------------
def test_drive_then_turn_re_snapshots_yaw():
    """The turn phase must measure its error from the pose AFTER the drive (the
    drive start yaw would be wrong the moment the drive itself turned/veered)."""
    st = _st(dist=0.3, deg=30)
    VEER = 0.2                                       # rad/s yaw slip while driving
    pose = (0.0, 0.0, 0.0)
    turn_start = None
    for _ in range(6000):
        v, w, done, _p, _e = _maneuver_step(st, pose)
        if st["phase"] == "turn" and turn_start is None and st["th0"] is not None:
            turn_start = st["th0"]                  # the snapshot the turn will use
        if done:
            break
        pose = (pose[0] + v * math.cos(pose[2]) * DT,
                pose[1] + v * math.sin(pose[2]) * DT,
                _wrap_angle(pose[2] + (w + (VEER if v else 0.0)) * DT))
    assert turn_start is not None
    assert turn_start > 0.3                         # it IS the veered yaw, not the 0 drive start
    assert abs(_wrap_angle(pose[2] - (turn_start + math.radians(30)))) <= MOVE_TURN_TOL + 1e-9


def test_turn_only_skips_drive():
    st = _st(deg=90)
    v, w, done, prog, err = _maneuver_step(st, (0.0, 0.0, 0.0))
    assert st["phase"] == "turn"                     # no drive phase at all
    assert v == 0.0
    assert w > 0.0                                   # CCW for +deg
    assert not done


# ---- turn phase ------------------------------------------------------------------
def test_turn_p_law_clamped():
    st = _st(deg=170)
    _v, w, done, _p, err = _maneuver_step(st, (0.0, 0.0, 0.0))
    assert abs(err - math.radians(170)) < 1e-9
    assert w == W_MAX                                # clamped at the cap


def test_turn_floor_saves_low_kp():
    """With a low turn_kp the P law under the tolerance band would crawl forever;
    the MOVE_TURN_MIN_W floor keeps it converging (with the default kp the band is
    above the floor point, so this only guards retuned params)."""
    st = _st(deg=2)
    st["turn_kp"] = 0.5                              # 0.5 * 0.035 rad = 0.017 < floor
    _v, w, done, _p, _e = _maneuver_step(st, (0.0, 0.0, 0.0))
    assert not done
    assert w == math.copysign(MOVE_TURN_MIN_W, 1)


def test_turn_inside_tolerance_finishes():
    st = _st(deg=1)                                  # 1° = 0.017 rad < 0.03 rad band
    _v, w, done, _p, _e = _maneuver_step(st, (0.0, 0.0, 0.0))
    assert done
    assert w == 0.0


def test_turn_finishes_within_tolerance():
    _n, pose, _p = _run(deg=120)
    assert abs(pose[2] - math.radians(120)) <= MOVE_TURN_TOL + 1e-9


def test_turn_negative_clockwise():
    _n, pose, _p = _run(deg=-75)
    assert abs(_wrap_angle(pose[2] - math.radians(-75))) <= MOVE_TURN_TOL + 1e-9


def test_turn_wraps_beyond_pi():
    """+200° from a start heading near +pi must wrap cleanly, not spin the long
    way or wedge on an un-wrapped error."""
    start = (0.0, 0.0, math.radians(170))
    _n, pose, _p = _run(deg=200, start=start)
    final = _wrap_angle(pose[2])
    expect = _wrap_angle(math.radians(170) + math.radians(200))   # = -170°
    assert abs(_wrap_angle(final - expect)) <= MOVE_TURN_TOL + 1e-9


def test_turn_zero_deg_finishes_immediately():
    st = _st(dist=0.2, deg=0)
    _v, _w, done, _p, _e = _maneuver_step(st, (0.0, 0.0, 0.0))    # drive tick
    assert not done
    # fast-forward the drive to done, then the turn with deg=0 must finish at once
    st2 = _st(dist=0.0, deg=0)                                    # deg=0, dist=0 never
    st2["phase"] = "turn"
    st2["deg"] = 0
    _v, _w, done2, _p2, _e2 = _maneuver_step(st2, (1.0, 1.0, 0.5))
    assert done2


# ---- full _run_maneuver loop on a fake node --------------------------------------
class _FakePub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeLog:
    def info(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class _FakeTelemetry:
    """Scripted /odom: the fake node's loop reads _odom; the test advances the
    pose BETWEEN reads via the commanded (v,w) the loop writes to the drive
    state — a perfect kinematic rollout with the loop's own 10 Hz tick."""

    def __init__(self):
        self._odom = (0.0, 0.0, 0.0)
        self._clients = 1
        self._goal_status = "idle"


def _fake_node():
    n = ws.WebServerNode.__new__(ws.WebServerNode)   # skip __init__ entirely
    n.telemetry = _FakeTelemetry()
    n._drive_pub = _FakePub()
    n._drive_lock = threading.Lock()
    n._drive_v = n._drive_w = 0.0
    n._drive_at = 0.0
    n._man_lock = threading.Lock()
    n._man_stop = threading.Event()
    n._man_cancel = False
    n._man_running = False
    n._maneuver_state = {"active": False, "phase": "idle"}
    n.get_logger = _FakeLog
    return n


def test_run_maneuver_completes_and_stops(monkeypatch):
    monkeypatch.setattr(ws, "MOVE_CTRL_HZ", 100000)   # shrink the tick sleep
    n = _fake_node()
    req = {"dist": 0.3, "deg": 0, "v_max": V_MAX, "w_max": W_MAX,
           "turn_kp": KP, "timeout": 30.0}

    def advance():                                    # odom follows the loop's commands
        with n._drive_lock:
            v, w = n._drive_v, n._drive_w
        x, y, yaw = n.telemetry._odom
        n.telemetry._odom = (x + v * math.cos(yaw) * DT,
                             y + v * math.sin(yaw) * DT, _wrap_angle(yaw + w * DT))

    real_step = ws._maneuver_step

    def step_and_advance(st, pose):
        out = real_step(st, pose)
        advance()
        return out

    monkeypatch.setattr(ws, "_maneuver_step", step_and_advance)
    n._run_maneuver(req)
    assert n._maneuver_state["active"] is False
    assert n._maneuver_state["result"] == "done"
    assert n._drive_v == 0.0 and n._drive_w == 0.0    # disarmed for the keepalive
    assert len(n._drive_pub.published) == 1           # exactly ONE direct stop
    stop = n._drive_pub.published[0]
    assert stop.linear.x == 0.0 and stop.angular.z == 0.0
    x, _y, _yaw = n.telemetry._odom
    assert abs(x - 0.3) < 3 * MOVE_DIST_TOL


def test_run_maneuver_cancel(monkeypatch):
    monkeypatch.setattr(ws, "MOVE_CTRL_HZ", 100000)
    n = _fake_node()
    req = {"dist": 5.0, "deg": 0, "v_max": V_MAX, "w_max": W_MAX,
           "turn_kp": KP, "timeout": 30.0}
    real_step = ws._maneuver_step
    ticks = [0]

    def step_and_cancel(st, pose):
        out = real_step(st, pose)
        ticks[0] += 1
        if ticks[0] == 5:                             # mid-drive joystick takeover
            with n._man_lock:
                n._man_cancel = True
        return out

    monkeypatch.setattr(ws, "_maneuver_step", step_and_cancel)
    n._run_maneuver(req)
    assert n._maneuver_state["result"] == "cancelled"
    assert n._maneuver_state["active"] is False
    assert abs(n.telemetry._odom[0]) < 1.0            # nowhere near the 5 m target


# ---- canned-move speed config (GET/POST /move/config) -----------------------------
def test_clamp_move_cfg():
    # in-range passes through; out-of-range clamps to the slider bounds
    assert ws._clamp_move_cfg(0.2, 0.4) == (0.2, 0.4)
    assert ws._clamp_move_cfg(0.0, 5.0) == (ws.MOVE_LIN_RANGE[0], ws.MOVE_ANG_RANGE[1])
    assert ws._clamp_move_cfg(-1.0, 0.01) == (ws.MOVE_LIN_RANGE[0], ws.MOVE_ANG_RANGE[0])
    # garbage is a ValueError (update_move_config turns it into an error reply)
    with pytest.raises(ValueError):
        ws._clamp_move_cfg("fast", 0.5)
