"""IMU mounting-interference self-test: automates walking the loose IMU around by
hand (see the IMU card's mag-noise hint) into a scripted, repeatable sequence.
While the robot sits parked, it cycles each nearby actuator one at a time -- LDS
spin, cooling fan, LED, an optional brief motor wiggle -- and scores how much each
one disturbs the magnetometer (and, for the motor phase, the reported yaw) against
a quiet baseline. Mag *magnitude* is rotation-invariant, so even the motor-wiggle
phase attributes cleanly without needing the robot to hold still during it.

Reuses the exact publishers web_control already gates behind `skills_allow_actions`
(the project's one flag for "web_control physically actuates something") rather
than opening a second, unguarded actuation path -- see WebServerNode.
_publish_skill_action. The fan phase goes through sys_monitor's `fan_override`
PARAM (not a raw /fan_pwm publish), since sys_monitor is /fan_pwm's continuous
owner and a competing publish would just be overwritten on its next tick.

Single-flight; aborts (and restores every touched actuator) if the robot stops
being provably stationary mid-run -- picked up, or driven.
"""
import threading
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

MIN_PHASE_SECS = 0.5
MAX_PHASE_SECS = 15.0


def _wrap_deg(d):
    return ((d + 180.0) % 360.0) - 180.0


def _clamp_secs(v, default):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return default
    return min(MAX_PHASE_SECS, max(MIN_PHASE_SECS, v))


class IMUInterferenceTest:
    """One run at a time. Needs `node` for its gated skill-action publishers
    (`node._skill_pubs`) and its telemetry hub (`node.telemetry`) for live mag/
    euler samples, the stationary check, and the fan_override param call."""

    def __init__(self, node, logger=None, lds_rpm=300.0, motor_ang=0.35):
        self._node = node
        self._log = logger or (lambda *a, **k: None)
        self.lds_rpm = float(lds_rpm)
        self.motor_ang = float(motor_ang)
        self._lock = threading.Lock()
        self._active = False
        self._phase = ""
        self._phase_i = 0
        self._phase_n = 0
        self._results = []
        self._error = ""

    # ---- public -----------------------------------------------------------------
    def start(self, include_motor=False, base_secs=3.0, lds_secs=4.0,
              fan_secs=4.0, led_secs=2.0, motor_secs=1.0):
        with self._lock:
            if self._active:
                return {"error": "interference test already running"}
            n = self._node
            if not getattr(n, "_skills_allow_actions", False):
                return {"error": "requires skills_allow_actions (Skills card) — "
                                  "needed to actuate the LDS/fan/LED/motors"}
            if n.telemetry._mag is None or n.telemetry._eul is None:
                return {"error": "IMU mag/euler not streaming yet"}
            if any(n._susp_eff()):
                return {"error": "robot is picked up — set it down first"}
            lin, ang = n.telemetry._cmd_vel
            eps = n.get_parameter("vision_bumper_cmd_eps").value
            if abs(lin) > eps or abs(ang) > eps:
                return {"error": "robot is being driven — park it first"}
            self._active = True
            self._error = ""
            self._results = []
            self._phase = "starting"
            self._phase_i = 0
            secs = (_clamp_secs(base_secs, 3.0), _clamp_secs(lds_secs, 4.0),
                    _clamp_secs(fan_secs, 4.0), _clamp_secs(led_secs, 2.0),
                    _clamp_secs(motor_secs, 1.0))
            self._phase_n = 5 if include_motor else 4
            threading.Thread(target=self._run, args=(bool(include_motor),) + secs,
                              daemon=True).start()
            return self.status()

    def stop(self):
        with self._lock:
            self._active = False
        return self.status()

    def status(self):
        with self._lock:
            return {"active": self._active, "phase": self._phase,
                    "phase_i": self._phase_i, "phase_n": self._phase_n,
                    "results": list(self._results), "error": self._error}

    # ---- run thread ---------------------------------------------------------------
    def _sample_window(self, secs, check_cmd=True):
        """Poll mag magnitude + yaw at ~10 Hz for `secs`, aborting early (returns
        None) if the robot is picked up, or -- unless this is the phase deliberately
        driving it (`check_cmd=False`) -- if it starts being driven. Returns
        (mag_noise, mag_mean, yaw_wobble_deg)."""
        n = self._node
        mags, yaws = [], []
        deadline = time.monotonic() + secs
        while time.monotonic() < deadline:
            with self._lock:
                if not self._active:
                    return None
            if any(n._susp_eff()):
                return None
            if check_cmd:
                lin, ang = n.telemetry._cmd_vel
                eps = n.get_parameter("vision_bumper_cmd_eps").value
                if abs(lin) > eps or abs(ang) > eps:
                    return None
            mag = n.telemetry._mag
            eul = n.telemetry._eul
            if mag is not None:
                mx, my, mz = mag
                mags.append((mx * mx + my * my + mz * mz) ** 0.5)
            if eul is not None:
                yaws.append(eul[2])
            time.sleep(0.1)
        if not mags:
            return None
        mag_noise = max(mags) - min(mags)
        mag_mean = sum(mags) / len(mags)
        if yaws:
            rel = [_wrap_deg(y - yaws[0]) for y in yaws]
            yaw_wobble = max(rel) - min(rel)
        else:
            yaw_wobble = 0.0
        return mag_noise, mag_mean, yaw_wobble

    def _finish(self, error=""):
        with self._lock:
            self._active = False
            self._phase = "done"
            self._error = error

    def _run(self, include_motor, base_secs, lds_secs, fan_secs, led_secs, motor_secs):
        n = self._node
        pubs = n._skill_pubs
        led_pub = pubs.get("/led")
        lds_pub = pubs.get("/lds_target_rpm")
        cmd_pub = pubs.get("/cmd_vel")
        # (name, on, off, duration, check_cmd) -- `off` is None where the actuator is
        # deliberately left as-is afterward (the LDS spin-down is slam_nav's own idle-
        # park logic's job, not this test's).
        phases = [
            ("baseline", None, None, base_secs, True),
            ("lds", (lambda: lds_pub.publish(Float32(data=self.lds_rpm))) if lds_pub else None,
             None, lds_secs, True),
            ("fan", lambda: n.telemetry.set_param_json(
                {"node": "sys_monitor", "name": "fan_override", "value": 1.0}),
             None, fan_secs, True),
            ("led", (lambda: led_pub.publish(Bool(data=True))) if led_pub else None,
             (lambda: led_pub.publish(Bool(data=False))) if led_pub else None, led_secs, True),
        ]
        if include_motor and cmd_pub is not None:
            def _motor_on():
                tw = Twist()
                tw.angular.z = self.motor_ang
                cmd_pub.publish(tw)

            phases.append(("motor", _motor_on, lambda: cmd_pub.publish(Twist()),
                           motor_secs, False))
        try:
            self._log(f"imu interference test: {len(phases)} phases, motor={include_motor}")
            for i, (name, on, off, secs, check_cmd) in enumerate(phases):
                with self._lock:
                    if not self._active:
                        return
                    self._phase, self._phase_i = name, i
                if on:
                    try:
                        on()
                    except Exception as exc:
                        self._log(f"imu interference test: phase {name} actuate failed: {exc}")
                sample = self._sample_window(secs, check_cmd=check_cmd)
                if off:
                    try:
                        off()
                    except Exception:
                        pass
                if sample is None:
                    self._finish("robot moved / was picked up — aborted")
                    return
                mag_noise, mag_mean, yaw_wobble = sample
                pct = (mag_noise / mag_mean * 100.0) if mag_mean > 1 else 0.0
                with self._lock:
                    self._results.append({
                        "phase": name, "mag_noise": round(mag_noise, 1),
                        "mag_noise_pct": round(pct, 1),
                        "yaw_wobble_deg": round(yaw_wobble, 2)})
            self._finish("")
        finally:
            self._safe_all(pubs)

    def _safe_all(self, pubs):
        """Best-effort return every touched actuator to its resting state."""
        try:
            self._node.telemetry.set_param_json(
                {"node": "sys_monitor", "name": "fan_override", "value": -1.0})
        except Exception:
            pass
        led_pub = pubs.get("/led")
        cmd_pub = pubs.get("/cmd_vel")
        if led_pub:
            try:
                led_pub.publish(Bool(data=False))
            except Exception:
                pass
        if cmd_pub:
            try:
                cmd_pub.publish(Twist())
            except Exception:
                pass
