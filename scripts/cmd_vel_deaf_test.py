#!/usr/bin/env python3
"""Controlled /cmd_vel-deaf reproduction + heal test (docs/TODO.md open bug).

The bug: after a router/stack restart the ESP32's zenoh session re-attaches
(hb/ticks/LDS flow, /motor_pid write->/wheel_pid readback flips) but /cmd_vel
SPECIFICALLY goes deaf — motion is the only observable, so this test drives a
short, slow pulse through the web gateway and watches /wheel_ticks.

Per iteration:
  1. (--restart-router) `sudo -n systemctl restart nano-router.service` (scoped
     sudoers rule) + wait for :7447 to accept and the gateway's SSE to return;
     --restart-target instead bounces the WHOLE nano-robot.target (the
     production deploy.sh/stack.sh trigger — the deaf bug was observed right
     after full deploys, ESP watchdog reboot + fresh publishers + router all
     at once).
  2. Wait for the ESP32 link (f.esp.hb_age < 3 s).
  3. MOTION-FREE RX probe: poke /motor_params [6, <dither flip>] (a physical
     no-op while parked — dither only applies when commanded) and watch the
     /wheel_params id-6 readback flip, then restore. General SBC->ESP delivery.
  4. CMD_VEL probe: POST /drive {v,0} re-asserted at ~3 Hz (the serial budget:
     scripted teleop stays <=3-4 Hz) for --pulse-secs, then {0,0}. Ticks
     advanced => delivered; ticks flat => DEAF.
  5. If deaf and healing allowed: `sudo -n systemctl restart nano-robot.target`
     (pings stop -> the ESP's LINK_RX watchdog esp_restart()s -> fresh session
     both ends — the verified heal), wait, re-probe.

PASSIVE apart from the two probes (the drive pulse IS the test). Exit codes:
0 = cmd_vel alive; 1 = deaf, healed by the bounce; 2 = deaf, still deaf;
3 = gateway/environment error. `--repeat N` walks N router restarts to
characterize the intermittency.
"""
import argparse
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.request

PULSE_HZ = 3.0          # /drive refresh while pulsing (serial budget: <=3-4 Hz)
ESP_WAIT_S = 90         # max wait for f.esp.hb_age fresh after a router restart:
                        # RX-watchdog detects the dead transport at 8 s -> ESP self-reboot
                        # -> up to the 40 s connect deadline (45 s was hit live 2026-09-23)
RX_WAIT_S = 12          # max wait for the /wheel_params readback flip (1 Hz topic)
TICKS_MOVED = 3         # cumulative ticks on either wheel = "delivered" (>=2.5 mm)
PULSE_COAST_S = 0.8     # keep sampling after the stop (coast-down ticks = delivery too)


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class Telemetry:
    """Background SSE reader (same shape as pid_tune.py's): latest parsed frame +
    a timestamped tick history — deltas come from consecutive parsed frames, never
    point-in-time reads (which alias when frames batch between samples)."""

    def __init__(self, base):
        self.base = base
        self.frame = {}
        self.alive = threading.Event()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self.hist = []          # [(parse_monotonic, build_t|None, (l, r) ticks)]

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            try:
                req = urllib.request.Request(self.base + "/telemetry")
                with urllib.request.urlopen(req, timeout=10) as r:
                    buf = b""
                    while not self._stop.is_set():
                        chunk = r.read1(4096) if hasattr(r, "read1") else r.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        while b"\n\n" in buf:
                            line, buf = buf.split(b"\n\n", 1)
                            line = line.strip()
                            if line.startswith(b"data: ") and line != b": ping":
                                try:
                                    self.frame = json.loads(line[6:])
                                except ValueError:
                                    pass
                                else:
                                    tk = (self.frame.get("esp") or {}).get("ticks")
                                    if tk is not None:
                                        self.hist.append((time.monotonic(),
                                                          self.frame.get("t"),
                                                          (tk[0], tk[1])))
                                        if len(self.hist) > 4000:
                                            del self.hist[:2000]
                                    self.alive.set()
            except Exception:
                self.alive.clear()
                self._stop.wait(0.5)

    def esp(self):
        return self.frame.get("esp") or {}

    def nav_status(self):
        return (self.frame.get("nav") or {}).get("status") or "idle"


class Gateway:
    """The write half: /drive pokes + whitelisted topic publishes."""

    def __init__(self, base):
        self.base = base

    def _post(self, path, body, timeout=5):
        req = urllib.request.Request(
            self.base + path, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode() or "{}")

    def drive(self, v, w=0.0):
        return self._post("/drive", {"v": v, "w": w})

    def set_params(self, pairs):
        flat = []
        for id_, val in pairs:
            flat += [int(id_), float(val)]
        return self._post("/publish", {"topic": "/motor_params", "value": flat})


def read_web_port():
    """web_port from robot.yaml (board install path first, then the src tree)."""
    candidates = [
        os.path.expanduser("~/Nano/install/robot_bringup/share/robot_bringup/config/robot.yaml"),
        os.path.expanduser("~/Nano/src/robot_bringup/config/robot.yaml"),
    ]
    for path in candidates:
        try:
            txt = open(path).read()
        except OSError:
            continue
        for line in txt.splitlines():
            s = line.strip()
            if s.startswith("web_port:"):
                try:
                    return int(s.split(":", 1)[1].strip().split("#")[0])
                except ValueError:
                    pass
    return 8080


def wait_gateway(tel, timeout):
    """The SSE reader reconnects on its own; wait until a frame arrives."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if tel.alive.is_set():
            return True
        time.sleep(0.5)
    return False


def wait_esp(tel, timeout):
    """ESP32 heartbeat fresh: hb present and hb_age < 3 s."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        esp = tel.esp()
        age = esp.get("hb_age")
        if esp.get("hb") is not None and age is not None and age < 3.0:
            return True
        time.sleep(0.5)
    return False


def wait_tcp_port(port, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1.0):
                return True
        except OSError:
            time.sleep(0.5)
    return False


def sudo_restart(unit):
    log(f"sudo -n systemctl restart {unit} ...")
    try:
        r = subprocess.run(["sudo", "-n", "/usr/bin/systemctl", "restart", unit],
                           capture_output=True, text=True, timeout=240)
    except subprocess.TimeoutExpired:
        # a target bounce can legitimately take ~3 min (nav's start timeout);
        # the restart keeps running server-side — treat as started-and-waiting
        log("  sudo still running after 240 s — continuing to wait on the probes")
        return True
    if r.returncode != 0:
        log(f"  sudo FAILED rc={r.returncode}: {r.stderr.strip()[:200]}")
        log("  (the scoped sudoers rule for this exact command is missing —"
            " see deploy/sudoers/nano-power)")
    return r.returncode == 0


def wheel_param(esp, pid_):
    p = esp.get("wheel_params") or []
    for i in range(0, len(p) - 1, 2):
        if int(p[i]) == pid_:
            return float(p[i + 1])
    return None


def rx_probe(tel, gw):
    """Motion-free SBC->ESP delivery check: flip the dither param (id 6) and
    watch the /wheel_params readback follow, then restore. While parked the
    dither is never applied (zero when parked / cmd-stale), so this moves
    nothing — it is exactly the poke that verified the 2026-09-22 heal."""
    v0 = wheel_param(tel.esp(), 6)
    if v0 is None:
        log("  rx-probe: /wheel_params readback absent (ESP link or params pub down)")
        return False
    tgt = 0.0 if v0 else 0.01           # flip whichever way, then restore
    log(f"  rx-probe: /motor_params [6, {tgt}] (was {v0}), waiting for readback flip")

    def flip_to(value):
        try:
            gw.set_params([(6, value)])
        except Exception as exc:
            log(f"  rx-probe: POST failed: {exc}")
            return False
        deadline = time.monotonic() + RX_WAIT_S
        while time.monotonic() < deadline:
            v = wheel_param(tel.esp(), 6)
            if v is not None and math.isclose(v, value, abs_tol=1e-6):
                return True
            time.sleep(0.5)
        return False

    ok = flip_to(tgt)
    if ok:
        flip_to(v0)                      # restore (best effort)
        log("  rx-probe: readback flipped + restored — SBC->ESP RX OK")
    else:
        log("  rx-probe: readback did NOT flip — SBC->ESP RX DEAF (worse than cmd_vel-only)")
    return ok


def drive_pulse_probe(tel, gw, speed, pulse_s):
    """THE test: a slow /drive pulse, scored by tick movement. Returns
    (delivered, dl, dr). Wheels must be FREE TO MOVE — a physically blocked
    robot also reads zero ticks (motion is the only observable)."""
    # Index into hist at pulse start; deltas measured over the pulse window.
    start_n = len(tel.hist)
    deadline = time.monotonic() + pulse_s
    log(f"  drive-pulse: v={speed} m/s for {pulse_s:.1f} s (~{PULSE_HZ} Hz POSTs)")
    while time.monotonic() < deadline:
        try:
            gw.drive(speed, 0.0)
        except Exception as exc:
            log(f"  drive-pulse: POST failed: {exc}")
            return None, 0, 0
        time.sleep(1.0 / PULSE_HZ)
    try:
        gw.drive(0.0, 0.0)               # explicit stop; keepalive brakes 1 s
    except Exception:
        pass
    time.sleep(PULSE_COAST_S)            # let coast-down ticks land

    window = tel.hist[start_n:]
    if not window:
        log("  drive-pulse: no telemetry frames during the pulse — gateway dead?")
        return None, 0, 0
    l0, r0 = window[0][2]
    l1, r1 = window[-1][2]
    dl, dr = l1 - l0, r1 - r0
    delivered = max(abs(dl), abs(dr)) >= TICKS_MOVED
    return delivered, dl, dr


def heal_and_wait(tel):
    """The verified heal: bounce the whole target — pings stop, the ESP's
    LINK_RX_TIMEOUT_MS (8 s) watchdog esp_restart()s, fresh handshake both
    ends. Allow ~90 s (nav's stop alone takes ~90 s; the ESP reboot adds 8+)."""
    log("  heal: sudo -n systemctl restart nano-robot.target")
    if not sudo_restart("nano-robot.target"):
        return False
    tel.alive.clear()
    ok = wait_gateway(tel, 120) and wait_esp(tel, ESP_WAIT_S + 45)
    if ok:
        time.sleep(3)                    # settle: readbacks refill after the ESP reboot
    return ok


def check_env(args, tel):
    """Refuse to run into a moving robot or a dead link."""
    if not wait_gateway(tel, 10):
        log("gateway /telemetry unreachable — is nano-app up? (web_port "
            f"{args.port} on {args.host})")
        return False
    st = tel.nav_status()
    if st in ("planning", "navigating", "canceling"):
        log(f"Nav2 is {st} — cancel the goal first (POST /nav/cancel); refusing to pulse")
        return False
    if not wait_esp(tel, ESP_WAIT_S):
        log("ESP32 heartbeat not fresh (hb_age None or >3 s) — link down; "
            "fix the coprocessor link before testing")
        return False
    return True


def run_once(args, tel, gw):
    """One test iteration. Returns 'alive' | 'deaf' | 'error'."""
    if args.restart_target:
        # The production trigger: deploy.sh/stack.sh end with a FULL target
        # bounce (router + ESP watchdog reboot + fresh publishers) — and the
        # deafness was observed twice right after exactly that.
        log("restart-target: sudo -n systemctl restart nano-robot.target")
        if not sudo_restart("nano-robot.target"):
            return "error"
        tel.alive.clear()
        if not wait_gateway(tel, 180):
            log("  gateway SSE did not return after the target bounce")
            return "error"
        if not wait_esp(tel, ESP_WAIT_S + 45):
            log("  ESP32 heartbeat never went fresh after the target bounce")
            return "error"
    elif args.restart_router:
        log("restart-router: sudo -n systemctl restart nano-router.service")
        if not sudo_restart("nano-router.service"):
            return "error"
        if not wait_tcp_port(7447, 45):
            log("  router :7447 did not accept within 45 s")
            return "error"
        tel.alive.clear()
        if not wait_gateway(tel, 60):
            log("  gateway SSE did not return after the router restart")
            return "error"
        if not wait_esp(tel, ESP_WAIT_S):
            log("  ESP32 heartbeat never went fresh after the router restart")
            return "error"

    rx_ok = rx_probe(tel, gw)
    delivered, dl, dr = drive_pulse_probe(tel, gw, args.speed, args.pulse_secs)
    if delivered is None:
        return "error"              # no frames during the pulse — gateway, not wheels
    if delivered:
        log(f"  RESULT: /cmd_vel DELIVERED (tick delta L{dl:+d} R{dr:+d}, rx-probe "
            f"{'ok' if rx_ok else 'DEAF'})")
        return "alive"
    log(f"  RESULT: /cmd_vel DEAF (tick delta L{dl:+d} R{dr:+d} during the pulse, "
        f"rx-probe {'ok' if rx_ok else 'DEAF'})")
    if not rx_ok:
        log("  NOTE: even the motion-free poke is deaf — not cmd_vel-specific; "
            "the whole SBC->ESP direction is down")
    return "deaf"


def selftest():
    """Offline check of the drive-pulse verdict logic (no network, no sudo).
    The fake gateway appends frames the way the real background SSE reader
    does — concurrently with the probe's POST loop."""
    class FakeGw:
        """drive() appends `step` ticks per call (0 = deaf), or nothing (dead)."""
        def __init__(self, tel, step):
            self.tel, self.step, self.n = tel, step, 0

        def drive(self, v, w=0.0):
            if self.step is not None:
                self.n += 1
                now = time.monotonic()
                self.tel.hist.append((now, now, (self.n * self.step, self.n * self.step)))
            return {}

    def fake_tel():
        tel = Telemetry.__new__(Telemetry)
        tel.frame, tel.alive, tel._stop, tel.base = {}, threading.Event(), \
            threading.Event(), ""
        tel.hist = [(time.monotonic(), time.monotonic(), (0, 0))]
        return tel

    tel = fake_tel()
    ok, dl, dr = drive_pulse_probe(tel, FakeGw(tel, 10), 0.05, 0.1)
    assert ok is True and dl > 0 and dr > 0, (ok, dl, dr)
    tel = fake_tel()
    ok, dl, dr = drive_pulse_probe(tel, FakeGw(tel, 0), 0.05, 0.1)
    assert ok is False and dl == 0 and dr == 0, (ok, dl, dr)
    tel = fake_tel()
    ok, dl, dr = drive_pulse_probe(tel, FakeGw(tel, None), 0.05, 0.1)
    assert ok is None and dl == 0 and dr == 0, (ok, dl, dr)
    # wheel_param: flat (id, value) pairs.
    assert wheel_param({"wheel_params": [0, 253.0, 6, 0.05]}, 6) == 0.05
    assert wheel_param({"wheel_params": []}, 6) is None
    print("selftest OK")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--selftest", action="store_true",
                    help="offline check of the verdict logic, no network/sudo")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None,
                    help="web gateway port (default: robot.yaml web_port)")
    ap.add_argument("--restart-router", action="store_true",
                    help="restart ONLY nano-router.service first (the repro trigger)")
    ap.add_argument("--restart-target", action="store_true",
                    help="restart the FULL nano-robot.target first (the production"
                         " deploy.sh/stack.sh trigger; ~3-4 min per iteration)")
    ap.add_argument("--repeat", type=int, default=1, metavar="N",
                    help="iterations (each --restart-router run restarts again)")
    ap.add_argument("--speed", type=float, default=0.05, help="pulse speed m/s")
    ap.add_argument("--pulse-secs", type=float, default=1.5)
    ap.add_argument("--no-heal", action="store_true",
                    help="report deaf without bouncing the target")
    ap.add_argument("--yes", action="store_true", help="skip the safety countdown")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    args.port = args.port or read_web_port()
    args.base = f"http://{args.host}:{args.port}"

    tel = Telemetry(args.base)
    gw = Gateway(args.base)
    tel.start()

    if not check_env(args, tel):
        return 3

    dist = args.speed * args.pulse_secs
    print(f"\nThis test DRIVES the robot: ~{dist:.2f} m at {args.speed} m/s"
          f" (x{args.repeat} iteration(s)).\nWheels must be FREE TO MOVE and there"
          " must be ~1 m of clear space ahead.\n", flush=True)
    if not args.yes:
        try:
            for i in range(5, 0, -1):
                print(f"  starting in {i}... (Ctrl-C to abort)", flush=True)
                time.sleep(1)
        except KeyboardInterrupt:
            print("\naborted")
            return 3

    verdicts = []
    for it in range(1, args.repeat + 1):
        log(f"--- iteration {it}/{args.repeat} ---")
        v = run_once(args, tel, gw)
        if v == "error":
            verdicts.append("error")
            continue
        if v == "deaf" and not args.no_heal:
            log("  deaf confirmed — healing with the verified target bounce")
            if heal_and_wait(tel):
                delivered, dl, dr = drive_pulse_probe(tel, gw, args.speed,
                                                      args.pulse_secs)
                v = ("error" if delivered is None
                     else "alive" if delivered else "deaf-still")
                log(f"  post-heal re-probe: tick delta L{dl:+d} R{dr:+d} -> {v}")
            else:
                v = "deaf-still"
        verdicts.append(v)
        if it < args.repeat:
            time.sleep(5)

    deaf = verdicts.count("deaf") + verdicts.count("deaf-still")
    errors = verdicts.count("error")
    log(f"VERDICT: {verdicts.count('alive')} alive / {deaf} deaf / {errors} errors "
        f"over {args.repeat} iteration(s)")
    if deaf == 0:
        if errors:
            return 3               # nothing actually tested — environment failures
        log("cmd_vel delivery OK across all iterations — bug did not reproduce"
            " (it is intermittent; more --repeat runs sharpen the estimate)")
        return 0
    if any(v == "deaf-still" for v in verdicts):
        return 2
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\naborted")
        sys.exit(3)
