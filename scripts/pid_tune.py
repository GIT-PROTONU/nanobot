#!/usr/bin/env python3
"""Wheel-PID retune harness for the closed-loop coprocessor drive (2026-09-20+).

Talks to the robot's web gateway ONLY (stdlib urllib/http.server — no ROS on the
dev PC): POST /drive arms web_control's ~3.3 Hz /cmd_vel keepalive, POST /publish
pokes the whitelisted /motor_pid + /motor_params topics, and the SSE /telemetry
stream supplies the /wheel_ticks + /wheel_pid + /wheel_params readbacks. Made for
the docs/TODO.md item "Retune the closed-loop wheel-PID in TRUE units": walk the
speed ladder 0.05 -> 0.15 m/s at fixed gains and flag stalls / sag / hunting per
rung, without ever putting more than ~2.5 Hz of POSTs (or any direct /cmd_vel) on
the wire.

    pixi run python scripts/pid_tune.py --host http://192.168.178.141:8080 ladder
    pixi run python scripts/pid_tune.py --host ... ladder --vlist 0.05,0.1 --secs 8
    pixi run python scripts/pid_tune.py --host ... ladder --spin 0.5        # in-place
    pixi run python scripts/pid_tune.py --host ... gains --set 1.1,45,0
    pixi run python scripts/pid_tune.py --host ... params --set 0=253,1=0.05
    pixi run python scripts/pid_tune.py --mock                             # offline self-test

Judgements (per rung, cruise = the last 60% of the run):
  STALL    a wheel's tick count frozen > stall-win while commanded — the 2026-09-19
           low-duty seizure signature; KI is what breaks away through stiction.
  SAG      mean wheel speed << target at the end (duty saturated / feedforward off).
  HUNT     peak-to-peak speed > hunt-frac x target (stick-slip oscillation) — KI too
           high or KP too low.
All speeds are in TRUE m/s (ticks/metre from f.esp.wheel_params ids 0+1).
"""
import argparse
import json
import math
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

POST_HZ = 2.5          # /drive refresh rate (HTTP-side; the keepalive still owns /cmd_vel)
STALL_WIN = 0.45       # s of zero tick movement while commanded = stall
HUNT_FRAC = 0.25       # peak-to-peak speed above this x target = hunting


class Telemetry:
    """Background SSE reader: keeps the latest parsed frame + a reachability flag,
    and a timestamped history of every parsed /wheel_ticks value — tick DELTAS are
    computed from consecutive parsed frames (never point-in-time reads, which alias
    when the reader batches two frames between samples)."""

    def __init__(self, base):
        self.base = base
        self.frame = {}
        self.alive = threading.Event()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self.hist = []          # [(monotonic, (l, r) ticks)] per parsed frame
        self._trim_at = 4000    # trim the history periodically (runs are short)

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
                            if line.startswith(b"data: "):
                                try:
                                    self.frame = json.loads(line[6:])
                                except ValueError:
                                    pass
                                else:
                                    tk = (self.frame.get("esp") or {}).get("ticks")
                                    if tk is not None:
                                        self.hist.append((time.monotonic(),
                                                          (tk[0], tk[1])))
                                        if len(self.hist) > self._trim_at:
                                            del self.hist[:self._trim_at // 2]
                                self.alive.set()
            except Exception:
                self.alive.clear()
                self._stop.wait(1.0)

    def esp(self):
        return self.frame.get("esp") or {}


class Gateway:
    """The write half: /drive (dead-man refresh) + whitelisted topic pokes."""

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

    def set_pid(self, kp, ki, kd):
        return self._post("/publish", {"topic": "/motor_pid", "value": [kp, ki, kd]})

    def set_params(self, pairs):
        flat = []
        for id_, val in pairs:
            flat += [int(id_), float(val)]
        return self._post("/publish", {"topic": "/motor_params", "value": flat})


def ticks_per_meter(esp):
    """TRUE scale from the /wheel_params readback (ids: 0 tpr, 1 wheel_radius_m)."""
    p = esp.get("wheel_params") or []
    d = {int(p[i]): p[i + 1] for i in range(0, len(p) - 1, 2)}
    tpr, r = d.get(0), d.get(1)
    if not tpr or not r or r <= 0:
        return None
    return tpr / (2.0 * math.pi * r)


class RunSampler:
    """Walks the reader's timestamped tick history over one rung and scores it."""

    def __init__(self, tel):
        self.tel = tel
        self.rows = []          # (dt, (dl,dr) tick deltas), dt = inter-frame gap
        self.gap = 0.0          # s since the LAST tick change (both wheels frozen)
        self._pos = 0

    def sample(self, dur):
        """Consume history for `dur` seconds (call repeatedly during the run)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            time.sleep(0.05)
        hist = self.tel.hist
        pos = self._pos
        while pos < len(hist) and hist[pos][0] < t0:
            pos += 1                           # skip entries from before this rung
        prev = hist[pos - 1] if pos > 0 else None
        while pos < len(hist):
            t, tk = hist[pos]
            if prev is not None:
                self.rows.append((t - prev[0], (tk[0] - prev[1][0],
                                                tk[1] - prev[1][1])))
            prev = (t, tk)
            pos += 1
        self._pos = pos
        self.gap = max(self.gap, time.monotonic() - prev[0] if prev else 0.0)

    def score(self, target_mps, tpm):
        """-> dict with mean speed, p2p, stall flags (ok=False if no tick data)."""
        if not self.rows or not tpm:
            return {"mean": None, "p2p": None, "stall": [False, False], "ok": False}
        # per-0.5 s bucket speed (m/s) per wheel — single-tick deltas at crawl speeds
        # quantize hard (1 tick = ~1.2 mm), which would fake "hunting"; bucketing
        # averages it out
        buckets, acc_t, acc = [], 0.0, [0, 0]
        for dt, (dl, dr) in self.rows:
            acc_t += dt
            acc[0] += dl
            acc[1] += dr
            if acc_t >= 0.5:
                buckets.append((acc_t, tuple(acc)))
                acc_t, acc = 0.0, [0, 0]
        if acc_t > 0.1:
            buckets.append((acc_t, tuple(acc)))
        speeds = [(abs(dl) / tpm / dt, abs(dr) / tpm / dt)
                  for dt, (dl, dr) in buckets if dt > 1e-4]
        cruise = speeds[int(len(speeds) * 0.4):] or speeds
        mean = sum(sum(p) / 2.0 for p in cruise) / len(cruise)
        p2p = (max(p[0] for p in cruise) - min(p[0] for p in cruise),
               max(p[1] for p in cruise) - min(p[1] for p in cruise))
        # stall: a wheel with zero movement across the tail while the other moved,
        # OR both frozen so long that no tick messages arrived at all
        tail = self.rows[int(len(self.rows) * 0.6):]
        moved = (sum(dl for _t, (dl, _dr) in tail), sum(dr for _t, (_dl, dr) in tail))
        both_frozen = self.gap > STALL_WIN
        stall = [both_frozen or abs(m) < 1e-3 for m in moved] if tail \
            else [both_frozen, both_frozen]
        return {"mean": mean, "p2p": p2p, "stall": stall, "ok": True}


def run_ladder(gw, tel, speeds, secs, reverse=False, spin=None):
    e = tel.esp()
    tpm = ticks_per_meter(e)
    if tpm is None:
        print("  !! /wheel_params readback missing (is the coprocessor up?) — "
              "speeds will read in ticks/s only")
    p = e.get("wheel_params") or []
    pd = {int(p[i]): p[i + 1] for i in range(0, len(p) - 1, 2)}
    sep = pd.get(2, 0.125)
    for v in speeds:
        if spin:
            cmd = (0.0, spin if not reverse else -spin)
            label = f"spin {spin:+.2f} rad/s (wheels ±{abs(spin) * sep / 2:.3f} m/s)"
            tgt = abs(spin) * sep / 2.0
        else:
            cmd = ((-v if reverse else v), 0.0)
            label = f"{v:+.3f} m/s{' (rev)' if reverse else ''}"
            tgt = abs(v)
        print(f"-- rung {label} for {secs:.0f} s ...")
        s = RunSampler(tel)
        t_end = time.monotonic() + secs
        try:
            gw.drive(*cmd)
            while time.monotonic() < t_end:
                gw.drive(*cmd)                    # refresh the dead-man (~2.5 Hz)
                s.sample(min(0.4, max(0.05, t_end - time.monotonic())))
        finally:
            gw.drive(0.0, 0.0)
        r = s.score(tgt, tpm)
        if not r["ok"]:
            print("   ?? no tick data — telemetry/ESP link down?")
            continue
        stall = "".join(("L" if r["stall"][0] else "")
                        + ("R" if r["stall"][1] else "")) or "-"
        mean = r["mean"] if r["mean"] is not None else float("nan")
        p2p = max(r["p2p"]) if r["p2p"] else 0.0
        flags = []
        if stall != "-":
            flags.append(f"STALL({stall})")
        if mean < tgt * 0.75:
            flags.append("SAG")
        if p2p > HUNT_FRAC * max(tgt, 1e-6):
            flags.append("HUNT")
        print(f"   mean {mean:+.3f} m/s (target {tgt:+.3f}), p2p {p2p:.3f}, "
              f"stall {stall} -> {'OK' if not flags else ' '.join(flags)}")


def show_state(gw, tel):
    e = tel.esp()
    print("wheel_pid   :", e.get("wheel_pid"))
    print("wheel_params:", e.get("wheel_params"))
    print("wheel_trim  :", e.get("wheel_trim"))
    print("ticks       :", e.get("ticks"), "hz", e.get("tick_hz"), "stray", e.get("stray"))


# ---- offline mock gateway (--mock): a tiny fake of the three endpoints -----------
class _Mock(BaseHTTPRequestHandler):
    state = {"ticks": [0, 0], "pid": [1.1, 45.0, 0.0],
             "params": [0, 253.0, 1, 0.05, 2, 0.125, 3, 0.4, 4, 0.8, 5, 2.0]}
    t0 = time.monotonic()

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n) or b"{}")
        if self.path == "/drive":
            # fake physics: cruise at the commanded speed (perfect PID)
            _Mock.state["v"] = body.get("v", 0.0)
            _Mock.state["w"] = body.get("w", 0.0)
            return self._json({"status": "ok"})
        if self.path == "/publish":
            if body.get("topic") == "/motor_pid":
                _Mock.state["pid"] = [round(float(x), 3) for x in body["value"]]
            elif body.get("topic") == "/motor_params":
                p = body["value"]
                for i in range(0, len(p) - 1, 2):
                    for j in range(0, len(_Mock.state["params"]), 2):
                        if _Mock.state["params"][j] == p[i]:
                            _Mock.state["params"][j + 1] = p[i + 1]
            return self._json({"status": "ok"})
        self._json({"error": "nope"}, 400)

    def do_GET(self):
        if self.path.startswith("/telemetry"):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.end_headers()
            last = time.monotonic()
            try:
                while True:
                    time.sleep(0.2)
                    now = time.monotonic()
                    dtf, last = now - last, now
                    v, w = _Mock.state.get("v", 0.0), _Mock.state.get("w", 0.0)
                    # ticks advance INCREMENTALLY at wheel speed x ticks/metre
                    # (perfect PID; w splits into per-wheel ±w*sep/2) — cumulative
                    # formulas jump when the commanded speed changes
                    tpm = _Mock.state["params"][1] / (2 * math.pi * _Mock.state["params"][3])
                    sep = _Mock.state["params"][5]          # id 2 = wheel_separation_m
                    t = _Mock.state["ticks"]
                    if not _Mock.state.get("freeze"):       # stall-scenario knob
                        _Mock.state["ticks"] = [
                            t[0] + int(round((v - w * sep / 2) * tpm * dtf)),
                            t[1] + int(round((v + w * sep / 2) * tpm * dtf))]
                    self.wfile.write(b"data: " + json.dumps(
                        {"esp": {"ticks": _Mock.state["ticks"], "tick_hz": 15.0,
                                 "wheel_pid": _Mock.state["pid"],
                                 "wheel_params": _Mock.state["params"],
                                 "hb": 1, "hb_age": 0.1, "stray": [0, 0],
                                 "wheel_trim": 0.0}}).encode() + b"\n\n")
                    self.wfile.flush()
                    time.sleep(0.2)
            except Exception:
                pass
        else:
            self._json({})


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="http://192.168.178.141:8080")
    ap.add_argument("--mock", action="store_true", help="serve a fake gateway on :8099")
    sub = ap.add_subparsers(dest="cmd")
    lad = sub.add_parser("ladder", help="speed ladder runs")
    lad.add_argument("--vlist", default="0.05,0.08,0.10,0.15")
    lad.add_argument("--secs", type=float, default=6.0)
    lad.add_argument("--reverse", action="store_true")
    lad.add_argument("--spin", type=float, default=None, metavar="RAD_S")
    lad.add_argument("--repeat", type=int, default=1, help="runs per rung (flakiness)")
    sub.add_parser("state", help="print wheel_pid/wheel_params/ticks readback")
    g = sub.add_parser("gains", help="set/show live PID gains")
    g.add_argument("--set", default=None, metavar="KP,KI,KD")
    p = sub.add_parser("params", help="set/show drivetrain params")
    p.add_argument("--set", default=None, metavar="ID=VAL,...")
    args = ap.parse_args()

    if args.mock:
        srv = ThreadingHTTPServer(("127.0.0.1", 8099), _Mock)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        args.host = "http://127.0.0.1:8099"
        print("[mock gateway on :8099]")

    gw = Gateway(args.host)
    tel = Telemetry(args.host)
    tel.start()
    if not tel.alive.wait(5.0):
        print("!! no /telemetry stream — is the gateway up? (continuing anyway)")
    if args.cmd in (None, "state"):
        time.sleep(1.5 if not args.mock else 0.7)
        show_state(gw, tel)
        return 0
    if args.cmd == "gains":
        if args.set:
            kp, ki, kd = (float(x) for x in args.set.split(","))
            print("set_pid ->", gw.set_pid(kp, ki, kd))
        time.sleep(1.0)
        show_state(gw, tel)
        return 0
    if args.cmd == "params":
        if args.set:
            pairs = [tuple(s.split("=")) for s in args.set.split(",")]
            print("set_params ->", gw.set_params(pairs))
        time.sleep(1.0)
        show_state(gw, tel)
        return 0
    if args.cmd == "ladder":
        try:
            speeds = [float(x) for x in args.vlist.split(",")]
            for rep in range(args.repeat):
                if rep:
                    print(f"-- repeat {rep + 1}/{args.repeat}")
                run_ladder(gw, tel, speeds, args.secs,
                           reverse=args.reverse, spin=args.spin)
        except KeyboardInterrupt:
            print("\n[interrupted] stopping")
        finally:
            try:
                gw.drive(0.0, 0.0)                 # ALWAYS land stopped
            except Exception:
                pass
        return 0
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
