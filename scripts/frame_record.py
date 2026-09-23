#!/usr/bin/env python3
"""Passive telemetry-frame recorder (ROS-free, stdlib-only).

Connects to the web gateway's SSE /telemetry stream and logs a compact JSON
line per frame (or whole frames with --all). Built for the 2026-09-22 open
residual in docs/TODO.md — the unexplained spin-leg episode (~12:57) where
legs read a sustained ~10x tick advance with correct gains: run this alongside
`pid_tune.py outback` and, if the phantom returns, the recording pinpoints
whether the frames themselves carried the bad ticks (plant/ESP side) or the
ticks were fine and the reader batched (delivery side).

PASSIVE: only GET /telemetry, no POSTs — safe to run under any tuning session.

Modes:
  record (default)  --secs N (0 = until Ctrl-C) -> --out FILE (jsonl)
  --report FILE     analyze a recorded file (no network), same summary
  --selftest        offline check of the summarizer on synthetic data

The summary scores per-wheel speed from BUILD stamps (frame "t", floored at
BURST_MIN_DT like pid_tune.py) — the burst-artifact gotcha — and reports the
top speed spikes plus the parse-gap/burst signature of a POST-stall.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
import urllib.request

BURST_MIN_DT = 0.15    # s floor on inter-frame dt (frames are built at 5 Hz = 0.2 s)
SPIKE_V = 0.5          # m/s per wheel: report any interval above this
DEFAULT_TPM = 1202.0   # ticks/m fallback (measured rollout 2026-09-20); the live
                       # value rides f.esp.wheel_params and wins when present


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


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


class SseRecorder:
    """Background SSE reader: writes one JSON line per parsed frame via the
    single-argument `sink` (called from the reader thread only)."""

    def __init__(self, base, sink):
        self.base = base
        self._sink = sink
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self.count = 0

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
                            if not line.startswith(b"data: "):
                                continue
                            try:
                                frame = json.loads(line[6:])
                            except ValueError:
                                continue
                            row = dict(frame)
                            row["rt"] = round(time.monotonic(), 3)
                            self._sink(row)
                            self.count += 1
            except Exception:
                self._stop.wait(0.5)


def compact_row(frame, pt):
    """The default logged shape — light, lossless for the tick-rate question."""
    esp = frame.get("esp") or {}
    return {"rt": round(pt, 3), "t": frame.get("t"),
            "ticks": esp.get("ticks"), "hb_age": esp.get("hb_age"),
            "tick_hz": esp.get("tick_hz"),
            "move": bool((frame.get("move") or {}).get("active", False)),
            "lds_tgt": (frame.get("lds") or {}).get("tgt")}


def norm_row(r):
    """--all recordings are whole frames; --report must handle both shapes."""
    if "esp" in r:
        return compact_row(r, r.get("rt", 0.0))
    return r


def ticks_per_meter(row):
    """TRUE scale from a recorded row's /wheel_params readback (ids 0 tpr, 1 radius)."""
    p = (row.get("esp") or {}).get("wheel_params") or []
    d = {int(p[i]): p[i + 1] for i in range(0, len(p) - 1, 2)} if p else {}
    tpr, r = d.get(0), d.get(1)
    if not tpr or not r or r <= 0:
        return None
    return tpr / (2.0 * math.pi * r)


def summarize(rows, tpm):
    """Score a recorded session. rows: compact or whole frames, arrival order.
    Speeds are scored against BUILD stamps (dt floored at BURST_MIN_DT) —
    after a POST stall the backlogged frames burst in and parse-time dt
    collapses, inflating speeds (the 2026-09-22 artifact pid_tune hit)."""
    if len(rows) < 2:
        return {"frames": len(rows), "error": "not enough frames"}

    max_parse_gap = max(b["rt"] - a["rt"] for a, b in zip(rows, rows[1:]))
    bursts = sum(1 for a, b in zip(rows, rows[1:]) if b["rt"] - a["rt"] < BURST_MIN_DT)

    gaps = [b["t"] - a["t"] for a, b in zip(rows, rows[1:])
            if a.get("t") is not None and b.get("t") is not None]
    max_build_gap = max(gaps) if gaps else None
    stalled = sum(1 for g in gaps if g > 1.0)

    spikes, max_speed = [], 0.0
    for a, b in zip(rows, rows[1:]):
        ta, tb, ka, kb = a.get("t"), b.get("t"), a.get("ticks"), b.get("ticks")
        if ta is None or tb is None or not ka or not kb or tb - ta <= 0:
            continue
        dt = max(tb - ta, BURST_MIN_DT)
        dl = (kb[0] - ka[0]) / dt
        dr = (kb[1] - ka[1]) / dt
        v = max(abs(dl), abs(dr)) / tpm if tpm else 0.0
        max_speed = max(max_speed, v)
        if v > SPIKE_V:
            spikes.append({"t": tb, "dt": round(dt, 3),
                           "ticks_per_s": [round(dl, 1), round(dr, 1)],
                           "v_ms": round(v, 3)})
    spikes.sort(key=lambda s: -s["v_ms"])

    out = {"frames": len(rows),
           "tpm": round(tpm, 1) if tpm else None,
           "max_parse_gap_s": round(max_parse_gap, 3),
           "burst_frames": bursts,
           "max_build_gap_s": round(max_build_gap, 3) if max_build_gap is not None else None,
           "build_gaps_over_1s": stalled,
           "max_wheel_speed_ms": round(max_speed, 3),
           "speed_spikes_over_v": spikes[:8]}
    if rows[0].get("t") is not None and rows[-1].get("t") is not None:
        out["span_s"] = round(rows[-1]["t"] - rows[0]["t"], 1)
    return out


def selftest():
    """Offline check of the summarizer on synthetic data."""
    t0 = 1000.0
    rows = []
    # 20 calm frames: 0 ticks/s, 0.2 s build gaps
    for i in range(20):
        rows.append({"rt": t0 + i * 0.2, "t": t0 + i * 0.2, "ticks": [i, i]})
    # a 10x-style spike: +121 ticks on L in one 0.2 s frame (~0.5 m/s equivalent)
    rows.append({"rt": t0 + 4.0, "t": t0 + 4.0, "ticks": [140, 20]})
    for i in range(1, 6):
        rows.append({"rt": t0 + 4.0 + i * 0.2, "t": t0 + 4.0 + i * 0.2,
                     "ticks": [120, 20 + i]})
    # a POST-stall burst: three frames arriving back-to-back (parse gap ~0)
    for i in range(3):
        rows.append({"rt": t0 + 5.2, "t": t0 + 5.0 + i * 0.2, "ticks": [120, 25 + i]})
    out = summarize(rows, DEFAULT_TPM)
    assert out["frames"] == 29, out
    assert out["burst_frames"] >= 2, out
    assert out["max_build_gap_s"] is not None and out["max_build_gap_s"] < 0.5, out
    assert out["speed_spikes_over_v"], out
    assert out["speed_spikes_over_v"][0]["v_ms"] > 0.5, out
    print(json.dumps(out, indent=2))
    print("selftest OK")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None,
                    help="web gateway port (default: robot.yaml web_port)")
    ap.add_argument("--secs", type=float, default=0.0,
                    help="record for N seconds (0 = until Ctrl-C)")
    ap.add_argument("--out", default=None, help="output .jsonl (default: auto name)")
    ap.add_argument("--all", action="store_true",
                    help="log whole frames instead of the compact row")
    ap.add_argument("--report", metavar="FILE",
                    help="analyze a recorded file instead of recording")
    ap.add_argument("--selftest", action="store_true",
                    help="offline check of the summarizer, no network")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if args.report:
        rows = [norm_row(json.loads(line)) for line in open(args.report) if line.strip()]
        tpm = next((ticks_per_meter(r) for r in rows if ticks_per_meter(r)), DEFAULT_TPM)
        print(json.dumps(summarize(rows, tpm), indent=2))
        return 0

    args.port = args.port or read_web_port()
    base = f"http://{args.host}:{args.port}"
    out = args.out or f"frame_record_{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    n0 = [0]

    with open(out, "w") as fh:
        def sink(row):
            fh.write(json.dumps(row) + "\n")
            if reader.count % 25 == 0:
                fh.flush()

        reader = SseRecorder(base, sink)
        reader.start()
        log(f"recording {base}/telemetry -> {out} "
            f"({'until Ctrl-C' if not args.secs else f'for {args.secs:.0f} s'})")
        try:
            t0 = time.monotonic()
            while not args.secs or time.monotonic() - t0 < args.secs:
                time.sleep(0.25)
        except KeyboardInterrupt:
            pass
        finally:
            reader.stop()
            fh.flush()
    log(f"{reader.count} frames -> {out}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nstopped")
        sys.exit(0)
