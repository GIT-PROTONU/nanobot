"""Stress-test mode (ROS-free): deliberately load the SBC's CPU to validate the
hardening tier (systemd watchdogs, the fan curve) under real load, WITHOUT starving the
web server that has to keep answering the browser during the test.

Workers are separate, NICED (19, the lowest scheduling priority) subprocesses each
running a tight busy loop. They aren't pinned away from any core — on an idle board they
peg every core to 100% (a genuine max), but the kernel's CFS scheduler always prefers a
normal-priority process (the web server thread, the other ROS hubs) the instant it has
work, so HTTP requests keep getting serviced. This is the same trick as
`nice -19 stress --cpu N` — no core reservation needed, and no memory is touched at all
(so there's no risk of tripping app_hub's systemd MemoryMax and getting the web server's
own unit OOM-killed).

A background watchdog auto-stops the run at its (clamped) duration regardless of the
caller — a forgotten test can't cook the board — and can also abort early on an
injected temperature reading past a threshold.
"""
import os
import subprocess
import sys
import threading
import time

NICE = 19                        # lowest scheduling priority: yields to everything else
DEFAULT_DURATION = 30.0          # s
DEFAULT_MAX_DURATION = 300.0     # s hard cap regardless of what's requested

# `python -c CODE duration` — sys.argv[1] is the duration in seconds.
_CPU_BUSY = (
    "import time,sys\n"
    "t=time.monotonic()+float(sys.argv[1])\n"
    "x=1.0\n"
    "while time.monotonic()<t:\n"
    "    x=x*1.0000001+1.0\n"
    "    if x>1e18: x=1.0\n"
)


def _nice_preexec():
    try:
        os.nice(NICE)
    except OSError:
        pass


class StressTest:
    """One CPU stress run at a time. Pure subprocess management (no ROS), so it's
    unit-testable offline and shared verbatim between the robot node and the dev
    harness."""

    def __init__(self, logger=None, max_duration=DEFAULT_MAX_DURATION,
                 abort_temp_c=0.0, read_temp=None):
        self._log = logger or (lambda *a, **k: None)
        self.max_duration = float(max_duration)
        self.abort_temp_c = float(abort_temp_c)     # <=0 disables the thermal auto-abort
        self._read_temp = read_temp
        self._lock = threading.RLock()
        self._procs = []                # [subprocess.Popen, ...]
        self._started_at = 0.0
        self._duration = 0.0
        self._cpu_workers = 0
        self._active = False

    def start(self, duration=DEFAULT_DURATION, workers=0):
        with self._lock:
            if self._active:
                return {"error": "stress test already running"}
            duration = max(1.0, min(float(duration), self.max_duration))
            ncpu = os.cpu_count() or 1
            workers = int(workers) if workers else ncpu
            workers = max(1, min(workers, ncpu))
            procs = []
            try:
                for _ in range(workers):
                    procs.append(subprocess.Popen(
                        [sys.executable, "-c", _CPU_BUSY, str(duration)],
                        preexec_fn=_nice_preexec, stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            except OSError as exc:
                for p in procs:
                    try:
                        p.kill()
                    except OSError:
                        pass
                return {"error": f"could not start workers: {exc}"}
            self._procs = procs
            self._started_at = time.monotonic()
            self._duration = duration
            self._cpu_workers = workers
            self._active = True
            self._log(f"stress: started cpu_workers={workers} duration={duration:.0f}s")
            threading.Thread(target=self._run_watchdog, args=(duration,), daemon=True).start()
            return self.status()

    def _run_watchdog(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            with self._lock:
                if not self._active:
                    return
            if self.abort_temp_c > 0 and self._read_temp:
                try:
                    t = self._read_temp()
                except Exception:
                    t = float("nan")
                if t == t and t >= self.abort_temp_c:
                    self._log(f"stress: aborting early — temp {t:.1f}C >= {self.abort_temp_c:.1f}C")
                    self.stop()
                    return
            time.sleep(1.0)
        self.stop()

    def stop(self):
        with self._lock:
            if not self._active:
                return self.status()
            for p in self._procs:
                try:
                    p.terminate()
                except OSError:
                    pass
            for p in self._procs:
                try:
                    p.wait(timeout=2.0)
                except Exception:
                    try:
                        p.kill()
                    except OSError:
                        pass
            self._procs = []
            self._active = False
            self._log("stress: stopped")
            return self.status()

    def status(self):
        with self._lock:
            if self._active and all(p.poll() is not None for p in self._procs):
                self._active = False        # every worker exited on its own (duration elapsed)
                self._procs = []
            elapsed = time.monotonic() - self._started_at if self._active else 0.0
            return {
                "active": self._active,
                "cpu_workers": self._cpu_workers if self._active else 0,
                "elapsed": round(elapsed, 1),
                "remaining": round(max(0.0, self._duration - elapsed), 1) if self._active else 0,
                "duration": self._duration if self._active else 0,
            }
