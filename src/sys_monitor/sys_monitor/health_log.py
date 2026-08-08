"""ROS-free health-event watch: a durable, timestamped record of ESP32-link and
LDS outages, so intermittent failures leave a trail to diagnose after the fact.

`HealthWatch.update()` is fed cheap signals once a second (heartbeat age, lidar
rpm/frame-rate from the ESP32, the /dev/shm scan-blob header from the SBC branch)
and emits transition lines only — DOWN with a *classified* cause, periodic
still-down counter snapshots, UP with the outage duration. The classification
uses the same discriminators as the lds-scan-path memory: rpm≈0 = lidar
power/motor, rpm ok + frames 0 = ESP32 RX branch, rx frozen = SBC branch dead,
err climbing = SBC branch garbled, blob mtime old = driver silent.

The log lives outside the repo/.run (default ~/.local/state/nanobot/health.log,
same home as tts.json/llm.json) so it survives reboots and stack restarts; a
size cap rotates it once to health.log.1. Pure logic — unit-tested offline in
test/test_health_log.py; monitor_node owns the subscriptions and the tick.
"""
import json
import os
import time
from collections import deque

ESP32_TIMEOUT = 5.0      # s without /esp32_heartbeat -> link down
LDS_BLOB_STALE = 6.0     # s since the scan blob was written -> driver silent
RPM_MIN = 60.0           # below this the lidar is "not spinning"
HZ_MIN = 100.0           # valid-frame rate below this = ESP32 sees no frames
START_GRACE = 30.0       # s after start before "never came up" counts as DOWN
PROGRESS_SECS = 60.0     # during an outage, snapshot counters this often
MAX_BYTES = 512 * 1024   # rotate health.log -> health.log.1 past this

# Localization-pipeline feed staleness (see FeedWatch below). These feeds publish
# continuously (raw /odom ~15 Hz, EKF /odometry/filtered ~15 Hz, /imu/euler ~50 Hz),
# but a STALE feed tells you the publisher (or an intermediate process like the EKF)
# died without necessarily tripping the ESP32/LDS watchers above — so we watch each
# one's age independently.
PIPE_FEEDS = ("odom", "ekf", "imu")   # the three feeds tracked per-launch
FEED_TIMEOUT = 3.0         # s without a message before a feed is declared DOWN
FEED_GRACE = 20.0          # s of boot grace before "never seen" counts as DOWN

BLOB_PATH = "/dev/shm/nano_scan.bin"


def read_scan_blob_header(path=BLOB_PATH):
    """Scan-blob JSON header + file age in seconds, or None if absent/unparsable."""
    try:
        st = os.stat(path)
        with open(path, "rb") as f:
            raw = f.read(300)
        hdr = json.loads(raw[:raw.find(b"}") + 1])
        hdr["age"] = time.time() - st.st_mtime
        return hdr
    except Exception:
        return None


class HealthWatch:
    """Feed update() once a second; append the returned lines to the log."""

    def __init__(self, path, now=None):
        self.path = path
        now = time.monotonic() if now is None else now
        self.started = now
        # None = unknown (still in the boot grace window), else bool
        self.esp32_up = None
        self.lds_up = None
        self.esp32_since = now          # when the current state was entered
        self.lds_since = now
        self._hist = deque(maxlen=8)    # (now, rx, err) blob-counter history
        self._stale = deque(maxlen=8)   # ditto, but only samples taken while down
        self._verdict_due = None        # emit the rx-frozen-vs-garbled verdict once
        self._progress_at = 0.0
        self._out_rx0 = 0               # counters at outage start (for the UP line)
        self._out_err0 = 0

    # -- log file ---------------------------------------------------------

    def write(self, lines):
        if not lines:
            return
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            try:
                if os.path.getsize(self.path) > MAX_BYTES:
                    os.replace(self.path, self.path + ".1")
            except OSError:
                pass
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
            with open(self.path, "a") as f:
                for ln in lines:
                    f.write(f"{stamp} {ln}\n")
        except Exception:
            pass                        # health logging must never hurt the stack

    # -- state machine ----------------------------------------------------

    def update(self, now, hb_age, rpm, hz, blob):
        """hb_age/rpm/hz: seconds since last /esp32_heartbeat and the last-seen
        lidar rpm / valid-frame rate (nan if stale); blob: header dict + 'age',
        or None. Returns the lines to log this tick (usually none)."""
        lines = []
        if blob is not None and "rx" in blob:
            self._hist.append((now, blob.get("rx", 0), blob.get("err", 0)))
            if self.lds_up is False:
                self._stale.append(self._hist[-1])

        hb_why = ("no heartbeat ever received" if hb_age == float("inf")
                  else f"no heartbeat for {hb_age:.0f}s")
        lines += self._edge("esp32", "esp32_up", "esp32_since", now,
                            up=hb_age <= ESP32_TIMEOUT, down_why=hb_why)
        lds_ok = (blob is not None and blob.get("age", 1e9) <= LDS_BLOB_STALE
                  and not blob.get("stale"))
        lines += self._edge("lds", "lds_up", "lds_since", now,
                            up=lds_ok,
                            down_why=self._classify(rpm, hz, blob))

        # The rx-frozen-vs-garbled call can't be made at the DOWN edge (the counter
        # window still holds pre-failure bytes) — judge it once from samples taken
        # entirely during the outage, a few seconds in.
        if self._verdict_due is not None and now >= self._verdict_due:
            if len(self._stale) >= 4:
                self._verdict_due = None
                lines.append(f"lds outage counters: {self._verdict()} "
                             f"(rpm {rpm:.0f}, frames {hz:.0f}/s)")
            elif now - self.lds_since > 4 * LDS_BLOB_STALE:
                self._verdict_due = None    # blob itself died; nothing to judge

        if self.lds_up is False and now - self._progress_at >= PROGRESS_SECS:
            self._progress_at = now
            lines.append(f"lds still down {now - self.lds_since:.0f}s: "
                         f"{self._deltas(self._stale)} (rpm {rpm:.0f}, frames {hz:.0f}/s)")
        return lines

    def _edge(self, name, up_attr, since_attr, now, up, down_why):
        was = getattr(self, up_attr)
        if up == was:
            return []
        in_grace = was is None and now - self.started < START_GRACE
        if not up and in_grace:
            return []                   # don't call it DOWN while booting
        held = now - getattr(self, since_attr)
        setattr(self, up_attr, up)
        setattr(self, since_attr, now)
        if up:
            if name == "lds":
                self._verdict_due = None
            if was is None:
                return [f"{name} UP ({held:.0f}s after start)"]
            return [f"{name} UP after {held:.0f}s down" + self._recovery(name)]
        suffix = "down since start" if was is None else f"was up {held:.0f}s"
        if name == "lds":
            self._progress_at = now
            self._out_rx0, self._out_err0 = self._latest_counters()
            self._stale.clear()
            # Only the "upstream fine" case needs the counter verdict; when the
            # cause is upstream (no spin / no frames / driver dead), a frozen rx
            # is expected and the SBC-branch verdict would mislead.
            if "verdict follows" in down_why:
                self._verdict_due = now + 5.0
            suffix += ", esp32 up" if self.esp32_up else ", esp32 DOWN too"
        return [f"{name} DOWN: {down_why} ({suffix})"]

    def _recovery(self, name):
        if name != "lds":
            return ""
        rx, err = self._latest_counters()
        if rx >= self._out_rx0:
            return f" (rx +{rx - self._out_rx0}, err +{err - self._out_err0} during outage)"
        return " (counters reset: driver restarted)"

    def _latest_counters(self):
        return self._hist[-1][1:] if self._hist else (0, 0)

    @staticmethod
    def _deltas(hist):
        """rx/err deltas over a (now, rx, err) history window."""
        if len(hist) < 2:
            return "no blob counters"
        t0, rx0, err0 = hist[0]
        t1, rx1, err1 = hist[-1]
        if rx1 < rx0:
            return "counters reset (driver restarted)"
        dt = max(1e-3, t1 - t0)
        return f"rx +{rx1 - rx0} ({(rx1 - rx0) / dt:.0f} B/s), err +{err1 - err0}"

    def _verdict(self):
        """Frozen vs garbled, judged only from during-outage samples."""
        deltas = self._deltas(self._stale)
        rx = self._stale[-1][1] - self._stale[0][1]
        if rx <= 0:
            return f"{deltas} -> no bytes at SBC, PA1 branch dead"
        return f"{deltas} -> SBC RX degraded/garbled"

    def _classify(self, rpm, hz, blob):
        """Why are there no scans? Uses the discriminators from the LDS memory.
        Called at the DOWN edge, so only instantly-reliable facts here — the
        SBC-branch frozen-vs-garbled verdict follows a few seconds later."""
        if blob is None:
            return "no scan blob file (driver never wrote one)"
        if blob.get("age", 0) > LDS_BLOB_STALE:
            return f"driver silent (blob {blob['age']:.0f}s old) -> node dead/stuck"
        if not blob.get("open", 1):
            return "serial port not open -> device missing/busy"
        if self.esp32_up is False or rpm != rpm:
            return "no scans; lidar rpm unknown (ESP32 link down)"
        if rpm < RPM_MIN:
            return f"lidar not spinning (rpm {rpm:.0f}) -> power/spin motor"
        if hz == hz and hz < HZ_MIN:
            return (f"spinning (rpm {rpm:.0f}) but ESP32 sees {hz:.0f} frames/s "
                    f"-- lidar TX or ESP32 RX branch")
        return (f"no complete revolutions at SBC (rpm {rpm:.0f}, frames {hz:.0f}/s "
                f"= upstream fine); counter verdict follows")


class FeedWatch:
    """Edge-tracking staleness watch for a single continuously-publishing ROS topic.

    Translates "how many seconds since the last message arrived" into up/down
    transitions with a boot grace, so a node that never came up isn't reported as a
    mid-flight failure. Mirrors HealthWatch's shape (edge-only lines, returns them for
    callers to write/echo) but stays perfectly generic — monitor_node feeds it the
    per-topic ages it already computes in its tick.

    A watcher that was healthy and then goes stale is reported with how long it was up;
    a feed held DOWN all boot then appears as "down since start". Distinct from
    HealthWatch's up/down bools (which callers compute from counts) by being age-driven
    so the SAME feed list drives both the durable log and the /diagnostics status.
    """

    def __init__(self, name, timeout=FEED_TIMEOUT, grace=FEED_GRACE, now=None):
        self.name = name
        self.timeout = timeout
        self.grace = grace
        now = time.monotonic() if now is None else now
        self.started = now
        self.up = None          # None (unknown, in boot) | True | False
        self.since = now        # when the current state began
        self.ever_seen = False  # feed ever delivered a message since this watch started

    def update(self, now, age):
        """age = seconds since the feed's last message (None/inf = never seen). Returns
        transition lines to log, or [] when nothing changed / still in boot grace."""
        if age is not None and age <= self.grace:
            self.ever_seen = True
        fresh = age is not None and age <= self.timeout

        if self.up == fresh:
            return []

        # Unknown boot state resolves the first time we have enough evidence.
        if self.up is None:
            if fresh:
                self.up, self.since = True, now
                return []
            if now - self.started >= self.grace:
                # Boot grace over and still not fresh. Whether we ever saw a message
                # (first msg fell inside grace, then the feed died) or never did,
                # it's DOWN as of the grace expiry — don't sit in "unknown" forever.
                self.up, self.since = False, now
                why = ("no message ever" if age is None else f"age {age:.0f}s")
                if self.ever_seen:
                    return [f"feed {self.name} DOWN ({why} > {self.timeout}s timeout)"]
                return [f"feed {self.name} DOWN since start "
                        f"({why} > {self.timeout}s timeout)"]
            return []                             # mid-boot, nothing to say yet

        # True up/down transition (boot resolved). `held` is how long the previous
        # state lasted.
        prev = self.up
        held = now - self.since
        self.up, self.since = fresh, now
        if fresh:
            return [f"feed {self.name} UP after {held:.0f}s down"]
        return [f"feed {self.name} DOWN: was up {held:.0f}s "
                f"(age {age:.0f}s > {self.timeout}s)"]
