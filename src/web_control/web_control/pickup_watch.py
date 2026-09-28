"""Pickup watch: the lift-stop + release-turn state machine (pure, stdlib-only,
unit-tested offline — the rclpy wiring lives in telemetry.py/web_server.py).

Behaviour (2026-09-28): as soon as BOTH wheels' off-ground microswitches read
"up" continuously for `stop_secs` (default 5 s, web-tunable), the robot is
picked up — latch a STOP and web_server kills every motion source (braked stop,
maneuvers cancelled, Nav2 goals cancelled; drives/moves/goals refused while
latched). When both wheels read grounded again for a short confirm window,
emit SPIN — a ~180° in-place re-orientation turn (the classic "done with you"
move; direction/angle = web_control's pickup_spin_deg param).

The /pickup_override test hook (Int8: -1 auto, 0 force-grounded, 1 force-lifted)
is honored here like every other suspension consumer, so the whole behaviour is
testable off the ground from the web Coprocessor card.

Time comes IN (monotonic `now`) — no clock reads, no I/O, fully deterministic.
"""
STOP = "stop"    # both wheels up >= stop_secs  -> latch, stop everything
SPIN = "spin"    # grounded again >= confirm    -> clear latch, do the turn

RELEASE_CONFIRM_SECS = 1.0   # s both wheels must be grounded before the release
                             # spin fires (mechanical-switch bounce debounce)


class PickupWatch:
    """Debounce + latch machine over the effective (override-aware) switch pair.

    `update()` is idempotent per state: STOP is emitted exactly once per latch,
    SPIN exactly once per release. One-wheel samples (carried tilted) reset BOTH
    timers — the 5 s must be BOTH-up-continuously, and the release confirm must
    be BOTH-grounded-continuously."""

    def __init__(self):
        self.latched = False          # STOP fired, waiting for the release SPIN
        self._both_up_since = None    # monotonic both wheels continuously up, or None
        self._grounded_since = None   # monotonic both wheels continuously down, or None

    def reset(self):
        """Feature disabled mid-latch: stand down quietly (no SPIN)."""
        self.latched = False
        self._both_up_since = None
        self._grounded_since = None

    @staticmethod
    def effective(l, r, override):
        """The override-aware switch pair (same contract as web_server._susp_eff)."""
        if override == 0:
            return False, False
        if override == 1:
            return True, True
        return bool(l), bool(r)

    def update(self, l, r, override, now, stop_secs, release_secs=RELEASE_CONFIRM_SECS):
        """Feed one sample (switch pair + override) at monotonic `now`; return a
        list of events (possibly empty). `stop_secs` is the lift duration that
        latches the stop; `release_secs` the grounded duration that fires the
        turn."""
        l, r = self.effective(l, r, override)
        both_up = l and r
        both_down = not l and not r
        if both_up:
            if self._both_up_since is None:
                self._both_up_since = now
            self._grounded_since = None
        else:
            self._both_up_since = None
            if both_down:
                if self._grounded_since is None:
                    self._grounded_since = now
            else:
                self._grounded_since = None
        events = []
        if not self.latched:
            if (self._both_up_since is not None
                    and now - self._both_up_since >= stop_secs):
                self.latched = True
                events.append(STOP)
        elif (both_down and self._grounded_since is not None
                and now - self._grounded_since >= release_secs):
            self.latched = False
            self._both_up_since = None
            self._grounded_since = None
            events.append(SPIN)
        return events

    def up_for(self, now):
        """Seconds both wheels have been up (0.0 when not), for the frame/UI."""
        return 0.0 if self._both_up_since is None else max(0.0, now - self._both_up_since)
