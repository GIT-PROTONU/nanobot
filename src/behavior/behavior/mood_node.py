"""Idle "feel alive" presence supervisor — the behaviour layer's ROS node.

This is the ROS *glue* around the brain. The slow, declarative thinking lives in two
ROS-free modules so it can be unit-tested offline (`pixi run python -m pytest
src/behavior/test`):

  * `presence.py` — the Sismic statechart (states + timed transitions + the personality
    context the guards read).
  * `brain.py` — everything else, in one module: the Purpose Engine (goals + intrinsic
    reward), the Horizon Planner (task queue + A/B bandit), and the platform-agnostic
    *orchestration* of both (`PurposeBrain`) + the chart-context personality (`Personality`).
    The SAME orchestration runs on the dev web harness (`scripts/dev_webui.py`) via injected
    adapters, so there's one base, not two.

This node just: parses params, builds the chart, maps raw ROS topics into the chart's
semantic signals, and on each tick steps the chart and delegates to `PurposeBrain` /
`Personality`. It is deliberately conservative:

  * **Expression only.** It NEVER publishes `/cmd_vel` (or anything that moves the robot),
    so it cannot affect motion safety. The worst it can do is show a face on the OLED.
  * **It yields the panel.** The OLED face has other legitimate owners (the web UI's manual
    mood buttons, TTS "karaoke" words, slam_nav's pick-up reaction). This node animates a
    face only during *true idle* and "stands down" the instant any of those takes over.
  * **Degrades to nothing.** If Sismic isn't importable, or the node is disabled by param,
    it spins doing nothing — the rest of the stack is unaffected.

Consumes (all light std/geometry msgs): `/cmd_vel`, `/goal_pose`, `/oled_word`,
`/oled_face` (to detect a manual/foreign mood vs our own echo), `/left|right_wheel_suspended`,
`/cognition/evolve`, `/cognition/reward`, `/reflect`. Drives `/oled_face`,
`/cognition/request`, `/reflect_request` (auto reflection-mode trigger), and the latched
`/cognition/traits` + `/purpose` + `/task_current` + `/experiments` readouts.
"""
import json
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped

from .presence import build_interpreter, BEATS
from .brain import PurposeBrain, Personality

# Sismic is a pure-python pip/pixi dependency (see pixi.toml [pypi-dependencies]).
# Import defensively so a board that hasn't run `pixi install` yet still boots the
# rest of the stack — the node just runs as a no-op (like oled_display without luma).
try:
    from sismic.model import Event
    from sismic.clock import SimulatedClock
    HAVE_SISMIC = True
except Exception as exc:  # pragma: no cover - depends on env
    HAVE_SISMIC = False
    _SISMIC_ERR = exc

SELF_ECHO_WINDOW = 1.5   # s: a /oled_face equal to our own recent publish = our echo


class MoodNode(Node):
    def __init__(self):
        super().__init__("behavior")
        self.declare_parameters("", [
            ("enable", True),
            ("tick_rate", 4.0),       # Hz the statechart is stepped (after() resolution)
            ("greet_secs", 3.0),      # boot "hello" face duration
            ("idle_secs", 90.0),      # continuous idle before a liveliness beat
            ("perform_secs", 4.0),    # liveliness beat duration
            ("motion_eps", 0.02),     # |cmd_vel| above this counts as moving
            ("motion_hold", 1.2),     # s after last motion still counted "active"
            ("speak_timeout", 5.0),   # s without an /oled_word before "not speaking"
            # --- LLM enrichment of the beats (additive; off if no LLM) ---
            ("enrich_enable", True),  # publish /cognition/request on enrichable beats
            ("enrich_min_interval", 45.0),  # s, per-beat rate-limit on requests
            ("camera_beats", True),   # allow the autonomous camera ("looking") beat
            ("look_every", 4),        # camera beat every Nth idle beat (>=1)
            # --- skill library: let the brain pick a capability (skills/*.md) on a beat ---
            ("skills_enable", True),  # allow the autonomous "skill" beat (web_control selects)
            ("skill_every", 6),       # a skill beat in place of every Nth body (musing) beat (>=1)
            # --- personality / evolution ---
            ("personality_path", ""),       # "" -> ~/.local/state/nanobot/personality.json
            ("smoothing_alpha", 0.1),       # exponential-smoothing rate for `evolve` (0..1)
            ("brain_timeout", 90.0),        # s without an evolve before reverting to baseline
            ("nudge_pickup_caution", 0.92), # fast rule: being picked up eases caution toward this
            ("nudge_pickup_playful", 0.3),  #            and playfulness toward this (startled)
            # --- Purpose Engine + Horizon Planner (goals/reward + A/B) ---
            ("purpose_enable", True),       # run the goal/reward layer (else pure presence)
            ("purpose_period", 600.0),      # s between purpose reflections (reads the decision log)
            ("pursue_min_interval", 180.0), # s, min gap between goal-pursuit ("pursuing") beats
            ("ab_epsilon", 0.2),            # A/B bandit exploration rate (0..1)
            ("reflect_face", "focused"),    # OLED face shown while in reflection mode
            ("greet_face", "happy"),        # OLED "startup mood" shown at boot (greeting)
            # --- autonomous reflection mode: enter on long idle, run a fixed stretch, wake ---
            ("reflect_auto_enable", True),  # let the robot enter reflection mode on its own
            ("reflect_auto_idle", 1200.0),  # s of continuous idle before auto-entering reflection
            ("reflect_auto_secs", 120.0),   # s a self-started reflection runs before auto-waking
            ("purpose_path", ""),           # "" -> ~/.local/state/nanobot/purpose.json
            ("experiments_path", ""),       # "" -> ~/.local/state/nanobot/experiments.json
            # The shared decision log written by web_control — the Purpose Engine's
            # experience input (read-only here). Same default path as web_server.
            ("cognition_log_path", ""),     # "" -> ~/.local/state/nanobot/cognition.log
        ])
        g = self.get_parameter
        self.enable = bool(g("enable").value)
        self.tick_rate = max(1.0, float(g("tick_rate").value))
        self.motion_eps = float(g("motion_eps").value)
        self.motion_hold = float(g("motion_hold").value)
        self.speak_timeout = float(g("speak_timeout").value)
        self.enrich_enable = bool(g("enrich_enable").value)
        self.enrich_min_interval = float(g("enrich_min_interval").value)
        self.camera_beats = bool(g("camera_beats").value)
        self.look_every = max(1, int(g("look_every").value))
        self._reflect_face = str(g("reflect_face").value or "focused")
        self._greet_face = str(g("greet_face").value or "happy")
        self._reflect_auto_enable = bool(g("reflect_auto_enable").value)
        self._reflect_auto_idle = max(0.0, float(g("reflect_auto_idle").value))
        self._reflect_auto_secs = max(1.0, float(g("reflect_auto_secs").value))

        # --- signals updated by callbacks, read by the tick ---
        self._susp_l = False
        self._susp_r = False
        self._last_motion = -1e9       # monotonic time of last cmd_vel motion / goal
        self._speaking = False
        self._last_word_t = -1e9
        self._external_active = False  # a non-empty /oled_face from someone else (web/slam)
        # echo suppression so we don't mistake our OWN /oled_face for a foreign mood
        self._self_face = ("", -1e9)
        self._owns_face = False        # we currently have a non-empty face shown
        # autonomous reflection mode: when did continuous idle begin, and (while WE started a
        # reflection) when should it auto-wake. None deadline = no self-started reflection running.
        self._idle_since = time.monotonic()
        self._auto_reflect_deadline = None

        self.face_pub = self.create_publisher(String, "oled_face", 10)
        # Fire-and-forget enrichment requests for the beat states. web_control executes
        # them (LLM [+camera] -> TTS + mood); we never wait on the reply. Rate-limited
        # per beat via _beat_last so a fast-cycling chart can't spam the API.
        self.cog_pub = self.create_publisher(String, "cognition/request", 10)
        # Ask web_control to run a reflection (consolidate + forge a skill). web_server turns
        # this into the /reflect state command we (and it) act on, so there's one entry path
        # for both the manual web toggle and this autonomous trigger.
        self.reflect_req_pub = self.create_publisher(Bool, "reflect_request", 10)
        self._beat_last = {}
        # Latched readouts so late subscribers (slam_nav's motion clamp, web_control's
        # reflection, the web UI brain card) get them immediately; PurposeBrain/Personality
        # also republish them on a slow heartbeat for late (volatile-QoS) rosbridge subs.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.traits_pub = self.create_publisher(String, "cognition/traits", latched)
        self.purpose_pub = self.create_publisher(String, "purpose", latched)
        self.task_pub = self.create_publisher(String, "task_current", latched)
        self.experiments_pub = self.create_publisher(String, "experiments", latched)

        # --- the brain (ROS-free orchestration; see brain.py) --------------------
        # Personality owns the chart-context traits/registry + their evolution/persist;
        # PurposeBrain owns the goal/reward layer + the A/B planner. Both publish their state
        # through the adapters below (here: latched ROS topics).
        self._personality = Personality(
            path=g("personality_path").value, logger=self.get_logger().warning,
            heartbeat_enable=self.enrich_enable, brain_timeout=float(g("brain_timeout").value),
            nudge_pickup_caution=float(g("nudge_pickup_caution").value),
            nudge_pickup_playful=float(g("nudge_pickup_playful").value),
            publish=lambda s: self.traits_pub.publish(String(data=json.dumps(s))))
        self._brain = PurposeBrain(
            name=self._personality.name, enable=bool(g("purpose_enable").value),
            rng=random.Random(), epsilon=float(g("ab_epsilon").value),
            pursue_min_interval=float(g("pursue_min_interval").value),
            reflect_period=float(g("purpose_period").value),
            skills_enable=bool(g("skills_enable").value),
            skill_every=max(1, int(g("skill_every").value)),
            purpose_path=g("purpose_path").value, experiments_path=g("experiments_path").value,
            cog_log_path=g("cognition_log_path").value,
            picked=lambda: self._susp_l and self._susp_r,
            traits_snapshot=self._personality.live_traits,
            publish_purpose=lambda o: self.purpose_pub.publish(String(data=json.dumps(o))),
            publish_task=lambda p: self.task_pub.publish(String(data=json.dumps(p))),
            publish_experiments=lambda s: self.experiments_pub.publish(String(data=json.dumps(s))),
            logger=self.get_logger().info)

        self._interp = None
        if not self.enable:
            self.get_logger().info("behavior disabled (enable:=false) — idle no-op")
        elif not HAVE_SISMIC:
            self.get_logger().error(
                f"sismic unavailable: {_SISMIC_ERR} — behaviour layer is a no-op "
                "(run `pixi install` to add it)")
        else:
            self._start_chart()

        # Subscriptions are created regardless (cheap), but only matter once the chart
        # is running. Faces/words/twists/switches are all small, low-rate messages.
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_subscription(PoseStamped, "goal_pose", self._on_goal, 5)
        self.create_subscription(String, "oled_word", self._on_word, 10)
        self.create_subscription(String, "oled_face", self._on_face, 10)
        self.create_subscription(Bool, "left_wheel_suspended", self._on_susp_l, 10)
        self.create_subscription(Bool, "right_wheel_suspended", self._on_susp_r, 10)
        # Trait/registry updates proposed by the cognitive layer (slow LLM reflection).
        self.create_subscription(String, "cognition/evolve", self._on_evolve, 10)
        # Human reward (from the web UI via web_control) -> A/B bandit credit.
        self.create_subscription(String, "cognition/reward", self._on_reward, 10)
        # Reflection-mode state command (from the web UI toggle or our own auto trigger,
        # mediated by web_control).
        self.create_subscription(Bool, "reflect", self._on_reflect, latched)

        if self._interp is not None:
            self.create_timer(1.0 / self.tick_rate, self._tick)

    # --- statechart setup ----------------------------------------------------
    def _start_chart(self):
        try:
            g = self.get_parameter
            self._t0 = time.monotonic()
            self._clock = SimulatedClock()
            self._clock.time = 0.0
            # build_interpreter runs the initial step, so the chart enters `greeting`
            # and shows the boot face. Best-effort: if the OLED node isn't up yet the
            # publish is simply lost.
            self._interp, _ = build_interpreter(
                self._emit_face, do_beat=self._do_beat,
                greet_secs=float(g("greet_secs").value),
                idle_secs=float(g("idle_secs").value),
                perform_secs=float(g("perform_secs").value),
                camera_beats=self.camera_beats, look_every=self.look_every,
                traits=self._personality.traits, registry=self._personality.registry,
                alpha=float(g("smoothing_alpha").value), clock=self._clock,
                reflect_face=self._reflect_face, greet_face=self._greet_face,
                rng=random.Random())          # the idle-beat lottery (priority-weighted)
            self._personality.attach(self._interp)
            self.get_logger().info(
                f"behavior up: presence statechart (personality '{self._personality.name}', "
                f"traits {self._personality.traits})")
        except Exception as exc:
            self._interp = None
            self.get_logger().error(f"statechart init failed: {exc} — running as no-op")

    # --- the injected face action + the node's own release path --------------
    def _emit_face(self, mood):
        """Publish a mood on /oled_face. Called both by the statechart (`on entry`) and
        by the node to release the panel. Records the value so the /oled_face
        subscription can tell our own echo from a foreign (web/slam_nav) mood."""
        mood = str(mood)
        self._self_face = (mood, time.monotonic())
        self._owns_face = bool(mood)
        self.face_pub.publish(String(data=mood))

    def _enrich_ready(self, name):
        """True if a fire-and-forget enrichment request for `name` is allowed now (enabled +
        not rate-limited)."""
        return (self.enrich_enable
                and (time.monotonic() - self._beat_last.get(name, -1e9))
                >= self.enrich_min_interval)

    def _do_beat(self, name):
        """Injected as the chart's `do_beat`. Always shows the beat's predefined default
        face (so a beat is meaningful even with no LLM), then — if enrichment is enabled and
        this beat isn't rate-limited — fires a fire-and-forget /cognition/request for
        web_control to enrich asynchronously (LLM line + camera + mood). Never blocks.

        The `musing` body beat (the chooser's highest-priority default) is upgraded by the
        brain: to a `pursuing` beat when the Horizon Planner has a verified task, else to a
        `skill` beat every Nth body beat — that's how the Purpose/Planner/skill layers reach
        expression. Goals win the slot, then skills, else the plain beat (which may be any of
        the chooser's picks: musing/looking/wondering/listening)."""
        beat = BEATS.get(name)
        if beat is None:
            return
        if name == "musing" and self._enrich_ready("pursuing"):
            spec = self._brain.next_pursuing(time.monotonic())
            if spec is not None:
                self._deliver_pursuing(spec, beat=BEATS["pursuing"])
                return
        if name == "musing" and self._brain.take_skill_beat() and self._enrich_ready("skill"):
            self._deliver_skill_beat()
            return
        self._emit_face(beat.face)                 # offline-safe default, echo-tracked
        if not self._enrich_ready(name):
            return
        self._beat_last[name] = time.monotonic()
        req = {"beat": name, "state": name, "prompt": beat.prompt,
               "camera": bool(beat.camera), "audio": bool(beat.audio),
               # carry the current personality so the executor can colour the line
               "traits": self._personality.live_traits()}
        self.cog_pub.publish(String(data=json.dumps(req)))

    def _deliver_pursuing(self, spec, beat):
        """Narrate the planner's current task as a `pursuing` beat: show the default face and
        fire an enrichment request carrying the task phrase + the A/B style hint + variant ids.
        (The task + moved A/B stats are already announced by PurposeBrain.next_pursuing.)"""
        self._emit_face(beat.face)
        self._beat_last["pursuing"] = time.monotonic()
        req = {"beat": "pursuing", "state": "pursuing",
               "prompt": self._brain.pursuing_prompt(spec, beat.prompt),
               "camera": bool(spec["camera"]), "audio": False,
               "exp": spec["exp"], "variant": spec["variant"], "task": spec["task"],
               "traits": self._personality.live_traits()}
        self.cog_pub.publish(String(data=json.dumps(req)))

    def _deliver_skill_beat(self):
        """Fire a `skill` beat: show the offline-safe default face, then hand off to
        web_control to pick a capability from the skill library and perform it. Like every
        beat this is fire-and-forget — a slow/absent brain just leaves the default face."""
        beat = BEATS.get("skill")
        if beat is not None:
            self._emit_face(beat.face)
        self._beat_last["skill"] = time.monotonic()
        req = {"beat": "skill", "state": "acting", "prompt": "", "camera": False,
               "audio": False, "traits": self._personality.live_traits()}
        self.cog_pub.publish(String(data=json.dumps(req)))

    # --- cognitive-layer inputs ----------------------------------------------
    def _on_evolve(self, msg: String):
        """A trait/registry proposal from the cognitive layer -> Personality (smoothed in the
        chart, feeds the brain-alive heartbeat)."""
        if self._interp is None:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if self._personality.on_evolve(payload):
            self.get_logger().info("cognitive layer reachable again")

    def _on_reward(self, msg: String):
        """A human reward from the web UI: credit the A/B arm that produced the narrated line
        (contextual). Global reward shapes the intrinsic-reward weights via the decision log on
        the next purpose reflection (web_control logs every reward there)."""
        try:
            r = json.loads(msg.data)
        except Exception:
            return
        self._brain.apply_reward(r.get("value", 0), r.get("target"),
                                 scope=str(r.get("scope")))

    def _on_reflect(self, msg: Bool):
        """Enter/leave reflection mode. While on, the chart shows a calm face and pauses beats;
        the brain consolidates (reflect on purpose + finalize A/B winners). The LLM reflection,
        phrase-bank regen, and skill workshop run on the web_control side."""
        on = bool(msg.data)
        if not self._brain.set_reflecting(on, traits=self._personality.live_traits()):
            return
        if self._interp is not None:
            self._interp.queue(Event("reflect" if on else "wake"))
        self.get_logger().info("reflecting — consolidating brain + forging skills (beats paused)"
                               if on else "reflection ended — resuming presence")

    # --- signal callbacks ----------------------------------------------------
    def _on_cmd(self, msg: Twist):
        if abs(msg.linear.x) > self.motion_eps or abs(msg.angular.z) > self.motion_eps:
            self._last_motion = time.monotonic()

    def _on_goal(self, _msg: PoseStamped):
        # A new goal means the robot is about to work — treat it as activity.
        self._last_motion = time.monotonic()

    def _on_word(self, msg: String):
        self._speaking = bool(msg.data.strip())
        self._last_word_t = time.monotonic()

    def _on_face(self, msg: String):
        """Track whether someone ELSE is driving the face (web manual mood, or
        slam_nav's pick-up reaction). Suppress our own echo so we don't stand down
        from a face we set ourselves."""
        data = msg.data
        sf, st = self._self_face
        if data == sf and (time.monotonic() - st) < SELF_ECHO_WINDOW:
            return                                  # our own publish bouncing back
        self._external_active = bool(data.strip())  # "" = the other owner released it

    def _on_susp_l(self, msg: Bool):
        self._susp_l = bool(msg.data)

    def _on_susp_r(self, msg: Bool):
        self._susp_r = bool(msg.data)

    # --- autonomous reflection-mode trigger ----------------------------------
    def _auto_reflect(self, now, stand):
        """Let the robot drift into reflection mode on its own after a long idle stretch, run
        it for a fixed duration, then wake. Best-effort + decoupled: we only publish a
        /reflect_request (web_control mediates it into the /reflect state we then act on) and
        manage our own request lifecycle — so a manual web toggle is never disturbed.

        `_auto_reflect_deadline` is set only while WE started the current reflection; a manual
        one leaves it None, so we never auto-wake the user's reflection."""
        if not self._reflect_auto_enable:
            return
        if self._auto_reflect_deadline is not None:        # a self-started reflection is running
            if now >= self._auto_reflect_deadline or stand:  # time's up, or activity resumed
                self.reflect_req_pub.publish(Bool(data=False))
                self._auto_reflect_deadline = None
                self._idle_since = now
            return
        if self._brain.reflecting:                          # a manual reflection — leave it be
            return
        if not stand and (now - self._idle_since) >= self._reflect_auto_idle:
            self.reflect_req_pub.publish(Bool(data=True))
            self._auto_reflect_deadline = now + self._reflect_auto_secs
            self.get_logger().info("idle a while — entering reflection mode on my own")

    # --- the periodic statechart step ----------------------------------------
    def _tick(self):
        now = time.monotonic()
        self._clock.time = now - self._t0

        picked = self._susp_l and self._susp_r
        manual = self._external_active
        speaking = self._speaking and (now - self._last_word_t) < self.speak_timeout
        active = (now - self._last_motion) < self.motion_hold
        stand = picked or manual or speaking or active
        if stand:
            self._idle_since = now                         # any activity resets the idle clock

        try:
            self._auto_reflect(now, stand)                 # maybe enter/exit a self-started reflection
            # Reflection is a deliberate, sticky mode (only `wake` leaves it), so we skip the
            # normal stand-down/resume arbitration while consolidating.
            dormant = "dormant" in self._interp.configuration
            if not self._brain.reflecting:
                if stand and not dormant:
                    # Hand the panel back. If we're standing down for *another writer*
                    # (pick-up / a manual web mood), stay silent — they own /oled_face and
                    # a stray "" from us would stomp their value. If it's just motion or
                    # speech (nobody else writes the face), release to the dashboard so we
                    # don't leave a stale happy face up while the robot drives/talks.
                    if (picked or manual):
                        self._owns_face = False
                    elif self._owns_face:
                        self._emit_face("")
                    self._interp.queue(Event("standdown"))
                elif (not stand) and dormant:
                    self._interp.queue(Event("resume"))
            if self._personality.tick_events(now, picked) == "lost":   # fast rules + heartbeat
                self.get_logger().warning(
                    "cognitive layer unreachable — reverting to baseline personality")
            self._interp.execute()
            self._personality.publish_and_persist(now)     # publish + persist on change
            self._brain.tick(now)                          # purpose reflection + latched republish
        except Exception as exc:
            # The behaviour layer must never take the process down; log + keep going.
            self.get_logger().error(f"statechart step failed: {exc}",
                                    throttle_duration_sec=10.0)

    def destroy_node(self):
        if self._interp is not None:               # persist the latest drift on clean exit
            self._personality.save()
        if self._brain.enable:                     # persist purpose + A/B state
            self._brain.save()
        super().destroy_node()


def main():
    rclpy.init()
    node = MoodNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
