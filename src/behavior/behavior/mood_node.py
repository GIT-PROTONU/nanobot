"""Idle "feel alive" presence supervisor — the first node of the behaviour layer.

This is the smallest genuinely-useful slice of the planned statechart behaviour
layer (see the behavior-layer-plan memory): a **Sismic** statechart that makes the
robot feel a little alive when it's otherwise sitting idle, by driving the OLED
*face* (`/oled_face`). It is deliberately conservative:

  * **Expression only.** It NEVER publishes `/cmd_vel` (or anything that moves the
    robot), so it cannot affect motion safety. The worst it can ever do is show a
    face on the OLED at the wrong moment.
  * **It yields the panel.** The OLED face has several legitimate owners already —
    the web UI's manual mood buttons, TTS "karaoke" words, and slam_nav's pick-up
    reaction. This node only animates a face during *true idle* and "stands down"
    (touches nothing) the instant any of those takes over, so it doesn't fight them.
  * **Degrades to nothing.** If Sismic isn't importable, or the node is disabled by
    param, it simply spins doing nothing — the rest of the stack is unaffected.

The statechart (embedded below as YAML) is the slow "brain": states + timed
transitions via Sismic's built-in ``after(...)`` guard. The node only translates raw
ROS topics into a handful of semantic signals and decides when to stand down. This
keeps the behaviour declarative and unit-testable in isolation (no ROS needed) —
which is exactly why Sismic was chosen over a hand-rolled FSM.

Lifecycle (single region):

    greeting --after(greet_secs)--> idle_life{ resting <-> performing }
       │                                  │
       └────────────── standdown ─────────┴──> dormant --resume--> idle_life

  * **greeting** — a brief happy face at boot ("hello"), then settle to the dashboard.
  * **resting**  — the dashboard (face cleared). After ``idle_secs`` of continuous
    idle, briefly come alive.
  * **performing** — a short happy "look around" beat (``perform_secs``), then back to
    resting. So during long idle the robot blinks to life every now and then.
  * **dormant** — something else owns the panel (the robot is moving / has a goal, is
    speaking, is picked up, or the web UI set a manual mood). Touch nothing until clear.

Consumes (all light std/geometry msgs): `/cmd_vel`, `/goal_pose`, `/oled_word`,
`/oled_face` (to detect a manual/foreign mood vs our own echo),
`/left_wheel_suspended`, `/right_wheel_suspended`. Drives `/oled_face` only.

The statechart itself lives in `presence.py` (ROS-free, so it's unit-testable offline:
`pixi run python -m pytest src/behavior/test`).
"""
import copy
import json
import os
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped

from .presence import build_interpreter, BEATS, DEFAULT_TRAITS, DEFAULT_REGISTRY
from .purpose import (default_purpose, merge_purpose, reflect_purpose,
                      summarize_experience)
from .planner import Planner

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
            ("meditate_face", "focused"),   # OLED face shown while meditating/consolidating
            ("greet_face", "happy"),        # OLED "startup mood" shown at boot (greeting)
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
        self._skills_enable = bool(g("skills_enable").value)
        self._skill_every = max(1, int(g("skill_every").value))
        self._body_beat_n = 0          # counts body (musing) beats, for the skill cadence
        self.smoothing_alpha = float(g("smoothing_alpha").value)
        self.brain_timeout = float(g("brain_timeout").value)
        self._nudge_pickup_caution = float(g("nudge_pickup_caution").value)
        self._nudge_pickup_playful = float(g("nudge_pickup_playful").value)
        self._purpose_enable = bool(g("purpose_enable").value)
        self._purpose_period = max(30.0, float(g("purpose_period").value))
        self._pursue_min_interval = float(g("pursue_min_interval").value)
        self._ab_epsilon = float(g("ab_epsilon").value)
        self._meditate_face = str(g("meditate_face").value or "focused")
        self._greet_face = str(g("greet_face").value or "happy")
        self.personality = self._load_personality()

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

        self.face_pub = self.create_publisher(String, "oled_face", 10)
        # Fire-and-forget enrichment requests for the beat states. web_control executes
        # them (LLM [+camera] -> TTS + mood); we never wait on the reply. Rate-limited
        # per beat via _beat_last so a fast-cycling chart can't spam the API.
        self.cog_pub = self.create_publisher(String, "cognition/request", 10)
        self._beat_last = {}
        # Latched publisher of the current personality so late subscribers (slam_nav's
        # motion clamp, web_control's reflection) get it immediately.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.traits_pub = self.create_publisher(String, "cognition/traits", latched)
        # --- Purpose Engine + Horizon Planner ---------------------------------
        # The slow "identity" layer (goals + intrinsic reward) and the "strategy" layer
        # (task queue + A/B bandit). Pure/local; narrative-only. Latched topics let the web
        # UI read the current purpose / task / experiment stats; they're also republished on a
        # slow heartbeat so a late (volatile-QoS) rosbridge subscriber still picks them up.
        self.purpose_pub = self.create_publisher(String, "purpose", latched)
        self.task_pub = self.create_publisher(String, "task_current", latched)
        self.experiments_pub = self.create_publisher(String, "experiments", latched)
        self._rng = random.Random()
        self._purpose = merge_purpose(self._load_json(self._purpose_file()),
                                      name=self.personality.get("name", "Nano"))
        self._planner = None
        if self._purpose_enable:
            self._planner = Planner(
                objective_id=self._purpose["objective"]["id"],
                state=self._load_json(self._experiments_file()),
                epsilon=self._ab_epsilon, min_interval=self._pursue_min_interval)
        self._purpose_last = 0.0           # monotonic of last purpose reflection
        self._pub_hb = 0.0                 # monotonic of last latched-topic republish
        self._meditating = False
        # Evolution state (drivers feed Sismic `evolve`/`brain_lost` events).
        self._last_brain = time.monotonic()    # last time the cognitive layer spoke
        self._brain_lost = False
        self._was_picked = False               # rising-edge detect for the pickup rule
        self._pers_last_pub = None             # last published traits/registry snapshot
        self._pers_dirty = False
        self._pers_last_save = 0.0

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
        # Meditation/consolidation toggle (from the web UI via web_control).
        self.create_subscription(Bool, "meditate", self._on_meditate, latched)

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
                traits=self.personality["traits"], registry=self.personality["registry"],
                alpha=self.smoothing_alpha, clock=self._clock,
                meditate_face=self._meditate_face, greet_face=self._greet_face)
            self.get_logger().info(
                f"behavior up: presence statechart (personality '{self.personality['name']}', "
                f"traits {self.personality['traits']})")
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
        face (so a beat is meaningful even with no LLM), then — if enrichment is enabled
        and this beat isn't rate-limited — fires a fire-and-forget /cognition/request for
        web_control to enrich asynchronously (LLM line + camera + mood). Never blocks.

        Goal-pursuit upgrade: the idle sensor beat (`musing`) becomes a `pursuing` beat
        whenever the Horizon Planner has a verified task to narrate (and pursuing isn't
        rate-limited) — that's how the Purpose/Planner layers reach expression."""
        beat = BEATS.get(name)
        if beat is None:
            return
        if (name == "musing" and self._planner is not None and self._purpose_enable
                and not self._meditating and self._enrich_ready("pursuing")):
            spec = self._planner.next_task(self._world_state(), self._rng,
                                           now=time.monotonic())
            if spec is not None:
                self._deliver_pursuing(spec)
                return
        # Skill-beat upgrade (mirrors pursuing): every Nth body beat, hand off to web_control
        # to PICK a capability from the skill library (skills/*.md) and perform it. Goals
        # (pursuing) win the slot; skills come next; otherwise it's a plain musing beat.
        if name == "musing" and self._skills_enable and not self._meditating:
            self._body_beat_n += 1
            if (self._body_beat_n % self._skill_every == 0
                    and self._enrich_ready("skill")):
                self._deliver_skill_beat()
                return
        self._emit_face(beat.face)                 # offline-safe default, echo-tracked
        if not self._enrich_ready(name):
            return
        self._beat_last[name] = time.monotonic()
        req = {"beat": name, "state": name, "prompt": beat.prompt,
               "camera": bool(beat.camera), "audio": False,
               # carry the current personality so the executor can colour the line
               "traits": dict(self._interp.context["traits"])}
        self.cog_pub.publish(String(data=json.dumps(req)))

    def _world_state(self):
        """The light world snapshot the planner verifies a task against (narrative-only)."""
        return {"picked": self._susp_l and self._susp_r,
                "sensors_fresh": True,
                "meditating": self._meditating}

    def _deliver_pursuing(self, spec):
        """Narrate the planner's current task as a `pursuing` beat: show the default face,
        publish the task (so the web UI + reward path can credit the right A/B arm), and fire
        an enrichment request carrying the task phrase + the A/B style hint + variant ids."""
        beat = BEATS["pursuing"]
        self._emit_face(beat.face)
        self._beat_last["pursuing"] = time.monotonic()
        target = {"task": spec["task"], "exp": spec["exp"], "variant": spec["variant"]}
        self.task_pub.publish(String(data=json.dumps(
            {**target, "text": spec["text"], "t": time.time()})))
        prompt = beat.prompt.format(task=spec["text"])
        if spec.get("style_hint"):
            prompt = prompt + " " + spec["style_hint"]
        req = {"beat": "pursuing", "state": "pursuing", "prompt": prompt,
               "camera": bool(spec["camera"]), "audio": False,
               "exp": spec["exp"], "variant": spec["variant"], "task": spec["task"],
               "traits": dict(self._interp.context["traits"])}
        self.cog_pub.publish(String(data=json.dumps(req)))
        self._publish_experiments()                # stats moved (new assignment)

    def _deliver_skill_beat(self):
        """Fire a `skill` beat: show the offline-safe default face, then hand off to
        web_control to pick a capability from the skill library and perform it. Like every
        beat this is fire-and-forget — a slow/absent brain just leaves the default face."""
        beat = BEATS.get("skill")
        if beat is not None:
            self._emit_face(beat.face)
        self._beat_last["skill"] = time.monotonic()
        req = {"beat": "skill", "state": "acting", "prompt": "", "camera": False,
               "audio": False, "traits": dict(self._interp.context["traits"])}
        self.cog_pub.publish(String(data=json.dumps(req)))

    # --- personality: load / evolve / heartbeat / publish / persist ----------
    def _personality_file(self):
        p = self.get_parameter("personality_path").value
        return p or os.path.expanduser("~/.local/state/nanobot/personality.json")

    def _load_personality(self):
        """Seed personality from personality.json (written by the creator / persisted as it
        drifts), merged over the frozen defaults. Best-effort."""
        base = {"name": "Nano", "persona": "", "traits": dict(DEFAULT_TRAITS),
                "registry": copy.deepcopy(DEFAULT_REGISTRY)}
        try:
            with open(self._personality_file(), encoding="utf-8") as f:
                saved = json.load(f)
            if isinstance(saved, dict):
                for k in ("name", "persona"):
                    if isinstance(saved.get(k), str):
                        base[k] = saved[k]
                if isinstance(saved.get("traits"), dict):
                    base["traits"].update(saved["traits"])
                if isinstance(saved.get("registry"), dict):
                    for n, patch in saved["registry"].items():
                        base["registry"].setdefault(n, {}).update(patch or {})
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.get_logger().warning(f"personality load failed ({exc}) — using defaults")
        return base

    def _on_evolve(self, msg: String):
        """A trait/registry proposal from the cognitive layer -> a Sismic `evolve` event
        (smoothed in the chart). Also feeds the heartbeat (the brain is alive)."""
        if self._interp is None:
            return
        try:
            p = json.loads(msg.data)
        except Exception:
            return
        self._interp.queue(Event("evolve", traits=p.get("traits") or {},
                                 registry=p.get("registry") or {}))
        self._last_brain = time.monotonic()
        if self._brain_lost:
            self._brain_lost = False
            self.get_logger().info("cognitive layer reachable again")

    def _personality_events(self, now, picked):
        """Queue the fast rule-nudges + the heartbeat revert (both as Sismic events, applied
        on this tick's execute)."""
        if picked and not self._was_picked:        # being handled -> warier, less playful
            self._interp.queue(Event("evolve", registry={}, traits={
                "caution": self._nudge_pickup_caution,
                "playfulness": self._nudge_pickup_playful}))
        self._was_picked = picked
        if (self.enrich_enable and not self._brain_lost
                and (now - self._last_brain) > self.brain_timeout):
            self._interp.queue(Event("brain_lost"))
            self._brain_lost = True
            self.get_logger().warning(
                "cognitive layer unreachable — reverting to baseline personality")

    def _personality_publish(self, now):
        """Publish (latched) the current personality on change; persist it, throttled."""
        ctx = self._interp.context
        snap = (dict(ctx["traits"]), copy.deepcopy(ctx["registry"]))
        if snap != self._pers_last_pub:
            self.traits_pub.publish(String(data=json.dumps(
                {"traits": snap[0], "registry": snap[1]})))
            self._pers_last_pub = snap
            self._pers_dirty = True
        if self._pers_dirty and (now - self._pers_last_save) > 15.0:
            self._save_personality()
            self._pers_dirty = False
            self._pers_last_save = now

    def _save_personality(self):
        try:
            ctx = self._interp.context
            self.personality["traits"] = dict(ctx["traits"])
            self.personality["registry"] = copy.deepcopy(ctx["registry"])
            path = self._personality_file()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.personality, f, indent=2, ensure_ascii=False)
            os.replace(tmp, path)
        except Exception as exc:
            self.get_logger().warning(f"could not persist personality ({exc})")

    # --- purpose engine + planner: load / reflect / reward / meditate --------
    def _state_path(self, param, default_name):
        p = self.get_parameter(param).value
        return p or os.path.expanduser(f"~/.local/state/nanobot/{default_name}")

    def _purpose_file(self):
        return self._state_path("purpose_path", "purpose.json")

    def _experiments_file(self):
        return self._state_path("experiments_path", "experiments.json")

    def _cog_log_file(self):
        return self._state_path("cognition_log_path", "cognition.log")

    def _load_json(self, path):
        try:
            with open(path, encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except Exception as exc:
            self.get_logger().warning(f"could not read {path} ({exc})")
            return None

    def _save_json(self, path, obj):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(obj, f, indent=2, ensure_ascii=False)
            os.replace(tmp, path)
        except Exception as exc:
            self.get_logger().warning(f"could not persist {path} ({exc})")

    def _read_cog_log(self, n=40):
        """Tail the last n JSON lines of the shared decision log (the Purpose Engine's
        experience input). Best-effort: [] if absent/unreadable."""
        try:
            with open(self._cog_log_file(), encoding="utf-8") as f:
                lines = f.readlines()[-n:]
        except Exception:
            return []
        out = []
        for ln in lines:
            ln = ln.strip()
            if ln:
                try:
                    out.append(json.loads(ln))
                except Exception:
                    pass
        return out

    def _run_purpose_reflection(self, force=False):
        """Reflect on recent experience (the shared log) + traits -> drift the intrinsic
        reward weights + keep the planner's objective in sync. Deterministic, local."""
        if self._planner is None or self._interp is None:
            return
        exp = summarize_experience(self._read_cog_log())
        traits = dict(self._interp.context["traits"])
        new, changed = reflect_purpose(self._purpose, exp, traits)
        self._purpose = new
        self._planner.set_objective(new["objective"]["id"])
        if changed or force:
            self._publish_purpose()
            self._save_json(self._purpose_file(), self._purpose)

    def _publish_purpose(self):
        self.purpose_pub.publish(String(data=json.dumps(self._purpose)))

    def _publish_experiments(self):
        if self._planner is not None:
            self.experiments_pub.publish(String(data=json.dumps(self._planner.summary())))

    def _save_experiments(self):
        if self._planner is not None:
            self._save_json(self._experiments_file(), self._planner.to_state())

    def _on_reward(self, msg: String):
        """A human reward from the web UI: credit the A/B arm that produced the narrated line
        (contextual). Global reward shapes the intrinsic-reward weights via the decision log on
        the next purpose reflection (web_control logs every reward there)."""
        if self._planner is None:
            return
        try:
            r = json.loads(msg.data)
        except Exception:
            return
        if str(r.get("scope")) != "global":
            target = r.get("target") if isinstance(r.get("target"), dict) else None
            if self._planner.on_reward(r.get("value", 0), target):
                self._save_experiments()
                self._publish_experiments()

    def _on_meditate(self, msg: Bool):
        """Toggle meditation/consolidation. While on, the chart shows a calm face and pauses
        beats; we consolidate the local brain (reflect on purpose + finalize A/B winners).
        The LLM reflection + phrase-bank regen run on the web_control side."""
        on = bool(msg.data)
        if on == self._meditating:
            return
        self._meditating = on
        if self._interp is None:
            return
        if on:
            self._interp.queue(Event("meditate"))
            self.get_logger().info("meditating — consolidating brain (beats paused)")
            self._run_purpose_reflection(force=True)
            self._finalize_experiments()
        else:
            self._interp.queue(Event("wake"))
            self.get_logger().info("meditation ended — resuming presence")

    def _finalize_experiments(self):
        if self._planner is None:
            return
        summ = self._planner.summary()
        winners = ", ".join(f"{eid}->{e['winner']}"
                            for eid, e in summ["experiments"].items())
        self.get_logger().info(f"A/B winners: {winners or '(none)'}")
        self._save_experiments()
        self._publish_experiments()

    def _purpose_tick(self, now):
        """Slow loop, driven off the chart tick: reflect periodically (faster while
        meditating) and republish the latched topics on a heartbeat for late subscribers."""
        if self._purpose_enable and self._planner is not None:
            period = 30.0 if self._meditating else self._purpose_period
            if (now - self._purpose_last) >= period:
                self._purpose_last = now
                self._run_purpose_reflection()
        if (now - self._pub_hb) >= 5.0:             # heartbeat republish (cheap, small msgs)
            self._pub_hb = now
            self._publish_purpose()
            self._publish_experiments()

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

    # --- the periodic statechart step ----------------------------------------
    def _tick(self):
        now = time.monotonic()
        self._clock.time = now - self._t0

        picked = self._susp_l and self._susp_r
        manual = self._external_active
        speaking = self._speaking and (now - self._last_word_t) < self.speak_timeout
        active = (now - self._last_motion) < self.motion_hold
        stand = picked or manual or speaking or active

        try:
            # Meditation is a deliberate, sticky mode (only `wake` leaves it), so we skip the
            # normal stand-down/resume arbitration while consolidating.
            dormant = "dormant" in self._interp.configuration
            if not self._meditating:
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
            self._personality_events(now, picked)      # fast rules + heartbeat
            self._interp.execute()
            self._personality_publish(now)             # publish + persist on change
            self._purpose_tick(now)                    # purpose reflection + latched republish
        except Exception as exc:
            # The behaviour layer must never take the process down; log + keep going.
            self.get_logger().error(f"statechart step failed: {exc}",
                                    throttle_duration_sec=10.0)

    def destroy_node(self):
        if self._interp is not None:               # persist the latest drift on clean exit
            self._save_personality()
        if self._planner is not None:              # persist purpose + A/B state
            self._save_json(self._purpose_file(), self._purpose)
            self._save_experiments()
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
