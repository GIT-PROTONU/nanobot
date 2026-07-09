---
name: skill-offline-fallback
description: "Autonomous skill picking can itself need the LLM even when the picked action doesn't; fixed with a local action-only fallback pick + a new expand-offline meta skill that grows that pool"
metadata: 
  node_type: memory
  type: project
  originSessionId: e64522b3-b606-4100-8ae6-0e8fb694193c
---

**Gotcha found + fixed 2026-07-09**: `run_skill_beat`'s autonomous picker asked
`llm.complete()` to choose which skill fits the moment. When the LLM was unavailable/
rate-limited that call returns `None` -> `"no-pick"` -> **nothing ran at all**, even a pure
`topic` (action) skill like `blink-led`/`cool-down` that needs zero model calls to actually
execute. On-demand invocation (`POST /skills/invoke`) was never affected — `_do_topic_skill`
never touched the LLM; only the autonomous beat's *selection step* did.

**Why this matters beyond the one fix:** the bug wasn't "this action needs the LLM", it was
"the pipeline that reaches the action needs the LLM, even though the action itself doesn't."
Any future autonomous-selection mechanism (a new beat type, a new meta skill, a new picker)
can hit the same trap — don't assume an LLM-free action is reachable just because it's
LLM-free; check whether the *selection* step in front of it is too.

**Fix**: when `llm.available()` is False, `run_skill_beat` now falls back to a plain random
pick among the currently offered `topic`-kind skills instead of calling the model picker (and
the likes-bias short-circuit is restricted to that same pool while the LLM is down).

**Also added**: a 4th meta skill, `expand-offline` (`skills/expand-offline.md`,
`action.kind: offline`, `CognitionCore.expand_offline_skills`), alongside `workshop`/
`phrases`. It reuses the workshop's propose→check→rehearse→critique→trial pipeline but
forces the proposal to be a pure `topic` capability (discarding non-conforming replies) —
so it uses the LLM *now*, while available, specifically to grow the pool the fallback above
draws on *later*, when it isn't. Full architecture lives in CLAUDE.md's skill-workshop
section (this memory is just the gotcha + why, not a duplicate of the docs). See
[[skill-library]] (not yet written) for the general skill-catalogue architecture if that
gets captured later.
