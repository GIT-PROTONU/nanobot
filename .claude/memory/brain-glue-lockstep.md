---
name: brain-glue-lockstep
description: "deploy.sh does NOT push nanobot-brain to the board — a stale brain silently breaks glue nodes at runtime (TypeError unexpected kwarg). Always sync the brain repo to /home/ibster/Nano/brain/src before restarting."
metadata:
  node_type: memory
  type: project
  originSessionId: 6f2c21a4-0778-4f6e-9469-9e2f8c1b4c47
---

# Brain/glue must stay in lockstep (two-repo deploy gotcha)

**Incident (2026-08-10):** after a normal `scripts/deploy.sh`, the web UI "stayed
disconnected". The app hub was up but `web_control` and `behavior` never registered.
Root cause: `deploy.sh` pushes **only this repo's `src/` + `scripts/`** — the brain
lives in the **separate `nanobot-brain` repo**, a standalone git checkout copied to the
board at `/home/ibster/Nano/brain/src` (added to `PYTHONPATH` there, not pip-installed).

The board's brain was stale, so the glue nodes constructed against an OLD brain API:
```
[app_hub] WebServerNode init failed: CognitionCore.__init__() got an unexpected keyword argument 'vision_diary_enable'
[app_hub] MoodNode init failed: Personality.__init__() got an unexpected keyword argument 'nudge_looming_caution'
[app_hub] hosting 1 nodes: oled_display
```
Because `app_hub` swallows per-node constructor exceptions and the HTTP daemon thread
starts *before* `telemetry` is initialized, the page partially served but `/telemetry`
+ `/brain/health` threw `AttributeError: … no attribute 'telemetry'/'_behavior_health'`.

## The fix (sync the brain to the board)

```bash
cd ~/Desktop/nanobot-brain
rsync -az --exclude '__pycache__' --exclude '.pytest_cache' --exclude '.git' \
  src/ nano:/home/ibster/Nano/brain/src/   # NOTE the src/ -> brain/src mapping
```

**Gotcha:** rsync `src/` into `brain/` lands at `brain/nanobot_brain/…` (wrong). The
destination must be `brain/src/` so it lands at `brain/src/nanobot_brain/…`.

Then cleanly restart: `stack.sh down` → verify port 8080 free → `stack.sh up`
(`restart` can leave a stale process on 8080).

## Related: presence chart is a second API to keep in sync

`nanobot_brain.behavior.build_interpreter` is called by BOTH `mood_node.py` (robot) and
`scripts/dev_webui.py` (dev harness) with `chart_path/beats/tempo/ambient_mood/
beat_boosts`. The brainsplit extraction lost those args once; they were restored
(commit `64c9174`). Glue (this repo) and brain (nanobot-brain repo) must agree — run the
brain test suite before/after touching either:

```bash
cd ~/Desktop/nanobot-brain
PYTHONPATH=src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/   # 94 pass
```

## Prevention checklist when the web UI or brain is down
1. `journalctl -u nano-app` for `init failed: … unexpected keyword argument` →
   stale brain → sync `nanobot-brain/src/` to board `brain/src/`.
2. `curl localhost:8080/brain/health` → `overall.all_healthy` — if app down,
   check for the same mismatch.
3. Verify the served page is the merged commit (self-contained SSE), not a stale
   split-file state — see [[single-webui-from-sbc]].
