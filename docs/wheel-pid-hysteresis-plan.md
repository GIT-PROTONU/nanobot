# Wheel-PID adaptive-filter N hysteresis — implementation plan (2026-09-21)

Held for later use. Residual drive roughness after the 2026-09-21 smoothness
pass II / III / IV: "robot still doesn't move smoothly, but it's better than
this morning" (user). The one legitimate gap found in an external review of
the PID block is **hysteresis on the adaptive velocity-filter window count
`N`** — everything else the review proposed is already implemented or would
regress verified tuning (see "Candidate fallbacks" at the bottom).

Status: **implemented 2026-09-22 (all 15 code steps + doc follow-ups) + flashed +
A/B'd on the live robot.** Result: **inconclusive — keep id 7 = 0.15** (compiled
default, NVS-persisted). Crawl 0.05 aggregate p2p 0.0216 (OFF) vs 0.0242 (ON);
boundary 0.055 0.0335 (OFF) vs 0.0285 (ON); 0.12 0.0218 (OFF) vs 0.0290 (ON);
spin ±0.8 0.018–0.022 (ON only). Differences are within run-to-run noise and the
worst legs on BOTH sides are DEADMAN freezes (web-gateway input-delivery stalls,
unrelated to filter length — cf. candidate fallback #4). No regression, marginal
boundary benefit; residual roughness is dominated by the input-delivery problem,
not filter-N flicker.

## Background

The adaptive velocity filter (`WHEEL_VEL_FILT_MAX 8` / `WHEEL_VEL_QUANT 0.25`,
main.cpp ~161) re-picks the averaged window count `N` EVERY tick from the
slewed setpoint:

```c
float want = WHEEL_VEL_QUANT * fabsf(l_tgt_s);
if (want > 1e-5f) nl = (uint8_t)ceilf(qstep / want);   // qstep = 1/(dt*g_tpm)
```

`N` has **no hysteresis**: if a held speed sits exactly at an `N`-threshold
boundary, tiny jitter can flip `N` back and forth (e.g. 4↔3), which changes
the filter's effective lag/phase tick-to-tag. The mitigation that keeps this
rare in practice is that `l_tgt_s` is SLEWED (smooth), so `N` transitions are
monotonic during ramps and constant at steady speed — but a held speed near a
boundary (e.g. the 0.05↔0.06 m/s crawl edge where N transitions 4→3) can
still flicker. Fix: a Schmitt band around each threshold.

## Design

Schmitt on the natural boundary. `n_raw = ceil(qstep/(QUANT·|tgt|))`; the
boundary between `n=k` and `n=k+1` sits at `|tgt| = qstep/(k·QUANT)`.
Hysteresis `H = 0.15` (±15%):

- switch **up** (`raw > cur`): only when `|tgt| < qstep/(cur·QUANT) · (1−H)`
- switch **down** (`raw < cur`): only when `|tgt| > qstep/((cur−1)·QUANT) · (1+H)`
- else keep `cur`
- `|tgt| ≤ 1e-5` or `qstep ≤ 0`: return `cur` (preserves today's zero-target behavior)
- raw clamped to `[2, WHEEL_VEL_FILT_MAX]`
- `H = 0` reduces EXACTLY to the current raw-ceil behavior (the A/B baseline)

Multi-step tracking still converges (2→3→4 fires over consecutive ticks —
hysteresis delays single-step flicker, it does not block ramps). Max `H`
clamp 0.5: bands never overlap, so `N` cannot deadlock.

**Live-tunable via `/motor_params` id 7** (NVS key `vhyst`, default 0.15,
range 0..0.5), mirroring the id-6 dither pattern end-to-end — so it can be
A/B'd on the live robot (`pid_tune.py params --set 7=0` vs `7=0.15`)
without reflashing. Readback rides `/wheel_params` id 7.

## Implementation — firmware (`firmware/nanobot_coprocessor/src/main.cpp`)

1. **New `#define`** after `WHEEL_DITHER_FADE` (~line 204):
   ```c
   #define WHEEL_VEL_HYST 0.15f   // Schmitt band (±15%) around each adaptive-filter N
                                  // threshold — stops N flickering when |setpoint| hovers
                                  // at a boundary. LIVE via /motor_params id 7 (0..0.5; 0=off)
   ```
2. **Global** — extend the `g_dither` declaration (~line 416):
   `... g_dither = WHEEL_DITHER, g_vhyst = WHEEL_VEL_HYST;`
3. **id-list comment** (~408–409): add `7 vel_hyst`.
4. **`set_param`** (~436): add `case 7: g_vhyst = clampf(v, 0.0f, 0.5f); break;`
5. **`hystN()` helper** after `slewTo()` (~971) — uses live `g_vhyst`:
   ```c
   #if WHEEL_PID_ENABLED
   static uint8_t hystN(uint8_t cur, float qstep, float tgtAbs){
     if (tgtAbs <= 1e-5f || qstep <= 0.0f) return cur;
     uint8_t raw = (uint8_t)ceilf(qstep / (WHEEL_VEL_QUANT * tgtAbs));
     if (raw < 2) raw = 2;
     if (raw > WHEEL_VEL_FILT_MAX) raw = WHEEL_VEL_FILT_MAX;
     if (raw == cur) return cur;
     float h = g_vhyst;
     if (raw > cur){
       float B = qstep / ((float)cur * WHEEL_VEL_QUANT);        // cur->cur+1 boundary
       if (tgtAbs < B * (1.0f - h)) return raw;
     } else {
       float B = qstep / ((float)(cur - 1) * WHEEL_VEL_QUANT);  // cur-1->cur boundary
       if (tgtAbs > B * (1.0f + h)) return raw;
     }
     return cur;
   }
   #endif
   ```
   (Down-branch at `cur==2` never fires — raw is clamped ≥2.)
6. **PID-block statics** (~1237–1242): add `static uint8_t nl_h=2, nr_h=2;`
   (the held hysteresis state, per wheel).
7. **Replace the `N` selection** (~1267–1274):
   ```c
   uint8_t nl = hystN(nl_h, qstep, fabsf(l_tgt_s));
   uint8_t nr = hystN(nr_h, qstep, fabsf(r_tgt_s));
   nl_h = nl; nr_h = nr;
   ```
   (delete the old `uint8_t nl=2, nr=2; if (qstep>0){…}` block; `qstep` stays).
8. **Jump guard** (~1257, after `vridx=0; vrcnt=0;`): add `nl_h=2; nr_h=2;`
9. **Direction-flip resets** (~1296–1299, after each `memset(vring_*…)`):
   add `nl_h=2;` / `nr_h=2;` — stale held-N must not survive a ring invalidation.
10. **Readback** (~940): `float pv[14]` → `float pv[16]`; after the id-6 pair add
    `pv[k++]=7; pv[k++]=g_vhyst;`.
11. **setup() NVS load** (~1125): add
    `set_param(7, g_prefs.getFloat("vhyst", WHEEL_VEL_HYST));`
12. **NVS save** (~1552): add `g_prefs.putFloat("vhyst", g_vhyst);`; append to the
    printf (~1554): `... slew=%.2f dith=%.2f vhyst=%.2f saved to NVS` + arg.

Buffer note: `/motor_params` cb's `uint8_t b[80]` holds exactly 8 pairs
(16 + 8·8 = 80) — no bump needed; the SBC whitelist is the pair cap.

## Implementation — SBC (`src/web_control/web_control/telemetry.py`)

13. Comment at ~246 (the `/motor_params` whitelist): add `7 vel_hyst` to the id list.
14. `_on_wheel_params` (~788–791): `d[:14]` → `d[:16]`; comment gains id 7.
15. `_mk_motor_params` (~796–804): "max 7 pairs" → "8 pairs"; `len(v) > 14` →
    `len(v) > 16`; `0 <= pid <= 6` → `0 <= pid <= 7`; update the error text.

No web-card change required — `f.esp.wheel_params` carries the new pair
automatically once the slice widens; drive it via
`pixi run python scripts/pid_tune.py --host http://<board>:8080 params --set 7=0.15`.
`pid_tune.py` has no id cap of its own (the gate is `_mk_motor_params`).
(UPDATE 2026-09-22, post-plan: the Coprocessor card now ALSO has a "Vel hyst"
slider — publishes `[7, v]` on release, re-seeds from `f.esp.wheel_params` id 7 on
first frame. Script-only remains valid for the other ids.)

## Verification (after flash + stack bounce)

Recorded baselines to beat (2026-09-21 pm II, dither OFF, gains 5/60/0):
crawl 0.05 m/s p2p **0.007**; 0.12 m/s p2p 0.018–0.037; spins stiction-bound.

1. `pio run -t upload` from the dev PC → expect the post-flash router desync →
   `sudo -n systemctl restart nano-robot.target` (forces the clean ESP re-handshake).
2. `pid_tune.py state`: `/wheel_params` carries id 7 = 0.15; ids 3/4 = 0.4/0.8;
   `/wheel_pid` = [5, 60, 0] (NVS-drift gotcha — confirm before judging anything).
3. **A/B**: `params --set 7=0` (hysteresis off = baseline) vs `7=0.15`, each with
   `pid_tune.py outback` (crawl + 0.12 + spin legs). Judge the AGGREGATE p2p.
4. The discriminating probe: hold a speed near an N boundary (0.05↔0.06 m/s)
   ~10 s and compare p2p — this is where hysteresis should show if flicker
   was a real contributor. Optional: throttled `Serial.printf` of `nl/nr` on
   change (USB console) to confirm N holds steady; remove after verifying.
5. Regression: fwd→rev no-lunge + clean-breakaway checks unchanged
   (hysteresis touches only the filter length, not the flip/reset logic).
6. Leave the winner in NVS (rate-limited while-parked save persists it).

## Candidate fallbacks (held in reserve — do NOT apply as a blind first move)

From the same external review; each was analyzed against the code and the
verified tuning history:

- **#1 — lower `WHEEL_TGT_SLEW` to 0.3–0.5 / inter-packet setpoint
  interpolation:** ALREADY in place — the 50 Hz `slewTo()` on
  `l_tgt_s`/`r_tgt_s` (main.cpp ~1328) IS the inter-packet interpolation; the
  slewed setpoint is C0-continuous and cannot staircase. Lowering the slew
  only adds start lag and regresses the verified "instant clean breakaway."
  Apply only if a measured setpoint discontinuity shows up.
- **#2 — lower `K_I` to 15–20 / raise `K_FF`:** contradicts the live
  `pid_tune.py` sweep (KI 30 stretched breakaway >1 s with no amplitude
  reduction; KI 60 won the aggregate). The "quantization spike" framing
  assumes RAW single-window velocity, but the PID already consumes the
  adaptive-filtered `g_left_vel` (step 0.042→0.010 m/s at crawl), and the
  stiction hammer is already rate-capped (`WHEEL_I_WIND_RATE`). `K_FF` is
  derived from measured geometry — raising it unmeasured overshoots at cruise.
- **#4 — wider deadband on the direction-flip reset:** the flip keys off the
  COMMANDED target (not measured speed) and already preserves the last
  direction at zero (`: l_dir_seen`, main.cpp ~1294), so measured-velocity
  sign noise cannot trigger it. A 0.02 m/s commanded deadband guards against
  `/cmd_vel` jitter but costs the flip-reset on sub-crawl reverses (the
  forward-wound integrator then drives the wrong way until friction stalls —
  the lurch the reset exists to prevent). Apply only if commanded-target
  jitter near zero is actually observed.

## Doc follow-ups (when implemented)

- AGENTS.md "Drivetrain geometry is LIVE-TUNABLE" block: add
  `7 vel_hyst (0..0.5)` to the id list; the `_mk_motor_params` "max 6 pairs"
  note → "max 8 pairs".
- The `_on_wheel_params`/`_mk_motor_params` comments carry the id list too.
