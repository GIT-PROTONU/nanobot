// Nano ESP32-WROOM coprocessor — NATIVE ZENOH (zenoh-pico), no micro-ROS, no DDS.
// Talks straight to the SBC's rmw_zenoh graph over a direct UART link, in rmw_zenoh's
// exact wire format (Humble / libzenohc 1.9.0). Replaced the old micro-ROS firmware,
// keeping the same topic contract:
//
//   sub  cmd_vel               geometry_msgs/Twist       -> diff-drive -> H-bridge PWM
//   sub  led                   std_msgs/Bool             -> onboard LED
//   sub  lds_target_rpm        std_msgs/Float32          -> LDS spin-speed PID setpoint
//   sub  fan_pwm               std_msgs/Float32 (0..1)   -> SBC cooling fan PWM duty
//   sub  motor_trim            std_msgs/Float32 (-0.3..0.3) -> manual L/R trim set/reset
//   sub  motor_pid             std_msgs/Float32MultiArray [kp,ki,kd] -> LIVE wheel-PID gains
//                                                         (duty units; persisted to NVS)
//   sub  motor_accel           std_msgs/Float32 (0.3..8.0)  -> accel-ramp rate (duty/s)
//                                                         (open-loop build only; compiled
//                                                         out under the wheel PID)
//   sub  reset_ticks           std_msgs/Bool (true)      -> zero wheel_ticks + wheel_stray_ticks
//   sub  laser_pwm             std_msgs/Int32MultiArray  [v1,v2] 0..255 -> line laser PWM 1-2
//   pub  wheel_ticks           std_msgs/Int64MultiArray  [L,R] raw cumulative counts
//   pub  wheel_pid             std_msgs/Float32MultiArray [kp,ki,kd] live wheel-PID gains
//                                                         (1 Hz readback for the web sliders)
//   pub  wheel_stray_ticks     std_msgs/Int64MultiArray  [L,R] cumulative ticks seen while the
//                                                         wheel was commanded+settled stopped
//                                                         (bad-encoder-signal diagnostic)
//   pub  wheel_trim            std_msgs/Float32          active straight-line trim (1 Hz)
//   pub  left/right_wheel_suspended std_msgs/Bool        per-wheel off-ground switch
//   pub  esp32_temp            std_msgs/Float32          die temperature (C)
//   pub  esp32_hall            std_msgs/Int32            internal hall sensor
//   pub  lds_rpm / lds_hz / lds_duty  std_msgs/Float32   spin-lidar speed / framerate / PID duty
//   pub  esp32_heartbeat       std_msgs/Int32            link-alive counter
//
// LINK: ESP32 UART2 (TX=GPIO17, RX=GPIO16) <-> SBC UART1 (/dev/ttyS1). The serial-
// capable zenohd LISTENs there + on TCP for the rest of the rmw_zenoh stack.
//
// MULTICORE: zenohTask (publishes + sub callbacks) is pinned to Core 0. zenoh-pico's read
// + lease tasks use plain xTaskCreate (no affinity) so they float, but run at HIGH priority
// (configMAX_PRIORITIES/2 = 12) — far above the Arduino loop() (loopTask = prio 1). Core 1
// runs loop() = REAL-TIME CONTROL (motors, cmd watchdog, LDS UART1 read + PID, sensors).
//
// Why the SBC zenoh link ALWAYS wins over the LDS:
// (1) the zenoh read/lease tasks (prio 12) preempt the LDS/control loop (prio 1)
// wherever they're scheduled; (2) the UART2 RX ISR lives on Core 0 (z_open runs here) while
// the UART1/LDS RX ISR lives on Core 1 (Serial1.begin runs in setup on Core 1), so the two
// UARTs never contend for the same core's interrupt time. (Pinning the zenoh tasks onto
// Core 0 was tried and REVERTED: it starved the prio-5 zenohTask's publisher declarations,
// so the board connected but never announced its topics.)
//
// State is shared via volatiles (32-bit aligned reads/writes are atomic on the ESP32).
// Only Core 0 ever touches the zenoh session — concurrent serial writes corrupt frames.
//
// Three hard-won zenoh-pico notes (see README): use the "serial/UART_2" device locator
// (the pin form skips the link handshake), the begin()-explicit-pins patch, and multi-
// thread mode (Z_FEATURE_MULTI_THREAD=1) — the blocking serial RX needs its own read
// task while the lease task + our publishes do TX (serialized by Z_FEATURE_BATCH_TX_MUTEX).
#include <Arduino.h>
#include <zenoh-pico.h>
#include <esp_system.h>   // esp_restart() — link-connect watchdog (see LINK_CONNECT_DEADLINE_MS)
#include <driver/gpio.h>  // gpio_set_pull_mode() — float the LDS RX pin (shared line, see setup())
#include <Preferences.h>  // NVS — persists the straight-line wheel trim across reboots
#include <string.h>
#include <math.h>

// ============================ pin / tunable config ============================
// Reassigned vs the micro-ROS build: GPIO16/17 are now the zenoh UART2 link, so the
// LDS data RX moved off 16 (briefly to 35, now GPIO14 via the UART1 matrix — see
// LDS_RX_PIN below).
// DRV8871 x2 (one per motor), verified against the actual harness 2026-07-04:
// LEFT motor = GPIO 26+27, RIGHT motor = GPIO 25+33. fwd/rev within each pair is a
// best guess — if a wheel runs backwards, flip that side's INVERT_* below.
#define LEFT_IN_FWD   26
#define LEFT_IN_REV   27
#define RIGHT_IN_FWD  25
#define RIGHT_IN_REV  33
#define LEFT_ENC      19
#define RIGHT_ENC      5
#define LEFT_SUSPEND_PIN   4
#define RIGHT_SUSPEND_PIN 21
#define SUSPEND_ACTIVE_HIGH true   // switch reads HIGH (INPUT_PULLUP) while the wheel is OFF the
                                   // ground (lifted); on the ground (switch closed) = pin pulled LOW.
                                   // Flipped 2026-07-16 so "suspended" now means lifted (up), not
                                   // down.
#define LED_PIN        2
// LDS data link = UART1 (Serial1). UART1's default pins (9/10) are the SPI flash, but the
// peripheral routes through the GPIO matrix, so RX is remapped to GPIO14 (TX=GPIO13 stays
// free — the LDS02RR only streams, we never transmit to it). 25/4 were rejected: they're
// the left-motor PWM. UART2 stays the SBC zenoh link, UART0 the debug console.
#define LDS_RX_PIN    14      // UART1 RX (was 35)
#define LDS_MOTOR_PIN 18

// SBC cooling fan PWM. Driven by sys_monitor's /fan_pwm (duty 0..1 from the SBC CPU
// temperature; web UI can override). GPIO22 is free here (it's the default I2C SCL, but
// this firmware uses no I2C). The ESP can't source fan current — drive the fan through a
// logic-level MOSFET/transistor gated by this pin. CONFIRM the pin against your wiring.
#define FAN_PIN       22
// Fan is parked (0 duty) whenever the SBC link isn't alive — boot race (before sys_monitor
// takes over /fan_pwm, ~30-60 s into SBC bring-up), a dropped link, or the SBC genuinely
// powered off — same "track true SBC presence" gating as the LDS spin motor below. There's
// no heat to move once the SBC isn't running, so the fan shouldn't run either. g_fan_duty
// just holds the last /fan_pwm value; only applied to hardware while alive.
#define FAN_BOOT_DUTY 0.0f

// Line laser PWM pins (laser 3 was removed — GPIO13 stayed stuck full-on via every PWM
// peripheral tried: LEDC low-speed ch 8 stuck the pin high silently, and MCPWM couldn't
// sink it either, so the laser was controlled by hardware, not /laser_pwm. Dropped from
// firmware + web UI on 2026-08-18; leave GPIO13 free so the pin can't be driven).
#define LASER1_PIN    23
#define LASER2_PIN    32

#define PWM_FREQ_HZ   20000
#define PWM_RES_BITS  10
static const uint32_t PWM_MAX = (1u << PWM_RES_BITS) - 1u;

#define WHEEL_SEPARATION  0.102f  // MEASURED 2026-09-21 (lidar cross-correlation): the old
                                  // 0.16 was a chassis-width guess — canned 90° turns
                                  // physically rotated 139-146°. This define is only the
                                  // NVS-absent fallback (erased flash / fresh ESP32); the
                                  // live value comes from NVS (/motor_params id 2) and
                                  // robot.yaml wheel_odometry.wheel_separation matches.
#define MAX_LINEAR_SPEED  0.4f
// Synced with robot.yaml web_control.drive_max_ang (was 3.0): the SLAM rotation-smear
// budget caps rotation at 0.8 rad/s (a scan spans ~0.2 s of lidar revolution, so a turn
// at w smears it by w*0.2 rad). The SBC clamps first; this is the firmware backstop.
// Keep the two in sync (AGENTS.md).
#define MAX_ANGULAR_SPEED 0.8f
#define CMD_TIMEOUT_MS    500
// Grace period after a wheel's duty drops to 0 before its ticks count as "stray" (see
// g_left_stray/g_right_stray) -- real wheel inertia keeps ticking briefly after power
// cuts, that's not encoder noise and shouldn't be flagged as such.
#define STRAY_SETTLE_MS   300
#define INVERT_LEFT  false
#define INVERT_RIGHT true   // 2026-07-15: right motor's fwd/rev harness pins were swapped
// |duty| below this = intended stop, not a crawl (shared by both control paths below).
#define MOTOR_DEADZONE 0.02f

// ---- closed-loop wheel velocity PID (THE motor control path since 2026-09-20) --
// Holds each wheel's commanded linear speed (m/s) via a per-wheel feedforward+PI(D) on
// encoder-tick velocity, replacing the open-loop duty map: the I-term integrates through
// stiction, so crawl speeds hold deterministically instead of riding on the stiction
// remap + breakaway push (both compiled out with this path — see the !WHEEL_PID_ENABLED
// block below). This is the standard ROS 2 control shape: /cmd_vel is a SETPOINT refresh
// (any cadence up to the CMD_TIMEOUT_MS dead-man) and this fixed-rate loop owns the
// dynamics deterministically, regardless of SBC load.
// Feedback is single-channel (ticks signed by COMMANDED direction): blind on
// reverse-through-zero / stall / slip / being pushed — accepted 2026-09-20, and
// PERMANENT (2026-09-21, user-decided): there is no 2nd quadrature channel and
// there never will be; the mitigations in this block are the final design.
// Tuning (hardware, watch the debug console's vel/tgt line): raise KI until a crawl
// breaks away in <0.5 s without stick-slip hunting (halve KI if it oscillates), then KP
// for stiffness (~0.5*KFF to start); KD stays 0 (tick quantization noise at 50 Hz makes
// D jittery). KP=KI=0 is feedforward only — it will NOT crawl (the stiction remap is
// gone), so never flash this path with both zero. The adaptive velocity filter +
// WHEEL_I_WIND_RATE (both above) already damp the two classic stutter sources —
// feedback-quantization chatter and the I-term breakaway hammer.
#define WHEEL_PID_ENABLED 1
#define WHEEL_RADIUS      0.0335f   // m  (matches robot.yaml wheel_odometry.wheel_radius)
#define TICKS_PER_REV     253.0f    // counts/wheel-rev as the ESP emits them (matches odom).
                                    // MEASURED 2026-09-20 rollout: 2235 ticks / 1.86 m = 1202
                                    // ticks/m => 253/rev (was 1440 — a 5.7x error that made
                                    // /odom + the PID's velocity world fictional; drive was
                                    // healthy all along, full duty = ~0.37 m/s loaded).
#define WHEEL_PID_HZ      50        // PID rate; longer window than the 100 Hz loop => less tick-quantization noise
// Feedback smoothing, ADAPTIVE (2026-09-21 smoothness pass II): per-window tick deltas go
// into a ring buffer and the velocity is the average of the last N windows, where N is
// chosen EACH TICK so the 1-tick quantization step (1/(N*dt) m/s) stays under
// WHEEL_VEL_QUANT of the moving setpoint — fast command -> short filter (lag matters),
// crawl -> long filter (the single-window step is 0.042 m/s (1202 ticks/m), 83% of a
// 0.05 m/s crawl, and even the fixed 2-window average left 0.021 m/s of step that kp=5
// turned into ±0.1 duty chatter = the felt stutter; N=4 at 0.05 m/s crushes it to
// 0.010 m/s). Wheels in a spin see per-wheel targets of only ±w*sep/2, so they
// automatically get the longest filter — that is what makes turns smooth too.
// Never below 2 windows (the raw single window was the pre-2026-09-21 behaviour).
#define WHEEL_VEL_FILT_MAX 8       // ring capacity = max averaged windows (8 * 20 ms = 160 ms)
#define WHEEL_VEL_QUANT   0.25f    // keep the quantization step <= this fraction of |setpoint|
// Full-scale wheel speed at duty=1 (the per-wheel max from the clamps above); KFF maps a
// target m/s to the baseline duty the I-term corrects from.
#define WHEEL_KFF   (1.0f/(MAX_LINEAR_SPEED + MAX_ANGULAR_SPEED*WHEEL_SEPARATION*0.5f))
#define WHEEL_KP    0.0f            // stiffness — tune after KI (start ~0.5*KFF)
#define WHEEL_KI    8.0f            // crawl breakaway <0.5 s (0.12 m/s stall -> +0.7 duty in ~0.46 s)
#define WHEEL_KD    0.0f
#define WHEEL_INTEG_MAX 1.0f        // anti-windup: integral clamp (duty units via KI)
// Stiction-aware I-term (2026-09-21 smoothness pass II): while a wheel is COMMANDED but
// badly lagging its setpoint (WHEEL_STUCK_FRAC), the tuned ki=60 turns a 0.05 m/s crawl
// error into 60*0.05 = 3 duty/s of I-term push — a hammer that breaks the wheel away
// violently, overshoots (nothing re-loads static friction as gently as a slow push) and
// re-sticks: the stick-slip limit cycle. Capping how fast the I-CONTRIBUTION (ki*integ,
// duty units) may move while stuck turns breakaway into a firm ~1 s ramp; integration is
// UNRESTRICTED while the wheel is tracking or braking (stuck is false there), so
// stopping and normal control are untouched.
#define WHEEL_STUCK_FRAC  0.30f     // |meas| < frac*|tgt| while commanded = fighting stiction
#define WHEEL_I_WIND_RATE 2.5f      // duty/s cap on the I-term ramp while stuck (2026-09-21:
                                    // 1.2 measured too slow to recover spin-band stick-slip —
                                    // turn SAG 0.036 -> 0.029 m/s mean; 2.5 keeps the ramp but
                                    // halves the recovery lag. Re-verify spins after flash.)
// Stiction DITHER (2026-09-21 smoothness pass II — the plant-level anti-stick-slip fix).
// Traced on hardware 2026-09-21 (ticktrace at 0.05 + 0.12 m/s): steady cruise dips 12 -> 7
// ticks/frame in 0.4-0.6 s clusters every ~1 s, both wheels together — the wheel SEIZES
// between I-term surges (carpet static friction re-engages) and KI 30 vs 60 changed
// nothing: it is the plant, not the gains. A small ALTERNATING duty keeps the gear mesh
// micro-moving so static friction never fully re-engages — the classic servo dither.
// Output-side only (added to the PID result, the integrator never sees it), gated:
// fades in above WHEEL_DITHER_IN of setpoint, out by WHEEL_DITHER_FADE, zero when parked
// or cmd-stale. Toggled every WHEEL_DITHER_TICKS PID windows (~12.5 Hz at 50 Hz PID).
#define WHEEL_DITHER       0.05f    // duty amplitude (0 = off); live via /motor_params id 6
#define WHEEL_DITHER_IN    0.02f    // m/s: full dither once |setpoint| passes this
#define WHEEL_DITHER_FADE  0.15f    // m/s: fade out by here (higher speeds track smoothly)
#define WHEEL_DITHER_TICKS 2
// Command shaping: the per-wheel SETPOINT is slewed (m/s per s) before the PID — smooths
// accel without lagging the loop (slewing the PID OUTPUT would add loop lag + integral
// windup). ~the old open-loop 3.0 duty/s feel at the 0.464 m/s full scale.
#define WHEEL_TGT_SLEW 1.5f
static const float TICKS_PER_METER = TICKS_PER_REV / (2.0f*3.14159265f*WHEEL_RADIUS);

#if !WHEEL_PID_ENABLED
// ---- legacy OPEN-LOOP path (compiled out under the wheel PID) ------------------
// Stiction deadband compensation: the gearmotors don't move below ~60% duty (only a
// full-scale 0.4 m/s command — duty 0.63 after the v+w normalization — moved at all, and
// in-place turns at 0.37 duty didn't). Remap any command above MOTOR_DEADZONE from
// (0..1] to [MOTOR_MIN_DUTY..1] so slow speeds and tank turns still overcome friction.
// 0.55 was measured (2026-09-19 drive test) to still re-stick at crawl: wheels jerked,
// ran 1.5-2.4 s at constant ~0.60-0.69 remapped duty, then seized mid-command (the
// documented "low-duty turn stall"), on BOTH crawl speeds (0.12 m/s) and in-place
// spins (0.5 rad/s). 0.70 + the breakaway push below keeps them turning; the cost is
// coarser low-speed duty resolution ([0.70..1] spans 0..0.4 m/s).
#define MOTOR_MIN_DUTY 0.70f
// Breakaway push: at crawl the wheel breaks away on the initial ramp, crawls ~1.5-2 s,
// then static friction re-seizes it at constant duty (it only restarts on a direction/
// phase change). 2026-09-20 hardware re-test on carpet showed the previous 80 ms full-
// duty kick PULSES judder without reliably breaking away (1-2 s of stall-kick-stall
// before motion) — a pulse doesn't sustain enough torque to exceed static friction, and
// each re-kick restarts from zero. Now: a SUSTAINED full-duty push whenever a wheel is
// powered but hasn't ticked for MOTOR_PUSH_RECHECK — held until ticks resume (smooth
// handoff back to the ramped duty), capped at MOTOR_PUSH_MAX_MS; a push that expires
// without breakaway backs off before the next attempt (backoff doubles per failure up
// to MOTOR_PUSH_MAX_BACKOFF, so a hard jam nudges occasionally instead of ramming).
// There is no separate start kick: a fresh start from stop IS a frozen wheel (ramp
// stalled at low duty), so the same detector catches it — on easy floor the wheel ticks
// immediately and no push ever fires. Keyed off the RAMPED duty (same as the stray
// gating below); the push bypasses the slew ramp (it IS the shove) but trim still
// applies, so it stays direction-correct.
#define MOTOR_PUSH_RECHECK    250   // powered-but-frozen this long -> start a push (also the post-failure backoff)
#define MOTOR_PUSH_MAX_MS     700   // cap one continuous full-duty push
#define MOTOR_PUSH_MAX_BACKOFF 1600 // capped doubling of the recheck after failed pushes
// Acceleration control: duty (a step function of the latest /cmd_vel) used to be applied
// to the motors instantly, so any joystick flick or direction reversal was a hard jolt.
// MOTOR_SLEW_DEFAULT caps how fast the APPLIED duty may follow the commanded duty (duty
// units/sec; duty spans -1..1, so 3.0 crosses the full range in ~0.67s) — a simple ramp,
// not a closed-loop accel controller. Live-tunable via /motor_accel (Float32, see
// motor_slew_cb below / the web UI's Coprocessor card "Accel ramp" slider), clamped to
// [MOTOR_SLEW_MIN..MOTOR_SLEW_MAX]; NOT persisted, so a reboot/reflash always starts from
// this safe default. Lower = gentler/laggier, higher = snappier/jerkier. Bypassed on a
// stale cmd (CMD_TIMEOUT_MS) so the dead-man stop still cuts power immediately instead of
// coasting down a ramp.
#define MOTOR_SLEW_DEFAULT 3.0f
#define MOTOR_SLEW_MIN     0.3f
#define MOTOR_SLEW_MAX     8.0f
#endif

// ---- straight-line trim (motor matching) --------------------------------------
// The two gearmotors don't run the same speed at the same duty, so open-loop straight
// commands arc. A single trim factor rebalances the sides in applyMotors():
//   left *= (1 - trim), right *= (1 + trim)   -> POSITIVE trim = robot was pulling RIGHT
// TRIM_AUTOCAL learns it from the encoders: whenever a straight drive is commanded
// (equal duties, cmd fresh, wheels on the ground, enough ticks in the window), the
// relative L/R tick-rate imbalance is folded into the trim a little each window, so a
// few seconds of driving forward converges it — that IS the calibration procedure.
// The result persists in NVS (survives reboot AND reflash). Manual path: publish
// std_msgs/Float32 on /motor_trim to set it directly (0 = reset); current value is
// republished on /wheel_trim at 1 Hz and in the status line below.
// (Autocal is compiled out under WHEEL_PID_ENABLED — a velocity PID equalizes the
// wheels itself; the manual /motor_trim offset still applies and the loop absorbs it.)
#define TRIM_AUTOCAL    1   // re-enabled 2026-09-17: with SUSPEND_ACTIVE_HIGH true (verified),
                            // the gate below means "both wheels on the ground" — the 2026-07-16
                            // wrong-way convergence ran under the old inverted-polarity gate,
                            // which only passed while the robot was LIFTED (free-spinning wheels).
                            // The loop math itself is sound negative feedback. If it still
                            // drifts the wrong way on hardware, set 0 + TRIM_DEFAULT and tune
                            // via /motor_trim.
#define TRIM_DEFAULT    -0.10f  // starting straight-line offset; NEGATIVE = boost left / cut right,
                                // counteracting a leftward veer. Tune live with POST /motor_trim.
#define TRIM_MAX        0.30f   // |trim| clamp — beyond this something is broken, not unmatched
#define TRIM_CAL_HZ     5       // adaptation windows/s (200 ms of ticks each)
#define TRIM_CAL_GAIN   0.08f   // fraction of the measured imbalance folded in per window
#define TRIM_ERR_CLAMP  0.5f    // per-window |imbalance| cap (limits spin-up transients)
#define TRIM_MIN_TICKS  40      // per-wheel ticks/window below this = too slow/stalled, skip
#define TRIM_MATCH_TOL  0.02f   // commanded duties must match within this = "straight"
#define TRIM_SAVE_MS    10000   // NVS write rate limit (flash wear)
#define TRIM_SAVE_DELTA 0.005f  // ...and only if it moved at least this much

#define LDS_BAUD       115200
#define LDS_TIMEOUT_MS 300
#define LDS_TARGET_RPM 300.0f
#define LDS_PID_HZ     50
#define LDS_PID_KFF    0.0020f
#define LDS_PID_KP     0.0010f
#define LDS_PID_KI     0.0015f
#define LDS_PID_KD     0.0f

// Jam guard (2026-09-21): a physically blocked rotor — string wrapped around the
// turret, debris — can't reach speed, so the PID pins at full duty trying to reach
// the target and the motor cooks. If the measured rpm stays under
// LDS_JAM_FRAC*target (or UART1 is stale, i.e. we can't see the tach at all) for
// LDS_JAM_MS continuously while a target is set, latch a jam and cut the PWM. The
// latch clears ONLY on target<=0 (the SBC idle controller's park after its idle
// timeout, or the web slider's 0) — so a continuously jammed motor is simply OFF
// (no periodic grind), each wake-from-idle retries at most once, and the web Lidar
// card shows the JAM state via /lds_jam. Spin-up from rest typically reaches
// 40% of target well inside 6 s, so a healthy motor never trips this; a dead/
// disconnected LDS (no UART1 frames at all) also latches — that is deliberate:
// driving the motor blind is the same overheat risk.
#define LDS_JAM_MS     6000
#define LDS_JAM_FRAC   0.40f
#define LDS_RPM_MAX    400.0f  // /lds_target_rpm clamp (mirrors web_control's LDS_RPM_MAX)

// LDS spin-lidar. We only want the current RPM to close the spin PID, so UART1 is drained
// once per PID tick (not every loop) — see loop(). Enabling adds a 2nd active UART; if the
// zenoh link (UART2) turns flaky under load, set back to 0 (all code stays compiled out).
#define LDS_ENABLED  1

// Periodic one-line health summary on the debug console (UART0 — separate from the zenoh
// UART2 link). Lets you watch the LDS + control stay live under load; 0 disables it.
#define STATUS_PRINT_MS 3000

// Low-power mode while the SBC link is down (boot race, SBC off, or a drop): downclock the
// CPU instead of idling at full speed. 80 MHz is still PLL-locked, so the APB bus (and thus
// UART baud timing for both the zenoh link and the LDS RX) stays accurate — only the core
// clock drops, so this is safe to flip live with no re-init. 0 disables (stays at 240 MHz).
#define CPU_MHZ_NORMAL    240
#define CPU_MHZ_LOWPOWER   80

// Link-connect watchdog. The ESP boots in ~1 s but the SBC takes ~30-60 s to bring up the
// serial zenohd. If the ESP boots first, its repeated failed serial handshakes leave the
// link in a state that an in-process z_open() retry won't re-sync — historically the only
// cure was a manual ESP power-cycle (a fresh boot sends a clean InitSyn the now-listening
// router accepts). So: if we haven't reached `ready` within this deadline of boot, reboot
// ourselves. A reboot == the manual power-cycle, and (running on Core 1) it also recovers a
// z_open() that wedged on Core 0. Tunable: shorter = faster auto-recovery once the SBC is up,
// but more wasted reboots while the SBC is still booting. 0 disables the watchdog.
#define LINK_CONNECT_DEADLINE_MS 40000

// Runtime link-liveness watchdog. The connect watchdog above only fires while UNconnected;
// it can't catch the router (zenohd) restarting AFTER a good connect — over a raw UART the
// session never notices the peer vanished (our writes just succeed into the void), so we'd
// keep publishing to nobody until a manual reset. Fix: the always-on SBC web_control node
// publishes /esp32_ping at 1 Hz; we subscribe, and if we're `ready` but no ping has arrived
// for this long, esp_restart() to re-handshake. FAILS SAFE: the timer only arms after the
// FIRST ping is seen, so if pings never come (topic mismatch / feature off) we never reboot
// from here. 0 disables. Keep > the 1 Hz ping period with margin.
#define LINK_RX_TIMEOUT_MS 8000

// First-ping deadline. The runtime watchdog above deliberately fails safe by arming only
// after the first ping — but that leaves one permanent wedge (hit 2026-07-04): z_open()
// succeeds against a router that dies before the first ping ever arrives (e.g. an SBC
// power-cycle races the connect), leaving `ready` true with `g_ping_seen` false forever.
// Neither watchdog can fire and the ESP sits silent until a manual reset. So: if we're
// `ready` but have never seen a ping within this deadline of the (re)connect, reboot —
// capped at LINK_FIRST_PING_MAX_REBOOTS consecutive SW reboots (counter in RTC noinit
// RAM, cleared on any ping and on non-SW resets) so a robot that legitimately never
// pings (feature off / topic mismatch) still can't boot-loop. 0 disables.
#define LINK_FIRST_PING_DEADLINE_MS 90000
#define LINK_FIRST_PING_MAX_REBOOTS 5

#define CH_LEFT_FWD  0
#define CH_LEFT_REV  1
#define CH_RIGHT_FWD 2
#define CH_RIGHT_REV 3
#define CH_LDS       4
#define CH_FAN       5
#define CH_LASER1    6
#define CH_LASER2    7
// no LASER3 pin/channel: laser 3 was removed (see LASER1_PIN note)

// ============================ shared cross-core state =========================
static volatile int32_t  g_left_ticks  = 0, g_right_ticks = 0;   // encoder ISR counts (signed)
// Single-channel encoders carry NO direction, so the ISR can't know forward/reverse.
// We sign each tick by the last commanded wheel direction (set in cmd_cb) — the best
// proxy available; an int8 so the ISR never touches the FPU (float math in an ESP32
// ISR is unsafe). Without this, /odom integrates every move as forward and SLAM breaks
// on reverse. Near-zero command holds the previous sign.
static volatile int8_t   g_left_dir = 1, g_right_dir = 1;
static volatile float    g_left_duty   = 0, g_right_duty   = 0;  // cmd -> motor duty
static volatile float    g_left_tgt    = 0, g_right_tgt    = 0;  // per-wheel target speed (m/s), PID input
static volatile float    g_left_vel    = 0, g_right_vel    = 0;  // measured wheel speed (m/s), debug/tuning
static volatile uint32_t g_last_cmd_ms = 0;
static volatile float    g_lds_rpm = 0, g_lds_duty = 0, g_lds_hz = 0;
static volatile uint32_t g_lds_frames = 0, g_lds_last_ms = 0;
static volatile float    g_lds_target = LDS_TARGET_RPM;
static volatile bool     g_lds_tgt_save = false; // cb -> 1 Hz block: target changed, NVS write pending
static volatile bool     g_lds_jam = false;   // jam guard latched (Core1 ldsControl, Core0 publishes)
static volatile float    g_fan_duty = FAN_BOOT_DUTY;   // /fan_pwm 0..1 (Core0 write, Core1 apply)
static volatile uint16_t g_laser[2] = {0,0};                 // /laser_pwm 0..255 per laser (Core0 write, Core1 apply)
// Straight-line trim: loaded from NVS in setup(), adapted on Core 1 (autocal), manually
// set from the zenoh RX task (/motor_trim cb). Aligned-32-bit volatile = atomic enough.
static volatile float    g_trim = 0;
#if WHEEL_PID_ENABLED
// Live wheel-PID gains (defaults = the WHEEL_KP/KI/KD defines; NVS-loaded in setup()).
// Changed at runtime via /motor_pid (Float32MultiArray [kp,ki,kd], duty units) — tune
// without reflashing — and persisted to NVS rate-limited like the trim, so a tuning
// session survives a reboot. Readback on /wheel_pid @1 Hz feeds the web sliders.
// Clamps mirror motor_pid_cb (kp 0..20, ki 0..100, kd 0..5).
static volatile float    g_wkp = WHEEL_KP, g_wki = WHEEL_KI, g_wkd = WHEEL_KD;
static volatile bool     g_pid_reset = false; // cb -> PID block: reset integ/prev on a gain change
static volatile bool     g_pid_save = false; // cb -> 1 Hz block: gains changed, NVS write pending
static float             g_pid_saved[3];      // last values written to NVS (Core 1 only)
#endif

// ---- Live drivetrain parameters (NVS-backed, settable via /motor_params — NO reflash
// to recalibrate the geometry). Wire format: Float32MultiArray, empty layout, data =
// (id,value) float pairs, ids:
//   0 ticks_per_rev   1 wheel_radius_m   2 wheel_separation_m
//   3 max_linear_ms   4 max_angular_rads 5 target_slew_mps2
// Readback on /wheel_params @1 Hz uses the SAME (id,value) layout. A change recomputes
// the derived ticks/meter + KFF full-scale map and resets the PID integrators (their
// error units just changed meaning). Defaults are the defines above; both build paths
// share cmd_cb's clamps/diff-drive, so this block is unconditional.
static volatile float    g_tpr = TICKS_PER_REV, g_wrad = WHEEL_RADIUS, g_wsep = WHEEL_SEPARATION,
                         g_maxlin = MAX_LINEAR_SPEED, g_maxang = MAX_ANGULAR_SPEED, g_slew = WHEEL_TGT_SLEW,
                         g_dither = WHEEL_DITHER;
static volatile float    g_tpm = TICKS_PER_REV/(2.0f*3.14159265f*WHEEL_RADIUS);   // derived: ticks/meter
static volatile float    g_kff = WHEEL_KFF;                                       // derived: duty per m/s
static volatile bool     g_par_save = false; // cb -> 1 Hz block: params changed, NVS write pending
static uint32_t          g_par_saved_ms = 0;
static float clampf(float v, float lo, float hi);   // defined below (shared helper)
static void recalc_drive_params(){
  float tpr = g_tpr, r = g_wrad;
  if (tpr >= 1.0f && r >= 0.001f)                      // never adopt a broken scale
    g_tpm = tpr / (2.0f*3.14159265f*r);
  g_kff = 1.0f / (g_maxlin + g_maxang*g_wsep*0.5f);
}
static bool set_param(int id, float v){   // clamp + assign; false = unknown id
  switch(id){
    case 0: g_tpr   = clampf(v, 10,    5000); break;
    case 1: g_wrad  = clampf(v, 0.005f, 0.5f); break;
    case 2: g_wsep  = clampf(v, 0.05f,  1.0f); break;
    case 3: g_maxlin= clampf(v, 0.05f,  2.0f); break;
    case 4: g_maxang= clampf(v, 0.05f,  5.0f); break;
    case 5: g_slew  = clampf(v, 0.05f, 10.0f); break;
    case 6: g_dither= clampf(v, 0.0f,   0.20f); break;
    default: return false;
  }
  return true;
}
#if !WHEEL_PID_ENABLED
static volatile float    g_motor_slew = MOTOR_SLEW_DEFAULT;   // live /motor_accel setpoint
#endif
static Preferences       g_prefs;          // NVS handle (namespace "nano", key "trim")
static float             g_trim_saved = 0; // last value written to NVS (Core 1 only)
static volatile float    g_temp = 0;
static volatile int32_t  g_hall = 0;
static volatile int32_t  g_reset_reason = 0;   // esp_reset_reason() latched at boot, pub @1 Hz
                                               // (1=poweron 2=ext 3=sw 4=panic 5=int_wdt
                                               // 6=task_wdt 9=brownout — remote drop triage)
static volatile bool     g_susp_l = false, g_susp_r = false;
static volatile bool     g_led = false, g_led_dirty = false;
static volatile uint32_t g_last_ping_ms = 0;   // last /esp32_ping rx (runtime liveness watchdog)
static volatile bool     g_ping_seen = false;  // arm the runtime watchdog only after 1st ping
#if LINK_RX_TIMEOUT_MS && LINK_FIRST_PING_DEADLINE_MS
// Survives esp_restart() (undefined at power-on — setup() clears it on non-SW resets).
RTC_NOINIT_ATTR static uint32_t g_fping_reboots;
#endif

// Bad-encoder-signal diagnostic: a tick that lands while a wheel is commanded (and has
// settled) stopped can only be electrical noise/ground-bounce on that GPIO, never real
// rotation — count those separately so the SBC can flag a wheel with a flaky encoder
// signal instead of silently corrupting /odom. `g_left_stopped`/`g_right_stopped` are
// set on Core 1 (real-time control loop, see STRAY_SETTLE_MS below) and only READ here;
// a plain bool check keeps the ISR FPU-free like the direction signing above.
static volatile int32_t  g_left_stray = 0, g_right_stray = 0;    // cumulative, never auto-cleared
static volatile bool     g_left_stopped = true, g_right_stopped = true;

static void IRAM_ATTR leftEncISR()  { g_left_ticks  += g_left_dir;  if (g_left_stopped)  g_left_stray++;  }
static void IRAM_ATTR rightEncISR() { g_right_ticks += g_right_dir; if (g_right_stopped) g_right_stray++; }

static inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

// ============================ CDR encoders (rmw wire) =========================
// rmw_zenoh payload = 4-byte CDR-LE encapsulation header + body. Alignment counts from
// buffer start (header included). xtensa is little-endian so memcpy gives LE.
static const uint8_t CDR_HDR[4] = {0x00, 0x01, 0x00, 0x00};

static size_t cdr_i32(uint8_t* b, int32_t v){ memcpy(b,CDR_HDR,4); memcpy(b+4,&v,4); return 8; }
static size_t cdr_f32(uint8_t* b, float v)  { memcpy(b,CDR_HDR,4); memcpy(b+4,&v,4); return 8; }
static size_t cdr_bool(uint8_t* b, bool v)  { memcpy(b,CDR_HDR,4); b[4]=v?1:0; return 5; }
// Int64MultiArray [a,b], empty layout: hdr | dim_len=0 | data_offset=0 | data_len=2 |
// PAD(4) | a | b. CDR aligns from the BODY start (after the 4-byte header): the int64
// data lands at body offset 16, i.e. buffer offset 20, so 4 pad bytes are required.
static size_t cdr_i64arr2(uint8_t* b, int64_t a, int64_t bb){
  memcpy(b,CDR_HDR,4);
  uint32_t z=0,two=2; memcpy(b+4,&z,4); memcpy(b+8,&z,4); memcpy(b+12,&two,4);
  memset(b+16,0,4);                        // pad to 8-align int64 from body start
  memcpy(b+20,&a,8); memcpy(b+28,&bb,8);
  return 36;
}
// Float32MultiArray [a,b,c], empty layout: hdr | dim_len=0 | data_offset=0 | data_len=3 |
// a | b | c. float32 is 4-aligned from the body start, so no pad bytes (unlike int64).
static size_t cdr_f32arr3(uint8_t* b, float a, float c, float d){
  memcpy(b,CDR_HDR,4);
  uint32_t z=0,three=3; memcpy(b+4,&z,4); memcpy(b+8,&z,4); memcpy(b+12,&three,4);
  memcpy(b+16,&a,4); memcpy(b+20,&c,4); memcpy(b+24,&d,4);
  return 28;
}
// Float32MultiArray with n floats (empty layout) — the /wheel_params readback's
// (id,value) pairs. Caller guarantees n*4 + 16 <= buffer size.
static size_t cdr_f32arr_n(uint8_t* b, const float* v, int n){
  memcpy(b,CDR_HDR,4);
  uint32_t z=0, ln=(uint32_t)n; memcpy(b+4,&z,4); memcpy(b+8,&z,4); memcpy(b+12,&ln,4);
  memcpy(b+16,v,4*n);
  return 16+4*n;
}

// ============================ zenoh session (Core 0) ==========================
#define DOMAIN "0"
#define KE(topic, type) DOMAIN "/" topic "/" type "/TypeHashNotSupported"
#define T_I32  "std_msgs::msg::dds_::Int32_"
#define T_F32  "std_msgs::msg::dds_::Float32_"
#define T_BOOL "std_msgs::msg::dds_::Bool_"
#define T_I64A "std_msgs::msg::dds_::Int64MultiArray_"
#define T_I32A "std_msgs::msg::dds_::Int32MultiArray_"
#define T_F32A "std_msgs::msg::dds_::Float32MultiArray_"
#define T_TWIST "geometry_msgs::msg::dds_::Twist_"

// Fixed session ZID so we can hardcode it in the rmw_zenoh liveliness tokens below.
// Without those tokens the ESP32 isn't a known graph participant and rmw_zenoh
// subscribers (rosbridge/web) only receive its data intermittently.
// Palindromic, all-nonzero (no leading-zero trimming, byte-order-agnostic) so the zid
// string is identical however zenoh formats it.
#define NODE_ZID "e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5"
#define NODE_NAME "nano_esp32"

static z_owned_session_t s;
static volatile bool ready = false;       // written Core 0 (zenohTask), read Core 1 (loop watchdog)
static volatile uint32_t g_boot_ms = 0;   // millis() at boot — link-connect watchdog reference

// `ready` only means z_open() succeeded LOCALLY — opening a UART peripheral needs no peer,
// so it goes true even with nothing wired to the other end (confirmed on the bench: it
// reaches "zenoh CONNECTED" with the link cable unplugged). The only real evidence the SBC
// is actually there is a received /esp32_ping (g_ping_seen — the runtime-liveness watchdog's
// signal, reset false on every (re)connect, set true only by ping_cb actually firing). Used
// to gate the LDS spin + CPU low-power mode below so they track true SBC presence, not just
// "the ESP tried". Falls back to bare `ready` if the ping watchdog is compiled out.
#if LINK_RX_TIMEOUT_MS
static inline bool linkAlive(){ return ready && g_ping_seen; }
#else
static inline bool linkAlive(){ return ready; }
#endif
// rmw_zenoh liveliness token = makes a publisher visible in the ROS graph. Format:
// @ros2_lv/<domain>/<zid>/<nid>/<eid>/MP/%/%/<node>/%<topic>/<type>/<typehash>/<qos>
static z_owned_liveliness_token_t g_lv[16]; static int g_lv_n = 0;
static void declare_lv(const char* topic, const char* type, int eid){
  char ke[260];
  snprintf(ke, sizeof(ke),
    "@ros2_lv/" DOMAIN "/" NODE_ZID "/0/%d/MP/%%/%%/" NODE_NAME "/%%%s/%s/TypeHashNotSupported/:1:,1:,:,:,,",
    eid, topic, type);
  z_view_keyexpr_t vke; z_view_keyexpr_from_str_unchecked(&vke, ke);
  if (g_lv_n >= (int)(sizeof(g_lv)/sizeof(g_lv[0]))) return;   // 2 spare slots today
  z_liveliness_declare_token(z_session_loan(&s), &g_lv[g_lv_n++], z_view_keyexpr_loan(&vke), NULL);
}

// one publisher + its rmw attachment identity
struct ZPub { z_owned_publisher_t p; int64_t seq; uint8_t gid[16]; };
static ZPub P_ticks, P_strayTicks, P_suspL, P_suspR, P_temp, P_hall, P_rpm, P_hz, P_duty, P_hb, P_trim, P_pid, P_params, P_jam, P_rst;

// Single source of truth for every publisher: topic/type, the attachment GID tag
// (last GID byte, unique per publisher) and the liveliness entity id (lv_eid). The
// declare loop and the liveliness loop both walk this, so the two can't drift. These
// wire identities are PROVEN-GOOD against the live graph — don't renumber existing
// entries; new ones take the next free tag/eid (g_lv[] currently has 2 spare slots
// after lds_jam).
struct PubDef { ZPub* zp; const char* topic; const char* type; uint8_t gid_tag; int lv_eid; bool lds_only; };
static const PubDef PUBS[] = {
  { &P_ticks, "wheel_ticks",           T_I64A, 1, 1, false },
  { &P_strayTicks, "wheel_stray_ticks", T_I64A, 11, 11, false },
  { &P_suspL, "left_wheel_suspended",  T_BOOL, 2, 2, false },
  { &P_suspR, "right_wheel_suspended", T_BOOL, 3, 3, false },
  { &P_temp,  "esp32_temp",            T_F32,  4, 4, false },
  { &P_rst,   "esp32_reset",           T_I32, 15, 15, false },
  { &P_hall,  "esp32_hall",            T_I32,  5, 5, false },
  { &P_hb,    "esp32_heartbeat",       T_I32,  9, 6, false },
  { &P_rpm,   "lds_rpm",               T_F32,  6, 7, true  },
  { &P_hz,    "lds_hz",                T_F32,  7, 8, true  },
  { &P_duty,  "lds_duty",              T_F32,  8, 9, true  },
  { &P_jam,   "lds_jam",               T_BOOL, 14, 14, true },
  { &P_trim,  "wheel_trim",            T_F32, 10, 10, false },
  { &P_pid,   "wheel_pid",             T_F32A, 12, 12, false },
  { &P_params,"wheel_params",          T_F32A, 13, 13, false },
};

static void zpub_declare(ZPub& zp, const char* topic, const char* type, uint8_t tag){
  char keyexpr[160];
  snprintf(keyexpr, sizeof(keyexpr), DOMAIN "/%s/%s/TypeHashNotSupported", topic, type);
  z_view_keyexpr_t ke; z_view_keyexpr_from_str_unchecked(&ke, keyexpr);
  // A failed declare is otherwise silent — "zenoh CONNECTED" would print while one
  // topic never appears in the graph. Log loudly; the ping watchdogs only catch a
  // TOTAL session failure, not one missing feed.
  if (z_declare_publisher(z_session_loan(&s), &zp.p, z_view_keyexpr_loan(&ke), NULL) < 0)
    Serial.printf("[nano] declare publisher FAILED: %s\n", topic);
  zp.seq = 0;
  static const uint8_t base[16] = {0x60,0x7c,0xc3,0x6d,0x07,0x32,0xd1,0x86,
                                   0xf5,0xb0,0x9b,0x47,0xb9,0xa6,0x22,0x00};
  memcpy(zp.gid, base, 16); zp.gid[15] = tag;   // unique gid per publisher
}
static void zpub_put(ZPub& zp, const uint8_t* pl, size_t len){
  uint8_t att[33];
  int64_t ts = (int64_t)esp_timer_get_time()*1000;
  int64_t sq = ++zp.seq;
  memcpy(att,&sq,8); memcpy(att+8,&ts,8); att[16]=0x10; memcpy(att+17,zp.gid,16);
  z_owned_bytes_t payload, attachment;
  z_bytes_copy_from_buf(&payload, pl, len);
  z_bytes_copy_from_buf(&attachment, att, sizeof(att));
  z_publisher_put_options_t o; z_publisher_put_options_default(&o);
  o.attachment = z_bytes_move(&attachment);
  z_publisher_put(z_publisher_loan(&zp.p), z_bytes_move(&payload), &o);
}

// read a sample's raw payload into a fixed buffer; returns length (0 on fail)
static size_t sample_bytes(const z_loaned_sample_t* sm, uint8_t* out, size_t cap){
  z_owned_slice_t sl;
  if (z_bytes_to_slice(z_sample_payload(sm), &sl) < 0) return 0;
  size_t n = z_slice_len(z_slice_loan(&sl));
  if (n > cap) n = cap;
  memcpy(out, z_slice_data(z_slice_loan(&sl)), n);
  z_slice_drop(z_slice_move(&sl));
  return n;
}

// --- subscription callbacks (run in the zenoh-pico read task: prio 12, floats cores) ---
static void cmd_cb(z_loaned_sample_t* sm, void*){
  // one-shot: report which core the read task is on (informational; it floats but at prio
  // 12 it always preempts the LDS/control loop). Prints when the first cmd_vel arrives.
  static bool core_printed=false;
  if (!core_printed){ core_printed=true; Serial.printf("[nano] zenoh rx task on core %d\n", xPortGetCoreID()); }
  uint8_t b[64]; size_t n = sample_bytes(sm, b, sizeof(b));
  if (n < 52) return;                       // hdr(4) + 6*f64(48); align from body start
  double v, w;
  memcpy(&v, b+4,  8);                       // linear.x  (body offset 0)
  memcpy(&w, b+44, 8);                       // angular.z (body offset 40)
  float fv = clampf((float)v, -g_maxlin,  g_maxlin);
  float fw = clampf((float)w, -g_maxang, g_maxang);
  float vl = fv - fw*g_wsep*0.5f, vr = fv + fw*g_wsep*0.5f;
#if WHEEL_PID_ENABLED
  g_left_tgt = vl; g_right_tgt = vr;            // the control loop's PID turns these into duty
#else
  static constexpr float mx = MAX_LINEAR_SPEED + MAX_ANGULAR_SPEED*WHEEL_SEPARATION*0.5f;
  g_left_duty  = clampf(vl/mx,-1,1);
  g_right_duty = clampf(vr/mx,-1,1);
#endif
  // sign the encoder ticks by commanded wheel direction (single-channel = no feedback)
  if (vl >  1e-4f) g_left_dir  =  1; else if (vl < -1e-4f) g_left_dir  = -1;
  if (vr >  1e-4f) g_right_dir =  1; else if (vr < -1e-4f) g_right_dir = -1;
  g_last_cmd_ms = millis();
}
static void led_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 5){ g_led = b[4]!=0; g_led_dirty = true; }
}
// /reset_ticks (Bool, true = reset): zeros the raw + stray counters for a clean baseline
// (bench calibration, or clearing a stray-tick count after fixing a wiring issue). The
// SBC side (wheel_odometry) also watches this topic to re-seed its own prev-tick state,
// else the next /odom integration step would see a huge fake jump from the old cumulative
// count down to 0.
static void reset_ticks_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 5 && b[4]!=0){
    g_left_ticks = g_right_ticks = 0;
    g_left_stray = g_right_stray = 0;
    Serial.println("[nano] wheel ticks reset");
  }
}
static void ldstgt_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8];
  if (sample_bytes(sm,b,sizeof(b)) >= 8){
    float f; memcpy(&f,b+4,4);
    if (isnan(f)) return;                    // clampf can't order NaN — reject outright
    float t = clampf(f,0,LDS_RPM_MAX);       // 0 = park; >LDS_RPM_MAX is a glitch, not a setpoint
    if (t != g_lds_target) g_lds_tgt_save = true;
    g_lds_target = t;
  }
}
static void fan_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 8){ float f; memcpy(&f,b+4,4); g_fan_duty = clampf(f,0,1); }
}
// /laser_pwm (Int32MultiArray [v1,v2], 0..255 per laser) -> line laser PWM 1-2. CDR body
// of an empty-layout std_msgs/Int32MultiArray: hdr(4) | dim_len=0 | data_offset=0 | data_len=2
// | int32 v1..v2. int32 is 4-aligned from the body start, so no pad bytes (unlike int64).
static void laser_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[40]; size_t n = sample_bytes(sm,b,sizeof(b));
  uint32_t dim=0, off=0;
  if (n >= 16){ memcpy(&dim,b+4,4); memcpy(&off,b+8,4); }
  if (dim != 0) return;                                  // empty layout only
  for (int i=0;i<2;i++){
    if (n >= 16+off+(i+1)*4){
      int32_t v; memcpy(&v,b+16+off+i*4,4);
      g_laser[i] = (uint16_t)(v<0?0:(v>255?255:v));
    }
  }
  Serial.printf("[nano] laser pwm %u %u\n",
                (unsigned)g_laser[0],(unsigned)g_laser[1]);
}
// /motor_trim (Float32): manual trim set/reset (0 clears). With TRIM_AUTOCAL on, the next
// straight drive re-adapts from here — so this is mainly a reset, or THE knob when autocal
// is compiled out. Persisted by the loop()'s rate-limited NVS save (within ~TRIM_SAVE_MS).
static void trim_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 8){
    float f; memcpy(&f,b+4,4);
    if (!isnan(f)){ g_trim = clampf(f,-TRIM_MAX,TRIM_MAX); Serial.printf("[nano] manual trim=%.3f\n", (double)g_trim); }
  }
}
#if WHEEL_PID_ENABLED
// /motor_pid (Float32MultiArray [kp, ki, kd], duty units): LIVE wheel-PID gains — tune
// without reflashing (the WHEEL_KP/KI/KD defines are just the defaults). Clamped; a change
// resets both integrators (windup accumulated under the old gains must not leak into the
// new tuning) and flags the rate-limited NVS save. Readback: /wheel_pid @1 Hz.
static void motor_pid_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[40]; size_t n = sample_bytes(sm,b,sizeof(b));
  uint32_t dim=0, off=0;
  if (n >= 16){ memcpy(&dim,b+4,4); memcpy(&off,b+8,4); }
  if (dim != 0 || n < 16+off+3*4) return;                // empty layout, 3 floats required
  float kp,ki,kd;
  memcpy(&kp,b+16+off,4); memcpy(&ki,b+20+off,4); memcpy(&kd,b+24+off,4);
  if (isnan(kp)||isnan(ki)||isnan(kd)) return;
  g_wkp = clampf(kp,0,20); g_wki = clampf(ki,0,100); g_wkd = clampf(kd,0,5);
  g_pid_reset = true; g_pid_save = true;
  Serial.printf("[nano] wheel PID kp=%.3f ki=%.3f kd=%.3f (integ reset)\n",
                (double)g_wkp,(double)g_wki,(double)g_wkd);
}

// /motor_params: live drivetrain parameters as (id,value) float pairs — see the
// g_tpr block above for ids/ranges. Any accepted change recomputes the derived
// ticks/meter + KFF and resets the PID integrators, then persists to NVS
// rate-limited like the gains (written once parked).
static void motor_params_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[80]; size_t n = sample_bytes(sm,b,sizeof(b));
  uint32_t dim=0, off=0;
  if (n >= 16){ memcpy(&dim,b+4,4); memcpy(&off,b+8,4); }
  if (dim != 0 || n < 16+off+8 || ((n-16-off) % 8)) return;   // empty layout, whole pairs
  int cnt = (int)(n-16-off)/8; bool changed=false;
  for (int i=0;i<cnt;i++){
    float id, v; memcpy(&id,b+16+off+8*i,4); memcpy(&v,b+20+off+8*i,4);
    if (isnan(id)||isnan(v)) continue;
    if (set_param((int)id, v)) changed=true;
  }
  if (!changed) return;
  recalc_drive_params();
  g_pid_reset = true; g_par_save = true;
  Serial.printf("[nano] params tpr=%.1f rad=%.4f sep=%.3f maxlin=%.2f maxang=%.2f slew=%.2f (tpm=%.1f kff=%.3f, integ reset)\n",
    (double)g_tpr,(double)g_wrad,(double)g_wsep,(double)g_maxlin,(double)g_maxang,(double)g_slew,
    (double)g_tpm,(double)g_kff);
}
#endif
#if !WHEEL_PID_ENABLED
// /motor_accel (Float32, duty/s): live acceleration-ramp rate — see MOTOR_SLEW_DEFAULT
// above / the web UI's Coprocessor card "Accel ramp" slider. Not persisted. (Open-loop
// build only: under the wheel PID, accel limiting is the WHEEL_TGT_SLEW setpoint slew.)
static void motor_slew_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 8){
    float f; memcpy(&f,b+4,4);
    if (!isnan(f)){
      g_motor_slew = clampf(f, MOTOR_SLEW_MIN, MOTOR_SLEW_MAX);
      Serial.printf("[nano] motor accel ramp=%.2f duty/s\n", (double)g_motor_slew);
    }
  }
}
#endif
#if LINK_RX_TIMEOUT_MS
// /esp32_ping (Int32) from the SBC web_control node — payload ignored; arrival = link alive.
static void ping_cb(z_loaned_sample_t*, void*){
  g_last_ping_ms = millis(); g_ping_seen = true;
#if LINK_FIRST_PING_DEADLINE_MS
  g_fping_reboots = 0;   // real pings flow — re-earn the full first-ping reboot budget
#endif
}
#endif

static bool zenohConnect();

// ---- subscriptions: one table, re-declarable (see SUB_REDECLARE_MS) ----------
// Subscriptions are the ONLY zenoh entities that must outlive a ROUTER restart on
// the SBC side: the ESP keeps publishing fine after a router bounce (its data path
// is self-sufficient), but the fresh router instance starts with an EMPTY remote-
// subscription table, and zenoh-pico only sends the declare once per session —
// z_declare_subscriber() here runs once at boot. Hit 2026-09-20 (open, docs/TODO.md):
// after a router restart the ESP re-attaches (heartbeat/ticks flow, /motor_pid
// write->readback flips) yet /cmd_vel specifically goes deaf — a subscriber the
// router never re-learned. Fix: periodically UNDECLARE + REDECLARE every
// subscription on the live session, so the router's table is refreshed in place
// (entity leak: the fresh declare supersedes; the old entity dies with the session).
// The 45 s period bounds the worst deaf window at 45 s, and a single re-declare
// burst (~10 small serial round-trips) is nothing against the 500 ms cmd watchdog.
// DISABLED 2026-09-21 pm: the live verification FAILED — after a full stack restart
// the ESP half-attached (hb/ticks flowed, /cmd_vel dead for 30+ min) and the burst
// re-declared every 45 s over the dead session WITHOUT healing it; worse, the two
// load-correlated ESP drops (2026-09-21 pm) landed ON redeclare moments. The
// reliable heal is the ping-watchdog esp_restart() (LINK_RX_TIMEOUT_MS) — a fully
// fresh session both ends. Set 0 = off; subsRedeclare() stays for manual reuse.
#define SUB_REDECLARE_MS 0
struct SubDef { const char* topic; const char* type; void (*cb)(z_loaned_sample_t*, void*); };
static const SubDef SUBS[] = {
  { "cmd_vel",       T_TWIST, cmd_cb },
  { "led",           T_BOOL,  led_cb },
  { "fan_pwm",       T_F32,   fan_cb },
  { "motor_trim",    T_F32,   trim_cb },
  { "reset_ticks",   T_BOOL,  reset_ticks_cb },
#if WHEEL_PID_ENABLED
  { "motor_pid",     T_F32A,  motor_pid_cb },
  { "motor_params",  T_F32A,  motor_params_cb },
#else
  { "motor_accel",   T_F32,   motor_slew_cb },
#endif
  { "laser_pwm",     T_I32A,  laser_cb },
#if LINK_RX_TIMEOUT_MS
  { "esp32_ping",    T_I32,   ping_cb },
#endif
#if LDS_ENABLED
  { "lds_target_rpm",T_F32,   ldstgt_cb },
#endif
};
static z_owned_subscriber_t g_subs[sizeof(SUBS) / sizeof(SUBS[0])];
static int g_subs_n = 0;

static void subDeclareAll(){
  g_subs_n = 0;
  for (auto& d : SUBS){
    char keyexpr[160];
    snprintf(keyexpr, sizeof(keyexpr), DOMAIN "/%s/%s/TypeHashNotSupported", d.topic, d.type);
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, keyexpr);
    z_owned_closure_sample_t cl;
    z_closure_sample(&cl, d.cb, NULL, NULL);
    // A failed declare is otherwise silent — one topic would never appear. Log loudly;
    // the ping watchdogs only catch a TOTAL session failure, not one missing feed.
    if (z_declare_subscriber(z_session_loan(&s), &g_subs[g_subs_n],
                             z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL) < 0)
      Serial.printf("[nano] declare subscriber FAILED: %s\n", d.topic);
    else
      g_subs_n++;
  }
}
static void subsRedeclare(){
  // ZENOH_C_STANDARD=99 (zenoh-pico's Arduino extra_script) compiles the z_move
  // macro out — use the explicit generated move (same convention as z_config_move).
  for (int i = 0; i < g_subs_n; i++)
    z_undeclare_subscriber(z_subscriber_move(&g_subs[i]));
  g_subs_n = 0;
  subDeclareAll();
  Serial.printf("[nano] subs re-declared (%d) — router table refreshed\n", g_subs_n);
}

static bool zenohConnect(){
  z_owned_config_t cfg; z_config_default(&cfg);
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_MODE_KEY, "client");
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_CONNECT_KEY, "serial/UART_2#baudrate=115200");
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_SESSION_ZID_KEY, NODE_ZID);  // fixed zid for liveliness
  if (z_open(&s, z_config_move(&cfg), NULL) < 0){ Serial.println("[nano] z_open failed"); return false; }
  // Dedicated tasks own the (blocking) serial RX + keepalive TX; our publishes are
  // TX-mutex-serialized against them.
  zp_start_read_task(z_session_loan_mut(&s), NULL);
  zp_start_lease_task(z_session_loan_mut(&s), NULL);

  for (auto& d : PUBS)
    if (!d.lds_only || LDS_ENABLED) zpub_declare(*d.zp, d.topic, d.type, d.gid_tag);

  subDeclareAll();
#if LINK_RX_TIMEOUT_MS
  g_last_ping_ms = millis(); g_ping_seen = false;   // (re)arm fresh on each (re)connect
#endif

  // Publisher liveliness tokens -> ESP32 shows up as a graph participant so rmw_zenoh
  // subscribers reliably receive its data. eid must be unique per entity.
  for (auto& d : PUBS)
    if (!d.lds_only || LDS_ENABLED) declare_lv(d.topic, d.type, d.lv_eid);

  Serial.println("[nano] zenoh CONNECTED");
  return true;
}

// Publishing runs here on Core 0. RX + keepalive are handled by the zenoh-pico read/
// lease tasks; we only PUT (TX-mutex-serialized against them), so nothing blocks.
static void zenohTask(void*){
  Serial.printf("[nano] zenoh task pinned to core %d\n", xPortGetCoreID());
  static uint32_t t_subdecl = 0;             // SUB_REDECLARE_MS cadence (router-table refresh)
  for(;;){
    if (!ready){
      ready = zenohConnect();
      if (!ready){ delay(1000); continue; }
      t_subdecl = millis();                  // first re-declare one full period after connect
    }

    static uint32_t t_ticks=0, t_lds=0, t_slow=0;
    uint32_t now = millis();

    // Periodic subscription re-declare (see SUB_REDECLARE_MS at the SUBS table):
    // the only known healing path for a router whose remote-sub table was wiped by
    // a restart while our session kept flowing. (DISABLED — see SUB_REDECLARE_MS.)
#if SUB_REDECLARE_MS
    if (t_subdecl && now - t_subdecl >= SUB_REDECLARE_MS){
      t_subdecl = now;
      subsRedeclare();
    }
#endif

    uint8_t buf[80];                                        // 64 needed by the /wheel_params readback
    if (now - t_ticks >= 66){                                // wheel_ticks @~15 Hz (was
                                                              // ~30 Hz; odom integrates
                                                              // cumulative counts, so the
                                                              // faster rate bought nothing
                                                              // but extra SBC executor wakeups)
      t_ticks = now;
      zpub_put(P_ticks, buf, cdr_i64arr2(buf,(int64_t)g_left_ticks,(int64_t)g_right_ticks));
      zpub_put(P_strayTicks, buf, cdr_i64arr2(buf,(int64_t)g_left_stray,(int64_t)g_right_stray));
    }
    // suspension: publish immediately on change (every ~2 ms loop), so the web UI
    // tracks a wheel lifting/dropping with no lag; the 1 Hz block below republishes
    // for late-joining subscribers.
    static bool pub_l=false, pub_r=false, susp_init=false;
    if (!susp_init || g_susp_l!=pub_l){ pub_l=g_susp_l; zpub_put(P_suspL,buf,cdr_bool(buf,pub_l)); }
    if (!susp_init || g_susp_r!=pub_r){ pub_r=g_susp_r; zpub_put(P_suspR,buf,cdr_bool(buf,pub_r)); }
    susp_init=true;
#if LDS_ENABLED
    if (now - t_lds >= 200){                                 // lds @5 Hz
      t_lds = now;
      bool stale = (now - g_lds_last_ms) > LDS_TIMEOUT_MS;
      zpub_put(P_rpm,  buf, cdr_f32(buf, stale?0.0f:g_lds_rpm));
      zpub_put(P_hz,   buf, cdr_f32(buf, g_lds_hz));
      zpub_put(P_duty, buf, cdr_f32(buf, g_lds_duty));
      zpub_put(P_jam,  buf, cdr_bool(buf, g_lds_jam));
    }
#else
    (void)t_lds;
#endif
    if (now - t_slow >= 1000){                               // temp/hall/heartbeat @1 Hz + suspension republish
      t_slow = now;
      static int32_t hb=0;
      zpub_put(P_temp, buf, cdr_f32(buf, g_temp));
      zpub_put(P_hall, buf, cdr_i32(buf, g_hall));
      zpub_put(P_hb,   buf, cdr_i32(buf, ++hb));
      zpub_put(P_rst,  buf, cdr_i32(buf, (int32_t)g_reset_reason));  // boot reason, 1 Hz —
                                     // readable remotely after ANY drop (brownout vs watchdog vs panic)
      zpub_put(P_suspL,buf, cdr_bool(buf, g_susp_l));
      zpub_put(P_suspR,buf, cdr_bool(buf, g_susp_r));
      zpub_put(P_trim, buf, cdr_f32(buf, g_trim));
#if WHEEL_PID_ENABLED
      zpub_put(P_pid,  buf, cdr_f32arr3(buf, g_wkp, g_wki, g_wkd));
      {   // /wheel_params readback: (id,value) pairs, same layout /motor_params writes
        float pv[14]; int k=0;
        pv[k++]=0; pv[k++]=g_tpr;   pv[k++]=1; pv[k++]=g_wrad;  pv[k++]=2; pv[k++]=g_wsep;
        pv[k++]=3; pv[k++]=g_maxlin; pv[k++]=4; pv[k++]=g_maxang; pv[k++]=5; pv[k++]=g_slew;
        pv[k++]=6; pv[k++]=g_dither;
        zpub_put(P_params, buf, cdr_f32arr_n(buf, pv, k));
      }
#endif
    }
    delay(2);
  }
}

// ============================ real-time control (Core 1) ======================
static void writeSide(int chf, int chr, float duty){
  duty = clampf(duty,-1,1);
  float m = fabsf(duty);
#if WHEEL_PID_ENABLED
  // Closed-loop path: the PID's I-term owns low-speed control (it integrates through
  // stiction), so the duty->PWM map stays linear — only an intended stop zeroes it.
  if (m < MOTOR_DEADZONE) m = 0.0f;
#else
  // Open-loop stiction remap (see MOTOR_MIN_DUTY above).
  m = (m < MOTOR_DEADZONE) ? 0.0f : MOTOR_MIN_DUTY + m*(1.0f - MOTOR_MIN_DUTY);
#endif
  if (duty>=0){ ledcWrite(chr,0); ledcWrite(chf,(uint32_t)(m*PWM_MAX)); }
  else        { ledcWrite(chf,0); ledcWrite(chr,(uint32_t)(m*PWM_MAX)); }
}
static inline float slewTo(float cur, float target, float maxDelta){
  float diff = target - cur;
  if (diff > maxDelta) diff = maxDelta; else if (diff < -maxDelta) diff = -maxDelta;
  return cur + diff;
}
static void applyMotors(float l, float r){
  // Straight-line trim: positive trim boosts RIGHT / cuts LEFT (robot was pulling right).
  // Open-loop: applied pre-remap so it stays monotonic through the stiction compensation.
  // PID: a static duty distortion the per-wheel loop simply absorbs (it still reaches the
  // commanded wheel speed). writeSide clamps, so a boosted side saturating just means the
  // cut side does the correcting.
  float t = clampf(g_trim, -TRIM_MAX, TRIM_MAX);
  l *= (1.0f - t); r *= (1.0f + t);
  writeSide(CH_LEFT_FWD, CH_LEFT_REV, INVERT_LEFT?-l:l);
  writeSide(CH_RIGHT_FWD,CH_RIGHT_REV,INVERT_RIGHT?-r:r);
}

// LDS02RR frame parser: extract RPM only (speed/64), checksum-validated.
static void ldsFeed(uint8_t byte){
  static uint8_t pkt[22]; static uint8_t len=0;
  if (len==0 && byte!=0xFA) return;
  pkt[len++]=byte; if (len<22) return; len=0;
  uint32_t chk=0; for(int i=0;i<20;i+=2) chk=(chk*2u+pkt[i]+(pkt[i+1]<<8))&0xFFFFFFFFu;
  uint32_t cs=((chk&0x7FFF)+(chk>>15))&0x7FFF;
  if ((cs&0xFF)==pkt[20] && ((cs>>8)&0xFF)==pkt[21]){
    g_lds_rpm = ((pkt[3]<<8)|pkt[2]) / 64.0f; g_lds_frames++; g_lds_last_ms = millis();
  }
}
static void ldsControl(float dt){
  static float integ=0, prev=0;
  static uint32_t stall_ms=0;   // start of the current continuous "can't reach speed" stretch
  static bool jam=false;        // latched: only target<=0 clears (no periodic grind)
  float target=g_lds_target;
  if (target<=0){
    integ=0; prev=0; g_lds_duty=0; ledcWrite(CH_LDS,0);
    if (jam){ jam=false; g_lds_jam=false; stall_ms=0;
              Serial.println("[nano] lds jam latch cleared (target 0)"); }
    return;
  }
  if (jam){ g_lds_duty=0; ledcWrite(CH_LDS,0); return; }  // parked until something sends target 0
  // Stalled = rpm far under target (threshold scales with the setpoint so a low
  // cruise target can't false-trip) OR no valid tach frames at all — a blocked
  // rotor and a dead UART1 both mean "driving blind", i.e. the same overheat risk.
  bool stale = (millis()-g_lds_last_ms) > LDS_TIMEOUT_MS;
  bool stalled = stale || (g_lds_rpm < LDS_JAM_FRAC*target);
  if (!stalled) stall_ms=0;
  else if (stall_ms==0) stall_ms=millis();
  else if (millis()-stall_ms > LDS_JAM_MS){
    jam=true; g_lds_jam=true; g_lds_duty=0; ledcWrite(CH_LDS,0);
    Serial.printf("[nano] LDS JAM: rpm %.0f (stale=%d) vs target %.0f for %u ms — motor parked, clears on target 0\n",
                  (double)g_lds_rpm, (int)stale, (double)target, (unsigned)LDS_JAM_MS);
    return;
  }
  float ff=LDS_PID_KFF*target, duty;
  if (millis()-g_lds_last_ms > LDS_TIMEOUT_MS){ integ=0; prev=0; duty=clampf(ff,0,1); }
  else {
    float err=target-g_lds_rpm, deriv=dt>0?(err-prev)/dt:0; prev=err;
    float u=ff+LDS_PID_KP*err+LDS_PID_KI*integ+LDS_PID_KD*deriv; duty=clampf(u,0,1);
    if (duty==u) integ+=err*dt;
  }
  g_lds_duty=duty; ledcWrite(CH_LDS,(uint32_t)(duty*PWM_MAX));
}
#if WHEEL_PID_ENABLED
// Per-wheel velocity PID: feedforward + PI(+D) with conditional integration + clamp.
// Gains are the LIVE g_wkp/g_wki/g_wkd (defaults = the defines; see motor_pid_cb).
// `stuck` (caller-computed, see WHEEL_STUCK_FRAC) rate-limits the I-term's duty-slew
// while the wheel fights stiction — no extra state, the cap recomputes from ki*integ.
struct WPid { float integ, prev; };
static float wheelPid(WPid& st, float tgt, float meas, float dt, bool stuck){
  float err = tgt - meas;
  float deriv = dt>0 ? (err - st.prev)/dt : 0; st.prev = err;
  float imax = g_wki > 1.0f ? 1.0f/g_wki : WHEEL_INTEG_MAX;
  // Clamp scaled to the LIVE ki so the I-term alone can never exceed full duty: the
  // fixed WHEEL_INTEG_MAX=1.0 was sized for ki~8 — at the tuned ki=60 one wound
  // integrator held ±60 duty of authority, so a stop-parked brake bias could lurch the
  // wheel on the next command before the loop could unwind it. 1/ki caps it at ±1 duty.
  float u = g_kff*tgt + g_wkp*err + g_wki*st.integ + g_wkd*deriv;
  float duty = clampf(u,-1,1);
  if (duty == u){   // integrate only when not saturated (anti-windup)
    float ni = clampf(st.integ + err*dt, -imax, imax);
    float it = g_wki*ni;
    if (stuck && g_wki > 1e-6f){
      float old = g_wki*st.integ, dmax = WHEEL_I_WIND_RATE*dt;
      if (it - old > dmax)       { it = old + dmax; ni = it/g_wki; }
      else if (old - it > dmax)  { it = old - dmax; ni = it/g_wki; }
    }
    st.integ = ni;
  }
  return duty;
}
#endif

static bool debounceSusp(int pin, bool& cand, uint8_t& stable, bool cur){
  bool lvl = digitalRead(pin)==HIGH, susp = SUSPEND_ACTIVE_HIGH?lvl:!lvl;
  if (susp==cand){ if(stable<3) stable++; } else { cand=susp; stable=0; }
  return (stable>=2)?cand:cur;
}

void setup(){
  // Motor safety FIRST — before Serial or anything else. The H-bridge IN pins sit in
  // their ROM-bootloader default (floating input) from power-on until something drives
  // them; Serial.begin()'s startup + the settle delay below used to be the first thing
  // that ran, stretching that floating window to ~300ms+ and letting it read as a brief
  // uncommanded spin on power-up. Drive them low immediately, then hand off to the LEDC
  // PWM channels (which also default to a 0 duty = low output).
  pinMode(LEFT_IN_FWD,OUTPUT);  digitalWrite(LEFT_IN_FWD,LOW);
  pinMode(LEFT_IN_REV,OUTPUT);  digitalWrite(LEFT_IN_REV,LOW);
  pinMode(RIGHT_IN_FWD,OUTPUT); digitalWrite(RIGHT_IN_FWD,LOW);
  pinMode(RIGHT_IN_REV,OUTPUT); digitalWrite(RIGHT_IN_REV,LOW);
  for (int c=0;c<4;c++) ledcSetup(c,PWM_FREQ_HZ,PWM_RES_BITS);
  ledcAttachPin(LEFT_IN_FWD,CH_LEFT_FWD); ledcAttachPin(LEFT_IN_REV,CH_LEFT_REV);
  ledcAttachPin(RIGHT_IN_FWD,CH_RIGHT_FWD); ledcAttachPin(RIGHT_IN_REV,CH_RIGHT_REV);
  applyMotors(0,0);

  Serial.begin(115200); delay(300);
  Serial.println("\n[nano] zenoh-pico coprocessor boot");
  // Reset reason on every boot: distinguishes a clean esp_restart() watchdog reboot
  // (ESP_RST_SW) from a brownout/power glitch (ESP_RST_BROWNOUT) / panic / WDT — the
  // 2026-09-21 load-correlated drops need exactly this discriminator on the console.
  Serial.printf("[nano] boot reset_reason=%d (1=poweron 2=ext 3=sw 4=panic 5=int_wdt 6=task_wdt 9=brownout)\n",
                (int)esp_reset_reason());

  pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,LOW);
  // SBC cooling fan PWM — off until the SBC link is alive (see FAN_BOOT_DUTY above).
  ledcSetup(CH_FAN,PWM_FREQ_HZ,PWM_RES_BITS); ledcAttachPin(FAN_PIN,CH_FAN);
  ledcWrite(CH_FAN,(uint32_t)(clampf(g_fan_duty,0,1)*PWM_MAX));
  // Line lasers 1-2 — off at boot; commanded via /laser_pwm once the link is up.
  ledcSetup(CH_LASER1,PWM_FREQ_HZ,PWM_RES_BITS); ledcAttachPin(LASER1_PIN,CH_LASER1); ledcWrite(CH_LASER1,0);
  ledcSetup(CH_LASER2,PWM_FREQ_HZ,PWM_RES_BITS); ledcAttachPin(LASER2_PIN,CH_LASER2); ledcWrite(CH_LASER2,0);

  pinMode(LEFT_ENC,INPUT_PULLUP);  attachInterrupt(digitalPinToInterrupt(LEFT_ENC),leftEncISR,RISING);
  pinMode(RIGHT_ENC,INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(RIGHT_ENC),rightEncISR,RISING);
  pinMode(LEFT_SUSPEND_PIN,INPUT_PULLUP); pinMode(RIGHT_SUSPEND_PIN,INPUT_PULLUP);

  // Straight-line trim from NVS (falls back to TRIM_DEFAULT if never calibrated / saved).
  g_prefs.begin("nano", false);
  g_trim = g_trim_saved = clampf(g_prefs.getFloat("trim", TRIM_DEFAULT), -TRIM_MAX, TRIM_MAX);
#if LDS_ENABLED
  // Last /lds_target_rpm setpoint persists like the trim: web_control's idle
  // controller owns the value, so an ESP32 reboot/power-cycle restores whatever the
  // SBC last said (usually 0 while idle) instead of spinning at the LDS_TARGET_RPM
  // boot default until the SBC's next re-assert. Clamped to the cb's range.
  g_lds_target = clampf(g_prefs.getFloat("ldstgt", LDS_TARGET_RPM), 0, LDS_RPM_MAX);
  Serial.printf("[nano] lds target %.0f rpm (NVS)\n", (double)g_lds_target);
#endif
#if WHEEL_PID_ENABLED
  // Live-tuned PID gains (motor_pid_cb) persist like the trim — a tuning session
  // survives reboot/reflash. Clamped to the same ranges the cb enforces.
  g_wkp = clampf(g_prefs.getFloat("kp", WHEEL_KP), 0, 20);
  g_wki = clampf(g_prefs.getFloat("ki", WHEEL_KI), 0, 100);
  g_wkd = clampf(g_prefs.getFloat("kd", WHEEL_KD), 0, 5);
  // Live drivetrain parameters (fall back to the corrected defines when absent).
  set_param(0, g_prefs.getFloat("tpr",   TICKS_PER_REV));
  set_param(1, g_prefs.getFloat("wrad",  WHEEL_RADIUS));
  set_param(2, g_prefs.getFloat("wsep",  WHEEL_SEPARATION));
  set_param(3, g_prefs.getFloat("maxlin",MAX_LINEAR_SPEED));
  set_param(4, g_prefs.getFloat("maxang",MAX_ANGULAR_SPEED));
  set_param(5, g_prefs.getFloat("slew",  WHEEL_TGT_SLEW));
  set_param(6, g_prefs.getFloat("dith",  WHEEL_DITHER));
  recalc_drive_params();
  g_pid_saved[0]=g_wkp; g_pid_saved[1]=g_wki; g_pid_saved[2]=g_wkd;
  Serial.printf("[nano] wheel PID gains kp=%.3f ki=%.3f kd=%.3f (NVS)\n",
                (double)g_wkp,(double)g_wki,(double)g_wkd);
#endif
  Serial.printf("[nano] wheel trim from NVS: %.3f\n", (double)g_trim);

#if LDS_ENABLED
  // LDS data on UART1 RX=GPIO14 (RX-only; UART2 is the zenoh link). Roomy RX buffer so a
  // burst of scan frames survives between PID ticks — we drain it only at the PID rate.
  Serial1.setRxBufferSize(1024);
  Serial1.begin(LDS_BAUD, SERIAL_8N1, LDS_RX_PIN, -1);
  // The LDS TX line fans out to BOTH this pin and the SBC's UART2 RX (PA1) — the ESP
  // reads RPM, the SBC reads the scan. uart_set_pin() (inside begin()) enables the
  // internal ~45k pull-up on RX; on the shared line that biases the LDS's weak TX
  // driver, which can corrupt the SBC's copy of the stream. Present a true
  // high-impedance input instead (the SBC side floats PA1 too — see deploy/sbc-setup.sh).
  gpio_set_pull_mode((gpio_num_t)LDS_RX_PIN, GPIO_FLOATING);
  ledcSetup(CH_LDS,PWM_FREQ_HZ,PWM_RES_BITS); ledcAttachPin(LDS_MOTOR_PIN,CH_LDS);
  Serial.printf("[nano] LDS on UART1 RX=%d, spin PID @%d Hz\n", LDS_RX_PIN, LDS_PID_HZ);
#endif

  g_temp = temperatureRead(); g_hall = hallRead();   // seed telemetry so first pub isn't 0
  g_reset_reason = (int32_t)esp_reset_reason();
  g_last_cmd_ms = millis();
  g_boot_ms = millis();                              // link-connect watchdog reference (see loop())
#if LINK_RX_TIMEOUT_MS && LINK_FIRST_PING_DEADLINE_MS
  // RTC noinit RAM is garbage at power-on; only an esp_restart() (SW reset) carries a
  // meaningful count. Everything else (power-on, brownout, panic) starts a fresh budget.
  if (esp_reset_reason() != ESP_RST_SW) g_fping_reboots = 0;
#endif
  // zenohTask pinned to Core 0; setup()/loop() (this code, + the LDS) run on Core 1. The
  // LDS can't starve the link: zenoh's read/lease tasks are prio 12 vs this loop's prio 1,
  // and the UART2 (zenoh) and UART1 (LDS) RX ISRs sit on Core 0 and Core 1 respectively.
  Serial.printf("[nano] control loop runs on core %d\n", xPortGetCoreID());
  xTaskCreatePinnedToCore(zenohTask, "zenoh", 16384, NULL, 5, NULL, 0);
}

void loop(){   // Core 1: real-time control
  static uint32_t last_pid=0, last_ctl=0, last_sens=0, last_slow=0;
  uint32_t now = millis();
  bool alive = linkAlive();   // true only once the SBC has actually pinged us (see linkAlive())

#if CPU_MHZ_LOWPOWER
  // millis()/FreeRTOS ticks come from a hardware timer independent of the CPU clock, so the
  // watchdog deadlines and PID loop rates below stay correctly timed at either frequency.
  // last_mode starts at an invalid sentinel (not "false") so the first loop() iteration
  // always applies — otherwise a board that boots and never truly connects (linkAlive()
  // stays false from power-on, same as the last_mode default) would never actually downclock.
  static int8_t last_mode = -1;
  int8_t want_mode = alive ? 1 : 0;
  if (want_mode != last_mode){
    last_mode = want_mode;
    setCpuFrequencyMhz(alive ? CPU_MHZ_NORMAL : CPU_MHZ_LOWPOWER);
    Serial.printf("[nano] link %s: CPU -> %d MHz\n", alive ? "up" : "down",
                  alive ? CPU_MHZ_NORMAL : CPU_MHZ_LOWPOWER);
  }
#endif

#if LINK_CONNECT_DEADLINE_MS
  // Link-connect watchdog: never came up within the deadline → reboot and re-handshake the
  // (by now likely-listening) router, instead of waiting for a manual power-cycle. Only fires
  // while still unconnected; once `ready`, we never reboot from here. Runs on Core 1 so it
  // also rescues a z_open() that wedged the zenohTask on Core 0.
  if (!ready && (now - g_boot_ms) > LINK_CONNECT_DEADLINE_MS){
    Serial.println("[nano] link not up within deadline — esp_restart() to re-handshake router");
    Serial.flush();
    esp_restart();
  }
#endif
#if LINK_RX_TIMEOUT_MS
  // Runtime liveness: connected + had pings + they stopped => the router/SBC restarted under
  // us (serial can't detect peer-gone). Reboot to re-handshake. Armed only after 1st ping.
  if (ready && g_ping_seen && (now - g_last_ping_ms) > LINK_RX_TIMEOUT_MS){
    Serial.println("[nano] /esp32_ping stopped — esp_restart() to re-join the graph");
    Serial.flush();
    esp_restart();
  }
#if LINK_FIRST_PING_DEADLINE_MS
  // First-ping deadline: `ready` but no ping EVER since the (re)connect — the session
  // opened against a router that vanished before the graph came up (see the define).
  // g_last_ping_ms was seeded with millis() at connect, so it doubles as the reference.
  if (ready && !g_ping_seen && (now - g_last_ping_ms) > LINK_FIRST_PING_DEADLINE_MS
      && g_fping_reboots < LINK_FIRST_PING_MAX_REBOOTS){
    g_fping_reboots++;
    Serial.printf("[nano] connected but no /esp32_ping ever — esp_restart() to re-handshake (%u/%u)\n",
                  (unsigned)g_fping_reboots, (unsigned)LINK_FIRST_PING_MAX_REBOOTS);
    Serial.flush();
    esp_restart();
  }
#endif
#endif

#if LDS_ENABLED
  if (now-last_pid >= (uint32_t)(1000/LDS_PID_HZ)){    // spin PID @50 Hz
    // Drain UART1 here, not every loop: every frame carries the current RPM, so flushing
    // the buffer right before the PID gives the freshest speed and skips idle polling.
    while (Serial1.available()) ldsFeed((uint8_t)Serial1.read());
    // Park the spin motor while the SBC link is down (boot race, SBC off, or a drop) —
    // g_lds_target defaults to LDS_TARGET_RPM at boot and just sits there otherwise, so
    // without this the LDS keeps spinning even with the SBC fully powered off. Resumes
    // on its own the instant `ready` goes true again (zenohTask, Core 0).
    if (alive) ldsControl((now-last_pid)/1000.0f);
    else { g_lds_duty = 0; ledcWrite(CH_LDS, 0); }
    last_pid=now;
  }
#else
  (void)last_pid;
#endif

#if WHEEL_PID_ENABLED
  static uint32_t last_wpid=0; static int32_t wp_l=0, wp_r=0; static WPid wpid_l{0,0}, wpid_r{0,0};
  static float l_tgt_s=0, r_tgt_s=0;                     // slewed setpoints (command shaping)
  static int32_t vring_l[WHEEL_VEL_FILT_MAX], vring_r[WHEEL_VEL_FILT_MAX];  // per-window tick deltas
  static uint8_t vridx=0, vrcnt=0;                       // ring cursor + filled count
  static bool vel_seed=false;                            // first-tick baseline (see below)
  static int8_t l_dir_seen=1, r_dir_seen=1;              // last commanded wheel direction
  if (now-last_wpid >= (uint32_t)(1000/WHEEL_PID_HZ)){     // wheel velocity PID @WHEEL_PID_HZ
    float dt=(now-last_wpid)/1000.0f; last_wpid=now;
    int32_t l=g_left_ticks, r=g_right_ticks;               // atomic 32-bit reads
    if (!vel_seed){ wp_l=l; wp_r=r; vel_seed=true; }       // first tick after boot: seed the
                                                           // baseline from the LIVE counts —
                                                           // the delta from 0 would read as a
                                                           // huge fake velocity and full-duty-
                                                           // jerk the first PID tick
    int32_t dl=l-wp_l, dr=r-wp_r; wp_l=l; wp_r=r;
    bool jump = fabsf((float)dl) > 3.0f*g_tpm*dt || fabsf((float)dr) > 3.0f*g_tpm*dt;
    if (jump){
      // Counter discontinuity (POST /reset_ticks while parked, or wrap) — not motion:
      // re-seed instead of letting the PID see a fake ±m/s velocity spike and lurch.
      dl = 0; dr = 0;
      vridx = 0; vrcnt = 0;                                // nothing in the ring is valid
    }
    // Ring push (zeros too — the filter is a moving average over the last N windows).
    vring_l[vridx]=dl; vring_r[vridx]=dr;
    vridx=(uint8_t)((vridx+1)%WHEEL_VEL_FILT_MAX);
    if (vrcnt < WHEEL_VEL_FILT_MAX) vrcnt++;
    // Adaptive window count (WHEEL_VEL_FILT_MAX/WHEEL_VEL_QUANT above): pick N so the
    // 1-tick quantization step (1/(N*dt) m/s) stays <= WHEEL_VEL_QUANT of the wheel's
    // moving setpoint — crawl/turn wheels get the longest filter (ripple crush), fast
    // commands the shortest (lag). Capped by what's actually in the ring.
    float qstep = 1.0f/(dt*g_tpm);                         // one window's quantization step
    uint8_t nl = 2, nr = 2;
    if (qstep > 0.0f){
      float want = WHEEL_VEL_QUANT*fabsf(l_tgt_s);
      if (want > 1e-5f) nl = (uint8_t)ceilf(qstep/want);
      want = WHEEL_VEL_QUANT*fabsf(r_tgt_s);
      if (want > 1e-5f) nr = (uint8_t)ceilf(qstep/want);
    }
    auto velAvg = [&](const int32_t* ring, uint8_t n){
      uint8_t cnt = vrcnt < n ? vrcnt : n;
      if (!cnt) cnt = 1;
      int32_t s = 0;
      for (uint8_t i=1; i<=cnt; i++)
        s += ring[(uint8_t)((vridx + WHEEL_VEL_FILT_MAX - i) % WHEEL_VEL_FILT_MAX)];
      return (float)s/(cnt*dt*g_tpm);
    };
    g_left_vel  = velAvg(vring_l, nl);
    g_right_vel = velAvg(vring_r, nr);
    // Commanded-direction flip reset: single-channel ticks are signed by the COMMANDED
    // direction, so the instant a reverse command lands the still-forward-rolling wheel
    // reads as ALREADY moving the other way (fabricated vel) — kp*err then drives the OLD
    // direction at (near) full duty until friction stalls the wheel, with the
    // forward-wound integrator adding to it (pause, then lurch into reverse). Zero the
    // PID state AND the delta ring (its entries still carry the OLD sign convention) so
    // the reversal starts from feedforward alone; the I-term rebuilds in the new
    // direction. (Single-channel feedback is permanent — see the header note; this
    // reset + ring zero is the final mitigation, not a stopgap.)
    int8_t ld = (g_left_tgt  > 1e-4f) ?  1 : (g_left_tgt  < -1e-4f) ? -1 : l_dir_seen;
    int8_t rd = (g_right_tgt > 1e-4f) ?  1 : (g_right_tgt < -1e-4f) ? -1 : r_dir_seen;
    if (ld != l_dir_seen){ l_dir_seen = ld; wpid_l.integ = 0; wpid_l.prev = 0;
                           memset(vring_l, 0, sizeof(vring_l)); }
    if (rd != r_dir_seen){ r_dir_seen = rd; wpid_r.integ = 0; wpid_r.prev = 0;
                           memset(vring_r, 0, sizeof(vring_r)); }
    // Parked-at-zero bleed: the web keepalive re-asserts {0,0} forever, so the command
    // never goes stale and the dead-man never resets the integrators — but a stop leaves
    // integ wound NEGATIVE (braking unwinds it below zero), which then holds a small
    // REVERSE duty on the parked wheel: a rollback nudge at every stop and an asymmetric
    // lurch on the next start. Once the slewed setpoint is ~0 AND the wheel has stopped,
    // drop the PID state. Skipped while any target is commanded (the I-term must keep
    // building through stiction). On a slope the reset lets the robot creep until the
    // I-term rebuilds — flat floors only, which is this robot's contract.
    if (fabsf(l_tgt_s) < 0.005f && fabsf(g_left_vel) < 0.03f){
      wpid_l.integ = 0; wpid_l.prev = 0;
    }
    if (fabsf(r_tgt_s) < 0.005f && fabsf(g_right_vel) < 0.03f){
      wpid_r.integ = 0; wpid_r.prev = 0;
    }
    if (g_pid_reset){                                      // live gain change (motor_pid_cb):
      g_pid_reset = false;                                 // stale integ/prev must not leak
      wpid_l.integ=wpid_l.prev=0; wpid_r.integ=wpid_r.prev=0;   // into the new tuning
    }
    if (now-g_last_cmd_ms > CMD_TIMEOUT_MS){               // cmd stale: stop + reset integrators
      wpid_l.integ=wpid_l.prev=0; wpid_r.integ=wpid_r.prev=0;
      l_tgt_s=r_tgt_s=0;
      g_left_duty=0; g_right_duty=0;
    } else {
      // Command shaping: slew the per-wheel SETPOINT toward the latest /cmd_vel target
      // (WHEEL_TGT_SLEW) — accel limiting OUTSIDE the loop, so the controller itself
      // stays lag-free (slewing the PID output instead would lag the plant response and
      // wind the integrator). A fresh command snaps the slewed setpoint's course; the
      // dead-man above snaps it to 0 instantly.
      float maxDv = g_slew*dt;
      l_tgt_s = slewTo(l_tgt_s, g_left_tgt,  maxDv);
      r_tgt_s = slewTo(r_tgt_s, g_right_tgt, maxDv);
      // Stiction detection (WHEEL_STUCK_FRAC): commanded but barely turning = the wheel
      // is fighting static friction -> the PID ramps its I-term (rate-limited) instead
      // of hammering; tracking/braking integrate freely.
      bool l_stuck = fabsf(l_tgt_s) > 0.01f
                     && fabsf(g_left_vel)  < WHEEL_STUCK_FRAC*fabsf(l_tgt_s);
      bool r_stuck = fabsf(r_tgt_s) > 0.01f
                     && fabsf(g_right_vel) < WHEEL_STUCK_FRAC*fabsf(r_tgt_s);
      g_left_duty  = wheelPid(wpid_l, l_tgt_s,  g_left_vel,  dt, l_stuck);
      g_right_duty = wheelPid(wpid_r, r_tgt_s, g_right_vel, dt, r_stuck);
      // Stiction dither (see WHEEL_DITHER above): alternating ±duty added OUTSIDE the
      // PID so the integrator never sees it. Per-wheel amplitude from that wheel's own
      // setpoint (a spin commands ±0.04 m/s wheels — full dither — while a fast drive
      // fades it out).
      static uint8_t dticks = 0; static bool dphase = false;
      if (++dticks >= WHEEL_DITHER_TICKS){ dticks = 0; dphase = !dphase; }
      float ds = dphase ? 1.0f : -1.0f;
      g_left_duty += ds * g_dither
        * clampf(fabsf(l_tgt_s)/WHEEL_DITHER_IN, 0.0f, 1.0f)
        * clampf(1.0f - fabsf(l_tgt_s)/WHEEL_DITHER_FADE, 0.0f, 1.0f);
      g_right_duty += ds * g_dither
        * clampf(fabsf(r_tgt_s)/WHEEL_DITHER_IN, 0.0f, 1.0f)
        * clampf(1.0f - fabsf(r_tgt_s)/WHEEL_DITHER_FADE, 0.0f, 1.0f);
      // Keep the commanded duty inside ±1 — the dither on a saturated PID output
      // could read 1.03 on the debug line (writeSide clamps again; this is cosmetic
      // and keeps the conditional-integration bookkeeping honest).
      g_left_duty  = clampf(g_left_duty,  -1.0f, 1.0f);
      g_right_duty = clampf(g_right_duty, -1.0f, 1.0f);
    }
  }
#endif

#if TRIM_AUTOCAL && !WHEEL_PID_ENABLED
  // Straight-line trim autocal: while a straight drive is commanded, fold the relative
  // L/R encoder-rate imbalance into g_trim. Left faster => robot veers right => positive
  // error => trim up (boost right / cut left). Skipped whenever the window isn't a clean
  // straight run: stale cmd, unequal/near-zero duties (arc, pivot, stop), a lifted wheel,
  // or too few ticks (stall/crawl — quantization would dominate).
  static uint32_t last_cal=0; static int32_t cal_l=0, cal_r=0;
  if (now-last_cal >= (uint32_t)(1000/TRIM_CAL_HZ)){
    last_cal=now;
    int32_t l=g_left_ticks, r=g_right_ticks;                 // atomic 32-bit reads
    int32_t dl=labs(l-cal_l), dr=labs(r-cal_r); cal_l=l; cal_r=r;
    float cl=g_left_duty, cr=g_right_duty;                   // commanded (pre-trim) duties
    if (now-g_last_cmd_ms <= CMD_TIMEOUT_MS
        && fabsf(cl-cr) <= TRIM_MATCH_TOL
        && fabsf(cl) > MOTOR_DEADZONE && fabsf(cr) > MOTOR_DEADZONE
        && !g_susp_l && !g_susp_r
        && dl >= TRIM_MIN_TICKS && dr >= TRIM_MIN_TICKS){
      float err = clampf((float)(dl-dr) / (0.5f*(float)(dl+dr)), -TRIM_ERR_CLAMP, TRIM_ERR_CLAMP);
      g_trim = clampf(g_trim + TRIM_CAL_GAIN*err, -TRIM_MAX, TRIM_MAX);
    }
  }
#endif

  if (now-last_ctl >= 10){                                  // motors + watchdog @100 Hz
    uint32_t ctl_dt_ms = now-last_ctl;                      // ms since the previous tick
    last_ctl=now;
    bool cmd_stale = (now-g_last_cmd_ms > CMD_TIMEOUT_MS);
    if (cmd_stale){ g_left_duty=0; g_right_duty=0; }
    static float l_ramped=0, r_ramped=0;
    float l_apply = 0, r_apply = 0;
#if WHEEL_PID_ENABLED
    // Closed-loop path: the applied duty IS the PID output (recomputed @50 Hz above, set
    // to 0 by the dead-man). No duty slew here — accel limiting is the SETPOINT slew in
    // the PID block (WHEEL_TGT_SLEW); slewing the output would lag the loop and wind the
    // integrator. The dead-man (cmd_stale) zeroes everything instantly either way.
    (void)ctl_dt_ms;
    l_ramped = g_left_duty; r_ramped = g_right_duty;
    l_apply = l_ramped; r_apply = r_ramped;
#else
    // Ramp the applied duty toward the commanded duty (see g_motor_slew / MOTOR_SLEW_DEFAULT) instead of
    // stepping straight to it — smooths starts, stops, and direction reversals. Skipped on
    // a stale cmd so the dead-man stop is instant, not a ramped coast-down.
    if (cmd_stale){ l_ramped=0; r_ramped=0; }
    else {
      float maxDelta = g_motor_slew * (ctl_dt_ms/1000.0f);
      l_ramped = slewTo(l_ramped, g_left_duty, maxDelta);
      r_ramped = slewTo(r_ramped, g_right_duty, maxDelta);
    }
    // Breakaway push (see MOTOR_PUSH_* above): a SUSTAINED full-duty push while a wheel
    // is powered-but-frozen, ending the instant ticks resume (smooth handoff back to the
    // ramped duty). Capped per push; a push that expires without breakaway re-arms the
    // frozen timer and doubles the wait before the next attempt (backoff resets the
    // moment the wheel moves again), so a hard jam nudges occasionally instead of
    // ramming. Ticks are the stall signal — at crawl they still flow every few 10 ms
    // windows, so 250 ms of frozen counts under nonzero duty can only be a seized rotor
    // (or a jam), not tick quantization.
    static int32_t l_kick_ticks=0, r_kick_ticks=0;
    static uint32_t l_stall_since=0, r_stall_since=0;
    static bool l_pushing=false, r_pushing=false;
    static uint32_t l_push_start=0, r_push_start=0;
    static uint32_t l_backoff=MOTOR_PUSH_RECHECK, r_backoff=MOTOR_PUSH_RECHECK;
    if (fabsf(l_ramped) > MOTOR_DEADZONE){
      if (l_stall_since && g_left_ticks == l_kick_ticks){
        if (l_pushing){
          if (now - l_push_start > MOTOR_PUSH_MAX_MS){      // capped push failed -> back off
            l_pushing = false; l_stall_since = now;
            uint32_t b = l_backoff*2;
            l_backoff = (b > MOTOR_PUSH_MAX_BACKOFF) ? MOTOR_PUSH_MAX_BACKOFF : b;
          }
        } else if (now - l_stall_since > l_backoff){
          l_pushing = true; l_push_start = now;
        }
      } else { l_stall_since = now; l_kick_ticks = g_left_ticks;
               l_pushing = false; l_backoff = MOTOR_PUSH_RECHECK; }
    } else { l_pushing = false; l_stall_since = 0; l_backoff = MOTOR_PUSH_RECHECK; }
    if (fabsf(r_ramped) > MOTOR_DEADZONE){
      if (r_stall_since && g_right_ticks == r_kick_ticks){
        if (r_pushing){
          if (now - r_push_start > MOTOR_PUSH_MAX_MS){      // capped push failed -> back off
            r_pushing = false; r_stall_since = now;
            uint32_t b = r_backoff*2;
            r_backoff = (b > MOTOR_PUSH_MAX_BACKOFF) ? MOTOR_PUSH_MAX_BACKOFF : b;
          }
        } else if (now - r_stall_since > r_backoff){
          r_pushing = true; r_push_start = now;
        }
      } else { r_stall_since = now; r_kick_ticks = g_right_ticks;
               r_pushing = false; r_backoff = MOTOR_PUSH_RECHECK; }
    } else { r_pushing = false; r_stall_since = 0; r_backoff = MOTOR_PUSH_RECHECK; }
    // Push direction keys off the RAMPED duty: a push only ever starts >=MOTOR_PUSH_RECHECK
    // after the ramp left the deadzone, so the ramp already carries the command's sign
    // (the old immediate start-kick needed the commanded-duty fallback; that's gone).
    l_apply = l_pushing ? copysignf(1.0f, l_ramped) : l_ramped;
    r_apply = r_pushing ? copysignf(1.0f, r_ramped) : r_ramped;
#endif
    applyMotors(l_apply, r_apply);
    // Stray-tick gating: a wheel counts as "stopped" STRAY_SETTLE_MS after its APPLIED
    // (ramped) duty last went to exactly 0 (coast-down grace period), and un-stops the
    // instant nonzero power is applied again. MUST key off the ramped duty (l_ramped/
    // r_ramped), not the commanded duty (g_*_duty): the ramp can still be driving the
    // motor for tens-to-hundreds of ms after a stop command zeroes the commanded duty,
    // and any real rotation during that window is genuine, not encoder noise — flagging
    // it as stray under-counts /odom and false-positives a flaky-encoder.
    static uint32_t l_zero_since=0, r_zero_since=0;
    if (fabsf(l_ramped) > MOTOR_DEADZONE){ l_zero_since=0; g_left_stopped=false; }
    else { if (!l_zero_since) l_zero_since=now; g_left_stopped = (now-l_zero_since) >= STRAY_SETTLE_MS; }
    if (fabsf(r_ramped) > MOTOR_DEADZONE){ r_zero_since=0; g_right_stopped=false; }
    else { if (!r_zero_since) r_zero_since=now; g_right_stopped = (now-r_zero_since) >= STRAY_SETTLE_MS; }
    // Fan tracks true SBC presence (like the LDS park above), not a /cmd_vel-style command
    // watchdog: park it whenever the link isn't alive (boot race, drop, or the SBC genuinely
    // off) since there's no SBC heat to move, and resume the instant sys_monitor reconnects.
    if (alive) ledcWrite(CH_FAN,(uint32_t)(clampf(g_fan_duty,0,1)*PWM_MAX));
    else       { g_fan_duty = 0; ledcWrite(CH_FAN, 0); }
    // Line lasers track true SBC presence like the fan: park (off) whenever the link
    // isn't alive and zero the setpoints so they resume at 0, not the stale pre-drop value.
    // 0..255 web value -> 10-bit LEDC duty.
    if (alive){
      ledcWrite(CH_LASER1, (uint32_t)g_laser[0]*PWM_MAX/255u);
      ledcWrite(CH_LASER2, (uint32_t)g_laser[1]*PWM_MAX/255u);
    } else {
      g_laser[0] = g_laser[1] = 0;
      ledcWrite(CH_LASER1,0); ledcWrite(CH_LASER2,0);
    }
  }
  if (now-last_sens >= 100){                                // suspension debounce + LED @10 Hz
    last_sens=now;
    static bool cl=false,cr=false; static uint8_t sl=0,sr=0;
    g_susp_l = debounceSusp(LEFT_SUSPEND_PIN, cl, sl, g_susp_l);
    g_susp_r = debounceSusp(RIGHT_SUSPEND_PIN,cr, sr, g_susp_r);
    if (g_led_dirty){ digitalWrite(LED_PIN, g_led?HIGH:LOW); g_led_dirty=false; }
#if LDS_ENABLED
    // compute LDS frame-rate (Hz)
    static uint32_t lf=0, lm=0;
    uint32_t f=g_lds_frames; g_lds_hz = (lm && now>lm)?(f-lf)*1000.0f/(now-lm):0; lf=f; lm=now;
#endif
  }
  if (now-last_slow >= 1000){                               // die telemetry @1 Hz (its pub rate)
    last_slow=now;
    g_temp = temperatureRead();
    g_hall = hallRead();
    // Persist the trim, rate-limited (flash wear) and only while the motors are stopped —
    // an NVS commit stalls flash cache for a few ms and shouldn't land mid-drive.
    static uint32_t last_save=0;
    if (now-g_last_cmd_ms > CMD_TIMEOUT_MS
        && fabsf(g_trim - g_trim_saved) > TRIM_SAVE_DELTA
        && now-last_save > TRIM_SAVE_MS){
      last_save=now; g_trim_saved=g_trim;
      g_prefs.putFloat("trim", g_trim_saved);
      Serial.printf("[nano] trim %.3f saved to NVS\n", (double)g_trim_saved);
    }
#if LDS_ENABLED
    // Same rate-limited NVS write for the LDS spin setpoint (g_lds_tgt_save set by
    // ldstgt_cb) — so an ESP32 reboot/power-cycle restores the SBC's last command
    // (usually 0 while idle) instead of the LDS_TARGET_RPM boot default. No
    // while-stopped gate needed: this isn't a drive motor, and a few ms of flash
    // stall can't overflow the 1 KB UART1 tach buffer at 115200 baud.
    static uint32_t last_lds_save=0;
    if (g_lds_tgt_save && now-last_lds_save > TRIM_SAVE_MS){
      last_lds_save=now; g_lds_tgt_save=false;
      g_prefs.putFloat("ldstgt", g_lds_target);
      Serial.printf("[nano] lds target %.0f saved to NVS\n", (double)g_lds_target);
    }
#endif
#if WHEEL_PID_ENABLED
    // Same deal for the live-tuned wheel-PID gains (g_pid_save set by motor_pid_cb).
    // Shares the trim's rate limit + while-stopped gate; the flag survives until a
    // quiet window actually writes, so gains tuned mid-drive persist once parked.
    static uint32_t last_pid_save=0;
    if (g_pid_save && now-g_last_cmd_ms > CMD_TIMEOUT_MS
        && now-last_pid_save > TRIM_SAVE_MS){
      last_pid_save=now;
      g_pid_saved[0]=g_wkp; g_pid_saved[1]=g_wki; g_pid_saved[2]=g_wkd;
      g_prefs.putFloat("kp", g_pid_saved[0]);
      g_prefs.putFloat("ki", g_pid_saved[1]);
      g_prefs.putFloat("kd", g_pid_saved[2]);
      g_pid_save = false;
      Serial.printf("[nano] wheel PID gains %.3f/%.3f/%.3f saved to NVS\n",
                    (double)g_pid_saved[0],(double)g_pid_saved[1],(double)g_pid_saved[2]);
    }
    // Same deal for the live drivetrain parameters (g_par_save set by motor_params_cb).
    static uint32_t last_par_save=0;
    if (g_par_save && now-g_last_cmd_ms > CMD_TIMEOUT_MS
        && now-g_par_saved_ms > TRIM_SAVE_MS){
      g_par_saved_ms = now;
      g_prefs.putFloat("tpr",   g_tpr);
      g_prefs.putFloat("wrad",  g_wrad);
      g_prefs.putFloat("wsep",  g_wsep);
      g_prefs.putFloat("maxlin",g_maxlin);
      g_prefs.putFloat("maxang",g_maxang);
      g_prefs.putFloat("slew",  g_slew);
      g_prefs.putFloat("dith",  g_dither);
      g_par_save = false;
      Serial.printf("[nano] drive params tpr=%.1f rad=%.4f sep=%.3f maxlin=%.2f maxang=%.2f slew=%.2f dith=%.2f saved to NVS\n",
        (double)g_tpr,(double)g_wrad,(double)g_wsep,(double)g_maxlin,(double)g_maxang,(double)g_slew,(double)g_dither);
    }
#endif
  }
#if STATUS_PRINT_MS
  static uint32_t last_dbg=0;
  if (now-last_dbg >= STATUS_PRINT_MS){                     // debug-console health line
    last_dbg=now;
    Serial.printf("[nano] ticks L=%ld R=%ld | trim %+.3f | lds rpm=%.0f hz=%.0f duty=%.2f jam=%d | susp %d/%d\n",
      (long)g_left_ticks,(long)g_right_ticks, (double)g_trim,
      g_lds_rpm, g_lds_hz, g_lds_duty, (int)g_lds_jam, (int)g_susp_l,(int)g_susp_r);
#if WHEEL_PID_ENABLED
    Serial.printf("[nano] wheel vel L=%.3f R=%.3f m/s | tgt L=%.3f R=%.3f | duty L=%.2f R=%.2f\n",
      g_left_vel, g_right_vel, g_left_tgt, g_right_tgt, g_left_duty, g_right_duty);
#endif
  }
#endif
  delay(1);
}
