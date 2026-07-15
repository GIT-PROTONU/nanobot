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
//   pub  wheel_ticks           std_msgs/Int64MultiArray  [L,R] raw cumulative counts
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
// LDS data RX moves 16->35 (input-only, RX-only).
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
#define SUSPEND_ACTIVE_HIGH true
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

#define PWM_FREQ_HZ   20000
#define PWM_RES_BITS  10
static const uint32_t PWM_MAX = (1u << PWM_RES_BITS) - 1u;

#define WHEEL_SEPARATION  0.16f
#define MAX_LINEAR_SPEED  0.4f
#define MAX_ANGULAR_SPEED 3.0f
#define CMD_TIMEOUT_MS    500
#define INVERT_LEFT  false
#define INVERT_RIGHT false
// Stiction deadband compensation: the gearmotors don't move below ~60% duty (only a
// full-scale 0.4 m/s command — duty 0.63 after the v+w normalization — moved at all, and
// in-place turns at 0.37 duty didn't). Remap any command above MOTOR_DEADZONE from
// (0..1] to [MOTOR_MIN_DUTY..1] so slow speeds and tank turns still overcome friction.
// Tune MOTOR_MIN_DUTY down to just below where the wheels reliably start on the ground.
#define MOTOR_MIN_DUTY 0.55f
#define MOTOR_DEADZONE 0.02f   // |duty| below this = intended stop, not a crawl

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
// (Compiled out under WHEEL_PID_ENABLED — a velocity PID equalizes the wheels itself.)
#define TRIM_AUTOCAL    1
#define TRIM_MAX        0.30f   // |trim| clamp — beyond this something is broken, not unmatched
#define TRIM_CAL_HZ     5       // adaptation windows/s (200 ms of ticks each)
#define TRIM_CAL_GAIN   0.08f   // fraction of the measured imbalance folded in per window
#define TRIM_ERR_CLAMP  0.5f    // per-window |imbalance| cap (limits spin-up transients)
#define TRIM_MIN_TICKS  40      // per-wheel ticks/window below this = too slow/stalled, skip
#define TRIM_MATCH_TOL  0.02f   // commanded duties must match within this = "straight"
#define TRIM_SAVE_MS    10000   // NVS write rate limit (flash wear)
#define TRIM_SAVE_DELTA 0.005f  // ...and only if it moved at least this much

// ---- closed-loop wheel velocity PID (OPTIONAL; OFF by default) ----------------
// Holds each wheel's commanded linear speed (m/s) via a per-wheel PID on encoder-tick
// velocity, replacing the open-loop duty = speed/full-scale map. DISABLED by default:
// an untuned PID can drive erratically, and the feedback is single-channel (blind on
// reverse-through-zero / stall / slip / being pushed — see [[esp32-pid-velocity-pending]]).
// To use: set =1, then tune KP/KI ON HARDWARE (watch wheel vel on the debug console). With
// KP=KI=KD=0 the feedforward alone reproduces today's open-loop behavior — a safe baseline.
#define WHEEL_PID_ENABLED 0
#define WHEEL_RADIUS      0.0335f   // m  (matches robot.yaml wheel_odometry.wheel_radius)
#define TICKS_PER_REV     1440.0f   // counts/wheel-rev as the ESP emits them (matches odom)
#define WHEEL_PID_HZ      50        // PID rate; longer window than the 100 Hz loop => less tick-quantization noise
// Full-scale wheel speed at duty=1; KFF = 1/that maps a target m/s straight to the
// open-loop duty, so feedforward-only == today's behavior.
#define WHEEL_KFF   (1.0f/(MAX_LINEAR_SPEED + MAX_ANGULAR_SPEED*WHEEL_SEPARATION*0.5f))
#define WHEEL_KP    0.0f            // <- tune up first
#define WHEEL_KI    0.0f            // <- then add a little to kill steady-state error
#define WHEEL_KD    0.0f
#define WHEEL_INTEG_MAX 1.0f        // anti-windup: integral clamp (duty units)
static const float TICKS_PER_METER = TICKS_PER_REV / (2.0f*3.14159265f*WHEEL_RADIUS);

#define LDS_BAUD       115200
#define LDS_TIMEOUT_MS 300
#define LDS_TARGET_RPM 300.0f
#define LDS_PID_HZ     50
#define LDS_PID_KFF    0.0020f
#define LDS_PID_KP     0.0010f
#define LDS_PID_KI     0.0015f
#define LDS_PID_KD     0.0f

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
static volatile float    g_fan_duty = FAN_BOOT_DUTY;   // /fan_pwm 0..1 (Core0 write, Core1 apply)
// Straight-line trim: loaded from NVS in setup(), adapted on Core 1 (autocal), manually
// set from the zenoh RX task (/motor_trim cb). Aligned-32-bit volatile = atomic enough.
static volatile float    g_trim = 0;
static Preferences       g_prefs;          // NVS handle (namespace "nano", key "trim")
static float             g_trim_saved = 0; // last value written to NVS (Core 1 only)
static volatile float    g_temp = 0;
static volatile int32_t  g_hall = 0;
static volatile bool     g_susp_l = false, g_susp_r = false;
static volatile bool     g_led = false, g_led_dirty = false;
static volatile uint32_t g_last_ping_ms = 0;   // last /esp32_ping rx (runtime liveness watchdog)
static volatile bool     g_ping_seen = false;  // arm the runtime watchdog only after 1st ping
#if LINK_RX_TIMEOUT_MS && LINK_FIRST_PING_DEADLINE_MS
// Survives esp_restart() (undefined at power-on — setup() clears it on non-SW resets).
RTC_NOINIT_ATTR static uint32_t g_fping_reboots;
#endif

static void IRAM_ATTR leftEncISR()  { g_left_ticks  += g_left_dir; }
static void IRAM_ATTR rightEncISR() { g_right_ticks += g_right_dir; }

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

// ============================ zenoh session (Core 0) ==========================
#define DOMAIN "0"
#define KE(topic, type) DOMAIN "/" topic "/" type "/TypeHashNotSupported"
#define T_I32  "std_msgs::msg::dds_::Int32_"
#define T_F32  "std_msgs::msg::dds_::Float32_"
#define T_BOOL "std_msgs::msg::dds_::Bool_"
#define T_I64A "std_msgs::msg::dds_::Int64MultiArray_"
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
static z_owned_liveliness_token_t g_lv[12]; static int g_lv_n = 0;
static void declare_lv(const char* topic, const char* type, int eid){
  char ke[260];
  snprintf(ke, sizeof(ke),
    "@ros2_lv/" DOMAIN "/" NODE_ZID "/0/%d/MP/%%/%%/" NODE_NAME "/%%%s/%s/TypeHashNotSupported/:1:,1:,:,:,,",
    eid, topic, type);
  z_view_keyexpr_t vke; z_view_keyexpr_from_str_unchecked(&vke, ke);
  z_liveliness_declare_token(z_session_loan(&s), &g_lv[g_lv_n++], z_view_keyexpr_loan(&vke), NULL);
}

// one publisher + its rmw attachment identity
struct ZPub { z_owned_publisher_t p; int64_t seq; uint8_t gid[16]; };
static ZPub P_ticks, P_suspL, P_suspR, P_temp, P_hall, P_rpm, P_hz, P_duty, P_hb, P_trim;

// Single source of truth for every publisher: topic/type, the attachment GID tag
// (last GID byte, unique per publisher) and the liveliness entity id (lv_eid, also
// unique). The declare loop and the liveliness loop both walk this, so the two can't
// drift. These wire identities are PROVEN-GOOD against the live graph — don't renumber.
struct PubDef { ZPub* zp; const char* topic; const char* type; uint8_t gid_tag; int lv_eid; bool lds_only; };
static const PubDef PUBS[] = {
  { &P_ticks, "wheel_ticks",           T_I64A, 1, 1, false },
  { &P_suspL, "left_wheel_suspended",  T_BOOL, 2, 2, false },
  { &P_suspR, "right_wheel_suspended", T_BOOL, 3, 3, false },
  { &P_temp,  "esp32_temp",            T_F32,  4, 4, false },
  { &P_hall,  "esp32_hall",            T_I32,  5, 5, false },
  { &P_hb,    "esp32_heartbeat",       T_I32,  9, 6, false },
  { &P_rpm,   "lds_rpm",               T_F32,  6, 7, true  },
  { &P_hz,    "lds_hz",                T_F32,  7, 8, true  },
  { &P_duty,  "lds_duty",              T_F32,  8, 9, true  },
  { &P_trim,  "wheel_trim",            T_F32, 10, 10, false },
};

static void zpub_declare(ZPub& zp, const char* topic, const char* type, uint8_t tag){
  char keyexpr[160];
  snprintf(keyexpr, sizeof(keyexpr), DOMAIN "/%s/%s/TypeHashNotSupported", topic, type);
  z_view_keyexpr_t ke; z_view_keyexpr_from_str_unchecked(&ke, keyexpr);
  z_declare_publisher(z_session_loan(&s), &zp.p, z_view_keyexpr_loan(&ke), NULL);
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
  float fv = clampf((float)v, -MAX_LINEAR_SPEED,  MAX_LINEAR_SPEED);
  float fw = clampf((float)w, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
  float vl = fv - fw*WHEEL_SEPARATION*0.5f, vr = fv + fw*WHEEL_SEPARATION*0.5f;
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
static void ldstgt_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 8){ float f; memcpy(&f,b+4,4); g_lds_target = f>0?f:0; }
}
static void fan_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 8){ float f; memcpy(&f,b+4,4); g_fan_duty = clampf(f,0,1); }
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
#if LINK_RX_TIMEOUT_MS
// /esp32_ping (Int32) from the SBC web_control node — payload ignored; arrival = link alive.
static void ping_cb(z_loaned_sample_t*, void*){
  g_last_ping_ms = millis(); g_ping_seen = true;
#if LINK_FIRST_PING_DEADLINE_MS
  g_fping_reboots = 0;   // real pings flow — re-earn the full first-ping reboot budget
#endif
}
#endif

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

  static z_owned_subscriber_t sub_cmd, sub_led, sub_tgt, sub_fan, sub_trim;   // kept alive (static)
  z_owned_closure_sample_t cl;
  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KE("cmd_vel",T_TWIST));
  z_closure_sample(&cl, cmd_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_cmd, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);
  z_view_keyexpr_from_str_unchecked(&ke, KE("led",T_BOOL));
  z_closure_sample(&cl, led_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_led, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);
  z_view_keyexpr_from_str_unchecked(&ke, KE("fan_pwm",T_F32));
  z_closure_sample(&cl, fan_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_fan, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);
  z_view_keyexpr_from_str_unchecked(&ke, KE("motor_trim",T_F32));
  z_closure_sample(&cl, trim_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_trim, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);
#if LINK_RX_TIMEOUT_MS
  static z_owned_subscriber_t sub_ping;
  z_view_keyexpr_from_str_unchecked(&ke, KE("esp32_ping",T_I32));
  z_closure_sample(&cl, ping_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_ping, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);
  g_last_ping_ms = millis(); g_ping_seen = false;   // (re)arm fresh on each (re)connect
#endif

  // Publisher liveliness tokens -> ESP32 shows up as a graph participant so rmw_zenoh
  // subscribers reliably receive its data. eid must be unique per entity.
  for (auto& d : PUBS)
    if (!d.lds_only || LDS_ENABLED) declare_lv(d.topic, d.type, d.lv_eid);

#if LDS_ENABLED
  z_view_keyexpr_from_str_unchecked(&ke, KE("lds_target_rpm",T_F32));
  z_closure_sample(&cl, ldstgt_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_tgt, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);
#endif

  Serial.println("[nano] zenoh CONNECTED");
  return true;
}

// Publishing runs here on Core 0. RX + keepalive are handled by the zenoh-pico read/
// lease tasks; we only PUT (TX-mutex-serialized against them), so nothing blocks.
static void zenohTask(void*){
  Serial.printf("[nano] zenoh task pinned to core %d\n", xPortGetCoreID());
  for(;;){
    if (!ready){ ready = zenohConnect(); if (!ready){ delay(1000); continue; } }

    static uint32_t t_ticks=0, t_lds=0, t_slow=0;
    uint32_t now = millis();

    uint8_t buf[40];
    if (now - t_ticks >= 66){                                // wheel_ticks @~15 Hz (was
                                                              // ~30 Hz; odom integrates
                                                              // cumulative counts, so the
                                                              // faster rate bought nothing
                                                              // but extra SBC executor wakeups)
      t_ticks = now;
      zpub_put(P_ticks, buf, cdr_i64arr2(buf,(int64_t)g_left_ticks,(int64_t)g_right_ticks));
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
      zpub_put(P_suspL,buf, cdr_bool(buf, g_susp_l));
      zpub_put(P_suspR,buf, cdr_bool(buf, g_susp_r));
      zpub_put(P_trim, buf, cdr_f32(buf, g_trim));
    }
    delay(2);
  }
}

// ============================ real-time control (Core 1) ======================
static void writeSide(int chf, int chr, float duty){
  duty = clampf(duty,-1,1);
  float m = fabsf(duty);
  m = (m < MOTOR_DEADZONE) ? 0.0f : MOTOR_MIN_DUTY + m*(1.0f - MOTOR_MIN_DUTY);
  if (duty>=0){ ledcWrite(chr,0); ledcWrite(chf,(uint32_t)(m*PWM_MAX)); }
  else        { ledcWrite(chf,0); ledcWrite(chr,(uint32_t)(m*PWM_MAX)); }
}
static void applyMotors(float l, float r){
  // Straight-line trim: positive trim boosts RIGHT / cuts LEFT (robot was pulling right).
  // Applied pre-remap so it stays monotonic through the stiction compensation; writeSide
  // clamps, so a boosted side saturating just means the cut side does the correcting.
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
  static float integ=0, prev=0; float target=g_lds_target;
  if (target<=0){ integ=0; prev=0; g_lds_duty=0; ledcWrite(CH_LDS,0); return; }
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
struct WPid { float integ, prev; };
static float wheelPid(WPid& st, float tgt, float meas, float dt){
  float err = tgt - meas;
  float deriv = dt>0 ? (err - st.prev)/dt : 0; st.prev = err;
  float u = WHEEL_KFF*tgt + WHEEL_KP*err + WHEEL_KI*st.integ + WHEEL_KD*deriv;
  float duty = clampf(u,-1,1);
  if (duty == u)   // integrate only when not saturated (anti-windup), then clamp the integral
    st.integ = clampf(st.integ + err*dt, -WHEEL_INTEG_MAX, WHEEL_INTEG_MAX);
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

  pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,LOW);
  // SBC cooling fan PWM — off until the SBC link is alive (see FAN_BOOT_DUTY above).
  ledcSetup(CH_FAN,PWM_FREQ_HZ,PWM_RES_BITS); ledcAttachPin(FAN_PIN,CH_FAN);
  ledcWrite(CH_FAN,(uint32_t)(clampf(g_fan_duty,0,1)*PWM_MAX));

  pinMode(LEFT_ENC,INPUT_PULLUP);  attachInterrupt(digitalPinToInterrupt(LEFT_ENC),leftEncISR,RISING);
  pinMode(RIGHT_ENC,INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(RIGHT_ENC),rightEncISR,RISING);
  pinMode(LEFT_SUSPEND_PIN,INPUT_PULLUP); pinMode(RIGHT_SUSPEND_PIN,INPUT_PULLUP);

  // Straight-line trim from NVS (0 until the first calibration drive / manual set).
  g_prefs.begin("nano", false);
  g_trim = g_trim_saved = clampf(g_prefs.getFloat("trim", 0), -TRIM_MAX, TRIM_MAX);
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
  if (now-last_wpid >= (uint32_t)(1000/WHEEL_PID_HZ)){     // wheel velocity PID @WHEEL_PID_HZ
    float dt=(now-last_wpid)/1000.0f; last_wpid=now;
    int32_t l=g_left_ticks, r=g_right_ticks;               // atomic 32-bit reads
    g_left_vel  = (l-wp_l)/TICKS_PER_METER/dt; wp_l=l;
    g_right_vel = (r-wp_r)/TICKS_PER_METER/dt; wp_r=r;
    if (now-g_last_cmd_ms > CMD_TIMEOUT_MS){               // cmd stale: stop + reset integrators
      wpid_l.integ=wpid_l.prev=0; wpid_r.integ=wpid_r.prev=0;
      g_left_duty=0; g_right_duty=0;
    } else {
      g_left_duty  = wheelPid(wpid_l, g_left_tgt,  g_left_vel,  dt);
      g_right_duty = wheelPid(wpid_r, g_right_tgt, g_right_vel, dt);
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
    last_ctl=now;
    if (now-g_last_cmd_ms > CMD_TIMEOUT_MS){ g_left_duty=0; g_right_duty=0; }
    applyMotors(g_left_duty, g_right_duty);
    // Fan tracks true SBC presence (like the LDS park above), not a /cmd_vel-style command
    // watchdog: park it whenever the link isn't alive (boot race, drop, or the SBC genuinely
    // off) since there's no SBC heat to move, and resume the instant sys_monitor reconnects.
    if (alive) ledcWrite(CH_FAN,(uint32_t)(clampf(g_fan_duty,0,1)*PWM_MAX));
    else       { g_fan_duty = 0; ledcWrite(CH_FAN, 0); }
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
  }
#if STATUS_PRINT_MS
  static uint32_t last_dbg=0;
  if (now-last_dbg >= STATUS_PRINT_MS){                     // debug-console health line
    last_dbg=now;
    Serial.printf("[nano] ticks L=%ld R=%ld | trim %+.3f | lds rpm=%.0f hz=%.0f duty=%.2f | susp %d/%d\n",
      (long)g_left_ticks,(long)g_right_ticks, (double)g_trim,
      g_lds_rpm, g_lds_hz, g_lds_duty, (int)g_susp_l,(int)g_susp_r);
#if WHEEL_PID_ENABLED
    Serial.printf("[nano] wheel vel L=%.3f R=%.3f m/s | tgt L=%.3f R=%.3f | duty L=%.2f R=%.2f\n",
      g_left_vel, g_right_vel, g_left_tgt, g_right_tgt, g_left_duty, g_right_duty);
#endif
  }
#endif
  delay(1);
}
