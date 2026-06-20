// Nano ESP32-WROOM coprocessor — NATIVE ZENOH (zenoh-pico), no micro-ROS, no DDS.
// Talks straight to the SBC's rmw_zenoh graph over a direct UART link, in rmw_zenoh's
// exact wire format (Humble / libzenohc 1.9.0). Replaced the old micro-ROS firmware,
// keeping the same topic contract:
//
//   sub  cmd_vel               geometry_msgs/Twist       -> diff-drive -> H-bridge PWM
//   sub  led                   std_msgs/Bool             -> onboard LED
//   sub  lds_target_rpm        std_msgs/Float32          -> LDS spin-speed PID setpoint
//   pub  wheel_ticks           std_msgs/Int64MultiArray  [L,R] raw cumulative counts
//   pub  left/right_wheel_suspended std_msgs/Bool        per-wheel off-ground switch
//   pub  esp32_temp            std_msgs/Float32          die temperature (C)
//   pub  esp32_hall            std_msgs/Int32            internal hall sensor
//   pub  lds_rpm / lds_hz / lds_duty  std_msgs/Float32   spin-lidar speed / framerate / PID duty
//   pub  esp32_heartbeat       std_msgs/Int32            link-alive counter
//
// LINK: ESP32 UART2 (TX=GPIO17, RX=GPIO16) <-> SBC UART1 (/dev/ttyS1). The serial-
// capable zenohd LISTENs there + on TCP for the rest of the rmw_zenoh stack.
//
// MULTICORE: Core 0 runs the ZENOH task (sole owner of the session/serial: zp_read +
// keepalive + all publishes + sub callbacks). Core 1 runs the Arduino loop() = REAL-
// TIME CONTROL (motors, cmd watchdog, LDS PID, sensor sampling). They share state via
// volatiles (32-bit aligned reads/writes are atomic on the ESP32). Only Core 0 ever
// touches the zenoh session — concurrent serial writes corrupt frames (lease timeout).
//
// Three hard-won zenoh-pico notes (see README): use the "serial/UART_2" device locator
// (the pin form skips the link handshake), the begin()-explicit-pins patch, and single-
// thread mode (Z_FEATURE_MULTI_THREAD=0).
#include <Arduino.h>
#include <zenoh-pico.h>
#include <string.h>
#include <math.h>

// ============================ pin / tunable config ============================
// Reassigned vs the micro-ROS build: GPIO16/17 are now the zenoh UART2 link, so the
// LDS data RX moves 16->35 (input-only, RX-only) and MOTOR_STBY moves 17->23.
#define LEFT_IN_FWD   25
#define LEFT_IN_REV    4
#define RIGHT_IN_FWD  32
#define RIGHT_IN_REV  33
#define MOTOR_STBY    23      // was 17 (now UART2 TX)
#define LEFT_ENC      19
#define RIGHT_ENC     26
#define LEFT_SUSPEND_PIN  18
#define RIGHT_SUSPEND_PIN 27
#define SUSPEND_ACTIVE_HIGH true
#define LED_PIN        2
#define LDS_RX_PIN    35      // was 16 (now UART2 RX); LDS data wire -> GPIO35 (UART1 RX)
#define LDS_MOTOR_PIN 21

#define PWM_FREQ_HZ   20000
#define PWM_RES_BITS  10
static const uint32_t PWM_MAX = (1u << PWM_RES_BITS) - 1u;

#define WHEEL_SEPARATION  0.16f
#define MAX_LINEAR_SPEED  0.4f
#define MAX_ANGULAR_SPEED 3.0f
#define CMD_TIMEOUT_MS    500
#define INVERT_LEFT  false
#define INVERT_RIGHT false

#define LDS_BAUD       115200
#define LDS_TIMEOUT_MS 300
#define LDS_TARGET_RPM 300.0f
#define LDS_PID_HZ     50
#define LDS_PID_KFF    0.0020f
#define LDS_PID_KP     0.0010f
#define LDS_PID_KI     0.0015f
#define LDS_PID_KD     0.0f

// LDS DISABLED: its UART1 read (Serial1) is an extra serial peripheral that can steal
// CPU/interrupt time from the zenoh serial link. Set to 1 to re-enable the spin-lidar
// RPM read + PID + lds_rpm/lds_hz/lds_duty pubs + lds_target_rpm sub. All code kept.
#define LDS_ENABLED  0

#define CH_LEFT_FWD  0
#define CH_LEFT_REV  1
#define CH_RIGHT_FWD 2
#define CH_RIGHT_REV 3
#define CH_LDS       4

// ============================ shared cross-core state =========================
static volatile uint32_t g_left_ticks  = 0, g_right_ticks = 0;   // encoder ISR counts
static volatile float    g_left_duty   = 0, g_right_duty   = 0;  // cmd -> motor duty
static volatile uint32_t g_last_cmd_ms = 0;
static volatile float    g_lds_rpm = 0, g_lds_duty = 0, g_lds_hz = 0;
static volatile uint32_t g_lds_frames = 0, g_lds_last_ms = 0;
static volatile float    g_lds_target = LDS_TARGET_RPM;
static volatile float    g_temp = 0;
static volatile int32_t  g_hall = 0;
static volatile bool     g_susp_l = false, g_susp_r = false;
static volatile bool     g_led = false, g_led_dirty = false;

static void IRAM_ATTR leftEncISR()  { g_left_ticks++; }
static void IRAM_ATTR rightEncISR() { g_right_ticks++; }

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
static bool ready = false;
// rmw_zenoh liveliness token = makes a publisher visible in the ROS graph. Format:
// @ros2_lv/<domain>/<zid>/<nid>/<eid>/MP/%/%/<node>/%<topic>/<type>/<typehash>/<qos>
static z_owned_liveliness_token_t g_lv[10]; static int g_lv_n = 0;
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
static ZPub P_ticks, P_suspL, P_suspR, P_temp, P_hall, P_rpm, P_hz, P_duty, P_hb;

static void zpub_declare(ZPub& zp, const char* keyexpr, uint8_t tag){
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

// --- subscription callbacks (run on Core 0 during zp_read) ---
static void cmd_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[64]; size_t n = sample_bytes(sm, b, sizeof(b));
  if (n < 52) return;                       // hdr(4) + 6*f64(48); align from body start
  double v, w;
  memcpy(&v, b+4,  8);                       // linear.x  (body offset 0)
  memcpy(&w, b+44, 8);                       // angular.z (body offset 40)
  float fv = clampf((float)v, -MAX_LINEAR_SPEED,  MAX_LINEAR_SPEED);
  float fw = clampf((float)w, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
  float vl = fv - fw*WHEEL_SEPARATION*0.5f, vr = fv + fw*WHEEL_SEPARATION*0.5f;
  float mx = MAX_LINEAR_SPEED + MAX_ANGULAR_SPEED*WHEEL_SEPARATION*0.5f;
  g_left_duty  = mx ? clampf(vl/mx,-1,1) : 0;
  g_right_duty = mx ? clampf(vr/mx,-1,1) : 0;
  g_last_cmd_ms = millis();
}
static void led_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 5){ g_led = b[4]!=0; g_led_dirty = true; }
}
static void ldstgt_cb(z_loaned_sample_t* sm, void*){
  uint8_t b[8]; if (sample_bytes(sm,b,sizeof(b)) >= 8){ float f; memcpy(&f,b+4,4); g_lds_target = f>0?f:0; }
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

  zpub_declare(P_ticks,KE("wheel_ticks",T_I64A),1);
  zpub_declare(P_suspL,KE("left_wheel_suspended",T_BOOL),2);
  zpub_declare(P_suspR,KE("right_wheel_suspended",T_BOOL),3);
  zpub_declare(P_temp, KE("esp32_temp",T_F32),4);
  zpub_declare(P_hall, KE("esp32_hall",T_I32),5);
#if LDS_ENABLED
  zpub_declare(P_rpm,  KE("lds_rpm",T_F32),6);
  zpub_declare(P_hz,   KE("lds_hz",T_F32),7);
  zpub_declare(P_duty, KE("lds_duty",T_F32),8);
#endif
  zpub_declare(P_hb,   KE("esp32_heartbeat",T_I32),9);

  static z_owned_subscriber_t sub_cmd, sub_led, sub_tgt;   // kept alive (static)
  z_owned_closure_sample_t cl;
  z_view_keyexpr_t ke;
  z_view_keyexpr_from_str_unchecked(&ke, KE("cmd_vel",T_TWIST));
  z_closure_sample(&cl, cmd_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_cmd, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);
  z_view_keyexpr_from_str_unchecked(&ke, KE("led",T_BOOL));
  z_closure_sample(&cl, led_cb, NULL, NULL);
  z_declare_subscriber(z_session_loan(&s), &sub_led, z_view_keyexpr_loan(&ke), z_closure_sample_move(&cl), NULL);

  // Publisher liveliness tokens -> ESP32 shows up as a graph participant so rmw_zenoh
  // subscribers reliably receive its data. eid must be unique per entity.
  declare_lv("wheel_ticks",T_I64A,1);
  declare_lv("left_wheel_suspended",T_BOOL,2);
  declare_lv("right_wheel_suspended",T_BOOL,3);
  declare_lv("esp32_temp",T_F32,4);
  declare_lv("esp32_hall",T_I32,5);
  declare_lv("esp32_heartbeat",T_I32,6);
#if LDS_ENABLED
  declare_lv("lds_rpm",T_F32,7);
  declare_lv("lds_hz",T_F32,8);
  declare_lv("lds_duty",T_F32,9);
#endif
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
  for(;;){
    if (!ready){ ready = zenohConnect(); if (!ready){ delay(1000); continue; } }

    static uint32_t t_ticks=0, t_lds=0, t_slow=0;
    uint32_t now = millis();

    uint8_t buf[40];
    if (now - t_ticks >= 33){                                // wheel_ticks @~30 Hz
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
    }
    delay(2);
  }
}

// ============================ real-time control (Core 1) ======================
static void writeSide(int chf, int chr, float duty){
  duty = clampf(duty,-1,1);
  if (duty>=0){ ledcWrite(chr,0); ledcWrite(chf,(uint32_t)(duty*PWM_MAX)); }
  else        { ledcWrite(chf,0); ledcWrite(chr,(uint32_t)(-duty*PWM_MAX)); }
}
static void applyMotors(float l, float r){
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
static bool debounceSusp(int pin, bool& cand, uint8_t& stable, bool cur){
  bool lvl = digitalRead(pin)==HIGH, susp = SUSPEND_ACTIVE_HIGH?lvl:!lvl;
  if (susp==cand){ if(stable<3) stable++; } else { cand=susp; stable=0; }
  return (stable>=2)?cand:cur;
}

void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n[nano] zenoh-pico coprocessor boot");

  for (int c=0;c<4;c++) ledcSetup(c,PWM_FREQ_HZ,PWM_RES_BITS);
  ledcAttachPin(LEFT_IN_FWD,CH_LEFT_FWD); ledcAttachPin(LEFT_IN_REV,CH_LEFT_REV);
  ledcAttachPin(RIGHT_IN_FWD,CH_RIGHT_FWD); ledcAttachPin(RIGHT_IN_REV,CH_RIGHT_REV);
  pinMode(MOTOR_STBY,OUTPUT); digitalWrite(MOTOR_STBY,HIGH);
  applyMotors(0,0);
  pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,LOW);

  pinMode(LEFT_ENC,INPUT_PULLUP);  attachInterrupt(digitalPinToInterrupt(LEFT_ENC),leftEncISR,RISING);
  pinMode(RIGHT_ENC,INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(RIGHT_ENC),rightEncISR,RISING);
  pinMode(LEFT_SUSPEND_PIN,INPUT_PULLUP); pinMode(RIGHT_SUSPEND_PIN,INPUT_PULLUP);

#if LDS_ENABLED
  // LDS data on UART1 remapped to GPIO35 (RX-only); UART2 is the zenoh link.
  Serial1.begin(LDS_BAUD, SERIAL_8N1, LDS_RX_PIN, -1);
  ledcSetup(CH_LDS,PWM_FREQ_HZ,PWM_RES_BITS); ledcAttachPin(LDS_MOTOR_PIN,CH_LDS);
#endif

  g_last_cmd_ms = millis();
  // Zenoh session owns Core 0; control loop() runs on Core 1.
  xTaskCreatePinnedToCore(zenohTask, "zenoh", 16384, NULL, 5, NULL, 0);
}

void loop(){   // Core 1: real-time control
#if LDS_ENABLED
  while (Serial1.available()) ldsFeed((uint8_t)Serial1.read());
#endif

  static uint32_t last_pid=0, last_ctl=0, last_sens=0;
  uint32_t now = millis();

#if LDS_ENABLED
  if (now-last_pid >= (uint32_t)(1000/LDS_PID_HZ)){ ldsControl((now-last_pid)/1000.0f); last_pid=now; }
#else
  (void)last_pid;
#endif

  if (now-last_ctl >= 10){                                  // motors + watchdog @100 Hz
    last_ctl=now;
    if (now-g_last_cmd_ms > CMD_TIMEOUT_MS){ g_left_duty=0; g_right_duty=0; }
    applyMotors(g_left_duty, g_right_duty);
  }
  if (now-last_sens >= 100){                                // sensors @10 Hz
    last_sens=now;
    static bool cl=false,cr=false; static uint8_t sl=0,sr=0;
    g_susp_l = debounceSusp(LEFT_SUSPEND_PIN, cl, sl, g_susp_l);
    g_susp_r = debounceSusp(RIGHT_SUSPEND_PIN,cr, sr, g_susp_r);
    g_temp = temperatureRead();
    g_hall = hallRead();
    if (g_led_dirty){ digitalWrite(LED_PIN, g_led?HIGH:LOW); g_led_dirty=false; }
#if LDS_ENABLED
    // compute LDS frame-rate (Hz)
    static uint32_t lf=0, lm=0;
    uint32_t f=g_lds_frames; g_lds_hz = (lm && now>lm)?(f-lf)*1000.0f/(now-lm):0; lf=f; lm=now;
#endif
  }
  delay(1);
}
