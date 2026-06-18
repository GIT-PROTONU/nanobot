// Nano robot ESP32-WROOM motor/encoder coprocessor (micro-ROS, serial transport).
//
//   sub  cmd_vel        geometry_msgs/Twist      -> diff-drive mix -> H-bridge PWM
//   sub  led            std_msgs/Bool            -> onboard LED (pipeline test)
//   pub  wheel_ticks    std_msgs/Int64MultiArray [left, right] raw cumulative counts
//   pub  left_wheel_suspended  std_msgs/Bool     left wheel off the ground (switch)
//   pub  right_wheel_suspended std_msgs/Bool     right wheel off the ground (switch)
//   pub  esp32_temp    std_msgs/Float32         ESP32 internal die temperature (deg C)
//   pub  esp32_hall    std_msgs/Int32           ESP32 internal hall sensor (raw)
//   pub  lds_rpm       std_msgs/Float32         spin-lidar speed (RPM, from UART2)
//   sub  lds_motor     std_msgs/Float32         LDS spin-motor PWM duty [0..1]
//
// Real-time work (single-channel encoder edge-counting, PWM, cmd watchdog) lives
// here so the SBC is offloaded. The SBC's wheel_odometry samples wheel_ticks on
// its own timer and integrates odom/TF exactly as it did from GPIO before.
//
// Design notes:
//  * Standard messages only -> any stock micro_ros_agent bridges it, no custom
//    type rebuild on the agent host.
//  * No MCU<->agent time sync: the SBC stamps + computes dt itself, so we never
//    call rmw_uros_sync_session (saves a round-trip and keeps the link quiet).
//  * Connection FSM auto-reconnects if the agent restarts, and coasts the motors
//    whenever the link is down.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp_bt.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

// ---- LEDC channel assignment (one per H-bridge input) ------------------------
#define CH_LEFT_FWD   0
#define CH_LEFT_REV   1
#define CH_RIGHT_FWD  2
#define CH_RIGHT_REV  3
#define CH_LDS        4   // LDS spin-motor PWM
static const uint32_t PWM_MAX = (1u << PWM_RES_BITS) - 1u;

// ---- micro-ROS entities ------------------------------------------------------
rcl_subscription_t cmd_sub;
rcl_subscription_t led_sub;
#if LDS_MOTOR_PIN >= 0
rcl_subscription_t lds_motor_sub;
#endif
rcl_publisher_t    enc_pub;
rcl_publisher_t    left_susp_pub;
#if RIGHT_SUSPEND_PIN >= 0
rcl_publisher_t    right_susp_pub;
#endif
rcl_publisher_t    temp_pub;
rcl_publisher_t    hall_pub;
rcl_publisher_t    lds_rpm_pub;
rcl_timer_t        control_timer;
rcl_timer_t        enc_timer;
rclc_executor_t    executor;
rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;

geometry_msgs__msg__Twist           cmd_msg;
std_msgs__msg__Bool                 led_msg;
std_msgs__msg__Int64MultiArray      enc_msg;
static int64_t                      enc_data[2];   // backing store for enc_msg.data
std_msgs__msg__Bool                 left_susp_msg;
#if RIGHT_SUSPEND_PIN >= 0
std_msgs__msg__Bool                 right_susp_msg;
#endif
std_msgs__msg__Float32              temp_msg;
std_msgs__msg__Int32                hall_msg;
std_msgs__msg__Float32              lds_rpm_msg;
#if LDS_MOTOR_PIN >= 0
std_msgs__msg__Float32              lds_motor_msg;
#endif

// ---- shared control state ----------------------------------------------------
// Single-channel encoders: each ISR just bumps a 32-bit counter (atomic to read
// on the ESP32, so no locking needed). Unsigned — no direction information.
static volatile uint32_t g_left_ticks  = 0;
static volatile uint32_t g_right_ticks = 0;
static volatile float    g_left_duty   = 0.0f;   // last commanded duty, [-1, 1]
static volatile float    g_right_duty  = 0.0f;
static volatile uint32_t g_last_cmd_ms = 0;

static void IRAM_ATTR leftEncISR()  { g_left_ticks++; }
#if RIGHT_ENC >= 0
static void IRAM_ATTR rightEncISR() { g_right_ticks++; }
#endif

static volatile float g_lds_rpm = 0.0f;   // latest valid spin-lidar speed (RPM)

// Minimal LDS02RR frame parser — extracts RPM only (scan data ignored). 22-byte
// packets: 0xFA, index, speed_lo, speed_hi, 16 data, chk_lo, chk_hi. RPM = speed/64.
// Checksum (same as the SBC lds_driver_py) is validated so noise can't fake an RPM.
static void ldsFeed(uint8_t byte) {
  static uint8_t pkt[22];
  static uint8_t len = 0;
  if (len == 0 && byte != 0xFA) return;     // hunt for the start byte
  pkt[len++] = byte;
  if (len < 22) return;
  len = 0;                                  // full frame captured; reset for next
  uint32_t chk = 0;
  for (int ix = 0; ix < 20; ix += 2)
    chk = (chk * 2u + pkt[ix] + (pkt[ix + 1] << 8)) & 0xFFFFFFFFu;
  uint32_t cs = ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF;
  if ((cs & 0xFF) == pkt[20] && ((cs >> 8) & 0xFF) == pkt[21])
    g_lds_rpm = ((pkt[3] << 8) | pkt[2]) / 64.0f;
}

// ---- connection state machine ------------------------------------------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState state = WAITING_AGENT;

#define RCCHECK(fn)     { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t last = -1; \
    if (last == -1) last = uxr_millis(); \
    if ((int64_t)uxr_millis() - last > (MS)) { X; last = uxr_millis(); } \
  } while (0)

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---- motor output ------------------------------------------------------------
// duty in [-1, 1]; sign = direction. Forward channel carries duty, reverse 0
// (and vice-versa) — slow-decay PWM, matches the old PCA9685 motor_node.
static void writeSide(int ch_fwd, int ch_rev, float duty) {
  duty = clampf(duty, -1.0f, 1.0f);
  if (duty >= 0.0f) {
    ledcWrite(ch_rev, 0);
    ledcWrite(ch_fwd, (uint32_t)(duty * PWM_MAX));
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, (uint32_t)(-duty * PWM_MAX));
  }
}

static void applyMotors(float left, float right) {
  writeSide(CH_LEFT_FWD,  CH_LEFT_REV,  INVERT_LEFT  ? -left  : left);
  writeSide(CH_RIGHT_FWD, CH_RIGHT_REV, INVERT_RIGHT ? -right : right);
}

static void stopMotors() { applyMotors(0.0f, 0.0f); }

// ---- callbacks ---------------------------------------------------------------
// /cmd_vel -> per-wheel duty (same kinematics as the SBC motor_node).
static void cmd_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = clampf((float)m->linear.x,  -MAX_LINEAR_SPEED,  MAX_LINEAR_SPEED);
  float w = clampf((float)m->angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
  float vl = v - w * WHEEL_SEPARATION * 0.5f;
  float vr = v + w * WHEEL_SEPARATION * 0.5f;
  float max_wheel = MAX_LINEAR_SPEED + MAX_ANGULAR_SPEED * WHEEL_SEPARATION * 0.5f;
  g_left_duty  = max_wheel ? clampf(vl / max_wheel, -1.0f, 1.0f) : 0.0f;
  g_right_duty = max_wheel ? clampf(vr / max_wheel, -1.0f, 1.0f) : 0.0f;
  g_last_cmd_ms = millis();
}

// /led -> drive the onboard LED. Standard Bool message so a stock agent bridges
// it; handy for an end-to-end "is the whole pipeline alive?" check.
#if LED_PIN >= 0
static void led_cb(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, m->data ? HIGH : LOW);
}
#endif

// /lds_motor -> LDS spin-motor PWM duty [0..1] (open-loop; clamped).
#if LDS_MOTOR_PIN >= 0
static void lds_motor_cb(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  ledcWrite(CH_LDS, (uint32_t)(clampf(m->data, 0.0f, 1.0f) * PWM_MAX));
}
#endif

// Control tick: apply latest command, or coast if cmd_vel went stale (watchdog).
static void control_cb(rcl_timer_t *, int64_t) {
  if (millis() - g_last_cmd_ms > CMD_TIMEOUT_MS) {
    g_left_duty = g_right_duty = 0.0f;
  }
  applyMotors(g_left_duty, g_right_duty);
}

// Debounce one suspension microswitch (needs a couple of stable samples to flip)
// and publish on change, plus a ~1 s heartbeat so a late subscriber syncs. State
// is carried in the caller's statics so each switch tracks independently.
struct SuspendState { bool published; bool cand; uint8_t stable; uint16_t since_pub; };
static void serviceSuspend(int pin, rcl_publisher_t *pub,
                           std_msgs__msg__Bool *msg, SuspendState *s) {
  bool lvl = (digitalRead(pin) == HIGH);
  bool suspended = SUSPEND_ACTIVE_HIGH ? lvl : !lvl;
  if (suspended == s->cand) { if (s->stable < 3) s->stable++; }
  else                      { s->cand = suspended; s->stable = 0; }
  s->since_pub++;
  if ((s->stable >= 2 && s->cand != s->published) || s->since_pub >= ENC_PUBLISH_HZ) {
    s->published = s->cand;
    msg->data = s->published;
    RCSOFTCHECK(rcl_publish(pub, msg, NULL));
    s->since_pub = 0;
  }
}

// Publish raw cumulative encoder counts, plus each suspension switch state.
static void enc_cb(rcl_timer_t *timer, int64_t) {
  if (timer == NULL) return;
  enc_data[0] = (int64_t)g_left_ticks;
  enc_data[1] = (int64_t)g_right_ticks;
  RCSOFTCHECK(rcl_publish(&enc_pub, &enc_msg, NULL));

  static SuspendState ls = {false, false, 0, 0xFFFF};  // force first publish
  serviceSuspend(LEFT_SUSPEND_PIN, &left_susp_pub, &left_susp_msg, &ls);
#if RIGHT_SUSPEND_PIN >= 0
  static SuspendState rs = {false, false, 0, 0xFFFF};
  serviceSuspend(RIGHT_SUSPEND_PIN, &right_susp_pub, &right_susp_msg, &rs);
#endif

  // Slow on-die telemetry at ~1 Hz (no need for 30 Hz): internal temperature
  // and hall sensor. Both use internal sensors only — no GPIO/pin conflicts.
  static uint16_t slow_div = ENC_PUBLISH_HZ;  // publish on the first tick
  if (++slow_div >= ENC_PUBLISH_HZ) {
    slow_div = 0;
    temp_msg.data = temperatureRead();
    RCSOFTCHECK(rcl_publish(&temp_pub, &temp_msg, NULL));
    hall_msg.data = hallRead();
    RCSOFTCHECK(rcl_publish(&hall_pub, &hall_msg, NULL));
  }

  // Spin-lidar RPM at ~5 Hz (more dynamic than temp/hall — useful for tuning speed).
  static uint8_t rpm_div = 0;
  if (++rpm_div >= ENC_PUBLISH_HZ / 5) {
    rpm_div = 0;
    lds_rpm_msg.data = g_lds_rpm;
    RCSOFTCHECK(rcl_publish(&lds_rpm_pub, &lds_rpm_msg, NULL));
  }
}

// ---- entity lifecycle (created on connect, destroyed on disconnect) ----------
static bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_coprocessor", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

#if LED_PIN >= 0
  RCCHECK(rclc_subscription_init_default(
      &led_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "led"));
#endif
#if LDS_MOTOR_PIN >= 0
  RCCHECK(rclc_subscription_init_default(
      &lds_motor_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "lds_motor"));
#endif

  // Best-effort: high-rate sensor stream, no point ACKing each sample over serial.
  // The SBC subscriber must match (best_effort) or it won't see these.
  RCCHECK(rclc_publisher_init_best_effort(
      &enc_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray), "wheel_ticks"));

  // Reliable (low-rate state): publish-on-change + heartbeat, so it must arrive.
  RCCHECK(rclc_publisher_init_default(
      &left_susp_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "left_wheel_suspended"));
#if RIGHT_SUSPEND_PIN >= 0
  RCCHECK(rclc_publisher_init_default(
      &right_susp_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "right_wheel_suspended"));
#endif

  RCCHECK(rclc_publisher_init_default(
      &temp_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "esp32_temp"));
  RCCHECK(rclc_publisher_init_default(
      &hall_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32_hall"));
  RCCHECK(rclc_publisher_init_default(
      &lds_rpm_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "lds_rpm"));

  RCCHECK(rclc_timer_init_default(
      &control_timer, &support, RCL_MS_TO_NS(1000 / CONTROL_LOOP_HZ), control_cb));
  RCCHECK(rclc_timer_init_default(
      &enc_timer, &support, RCL_MS_TO_NS(1000 / ENC_PUBLISH_HZ), enc_cb));

  // Handles: cmd_sub + control_timer + enc_timer, plus led_sub / lds_motor_sub.
  size_t num_handles = 3;
#if LED_PIN >= 0
  num_handles += 1;
#endif
#if LDS_MOTOR_PIN >= 0
  num_handles += 1;
#endif
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_sub, &cmd_msg, &cmd_cb, ON_NEW_DATA));
#if LED_PIN >= 0
  RCCHECK(rclc_executor_add_subscription(
      &executor, &led_sub, &led_msg, &led_cb, ON_NEW_DATA));
#endif
#if LDS_MOTOR_PIN >= 0
  RCCHECK(rclc_executor_add_subscription(
      &executor, &lds_motor_sub, &lds_motor_msg, &lds_motor_cb, ON_NEW_DATA));
#endif
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &enc_timer));
  return true;
}

static void destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&cmd_sub, &node);
#if LED_PIN >= 0
  rcl_subscription_fini(&led_sub, &node);
#endif
#if LDS_MOTOR_PIN >= 0
  rcl_subscription_fini(&lds_motor_sub, &node);
#endif
  rcl_publisher_fini(&enc_pub, &node);
  rcl_publisher_fini(&left_susp_pub, &node);
#if RIGHT_SUSPEND_PIN >= 0
  rcl_publisher_fini(&right_susp_pub, &node);
#endif
  rcl_publisher_fini(&temp_pub, &node);
  rcl_publisher_fini(&hall_pub, &node);
  rcl_publisher_fini(&lds_rpm_pub, &node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&enc_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ---- setup / loop ------------------------------------------------------------
void setup() {
  // Radios are unused — the micro-ROS link is USB serial. WiFi and Bluetooth are
  // never initialized, so the RF stays powered down (cuts current draw + RF noise
  // near the motor/encoder wiring). We deliberately do NOT pull in <WiFi.h> just to
  // call WIFI_OFF — it links the whole stack (~600 KB flash). Releasing the BT
  // controller's reserved RAM back to the heap makes "BT off" explicit too.
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

  // Serial0 (USB) is the micro-ROS transport.
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // LEDC PWM on the four H-bridge inputs.
  for (int ch = 0; ch < 4; ++ch) ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(LEFT_IN_FWD,  CH_LEFT_FWD);
  ledcAttachPin(LEFT_IN_REV,  CH_LEFT_REV);
  ledcAttachPin(RIGHT_IN_FWD, CH_RIGHT_FWD);
  ledcAttachPin(RIGHT_IN_REV, CH_RIGHT_REV);
#if MOTOR_STBY >= 0
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);   // enable the H-bridge
#endif
  stopMotors();

#if LED_PIN >= 0
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
#endif

  // Single-channel encoders: count rising edges via GPIO interrupts (pull-ups on).
  pinMode(LEFT_ENC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC), leftEncISR, RISING);
#if RIGHT_ENC >= 0
  pinMode(RIGHT_ENC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC), rightEncISR, RISING);
#endif

  // Wheel-suspension microswitches (read in enc_cb).
  pinMode(LEFT_SUSPEND_PIN, INPUT_PULLUP);
#if RIGHT_SUSPEND_PIN >= 0
  pinMode(RIGHT_SUSPEND_PIN, INPUT_PULLUP);
#endif

  // Spin lidar: UART2 RX-only for the data stream (we parse RPM in loop()). Bigger
  // RX buffer so a slow loop pass doesn't drop bytes at 115200.
  Serial2.setRxBufferSize(512);
  Serial2.begin(LDS_BAUD, SERIAL_8N1, LDS_RX_PIN, -1);
#if LDS_MOTOR_PIN >= 0
  ledcSetup(CH_LDS, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(LDS_MOTOR_PIN, CH_LDS);
  ledcWrite(CH_LDS, (uint32_t)(LDS_MOTOR_DUTY * PWM_MAX));   // start spinning
#endif

  // Int64MultiArray payload points at our static buffer (no dynamic alloc).
  enc_msg.data.data = enc_data;
  enc_msg.data.size = 2;
  enc_msg.data.capacity = 2;
  enc_msg.layout.dim.data = NULL;
  enc_msg.layout.dim.size = 0;
  enc_msg.layout.dim.capacity = 0;
  enc_msg.layout.data_offset = 0;

  g_last_cmd_ms = millis();
}

void loop() {
  // Always drain the LDS UART so its buffer never overflows (independent of the
  // agent link); ldsFeed() updates g_lds_rpm from valid frames.
  while (Serial2.available()) ldsFeed((uint8_t)Serial2.read());

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE
                                                            : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED
                                                            : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;

    case AGENT_DISCONNECTED:
      stopMotors();
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}
