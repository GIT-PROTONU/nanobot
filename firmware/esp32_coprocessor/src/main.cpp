// Nano robot ESP32-WROOM motor/encoder coprocessor (micro-ROS, serial transport).
//
//   sub  cmd_vel     geometry_msgs/Twist       -> diff-drive mix -> H-bridge PWM
//   pub  wheel_ticks std_msgs/Int64MultiArray  [left, right] raw cumulative counts
//
// Real-time work (quadrature decode via hardware PCNT, PWM, cmd watchdog) lives
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

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int64_multi_array.h>

#include <ESP32Encoder.h>

#include "config.h"

// ---- LEDC channel assignment (one per H-bridge input) ------------------------
#define CH_LEFT_FWD   0
#define CH_LEFT_REV   1
#define CH_RIGHT_FWD  2
#define CH_RIGHT_REV  3
static const uint32_t PWM_MAX = (1u << PWM_RES_BITS) - 1u;

// ---- micro-ROS entities ------------------------------------------------------
rcl_subscription_t cmd_sub;
rcl_publisher_t    enc_pub;
rcl_timer_t        control_timer;
rcl_timer_t        enc_timer;
rclc_executor_t    executor;
rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;

geometry_msgs__msg__Twist           cmd_msg;
std_msgs__msg__Int64MultiArray      enc_msg;
static int64_t                      enc_data[2];   // backing store for enc_msg.data

// ---- shared control state ----------------------------------------------------
ESP32Encoder enc_left;
ESP32Encoder enc_right;
static volatile float   g_left_duty  = 0.0f;   // last commanded duty, [-1, 1]
static volatile float   g_right_duty = 0.0f;
static volatile uint32_t g_last_cmd_ms = 0;

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

// Control tick: apply latest command, or coast if cmd_vel went stale (watchdog).
static void control_cb(rcl_timer_t *, int64_t) {
  if (millis() - g_last_cmd_ms > CMD_TIMEOUT_MS) {
    g_left_duty = g_right_duty = 0.0f;
  }
  applyMotors(g_left_duty, g_right_duty);
}

// Publish raw cumulative encoder counts.
static void enc_cb(rcl_timer_t *timer, int64_t) {
  if (timer == NULL) return;
  int64_t l = enc_left.getCount();
  int64_t r = enc_right.getCount();
  enc_data[0] = INVERT_LEFT_ENC  ? -l : l;
  enc_data[1] = INVERT_RIGHT_ENC ? -r : r;
  RCSOFTCHECK(rcl_publish(&enc_pub, &enc_msg, NULL));
}

// ---- entity lifecycle (created on connect, destroyed on disconnect) ----------
static bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_coprocessor", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // Best-effort: high-rate sensor stream, no point ACKing each sample over serial.
  // The SBC subscriber must match (best_effort) or it won't see these.
  RCCHECK(rclc_publisher_init_best_effort(
      &enc_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray), "wheel_ticks"));

  RCCHECK(rclc_timer_init_default(
      &control_timer, &support, RCL_MS_TO_NS(1000 / CONTROL_LOOP_HZ), control_cb));
  RCCHECK(rclc_timer_init_default(
      &enc_timer, &support, RCL_MS_TO_NS(1000 / ENC_PUBLISH_HZ), enc_cb));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_sub, &cmd_msg, &cmd_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &enc_timer));
  return true;
}

static void destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_subscription_fini(&cmd_sub, &node);
  rcl_publisher_fini(&enc_pub, &node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&enc_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ---- setup / loop ------------------------------------------------------------
void setup() {
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

  // Hardware quadrature decode (PCNT). Internal pull-ups on the encoder lines.
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_left.attachFullQuad(LEFT_ENC_A, LEFT_ENC_B);
  enc_right.attachFullQuad(RIGHT_ENC_A, RIGHT_ENC_B);
  enc_left.clearCount();
  enc_right.clearCount();

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
