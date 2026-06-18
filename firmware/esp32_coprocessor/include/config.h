// All firmware tunables in one place (mirrors robot.yaml's motor_control block).
// Pins are ESP32-WROOM-32 GPIO numbers. Avoid the flash pins 6-11; input-only
// pins 34-39 have NO internal pull-up, so encoders use pull-up-capable GPIOs.
#pragma once

// ---- Motor H-bridge (DRV8833 / TB6612-style: two PWM inputs per motor) -------
// Forward = PWM on INx_FWD, INx_REV held low; reverse swaps them (slow-decay).
#define LEFT_IN_FWD    25
#define LEFT_IN_REV    26
#define RIGHT_IN_FWD   32
#define RIGHT_IN_REV   33
#define MOTOR_STBY     27   // TB6612 STBY (HIGH=enable). -1 if your driver lacks it.

// ---- Wheel encoders (single-channel, rising-edge count via GPIO interrupt) ----
// One signal wire per wheel: counts rising edges only (unsigned magnitude — no
// direction). Lines use internal pull-ups. Only the left wheel is wired for now;
// set RIGHT_ENC to its GPIO to enable the right wheel (its count stays 0 until).
#define LEFT_ENC       19
#define RIGHT_ENC      -1      // not wired yet; set to a GPIO to enable

// ---- Wheel-suspension microswitch --------------------------------------------
// INPUT_PULLUP; tells whether the wheel is suspended (off the ground). Published
// as std_msgs/Bool on /wheel_suspended (true = suspended).
#define SUSPEND_PIN          18
#define SUSPEND_ACTIVE_HIGH  true   // pin reads HIGH when suspended (flip if inverted)

// ---- PWM (LEDC) --------------------------------------------------------------
#define PWM_FREQ_HZ    20000   // 20 kHz = inaudible; freq * 2^res must be <= 80 MHz
#define PWM_RES_BITS   10      // duty range 0..1023

// ---- Differential drive (keep in sync with robot.yaml motor_control) ---------
#define WHEEL_SEPARATION   0.16f   // m between wheel contact points
#define MAX_LINEAR_SPEED   0.4f    // m/s mapped to full PWM
#define MAX_ANGULAR_SPEED  3.0f    // rad/s mapped to full PWM
#define CMD_TIMEOUT_MS     500     // stop motors if no cmd_vel within this window

// ---- Wiring sign fixes (flip if a wheel runs backwards) ----------------------
#define INVERT_LEFT        false   // motor direction
#define INVERT_RIGHT       false

// ---- Loop rates --------------------------------------------------------------
#define CONTROL_LOOP_HZ    100     // PWM apply + watchdog
#define ENC_PUBLISH_HZ     30      // wheel_ticks publish rate (match SBC odom rate)

// ---- Built-in LED (end-to-end test of the agent->graph->firmware path) -------
// On most ESP32-WROOM-32 dev boards the onboard LED is GPIO2. Driven by the /led
// Bool topic (true=on). GPIO2 is a boot strapping pin but is free to drive after
// boot. Set to -1 if your board has no usable onboard LED.
#define LED_PIN            2
