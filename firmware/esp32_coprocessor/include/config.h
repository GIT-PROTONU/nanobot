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

// ---- Quadrature encoders (hardware PCNT via ESP32Encoder = ~0 CPU) -----------
#define LEFT_ENC_A     18
#define LEFT_ENC_B     19
#define RIGHT_ENC_A    16
#define RIGHT_ENC_B    17

// ---- PWM (LEDC) --------------------------------------------------------------
#define PWM_FREQ_HZ    20000   // 20 kHz = inaudible; freq * 2^res must be <= 80 MHz
#define PWM_RES_BITS   10      // duty range 0..1023

// ---- Differential drive (keep in sync with robot.yaml motor_control) ---------
#define WHEEL_SEPARATION   0.16f   // m between wheel contact points
#define MAX_LINEAR_SPEED   0.4f    // m/s mapped to full PWM
#define MAX_ANGULAR_SPEED  3.0f    // rad/s mapped to full PWM
#define CMD_TIMEOUT_MS     500     // stop motors if no cmd_vel within this window

// ---- Wiring sign fixes (flip if a wheel/encoder runs backwards) --------------
#define INVERT_LEFT        false   // motor direction
#define INVERT_RIGHT       false
#define INVERT_LEFT_ENC    false   // encoder count sign
#define INVERT_RIGHT_ENC   false

// ---- Loop rates --------------------------------------------------------------
#define CONTROL_LOOP_HZ    100     // PWM apply + watchdog
#define ENC_PUBLISH_HZ     30      // wheel_ticks publish rate (match SBC odom rate)
