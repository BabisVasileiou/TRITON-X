/********** TRITON-X UNO configuration
 * Purpose
 *   Defines the fixed hardware allocation and calibrated constants for
 *   the integrated five-mode TRITON-X Arduino UNO firmware.
 * Hardware
 *   Arduino UNO R3, Sensor Shield v5.0, L298N, HC-SR04, SG90,
 *   PCF8574T, LCD1602 I2C, MPU-6050 and two wheel encoders.
 * Software
 *   Constants are stored as compile-time values. No dynamic allocation
 *   is used.
 * Safety
 *   Motor and cliff settings match the validated standalone tests.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_CONFIGURATION_H
#define TRITON_X_CONFIGURATION_H

#include <Arduino.h>

/* Pin allocation. */
const uint8_t CLIFF_RF_PIN = 2;
const uint8_t CLIFF_LF_PIN = 3;
const uint8_t ESP_RX_PIN = 4;
const uint8_t ENA_PIN = 5;
const uint8_t ENB_PIN = 6;
const uint8_t OBST_FL_PIN = 7;
const uint8_t TRIG_PIN = 8;
const uint8_t ECHO_PIN = 9;
const uint8_t ENC_LEFT_PIN = 10;
const uint8_t SERVO_PIN = 11;
const uint8_t ENC_RIGHT_PIN = 12;
const uint8_t ESP_TX_PIN = 13;

const uint8_t IN4_PIN = A0;
const uint8_t IN3_PIN = A1;
const uint8_t IN2_PIN = A2;
const uint8_t IN1_PIN = A3;

/* I2C addresses and PCF8574 bit allocation. */
const uint8_t PCF8574_ADDR = 0x20;
const uint8_t LCD_ADDR = 0x27;
const uint8_t MPU6050_ADDR = 0x68;

const uint8_t BIT_OBST_FR = 0;
const uint8_t BIT_OBST_RL = 1;
const uint8_t BIT_OBST_RR = 2;
const uint8_t BIT_CLIFF_RL = 3;
const uint8_t BIT_CLIFF_RR = 4;

/* Mechanical calibration. */
const float WHEEL_DIAMETER_CM = 6.5f;
const float WHEEL_TRACK_CM = 14.3f;
/* Empirically calibrated: 30.7 cm real vs 35.7 cm reported
 * with the nominal 20.0 value (July 2026 straight-line test). */
const float ENCODER_PULSES_PER_REV = 23.26f;
const float DISTANCE_PER_PULSE_CM =
  (PI * WHEEL_DIAMETER_CM) / ENCODER_PULSES_PER_REV;
const float GYRO_Z_SIGN = 1.0f;

/* Common motion limits. */
const int MAX_MOTOR_PWM = 140;
const int MANUAL_SPEED_PWM = 120;
const int LEVEL_SPEED_PWM = 120;
const int LINE_SPEED_PWM = 110;
const int OBSTACLE_SPEED_PWM = 120;
const int MAPPING_CRUISE_PWM = 105;
const int MAPPING_TURN_PWM = 95;
const int ESCAPE_PWM = 70;
const int CLIFF_TURN_PWM = 90;

/* Manual and level control. */
const int JOYSTICK_DEADZONE = 7;
const int TILT_COMMAND_DEADZONE = 5;
const unsigned long COMMAND_TIMEOUT_MS = 500UL;

/* Sensor filtering and system communication. */
const unsigned long SENSOR_SAMPLE_MS = 5UL;
const unsigned long INTERRUPT_CONFIRM_MS = 40UL;
const uint8_t SENSOR_DEBOUNCE_SAMPLES = 3;
const uint8_t PCF_RECOVERY_SAMPLES = 5;
const unsigned long LINK_TIMEOUT_MS = 1500UL;
const unsigned long SYSTEM_TELEMETRY_MS = 250UL;

/* Cliff recovery, validated by MANUAL and LEVEL v1.2. */
const unsigned long SAFETY_STOP_MS = 120UL;
const unsigned long CLIFF_LINEAR_ESCAPE_MS = 550UL;
const unsigned long CLIFF_TURN_MS = 1200UL;
const unsigned long CLIFF_RECOVERY_MAX_MS = 6000UL;
const unsigned long POST_RECOVERY_HOLD_MS = 250UL;
const unsigned long CLIFF_CLEAR_RELEASE_MS = 400UL;

/* LINE mode, validated standalone values. */
const float LINE_KP = 1.5f;
const float LINE_KD = 0.3f;
const float LINE_OBSTACLE_STOP_CM = 15.0f;
const unsigned long LINE_SONAR_INTERVAL_MS = 80UL;

/* OBSTACLE mode, validated standalone values. */
const float OBSTACLE_SAFE_DIST_CM = 25.0f;
const float OBSTACLE_CRIT_DIST_CM = 10.0f;
const unsigned long OBSTACLE_SERVO_DELAY_MS = 250UL;
const uint8_t OBSTACLE_ANTI_LOOP_TURNS = 3;

/* Mapping calibration and navigation. */
const int HEADING_CORR_MAX_PWM = 30;
const float HEADING_HOLD_KP = 1.8f;
const float TURN_TOLERANCE_DEG = 5.0f;
const float MAP_SAFE_FRONT_CM = 28.0f;
const float MAP_CRITICAL_FRONT_CM = 12.0f;
const float MAP_SIDE_BLOCKED_CM = 32.0f;
const float SONAR_MAP_MAX_CM = 180.0f;
const float MAP_OBSTACLE_REVERSE_CM = 15.0f;
const float MAP_DEAD_END_REVERSE_CM = 20.0f;
const float MAP_CLIFF_ESCAPE_CM = 12.0f;
const float MAP_NORMAL_TURN_DEG = 78.0f;
const float MAP_DEAD_END_TURN_DEG = 150.0f;
const float MAP_CLIFF_TURN_DEG = 92.0f;

const unsigned long ODOMETRY_INTERVAL_US = 10000UL;
const unsigned long CENTER_SONAR_INTERVAL_MS = 110UL;
const unsigned long SERVO_SETTLE_MS = 300UL;
const unsigned long SURVEY_INTERVAL_MS = 4500UL;
const unsigned long POSE_TELEMETRY_MS = 250UL;
const unsigned long TURN_TIMEOUT_MS = 3500UL;

/* Servo positions. */
const uint8_t SERVO_CENTER = 90;
const uint8_t SERVO_LEFT = 150;
const uint8_t SERVO_RIGHT = 30;

#endif
