/********** TRITON-X ESP32-CAM configuration
 * Purpose
 *   Defines network, UART, mapping and AI-Thinker camera constants.
 * Hardware
 *   AI-Thinker ESP32-CAM with OV2640. GPIO13/GPIO14 are used for UART2;
 *   the microSD interface is intentionally unused.
 * Software
 *   HTTP port 80, WebSocket port 81 and MJPEG port 82.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_ESP_CONFIGURATION_H
#define TRITON_X_ESP_CONFIGURATION_H

#include <Arduino.h>

const char* AP_SSID = "TRITON-X";
const char* AP_PASSWORD = "triton2026";

const uint16_t HTTP_PORT = 80;
const uint16_t WS_PORT = 81;
const uint16_t CAMERA_PORT = 82;

const uint8_t AP_CHANNEL = 1;
const uint8_t AP_MAX_CLIENTS = 1;
const unsigned long CAMERA_FRAME_INTERVAL_MS = 100UL;

const int ROBOT_RX_PIN = 14;
const int ROBOT_TX_PIN = 13;
const uint32_t ROBOT_BAUD = 9600;

const unsigned long HEARTBEAT_INTERVAL_MS = 250UL;
const unsigned long MAP_BROADCAST_MS = 1000UL;
const unsigned long TELEMETRY_BROADCAST_MS = 500UL;
const unsigned long UNO_OFFLINE_TIMEOUT_MS = 3000UL;
const unsigned long CONTROLLER_TIMEOUT_MS = 4000UL;

const int MAP_SIZE = 30;
const int MAP_CELL_COUNT = MAP_SIZE * MAP_SIZE;
const float CELL_SIZE_CM = 10.0f;
const float MAP_RAY_MAX_CM = 180.0f;

/* AI-Thinker ESP32-CAM pin assignment. */
const int CAM_PWDN_PIN = 32;
const int CAM_RESET_PIN = -1;
const int CAM_XCLK_PIN = 0;
const int CAM_SIOD_PIN = 26;
const int CAM_SIOC_PIN = 27;
const int CAM_Y9_PIN = 35;
const int CAM_Y8_PIN = 34;
const int CAM_Y7_PIN = 39;
const int CAM_Y6_PIN = 36;
const int CAM_Y5_PIN = 21;
const int CAM_Y4_PIN = 19;
const int CAM_Y3_PIN = 18;
const int CAM_Y2_PIN = 5;
const int CAM_VSYNC_PIN = 25;
const int CAM_HREF_PIN = 23;
const int CAM_PCLK_PIN = 22;

#endif
