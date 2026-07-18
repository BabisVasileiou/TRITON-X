/********** TRITON-X integrated gateway (ESP32-CAM)
 * Purpose
 *   Hosts the unified browser control station, bridges five-mode
 *   commands to the Arduino UNO, builds the occupancy map and serves an
 *   optional QVGA MJPEG camera stream.
 * Hardware
 *   AI-Thinker ESP32-CAM with OV2640. UART2 GPIO13 TX -> Arduino D4 RX;
 *   GPIO14 RX <- level-shifted Arduino D13 TX. microSD is not used.
 * Software
 *   Wi-Fi AP; HTTP 80; WebSocket 81; MJPEG 82; 9600 baud UART2.
 * Safety
 *   One active browser controller is permitted. Its disconnection sends
 *   STOP immediately. Camera control is independent of drive mode.
 * Limitations
 *   QVGA is selected to preserve Wi-Fi and mapping responsiveness.
 * Reference
 *   v4.0 Final, C. Vasileiou, MSc PADA, July 2026.
 **********/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Configuration.h"
#include "MappingGrid.h"
#include "CameraServer.h"
#include "dashboard.h"
#include "RobotProtocol.h"

/******** function handleRoot
 * Purpose
 *   Serves the unified dashboard from program flash.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void handleRoot(void) {
  httpServer.sendHeader("Content-Encoding", "gzip");
  httpServer.sendHeader("Cache-Control", "no-store");
  httpServer.send_P(
    200,
    "text/html; charset=utf-8",
    (PGM_P)DASHBOARD_GZIP,
    DASHBOARD_GZIP_SIZE
  );
}

/******** function setup
 * Purpose
 *   Initializes the controller, peripherals and communication services.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void setup(void) {
  Serial.begin(115200);
  robotSerial.begin(
    ROBOT_BAUD,
    SERIAL_8N1,
    ROBOT_RX_PIN,
    ROBOT_TX_PIN
  );

  clearMap();
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.softAP(
    AP_SSID,
    AP_PASSWORD,
    AP_CHANNEL,
    false,
    AP_MAX_CLIENTS
  );

  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/favicon.ico", HTTP_GET, []() {
    httpServer.send(204, "text/plain", "");
  });
  httpServer.onNotFound([]() {
    httpServer.send(404, "text/plain", "Not found");
  });
  httpServer.begin();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  /*
   * Initialize the camera once during startup. CAMERA ON only enables
   * the already prepared stream and therefore cannot block WebSocket
   * heartbeat processing.
   */
  cameraReady = initializeCamera();
  if (cameraReady) {
    cameraReady = startCameraServer();
  }
  cameraAvailable = cameraReady;
  cameraEnabled = false;

  Serial.print(F("Camera: "));
  if (cameraReady) {
    Serial.println(F("READY"));
  } else {
    Serial.print(F("ERROR 0x"));
    Serial.println((uint32_t)cameraLastError, HEX);
  }

  sendStop();
}

/******** function loop
 * Purpose
 *   Runs the non-blocking supervisor and selected mode continuously.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void loop(void) {
  serviceProtocol();
}
