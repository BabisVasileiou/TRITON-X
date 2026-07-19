/********** TRITON-X ESP32 supervisor and protocol bridge
 * Purpose
 *   Bridges browser commands to the Arduino UNO, parses telemetry and
 *   enforces a browser-disconnect STOP for every robot mode.
 * Hardware
 *   UART2: GPIO13 TX to Arduino D4; GPIO14 RX from level-shifted D13.
 * Software
 *   WebServer port 80, WebSocketsServer port 81 and fixed UART buffers.
 * Safety
 *   Only one browser is the active controller. Its disconnection sends
 *   STOP immediately and disables the optional camera stream.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_ROBOT_PROTOCOL_H
#define TRITON_X_ROBOT_PROTOCOL_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

WebServer httpServer(HTTP_PORT);
WebSocketsServer webSocket(WS_PORT);
HardwareSerial robotSerial(2);

int activeControllerClient = -1;
char serialBuffer[128];
uint8_t serialIndex = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastMapBroadcastMs = 0;
unsigned long lastTelemetryBroadcastMs = 0;
unsigned long lastUnoLineMs = 0;
unsigned long lastControllerHeartbeatMs = 0;
bool controllerTimeoutLatched = false;

char robotMode[16] = "IDLE";
char robotSafety[16] = "SAFE";
char robotState[24] = "IDLE";
char lastEvent[48] = "Waiting for UNO";
bool robotRunning = false;
bool robotPcfHealthy = false;
bool robotMpuHealthy = false;
uint8_t robotCliffBits = 0;
uint8_t robotObstacleBits = 0;
float robotX = 0.0f;
float robotY = 0.0f;
float robotHeading = 0.0f;
float robotSonar = MAP_RAY_MAX_CM;
float robotTravel = 0.0f;
long robotEncoderLeft = 0;
long robotEncoderRight = 0;
unsigned long robotElapsedSeconds = 0;

/******** function sendRobotCommand
 * Purpose
 *   Sends one newline-delimited command to the Arduino UNO.
 * Arguments
 *   command Null-terminated UART or browser command.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void sendRobotCommand(const char* command) {
  robotSerial.println(command);
}

/******** function sendStop
 * Purpose
 *   Sends the global STOP command to the Arduino UNO.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void sendStop(void) {
  sendRobotCommand("CMD:STOP");
}

/******** function unoOnline
 * Purpose
 *   Checks whether recent valid UART telemetry was received.
 * Arguments
 *   None.
 * Results
 *   Returns true when the reported condition or operation succeeds.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
bool unoOnline(void) {
  return lastUnoLineMs != 0 &&
    millis() - lastUnoLineMs <= UNO_OFFLINE_TIMEOUT_MS;
}

/******** function resetMapTelemetry
 * Purpose
 *   Clears the ESP32 map and associated pose telemetry.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void resetMapTelemetry(void) {
  clearMap();
  robotX = 0.0f;
  robotY = 0.0f;
  robotHeading = 0.0f;
  robotTravel = 0.0f;
  robotElapsedSeconds = 0;
  robotEncoderLeft = 0;
  robotEncoderRight = 0;
}

/******** function parseSystemLine
 * Purpose
 *   Parses one Arduino system telemetry record.
 * Arguments
 *   payload Null-terminated command payload to parse.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void parseSystemLine(const char* payload) {
  char mode[16] = "IDLE";
  int running = 0;
  char safety[16] = "SAFE";
  char state[24] = "IDLE";
  unsigned int cliffBits = 0;
  unsigned int obstacleBits = 0;
  int pcf = 0;
  int mpu = 0;
  float sonar = MAP_RAY_MAX_CM;
  float heading = 0.0f;

  int parsed = sscanf(
    payload,
    "%15[^,],%d,%15[^,],%23[^,],%x,%x,%d,%d,%f,%f",
    mode,
    &running,
    safety,
    state,
    &cliffBits,
    &obstacleBits,
    &pcf,
    &mpu,
    &sonar,
    &heading
  );

  if (parsed == 10) {
    strncpy(robotMode, mode, sizeof(robotMode) - 1);
    robotMode[sizeof(robotMode) - 1] = '\0';
    strncpy(robotSafety, safety, sizeof(robotSafety) - 1);
    robotSafety[sizeof(robotSafety) - 1] = '\0';
    strncpy(robotState, state, sizeof(robotState) - 1);
    robotState[sizeof(robotState) - 1] = '\0';
    robotRunning = running != 0;
    robotCliffBits = (uint8_t)cliffBits;
    robotObstacleBits = (uint8_t)obstacleBits;
    robotPcfHealthy = pcf != 0;
    robotMpuHealthy = mpu != 0;
    robotSonar = sonar;
    robotHeading = heading;
  }
}

/******** function parsePoseLine
 * Purpose
 *   Parses one Arduino mapping pose telemetry record.
 * Arguments
 *   payload Null-terminated command payload to parse.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void parsePoseLine(const char* payload) {
  float x = 0.0f;
  float y = 0.0f;
  float heading = 0.0f;
  float sonar = 0.0f;
  char state[24] = "";
  long leftTicks = 0;
  long rightTicks = 0;
  float travel = 0.0f;
  unsigned long elapsed = 0;

  int parsed = sscanf(
    payload,
    "%f,%f,%f,%f,%23[^,],%ld,%ld,%f,%lu",
    &x,
    &y,
    &heading,
    &sonar,
    state,
    &leftTicks,
    &rightTicks,
    &travel,
    &elapsed
  );

  if (parsed == 9) {
    robotX = x;
    robotY = y;
    robotHeading = heading;
    robotSonar = sonar;
    robotEncoderLeft = leftTicks;
    robotEncoderRight = rightTicks;
    robotTravel = travel;
    robotElapsedSeconds = elapsed;
    strncpy(robotState, state, sizeof(robotState) - 1);
    robotState[sizeof(robotState) - 1] = '\0';
    markPath(robotX, robotY);
  }
}

/******** function parseRayLine
 * Purpose
 *   Parses and integrates one Arduino sonar ray record.
 * Arguments
 *   payload Null-terminated command payload to parse.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void parseRayLine(const char* payload) {
  float x = 0.0f;
  float y = 0.0f;
  float bearing = 0.0f;
  float distance = 0.0f;
  int hit = 0;
  int parsed = sscanf(
    payload,
    "%f,%f,%f,%f,%d",
    &x,
    &y,
    &bearing,
    &distance,
    &hit
  );
  if (parsed == 5) {
    integrateRay(x, y, bearing, distance, hit != 0);
  }
}

/******** function handleRobotLine
 * Purpose
 *   Classifies and processes one complete Arduino UART line.
 * Arguments
 *   line Complete null-terminated UART line.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void handleRobotLine(const char* line) {
  lastUnoLineMs = millis();
  if (strncmp(line, "SYS:", 4) == 0) {
    parseSystemLine(line + 4);
  } else if (strncmp(line, "POSE:", 5) == 0) {
    parsePoseLine(line + 5);
  } else if (strncmp(line, "RAY:", 4) == 0) {
    parseRayLine(line + 4);
  } else if (strncmp(line, "MODE:", 5) == 0) {
    strncpy(robotMode, line + 5, sizeof(robotMode) - 1);
    robotMode[sizeof(robotMode) - 1] = '\0';
  } else if (strncmp(line, "EVENT:", 6) == 0) {
    strncpy(lastEvent, line + 6, sizeof(lastEvent) - 1);
    lastEvent[sizeof(lastEvent) - 1] = '\0';
    if (strcmp(lastEvent, "MAP_RESET") == 0) {
      resetMapTelemetry();
    }
    char eventMessage[64];
    snprintf(
      eventMessage,
      sizeof(eventMessage),
      "EVENT:%s",
      lastEvent
    );
    webSocket.broadcastTXT(eventMessage);
  }
}

/******** function processRobotSerial
 * Purpose
 *   Builds newline-delimited Arduino telemetry records.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void processRobotSerial(void) {
  while (robotSerial.available()) {
    char incoming = (char)robotSerial.read();
    if (incoming == '\n') {
      serialBuffer[serialIndex] = '\0';
      handleRobotLine(serialBuffer);
      serialIndex = 0;
    } else if (incoming != '\r') {
      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = incoming;
      } else {
        serialIndex = 0;
      }
    }
  }
}

/******** function broadcastMap
 * Purpose
 *   Broadcasts the serialized occupancy map to browser clients.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void broadcastMap(void) {
  /*
   * WebSockets library releases that expose broadcastTXT(String&)
   * cannot accept the temporary String returned by buildMapMessage().
   * Store the message in a named lvalue before broadcasting it.
   */
  String mapMessage = buildMapMessage();
  webSocket.broadcastTXT(mapMessage);
}

/******** function broadcastTelemetry
 * Purpose
 *   Broadcasts consolidated JSON telemetry to browser clients.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void broadcastTelemetry(void) {
  char payload[600];
  snprintf(
    payload,
    sizeof(payload),
    "TEL:{\"uno\":%d,\"mode\":\"%s\",\"running\":%d,"
    "\"safety\":\"%s\",\"cliff\":%u,\"obstacle\":%u,"
    "\"pcf\":%d,\"mpu\":%d,\"sonar\":%.1f,"
    "\"heading\":%.1f,\"x\":%.1f,\"y\":%.1f,"
    "\"state\":\"%s\",\"encL\":%ld,\"encR\":%ld,"
    "\"travel\":%.1f,\"elapsed\":%lu,\"event\":\"%s\","
    "\"cameraReady\":%d,\"cameraOn\":%d}",
    unoOnline() ? 1 : 0,
    robotMode,
    robotRunning ? 1 : 0,
    robotSafety,
    robotCliffBits,
    robotObstacleBits,
    robotPcfHealthy ? 1 : 0,
    robotMpuHealthy ? 1 : 0,
    robotSonar,
    robotHeading,
    robotX,
    robotY,
    robotState,
    robotEncoderLeft,
    robotEncoderRight,
    robotTravel,
    robotElapsedSeconds,
    lastEvent,
    cameraReady ? 1 : 0,
    cameraEnabled ? 1 : 0
  );
  webSocket.broadcastTXT(payload);
}

/******** function notifyCameraState
 * Purpose
 *   Sends current camera availability to one browser client.
 * Arguments
 *   clientId WebSocket client identifier.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void notifyCameraState(uint8_t clientId) {
  if (!cameraAvailable) {
    webSocket.sendTXT(clientId, "CAM:ERROR");
    return;
  }
  webSocket.sendTXT(
    clientId, cameraEnabled ? "CAM:ON" : "CAM:OFF"
  );
}

/******** function handleControllerCommand
 * Purpose
 *   Validates and routes one active-browser command.
 * Arguments
 *   clientId WebSocket client identifier.
 *   command Null-terminated UART or browser command.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void handleControllerCommand(uint8_t clientId, const char* command) {
  lastControllerHeartbeatMs = millis();
  controllerTimeoutLatched = false;

  if (strcmp(command, "CTRL:HB") == 0) {
    return;
  }
  if (strcmp(command, "CAM:ON") == 0) {
    setCameraEnabled(true);
    notifyCameraState(clientId);
    return;
  }
  if (strcmp(command, "CAM:OFF") == 0) {
    setCameraEnabled(false);
    notifyCameraState(clientId);
    return;
  }

  if (strcmp(command, "CMD:RESET_MAP") == 0) {
    resetMapTelemetry();
    sendRobotCommand("CMD:RESET_MAP");
    broadcastMap();
    return;
  }

  if (strncmp(command, "MODE:", 5) == 0) {
    if (strcmp(command + 5, "MAPPING") == 0) {
      resetMapTelemetry();
      broadcastMap();
    }
    sendRobotCommand(command);
    return;
  }

  if (strncmp(command, "JOY:", 4) == 0 ||
      strncmp(command, "TLT:", 4) == 0 ||
      strncmp(command, "CMD:", 4) == 0) {
    sendRobotCommand(command);
  }
}

/******** function onWebSocketEvent
 * Purpose
 *   Processes browser connection, command and close events.
 * Arguments
 *   clientId Identifier of the WebSocket client.
 *   type WebSocket event type reported by the library.
 *   payload Pointer to the received WebSocket payload bytes.
 *   length Number of valid bytes in the received payload.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM Wi-Fi, WebSocket and UART resources.
 * Software
 *   Uses fixed buffers and the ESP32 Arduino communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void onWebSocketEvent(uint8_t clientId, WStype_t type,
                      uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      if (activeControllerClient < 0) {
        activeControllerClient = clientId;
        lastControllerHeartbeatMs = millis();
        controllerTimeoutLatched = false;
        webSocket.sendTXT(clientId, "CTRL:ACTIVE");
      } else {
        webSocket.sendTXT(clientId, "CTRL:BUSY");
      }
      notifyCameraState(clientId);
      if (strcmp(robotMode, "MAPPING") == 0) {
        broadcastMap();
      }
      broadcastTelemetry();
      break;

    case WStype_DISCONNECTED:
      if (activeControllerClient == clientId) {
        sendStop();
        setCameraEnabled(false);
        activeControllerClient = -1;
        controllerTimeoutLatched = false;
        strncpy(
          lastEvent,
          "Browser disconnected - STOP",
          sizeof(lastEvent) - 1
        );
        lastEvent[sizeof(lastEvent) - 1] = '\0';
      }
      break;

    case WStype_TEXT: {
      if (activeControllerClient != clientId) {
        webSocket.sendTXT(clientId, "CTRL:BUSY");
        return;
      }
      char command[96];
      size_t copyLength = min(length, sizeof(command) - 1);
      memcpy(command, payload, copyLength);
      command[copyLength] = '\0';
      handleControllerCommand(clientId, command);
      break;
    }

    default:
      break;
  }
}

/******** function serviceProtocol
 * Purpose
 *   Services HTTP, WebSocket, UART, heartbeat and broadcasts.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0-rc5, C. Vasileiou, July 2026.
 **********/
void serviceProtocol(void) {
  httpServer.handleClient();
  webSocket.loop();
  processRobotSerial();

  unsigned long now = millis();
  if (activeControllerClient >= 0 &&
      now - lastControllerHeartbeatMs > CONTROLLER_TIMEOUT_MS &&
      !controllerTimeoutLatched) {
    controllerTimeoutLatched = true;
    sendStop();
    setCameraEnabled(false);
    strncpy(
      lastEvent,
      "Browser heartbeat lost - STOP",
      sizeof(lastEvent) - 1
    );
    lastEvent[sizeof(lastEvent) - 1] = '\0';
  }
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    sendRobotCommand("HB");
  }
  if (activeControllerClient >= 0 &&
      strcmp(robotMode, "MAPPING") == 0 &&
      now - lastMapBroadcastMs >= MAP_BROADCAST_MS) {
    lastMapBroadcastMs = now;
    broadcastMap();
  }
  if (activeControllerClient >= 0 &&
      now - lastTelemetryBroadcastMs >= TELEMETRY_BROADCAST_MS) {
    lastTelemetryBroadcastMs = now;
    broadcastTelemetry();
  }
}

#endif
