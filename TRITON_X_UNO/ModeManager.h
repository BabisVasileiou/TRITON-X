/********** TRITON-X mode manager state
 * Purpose
 *   Stores the selected mode, run permission and communication state.
 * Hardware
 *   None directly.
 * Software
 *   Mode changes are completed only through the protocol supervisor.
 * Safety
 *   Selection never starts motion; a separate START command is required.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_MODE_MANAGER_H
#define TRITON_X_MODE_MANAGER_H

RobotMode selectedMode = modeIdle;
bool modeRunning = false;
bool linkSeen = false;
unsigned long lastHeartbeatMs = 0;
unsigned long lastSystemTelemetryMs = 0;

char commandBuffer[32];
uint8_t commandIndex = 0;

void sendEvent(const __FlashStringHelper* text);
void sendEventRam(const char* text);
void onSafetyRecovered(void);
void forceSystemStop(bool preserveMode);
void neutralizeOperatorCommands(void);

/******** function modeLabel
 * Purpose
 *   Returns a flash-resident display label for a robot mode.
 * Arguments
 *   mode Robot operating mode to process.
 * Results
 *   Returns a flash-resident display string.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
const __FlashStringHelper* modeLabel(RobotMode mode) {
  switch (mode) {
    case modeManual:
      return F("MANUAL");
    case modeLevel:
      return F("LEVEL");
    case modeLine:
      return F("LINE");
    case modeObstacle:
      return F("OBSTACLE");
    case modeMapping:
      return F("MAPPING");
    default:
      return F("IDLE");
  }
}

#endif
