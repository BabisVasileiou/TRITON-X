/********** TRITON-X MANUAL mode
 * Purpose
 *   Preserves the validated Wi-Fi joystick behavior from MANUAL v1.2.
 * Hardware
 *   Uses the shared motor and cliff-safety hardware layers.
 * Software
 *   Differential mixing, 500 ms watchdog and neutral re-arming.
 * Safety
 *   Cliff recovery always overrides operator commands.
 * Reference
 *   v4.0 Final, based on MANUAL standalone v1.2.
 **********/

#ifndef TRITON_X_MANUAL_MODE_H
#define TRITON_X_MANUAL_MODE_H

int manualLeftSpeed = 0;
int manualRightSpeed = 0;
bool manualValidCommandSeen = false;
bool manualCommandTimedOut = false;
bool manualNeutralRequired = true;
unsigned long manualLastCommandMs = 0;

/******** function resetManualMode
 * Purpose
 *   Resets MANUAL commands, watchdog and neutral interlock.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void resetManualMode(void) {
  manualLeftSpeed = 0;
  manualRightSpeed = 0;
  manualValidCommandSeen = false;
  manualCommandTimedOut = false;
  manualNeutralRequired = true;
  manualLastCommandMs = 0;
}

/******** function mixManualCommand
 * Purpose
 *   Converts joystick axes to bounded differential wheel commands.
 * Arguments
 *   x Horizontal control or coordinate value.
 *   y Vertical control or coordinate value.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void mixManualCommand(int x, int y) {
  int mixedLeft = y + x;
  int mixedRight = y - x;
  int maximum = max(abs(mixedLeft), abs(mixedRight));
  if (maximum > 100) {
    mixedLeft = (mixedLeft * 100L) / maximum;
    mixedRight = (mixedRight * 100L) / maximum;
  }
  manualLeftSpeed = (mixedLeft * MANUAL_SPEED_PWM) / 100;
  manualRightSpeed = (mixedRight * MANUAL_SPEED_PWM) / 100;
}

/******** function handleJoystickCommand
 * Purpose
 *   Validates and applies one MANUAL UART command payload.
 * Arguments
 *   payload Null-terminated command payload to parse.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void handleJoystickCommand(const char* payload) {
  char* endX = NULL;
  long parsedX = strtol(payload, &endX, 10);
  if (endX == payload || *endX != ',') {
    return;
  }
  char* endY = NULL;
  long parsedY = strtol(endX + 1, &endY, 10);
  if (endY == endX + 1 || *endY != '\0') {
    return;
  }

  int x = constrain((int)parsedX, -100, 100);
  int y = constrain((int)parsedY, -100, 100);
  if (abs(x) <= JOYSTICK_DEADZONE) {
    x = 0;
  }
  if (abs(y) <= JOYSTICK_DEADZONE) {
    y = 0;
  }

  manualLastCommandMs = millis();
  manualValidCommandSeen = true;
  manualCommandTimedOut = false;
  bool neutral = (x == 0 && y == 0);

  if (manualNeutralRequired) {
    manualLeftSpeed = 0;
    manualRightSpeed = 0;
    if (safetyState == safetyIdle && neutral) {
      manualNeutralRequired = false;
      reportStatus(F("MANUAL"), F("Armed - neutral"));
    } else if (safetyState == safetyIdle) {
      reportStatus(F("MANUAL"), F("Center joystick"));
    }
    return;
  }

  if (safetyState != safetyIdle || !modeRunning ||
      selectedMode != modeManual) {
    manualLeftSpeed = 0;
    manualRightSpeed = 0;
    return;
  }

  mixManualCommand(x, y);
}

/******** function runManualMode
 * Purpose
 *   Runs MANUAL motion and its command watchdog.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void runManualMode(void) {
  if (!modeRunning || safetyState != safetyIdle) {
    motors.stop();
    return;
  }

  if (!manualValidCommandSeen ||
      millis() - manualLastCommandMs > COMMAND_TIMEOUT_MS) {
    if (!manualCommandTimedOut) {
      manualCommandTimedOut = true;
      manualNeutralRequired = true;
      manualLeftSpeed = 0;
      manualRightSpeed = 0;
      motors.stop();
      sendEvent(F("MANUAL_COMMAND_TIMEOUT"));
      reportStatus(F("LINK LOST"), F("Center joystick"));
    }
    return;
  }

  if (manualNeutralRequired) {
    motors.stop();
    return;
  }
  motors.setMotors(manualLeftSpeed, manualRightSpeed);
}

#endif
