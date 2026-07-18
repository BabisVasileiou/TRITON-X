/********** TRITON-X LEVEL mode
 * Purpose
 *   Preserves the validated calibrated phone tilt behavior from LEVEL
 *   v1.2 while sharing the integrated hardware and safety layers.
 * Hardware
 *   Phone orientation is received through the ESP32 UART bridge.
 * Software
 *   Normalized forward/turn mixing, watchdog and neutral re-arming.
 * Safety
 *   Cliff recovery always overrides phone commands.
 * Reference
 *   v4.0 Final, based on LEVEL standalone v1.2.
 **********/

#ifndef TRITON_X_LEVEL_MODE_H
#define TRITON_X_LEVEL_MODE_H

int levelLeftSpeed = 0;
int levelRightSpeed = 0;
bool levelValidCommandSeen = false;
bool levelCommandTimedOut = false;
bool levelNeutralRequired = true;
unsigned long levelLastCommandMs = 0;

/******** function resetLevelMode
 * Purpose
 *   Resets LEVEL commands, watchdog and neutral interlock.
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
void resetLevelMode(void) {
  levelLeftSpeed = 0;
  levelRightSpeed = 0;
  levelValidCommandSeen = false;
  levelCommandTimedOut = false;
  levelNeutralRequired = true;
  levelLastCommandMs = 0;
}

/******** function mixLevelCommand
 * Purpose
 *   Converts phone tilt commands to differential wheel commands.
 * Arguments
 *   forward Normalized forward command in the range -100 to +100.
 *   turn Normalized turn command in the range -100 to +100.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void mixLevelCommand(int forward, int turn) {
  int mixedLeft = forward + turn;
  int mixedRight = forward - turn;
  int maximum = max(abs(mixedLeft), abs(mixedRight));
  if (maximum > 100) {
    mixedLeft = (mixedLeft * 100L) / maximum;
    mixedRight = (mixedRight * 100L) / maximum;
  }
  levelLeftSpeed = (mixedLeft * LEVEL_SPEED_PWM) / 100;
  levelRightSpeed = (mixedRight * LEVEL_SPEED_PWM) / 100;
}

/******** function handleTiltCommand
 * Purpose
 *   Validates and applies one LEVEL UART command payload.
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
void handleTiltCommand(const char* payload) {
  char* endForward = NULL;
  long parsedForward = strtol(payload, &endForward, 10);
  if (endForward == payload || *endForward != ',') {
    return;
  }
  char* endTurn = NULL;
  long parsedTurn = strtol(endForward + 1, &endTurn, 10);
  if (endTurn == endForward + 1 || *endTurn != '\0') {
    return;
  }

  int forward = constrain((int)parsedForward, -100, 100);
  int turn = constrain((int)parsedTurn, -100, 100);
  if (abs(forward) <= TILT_COMMAND_DEADZONE) {
    forward = 0;
  }
  if (abs(turn) <= TILT_COMMAND_DEADZONE) {
    turn = 0;
  }

  levelLastCommandMs = millis();
  levelValidCommandSeen = true;
  levelCommandTimedOut = false;
  bool neutral = (forward == 0 && turn == 0);

  if (levelNeutralRequired) {
    levelLeftSpeed = 0;
    levelRightSpeed = 0;
    if (safetyState == safetyIdle && neutral) {
      levelNeutralRequired = false;
      reportStatus(F("LEVEL"), F("Armed - neutral"));
    } else if (safetyState == safetyIdle) {
      reportStatus(F("LEVEL"), F("Level phone"));
    }
    return;
  }

  if (safetyState != safetyIdle || !modeRunning ||
      selectedMode != modeLevel) {
    levelLeftSpeed = 0;
    levelRightSpeed = 0;
    return;
  }

  mixLevelCommand(forward, turn);
}

/******** function runLevelMode
 * Purpose
 *   Runs LEVEL motion and its command watchdog.
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
void runLevelMode(void) {
  if (!modeRunning || safetyState != safetyIdle) {
    motors.stop();
    return;
  }

  if (!levelValidCommandSeen ||
      millis() - levelLastCommandMs > COMMAND_TIMEOUT_MS) {
    if (!levelCommandTimedOut) {
      levelCommandTimedOut = true;
      levelNeutralRequired = true;
      levelLeftSpeed = 0;
      levelRightSpeed = 0;
      motors.stop();
      sendEvent(F("LEVEL_COMMAND_TIMEOUT"));
      reportStatus(F("LINK LOST"), F("Level phone"));
    }
    return;
  }

  if (levelNeutralRequired) {
    motors.stop();
    return;
  }
  motors.setMotors(levelLeftSpeed, levelRightSpeed);
}

#endif
