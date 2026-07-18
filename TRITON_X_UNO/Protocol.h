/********** TRITON-X supervisor and UART protocol
 * Purpose
 *   Selects modes, applies START/STOP semantics, parses browser commands
 *   and publishes compact system telemetry to the ESP32.
 * Hardware
 *   Bidirectional 9600 baud UART on D4/D13 through FixedTimerUart.
 * Software
 *   Fixed C buffers and flash strings avoid SRAM fragmentation.
 * Safety
 *   Mode changes stop immediately. Browser-link loss stops every mode.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_PROTOCOL_H
#define TRITON_X_PROTOCOL_H

/******** function printModeName
 * Purpose
 *   Writes the selected robot mode to a Print destination.
 * Arguments
 *   output Print destination receiving formatted data.
 *   mode Robot operating mode to process.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void printModeName(Print& output, RobotMode mode) {
  switch (mode) {
    case modeManual:
      output.print(F("MANUAL"));
      break;
    case modeLevel:
      output.print(F("LEVEL"));
      break;
    case modeLine:
      output.print(F("LINE"));
      break;
    case modeObstacle:
      output.print(F("OBSTACLE"));
      break;
    case modeMapping:
      output.print(F("MAPPING"));
      break;
    default:
      output.print(F("IDLE"));
      break;
  }
}

/******** function printSafetyName
 * Purpose
 *   Writes the current safety state to a Print destination.
 * Arguments
 *   output Print destination receiving formatted data.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void printSafetyName(Print& output) {
  switch (safetyState) {
    case safetyStopping:
      output.print(F("STOPPING"));
      break;
    case safetyLinearEscape:
      output.print(F("ESCAPE"));
      break;
    case safetyTurning:
      output.print(F("TURNING"));
      break;
    case safetyPostHold:
      output.print(F("HOLD"));
      break;
    case safetyLocked:
      output.print(F("LOCKED"));
      break;
    default:
      output.print(F("SAFE"));
      break;
  }
}

/******** function sendEvent
 * Purpose
 *   Sends a flash-resident event message to the ESP32.
 * Arguments
 *   text Input parameter defined by the function signature.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void sendEvent(const __FlashStringHelper* text) {
  espSerial.print(F("EVENT:"));
  espSerial.println(text);
}

/******** function sendEventRam
 * Purpose
 *   Sends a RAM-resident event message to the ESP32.
 * Arguments
 *   text Input parameter defined by the function signature.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void sendEventRam(const char* text) {
  espSerial.print(F("EVENT:"));
  espSerial.println(text);
}

/******** function resetModeControllers
 * Purpose
 *   Resets all five mode controllers during a transition.
 * Arguments
 *   resetMapPose Input parameter defined by the function signature.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void resetModeControllers(bool resetMapPose) {
  resetManualMode();
  resetLevelMode();
  resetLineMode();
  resetObstacleMode();
  resetMappingMode(resetMapPose);
}

/******** function neutralizeOperatorCommands
 * Purpose
 *   Clears MANUAL and LEVEL commands and requires neutral re-arming.
 * Arguments
 *   None.
 * Results
 *   Zeros all operator commands and returns no value.
 * Hardware
 *   Does not directly access hardware.
 * Software
 *   Updates the MANUAL and LEVEL command state.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void neutralizeOperatorCommands(void) {
  manualLeftSpeed = 0;
  manualRightSpeed = 0;
  levelLeftSpeed = 0;
  levelRightSpeed = 0;
  manualNeutralRequired = true;
  levelNeutralRequired = true;
}

/******** function forceSystemStop
 * Purpose
 *   Stops motion and clears commands while optionally retaining mode.
 * Arguments
 *   preserveMode True to retain the selected mode after STOP.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void forceSystemStop(bool preserveMode) {
  modeRunning = false;
  motors.stop();
  if (safetyState != safetyLocked) {
    safetyState = safetyIdle;
  }
  frontInterruptHoldActive = false;
  sonarServo.write(SERVO_CENTER);
  neutralizeOperatorCommands();
  if (!preserveMode) {
    selectedMode = modeIdle;
  }
}

/******** function onSafetyRecovered
 * Purpose
 *   Applies the correct post-recovery behavior for the active mode.
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
void onSafetyRecovered(void) {
  motors.stop();
  if (selectedMode == modeManual) {
    manualNeutralRequired = true;
    manualLeftSpeed = 0;
    manualRightSpeed = 0;
    reportStatus(F("MANUAL"), F("Center joystick"));
  } else if (selectedMode == modeLevel) {
    levelNeutralRequired = true;
    levelLeftSpeed = 0;
    levelRightSpeed = 0;
    reportStatus(F("LEVEL"), F("Level phone"));
  } else if (selectedMode == modeObstacle && modeRunning) {
    resetObstacleMode();
    reportStatus(F("OBSTACLE"), F("Resuming"));
  } else if (selectedMode == modeMapping && modeRunning) {
    mappingDesiredHeadingDeg = headingDeg;
    resumeMappingCruise();
  }
}

/******** function parseModeName
 * Purpose
 *   Decodes a textual browser mode selection.
 * Arguments
 *   name Null-terminated mode name.
 * Results
 *   Returns the decoded mode, or modeIdle when invalid.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
RobotMode parseModeName(const char* name) {
  if (strcmp_P(name, PSTR("MANUAL")) == 0) {
    return modeManual;
  }
  if (strcmp_P(name, PSTR("LEVEL")) == 0) {
    return modeLevel;
  }
  if (strcmp_P(name, PSTR("LINE")) == 0) {
    return modeLine;
  }
  if (strcmp_P(name, PSTR("OBSTACLE")) == 0) {
    return modeObstacle;
  }
  if (strcmp_P(name, PSTR("MAPPING")) == 0) {
    return modeMapping;
  }
  return modeIdle;
}

/******** function selectRobotMode
 * Purpose
 *   Performs the mandatory safe transition to a selected mode.
 * Arguments
 *   nextMode Mode requested by the browser.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void selectRobotMode(RobotMode nextMode) {
  forceSystemStop(true);
  bool resetMapPose = (nextMode == modeMapping);
  resetModeControllers(resetMapPose);
  selectedMode = nextMode;
  sonarServo.write(SERVO_CENTER);

  if (nextMode == modeMapping) {
    sendEvent(F("MAP_RESET"));
  }
  espSerial.print(F("MODE:"));
  printModeName(espSerial, selectedMode);
  espSerial.println();
  reportStatus(modeLabel(selectedMode), F("Press START"));
}

/******** function selectedModeCanStart
 * Purpose
 *   Checks sensor and safety preconditions for START.
 * Arguments
 *   None.
 * Results
 *   Returns true when the reported condition or operation succeeds.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
bool selectedModeCanStart(void) {
  if (selectedMode == modeIdle) {
    sendEvent(F("START_REJECTED_NO_MODE"));
    return false;
  }
  if (safetyState == safetyLocked) {
    sendEvent(F("START_REJECTED_LOCKED"));
    return false;
  }
  if (selectedMode != modeLine && !sensors.pcfHealthy) {
    sendEvent(F("START_REJECTED_PCF"));
    return false;
  }
  if (selectedMode == modeMapping && !mpuHealthy) {
    sendEvent(F("START_REJECTED_MPU"));
    return false;
  }
  return true;
}

/******** function startSelectedMode
 * Purpose
 *   Arms the selected mode after an explicit START command.
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
void startSelectedMode(void) {
  if (!selectedModeCanStart()) {
    motors.stop();
    reportStatus(F("START REJECTED"), F("Check status"));
    return;
  }

  modeRunning = true;
  switch (selectedMode) {
    case modeManual:
      manualNeutralRequired = true;
      manualValidCommandSeen = false;
      reportStatus(F("MANUAL"), F("Center joystick"));
      break;
    case modeLevel:
      levelNeutralRequired = true;
      levelValidCommandSeen = false;
      reportStatus(F("LEVEL"), F("Level phone"));
      break;
    case modeLine:
      resetLineMode();
      reportStatus(F("LINE"), F("Following"));
      break;
    case modeObstacle:
      resetObstacleMode();
      reportStatus(F("OBSTACLE"), F("Cruising"));
      break;
    case modeMapping:
      startMappingMode();
      break;
    default:
      modeRunning = false;
      break;
  }
  if (modeRunning) {
    sendEvent(F("STARTED"));
  }
}

/******** function stopSelectedMode
 * Purpose
 *   Performs an immediate stop without changing selected mode.
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
void stopSelectedMode(void) {
  forceSystemStop(true);
  sendEvent(F("STOPPED"));
  reportStatus(modeLabel(selectedMode), F("Press START"));
}

/******** function handleCommandLine
 * Purpose
 *   Dispatches one complete command received from the ESP32.
 * Arguments
 *   line Complete null-terminated UART line.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void handleCommandLine(const char* line) {
  if (strcmp_P(line, PSTR("HB")) == 0) {
    linkSeen = true;
    lastHeartbeatMs = millis();
    return;
  }

  if (strncmp_P(line, PSTR("MODE:"), 5) == 0) {
    selectRobotMode(parseModeName(line + 5));
    return;
  }

  if (strcmp_P(line, PSTR("CMD:START")) == 0) {
    startSelectedMode();
    return;
  }
  if (strcmp_P(line, PSTR("CMD:STOP")) == 0) {
    stopSelectedMode();
    return;
  }
  if (strcmp_P(line, PSTR("CMD:CLEAR_LOCK")) == 0) {
    clearSafetyLock();
    return;
  }
  if (strcmp_P(line, PSTR("CMD:RESET_MAP")) == 0) {
    forceSystemStop(true);
    resetMappingMode(true);
    sendEvent(F("MAP_RESET"));
    reportStatus(F("MAPPING"), F("Map reset"));
    return;
  }

  if (strncmp_P(line, PSTR("JOY:"), 4) == 0) {
    handleJoystickCommand(line + 4);
    return;
  }
  if (strncmp_P(line, PSTR("TLT:"), 4) == 0) {
    handleTiltCommand(line + 4);
  }
}

/******** function processIncomingCommands
 * Purpose
 *   Builds newline-delimited UART commands in a fixed buffer.
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
void processIncomingCommands(void) {
  while (espSerial.available()) {
    char incoming = (char)espSerial.read();
    if (incoming == '\n') {
      commandBuffer[commandIndex] = '\0';
      handleCommandLine(commandBuffer);
      commandIndex = 0;
    } else if (incoming != '\r') {
      if (commandIndex < sizeof(commandBuffer) - 1) {
        commandBuffer[commandIndex++] = incoming;
      } else {
        commandIndex = 0;
      }
    }
  }
}

/******** function handleLinkWatchdog
 * Purpose
 *   Stops an active mode when ESP32 heartbeat data is lost.
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
void handleLinkWatchdog(void) {
  if (!linkSeen) {
    return;
  }
  if (millis() - lastHeartbeatMs <= LINK_TIMEOUT_MS) {
    return;
  }
  linkSeen = false;
  if (modeRunning) {
    forceSystemStop(true);
    sendEvent(F("BROWSER_LINK_LOST"));
    reportStatus(F("LINK LOST"), F("Press START"));
  }
}

/******** function buildCliffBits
 * Purpose
 *   Packs the four cliff sensor states into one telemetry byte.
 * Arguments
 *   None.
 * Results
 *   Returns the packed or calculated unsigned byte value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
uint8_t buildCliffBits(void) {
  uint8_t bits = 0;
  if (sensors.cliffFrontLeft) {
    bits |= 1U << 0;
  }
  if (sensors.cliffFrontRight) {
    bits |= 1U << 1;
  }
  if (sensors.cliffRearLeft) {
    bits |= 1U << 2;
  }
  if (sensors.cliffRearRight) {
    bits |= 1U << 3;
  }
  return bits;
}

/******** function buildObstacleBits
 * Purpose
 *   Packs the four obstacle sensor states into one telemetry byte.
 * Arguments
 *   None.
 * Results
 *   Returns the packed or calculated unsigned byte value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
uint8_t buildObstacleBits(void) {
  uint8_t bits = 0;
  if (sensors.obstacleFrontLeft) {
    bits |= 1U << 0;
  }
  if (sensors.obstacleFrontRight) {
    bits |= 1U << 1;
  }
  if (sensors.obstacleRearLeft) {
    bits |= 1U << 2;
  }
  if (sensors.obstacleRearRight) {
    bits |= 1U << 3;
  }
  return bits;
}

/******** function printModeState
 * Purpose
 *   Writes the mode-specific finite-state-machine state.
 * Arguments
 *   output Print destination receiving formatted data.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void printModeState(Print& output) {
  if (safetyState != safetyIdle) {
    printSafetyName(output);
    return;
  }

  switch (selectedMode) {
    case modeManual:
      if (!modeRunning) {
        output.print(F("WAIT_START"));
      } else if (manualNeutralRequired) {
        output.print(F("WAIT_NEUTRAL"));
      } else if (!manualValidCommandSeen) {
        output.print(F("WAIT_COMMAND"));
      } else {
        output.print(F("ACTIVE"));
      }
      break;

    case modeLevel:
      if (!modeRunning) {
        output.print(F("WAIT_START"));
      } else if (levelNeutralRequired) {
        output.print(F("WAIT_NEUTRAL"));
      } else if (!levelValidCommandSeen) {
        output.print(F("WAIT_COMMAND"));
      } else {
        output.print(F("ACTIVE"));
      }
      break;

    case modeLine:
      if (lineTerminalStop) {
        output.print(F("OBSTACLE_STOP"));
      } else if (modeRunning) {
        output.print(F("FOLLOWING"));
      } else {
        output.print(F("WAIT_START"));
      }
      break;

    case modeObstacle:
      if (!modeRunning) {
        output.print(F("WAIT_START"));
        break;
      }
      switch (obstacleState) {
        case obstacleCruising:
          output.print(F("CRUISE"));
          break;
        case obstacleDetected:
          output.print(F("DETECTED"));
          break;
        case obstacleScanningLeft:
        case obstacleScanningRight:
        case obstacleResettingHead:
          output.print(F("SCANNING"));
          break;
        case obstacleDeciding:
          output.print(F("DECIDE"));
          break;
        case obstacleTurning:
          output.print(F("TURNING"));
          break;
        case obstacleEscaping:
          output.print(F("REVERSING"));
          break;
        default:
          output.print(F("UNKNOWN"));
          break;
      }
      break;

    case modeMapping:
      printMappingState(output);
      break;

    default:
      output.print(F("IDLE"));
      break;
  }
}

/******** function currentSonarDistance
 * Purpose
 *   Returns the sonar value relevant to the selected mode.
 * Arguments
 *   None.
 * Results
 *   Returns the calculated value as a floating-point number.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
float currentSonarDistance(void) {
  if (selectedMode == modeLine) {
    return lineLatestDistanceCm;
  }
  if (selectedMode == modeObstacle) {
    return obstacleCurrentCm;
  }
  if (selectedMode == modeMapping) {
    return mappingCenterDistanceCm;
  }
  return SONAR_MAP_MAX_CM;
}

/******** function sendSystemTelemetry
 * Purpose
 *   Sends periodic system, mode, sensor and heading telemetry.
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
void sendSystemTelemetry(void) {
  unsigned long now = millis();
  unsigned long interval =
    (selectedMode == modeMapping) ? 500UL : SYSTEM_TELEMETRY_MS;
  if (now - lastSystemTelemetryMs < interval) {
    return;
  }
  lastSystemTelemetryMs = now;

  espSerial.print(F("SYS:"));
  printModeName(espSerial, selectedMode);
  espSerial.print(',');
  espSerial.print(modeRunning ? 1 : 0);
  espSerial.print(',');
  printSafetyName(espSerial);
  espSerial.print(',');
  printModeState(espSerial);
  espSerial.print(',');
  espSerial.print(buildCliffBits(), HEX);
  espSerial.print(',');
  espSerial.print(buildObstacleBits(), HEX);
  espSerial.print(',');
  espSerial.print(sensors.pcfHealthy ? 1 : 0);
  espSerial.print(',');
  espSerial.print(mpuHealthy ? 1 : 0);
  espSerial.print(',');
  espSerial.print(currentSonarDistance(), 1);
  espSerial.print(',');
  espSerial.println(headingDeg, 1);
}

/******** function runSelectedMode
 * Purpose
 *   Dispatches one loop iteration to the selected mode controller.
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
void runSelectedMode(void) {
  switch (selectedMode) {
    case modeManual:
      runManualMode();
      break;
    case modeLevel:
      runLevelMode();
      break;
    case modeLine:
      runLineMode();
      break;
    case modeObstacle:
      runObstacleMode();
      break;
    case modeMapping:
      runMappingMode();
      break;
    default:
      motors.stop();
      break;
  }
}

#endif
