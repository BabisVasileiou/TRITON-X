/********** TRITON-X global cliff safety supervisor
 * Purpose
 *   Overrides MANUAL, LEVEL, OBSTACLE and MAPPING whenever a cliff is
 *   detected. LINE retains its validated dual-purpose line-sensor logic.
 * Hardware
 *   Front cliff inputs D2/D3 and rear cliff inputs PCF8574 P3/P4.
 * Software
 *   Non-blocking finite-state recovery with linear escape, slow turn,
 *   post-recovery hold and latched safety lock.
 * Safety
 *   A failed recovery stops all motion and requires CLEAR SAFETY LOCK.
 * Reference
 *   v4.0 Final, based on MANUAL/LEVEL v1.2 and MAPPING v1.4.
 **********/

#ifndef TRITON_X_SAFETY_MANAGER_H
#define TRITON_X_SAFETY_MANAGER_H

SafetyState safetyState = safetyIdle;
int8_t safetyLinearDirection = 0;
int8_t safetyTurnDirection = 1;
unsigned long safetyStateStartMs = 0;
unsigned long safetyRecoveryStartMs = 0;
float safetyStartX = 0.0f;
float safetyStartY = 0.0f;
float safetyTargetHeading = 0.0f;
bool frontInterruptHoldActive = false;
unsigned long frontInterruptHoldStartMs = 0;
bool safetyTurnSawClear = false;
unsigned long safetyCliffClearSinceMs = 0;

/******** function processFrontCliffInterrupt
 * Purpose
 *   Applies the immediate motor stop required by a front cliff edge.
 * Arguments
 *   None.
 * Results
 *   Starts a short confirmation hold and returns no value.
 * Hardware
 *   Uses D2/D3 external interrupts and the shared motor driver.
 * Software
 *   LINE mode ignores these edges because D2/D3 are line sensors.
 * Reference
 *   v4.0 Final, based on MANUAL and LEVEL standalone v1.2.
 **********/
void processFrontCliffInterrupt(void) {
  bool interruptSeen = false;
  noInterrupts();
  if (frontCliffInterruptFlag) {
    frontCliffInterruptFlag = false;
    interruptSeen = true;
  }
  interrupts();

  if (!interruptSeen || !modeRunning ||
      selectedMode == modeLine || selectedMode == modeIdle) {
    return;
  }

  motors.stop();
  neutralizeOperatorCommands();
  frontInterruptHoldActive = true;
  frontInterruptHoldStartMs = millis();
}

/******** function frontInterruptHoldBlocksMotion
 * Purpose
 *   Holds the robot stopped while a front cliff edge is confirmed.
 * Arguments
 *   None.
 * Results
 *   Returns true while normal mode execution must remain inhibited.
 * Hardware
 *   Stops the shared L298N motor driver during the confirmation hold.
 * Software
 *   Releases automatically or when the cliff FSM takes control.
 * Reference
 *   v4.0 Final, based on MANUAL and LEVEL standalone v1.2.
 **********/
bool frontInterruptHoldBlocksMotion(void) {
  if (!frontInterruptHoldActive) {
    return false;
  }
  if (safetyState != safetyIdle) {
    frontInterruptHoldActive = false;
    return false;
  }
  if (millis() - frontInterruptHoldStartMs >= INTERRUPT_CONFIRM_MS) {
    frontInterruptHoldActive = false;
    return false;
  }
  motors.stop();
  return true;
}

/******** function startCliffRecovery
 * Purpose
 *   Captures the hazard geometry and starts safe recovery.
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
void startCliffRecovery(void) {
  motors.stop();
  safetyRecoveryStartMs = millis();
  safetyStateStartMs = millis();
  safetyTurnSawClear = false;
  safetyCliffClearSinceMs = 0;
  safetyTurnDirection = turnAwayFromCliff();

  bool front = frontCliffDetected();
  bool rear = rearCliffDetected();
  if (front && !rear && rearPathSafe()) {
    safetyLinearDirection = -1;
  } else if (rear && !front && frontPathSafe()) {
    safetyLinearDirection = 1;
  } else {
    safetyLinearDirection = 0;
  }

  safetyState = safetyStopping;
  sendEvent(F("CLIFF_DETECTED"));
  if (front) {
    reportStatus(F("! CLIFF !"), F("Front detected"));
  } else {
    reportStatus(F("! CLIFF !"), F("Rear detected"));
  }
}

/******** function lockCliffSafety
 * Purpose
 *   Latches the final cliff safety lock and stops motion.
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
void lockCliffSafety(void) {
  motors.stop();
  safetyState = safetyLocked;
  modeRunning = false;
  safetyCliffClearSinceMs = 0;
  sendEvent(F("CLIFF_LOCKED"));
  reportStatus(F("CLIFF LOCK"), F("Clear when safe"));
}

/******** function beginTimedSafetyTurn
 * Purpose
 *   Starts the slow turn-away phase of cliff recovery.
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
void beginTimedSafetyTurn(void) {
  motors.stop();
  safetyState = safetyTurning;
  safetyStateStartMs = millis();
  safetyTurnSawClear = false;
  safetyCliffClearSinceMs = 0;
  if (safetyTurnDirection > 0) {
    reportStatus(F("CLIFF ESCAPE"), F("Slow turn left"));
  } else {
    reportStatus(F("CLIFF ESCAPE"), F("Slow turn right"));
  }
}

/******** function handleCliffSafety
 * Purpose
 *   Runs the global non-blocking cliff-recovery supervisor.
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
void handleCliffSafety(void) {
  if (safetyState == safetyLocked) {
    motors.stop();
    if (anyCliffDetected()) {
      safetyCliffClearSinceMs = 0;
    } else if (safetyCliffClearSinceMs == 0) {
      safetyCliffClearSinceMs = millis();
    }
    return;
  }

  if (selectedMode == modeLine || selectedMode == modeIdle) {
    return;
  }

  if (safetyState == safetyIdle) {
    if (modeRunning && anyCliffDetected()) {
      startCliffRecovery();
    }
    return;
  }

  if (safetyState != safetyLocked &&
      safetyState != safetyPostHold &&
      millis() - safetyRecoveryStartMs > CLIFF_RECOVERY_MAX_MS) {
    lockCliffSafety();
    return;
  }

  switch (safetyState) {
    case safetyStopping:
      motors.stop();
      if (millis() - safetyStateStartMs < SAFETY_STOP_MS) {
        break;
      }
      if (safetyLinearDirection < 0 && rearPathSafe()) {
        safetyStartX = poseXcm;
        safetyStartY = poseYcm;
        motors.setMotors(-ESCAPE_PWM, -ESCAPE_PWM);
        safetyState = safetyLinearEscape;
        safetyStateStartMs = millis();
        reportStatus(F("CLIFF ESCAPE"), F("Safe reverse"));
      } else if (safetyLinearDirection > 0 && frontPathSafe()) {
        safetyStartX = poseXcm;
        safetyStartY = poseYcm;
        motors.setMotors(ESCAPE_PWM, ESCAPE_PWM);
        safetyState = safetyLinearEscape;
        safetyStateStartMs = millis();
        reportStatus(F("CLIFF ESCAPE"), F("Safe forward"));
      } else {
        if (selectedMode == modeMapping) {
          safetyTargetHeading = wrapAngle180(
            headingDeg + safetyTurnDirection * MAP_CLIFF_TURN_DEG
          );
        }
        beginTimedSafetyTurn();
      }
      break;

    case safetyLinearEscape: {
      bool directionSafe =
        (safetyLinearDirection < 0) ? rearPathSafe() : frontPathSafe();
      bool finished;
      if (selectedMode == modeMapping) {
        float moved = distanceBetween(
          safetyStartX, safetyStartY, poseXcm, poseYcm
        );
        finished = moved >= MAP_CLIFF_ESCAPE_CM;
      } else {
        finished =
          millis() - safetyStateStartMs >= CLIFF_LINEAR_ESCAPE_MS;
      }

      if (!directionSafe || finished) {
        beginTimedSafetyTurn();
        if (selectedMode == modeMapping) {
          safetyTargetHeading = wrapAngle180(
            headingDeg + safetyTurnDirection * MAP_CLIFF_TURN_DEG
          );
        }
      }
      break;
    }

    case safetyTurning: {
      bool cliffNow = anyCliffDetected();
      if (!cliffNow) {
        if (!safetyTurnSawClear) {
          safetyTurnSawClear = true;
          safetyCliffClearSinceMs = millis();
        }
      } else {
        if (safetyTurnSawClear) {
          int8_t saferDirection = turnAwayFromCliff();
          if (saferDirection != safetyTurnDirection) {
            safetyTurnDirection = saferDirection;
            safetyStateStartMs = millis();
            if (selectedMode == modeMapping) {
              safetyTargetHeading = wrapAngle180(
                headingDeg + safetyTurnDirection * 40.0f
              );
            }
          }
        }
        safetyTurnSawClear = false;
        safetyCliffClearSinceMs = 0;
      }

      bool turnFinished;
      if (selectedMode == modeMapping) {
        turnFinished = runHeadingTurn(
          CLIFF_TURN_PWM, safetyTargetHeading
        );
        if (millis() - safetyStateStartMs > TURN_TIMEOUT_MS) {
          turnFinished = true;
        }
      } else {
        if (safetyTurnDirection > 0) {
          motors.setMotors(-CLIFF_TURN_PWM, CLIFF_TURN_PWM);
        } else {
          motors.setMotors(CLIFF_TURN_PWM, -CLIFF_TURN_PWM);
        }
        turnFinished =
          millis() - safetyStateStartMs >= CLIFF_TURN_MS;
      }

      if (turnFinished) {
        motors.stop();
        if (!cliffNow && safetyTurnSawClear) {
          safetyState = safetyPostHold;
          safetyStateStartMs = millis();
          reportStatus(F("CLIFF CLEARED"), F("Stabilizing"));
        } else {
          safetyTurnDirection = turnAwayFromCliff();
          safetyStateStartMs = millis();
          safetyTurnSawClear = false;
          safetyCliffClearSinceMs = 0;
          if (selectedMode == modeMapping) {
            safetyTargetHeading = wrapAngle180(
              headingDeg + safetyTurnDirection * 40.0f
            );
          }
        }
      }
      break;
    }

    case safetyPostHold:
      motors.stop();
      if (anyCliffDetected()) {
        startCliffRecovery();
      } else if (millis() - safetyStateStartMs >=
                 POST_RECOVERY_HOLD_MS) {
        safetyState = safetyIdle;
        onSafetyRecovered();
        sendEvent(F("CLIFF_CLEARED"));
      }
      break;

    case safetyLocked:
      motors.stop();
      break;

    default:
      break;
  }
}

/******** function clearSafetyLock
 * Purpose
 *   Clears a latched cliff lock only on confirmed safe floor.
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
bool clearSafetyLock(void) {
  refreshSensors(true);
  if (safetyState != safetyLocked) {
    return true;
  }
  bool stableSafeFloor = safetyCliffClearSinceMs != 0 &&
    millis() - safetyCliffClearSinceMs >= CLIFF_CLEAR_RELEASE_MS;
  if (anyCliffDetected() || !sensors.pcfHealthy ||
      !stableSafeFloor) {
    sendEvent(F("CLEAR_LOCK_REJECTED"));
    reportStatus(F("LOCK ACTIVE"), F("Wait on safe floor"));
    return false;
  }
  safetyState = safetyIdle;
  modeRunning = false;
  motors.stop();
  sonarServo.write(SERVO_CENTER);
  sendEvent(F("SAFETY_LOCK_CLEARED"));
  reportStatus(modeLabel(selectedMode), F("Press START"));
  return true;
}

#endif
