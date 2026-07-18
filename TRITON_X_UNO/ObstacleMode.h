/********** TRITON-X OBSTACLE mode
 * Purpose
 *   Preserves the validated Guardian FSM: cruise, detect, scan, decide,
 *   turn or rear-aware escape.
 * Hardware
 *   HC-SR04 on SG90, four obstacle sensors and four cliff sensors.
 * Software
 *   Includes dead-end reverse, adaptive turn duration and anti-loop
 *   U-turn after three consecutive normal turns.
 * Safety
 *   The global cliff supervisor overrides this FSM at all times.
 * Reference
 *   v4.0 Final, based on OBSTACLE standalone and final 90 PWM cliff turn.
 **********/

#ifndef TRITON_X_OBSTACLE_MODE_H
#define TRITON_X_OBSTACLE_MODE_H

ObstacleState obstacleState = obstacleCruising;
unsigned long obstacleStateStartMs = 0;
unsigned long obstacleActionDurationMs = 0;
unsigned long obstacleAntiLoopMs = 0;
uint8_t obstacleConsecutiveTurns = 0;
float obstacleCurrentCm = SONAR_MAP_MAX_CM;
float obstacleFilteredCm = 100.0f;
float obstacleLastValidCm = 100.0f;
float obstacleFilterEstimate = 0.0f;
float obstacleFilterLastEstimate = 0.0f;
float obstacleFilterError = 2.0f;
float obstacleLeftCm = 100.0f;
float obstacleRightCm = 100.0f;

/******** function setObstacleState
 * Purpose
 *   Changes the Guardian finite-state machine and starts its timer.
 * Arguments
 *   state Requested finite-state-machine state.
 *   durationMs State or action duration in milliseconds.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void setObstacleState(ObstacleState state, unsigned long durationMs) {
  obstacleState = state;
  obstacleStateStartMs = millis();
  obstacleActionDurationMs = durationMs;
}

/******** function obstacleTimerExpired
 * Purpose
 *   Checks whether the current Guardian state timer has expired.
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
bool obstacleTimerExpired(void) {
  return millis() - obstacleStateStartMs >= obstacleActionDurationMs;
}

/******** function resetObstacleMode
 * Purpose
 *   Resets the Guardian FSM, sonar filter and scan head.
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
void resetObstacleMode(void) {
  obstacleState = obstacleCruising;
  obstacleStateStartMs = millis();
  obstacleActionDurationMs = 0;
  obstacleAntiLoopMs = millis();
  obstacleConsecutiveTurns = 0;
  obstacleCurrentCm = SONAR_MAP_MAX_CM;
  obstacleFilteredCm = 100.0f;
  obstacleLastValidCm = 100.0f;
  obstacleFilterEstimate = 0.0f;
  obstacleFilterLastEstimate = 0.0f;
  obstacleFilterError = 2.0f;
  obstacleLeftCm = 100.0f;
  obstacleRightCm = 100.0f;
  sonarServo.write(SERVO_CENTER);
}


/******** function readObstacleFilteredDistance
 * Purpose
 *   Applies the validated one-dimensional sonar Kalman filter.
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
float readObstacleFilteredDistance(void) {
  bool hit = false;
  float measurement = sonar.readDistanceCm(hit);
  if (!hit) {
    return obstacleLastValidCm;
  }

  const float measureError = 2.0f;
  const float processNoise = 0.02f;
  float gain = obstacleFilterError /
    (obstacleFilterError + measureError);
  obstacleFilterEstimate = obstacleFilterEstimate +
    gain * (measurement - obstacleFilterEstimate);
  obstacleFilterError = (1.0f - gain) * obstacleFilterError +
    fabs(obstacleFilterLastEstimate - obstacleFilterEstimate) *
    processNoise;
  obstacleFilterLastEstimate = obstacleFilterEstimate;
  obstacleLastValidCm = obstacleFilterEstimate;
  return obstacleLastValidCm;
}

/******** function readObstacleScanDistance
 * Purpose
 *   Reads one unfiltered directional sonar scan measurement.
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
float readObstacleScanDistance(void) {
  bool hit = false;
  float distance = sonar.readDistanceCm(hit);
  return hit ? distance : 250.0f;
}

/******** function beginObstacleReverse
 * Purpose
 *   Starts a rear-aware reverse or a boxed-in rescan.
 * Arguments
 *   durationMs State or action duration in milliseconds.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void beginObstacleReverse(unsigned long durationMs) {
  if (rearPathSafe()) {
    motors.setMotors(-OBSTACLE_SPEED_PWM, -OBSTACLE_SPEED_PWM);
    setObstacleState(obstacleEscaping, durationMs);
    reportStatus(F("OBSTACLE"), F("Rear-aware back"));
  } else {
    motors.stop();
    setObstacleState(obstacleDetected, 100UL);
    reportStatus(F("BOXED IN"), F("Scanning"));
  }
}

/******** function makeObstacleDecision
 * Purpose
 *   Selects dead-end, anti-loop, left or right avoidance action.
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
void makeObstacleDecision(void) {
  if (obstacleConsecutiveTurns >= OBSTACLE_ANTI_LOOP_TURNS) {
    motors.setMotors(-OBSTACLE_SPEED_PWM, OBSTACLE_SPEED_PWM);
    reportStatus(F("DECISION"), F("U-turn anti-loop"));
    setObstacleState(obstacleTurning, 900UL);
    obstacleConsecutiveTurns = 0;
    return;
  }

  if (obstacleLeftCm < OBSTACLE_SAFE_DIST_CM &&
      obstacleRightCm < OBSTACLE_SAFE_DIST_CM) {
    reportStatus(F("DECISION"), F("Dead end"));
    beginObstacleReverse(600UL);
    return;
  }

  obstacleConsecutiveTurns++;
  float difference = fabs(obstacleLeftCm - obstacleRightCm);
  long mapped = map((long)constrain(difference, 0.0f, 100.0f),
                    0L, 100L, 250L, 600L);
  unsigned long turnMs = (unsigned long)constrain(mapped, 250L, 800L);

  if (obstacleLeftCm > obstacleRightCm) {
    motors.setMotors(-OBSTACLE_SPEED_PWM, OBSTACLE_SPEED_PWM);
    reportStatus(F("DECISION"), F("Turn left"));
  } else {
    motors.setMotors(OBSTACLE_SPEED_PWM, -OBSTACLE_SPEED_PWM);
    reportStatus(F("DECISION"), F("Turn right"));
  }
  setObstacleState(obstacleTurning, turnMs);
}

/******** function runObstacleMode
 * Purpose
 *   Runs the validated Guardian obstacle-avoidance FSM.
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
void runObstacleMode(void) {
  if (!modeRunning || safetyState != safetyIdle) {
    motors.stop();
    return;
  }

  switch (obstacleState) {
    case obstacleCruising: {
      obstacleFilteredCm = readObstacleFilteredDistance();
      obstacleCurrentCm = obstacleFilteredCm;

      if (frontObstacleDetected() ||
          obstacleFilteredCm < OBSTACLE_CRIT_DIST_CM) {
        sendEvent(F("OBSTACLE_FRONT"));
        beginObstacleReverse(500UL);
      } else if (obstacleFilteredCm < OBSTACLE_SAFE_DIST_CM) {
        motors.stop();
        setObstacleState(obstacleDetected, 100UL);
        reportStatus(F("OBSTACLE"), F("Preparing scan"));
      } else {
        motors.setMotors(OBSTACLE_SPEED_PWM, OBSTACLE_SPEED_PWM);
        if (millis() - obstacleAntiLoopMs >= 2000UL) {
          obstacleConsecutiveTurns = 0;
          obstacleAntiLoopMs = millis();
        }
      }
      break;
    }

    case obstacleDetected:
      if (obstacleTimerExpired()) {
        motors.stop();
        sonarServo.write(SERVO_LEFT);
        setObstacleState(
          obstacleScanningLeft, OBSTACLE_SERVO_DELAY_MS
        );
        reportStatus(F("SCANNING"), F("Left"));
      }
      break;

    case obstacleScanningLeft:
      if (obstacleTimerExpired()) {
        obstacleLeftCm = readObstacleScanDistance();
        sonarServo.write(SERVO_RIGHT);
        setObstacleState(
          obstacleScanningRight, OBSTACLE_SERVO_DELAY_MS
        );
        reportStatus(F("SCANNING"), F("Right"));
      }
      break;

    case obstacleScanningRight:
      if (obstacleTimerExpired()) {
        obstacleRightCm = readObstacleScanDistance();
        sonarServo.write(SERVO_CENTER);
        setObstacleState(
          obstacleResettingHead, OBSTACLE_SERVO_DELAY_MS
        );
      }
      break;

    case obstacleResettingHead:
      if (obstacleTimerExpired()) {
        obstacleState = obstacleDeciding;
      }
      break;

    case obstacleDeciding:
      makeObstacleDecision();
      break;

    case obstacleTurning:
      if (obstacleTimerExpired()) {
        motors.stop();
        setObstacleState(obstacleCruising, 100UL);
      }
      break;

    case obstacleEscaping:
      if (obstacleTimerExpired()) {
        motors.stop();
        setObstacleState(obstacleDetected, 100UL);
      }
      break;

    default:
      resetObstacleMode();
      break;
  }
}

#endif
