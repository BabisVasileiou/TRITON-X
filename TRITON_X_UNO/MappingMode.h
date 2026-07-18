/********** TRITON-X MAPPING mode
 * Purpose
 *   Preserves autonomous exploration, encoder/gyro dead-reckoning and
 *   sonar-ray telemetry for the browser occupancy grid.
 * Hardware
 *   Two 20-pulse encoders, MPU-6050, HC-SR04 and SG90 scan head.
 * Software
 *   Heading hold, periodic room survey, obstacle scan, dead-end reverse
 *   and finite-state turning. The ESP32 stores the 30 x 30 grid.
 * Limitations
 *   Dead-reckoning mapping is not full SLAM and accumulates drift.
 * Reference
 *   v4.0 Final, based on MAPPING v1.4 UART2-9600.
 **********/

#ifndef TRITON_X_MAPPING_MODE_H
#define TRITON_X_MAPPING_MODE_H

MappingState mappingState = mapIdle;
bool mappingScanForObstacle = false;
bool mappingPendingDeadEndTurn = false;
float mappingDesiredHeadingDeg = 0.0f;
float mappingTargetHeadingDeg = 0.0f;
float mappingScanLeftCm = SONAR_MAP_MAX_CM;
float mappingScanRightCm = SONAR_MAP_MAX_CM;
float mappingScanCenterCm = SONAR_MAP_MAX_CM;
float mappingCenterDistanceCm = SONAR_MAP_MAX_CM;
int8_t mappingLinearDirection = 0;
float mappingLinearTargetCm = 0.0f;
float mappingLinearStartX = 0.0f;
float mappingLinearStartY = 0.0f;
unsigned long mappingStateStartMs = 0;
unsigned long mappingLastCenterSonarMs = 0;
unsigned long mappingLastSurveyMs = 0;
unsigned long mappingRunStartMs = 0;
unsigned long mappingLastPoseMs = 0;

/******** function printMappingState
 * Purpose
 *   Writes the current mapping or cliff-recovery state.
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
void printMappingState(Print& output) {
  if (safetyState != safetyIdle) {
    switch (safetyState) {
      case safetyStopping:
        output.print(F("CLIFF_STOP"));
        return;
      case safetyLinearEscape:
        output.print(F("CLIFF_ESCAPE"));
        return;
      case safetyTurning:
        output.print(F("CLIFF_TURN"));
        return;
      case safetyPostHold:
        output.print(F("CLIFF_CLEAR"));
        return;
      case safetyLocked:
        output.print(F("CLIFF_LOCK"));
        return;
      default:
        break;
    }
  }

  switch (mappingState) {
    case mapIdle:
      output.print(F("IDLE"));
      break;
    case mapCruise:
      output.print(F("CRUISE"));
      break;
    case mapScanLeft:
    case mapScanRight:
    case mapScanCenter:
      output.print(F("SCANNING"));
      break;
    case mapDecide:
      output.print(F("DECIDE"));
      break;
    case mapLinearMove:
      output.print(F("REVERSING"));
      break;
    case mapTurning:
      output.print(F("TURNING"));
      break;
    case mapFault:
      output.print(F("FAULT"));
      break;
    default:
      output.print(F("UNKNOWN"));
      break;
  }
}

/******** function sendMappingRay
 * Purpose
 *   Sends one world-referenced sonar ray to the ESP32 map.
 * Arguments
 *   relativeAngleDeg Sonar angle relative to robot heading in degrees.
 *   distanceCm Distance in centimetres.
 *   hit Reference set true when a valid obstacle echo exists.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void sendMappingRay(float relativeAngleDeg, float distanceCm, bool hit) {
  float bearing = wrapAngle180(headingDeg + relativeAngleDeg);
  espSerial.print(F("RAY:"));
  espSerial.print(poseXcm, 1);
  espSerial.print(',');
  espSerial.print(poseYcm, 1);
  espSerial.print(',');
  espSerial.print(bearing, 1);
  espSerial.print(',');
  espSerial.print(distanceCm, 1);
  espSerial.print(',');
  espSerial.println(hit ? 1 : 0);
}

/******** function sendMappingPose
 * Purpose
 *   Sends periodic pose, encoder and mapping-state telemetry.
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
void sendMappingPose(void) {
  unsigned long now = millis();
  if (selectedMode != modeMapping ||
      now - mappingLastPoseMs < POSE_TELEMETRY_MS) {
    return;
  }
  mappingLastPoseMs = now;

  long leftTicks;
  long rightTicks;
  noInterrupts();
  leftTicks = encoderLeftTicks;
  rightTicks = encoderRightTicks;
  interrupts();

  unsigned long elapsed = modeRunning ?
    (now - mappingRunStartMs) / 1000UL : 0UL;

  espSerial.print(F("POSE:"));
  espSerial.print(poseXcm, 1);
  espSerial.print(',');
  espSerial.print(poseYcm, 1);
  espSerial.print(',');
  espSerial.print(headingDeg, 1);
  espSerial.print(',');
  espSerial.print(mappingCenterDistanceCm, 1);
  espSerial.print(',');
  printMappingState(espSerial);
  espSerial.print(',');
  espSerial.print(leftTicks);
  espSerial.print(',');
  espSerial.print(rightTicks);
  espSerial.print(',');
  espSerial.print(totalTravelCm, 1);
  espSerial.print(',');
  espSerial.println(elapsed);
}

/******** function readAndSendMappingSonar
 * Purpose
 *   Measures sonar distance and transmits the corresponding ray.
 * Arguments
 *   relativeAngle Sonar angle relative to robot heading in degrees.
 *   hit Reference set true when a valid obstacle echo exists.
 * Results
 *   Returns the calculated value as a floating-point number.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
float readAndSendMappingSonar(float relativeAngle, bool& hit) {
  float distance = sonar.readDistanceCm(hit);
  sendMappingRay(relativeAngle, distance, hit);
  if (fabs(relativeAngle) < 1.0f) {
    mappingCenterDistanceCm = distance;
  }
  return distance;
}

/******** function setMappingServoState
 * Purpose
 *   Moves the scan servo and enters the requested map state.
 * Arguments
 *   angle Servo or geometric angle in degrees.
 *   state Requested finite-state-machine state.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void setMappingServoState(uint8_t angle, MappingState state) {
  sonarServo.write(angle);
  mappingState = state;
  mappingStateStartMs = millis();
}

/******** function beginMappingTurn
 * Purpose
 *   Starts a gyro-controlled signed mapping turn.
 * Arguments
 *   direction Signed direction: +1 left/forward, -1 right/reverse.
 *   degrees Requested turn magnitude in degrees.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void beginMappingTurn(int8_t direction, float degrees) {
  motors.stop();
  mappingTargetHeadingDeg = wrapAngle180(
    headingDeg + direction * degrees
  );
  mappingState = mapTurning;
  mappingStateStartMs = millis();
  if (direction > 0) {
    reportStatus(F("MAPPING"), F("Turning left"));
  } else {
    reportStatus(F("MAPPING"), F("Turning right"));
  }
}

/******** function beginMappingLinearMove
 * Purpose
 *   Starts a measured forward or reverse mapping movement.
 * Arguments
 *   direction Signed direction: +1 left/forward, -1 right/reverse.
 *   distanceCm Distance in centimetres.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void beginMappingLinearMove(int8_t direction, float distanceCm) {
  mappingLinearDirection = direction;
  mappingLinearTargetCm = distanceCm;
  mappingLinearStartX = poseXcm;
  mappingLinearStartY = poseYcm;
  mappingState = mapLinearMove;
  mappingStateStartMs = millis();
  motors.setMotors(direction * ESCAPE_PWM, direction * ESCAPE_PWM);
  if (direction < 0) {
    reportStatus(F("MAPPING"), F("Backing away"));
  } else {
    reportStatus(F("MAPPING"), F("Moving forward"));
  }
}

/******** function mappingLinearMoveFinished
 * Purpose
 *   Checks whether the requested linear distance was covered.
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
bool mappingLinearMoveFinished(void) {
  return distanceBetween(
    mappingLinearStartX, mappingLinearStartY, poseXcm, poseYcm
  ) >= mappingLinearTargetCm;
}

/******** function beginMappingScan
 * Purpose
 *   Starts a left-right-center autonomous sonar survey.
 * Arguments
 *   obstacleDecision True when the scan must choose an avoidance path.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void beginMappingScan(bool obstacleDecision) {
  motors.stop();
  mappingScanForObstacle = obstacleDecision;
  mappingScanLeftCm = SONAR_MAP_MAX_CM;
  mappingScanRightCm = SONAR_MAP_MAX_CM;
  mappingScanCenterCm = SONAR_MAP_MAX_CM;
  setMappingServoState(SERVO_LEFT, mapScanLeft);
  if (obstacleDecision) {
    reportStatus(F("MAPPING"), F("Obstacle scan"));
  } else {
    reportStatus(F("MAPPING"), F("Room survey"));
  }
}

/******** function resumeMappingCruise
 * Purpose
 *   Centers the sensor head and returns to heading-held cruise.
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
void resumeMappingCruise(void) {
  sonarServo.write(SERVO_CENTER);
  mappingDesiredHeadingDeg = headingDeg;
  mappingState = modeRunning ? mapCruise : mapIdle;
  mappingStateStartMs = millis();
  mappingLastSurveyMs = millis();
  if (modeRunning) {
    reportStatus(F("MAPPING"), F("Exploring"));
  } else {
    motors.stop();
    reportStatus(F("MAPPING"), F("Stopped"));
  }
}

/******** function resetMappingMode
 * Purpose
 *   Resets the mapping FSM and optionally zeros dead reckoning.
 * Arguments
 *   resetPose True to zero pose and encoder dead reckoning.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void resetMappingMode(bool resetPose) {
  motors.stop();
  sonarServo.write(SERVO_CENTER);
  mappingState = mapIdle;
  mappingScanForObstacle = false;
  mappingPendingDeadEndTurn = false;
  mappingDesiredHeadingDeg = headingDeg;
  mappingTargetHeadingDeg = headingDeg;
  mappingScanLeftCm = SONAR_MAP_MAX_CM;
  mappingScanRightCm = SONAR_MAP_MAX_CM;
  mappingScanCenterCm = SONAR_MAP_MAX_CM;
  mappingCenterDistanceCm = SONAR_MAP_MAX_CM;
  mappingStateStartMs = millis();
  mappingLastCenterSonarMs = 0;
  mappingLastSurveyMs = millis();
  mappingLastPoseMs = 0;
  if (resetPose) {
    resetPoseAndEncoders();
  }
}

/******** function startMappingMode
 * Purpose
 *   Starts a new autonomous mapping run.
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
void startMappingMode(void) {
  mappingRunStartMs = millis();
  mappingDesiredHeadingDeg = headingDeg;
  mappingState = mapCruise;
  mappingStateStartMs = millis();
  mappingLastSurveyMs = millis();
  mappingLastCenterSonarMs = 0;
  reportStatus(F("MAPPING"), F("Exploring"));
}

/******** function runMappingCruise
 * Purpose
 *   Runs heading-held mapping cruise and periodic sonar surveys.
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
void runMappingCruise(void) {
  if (!modeRunning) {
    motors.stop();
    mappingState = mapIdle;
    return;
  }

  if (frontObstacleDetected() ||
      mappingCenterDistanceCm < MAP_SAFE_FRONT_CM) {
    motors.stop();
    if (mappingCenterDistanceCm < MAP_CRITICAL_FRONT_CM &&
        rearPathSafe()) {
      mappingPendingDeadEndTurn = false;
      beginMappingLinearMove(-1, MAP_OBSTACLE_REVERSE_CM);
    } else {
      beginMappingScan(true);
    }
    return;
  }

  float headingError = angleDifference(
    mappingDesiredHeadingDeg, headingDeg
  );
  int correction = (int)(HEADING_HOLD_KP * headingError);
  correction = constrain(
    correction, -HEADING_CORR_MAX_PWM, HEADING_CORR_MAX_PWM
  );
  motors.setMotors(
    MAPPING_CRUISE_PWM - correction,
    MAPPING_CRUISE_PWM + correction
  );

  unsigned long now = millis();
  if (now - mappingLastCenterSonarMs >= CENTER_SONAR_INTERVAL_MS) {
    mappingLastCenterSonarMs = now;
    bool hit = false;
    mappingCenterDistanceCm =
      readAndSendMappingSonar(0.0f, hit);
  }
  if (now - mappingLastSurveyMs >= SURVEY_INTERVAL_MS) {
    beginMappingScan(false);
  }
}

/******** function decideMappingPath
 * Purpose
 *   Chooses a turn or dead-end reverse from scan distances.
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
void decideMappingPath(void) {
  bool leftBlocked = mappingScanLeftCm < MAP_SIDE_BLOCKED_CM;
  bool rightBlocked = mappingScanRightCm < MAP_SIDE_BLOCKED_CM;

  if (leftBlocked && rightBlocked) {
    if (rearPathSafe()) {
      mappingPendingDeadEndTurn = true;
      beginMappingLinearMove(-1, MAP_DEAD_END_REVERSE_CM);
    } else {
      beginMappingTurn(1, MAP_DEAD_END_TURN_DEG);
    }
    return;
  }

  if (mappingScanLeftCm >= mappingScanRightCm) {
    beginMappingTurn(1, MAP_NORMAL_TURN_DEG);
  } else {
    beginMappingTurn(-1, MAP_NORMAL_TURN_DEG);
  }
}

/******** function runMappingMode
 * Purpose
 *   Runs the autonomous mapping finite-state machine.
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
void runMappingMode(void) {
  if (!modeRunning || safetyState != safetyIdle) {
    motors.stop();
    return;
  }

  switch (mappingState) {
    case mapIdle:
      motors.stop();
      break;

    case mapCruise:
      runMappingCruise();
      break;

    case mapScanLeft:
      motors.stop();
      if (millis() - mappingStateStartMs >= SERVO_SETTLE_MS) {
        bool hit = false;
        mappingScanLeftCm =
          readAndSendMappingSonar(60.0f, hit);
        setMappingServoState(SERVO_RIGHT, mapScanRight);
      }
      break;

    case mapScanRight:
      motors.stop();
      if (millis() - mappingStateStartMs >= SERVO_SETTLE_MS) {
        bool hit = false;
        mappingScanRightCm =
          readAndSendMappingSonar(-60.0f, hit);
        setMappingServoState(SERVO_CENTER, mapScanCenter);
      }
      break;

    case mapScanCenter:
      motors.stop();
      if (millis() - mappingStateStartMs >= SERVO_SETTLE_MS) {
        bool hit = false;
        mappingScanCenterCm =
          readAndSendMappingSonar(0.0f, hit);
        mappingCenterDistanceCm = mappingScanCenterCm;
        mappingState = mapDecide;
      }
      break;

    case mapDecide:
      if (mappingScanForObstacle) {
        decideMappingPath();
      } else {
        resumeMappingCruise();
      }
      break;

    case mapLinearMove: {
      bool directionSafe = (mappingLinearDirection < 0) ?
        rearPathSafe() : frontPathSafe();
      if (!directionSafe || mappingLinearMoveFinished()) {
        motors.stop();
        if (mappingPendingDeadEndTurn) {
          mappingPendingDeadEndTurn = false;
          int8_t direction =
            (mappingScanLeftCm >= mappingScanRightCm) ? 1 : -1;
          beginMappingTurn(direction, MAP_DEAD_END_TURN_DEG);
        } else {
          beginMappingScan(true);
        }
      }
      break;
    }

    case mapTurning:
      if (runHeadingTurn(
            MAPPING_TURN_PWM, mappingTargetHeadingDeg
          ) ||
          millis() - mappingStateStartMs > TURN_TIMEOUT_MS) {
        motors.stop();
        mappingDesiredHeadingDeg = headingDeg;
        resumeMappingCruise();
      } else if (millis() - mappingLastCenterSonarMs >= 180UL) {
        mappingLastCenterSonarMs = millis();
        bool hit = false;
        readAndSendMappingSonar(0.0f, hit);
      }
      break;

    case mapFault:
      motors.stop();
      break;

    default:
      resetMappingMode(false);
      break;
  }
}

#endif
