/********** TRITON-X LINE mode
 * Purpose
 *   Preserves the validated two-sensor PD line follower and terminal
 *   sonar obstacle stop.
 * Hardware
 *   D3 left and D2 right reflectance sensors, HC-SR04 fixed forward.
 * Software
 *   The black line is HIGH and the light floor is LOW. Sensors are
 *   mounted outside the tape, so both LOW is the normal centered state.
 * Limitations
 *   The two sensors cannot distinguish centered travel from a lost line.
 *   D2/D3 are treated as line sensors, not cliff sensors, in this mode.
 * Reference
 *   v4.0 Final, based on LINE standalone v1.0.
 **********/

#ifndef TRITON_X_LINE_MODE_H
#define TRITON_X_LINE_MODE_H

int lineLastError = 0;
bool lineTerminalStop = false;
float lineLatestDistanceCm = SONAR_MAP_MAX_CM;
unsigned long lineLastSonarMs = 0;
uint8_t lineDisplayState = 255;

/******** function resetLineMode
 * Purpose
 *   Resets LINE controller and terminal obstacle state.
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
void resetLineMode(void) {
  lineLastError = 0;
  lineTerminalStop = false;
  lineLatestDistanceCm = SONAR_MAP_MAX_CM;
  lineLastSonarMs = 0;
  lineDisplayState = 255;
}

/******** function updateLineDisplay
 * Purpose
 *   Displays the current two-sensor line interpretation.
 * Arguments
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
void updateLineDisplay(uint8_t state) {
  if (state == lineDisplayState) {
    return;
  }
  lineDisplayState = state;
  switch (state) {
    case 0:
      reportStatus(F("LINE FOLLOW"), F("Centered"));
      break;
    case 1:
      reportStatus(F("LINE FOLLOW"), F("Correct left"));
      break;
    case 2:
      reportStatus(F("LINE FOLLOW"), F("Correct right"));
      break;
    case 3:
      reportStatus(F("LINE FOLLOW"), F("Crossing"));
      break;
    default:
      break;
  }
}

/******** function runLineMode
 * Purpose
 *   Runs the validated PD line follower and terminal sonar stop.
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
void runLineMode(void) {
  if (!modeRunning || lineTerminalStop) {
    motors.stop();
    return;
  }

  if (millis() - lineLastSonarMs >= LINE_SONAR_INTERVAL_MS) {
    lineLastSonarMs = millis();
    bool hit = false;
    lineLatestDistanceCm = sonar.readDistanceCm(hit);
    if (hit && lineLatestDistanceCm < LINE_OBSTACLE_STOP_CM) {
      motors.stop();
      lineTerminalStop = true;
      modeRunning = false;
      sendEvent(F("LINE_OBSTACLE_STOP"));
      reportStatus(F("OBSTACLE!"), F("LINE stopped"));
      return;
    }
  }

  bool leftOnLine = digitalRead(CLIFF_LF_PIN) == HIGH;
  bool rightOnLine = digitalRead(CLIFF_RF_PIN) == HIGH;
  int error = 0;
  uint8_t displayState = 0;

  if (leftOnLine && !rightOnLine) {
    error = -1;
    displayState = 1;
  } else if (rightOnLine && !leftOnLine) {
    error = 1;
    displayState = 2;
  } else if (leftOnLine && rightOnLine) {
    displayState = 3;
  }

  float derivative = error - lineLastError;
  lineLastError = error;
  float correction = LINE_KP * error + LINE_KD * derivative;
  int leftSpeed =
    LINE_SPEED_PWM + (int)(correction * LINE_SPEED_PWM);
  int rightSpeed =
    LINE_SPEED_PWM - (int)(correction * LINE_SPEED_PWM);
  leftSpeed = constrain(
    leftSpeed, -LINE_SPEED_PWM, LINE_SPEED_PWM
  );
  rightSpeed = constrain(
    rightSpeed, -LINE_SPEED_PWM, LINE_SPEED_PWM
  );
  motors.setMotors(leftSpeed, rightSpeed);
  updateLineDisplay(displayState);
}

#endif
