/********** TRITON-X integrated five-mode controller (Arduino UNO)
 * Purpose
 *   Runs MANUAL, LEVEL, LINE, OBSTACLE and MAPPING modes selected from
 *   the ESP32-CAM browser dashboard. Selection never starts motion; the
 *   operator must issue START and may always issue STOP.
 * Hardware
 *   Arduino UNO R3 with Sensor Shield v5.0; L298N; HC-SR04; SG90;
 *   four obstacle sensors; four cliff sensors; PCF8574T; LCD1602 I2C;
 *   MPU-6050; two 20-pulse wheel encoders; ESP32-CAM UART bridge.
 * Software
 *   Modular Arduino C/C++, non-blocking finite-state machines, fixed
 *   buffers, flash-resident status strings and a Timer2 UART at 9600.
 * Safety
 *   Mode change, STOP, browser loss and communication timeout stop the
 *   motors. Cliff recovery has final authority and may latch a lock.
 * Limitations
 *   LINE uses D2/D3 as line sensors and cannot simultaneously classify
 *   those signals as cliffs. MAPPING is dead-reckoning, not full SLAM.
 * Reference
 *   v4.0 Final, C. Vasileiou, MSc PADA, July 2026.
 **********/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Configuration.h"
#include "RobotTypes.h"
#include "TimerUart.h"
#include "Hardware.h"
#include "ModeManager.h"
#include "SafetyManager.h"
#include "ManualMode.h"
#include "LevelMode.h"
#include "LineMode.h"
#include "ObstacleMode.h"
#include "MappingMode.h"
#include "Protocol.h"

/******** function setup
 * Purpose
 *   Initializes the controller, peripherals and communication services.
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
void setup(void) {
  Wire.begin();
  Wire.setWireTimeout(25000UL, true);

  lcd.init();
  lcd.backlight();
  reportStatus(F("TRITON-X v4"), F("Init hardware"));

  motors.begin();
  sonar.begin();
  pinMode(OBST_FL_PIN, INPUT);
  pinMode(CLIFF_LF_PIN, INPUT);
  pinMode(CLIFF_RF_PIN, INPUT);

  bool pcfOk = expander.begin(PCF8574_ADDR);
  pcfFaultLatched = !pcfOk;
  sensors.pcfHealthy = pcfOk;

  sonarServo.attach(SERVO_PIN);
  sonarServo.write(SERVO_CENTER);

  setupEncoderInterrupts();
  attachInterrupt(
    digitalPinToInterrupt(CLIFF_LF_PIN), cliffLeftFrontIsr, RISING
  );
  attachInterrupt(
    digitalPinToInterrupt(CLIFF_RF_PIN), cliffRightFrontIsr, RISING
  );

  reportStatus(F("TRITON-X v4"), F("Init MPU"));
  mpuHealthy = gyro.begin();
  if (mpuHealthy) {
    reportStatus(F("MPU CALIBRATE"), F("Keep robot still"));
    mpuHealthy = gyro.calibrate(400);
  }

  resetPoseAndEncoders();
  for (uint8_t sample = 0;
       sample < SENSOR_DEBOUNCE_SAMPLES;
       sample++) {
    refreshSensors(true);
    delay(5);
  }
  resetModeControllers(true);

  /* Start Timer2 only after all I2C initialization is complete. */
  espSerial.begin(9600);

  if (!sensors.pcfHealthy) {
    reportStatus(F("PCF8574 FAULT"), F("Check I2C"));
  } else if (!mpuHealthy) {
    reportStatus(F("READY / MPU ERR"), F("No mapping"));
  } else {
    reportStatus(F("TRITON-X READY"), F("Select mode"));
  }
}

/******** function loop
 * Purpose
 *   Runs the non-blocking supervisor and selected mode continuously.
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
void loop(void) {
  processIncomingCommands();
  refreshSensors();
  processFrontCliffInterrupt();
  updateOdometry();
  handleLinkWatchdog();

  if (!sensors.pcfHealthy && modeRunning &&
      selectedMode != modeLine) {
    forceSystemStop(true);
    sendEvent(F("PCF_FAULT_STOP"));
    reportStatus(F("PCF8574 FAULT"), F("Motion stopped"));
  }

  handleCliffSafety();
  bool interruptHold = frontInterruptHoldBlocksMotion();
  if (safetyState == safetyIdle && !interruptHold) {
    runSelectedMode();
  }

  sendMappingPose();
  sendSystemTelemetry();
}
