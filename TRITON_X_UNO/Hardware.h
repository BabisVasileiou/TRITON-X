/********** TRITON-X common hardware layer
 * Purpose
 *   Provides one shared implementation for motors, sonar, I2C inputs,
 *   MPU-6050 heading, wheel encoders, LCD and sensor filtering.
 * Hardware
 *   The exact pin allocation is defined in Configuration.h.
 * Software
 *   Non-blocking sensor filtering is combined with 10 ms odometry.
 * Safety
 *   PCF8574 communication failure is latched and prevents operation.
 * Reference
 *   v4.0 Final, derived from validated standalone tests and MAPPING v1.4,
 *   C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_HARDWARE_H
#define TRITON_X_HARDWARE_H

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;
volatile int8_t leftMotorDirection = 0;
volatile int8_t rightMotorDirection = 0;
volatile uint8_t previousPortB = 0;
volatile bool frontCliffInterruptFlag = false;

ISR(PCINT0_vect) {
  uint8_t currentPortB = PINB;
  bool leftNow = (currentPortB & _BV(PB2)) != 0;
  bool leftBefore = (previousPortB & _BV(PB2)) != 0;
  bool rightNow = (currentPortB & _BV(PB4)) != 0;
  bool rightBefore = (previousPortB & _BV(PB4)) != 0;

  if (leftNow && !leftBefore && leftMotorDirection != 0) {
    encoderLeftTicks += leftMotorDirection;
  }
  if (rightNow && !rightBefore && rightMotorDirection != 0) {
    encoderRightTicks += rightMotorDirection;
  }
  previousPortB = currentPortB;
}

/******** function wrapAngle180
 * Purpose
 *   Normalizes an angle to the interval -180 to +180 degrees.
 * Arguments
 *   angle Servo or geometric angle in degrees.
 * Results
 *   Returns the calculated value as a floating-point number.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
float wrapAngle180(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle <= -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

/******** function angleDifference
 * Purpose
 *   Calculates the signed shortest angular error.
 * Arguments
 *   target Target angle in degrees.
 *   current Current angle in degrees.
 * Results
 *   Returns the calculated value as a floating-point number.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
float angleDifference(float target, float current) {
  return wrapAngle180(target - current);
}

/******** function distanceBetween
 * Purpose
 *   Calculates planar distance between two positions.
 * Arguments
 *   x1 First X coordinate in centimetres.
 *   y1 First Y coordinate in centimetres.
 *   x2 Second X coordinate in centimetres.
 *   y2 Second Y coordinate in centimetres.
 * Results
 *   Returns the calculated value as a floating-point number.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
float distanceBetween(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

/***** class MotorDriver
 * Purpose
 *   Controls the L298N as a signed differential-drive actuator.
 * Specifiers
 *   None. Pins are compile-time constants.
 * Methods
 *   begin() configures outputs. setMotors() commands signed PWM.
 *   stop() removes PWM and drives all direction inputs LOW.
 * Hardware
 *   L298N IN1..IN4 and ENA/ENB.
 * Software
 *   Updates encoder direction inference atomically.
 * Reference
 *   v4.0 Final, physical polarity validated on TRITON-X.
 **********/
class MotorDriver {
  public:
    void begin(void) {
      pinMode(IN1_PIN, OUTPUT);
      pinMode(IN2_PIN, OUTPUT);
      pinMode(IN3_PIN, OUTPUT);
      pinMode(IN4_PIN, OUTPUT);
      pinMode(ENA_PIN, OUTPUT);
      pinMode(ENB_PIN, OUTPUT);
      stop();
    }

    void setMotors(int leftSpeed, int rightSpeed) {
      leftSpeed = constrain(leftSpeed, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
      rightSpeed = constrain(rightSpeed, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

      noInterrupts();
      leftMotorDirection =
        (leftSpeed > 0) ? 1 : ((leftSpeed < 0) ? -1 : 0);
      rightMotorDirection =
        (rightSpeed > 0) ? 1 : ((rightSpeed < 0) ? -1 : 0);
      interrupts();

      driveSide(leftSpeed, IN1_PIN, IN2_PIN, ENA_PIN);
      driveSide(rightSpeed, IN3_PIN, IN4_PIN, ENB_PIN);
    }

    void stop(void) {
      noInterrupts();
      leftMotorDirection = 0;
      rightMotorDirection = 0;
      interrupts();
      analogWrite(ENA_PIN, 0);
      analogWrite(ENB_PIN, 0);
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, LOW);
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, LOW);
    }

  private:
    void driveSide(int speed, uint8_t inA, uint8_t inB,
                   uint8_t enablePin) {
      if (speed > 0) {
        digitalWrite(inA, LOW);
        digitalWrite(inB, HIGH);
        analogWrite(enablePin, speed);
      } else if (speed < 0) {
        digitalWrite(inA, HIGH);
        digitalWrite(inB, LOW);
        analogWrite(enablePin, abs(speed));
      } else {
        digitalWrite(inA, LOW);
        digitalWrite(inB, LOW);
        analogWrite(enablePin, 0);
      }
    }
};

/***** class Sonar
 * Purpose
 *   Measures HC-SR04 distance for collision avoidance and mapping.
 * Specifiers
 *   Trigger and echo pin numbers.
 * Methods
 *   begin() configures pins. readDistanceCm() returns 2..180 cm and
 *   reports whether a valid map hit was received.
 * Hardware
 *   HC-SR04 on D8/D9.
 * Software
 *   Uses pulseIn() with a bounded 12 ms timeout.
 * Reference
 *   v4.0 Final, derived from MAPPING v1.4.
 **********/
class Sonar {
  public:
    Sonar(uint8_t triggerPin, uint8_t echoPin)
      : trig(triggerPin), echo(echoPin) {}

    void begin(void) {
      pinMode(trig, OUTPUT);
      pinMode(echo, INPUT);
      digitalWrite(trig, LOW);
    }

    float readDistanceCm(bool& hit) {
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      unsigned long duration = pulseIn(echo, HIGH, 12000UL);

      if (duration == 0) {
        hit = false;
        return SONAR_MAP_MAX_CM;
      }

      float distance = duration * 0.0343f / 2.0f;
      if (distance < 2.0f) {
        hit = true;
        return 2.0f;
      }
      if (distance >= SONAR_MAP_MAX_CM) {
        hit = false;
        return SONAR_MAP_MAX_CM;
      }
      hit = true;
      return distance;
    }

  private:
    uint8_t trig;
    uint8_t echo;
};

/***** class IoExpander
 * Purpose
 *   Reads the PCF8574T obstacle and rear-cliff inputs.
 * Specifiers
 *   I2C address supplied to begin().
 * Methods
 *   begin() enables quasi-input mode. readByte() returns one input byte.
 * Hardware
 *   PCF8574T at address 0x20.
 * Software
 *   Returns false when I2C communication fails.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
class IoExpander {
  public:
    bool begin(uint8_t address) {
      addr = address;
      Wire.beginTransmission(addr);
      Wire.write(0xFF);
      return Wire.endTransmission() == 0;
    }

    bool readByte(uint8_t& value) {
      uint8_t received = Wire.requestFrom(addr, (uint8_t)1);
      if (received != 1 || !Wire.available()) {
        return false;
      }
      value = Wire.read();
      return true;
    }

  private:
    uint8_t addr = 0;
};

class DebouncedSignal {
  public:
    void update(bool rawValue) {
      if (!initialized) {
        initialized = true;
        stableValue = rawValue;
        candidateValue = rawValue;
        sampleCount = 1;
        return;
      }
      if (rawValue == candidateValue) {
        if (sampleCount < SENSOR_DEBOUNCE_SAMPLES) {
          sampleCount++;
        }
      } else {
        candidateValue = rawValue;
        sampleCount = 1;
      }
      if (sampleCount >= SENSOR_DEBOUNCE_SAMPLES) {
        stableValue = candidateValue;
      }
    }

    bool value(void) const {
      return stableValue;
    }

  private:
    bool initialized = false;
    bool stableValue = false;
    bool candidateValue = false;
    uint8_t sampleCount = 0;
};

/***** class Mpu6050Gyro
 * Purpose
 *   Supplies calibrated Z-axis angular rate for heading estimation.
 * Specifiers
 *   None. Uses address 0x68 and the +/-250 degree/s range.
 * Methods
 *   begin() configures the sensor. calibrate() estimates stationary bias.
 *   readZDegPerSec() returns the corrected angular rate.
 * Hardware
 *   MPU-6050 on the common A4/A5 I2C bus.
 * Software
 *   Direct register access avoids an external MPU library.
 * Reference
 *   v4.0 Final, derived from MAPPING v1.4.
 **********/
class Mpu6050Gyro {
  public:
    bool begin(void) {
      if (!writeRegister(0x6B, 0x00)) {
        return false;
      }
      delay(50);
      if (!writeRegister(0x1A, 0x03)) {
        return false;
      }
      if (!writeRegister(0x1B, 0x00)) {
        return false;
      }
      return true;
    }

    bool calibrate(uint16_t samples) {
      long sum = 0;
      uint16_t valid = 0;
      for (uint16_t i = 0; i < samples; i++) {
        int16_t rawZ = 0;
        if (readRawGyroZ(rawZ)) {
          sum += rawZ;
          valid++;
        }
        delay(3);
      }
      if (valid < samples / 2) {
        return false;
      }
      biasRaw = (float)sum / (float)valid;
      return true;
    }

    bool readZDegPerSec(float& value) {
      int16_t rawZ = 0;
      if (!readRawGyroZ(rawZ)) {
        return false;
      }
      value = ((float)rawZ - biasRaw) / 131.0f;
      return true;
    }

  private:
    float biasRaw = 0.0f;

    bool writeRegister(uint8_t reg, uint8_t value) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(reg);
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    bool readRawGyroZ(int16_t& rawZ) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x43);
      if (Wire.endTransmission(false) != 0) {
        return false;
      }
      uint8_t count = Wire.requestFrom(MPU6050_ADDR, (uint8_t)6);
      if (count != 6) {
        return false;
      }
      uint8_t bytes[6];
      for (uint8_t i = 0; i < 6; i++) {
        if (!Wire.available()) {
          return false;
        }
        bytes[i] = Wire.read();
      }
      rawZ = (int16_t)(((uint16_t)bytes[4] << 8) | bytes[5]);
      return true;
    }
};

FixedTimerUart espSerial;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
MotorDriver motors;
Sonar sonar(TRIG_PIN, ECHO_PIN);
IoExpander expander;
Mpu6050Gyro gyro;
Servo sonarServo;

SensorSnapshot sensors = {
  false, false, false, false,
  false, false, false, false,
  false
};

DebouncedSignal debouncedSensors[sensorCount];
unsigned long lastSensorSampleMs = 0;
bool pcfFaultLatched = false;
uint8_t pcfRecoveryCounter = 0;
bool mpuHealthy = false;

float poseXcm = 0.0f;
float poseYcm = 0.0f;
float headingDeg = 0.0f;
float encoderHeadingDeg = 0.0f;
float totalTravelCm = 0.0f;
long previousLeftTicks = 0;
long previousRightTicks = 0;
unsigned long lastOdometryUs = 0;

const __FlashStringHelper* lastLcdLine1 = NULL;
const __FlashStringHelper* lastLcdLine2 = NULL;

void reportStatus(const __FlashStringHelper* line1,
                  const __FlashStringHelper* line2) {
  if (line1 == lastLcdLine1 && line2 == lastLcdLine2) {
    return;
  }
  lastLcdLine1 = line1;
  lastLcdLine2 = line2;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

/******** function cliffLeftFrontIsr
 * Purpose
 *   Records a front-left cliff interrupt for foreground handling.
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
void cliffLeftFrontIsr(void) {
  frontCliffInterruptFlag = true;
}

/******** function cliffRightFrontIsr
 * Purpose
 *   Records a front-right cliff interrupt for foreground handling.
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
void cliffRightFrontIsr(void) {
  frontCliffInterruptFlag = true;
}

/******** function setupEncoderInterrupts
 * Purpose
 *   Configures D10 and D12 pin-change encoder interrupts.
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
void setupEncoderInterrupts(void) {
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  previousPortB = PINB;
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT2) | _BV(PCINT4);
}

/******** function copySensors
 * Purpose
 *   Copies debounced input states into the shared sensor snapshot.
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
void copySensors(void) {
  sensors.obstacleFrontLeft =
    debouncedSensors[sensorObstacleFrontLeft].value();
  sensors.obstacleFrontRight =
    debouncedSensors[sensorObstacleFrontRight].value();
  sensors.obstacleRearLeft =
    debouncedSensors[sensorObstacleRearLeft].value();
  sensors.obstacleRearRight =
    debouncedSensors[sensorObstacleRearRight].value();
  sensors.cliffFrontLeft =
    debouncedSensors[sensorCliffFrontLeft].value();
  sensors.cliffFrontRight =
    debouncedSensors[sensorCliffFrontRight].value();
  sensors.cliffRearLeft =
    debouncedSensors[sensorCliffRearLeft].value();
  sensors.cliffRearRight =
    debouncedSensors[sensorCliffRearRight].value();
}

/******** function refreshSensors
 * Purpose
 *   Samples and debounces direct and PCF8574 sensor inputs.
 * Arguments
 *   forceUpdate True to bypass the normal sensor sampling interval.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void refreshSensors(bool forceUpdate = false) {
  unsigned long now = millis();
  if (!forceUpdate && now - lastSensorSampleMs < SENSOR_SAMPLE_MS) {
    return;
  }
  lastSensorSampleMs = now;

  debouncedSensors[sensorObstacleFrontLeft].update(
    digitalRead(OBST_FL_PIN) == LOW
  );
  debouncedSensors[sensorCliffFrontLeft].update(
    digitalRead(CLIFF_LF_PIN) == HIGH
  );
  debouncedSensors[sensorCliffFrontRight].update(
    digitalRead(CLIFF_RF_PIN) == HIGH
  );

  uint8_t bits = 0xFF;
  bool pcfOk = expander.readByte(bits);
  if (!pcfOk) {
    pcfFaultLatched = true;
    pcfRecoveryCounter = 0;
    sensors.pcfHealthy = false;
    copySensors();
    return;
  }

  if (pcfFaultLatched) {
    if (pcfRecoveryCounter < PCF_RECOVERY_SAMPLES) {
      pcfRecoveryCounter++;
    }
    if (pcfRecoveryCounter >= PCF_RECOVERY_SAMPLES) {
      pcfFaultLatched = false;
    }
  }
  sensors.pcfHealthy = !pcfFaultLatched;

  debouncedSensors[sensorObstacleFrontRight].update(
    (bits & (1U << BIT_OBST_FR)) == 0
  );
  debouncedSensors[sensorObstacleRearLeft].update(
    (bits & (1U << BIT_OBST_RL)) == 0
  );
  debouncedSensors[sensorObstacleRearRight].update(
    (bits & (1U << BIT_OBST_RR)) == 0
  );
  debouncedSensors[sensorCliffRearLeft].update(
    (bits & (1U << BIT_CLIFF_RL)) != 0
  );
  debouncedSensors[sensorCliffRearRight].update(
    (bits & (1U << BIT_CLIFF_RR)) != 0
  );
  copySensors();
}

/******** function frontCliffDetected
 * Purpose
 *   Reports whether either front cliff sensor is active.
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
bool frontCliffDetected(void) {
  return sensors.cliffFrontLeft || sensors.cliffFrontRight;
}

/******** function rearCliffDetected
 * Purpose
 *   Reports whether either rear cliff sensor is active.
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
bool rearCliffDetected(void) {
  return sensors.cliffRearLeft || sensors.cliffRearRight;
}

/******** function anyCliffDetected
 * Purpose
 *   Reports whether any cliff sensor is active.
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
bool anyCliffDetected(void) {
  return frontCliffDetected() || rearCliffDetected();
}

/******** function frontObstacleDetected
 * Purpose
 *   Reports whether either front obstacle sensor is active.
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
bool frontObstacleDetected(void) {
  return sensors.obstacleFrontLeft || sensors.obstacleFrontRight;
}

/******** function rearObstacleDetected
 * Purpose
 *   Reports whether either rear obstacle sensor is active.
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
bool rearObstacleDetected(void) {
  return sensors.obstacleRearLeft || sensors.obstacleRearRight;
}

/******** function frontPathSafe
 * Purpose
 *   Checks whether forward automatic motion is currently safe.
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
bool frontPathSafe(void) {
  return !frontCliffDetected() && !frontObstacleDetected();
}

/******** function rearPathSafe
 * Purpose
 *   Checks whether reverse automatic motion is currently safe.
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
bool rearPathSafe(void) {
  return !rearCliffDetected() && !rearObstacleDetected();
}

/******** function turnAwayFromCliff
 * Purpose
 *   Selects a signed turn direction away from the detected cliff.
 * Arguments
 *   None.
 * Results
 *   Returns a signed direction value.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
int8_t turnAwayFromCliff(void) {
  bool left = sensors.cliffFrontLeft || sensors.cliffRearLeft;
  bool right = sensors.cliffFrontRight || sensors.cliffRearRight;
  if (left && !right) {
    return -1;
  }
  if (right && !left) {
    return 1;
  }
  return 1;
}

/******** function resetPoseAndEncoders
 * Purpose
 *   Zeros encoder counts, pose, heading and travelled distance.
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
void resetPoseAndEncoders(void) {
  noInterrupts();
  encoderLeftTicks = 0;
  encoderRightTicks = 0;
  interrupts();
  previousLeftTicks = 0;
  previousRightTicks = 0;
  poseXcm = 0.0f;
  poseYcm = 0.0f;
  headingDeg = 0.0f;
  encoderHeadingDeg = 0.0f;
  totalTravelCm = 0.0f;
  lastOdometryUs = micros();
}

/******** function updateOdometry
 * Purpose
 *   Updates pose and heading from encoders and MPU-6050 gyro rate.
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
void updateOdometry(void) {
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastOdometryUs;
  if (elapsedUs < ODOMETRY_INTERVAL_US) {
    return;
  }
  lastOdometryUs = nowUs;

  long leftTicks;
  long rightTicks;
  noInterrupts();
  leftTicks = encoderLeftTicks;
  rightTicks = encoderRightTicks;
  interrupts();

  long deltaLeftTicks = leftTicks - previousLeftTicks;
  long deltaRightTicks = rightTicks - previousRightTicks;
  previousLeftTicks = leftTicks;
  previousRightTicks = rightTicks;

  float leftDistance = deltaLeftTicks * DISTANCE_PER_PULSE_CM;
  float rightDistance = deltaRightTicks * DISTANCE_PER_PULSE_CM;
  float centerDistance = (leftDistance + rightDistance) * 0.5f;
  float encoderDeltaHeading =
    ((rightDistance - leftDistance) / WHEEL_TRACK_CM) * 180.0f / PI;
  encoderHeadingDeg = wrapAngle180(
    encoderHeadingDeg + encoderDeltaHeading
  );

  float gyroRate = 0.0f;
  bool gyroReadOk = mpuHealthy && gyro.readZDegPerSec(gyroRate);
  if (gyroReadOk) {
    float elapsedSec = elapsedUs / 1000000.0f;
    float gyroHeading = wrapAngle180(
      headingDeg + GYRO_Z_SIGN * gyroRate * elapsedSec
    );
    headingDeg = wrapAngle180(
      0.96f * gyroHeading + 0.04f * encoderHeadingDeg
    );
  } else {
    headingDeg = encoderHeadingDeg;
  }

  float headingRad = headingDeg * PI / 180.0f;
  poseXcm += centerDistance * cos(headingRad);
  poseYcm += centerDistance * sin(headingRad);
  totalTravelCm += fabs(centerDistance);
}

/******** function runHeadingTurn
 * Purpose
 *   Commands an in-place turn until a target heading is reached.
 * Arguments
 *   pwm Unsigned motor PWM magnitude.
 *   targetHeading Target heading in degrees.
 * Results
 *   Returns true when the reported condition or operation succeeds.
 * Hardware
 *   Uses the shared Arduino UNO hardware layer as required.
 * Software
 *   Uses fixed state and bounded operations; no dynamic allocation.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
bool runHeadingTurn(int pwm, float targetHeading) {
  float error = angleDifference(targetHeading, headingDeg);
  if (fabs(error) <= TURN_TOLERANCE_DEG) {
    motors.stop();
    return true;
  }
  if (error > 0.0f) {
    motors.setMotors(-pwm, pwm);
  } else {
    motors.setMotors(pwm, -pwm);
  }
  return false;
}

#endif
