/********** TRITON-X shared types
 * Purpose
 *   Declares the operating modes, safety states and sensor snapshot
 *   used by the integrated Arduino UNO firmware.
 * Hardware
 *   None directly.
 * Software
 *   Strongly typed enumerations reduce ambiguous state transitions.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/

#ifndef TRITON_X_ROBOT_TYPES_H
#define TRITON_X_ROBOT_TYPES_H

#include <Arduino.h>

enum RobotMode : uint8_t {
  modeIdle,
  modeManual,
  modeLevel,
  modeLine,
  modeObstacle,
  modeMapping
};

enum SafetyState : uint8_t {
  safetyIdle,
  safetyStopping,
  safetyLinearEscape,
  safetyTurning,
  safetyPostHold,
  safetyLocked
};

enum MappingState : uint8_t {
  mapIdle,
  mapCruise,
  mapScanLeft,
  mapScanRight,
  mapScanCenter,
  mapDecide,
  mapLinearMove,
  mapTurning,
  mapFault
};

enum ObstacleState : uint8_t {
  obstacleCruising,
  obstacleDetected,
  obstacleScanningLeft,
  obstacleScanningRight,
  obstacleResettingHead,
  obstacleDeciding,
  obstacleTurning,
  obstacleEscaping
};

enum SensorIndex : uint8_t {
  sensorObstacleFrontLeft,
  sensorObstacleFrontRight,
  sensorObstacleRearLeft,
  sensorObstacleRearRight,
  sensorCliffFrontLeft,
  sensorCliffFrontRight,
  sensorCliffRearLeft,
  sensorCliffRearRight,
  sensorCount
};

struct SensorSnapshot {
  bool obstacleFrontLeft;
  bool obstacleFrontRight;
  bool obstacleRearLeft;
  bool obstacleRearRight;
  bool cliffFrontLeft;
  bool cliffFrontRight;
  bool cliffRearLeft;
  bool cliffRearRight;
  bool pcfHealthy;
};

#endif
