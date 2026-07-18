/********** TRITON-X occupancy grid
 * Purpose
 *   Maintains a 30 x 30 browser map from Arduino pose and sonar rays.
 * Hardware
 *   None directly.
 * Software
 *   Uses bounded log-odds-like confidence and half-cell ray sampling.
 * Limitations
 *   The map is fixed at 3 x 3 m with 10 cm cells and is not full SLAM.
 * Reference
 *   v4.0 Final, based on MAPPING v1.4.
 **********/

#ifndef TRITON_X_MAPPING_GRID_H
#define TRITON_X_MAPPING_GRID_H

#include <Arduino.h>
#include <math.h>
#include <string.h>

int8_t occupancy[MAP_CELL_COUNT];
uint8_t pathCells[MAP_CELL_COUNT];

/******** function mapIndex
 * Purpose
 *   Converts grid coordinates to a linear array index.
 * Arguments
 *   x Horizontal control or coordinate value.
 *   y Vertical control or coordinate value.
 * Results
 *   Returns the calculated signed integer value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
int mapIndex(int x, int y) {
  return y * MAP_SIZE + x;
}

/******** function inMap
 * Purpose
 *   Checks whether grid coordinates are inside the occupancy map.
 * Arguments
 *   x Horizontal control or coordinate value.
 *   y Vertical control or coordinate value.
 * Results
 *   Returns true when the reported condition or operation succeeds.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
bool inMap(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

/******** function worldToCell
 * Purpose
 *   Converts world coordinates in centimetres to map cells.
 * Arguments
 *   xCm Input parameter defined by the function signature.
 *   yCm Input parameter defined by the function signature.
 *   cellX Reference receiving the map X cell.
 *   cellY Reference receiving the map Y cell.
 * Results
 *   Returns true when the reported condition or operation succeeds.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
bool worldToCell(float xCm, float yCm, int& cellX, int& cellY) {
  cellX = MAP_SIZE / 2 + (int)roundf(xCm / CELL_SIZE_CM);
  cellY = MAP_SIZE / 2 - (int)roundf(yCm / CELL_SIZE_CM);
  return inMap(cellX, cellY);
}

/******** function clearMap
 * Purpose
 *   Clears occupancy confidence and robot path history.
 * Arguments
 *   None.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void clearMap(void) {
  memset(occupancy, 0, sizeof(occupancy));
  memset(pathCells, 0, sizeof(pathCells));
  int centerX;
  int centerY;
  if (worldToCell(0.0f, 0.0f, centerX, centerY)) {
    pathCells[mapIndex(centerX, centerY)] = 1;
  }
}

/******** function updateFreeCell
 * Purpose
 *   Decreases occupancy confidence for an observed free cell.
 * Arguments
 *   x Horizontal control or coordinate value.
 *   y Vertical control or coordinate value.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void updateFreeCell(int x, int y) {
  if (!inMap(x, y)) {
    return;
  }
  int index = mapIndex(x, y);
  occupancy[index] = constrain(occupancy[index] - 1, -10, 10);
}

/******** function updateOccupiedCell
 * Purpose
 *   Increases occupancy confidence for an observed obstacle cell.
 * Arguments
 *   x Horizontal control or coordinate value.
 *   y Vertical control or coordinate value.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void updateOccupiedCell(int x, int y) {
  if (!inMap(x, y)) {
    return;
  }
  int index = mapIndex(x, y);
  occupancy[index] = constrain(occupancy[index] + 4, -10, 10);
}

/******** function markPath
 * Purpose
 *   Marks the current robot cell as traversed and free.
 * Arguments
 *   xCm Input parameter defined by the function signature.
 *   yCm Input parameter defined by the function signature.
 * Results
 *   Updates state and/or hardware and returns no value.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
void markPath(float xCm, float yCm) {
  int x;
  int y;
  if (worldToCell(xCm, yCm, x, y)) {
    pathCells[mapIndex(x, y)] = 1;
    updateFreeCell(x, y);
  }
}

void integrateRay(float originX, float originY, float bearingDeg,
                  float distanceCm, bool hit) {
  float limitedDistance = constrain(
    distanceCm, 0.0f, MAP_RAY_MAX_CM
  );
  float bearingRad = bearingDeg * DEG_TO_RAD;
  float freeDistance = hit ?
    max(0.0f, limitedDistance - CELL_SIZE_CM * 0.55f) :
    limitedDistance;

  for (float distance = 0.0f;
       distance <= freeDistance;
       distance += CELL_SIZE_CM * 0.5f) {
    float worldX = originX + cosf(bearingRad) * distance;
    float worldY = originY + sinf(bearingRad) * distance;
    int cellX;
    int cellY;
    if (worldToCell(worldX, worldY, cellX, cellY)) {
      updateFreeCell(cellX, cellY);
    }
  }

  if (hit) {
    float worldX = originX + cosf(bearingRad) * limitedDistance;
    float worldY = originY + sinf(bearingRad) * limitedDistance;
    int cellX;
    int cellY;
    if (worldToCell(worldX, worldY, cellX, cellY)) {
      updateOccupiedCell(cellX, cellY);
    }
  }
}

/******** function buildMapMessage
 * Purpose
 *   Serializes the occupancy grid for browser transmission.
 * Arguments
 *   None.
 * Results
 *   Returns the generated text message.
 * Hardware
 *   Uses ESP32-CAM communication or camera resources as required.
 * Software
 *   Uses bounded state and the ESP32 framework communication APIs.
 * Reference
 *   v4.0 Final, C. Vasileiou, July 2026.
 **********/
String buildMapMessage(void) {
  String payload;
  payload.reserve(MAP_CELL_COUNT + 5);
  payload = "MAP:";
  for (int i = 0; i < MAP_CELL_COUNT; i++) {
    char state = '0';
    if (occupancy[i] >= 2) {
      state = '2';
    } else if (pathCells[i]) {
      state = '3';
    } else if (occupancy[i] <= -2) {
      state = '1';
    }
    payload += state;
  }
  return payload;
}

#endif
