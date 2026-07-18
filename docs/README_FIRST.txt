================================================================
TRITON-X  --  Multi-Mode Autonomous UGV
Integrated Firmware v4.0 Final
C. Vasileiou, MSc PADA, University of West Attica, July 2026
================================================================

1. PURPOSE
----------------------------------------------------------------
TRITON-X is a two-wheel differential-drive unmanned ground
vehicle with six operating states supervised by a global cliff
safety layer:

  IDLE      Vehicle armed but stationary (safe default).
  MANUAL    Wi-Fi virtual joystick from the browser dashboard.
  LEVEL     Phone tilt (DeviceOrientation) drive control.
  LINE      Two-sensor PD line follower with sonar terminal stop.
  OBSTACLE  Autonomous Guardian avoidance FSM with Kalman sonar.
  MAPPING   Dead-reckoning exploration with live occupancy grid.

The system uses two cooperating controllers:

  Arduino UNO R3   Motors, sensors, odometry, safety, mode FSMs.
  ESP32-CAM        Wi-Fi AP, browser dashboard, camera stream,
                   occupancy grid storage, UART bridge.

2. PACKAGE CONTENTS
----------------------------------------------------------------
  TRITON_X_UNO/      Arduino UNO sketch (open TRITON_X_UNO.ino).
  TRITON_X_ESP32/    ESP32-CAM sketch (open TRITON_X_ESP32.ino).
  docs/              This file, protocol, validation, changelog,
                     SHA-256 manifest.

3. REQUIRED LIBRARIES
----------------------------------------------------------------
Arduino UNO (board: Arduino UNO):
  - LiquidCrystal_I2C
  - Servo (bundled with the AVR core)
  Note: SoftwareSerial is intentionally NOT used. UART to the
  ESP32 is implemented by TimerUart.h (Timer2, 9600 8N1) so that
  the D10/D12 encoder pin-change interrupt remains available.

ESP32-CAM (board: AI Thinker ESP32-CAM, ESP32 Arduino core):
  - WebSockets (Markus Sattler / links2004)
  - WiFi, WebServer, esp_camera, esp_http_server (bundled).

4. UART WIRING
----------------------------------------------------------------
  ESP32 GPIO13 (TX) ------------------> Arduino D4 (RX)
  Arduino D13 (TX) --[5V->3.3V divider]-> ESP32 GPIO14 (RX)
  Common GND between the two 5 V rails is mandatory.
  Baud rate: 9600, format 8N1.

  GPIO15 must NOT be used as ESP32 TX: it is a strapping pin and
  corrupts boot. GPIO13 is the validated choice.

5. POWER
----------------------------------------------------------------
  6xAA (9 V) -> switch -> L298N 12V IN (motors)
                       -> L298N 5V regulator -> Sensor Shield VCC
                          (Arduino, sensors, servo, LCD, PCF8574T)
  Same 9 V rail -> DC-DC step-down (set to 5.0 V) -> ESP32-CAM
  A 470 uF/16 V capacitor decouples the ESP32-CAM supply.
  The L298N onboard regulator cannot supply ESP32 Wi-Fi current
  peaks; the dedicated step-down converter is mandatory.

  L298N jumpers: the 5V-EN regulator jumper stays IN PLACE; the
  ENA/ENB jumpers stay REMOVED (PWM from D5/D6).

6. FIRST START SEQUENCE
----------------------------------------------------------------
  1. Upload TRITON_X_UNO.ino to the Arduino UNO.
  2. Upload TRITON_X_ESP32.ino to the ESP32-CAM (GPIO0 to GND
     for flashing, then release and reset).
  3. Power the robot. Keep it STILL during MPU calibration
     (LCD: "MPU CALIBRATE / Keep robot still").
  4. On a phone, join Wi-Fi "TRITON-X", password "triton2026".
  5. Browse to http://192.168.4.1
  6. Select a mode, then press START. Selection alone never
     starts motion.

7. SAFETY MODEL (SUMMARY)
----------------------------------------------------------------
  - START/STOP semantics: no mode moves without explicit START.
  - Mode change, STOP, browser disconnect, browser heartbeat
    loss and UNO link loss all stop the motors immediately.
  - Global cliff supervisor overrides MANUAL, LEVEL, OBSTACLE
    and MAPPING (LINE re-purposes D2/D3 as line sensors).
  - A failed cliff recovery latches CLIFF LOCK. Clearing needs:
    safe level floor, sensors clear for 400 ms, CLEAR SAFETY
    LOCK, then a new START.
  - MANUAL and LEVEL require neutral re-arming after any stop.

8. CAMERA
----------------------------------------------------------------
  OV2640, QVGA 320x240 MJPEG, about 10 fps, on port 82. The
  camera initializes at boot but streams only after CAMERA ON.
  It may run concurrently with any mode, including MAPPING.

9. TEST SEQUENCE (RECOMMENDED)
----------------------------------------------------------------
  1. IDLE telemetry visible on the dashboard (UNO online).
  2. MANUAL: arm with centered joystick, drive, release, verify
     500 ms watchdog stop.
  3. LEVEL: calibrate neutral, tilt drive, verify watchdog.
  4. LINE: follow the track, place an obstacle, verify the
     15 cm terminal stop.
  5. OBSTACLE: verify cruise-scan-decide cycle and anti-loop
     U-turn after three consecutive turns.
  6. MAPPING: verify pose trace, occupancy grid growth and
    automatic map reset when the mode is reselected.
  7. Cliff: approach a table edge in MANUAL, verify recovery,
     then verify CLIFF LOCK behavior with a blocked recovery.

10. MEMORY FOOTPRINT (AS COMPILED)
----------------------------------------------------------------
  Arduino UNO : Flash 25,650 bytes (79%), SRAM 767 bytes (37%),
                free SRAM 1,281 bytes.
  ESP32-CAM   : Flash 1,139,368 bytes (36%), RAM 62,516 bytes
                (19%), free RAM 265,164 bytes.
================================================================
