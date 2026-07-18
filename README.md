<div align="center">

# 🤖 TRITON-X
### Multi-Mode Autonomous Unmanned Ground Vehicle (UGV)

**Μελέτη και Ανάπτυξη Μη Επανδρωμένου Συστήματος — Εργασία 2**
MSc Πρόγραμμα ΠΑΔΑ (PADA) · Πανεπιστήμιο Δυτικής Αττικής

[![Platform](https://img.shields.io/badge/platform-Arduino%20UNO%20R3-00979D?logo=arduino&logoColor=white)](TRITON_X_UNO)
[![Co-processor](https://img.shields.io/badge/co--processor-ESP32--CAM-E7352C?logo=espressif&logoColor=white)](TRITON_X_ESP32)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-active%20development-yellow)]()

*Ένα ρομποτικό όχημα εδάφους έξι λειτουργικών καταστάσεων, με τηλεχειρισμό μέσω Wi-Fi, ζωντανή μετάδοση βίντεο, αυτόνομη αποφυγή εμποδίων και χαρτογράφηση κάλυψης — βασισμένο στο μηχατρονικό μοντέλο Controller–Plant.*

[Αρχιτεκτονική](#-αρχιτεκτονική-συστήματος) ·
[Λειτουργίες](#-6-λειτουργικές-καταστάσεις-fsm) ·
[Firmware](#-λογισμικό--firmware) ·
[Τεκμηρίωση](#-τεκμηρίωση--docs) ·
[Εγκατάσταση](#-εγκατάσταση--γρήγορη-εκκίνηση)

</div>

---

## 📖 Περιγραφή Έργου

Το **TRITON-X** είναι η εξέλιξη ενός απλούστερου ρομπότ αποφυγής εμποδίων 4 αισθητήρων (Εργασία 1) σε ένα πλήρες, πολυλειτουργικό αυτόνομο όχημα εδάφους (UGV). Ο πυρήνας ελέγχου βασίζεται σε **Arduino UNO R3**, ενισχυμένος με έναν **ESP32-CAM** ως ασύρματο συνεπεξεργαστή (co-processor) για video streaming και web-based τηλεχειρισμό.

Το σύστημα ακολουθεί το **βασικό μηχατρονικό μοντέλο** (Controller–Plant–Feedback) της προδιαγραφής εργασίας: αισθητήρια (S) → μικροελεγκτής (μC) → ενεργοποιητές (A), με πλήρη ανάδραση (sensor fusion, Kalman filtering, closed-loop control).

Το ρομπότ διαθέτει **6 ανεξάρτητες λειτουργικές καταστάσεις** (Finite State Machine) — από απλό χειροκίνητο τηλεχειρισμό μέσω κινητού, μέχρι πλήρως αυτόνομη χαρτογράφηση χώρου με occupancy grid.

### Γιατί TRITON-X;

| Χαρακτηριστικό | Περιγραφή |
|---|---|
| 🎮 **Dual-mode τηλεχειρισμός** | Virtual joystick *και* έλεγχος με κλίση κινητού (gyroscope) μέσω browser |
| 📷 **Live FPV streaming** | ESP32-CAM MJPEG stream ενσωματωμένο στο web dashboard |
| 🧭 **Sensor fusion** | Kalman filtering στο υπερηχητικό, IMU 6-άξονων, 8× IR αισθητήρες, οδομετρία encoder |
| 🗺️ **Αυτόνομη χαρτογράφηση** | Boustrophedon coverage algorithm + occupancy grid 20×20, live μετάδοση στον browser |
| ⚙️ **OOP αρχιτεκτονική** | Καθαρές C++ κλάσεις (`KalmanFilter`, `Sonar`, `MotorDriver`, `OccupancyGrid`) |
| ⏱️ **Non-blocking σχεδίαση** | Αρχιτεκτονική `millis()` σε όλο το firmware — καμία μπλοκαριστική `delay()` |

---

## 🏗️ Αρχιτεκτονική Συστήματος

```
┌─────────────────────────────────────────────────────────────┐
│                      ARDUINO UNO R3                          │
│                 (Κύριος Ελεγκτής — μC)                       │
│                                                                │
│  ┌─────────── SENSORS (S) ───────────┐  ┌── ACTUATORS (A) ─┐ │
│  │ • HC-SR04 Ultrasonic (on SG90)     │  │ • L298N →        │ │
│  │ • 8× IR (4 obstacle + 4 cliff)     │  │   2× DC Motors    │ │
│  │ • MPU-6050 IMU (I2C)               │  │ • SG90 Servo      │ │
│  │ • 2× Photoelectric Encoders        │  │ • LCD 1602 (HMI)  │ │
│  │ • PCF8574T I2C Expander            │  └───────────────────┘ │
│  └─────────────────────────────────────┘                      │
│                                                                │
│         Λογική Ελέγχου: 6-mode FSM · non-blocking             │
│         millis() · Kalman Filter · OOP (C++ classes)          │
└───────────────────────────┬────────────────────────────────────┘
                             │ UART (SoftwareSerial, polling encoders)
                             │ MODE: / JOY: / GYR: / TEL: / MAP:
┌───────────────────────────┴────────────────────────────────────┐
│                    ESP32-CAM (AI-Thinker)                       │
│               (Ασύρματος Συνεπεξεργαστής)                       │
│                                                                  │
│  • Wi-Fi Access Point  ("TRITON-X")                              │
│  • HTTP Server (port 80)  — Web Dashboard                        │
│  • WebSocket Server (port 81) — Joystick / Gyro / Telemetry / Map │
│  • MJPEG Camera Stream (port 82) — OV2640                        │
└───────────────────────────┬──────────────────────────────────────┘
                             │ Wi-Fi 802.11
┌───────────────────────────┴──────────────────────────────────────┐
│                     📱 Browser (Mobile / Desktop)                  │
│  • Live FPV video feed        • Virtual joystick (touch)           │
│  • Gyroscope tilt control     • Live occupancy grid canvas         │
│  • Telemetry bar (distance, battery, state)  • Mode selector       │
└─────────────────────────────────────────────────────────────────┘
```

Πλήρες πρωτόκολλο επικοινωνίας UART μεταξύ των δύο μονάδων: [`docs/UART_PROTOCOL.txt`](docs/UART_PROTOCOL.txt)

---

## ⚡ Ισχύς — Power Architecture

Δύο **ανεξάρτητες** ρελές 5V, με κοινή γείωση, ώστε το ESP32-CAM (με τις απότομες αιχμές ρεύματος του Wi-Fi) να μην προκαλεί brown-out στο Arduino:

```
6× AA (9V) battery pack
        │
        ├──► Rocker Switch ──► L298N 12V_IN
        │                          │
        │                    Onboard 5V regulator
        │                          │
        │                          ▼
        │                  +5V_MCU rail ──► Arduino, sensors, servo
        │
        └──► DC-DC Step-Down (6.5–40V → 5V, 1.5A)
                                   │
                                   ▼
                          +5V_CAM rail ──► ESP32-CAM (exclusively)
```

> ⚠️ **Κρίσιμη σημείωση:** Η τροφοδοσία είναι πάντα **6×AA (9V)**, ποτέ Li-Po. Ο εσωτερικός ρυθμιστής του L298N **δεν επαρκεί** για τις αιχμές ρεύματος του ESP32-CAM στο Wi-Fi — απαιτείται ξεχωριστός DC-DC step-down μετατροπέας.

---

## 🎛️ 6 Λειτουργικές Καταστάσεις (FSM)

| # | Mode | Περιγραφή | Βασικά στοιχεία |
|---|------|-----------|------------------|
| 0 | **IDLE** | Κατάσταση αναμονής | Κινητήρες σταματημένοι, LCD εμφανίζει επιλογή mode |
| 1 | **MANUAL** | Τηλεχειρισμός με virtual joystick από browser | Touch UI → ESP32-CAM → UART → PWM differential drive |
| 2 | **LEVEL** | Τηλεχειρισμός με κλίση κινητού (tilt-to-drive) | DeviceOrientation API, dead-zone & smoothing filter |
| 3 | **LINE** | Ακολούθηση γραμμής | 2× IR κάτω, PD controller, ταυτόχρονη αποφυγή εμποδίων με HC-SR04 |
| 4 | **OBSTACLE** | Αυτόνομη αποφυγή εμποδίων | FSM `CRUISING → SCAN → ESCAPE → CLIFF_ESCAPE`, Kalman filter, hardware interrupts |
| 5 | **MAP** | Αυτόνομη χαρτογράφηση κάλυψης | Boustrophedon algorithm, dead-reckoning odometry, occupancy grid 20×20 |

---

## 🔧 Εξοπλισμός — Hardware

### Από Εργασία 1 (υπάρχοντα)

Arduino UNO R3 + Sensor Shield v5.0 · L298N Dual H-Bridge · 2× DC Gear Motor + caster wheel · Ακρυλικό σασί 2WD · HC-SR04 + SG90 Servo · 4× IR sensors (μπροστά) · LCD 1602 I2C

### Νέα προμήθεια για Εργασία 2

ESP32-CAM (AI-Thinker, OV2640) · DC-DC Step-Down 6.5–40V→5V 1.5A · 2× Photoelectric Encoder (18 PPR εμπειρικά) · MPU-6050 GY-521 IMU · 4× IR sensors (πίσω) · PCF8574T I2C Expander

Πλήρης χάρτης ακίδων (pin map): αντιστοιχεί στον **Πίνακα 2.4** της αναφοράς — δες το κύριο έγγραφο αναφοράς στο [`docs/`](docs/).

---

## 💻 Λογισμικό — Firmware

```
TRITON_X_UNO/              # Arduino UNO — κύριο firmware
└── TRITON_X.ino            # 6-mode FSM, OOP classes, non-blocking control loop

TRITON_X_ESP32/             # ESP32-CAM — ασύρματος συνεπεξεργαστής
└── ESP32_CAM.ino            # Wi-Fi AP, HTTP/WebSocket/MJPEG server, web dashboard
```

### Βασικές OOP κλάσεις (Arduino)

| Κλάση | Ευθύνη |
|---|---|
| `KalmanFilter` | Φιλτράρισμα θορύβου μετρήσεων HC-SR04· reset ανά γωνία σάρωσης |
| `Sonar` | Έλεγχος HC-SR04 + servo σάρωσης, non-blocking |
| `MotorDriver` | Αφαίρεση (abstraction) του L298N — differential drive, PWM |
| `OccupancyGrid` | Δομή 20×20 grid, ενημέρωση mode MAP, σειριοποίηση για μετάδοση |

### Πρωτόκολλο επικοινωνίας UART (Arduino ⇄ ESP32-CAM)

| Πρόθεμα | Κατεύθυνση | Περιεχόμενο |
|---|---|---|
| `MODE:` | ESP32 → Arduino | Αλλαγή λειτουργικής κατάστασης |
| `JOY:` | ESP32 → Arduino | Συντεταγμένες joystick (x, y) |
| `GYR:` | ESP32 → Arduino | Δεδομένα κλίσης κινητού |
| `TEL:` | Arduino → ESP32 | Τηλεμετρία (απόσταση, mode, μπαταρία) |
| `MAP:` | Arduino → ESP32 | Ενημέρωση occupancy grid κελιού |

Αναλυτικά: [`docs/UART_PROTOCOL.txt`](docs/UART_PROTOCOL.txt)

> 📌 **Γνωστός περιορισμός:** Η βιβλιοθήκη `SoftwareSerial` δεσμεύει όλα τα PCINT vectors, καθιστώντας αδύνατη τη χρήση interrupt-driven ανάγνωσης για τους encoders όταν είναι ενεργή η UART επικοινωνία με το ESP32-CAM. Οι encoders διαβάζονται επομένως με **polling**.

---

## 📚 Τεκμηρίωση — `docs/`

| Αρχείο | Περιεχόμενο |
|---|---|
| [`README_FIRST.txt`](docs/README_FIRST.txt) | Οδηγός εκκίνησης — τι να διαβάσεις πρώτο |
| [`UART_PROTOCOL.txt`](docs/UART_PROTOCOL.txt) | Πλήρης προδιαγραφή πρωτοκόλλου επικοινωνίας Arduino ⇄ ESP32-CAM |
| [`VALIDATION_REPORT.txt`](docs/VALIDATION_REPORT.txt) | Αποτελέσματα δοκιμών ανά λειτουργική κατάσταση |
| [`CHANGELOG.txt`](docs/CHANGELOG.txt) | Ιστορικό εκδόσεων firmware |
| [`SHA256_MANIFEST.txt`](docs/SHA256_MANIFEST.txt) | Checksums αρχείων για επαλήθευση ακεραιότητας |

---

## 🚀 Εγκατάσταση & Γρήγορη Εκκίνηση

### Προαπαιτούμενα

- [Arduino IDE](https://www.arduino.cc/en/software) 2.x
- Board package: **ESP32 by Espressif Systems** (Boards Manager)
- Βιβλιοθήκες: `Servo`, `Wire`, `LiquidCrystal_I2C`, `SoftwareSerial`, `WebSocketsServer` (Links2004), `ArduinoJson`, `esp32-camera`

### Βήματα

1. **Clone το repository**
   ```bash
   git clone https://github.com/BabisVasileiou/TRITON-X.git
   cd TRITON-X
   ```

2. **Arduino UNO** — άνοιξε `TRITON_X_UNO/TRITON_X.ino`, επίλεξε board *Arduino Uno*, upload.

3. **ESP32-CAM** — άνοιξε `TRITON_X_ESP32/ESP32_CAM.ino`, board *AI Thinker ESP32-CAM*.
   ⚠️ Χρειάζεται εξωτερικό USB-TTL adapter (GPIO0 → GND κατά το flashing). Μετά το upload, αφαίρεσε τη γέφυρα και κάνε reset.

4. **Σύνδεση** — ενεργοποίησε το ρομπότ, σύνδεσε το κινητό στο Wi-Fi **`TRITON-X`**, άνοιξε browser στο `http://192.168.4.1`, επίλεξε mode.

---

## 👤 Συγγραφέας

**Βασιλείου Χαράλαμπος**
MSc DRONES — Πανεπιστήμιο Δυτικής Αττικής
Μάθημα: Μελέτη και Ανάπτυξη Μη Επανδρωμένου Συστήματος

## 📜 Άδεια Χρήσης

Το firmware διανέμεται υπό την άδεια [MIT](LICENSE).

---

<div align="center">

*Αναπτύχθηκε ως τμήμα του μαθήματος "Μελέτη και Ανάπτυξη Μη Επανδρωμένου Συστήματος" — ΠΑΔΑ, 2026*

</div>
