# Αναφορά Πηγών Τρισδιάστατων Μοντέλων (CAD Attribution)

Το assembly του TRITON-X ενσωματώνει έτοιμα 3D μοντέλα εξαρτημάτων (Arduino, αισθητήρες, driver, κλπ.) τα οποία **δεν σχεδιάστηκαν από τον συγγραφέα** αλλά ελήφθησαν από δημόσιες βιβλιοθήκες CAD. Σύμφωνα με ακαδημαϊκή δεοντολογία και τους όρους χρήσης των περισσότερων πλατφορμών (GrabCAD, Thingiverse κ.ά.), κάθε τέτοιο μοντέλο αναφέρεται ρητά παρακάτω, με τον αρχικό δημιουργό και την πηγή.

> Τα μοντέλα αυτά χρησιμοποιούνται **αποκλειστικά για ακαδημαϊκή/εκπαιδευτική οπτικοποίηση** της διάταξης εντός του assembly `triton_x_assembly.SLDASM`, και δεν αποτελούν πρωτότυπη εργασία του συγγραφέα. Η γεωμετρία του σασί, των στηριγμάτων αισθητήρων (mounts) και κάθε custom εξαρτήματος σχεδιάστηκε από τον συγγραφέα και εξαιρείται από τον παρακάτω πίνακα.

## Πίνακας Πηγών

| # | Εξάρτημα (στο assembly) | Δημιουργός / Publisher | Πηγή (link) | Άδεια χρήσης |
|---|---|---|---|---|
| 1 | Σασί + κινητήρες + τροχοί (chassis+motors+wheels) | Mukarram Ibrahim | [grabcad.com/library/smart-car-kit-mechanical-design-for-educational-robotics-platform-1](https://grabcad.com/library/smart-car-kit-mechanical-design-for-educational-robotics-platform-1) | GrabCAD Open Engineering |
| 2 | Arduino UNO R3 | Andrew Whitham | [grabcad.com/library/arduino-uno-r3-1](https://grabcad.com/library/arduino-uno-r3-1) | GrabCAD Open Engineering |
| 3 | Arduino Sensor Shield v5.0 | Tijani JOUINI | [grabcad.com/library/sensor-shield-v5-0-1](https://grabcad.com/library/sensor-shield-v5-0-1) | GrabCAD Open Engineering |
| 4 | ESP32-CAM | Dũng Phan | [grabcad.com/library/esp32-cam-wifi-bluetooth-1](https://grabcad.com/library/esp32-cam-wifi-bluetooth-1) | GrabCAD Open Engineering |
| 5 | PCF8574 I2C Expander | roberto valeri | [grabcad.com/library/pcf8574-multiplexer-1](https://grabcad.com/library/pcf8574-multiplexer-1) | GrabCAD Open Engineering |
| 6 | L298N Motor Driver | NITHISH CADEX | [grabcad.com/library/motor-driver-l298n-2](https://grabcad.com/library/motor-driver-l298n-2) | GrabCAD Open Engineering |
| 7 | Photoelectric Encoder | Gauthier Heiss | [grabcad.com/library/feetech-speed-sensor-ft-en-001-1](https://grabcad.com/library/feetech-speed-sensor-ft-en-001-1) | GrabCAD Open Engineering |
| 8 | HC-SR04 Ultrasonic | Henrique Fernandes | [grabcad.com/library/sensor-ultrassonico-hc-sr04-1](https://grabcad.com/library/sensor-ultrassonico-hc-sr04-1) | GrabCAD Open Engineering |
| 9 | SG90 Micro Servo | Matheus Frasson | [grabcad.com/library/sg90-micro-servo-9g-tower-pro-1](https://grabcad.com/library/sg90-micro-servo-9g-tower-pro-1) | GrabCAD Open Engineering |
| 10 | IR Sensor (Obstacle) | MRMS - WORKSHOP | [grabcad.com/library/ir-sensor-7](https://grabcad.com/library/ir-sensor-7) | GrabCAD Open Engineering |
| 11 | IR Sensor (Cliff, HW-201) | Gaborcz | [grabcad.com/library/hw-201-2](https://grabcad.com/library/hw-201-2) | GrabCAD Open Engineering |
| 12 | MPU-6050 Accelerometer/Gyro Module | Nelson Stoldt | [grabcad.com/library/mpu6050-accelerometer-module-1](https://grabcad.com/library/mpu6050-accelerometer-module-1) | GrabCAD Open Engineering |
| 13 | LCD 1602 I2C (16x2 Character Display) | Anders Lohmann | [grabcad.com/library/16x2-character-lcd-display-1](https://grabcad.com/library/16x2-character-lcd-display-1) | GrabCAD Open Engineering |
| 14 | Battery Holder (6× AA) | Hallo | [grabcad.com/library/battery-holder-5xaa-1](https://grabcad.com/library/battery-holder-5xaa-1) | GrabCAD Open Engineering |
| 15 | Rocker Switch (KCD1, 2-Pin) | PENNEL Patrice | [grabcad.com/library/kcd1-on-off-rocker-switch-2-pins-1](https://grabcad.com/library/kcd1-on-off-rocker-switch-2-pins-1) | GrabCAD Open Engineering |
| 16 | DC-DC Step-Down Converter | MainDesign | [grabcad.com/library/usb-dc-dc-step-down-1](https://grabcad.com/library/usb-dc-dc-step-down-1) | GrabCAD Open Engineering |

> ⚠️ **Σημείωση για τη στήλη "Άδεια χρήσης":** Όλα τα παραπάνω μοντέλα προέρχονται από το GrabCAD, όπου η προεπιλεγμένη άδεια δημοσίευσης είναι η **"GrabCAD Open Engineering"**, η οποία επιτρέπει ελεύθερη χρήση/τροποποίηση με υποχρεωτική αναφορά του δημιουργού. Πριν την τελική παράδοση, **επιβεβαίωσε ξεχωριστά σε κάθε σελίδα του GrabCAD** ότι δεν αναγράφεται διαφορετική/αυστηρότερη άδεια (σπάνιο αλλά πιθανό), ειδικά για τα part #1 (πλήρες σασί kit) και #16 (DC-DC converter), όπου μερικοί δημιουργοί περιορίζουν σε "Non-Commercial only".
