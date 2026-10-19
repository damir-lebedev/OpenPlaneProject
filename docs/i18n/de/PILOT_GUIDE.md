# Pilotenleitfaden für OpenPlaneProject

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../PILOT_GUIDE.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

Das ist ein praktischer Leitfaden dazu, „was wohin angeschlossen wird und wie man fliegt“, für alle, die einen Lötkolben und einen Sender in der Hand halten und nicht den Quellcode lesen. Wer die Architektur des Codes verstehen will, findet sie in den anderen Dokumenten des Repositorys. Hier geht es nur um Hardware, Kanäle, Firmware und Flüge.

Repository: https://github.com/damir-lebedev/OpenPlaneProject, Branch `main`.

Gleich vorweg ehrlich: Das Projekt befindet sich in aktiver Entwicklung und ist **kein fertiges Produkt**. Der erste Prototyp ist schon geflogen, aber mit Vorbehalten, die weiter unten in einem eigenen Abschnitt beschrieben sind. Lesen Sie ihn vor dem Flug, nicht danach.

---

## Inhalt

1. [Welche Hardware benötigt wird](#welche-hardware-benötigt-wird)
2. [Boardauswahl und Pinbelegung](#boardauswahl-und-pinbelegung)
3. [Empfänger anschließen](#empfänger-anschließen)
4. [RC-Kanalbelegung](#rc-kanalbelegung)
5. [Autopilot-Kanäle](#autopilot-kanäle)
6. [ARM und Failsafe](#arm-und-failsafe)
7. [Board flashen](#board-flashen)
8. [Web-Dashboard im Feld](#web-dashboard-im-feld)
9. [Blackbox](#blackbox)
10. [Checkliste vor dem Flug und Sicherheit](#checkliste-vor-dem-flug-und-sicherheit)
11. [Fehlersuche](#fehlersuche)
12. [Aktueller Stand der Prototypzelle](#aktueller-stand-der-prototyp-flugzeugzelle)

---

## Welche Hardware benötigt wird

Der Bausatz des aktuellen Aufbaus (die Firmware wurde darauf geprüft):

- **Ein STM32H743-Board DevEBox H743** (MCUDEV) – das Hauptboard. Das frühere Hauptboard, das ESP32-S3 unten, wird ebenfalls unterstützt.
- **Ein ESP32-S3-N16R8-Board** (ein DevKitC-1-Klon mit zwei USB-C-Buchsen: „USB“ und „COM“).
- **Ein Sender FS-i6 + Empfänger FS-iA6B** (iBUS-Protokoll, 10 Kanäle). Benötigt wird eine einzige Datenleitung – der Anschluss iBUS SERVO. Der Sender muss Failsafe einstellen können – diese Einstellung ist Pflicht, siehe den Abschnitt zu Failsafe.
- **Oder ein anderer Sender mit SBUS-Empfänger** (FrSky, Futaba, RadioMaster, Jumper …) **oder CRSF-Empfänger** (ExpressLRS, TBS Crossfire) – das Protokoll wird in `Config.h` gewählt (`RX_PROTOCOL`), der Anschluss steht weiter unten.
- **2 Servos MG90S** für die Querruder – eines für jede Tragflächenhälfte (zwei unabhängige Servos, nicht eines für beide Flächen).
- **1 Servo MG90S** für das Höhenruder.
- **1 Servo MG90S** für das Seitenruder – auf dessen Achse sitzt auch das Lenkrad des Fahrwerks (Rollen am Boden). Das Fahrwerk: zwei Räder vorn und ein lenkbares Spornrad (`LANDING_GEAR = TAIL_WHEEL` in `Config.h`; ein Bugradfahrwerk ist `NOSE_WHEEL`).
- **Ein Fahrtregler (ESC) BLHeli 80A, bidirektional** (Modus Bidirectional/3D) mit 5-V-BEC (das BEC versorgt die Servos und den Empfänger). Ein gewöhnlicher Regler mit nur einer Drehrichtung geht auch – `ESC_BIDIRECTIONAL = false` in `Config.h`, siehe [Motorumkehr](#motorumkehr-swd).
- **Ein Motor D3548 1100KV** + **ein Propeller 10x5**.
- **Ein LiPo-Akku 3S**.

Sensoren des Autopiloten (alle über I2C; ohne sie fliegt das Flugzeug im manuellen Modus):

- **GY-521** – Gyroskop + Beschleunigungssensor (auf der Platine kann ein MPU6050 oder, wie bei uns, ein MPU6500 sitzen – beide werden unterstützt).
- **BMP581** – Barometer (der frühere BMP388 wird ebenfalls unterstützt).
- **GY-273** – Kompass (bei uns sitzt darauf ein QMC5883P; der QMC5883L wird ebenfalls unterstützt).
- Optional ein **OLED 128×64 SSD1306** (I2C) – ein Statusdisplay an Bord.

Zelle: Spannweite 1200 mm, Flügeltiefe 250 mm, Profil NACA 4412, Aufbau aus PETG (3D-Druck). Der erste Prototyp flog mit einem ESP32-C3, einem Motor D2212 1000KV und einem 40-A-Fahrtregler.

---

## Boardauswahl und Pinbelegung

Die Firmware unterstützt fünf Boards; umgeschaltet wird mit einem einzigen Build-Parameter (`pio run -e <Name der Umgebung>`). Jedes Board hat seine eigene Pinbelegung, die in der Firmware für die jeweilige Umgebung fest verdrahtet ist – stecken Sie Leitungen nicht eigenmächtig um, sondern halten Sie sich an die Tabelle für Ihr Board.

> **Wichtig:** Das Hauptboard ist jetzt das **STM32H743 (DevEBox H743)** – darauf sind Start, USB-Konsole, SD-Karte, iBUS, Servos und Motor geprüft; die Sensoren werden zum ersten Mal angeschlossen. Das **esp32-s3 (N16R8)** ist das frühere Hauptboard, seine Pinbelegung wurde auf dem Prüfstand mit allen Sensoren geprüft. Das **esp32-c3** ist der alte Prototyp, der geflogen ist. Die Pinbelegung des **esp32-dev** wurde anhand der Chip-Dokumentation gewählt und **nicht auf echter Hardware geprüft**.

### STM32H743 (DevEBox H743) – Hauptboard

`pio run -e stm32h743-devebox`, Board MCUDEV DevEBox H743 (STM32H743VIT6). Das ist das Standardboard (`default_envs = stm32h743-devebox`). Darauf sind Start, USB-Konsole, SD-Karte und Blackbox, iBUS-Empfang, ARM sowie Servos und Motor vom Sender bereits geprüft; die Sensoren werden zum ersten Mal angeschlossen.

| Verwendung | Pin |
|---|---|
| Querruder, linke Tragflächenhälfte | PA0 |
| Querruder, rechte Tragflächenhälfte | PA1 |
| Höhenruder | PA2 |
| ESC (Gas) | PA3 |
| Seitenruder + Lenkrad | PD14 |
| Empfänger (RX): iBUS, SBUS, CRSF | PE7 |
| I2C der Sensoren SDA / SCL (MPU, BMP581, Kompass) | PB11 / PB10 |
| I2C des OLED SDA / SCL (eigener Bus) | PB9 / PB8 |
| GPS: RX (← TX des GPS) / TX (→ RX des GPS) | PD9 / PD8 |
| MAVLink-Telemetrie (Funkmodem): RX / TX | PD0 / PD1 |
| AUX1 / AUX2 (Servos), Summer | PD15 / PE9, PE15 |
| SPI der Sensoren SCK / MISO / MOSI, CS IMU / CS Barometer | PB13 / PB14 / PB15, PB12 / PD10 |
| Akku / Stromsensor (ADC, zeichnet die Blackbox auf) | PC0 / PC1 |

Konsole, Log und Blackbox-Download laufen über die USB-C-Buchse des Boards (virtueller COM-Port). Nicht belegen: PA11/PA12 (USB), PA13/PA14 (SWD), PC8–PC12 und PD2 (µSD-Slot), PE3 und PC5 (Tasten K1/K2).

Anschluss der Sensoren auf dem Prüfstand (alle Module laufen mit **3,3 V**, nicht mit 5 V):

| Modul | Pins |
|---|---|
| MPU-6050 / GY-521 (auf der Platine kann ein MPU6500 sitzen – das ist normal) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, AD0–GND, INT/XDA/XCL – nicht anschließen. Separates MPU-6500-Modul (10 Pins): dasselbe, dazu **NCS–3.3V** (sonst wechselt der Chip auf SPI) und FSYNC–GND; EDA/ECL – nicht anschließen. Chip nach oben, X-Pfeil zur Nase; die Drehung der Chipachsen wird mit `IMU_ROTATION_CW_DEG` in `Config.h` eingestellt (bei unserem Klon 90) |
| BMP581 | VCC–3.3V (**nur 3.3V**: viele Module haben keinen eigenen Spannungsregler), GND–GND, SCL–PB10, SDA–PB11, **SDO–GND** (Adresse 0x46; nicht offen lassen), **CSB–3.3V** (sonst wechselt der Chip auf SPI), INT – nicht anschließen |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–PB10, SDA–PB11, DRDY – nicht anschließen. Möglichst weit weg von den Leitungen der Servos, des ESC und des Motors |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–PB8, SDA–PB9 |

Die Servos werden **nicht über das Board** versorgt, sondern über das BEC des Fahrtreglers (oder über ein separates 5-V-Netzteil mit mindestens 2 A); die Massen aller Quellen sind gemeinsam. Schließen Sie die rote Leitung des ESC nicht an die 5 V des Boards an, solange USB angesteckt ist.

### Matek H743-WING V3 – fertiger Flugcontroller

`pio run -e matek-h743` – das Board Matek H743-WING V3 (H743-WLITE und H743-SLIM V3 haben dieselbe Pinbelegung). Ein fertiger Flugcontroller mit demselben STM32H743: Spannungsregler, Stromsensor, IMU ICM-42688-P, Barometer DPS310 und microSD-Slot sind schon auf dem Board – nichts zu löten; Servos, Regler, Empfänger und GPS kommen an die beschrifteten Pads. **Auf einem echten Board wurde die Firmware noch nicht ausprobiert** (die Pinbelegung stammt aus der Boardbeschreibung in ArduPilot, die ganze Firmware ist durch Tests auf dem PC geprüft) – prüfen Sie vor dem ersten Flug am Boden Ruder, Motor und Modi.

| Zweck | Pad | Pin |
|---|---|---|
| ESC (Gas) | S1 | PB0 |
| Querruder, linke / rechte Fläche | S3 / S4 | PA0 / PA1 |
| Höhenruder | S5 | PA2 |
| Seitenruder + Lenkrad | S6 | PA3 |
| AUX1 / AUX2 (Lastabwurf, Kamera) | S7 / S8 | PD12 / PD13 |
| Empfänger: iBUS, SBUS, CRSF (TX – nur für CRSF) | R6 / T6 | PC7 / PC6 |
| GPS: RX (← TX des GPS) / TX (→ RX des GPS) | R2 / T2 | PD6 / PD5 |
| MAVLink-Telemetrie (Funkmodem): RX / TX | R7 / T7 | PE7 / PE8 |
| Kompass, Pitotrohr (externes I2C) | SDA / SCL | PB7 / PB6 |
| Summer | BZ+ / BZ− | PA15 |
| Akku / Stromsensor | VBAT / Curr | PC0 / PC1 |

- **Sensoren auf dem Board** – IMU und Barometer; ihr Set (`SENSOR_KIT_MATEK_H743`) legt die Umgebung `matek-h743` fest, in `SensorSelection.h` ist nichts zu ändern. Einen Kompass hat das Board nicht: Ein GPS-Modul mit Kompass kommt an die Pads SDA/SCL – in OpenPlane Studio ist das die Kompasswahl im Schritt „Sensoren“, auf der Kommandozeile das Flag `-D SENSOR_MAG=SENSOR_MAG_QMC5883L` in `platformio.ini`.
- **Das Board darf beliebig sitzen**, sogar mit dem Pfeil nach hinten: Die IMU ist aufgelötet, deshalb braucht es vor dem ersten ARM mit Stabilisierung die Einbaukalibrierung mit drei Posen (`o` in der Konsole oder „Einrichtung“ → „IMU-Einbau“ in der App). Ohne sie wird ARM in stabilisierten Modi mit Erklärung abgelehnt; in MANUAL geht es.
- **Erste Firmware:** BOOT-Taste halten und USB einstecken – das Board ist im DFU-Modus (Windows braucht den WinUSB-Treiber wie für das DevEBox); dann `pio run -e matek-h743 -t upload` oder die Schaltfläche „Flashen“ in OpenPlane Studio. Danach startet die Taste `D` in der Konsole das Board selbst in den Bootloader neu.
- **Die Blackbox** schreibt auf die microSD-Karte im Slot des Boards. Der Akku wird über den Teiler des Boards (×11) gemessen, der Stromsensor als seine Ausgangsspannung (40 A pro Volt).
- S2 und S9–S12 sind frei. Die Servos versorgt das BEC des Boards (Schiene Vx) oder das BEC des Reglers – wie in der Anleitung von Matek; die Massen aller Quellen sind verbunden.

### esp32-s3 (N16R8) – früheres Hauptboard, auf dem Prüfstand geprüft

`pio run -e esp32-s3`, Board `esp32-s3-devkitc-1` mit den Einstellungen für das Modul N16R8 (16 MB Flash, 8 MB Octal-PSRAM).

| Verwendung | GPIO |
|---|---|
| Querruder, linke Tragflächenhälfte | GPIO4 |
| Querruder, rechte Tragflächenhälfte | GPIO5 |
| Höhenruder | GPIO6 |
| ESC (Gas) | GPIO7 |
| Seitenruder + Lenkrad | GPIO18 |
| Empfänger (RX): iBUS, SBUS, CRSF | GPIO17 |
| I2C der Sensoren SDA / SCL (MPU, BMP581, Kompass) | GPIO41 / GPIO42 |
| I2C des OLED SDA / SCL (eigener Bus) | GPIO1 / GPIO2 |
| Reserve: GPS RX / TX | GPIO39 / GPIO40 |
| Reserve: AUX1 / AUX2 (Servos), AUX3, Summer, LIGHT | GPIO15 / 16, 47, 38, 21 |
| Akku / Stromsensor (ADC, wird von der Blackbox aufgezeichnet); Reserve: Telemetrie TX / RX | GPIO8 / GPIO3, GPIO9 / GPIO10 |
| Nur Prüfstand: SPI (ICM42688) SCK / MISO / MOSI / CS | GPIO12 / 13 / 11 / 14 (+ CS des BMP388 – GPIO21) |

> Der Sensorbus lag früher auf GPIO8/9 – er wurde auf 41/42 verlegt, passend zum Layout
> der Flugsteuerungsplatine. Auf dem Prüfstand: SDA 8→41, SCL 9→42.

Nicht belegen: GPIO0/45/46 (von ihnen hängt der Boot-Modus ab), 19/20 (USB), 26–32 (Flash), 33–37 (PSRAM beim N16R8), 43/44 (Anschluss „COM“), 48 (RGB-LED). Die freien Pins sind bereits als Reserve verteilt – eine Trägerplatine mit Steckverbindern für die Zukunft: [`FC_BOARD.md`](FC_BOARD.md).

Anschluss der Sensoren auf dem Prüfstand (alle Module laufen mit **3,3 V**, nicht mit 5 V):

| Modul | Pins |
|---|---|
| MPU-6050 / GY-521 (auf der Platine kann ein MPU6500 sitzen – das ist normal) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, AD0–GND, INT/XDA/XCL – nicht anschließen. Separates MPU-6500-Modul (10 Pins): dasselbe, dazu **NCS–3.3V** (sonst wechselt der Chip auf SPI) und FSYNC–GND; EDA/ECL – nicht anschließen. Chip nach oben, X-Pfeil zur Nase; die Drehung der Chipachsen wird mit `IMU_ROTATION_CW_DEG` in `Config.h` eingestellt (bei unserem Klon 90) |
| BMP581 | VCC–3.3V (**nur 3.3V**: viele Module haben keinen eigenen Spannungsregler), GND–GND, SCL–GPIO42, SDA–GPIO41, **SDO–GND** (Adresse 0x46; nicht offen lassen), **CSB–3.3V** (sonst wechselt der Chip auf SPI), INT – nicht anschließen |
| GY-273 (QMC5883P) | VCC–3.3V, GND–GND, SCL–GPIO42, SDA–GPIO41, DRDY – nicht anschließen. Möglichst weit weg von den Leitungen der Servos, des ESC und des Motors |
| OLED 128×64 SSD1306 | VCC–3.3V, GND–GND, SCL–GPIO2, SDA–GPIO1 |

Die Servos werden **nicht über das Board** versorgt, sondern über das BEC des Fahrtreglers (oder über ein separates 5-V-Netzteil mit mindestens 2 A); die Massen aller Quellen sind gemeinsam. Schließen Sie die rote Leitung des ESC nicht an die 5 V des Boards an, solange USB angesteckt ist.

### esp32-c3 – der alte Prototyp, geflogen

`pio run -e esp32-c3`, Board `esp32-c3-devkitm-1`.

| Verwendung | GPIO |
|---|---|
| Querruder, linke Tragflächenhälfte | GPIO5 |
| Querruder, rechte Tragflächenhälfte | GPIO4 |
| Höhenruder | GPIO6 |
| ESC (Gas) | GPIO7 |
| Seitenruder | – (keine freien Pins) |
| Empfänger (RX): iBUS, SBUS, CRSF | GPIO8 |
| I2C SDA (Sensoren) | GPIO1 |
| I2C SCL (Sensoren) | GPIO3 |

### esp32-dev (der gewöhnliche klassische ESP32, 38 Pins) – für Prüfstand und Fehlersuche, NICHT GEFLOGEN

`pio run -e esp32-dev`, Board `esp32dev`.

| Verwendung | GPIO |
|---|---|
| Querruder, linke Tragflächenhälfte | GPIO13 |
| Querruder, rechte Tragflächenhälfte | GPIO14 |
| Höhenruder | GPIO27 |
| ESC (Gas) | GPIO26 |
| Seitenruder | GPIO25 |
| Empfänger (RX): iBUS, SBUS, CRSF | GPIO16 |
| I2C SDA (Sensoren) | GPIO21 |
| I2C SCL (Sensoren) | GPIO22 |

Praktisch ist es, weil es das verbreitetste und günstigste Board der Reihe ist – geeignet zur Fehlersuche an der Firmware auf dem Prüfstand, aber seine Pinbelegung wurde nicht auf Hardware geprüft.

---

## Empfänger anschließen

Der Empfänger wird mit einer einzigen Datenleitung an den **Empfänger-RX-Pin** aus
der Tabelle Ihres Boards angeschlossen (`PIN_RC_RX`: PE7 beim DevEBox, GPIO17 beim
esp32-s3, GPIO8 beim esp32-c3, GPIO16 beim esp32-dev). Das Protokoll muss zu Ihrem
Empfänger passen; es wird mit `RX_PROTOCOL` in `Config.h` festgelegt (in OpenPlane
Studio — der Schritt „Sender“):

| `RX_PROTOCOL` | Empfänger | Was wohin |
|---|---|---|
| `IBUS` (Standard) | FlySky FS-iA6B, FS-iA10B | Port iBUS SERVO (iBUS-Ausgang) → RX-Pin |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper und die meisten anderen | SBUS-Ausgang → RX-Pin; kein Inverter nötig — der UART des Boards invertiert das Signal selbst |
| `CRSF` | ExpressLRS, TBS Crossfire und Tracer | TX des Empfängers → RX-Pin; RX des Empfängers kann an den TX-Pin (`PIN_RC_TX`) — noch ungenutzt |

- **Masse (GND) des Empfängers** → **GND des Boards**. Pflicht: Ohne gemeinsame Masse funktioniert das Protokoll nicht.
- **Versorgung des Empfängers** — 5 V aus einem separaten BEC/Regler oder vom 5V des Boards.

Die Portgeschwindigkeit setzt die Firmware: iBUS — 115200 Baud, SBUS — 100000
(8E2), CRSF — 420000 (der ExpressLRS-Standard). Richten Sie einen EdgeTX-Sender
genauso ein wie den FS-i6 (siehe Kanalbelegung unten): CH5 — ARM, CH6–CH8 —
Schalter, CH9–CH10 — Drehregler.

---

## RC-Kanalbelegung

Die Belegung wurde auf dem Prüfstand mit einem Sender FS-i6 (10 Kanäle, Modus 2) und einem Empfänger FS-iA6B geprüft.

| Kanal | Bedienelement am Sender | Name | Was es tut |
|---|---|---|---|
| CH1 | rechter Knüppel ←→ | AILERON | Rollen – Querruder (2000 = nach rechts) |
| CH2 | rechter Knüppel ↑↓ | ELEVATOR | Nicken – Höhenruder (2000 = Knüppel nach vorn, Nase nach unten) |
| CH3 | linker Knüppel ↑↓ | THROTTLE | Gas. 1000 µs = aus, 2000 µs = Maximum, ohne Begrenzung |
| CH4 | linker Knüppel ←→ | RUDDER | Seitenruder und Lenkrad des Fahrwerks (ein Servo) |
| CH5 | SwA | ARM | ARM-Schalter – siehe den Abschnitt zu ARM weiter unten |
| CH6 | SwB | SWB | Standardmäßig Klappen: unten, zu sich hin – ausgefahren, oben – eingefahren (siehe unten) |
| CH7 | SwC (3 Stellungen) | SWC | Standardmäßig der Modus: oben MANUAL, Mitte STABILIZE, unten AUTO_TAKEOFF |
| CH8 | SwD | SWD | Standardmäßig die Motorumkehr; schaltet nur im DISARM um (siehe unten) |
| CH9 | VrA | VRA | Standardmäßig die Stärke der Stabilisierung |
| CH10 | VrB | VRB | Standardmäßig die Reisegeschwindigkeit |

CH6–CH10 lassen sich in `include/config/Controls.h` **mit einer einzigen Zeile beliebig belegen**: mit jedem der 12 Modi, der 11 Funktionen (Klappen, Bremse, Lastabwurf, Geofence, Summer, Motorumkehr …) und der 8 Drehregler. Beim Einschalten des Boards wird die tatsächliche Belegung im seriellen Monitor ausgegeben. Alles zu den Modi und Zuordnungen steht in [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

Alle Kanalwerte liegen im üblichen Impulsbereich des Empfängers von 1000–2000 µs, wobei 1500 µs die Mitte/Neutralstellung ist.

### Klappen (Flaperons)

Eigene Klappen gibt es nicht – ihre Rolle übernehmen die Querruder. **SwB nach unten** (zu sich hin): Beide Querruder senken sich sanft, in etwa einer Sekunde, um denselben Winkel (`FLAPS_DEPLOYED_US` = 220 µs ≈ 20° Drehung des Servohebels – einstellbar in `include/config/Config.h`). Das erhöht den Auftrieb, damit Start und Landung mit geringerer Geschwindigkeit möglich sind. Das Rollen per Knüppel und durch den Autopiloten funktioniert wie gewohnt – die Querruder schlagen gegensinnig aus, nun aber um die abgesenkte Stellung herum. **SwB nach oben** – sie fahren ebenso sanft wieder ein. Auf dem OLED leuchtet bei ausgefahrenen Klappen in der ersten Zeile `FL`.

Bei vollem Rollen mit ausgefahrenen Klappen erreicht das nach unten ausschlagende Querruder seinen Anschlag früher als das nach oben ausschlagende – das ist normal und wirkt wie eine Querruderdifferenzierung.

Das Ausfahren der Klappen hebt meist die Nase – seien Sie darauf gefasst, den Knüppel etwas nach vorn zu geben; ist der Effekt stark, lässt er sich durch Verkleinern von `FLAPS_DEPLOYED_US` abstimmen.

---

## Autopilot-Kanäle

Kurz gesagt – die Standardbelegung (`include/config/Controls.h`):

| Schalter | Was er tut |
|---|---|
| **SwC** (CH7) | oben **MANUAL** · Mitte **STABILIZE** · unten **AUTO_TAKEOFF** |
| **SwD** (CH8) | **Motorumkehr** – nur im DISARM (RTH kommt mit einer Zeile wieder auf den Schalter) |
| **SwB** (CH6) | Klappen |
| **VrA / VrB** (CH9/10) | Stärke der Stabilisierung / Reisegeschwindigkeit |

- **STABILIZE – „der Knüppel gibt den Winkel vor“.** Voller Knüppel bedeutet 45° Rollen und 25° Nicken; loslassen, und das Flugzeug richtet sich von selbst waagerecht aus. Das Gas gehört Ihnen.
- **AUTO_TAKEOFF.** Start von der Piste in Etappen. Nach ARM bleibt der Motor stehen, bis Sie das Gas über die Hälfte schieben; das Flugzeug prüft ~1 s lang, ob es so steht wie bei der Kalibrierung, und macht dann alles selbst: Heck hoch mit Tiefenruder, Startlauf, Abheben über die Tragfläche, Steigflug auf 30 m, danach hält es Kurs und Höhe. Das Gas gehört dem Programm; Gasknüppel nach unten vor dem Abheben bedeutet Startabbruch. Braucht die [Bodenkalibrierung des Flugzeugs](#bodenkalibrierung-des-flugzeugs-horizont-und-standlage).
- **RTH.** Kurs auf den ARM-Punkt, Höhe 40 m, über dem Startpunkt Kreise. Ein GPS mit 3D-Fix wird **vor dem ARM** benötigt.

Alle 12 Modi (ALT_HOLD, ACRO, CRUISE, LOITER, LAUNCH per Hand, AUTO_LAND, SOARING, RESCUE …), welche Sensoren jeder braucht und wie man ihn auf einen Schalter legt, steht in [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md). Ein vom Dashboard oder von der Bodenstation gewählter Modus bleibt bestehen, bis Sie den Modusschalter betätigen.

Die Stabilisierung bewegt die Ruder auch dann, wenn das Flugzeug **nicht scharfgeschaltet** ist – so sieht man auf dem Tisch, in welche Richtung es auf eine Neigung reagiert. Der Integrator sammelt sich dabei nicht auf.

Vorzeichen der Winkel (wie auf dem OLED und im Log): **Rollen R > 0 – rechter Flügel nach unten, Nicken P > 0 – Nase nach oben.**

---

## ARM und Failsafe

### ARM-Ablauf

- **ARM:** Gas (CH3) unten → Schalter **SwA (CH5) nach unten, zu sich hin** (bei der FS-i6 ist das CH5 = 2000; oben = 1000). In Serial erscheint `ArmingManager: ARM`, auf dem OLED `ARMED`.
- **DISARM:** SwA nach oben, von sich weg – sofort, jederzeit; der Motor stoppt augenblicklich.
- Wird SwA auf ARM gelegt, obwohl das Gas nicht unten ist, oder wurden die Prüfungen vor dem Flug nicht bestanden, findet ARM **nicht** statt (der Grund wird in Serial ausgegeben). Dann muss SwA wieder nach oben gelegt, das Gas zurückgenommen und der Schalter erneut nach unten gelegt werden.
- Wird das Board eingeschaltet, während SwA bereits in der ARM-Stellung (unten) steht, wird es **nicht** scharf: Die Firmware muss SwA zuerst oben (OFF) sehen.

**ARM sperrt das Gas tatsächlich:** Solange das Flugzeug nicht scharfgeschaltet ist, wird das Gas am ESC unabhängig vom Knüppel zwangsweise auf null gehalten (bei einem bidirektionalen ESC ist das der Neutralimpuls von 1500 µs). Vor dem Scharfschalten wird geprüft, ob die für den gewählten Modus nötigen Sensoren antworten – STABILIZE ohne funktionierende IMU etwa lässt sich nicht scharfschalten, bis Sie auf MANUAL wechseln. Verhalten Sie sich dennoch so, als könnte der Propeller nach dem ARM jeden Moment anlaufen.

### Motorumkehr (SwD)

Der Hauptregler ist ein bidirektionaler BLHeli 80A: Der Motor dreht in beide Richtungen, **SwD nach unten** – rückwärts, nach oben – vorwärts. Die Umkehr dient zum Zurückrollen am Boden.

- **Die Richtung wechselt nur im DISARM.** SwD bei ARM umgelegt – nichts passiert (im Serial steht `MotorReverse: реверс НЕ переключён` – „Umkehr NICHT umgeschaltet“), der Motor dreht weiter wie bisher. Die neue Richtung gilt, sobald Sie disarmen. So lässt sich die Umkehr nicht bei Vollgas einschalten und Regler und Motor verbrennen.
- **3 Sekunden nach einem Richtungswechsel ist kein ARM möglich** (`REVERSE_SETTLE_MS`): Wurde mit Gas disarmt, dreht der Motor noch nach – lassen Sie ihn stehen bleiben. In dieser Zeit wird ARM mit Begründung im Serial abgelehnt.
- **Rückwärts – nur Ihr Gas und höchstens 50 %** (`REVERSE_THROTTLE_LIMIT_PCT`): voller Knüppel = halbe Leistung. In der Umkehr fasst der Autopilot das Gas nicht an (die Ruder arbeiten wie gewohnt), bei Verbindungsverlust bleibt der Motor einfach stehen.
- Zu sehen auf dem OLED (`REV` in der ersten Zeile) und im Dashboard (Badge „Мотор: РЕВЕРС“ – „Motor: UMKEHR“).

**Einstellung des Reglers in BLHeliSuite** (einmalig, Propeller ab): Motor Direction – **Bidirectional** (bei BLHeli_32 – „Bidirectional“/3D), PPM Min Throttle – 1000, PPM Max Throttle – 2000, Center – 1500. Der Regler schaltet scharf, wenn er die Neutralstellung 1500 µs sieht, und die Firmware gibt sie ab dem ersten Impuls aus. Dreht der Motor bei Knüppel nach vorn falsch herum, setzen Sie `MOTOR_REVERSED = true` in `Config.h` (eine Zeile statt Umlöten).

Gewöhnlicher Regler mit einer Drehrichtung: `ESC_BIDIRECTIONAL = false` in `Config.h`, sonst gibt er bei „Stopp“ (1500) halbes Gas! Die Umkehr wird dann nie eingeschaltet.

### Failsafe

Der Verbindungsverlust hat **absoluten Vorrang** vor allem anderen:

- **Flugzeug scharfgeschaltet, GPS und Startpunkt vorhanden** – **Rückkehr nach Hause mit Motor** (wie bei ArduPilot/INAV): Kurs auf den Startpunkt, Höhe 40 m, Kreise darüber, bis die Verbindung zurückkehrt. Auf dem OLED `FSRTH`. Abschaltbar mit `FAILSAFE_RTH = false` in `Config.h`;
- **Flugzeug scharfgeschaltet, kein GPS** – **Gleitflug**: Motor aus; der Autopilot hält in jedem Modus, sogar in MANUAL, die Flügel waagerecht und die Nase leicht unter dem Horizont (−3°), die Klappen fahren ein. Auf dem OLED `RX LOST ... GLIDE`. Setzt man `FAILSAFE_GLIDE_ROLL_DEG` = 10–20, kreist das Flugzeug in einer flachen Spirale über Ihnen;
- **Motor in Umkehr** – der Motor ist in jedem Fall aus, der Autopilot steuert nur mit den Rudern (nach Hause im Gleitflug);
- **nicht scharfgeschaltet** (am Boden) oder die IMU antwortet nicht – Motor aus, Querruder, Höhen- und Seitenruder in Neutralstellung (1500 µs); nach 10 s ohne Verbindung piept der Summer „Ich bin hier“ (falls er eingelötet ist).

Die Firmware erkennt den Verbindungsverlust auf drei Arten:

1. **Länger als 500 ms keine Frames vom Empfänger** – Leitungsbruch, Empfänger ohne Strom oder ein Empfänger, der bei Verbindungsverlust verstummt (so macht es ExpressLRS standardmäßig).
2. **Der Empfänger meldet den Verbindungsverlust selbst** – das Failsafe-Flag im SBUS-Frame, Verbindungsqualität 0 % bei CRSF. Nichts einzurichten.
3. **Gas unter 950 µs** – so meldet der Empfänger, dass er den Sender verloren hat. **Das erfordert die Einrichtung von Failsafe im Sender** (siehe unten): Bei Verbindungsverlust hört der FS-iA6B NICHT auf, Frames zu senden, sondern wiederholt die letzten Knüppelwerte – ohne diese Einrichtung sieht die Firmware den Verbindungsverlust nicht, und das Flugzeug fliegt mit dem letzten Gas weiter.

ARM wird bei Failsafe **nicht** zurückgesetzt: Kehrt die Verbindung zurück, gehorcht das Flugzeug wieder den Knüppeln und dem gewählten Modus, ohne erneut scharfgeschaltet zu werden (den Schalter in der Luft bei Gas null umzulegen wäre gefährlicher).

Test auf dem Tisch (Propeller abgenommen): ARM → Sender ausschalten → auf dem OLED `RX LOST ... GLIDE`, der Motor ist stehen geblieben; neigen Sie das Flugzeug – die Ruder sollten es in die Waagerechte zurückführen. Sender einschalten – `RX ok`, die Steuerung liegt wieder bei den Knüppeln.

### Failsafe im Sender FS-i6 einrichten (Pflicht, einmalig)

Die Idee: Bei Verbindungsverlust soll der Empfänger ein Gas von ca. 900 µs ausgeben – unter dem normalen Minimum von 1000.

1. `Menu → Functions setup → End points` → Kanal 3: den unteren Punkt (den linken Wert) auf **120 %** setzen. Speichern (Cancel lange drücken).
2. Gas – **ganz nach unten**.
3. `Menu → Functions setup → Failsafe` → Channel 3 → **On**, Gas weiterhin unten → mit langem Druck auf Cancel speichern. Der Empfänger merkt sich ca. 900 µs.
4. Zurück zu `End points` → Kanal 3 → den unteren Punkt wieder auf **100 %** setzen. Speichern.
5. Test: ARM ist nicht nötig. Schalten Sie den Sender aus – nach etwa 1 s erscheint in Serial `RX=LOST(failsafe пульта)` und auf dem OLED die invertierte Zeile `RX LOST`. Sender einschalten – `RX=OK`.

Halten Sie die Gastrimmung in der Mitte: Bei stark abgesenkter Trimmung kann das Gas unter 950 fallen, und die Firmware hält das für einen Verbindungsverlust.

Früher gab es hier einen **Boost auf CH8** und eine Gasbegrenzung auf 40 % – beides wurde entfernt: Die Begrenzung schonte einen schwachen 3S1P-Aufbau, und neuen Akkus macht Vollgas nichts aus. Der Boost schaltete sich außerdem von selbst ein, wenn SwD beim Einschalten des Boards oben stand.

---

## Board flashen

Die Firmware wird mit **PlatformIO** gebaut (Arduino-Framework, C++).

### PlatformIO installieren

Am einfachsten installiert man die Erweiterung **PlatformIO IDE** in VS Code (Extensions → „PlatformIO IDE“ suchen → Install); dann stehen sowohl die CLI als auch bequeme Build-Schaltflächen in der Oberfläche zur Verfügung. Man kann es auch mit `pip install platformio` installieren und im Terminal arbeiten – beide Varianten nutzen dieselben `pio`-Befehle.

### Bauen und hochladen

Öffnen Sie das Projekt (den Ordner des Repositorys) in VS Code mit installiertem PlatformIO, schließen Sie das Board per USB an und führen Sie im Terminal den Befehl für Ihr Board aus:

```bash
# STM32H743 DevEBox (Hauptboard)
pio run -e stm32h743-devebox -t upload

# Matek H743-WING V3 (fertiger Flugcontroller): BOOT-Taste + USB
pio run -e matek-h743 -t upload

# esp32-s3 N16R8 (früheres Hauptboard)
pio run -e esp32-s3 -t upload

# esp32-c3 (alter Prototyp)
pio run -e esp32-c3 -t upload

# esp32-dev (klassischer ESP32 38-Pin, für den Prüfstand)
pio run -e esp32-dev -t upload
```

Wenn Sie `-e` gar nicht angeben, wird das Standardboard gebaut – `stm32h743-devebox`.

**STM32 DevEBox:** Die erste Firmware kommt über USB-DFU: Brücke BT0→3V3, RST drücken, dann der Befehl oben (Windows braucht den WinUSB-Treiber für „STM32 BOOTLOADER“, installiert mit Zadig). Danach startet die Taste `D` in der Konsole das Board selbst in den Bootloader neu, die Brücke wird nicht mehr gebraucht. Die Konsole läuft über dieselbe USB-C-Buchse.

**Matek H743:** Die erste Firmware kommt ebenfalls über USB-DFU, aber ohne Brücke: BOOT-Taste auf dem Board halten, USB einstecken und den Befehl oben ausführen. Danach – die Taste `D` in der Konsole.

**esp32-s3:** Das Board hat zwei USB-C-Buchsen. Flashen und Serial laufen über die Buchse **„COM“** (CH343-Brücke; unter Windows „USB-Enhanced-SERIAL CH343“). Die Buchse „USB“ (der native USB des Chips) wird für den Betrieb nicht gebraucht, darf aber angesteckt sein – sie stört nicht.

### Serieller Monitor

Um die Debug-Ausgabe (Kanalzustände, ARM, Sensoren) direkt in der Konsole über USB zu sehen:

```bash
pio device monitor -b 115200
```

Die Geschwindigkeit muss unbedingt 115200 betragen – sonst sehen Sie statt Text unlesbaren Zeichensalat. Alle 10 Sekunden wird eine Zeile `SYS` ausgegeben: die Schleifenfrequenz (sie sollte bei ca. 500 Hz liegen), die mittlere und die schlechteste Schleifenzeit der letzten 10 s, die iBUS-Zähler, der freie Speicher. Alles Übrige kommt über die Kanäle, die im Log-Menü eingeschaltet sind (siehe unten).

### Konsole: Menü und Log (serieller Monitor)

Ein Tastendruck wirkt sofort, Enter ist nicht nötig (bei einem Monitor, der zeilenweise sendet: Buchstabe + Enter). Kalibrierungen und die Ausgangsprüfung funktionieren nur, wenn das Flugzeug nicht scharfgeschaltet ist.

| Taste | Was sie tut |
|---|---|
| `h` | **Hauptmenü** (textbasiert, die Punkte sind nummeriert) |
| `l` | Menü „Was ins Log ausgegeben wird“ |
| Leertaste | Log anhalten / fortsetzen |
| `s` | ausführlicher Status aller Sensoren (einschließlich der I2C-Fehlerzähler) |
| `i` | Gyroskop neu kalibrieren und IMU-Prüfung vor dem Flug – 2 s, das Flugzeug nicht bewegen |
| `o` | **Kalibrierung der IMU-Einbaulage** – einmalig, nachdem die Platine ins Flugzeug eingebaut wurde (siehe unten) |
| `c` | **Bodenkalibrierung des Flugzeugs** – Horizont und Standlage (siehe unten); läuft sie, wird sie abgebrochen |
| `e` | Signal: Rudernicken und ESC-Melodie – prüfen, ob der Regler sie spielt (Propeller abnehmen!) |
| `m` | Kompasskalibrierung – 15 s lang die Platine bzw. das Flugzeug um alle Achsen drehen. Das Ergebnis wird im Flash gespeichert und übersteht einen Neustart |
| `p` | Ausgangsprüfung: der tatsächliche Impuls an jedem Pin |

**Log nach Kanälen.** Jede Art von Daten ist eine eigene Zeile mit eigenem Präfix und hat ihren eigenen Modus: **aus**, **bei Änderung** (eine Zeile erscheint nur, wenn sich die Werte wirklich geändert haben – Knüppelzittern und Sensorrauschen zählen nicht) oder **dauerhaft** (alle 0,2 / 0,5 / 1 / 2 s – die Periode wird im selben Menü eingestellt).

| Kanal | Was er zeigt | Standard |
|---|---|---|
| `STAT` | Verbindung, ARM, Modus, Klappen, ob IMU und Barometer in Ordnung sind | bei Änderung |
| `RC` | Senderkanäle, µs | aus |
| `OUT` | Ausgänge an die Ruder und den ESC, µs | aus |
| `ATT` | Rollen, Nicken, Kurs | aus |
| `AP` | Autopilot: Sollwerte und Korrekturen | aus |
| `ALT` | Höhe, Vertikalgeschwindigkeit, Sollwert von ALT_HOLD | aus |
| `MAG` | Kurs laut Kompass | aus |
| `GPS` | Fix, Satelliten, Koordinaten, Geschwindigkeit | aus |
| `IMU` | Gyroskop und Beschleunigungssensor | aus |
| `SYS` | Schleifenfrequenz und -dauer, Speicher (alle 10 s) | ein |

Solange das Menü offen ist, schweigt das Log, damit das Menü nicht wegscrollt; nach dem Verlassen werden alle eingeschalteten Kanäle neu ausgegeben. Die Auswahl wird beim Verlassen des Menüs im Flash gespeichert und übersteht einen Neustart. Bei scharfgeschaltetem Flugzeug wirkt sie sofort, wird aber erst nach dem DISARM geschrieben: Ein Schreibvorgang im Flash hält die Flugschleife für ca. 0,4 s an.

### IMU-Einbau: beliebig, eine Kalibrierung

Die Platine mit der IMU lässt sich **so, wie es bequem ist,** ins Flugzeug einbauen – in jedem Winkel, auf der Seite, auf dem Kopf: Die Firmware stellt selbst fest, wo ihre Nase und ihre Oberseite sind. Das geschieht **einmalig** nach dem Einbau (und erneut, wenn die Platine umgesetzt wurde):

1. Das Flugzeug auf dem Tisch, der Sender wird nicht gebraucht, der Motor ist nicht scharfgeschaltet. Im seriellen Monitor `o` drücken.
2. **Schritt 1:** Das Flugzeug steht waagerecht wie im Geradeausflug (ein Spornradflugzeug darf auf den Rädern stehen – den Horizont setzt später die Bodenkalibrierung des Flugzeugs). ~3 s nicht berühren.
3. **Schritt 2:** Heben Sie die **Nase** um 30–60° an, die Flügel bleiben waagerecht, und halten Sie etwa 1 s still.
4. **Schritt 3:** Nase wieder zurück, senken Sie den **rechten Flügel** um 30–60° und halten Sie etwa 1 s.

Jeder Schritt wird von selbst gewertet (im Log steht „засчитано“, also „gewertet“). Am Ende steht, was dabei herausgekommen ist („нос = +Y чипа, верх = −Z чипа“, also „Nase = +Y des Chips, oben = −Z des Chips“), und „установка сохранена“ („Einbaulage gespeichert“). Haben Sie etwas verwechselt (die Nase gesenkt statt gehoben, den linken Flügel statt des rechten), wird die Kalibrierung mit einer Erklärung abgelehnt; wiederholen Sie einfach `o`. Das Ergebnis liegt im Flash und übersteht einen Neustart. Kontrolle: Nase hoch → P auf dem OLED wird positiv; rechter Flügel nach unten → R wird positiv.

Solange es keine Einbaukalibrierung gibt, gilt das alte Verfahren: Die Platine muss mit dem Chip nach oben liegen, die Drehung ist `IMU_ROTATION_CW_DEG` in `Config.h`. Nach `o` die Bodenkalibrierung des Flugzeugs wiederholen: Die Standlage wurde vom alten Horizont aus gemessen.

### Bodenkalibrierung des Flugzeugs: Horizont und Standlage

Ein Spornradflugzeug steht am Boden mit erhobener Nase – um wie viel, weiß vorher niemand. Die Kalibrierung misst das selbst und speichert es im Flash: den **Horizont** (0° Nick und Roll für alle Modi – nicht mehr „wie es beim Einschalten stand“) und die **Standlage**, aus der der Autostart ableitet, wie weit das Heck hoch muss und wo das Spornrad die Piste berühren würde. Einmal nach dem Zusammenbau (und erneut, wenn die Platine versetzt wurde oder nach `o`). Kein Laptop nötig:

1. Flugzeug auf ebenem Grund, Sender an, **kein ARM**, Gas unten.
2. **SwB (Klappen) dreimal hintereinander runter und hoch** (innerhalb von 4 s) – oder `c` in der Konsole. Ein Signal – es geht los.
3. **Schritt 1 – Horizont.** Heben Sie das Heck, bis der Rumpf waagerecht steht wie im Flug (Haupträder am Boden; eine Wasserwaagen-App auf dem Handy oben auf dem Rumpf hilft), und halten Sie ~2 s still. Schauen Sie dabei, wie viel Platz zwischen Propeller und Boden bleibt: Etwa so rollt das Flugzeug beim Start. Ein Signal – übernommen.
4. **Schritt 2 – Standlage.** Stellen Sie das Flugzeug auf seine drei Räder und nehmen Sie die Hände weg. Nach ~2 s – zwei Signale: fertig, im Log steht „Стоянка: нос вверх на 9.4°, крен 0.3°“ (Standlage: Nase 9,4° hoch, Roll 0,3°).

Drei Signale – ein Fehler, nichts gespeichert (der Grund steht in der Konsole): Das Flugzeug stand eine Minute lang nicht still, in der Standlage ist die Nase kaum erhoben (Heck nicht abgesetzt) oder um mehr als 25°, oder der Roll liegt über 10°. Abbrechen: dieselbe SwB-Geste oder ARM kurz umlegen (dieses Umlegen schaltet nicht scharf). Mit Bugrad (`NOSE_WHEEL`) kann Schritt 1 direkt auf den Rädern erfolgen, wenn das Flugzeug waagerecht steht: Schritt 2 wird dann nach 5 s von selbst übernommen. Der Schalter für die Geste ist `CAL_SWITCH` in `Config.h` (SwD geht nicht: Er schaltet die Motorumkehr).

**Signale.** Ein Signal: Höhen- und Seitenruder nicken (in Schritt 1 haben Sie das Heck in der Hand und spüren es), und ohne ARM singt der Motor: Die Impulse zum ESC verschwinden für eine Sekunde, der Regler hält das Signal für verloren und spielt, wenn die Impulse zurückkommen, seine Bereitschaftsmelodie – wie beim Einschalten. Der Motor dreht sich dabei nicht. Einen anderen „Piep“-Befehl kennt ein Regler über PWM nicht, deshalb auf dem Prüfstand testen (**Propeller ab**): Taste `e` in der Konsole – die Melodie muss erklingen. Erklingt sie nicht, `SIGNAL_MOTOR = false` in `Config.h` setzen, dann bleibt das Nicken. Ein Summer oder eine LED mit Vorwiderstand an PE15 (S3: GPIO38) leuchtet während des Nickens; die LED D2 auf der DevEBox selbst hängt an PA1, dem Ausgang des rechten Querruders, und taugt deshalb nicht für Signale. Solange ein Signal läuft, wird ARM abgelehnt: Der Regler hat sich vielleicht noch nicht wieder scharfgeschaltet. Auf dem OLED zeigt die Lagezeile `CAL1`, `CAL2`, als Ergebnis `CALOK` oder `CALER`.

### Prüfung vor dem Flug beim Einschalten

Bei jedem Einschalten kalibriert die IMU ca. 2 s lang das Gyroskop und prüft dabei gleichzeitig sich selbst:

- **das Flugzeug ist unbewegt** – wurde es in diesem Moment in der Hand gehalten oder bewegt, ist der Gyroskop-Offset falsch;
- der Beschleunigungssensor zeigt in Ruhe 1g;
- **„oben“ stimmt mit der Einbaukalibrierung überein** – wurde die Platine umgesetzt oder umgedreht, sieht man das sofort (ein Flugzeug auf dem Spornrad oder an einem Hang ist unproblematisch, die Toleranz beträgt 45°).

**Schalten Sie das Flugzeug im Stillstand ein.** Waagerecht muss es nicht stehen, wenn der Einbau kalibriert ist (`o` oder die Bodenkalibrierung des Flugzeugs; sonst wird die Lage beim Einschalten zum Horizont). Das Ergebnis steht im Log: `предполётная проверка пройдена` („Prüfung vor dem Flug bestanden“) oder `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА — <причина>` („PRÜFUNG VOR DEM FLUG NICHT BESTANDEN“, gefolgt vom Grund). Wurde sie nicht bestanden, ist **ARM in STABILIZE und AUTO_TAKEOFF verboten** (der Grund wird beim Versuch ausgegeben), und der Autopilot gibt in keinem Modus Korrekturen aus, auch nicht beim Gleitflug bei Verbindungsverlust: Mit falschen Winkeln würde er in die falsche Richtung steuern. ARM in MANUAL ist möglich. Abhilfe: das Flugzeug ruhig hinstellen und den Akku neu anstecken (oder `i`); wurde die Platine umgesetzt, `o`.

### OLED-Display

Ist ein OLED angeschlossen (GPIO1/GPIO2), zeigt es 5-mal pro Sekunde Folgendes:

```
RX ok disarm STAB        <- Verbindung / ARM / Modus / REV – Motor rückwärts (bei Verbindungsverlust ist die Zeile invertiert)
R  +1.2 P  -0.4 ROLL     <- Roll / Nick, °; Kalibrierschritt oder Startetappe
Alt +0.3 Vz +0.1         <- Höhe ab dem Einschaltpunkt, m / Vertikalgeschwindigkeit, m/s
Hdg 123  Thr 1000        <- Kompasskurs / Gas 1000..2000 = 0..100 % (auch rückwärts), µs
L1500 R1500 E1500        <- PWM von Querrudern und Höhenruder, µs
Loop 500Hz max 1100us    <- Frequenz und schlechteste Schleifenzeit
```

---

## Web-Dashboard im Feld

Auf dem Board läuft ein eigener WLAN-Zugangspunkt – ein Router zu Hause oder Internet wird nicht gebraucht, alles funktioniert direkt im Feld mit dem Smartphone.

**So verbinden Sie sich:**

1. Öffnen Sie auf dem Smartphone oder Laptop die Liste der WLAN-Netze.
2. Verbinden Sie sich mit dem Netz **`OpenPlane-Debug`**, Passwort **`12345678`**.
3. Öffnen Sie im Browser die Adresse **`http://192.168.4.1`**.

Eine App muss nicht installiert werden – es ist eine ganz normale Webseite.

**Was sich im Feld vom Smartphone aus ganz ohne Programmieren tun lässt:**

- Die **Live-Balken aller 10 RC-Kanäle** ansehen – praktisch, um zu prüfen, dass Sender und Empfänger wirklich das senden, was Sie am Knüppel bewegen, noch bevor die Servos angeschlossen sind.
- Den Status jedes Ausgangs sehen (Querruder links/rechts, Höhenruder, ESC) – ob der Kanal softwareseitig angeschlossen ist.
- Sehen, ob die Sensoren (IMU, Barometer) antworten, falls sie verlötet sind – ehrlich angezeigt werden entweder echte Daten (Rollen/Nicken/Höhe) oder ein ausdrücklicher Vermerk, dass der Sensor physisch nicht vorhanden ist oder nicht antwortet.
- Den **Autopilot-Modus umschalten** per Schaltflächen (manuell / Stabilisierung / automatischer Start / Höhenhaltung) direkt von der Seite aus – ohne Sender.
- Die **Koeffizienten des PID-Reglers einstellen** (für Rollen und Nicken) über ein Formular auf der Seite – nützlich, um die Stabilisierung nach und nach abzustimmen, ohne neu zu flashen.

Die Reichweite dieses Zugangspunkts beträgt in der Praxis einige Dutzend Meter; es ist das gewöhnliche WLAN des ESP32, keine Telemetrie mit großer Reichweite. Er ist ein Werkzeug zum Abstimmen auf dem Tisch, auf dem Prüfstand und neben dem Feld – nicht zum Steuern des Flugzeugs im Flug aus der Ferne.

---

## Blackbox

Die Firmware des ESP32-S3 schreibt jeden Flug selbst in den Flash: alles, was die Sensoren gesehen haben, was die Knüppel getan haben, wohin die Servos gefahren sind und was der Autopilot entschieden hat. Man muss nichts tun:

- die Aufzeichnung **beginnt**, sobald das Flugzeug scharfgeschaltet und Gas gegeben ist (plus die 10 s davor);
- sie **endet** 10 s nach dem DISARM – oder wenn ein scharfgeschaltetes Flugzeug 30 s lang bei ausgeschaltetem Motor regungslos steht (gelandet oder abgestürzt, und das DISARM wurde vergessen);
- Verbindungsverlust, Motor auf null und Gleitflug beenden sie **nicht**.

Beim Einschalten zeigt der serielle Monitor, wie viel Platz für einen Flug vorhanden ist (die Konsole gibt Russisch aus; die Zeile unten bedeutet „wartet auf ARM und Gas | voraus gelöscht: 12,9 MB (≈11 min) von 13,9 MB | Flüge: 1“):

```
BlackBox: ждёт ARM и газ | стёрто впереди 12.9 МБ (≈11 мин) из 13.9 МБ | полётов 1
```

**Nach dem Flug** USB anschließen (die COM-Buchse), den seriellen Monitor schließen und den Flug herunterladen:

```bash
python tools/blackbox.py download
```

Der Flug landet zusammen mit der Auswertung im Ordner `blackbox/`: `summary.txt` (Zusammenfassung und Ereignisse), `events.txt` und CSV-Tabellen je Sensor. Alte Flüge löscht die Blackbox selbst, wenn sie Platz braucht – **laden Sie nach jedem Flug herunter**. Das Menü in der Konsole ist die Taste `k`. Alles Weitere steht in [BLACKBOX.md](BLACKBOX.md).

**Auf dem Board STM32H743** schreibt die Blackbox auf eine SD-Karte: Die Karte wird als FAT32 formatiert und einmalig an einem PC vorbereitet (`python tools/blackbox.py sd-prepare E:`). Nach dem Flug lässt sich die Aufzeichnung mit demselben Befehl `download` über USB herunterladen, oder man zieht die Karte heraus und wertet die Datei direkt von ihr aus: `python tools/blackbox.py ring E:/BLACKBOX.BIN`.

---

## Checkliste vor dem Flug und Sicherheit

Lesen Sie diesen Abschnitt vollständig **vor** dem ersten Einschalten, nicht erst nach einem Zwischenfall.

### Pflicht vor jedem Prüfstandstest

- [ ] **Der Propeller ist physisch ABGENOMMEN**, wenn Sie Kanäle, ARM oder das Dashboard prüfen, den PID einstellen oder das Board einfach zum ersten Mal mit einer neuen Pinbelegung einschalten. Der ESC kann den Motor in jeder Phase eines Tests ruckeln lassen – das ist kein hypothetisches Risiko, sondern normales Verhalten beim ersten Einschalten.
- [ ] Der LiPo-Akku wurde auf Aufblähung und Beschädigung geprüft, mit einem passenden LiPo-Ladegerät geladen und wird auf einer nicht brennbaren Unterlage gelagert und geladen.
- [ ] Der Sender ist eingeschaltet, und seine Kanäle wurden im Dashboard (`http://192.168.4.1`) geprüft, **bevor** der Akku an den ESC angeschlossen wird.

### Vor dem Flug

- [ ] Ein offenes Gelände, ohne Menschen und Bauten in einem Umkreis, der für ein Flugzeug mit 1200 mm Spannweite bei manueller Steuerung und abnormalem Verhalten von Servos oder Tragfläche ausreicht (siehe unten den Abschnitt zu den Einschränkungen des Prototyps – Motorbefestigung und Tragfläche sind noch nicht mit Carbon verstärkt).
- [ ] Alle drei Ruder bewegen sich in die richtige Richtung – prüfen Sie das vor jedem Flug am Tisch und verlassen Sie sich nicht auf die Erinnerung vom letzten Mal:
  - rechter Knüppel nach rechts → **rechtes Querruder hoch, linkes runter**;
  - rechter Knüppel zu sich → **Höhenruder hoch**;
  - linker Knüppel nach rechts → **Seitenruder und Lenkrad nach rechts**;
  - SwB (Klappen) nach unten → **beide Querruder gehen sanft nach unten**, und das Rollen per Knüppel spreizt sie dabei weiterhin in entgegengesetzte Richtungen;
  - Gas nach vorn (Propeller ab – auf die Drehrichtung achten) → **der Propeller zieht nach vorn**;
  - in STABILIZE das Flugzeug mit dem rechten Flügel nach unten neigen → **rechtes Querruder runter, linkes hoch** (die Ruder bringen es zurück in die Waagerechte); Nase nach unten → **Höhenruder hoch**.
  Stimmt etwas nicht, ändern Sie eine Zeile im Block „Umkehr“ von `include/config/Config.h`: jedes Querruder, Höhenruder, Seitenruder, AUX, der Motor (`MOTOR_REVERSED`) und die Klappen (`FLAPS_REVERSED`) werden einzeln umgekehrt, sonst wird nichts berührt. Servos nicht im Sender umkehren – sonst gehen Knüppel und Autopilot auseinander; einen Kanal umgekehrt (Knüppel, Schalter, Drehregler) – `CHANNEL_REVERSED` an derselben Stelle.
- [ ] Beim Einschalten stand das Flugzeug still, und im Log bzw. im Dashboard steht kein `ПРЕДПОЛЁТНАЯ ПРОВЕРКА НЕ ПРОЙДЕНА`; beim Neigen des Flugzeugs in den Händen ändern sich P und R auf dem OLED in die richtige Richtung.
- [ ] Für AUTO_TAKEOFF: Die Bodenkalibrierung des Flugzeugs ist gemacht (beim Einschalten steht im Log „стоянка на колёсах — тангаж …“), das Spornrad ist heil, das Flugzeug steht gegen den Wind auf der Piste. Die ersten Läufe mit Gasbegrenzung (Drehregler `THROTTLE_LIMIT`) und der Hand am Gas: nach unten heißt Startabbruch.
- [ ] Failsafe ist im Sender eingerichtet und geprüft: Sender ausgeschaltet → `RX LOST` auf dem OLED bzw. in Serial (siehe den Abschnitt zu Failsafe).
- [ ] Die Reichweite des Senders wurde geprüft, der Senderakku ist geladen.
- [ ] Beim Einschalten steht im Log `BlackBox: ... стёрто впереди N МБ (≈M мин)` – der Platz reicht für den Flug. Der vorige Flug wurde heruntergeladen (`python tools/blackbox.py download`), falls Sie ihn brauchen.
- [ ] Halten Sie Hände und Gesicht jederzeit vom Propeller fern, solange ein LiPo-Akku am ESC angeschlossen ist – nach dem ARM folgt das Gas sofort dem Knüppel, ohne weitere Warnung. DISARM – SwA nach oben.
- [ ] Stellen Sie sicher, dass Sie die Stromversorgung physisch schnell trennen können (Zugang zum LiPo-Stecker), statt sich allein auf Failsafe bei Signalverlust zu verlassen.

### Allgemeine Sicherheit mit LiPo-Akkus

- Lassen Sie einen ladenden LiPo-Akku niemals unbeaufsichtigt.
- Schließen Sie den LiPo-Akku nicht am ESC an und trennen Sie ihn nicht, wenn Sie neben der Rotationsebene des Propellers stehen.
- Transportieren und lagern Sie LiPo-Akkus in einer Schutztasche bzw. einem Schutzbehälter.

---

## Fehlersuche

**„Kein Empfängersignal“ / RX sieht normal aus, aber die Kanäle im Dashboard bewegen sich nicht**
Prüfen Sie, dass die Datenleitung des Empfängers genau am Empfänger-RX-Pin aus der Tabelle Ihres Boards hängt (PE7/GPIO17/GPIO8/GPIO16), nicht mit Masse oder Versorgung verwechselt ist, und dass die Massen von Empfänger und Board verbunden sind. Wenn die Belegung stimmt und die Leitungen heil sind, aber trotzdem kein Signal kommt, prüfen Sie, ob der Empfänger mit dem Sender gebunden ist (bei ExpressLRS — dieselbe Binding Phrase und Version wie das Sendemodul) und ob sein Ausgang zu `RX_PROTOCOL` passt: iBUS, SBUS oder CRSF, nicht PPM.

**Ein Sensor (IMU, Barometer, Kompass) zeigt „antwortet nicht“ / NO_RESPONSE**
Die Firmware meldet ehrlich, dass der Sensor nicht antwortet, statt Nullen auszugeben. Prüfen Sie: (1) die Versorgung des Moduls – 3.3V und Masse des Boards; (2) SDA/SCL – an den I2C-Pins genau Ihres Boards; (3) die Adresse auf der Leitung: MPU 0x68 (AD0 an GND), BMP581 0x46 (SDO an GND, CSB an 3.3V; 0x47, wenn SDO an 3.3V liegt), BMP388 0x76 (SDO an GND, **CSB an 3.3V** – sonst befindet sich der Chip im SPI-Modus), QMC5883P 0x2C, QMC5883L 0x0D. Antwortet der Sensor mal ja, mal nein (oder meldet er sich unter einer fremden Adresse), ist es ein Wackelkontakt auf dem Steckbrett: Drücken Sie VCC/GND/SDA/SCL fest, und versorgen Sie jedes Modul am besten direkt aus 3.3V/GND des Boards. Der Befehl `s` in der Konsole zeigt die I2C-Fehlerzähler für jeden Sensor.

**Die Winkel auf dem OLED sind vertauscht (Nase hoch ändert R statt P) oder haben das falsche Vorzeichen**
Führen Sie die Kalibrierung des IMU-Einbaus durch (`o`, siehe „IMU-Einbau“) – sie hängt nicht davon ab, wie der Chip auf dem Modul verlötet ist und wie das Modul im Flugzeug sitzt. Ohne sie: Bei Klonen des GY-521 ist der Chip manchmal gegenüber den aufgedruckten Pfeilen gedreht verlötet – drehen Sie die Achsen in `Config.h` → `IMU_ROTATION_CW_DEG` (0/90/180/270). Kontrolle: Nase hoch → P wird positiv, rechter Flügel nach unten → R wird positiv.

**ARM abgelehnt: „IMU: ...“**
Die IMU-Prüfung vor dem Flug wurde nicht bestanden (siehe „Prüfung vor dem Flug beim Einschalten“): Das Flugzeug wurde beim Einschalten bewegt – ruhig hinstellen und den Akku neu anstecken; „oben“ stimmt nicht mit der Kalibrierung überein – die Platine wurde umgesetzt, führen Sie `o` aus; „die Platine liegt nicht mit dem Chip nach oben“ – der Einbau ist nicht kalibriert, führen Sie `o` aus.

**ARM abgelehnt: „нет калибровки шасси“ (keine Fahrwerkskalibrierung)**
AUTO_TAKEOFF mit Spornrad weiß nicht, wo das Heck die Piste berühren würde. Führen Sie die Bodenkalibrierung des Flugzeugs durch: SwB dreimal runter und hoch ohne ARM (oder `c`).

**„Автовзлёт: старт отменён — тангаж не стояночный“ (Startabbruch vor dem Start – Nick ist nicht die Standlage; die Ruder haben dreimal genickt)**
Das Flugzeug steht nicht so wie bei der Kalibrierung: nicht auf drei Rädern, Hang, gebrochenes Spornrad oder verschobene Platine. Stellen Sie es eben hin; wurde die Platine versetzt, `o` und die Bodenkalibrierung wiederholen.

**Ein Servo oder der ESC reagiert auf den falschen Knüppel / reagiert nicht**
Der Befehl `p` in der Konsole misst den tatsächlichen Impuls an jedem Ausgang (GPIO4–7) und vergleicht ihn mit dem erwarteten. Ist alles „OK“ und das Servo bewegt sich trotzdem nicht, liegt das Problem hinter dem Board: (1) das Servo ist nicht versorgt (BEC/5V, gemeinsame Masse); (2) die Signalleitung hängt am falschen Pin; (3) die Mechanik klemmt. „НЕ СОВПАДАЕТ“ („STIMMT NICHT ÜBEREIN“) bedeutet, dass das Problem in der Firmware oder der Peripherie liegt – melden Sie es dem Entwickler.

**Der Motor dreht sich überhaupt nicht, obwohl das Gas auf dem OLED bzw. im Dashboard dem Knüppel folgt**
Prüfen Sie, dass der LiPo-Akku am ESC angeschlossen ist und dass das Flugzeug wirklich scharfgeschaltet ist – bis zum ARM wird das Gas zum ESC zwangsweise auf dem Minimum gehalten, das ist kein Defekt. Ein ESC, der beim Einschalten kein minimales Gas gesehen hat, piept womöglich dauerhaft und schaltet sich nicht scharf – stecken Sie den Akku mit Gas unten neu an. Erscheint nach dem Absenken von SwA kein `ArmingManager: ARM`, schauen Sie in Serial – die Firmware gibt den Grund aus (Gas nicht unten, ein Sensor antwortet nicht, ein Sensor, den der aktuell auf CH7 gewählte Modus braucht); bringen Sie SwA zurück nach oben, beseitigen Sie die Ursache und stellen Sie ihn erneut nach unten.

**Der Motor würgt ab oder ruckelt beim Hochlaufen**
Wird der ESC aus einem Labornetzteil gespeist, stößt man an die Strombegrenzung des Netzteils: Selbst ohne Propeller zieht der Motor kurzzeitig mehrere Ampere, die Spannung bricht ein und der ESC startet neu. Erhöhen Sie das Stromlimit oder verwenden Sie einen LiPo-Akku. Die Störspitzen eines solchen Neustarts können die USB-Bridge des Boards aufhängen (der Port „COM“ lässt sich nicht mehr öffnen) – stecken Sie das Kabel neu an.

**Nach dem Flashen meldet sich das Board nicht / der Zugangspunkt `OpenPlane-Debug` erscheint nicht**
Vergewissern Sie sich, dass der Upload (`pio run -e <Ihr Board> -t upload`) ohne Fehler durchgelaufen ist und dass Sie genau die Umgebung geflasht haben, die Sie physisch in der Hand halten (der esp32-c3 unterscheidet sich vom esp32-s3 und vom esp32-dev nicht nur in den Pins, sondern auch im Chip – eine Firmware für einen fremden Chip lässt sich auf dem Board nicht installieren oder wird fehlerhaft installiert). Prüfen Sie die Ausgabe des seriellen Monitors (`pio device monitor -b 115200`) gleich nach dem Neustart des Boards – dort wird ausgegeben, in welcher Phase von setup() sich das Board befindet.

---

## Aktueller Stand der Prototyp-Flugzeugzelle

Damit die Erwartungen ehrlich bleiben:

- Der erste Prototyp **ist bereits geflogen**. Dabei wurden Probleme festgestellt: **unzureichende Festigkeit der Motorbefestigung** und **unzureichende Festigkeit der Tragfläche** – die Tragfläche braucht eine Carbonverstärkung. Außerdem müssen die Servos weiter abgestimmt werden. Berücksichtigen Sie das bei der Planung Ihrer eigenen Flüge – das ist kein abstrakter Vorbehalt, sondern ein realer Ausfall, der an diesem Prototyp bereits aufgetreten ist.
- **Der Prüfstand mit dem esp32-s3 ist mit allen Sensoren aufgebaut** (GY-521 mit MPU6500, BMP388 über I2C, GY-273 mit QMC5883P, OLED) – alle antworten, die Schleife läuft mit 500 Hz. Die Autopilot-Modi wurden am Tisch geprüft, aber **noch nicht im Flug erprobt**.
- Standardbarometer ist jetzt der BMP581 (der BMP388 des Prüfstands ist live geprüft, der BMP581 auf Hardware noch nicht). Die Höhe ist relativ zum Einschaltpunkt. Die absolute Höhe über dem Meer ergibt sich aus der Standardatmosphäre, ohne Wetterkorrektur.
- Der Kompass QMC5883P muss (mit `m` in der Konsole) am bereits zusammengebauten Flugzeug kalibriert werden – neben Motor und Leitungen sind die Offsets anders als auf dem Steckbrett. Der Kurs hat noch keine Neigungskompensation und wird von keinem Modus verwendet.
- Die Lizenz ist die OpenPlane License: MIT mit verpflichtender Nennung des Autors (Damir Lebedev), Verbot der militärischen Nutzung und Verbot, Menschen und Sachen ohne deren Einwilligung vorsätzlich zu schaden, siehe [LICENSE](LICENSE.md).

Wenn Sie Ihr eigenes Fluggerät nach dieser Anleitung bauen, fliegen Sie es zuerst unter manueller Steuerung (MANUAL) und gehen Sie erst danach zum Autopilot über.
