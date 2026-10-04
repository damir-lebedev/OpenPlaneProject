# DEVELOPER_GUIDE.md – Entwicklerhandbuch zu OpenPlaneProject

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../DEVELOPER_GUIDE.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

Eine technische Karte der Firmware: welche Datei wofür zuständig ist, wie die Daten vom Empfänger und von den Sensoren zu den Servos fließen, welche Vorzeichenkonventionen die ganze Kette zusammenhalten, wie die Web-API aufgebaut ist und wie man das Projekt erweitert. Sie ist für Entwickler gedacht, die C++ schreiben und sich in diesem Repository (Branch `main`) schnell zurechtfinden wollen – nicht zum Erlernen der Grundlagen der Sprache oder von PlatformIO.

Einen Überblick über das Projekt und den Stand des Prototyps gibt [`../README.md`](README.md), was wo anzuschließen ist und wie man fliegt, steht in [`PILOT_GUIDE.md`](PILOT_GUIDE.md), die Pläne in [`ROADMAP.md`](ROADMAP.md). Hier geht es nur um Code. Die gesamte Architektur (Schichten, Tasks, Zustandsautomaten) steht in [`ARCHITECTURE.md`](ARCHITECTURE.md), die Referenz zu jeder Klasse in [`reference/`](reference/README.md), die Tests in [`TESTING.md`](TESTING.md).

> Das Projekt befindet sich in aktiver Entwicklung. Der ESP32-S3-Prüfstand ist aufgebaut und mit allen Sensoren geprüft, aber **der Autopilot ist im Flug noch nicht erprobt** – das ist dort vermerkt, wo es ein konkretes Modul betrifft. Wenn Sie bezweifeln, was der Code tut, lesen Sie den Quelltext noch einmal, nicht das Dokument.

---

## Inhalt

1. [Architektur der Schichten](#architektur-der-schichten)
2. [FreeRTOS-Tasks und Regelschleife](#freertos-tasks-und-regelschleife)
3. [Dateireferenz](#dateireferenz)
4. [Vorzeichenkonvention: von der IMU bis zum Servo](#vorzeichenkonvention-von-der-imu-bis-zum-servo)
5. [Karte der RC-Kanäle, ARM und Failsafe](#karte-der-rc-kanäle-arm-und-failsafe)
6. [Sensordaten](#sensordaten)
7. [Aufschlüsselung von FlightController::update()](#aufschlüsselung-von-flightcontrollerupdate)
8. [HTTP-API des Web-Dashboards](#http-api-des-web-dashboards)
9. [Konsole und Diagnose](#konsole-und-diagnose)
10. [Boardauswahl und Pinbelegung](#boardauswahl-und-pinbelegung)
11. [Wie man einen neuen Sensor hinzufügt](#wie-man-einen-neuen-sensor-hinzufügt)
12. [Wie man einen neuen Autopilot-Modus hinzufügt](#wie-man-einen-neuen-autopilot-modus-hinzufügt)
13. [Rückführung (Vorarbeit, nicht angeschlossen)](#rückführung-vorarbeit-nicht-angeschlossen)
14. [Wie man ein neues Board hinzufügt](#wie-man-ein-neues-board-hinzufügt)
15. [Befehle zum Bauen, Aufspielen und Überwachen](#befehle-zum-bauen-aufspielen-und-überwachen)
16. [Bekannte Einschränkungen](#bekannte-einschränkungen)
17. [Wie man Änderungen vornimmt](#wie-man-änderungen-vornimmt)

---

## Architektur der Schichten

Fast alle Klassen leben in Headern, die in den Ordnern `include/<Schicht>/` liegen. Jeder Header bindet selbst ein, was er benutzt (`#include "config/Config.h"`, `"hal/II2CBus.h"`, ... – Pfade ab `include/`). `src/main.cpp` ist der einzige Zusammenbaupunkt (Composition Root): Er erzeugt alle Objekte, verknüpft sie und führt `setup()`/`loop()` aus. Die Abhängigkeiten sind einseitig – eine untere Schicht weiß nichts von einer oberen.

```
include/
├── config/      Config.h (Pins, alle Einstellungen), Channels.h (Kanalnamen),
│                Controls.h (was jeder Schalter tut – eine Zeile je Kanal)
├── hal/         IBoard, II2CBus, ISpiBus, IUartPort, IServoOutput,
│   │            RegisterDevice (ein Registergerät über I2C/SPI), Rtos
│   ├── esp32/   Esp32Board + Hüllen um Wire/SPI/HardwareSerial/LEDC
│   └── stm32/   Stm32Board + Wire/SPI/Uart/HardwareTimer, Stm32FlashStorage,
│                compat/Preferences.h (Einstellungen im Flash statt NVS)
├── storage/     KeyValueStore, KvPreferences — Einstellungsspeicher ohne NVS
├── rc/          RcChannelState, RcInput, IBusReceiver
├── control/     ControlCommand, ControlMixer, FlapsController,
│                ThrottleManager, ArmingManager, FlightOutputState,
│                FlightOutputs, Beeper, FlightController
├── autopilot/   AutopilotTypes, ControlBinding, PilotSwitches, Autopilot,
│   │            Navigation, AltitudeSpeedController, LaunchController,
│   │            SoaringController, AutoTrim, PidController
│   └── feedback/  Vorarbeit zur Rückführung – NICHT angeschlossen (siehe den Abschnitt unten)
├── sensors/     SensorInterface, SensorSelection, SensorMounting
│   ├── imu/     ImuSensorBase, AttitudeEstimator, MPU6050, ICM42688, LSM6DSV, ICM45686
│   ├── baro/    BarometerBase, BMP388, BME280, SPL06, BMP581
│   ├── mag/     MagnetometerBase, QMC5883P, QMC5883L, QMC6309
│   ├── gps/     UbloxM10_Gps
│   └── airspeed/ AirspeedSensor, PitotDualBaroAirspeed (ein Rohr aus zwei Barometern)
└── telemetry/   DebugLogger, DebugConsole, WebDebugServer, WebDashboardPage,
                 OledDisplay, LoopStats, MavlinkCodec, MavlinkTelemetry
src/main.cpp        — ESP32-Firmware (S3, C3, 38-Pin)
src/stm32/main.cpp  — STM32H743-Firmware (FreeRTOS-Tasks, MAVLink)
```

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION  src/main.cpp / src/stm32/main.cpp — Objekt-Zusammenbau   │
└──────────────────────────────┬────────────────────────────────────────┘
                               ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION  control/FlightController — Reihenfolge der Operationen   │
│ TELEMETRY     DebugLogger, DebugConsole, Web (ESP32) / MAVLink, OLED   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        ▼                       ▼                       ▼
┌────────────────────┐  ┌──────────────────────┐  ┌──────────────────────┐
│ CONTROL             │  │ AUTOPILOT             │  │ RC                    │
│ ControlMixer        │  │ Autopilot: 12 Modi    │  │ IBusReceiver          │
│  └ FlapsController  │  │  └ Navigation, PID    │  │ RcChannelState        │
│ ThrottleManager     │  │ PilotSwitches         │  │ RcInput               │
│ ArmingManager       │  └──────────┬────────────┘  └───────────────────────┘
│ FlightOutputs       │             │ ImuSensor* / BarometerSensor* / ...
└─────────┬───────────┘             ▼
          │           ┌─────────────────────────────────────────────────┐
          │           │ SENSORS                                          │
          │           │ ImuSensorBase ── MPU6050, ICM42688, LSM6DSV,     │
          │           │   └ AttitudeEstimator     ICM45686               │
          │           │ BarometerBase ── BMP388, BME280, SPL06, BMP581   │
          │           │ MagnetometerBase ── QMC5883P / L, QMC6309        │
          │           │ UbloxM10_Gps, PitotDualBaroAirspeed              │
          │           └──────────────────────┬──────────────────────────┘
          ▼                                  ▼ IRegisterDevice / IUartPort
┌───────────────────────────────────────────────────────────────────────┐
│ HAL  IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput             │
│      RegisterDevice: I2cRegisterDevice, SpiRegisterDevice              │
│      esp32/Esp32Board — Wire, Wire1, SPI, HardwareSerial, LEDC         │
│      stm32/Stm32Board — Wire, I2C1, SPI, Uart, HardwareTimer, Flash    │
└───────────────────────────────────────────────────────────────────────┘
```

Die Regeln, die die Architektur sauber halten:

- Die **HAL** ist die einzige Schicht, die eine konkrete MCU kennen darf (`Wire`, `SPI`, `HardwareSerial`, `ledc*`). Alles darüber arbeitet nur mit Schnittstellen. Der Wechsel auf eine andere MCU bedeutet ein neues `hal/<mcu>/<Mcu>Board.h`; der übrige Code ändert sich nicht (ein Beispiel ist `hal/stm32/` für den STM32H743).
- **Sensortreiber kennen den Bus nicht.** Sie erhalten ein `IRegisterDevice&` – das I2C-Gerät mit Adresse oder das SPI-Gerät mit CS wird in `SensorSelection.h` erzeugt. Derselbe `BMP388_Sensor` arbeitet sowohl über I2C als auch über SPI.
- **Gemeinsames liegt in den Basisklassen.** Kalibrierung, Achsendrehung, Vorzeichen, Orientierungsfilter, Höhe und Vertikalgeschwindigkeit, Speicherung der Kompasskalibrierung, Zählen der Busfehler – in `ImuSensorBase`/`BarometerBase`/`MagnetometerBase`. Ein Chip-Treiber enthält nur die Register und die Formeln aus dem Datenblatt.
- **RC und Outputs** wissen nichts vom Flugzeug: iBUS-Bytes → Kanäle, PWM-Werte → Ausgänge.
- **Control und Autopilot** sind Logik über Daten, ohne UART, PWM oder WLAN. Die Zeit wird dort, wo sie gebraucht wird (Klappen), als Parameter übergeben.
- **Coordination** (`FlightController`) ist die einzige Klasse, die mehrere untere Schichten gleichzeitig sieht und die Reihenfolge der Operationen festlegt.
- **Application** (`main.cpp`) ist der einzige Ort, an dem `Esp32Board`, die Geräte und die Sensoren erzeugt und an dem alles von Hand verdrahtet wird, ohne DI-Framework.

---

## FreeRTOS-Tasks und Regelschleife

| Wo | Was | Periode |
|---|---|---|
| Kern 1, `loop()` (der Arduino-loopTask) | `applyPendingCommands()` → `FlightController::update()` → `DebugLogger::update()` → `DebugConsole::update()` | `Config::LOOP_PERIOD_MS` = 2 ms (500 Hz), `vTaskDelayUntil` |
| Kern 0, der Task `web` | `WebServer::handleClient()` | alle 2 ms |
| Kern 0, der Task `oled` | Zeichnen des SSD1306 über den zweiten I2C-Bus | 200 ms |
| Kern 0 | der WLAN-Stack von ESP-IDF | — |

- Die Periode der Schleife wird durch `vTaskDelayUntil` gehalten, nicht durch ein `delay(2)` nach der Arbeit — die Frequenz hängt nicht davon ab, wie lange der Zyklus dauerte. Nach einer langen Blockade (eine Kalibrierung von der Konsole aus) beginnt die Zählung von vorn, und verpasste Zyklen werden nicht in einem Schwung nachgeholt.
- Auf dem Prüfstand (ESP32-S3, alle Sensoren): 500 Hz, im Mittel ~0,7 ms Arbeit pro Zyklus, der schlechteste Zyklus ~1,4 ms. Das wird alle 10 s als Zeile `SYS:` ausgegeben.
- Das Timeout einer I2C-Transaktion beträgt 5 ms (das Standard-Timeout von Wire ist 50 ms): Eine durch Störungen hängende Transaktion hält die Schleife nicht lange auf.
- **Datentrennung zwischen den Tasks.** Web und OLED *lesen* den Zustand nur (`FlightController`/`Autopilot`/`LoopStats`) — das sind einzelne 16/32-Bit-Felder, im ungünstigsten Fall sind also die Werte benachbarter Zyklen zu sehen. Die *Befehle* des Dashboards (`setmode`/`setpid`) werden nicht direkt aus dem Web-Task angewendet: Sie werden unter `portMUX` in ein „Postfach“ gelegt und von der Flugschleife in `WebDebugServer::applyPendingCommands()` abgeholt.
- `Serial` (UART0 → die CH343-Brücke → der Anschluss „COM“) mit 4-KB-Sendepuffer: Ein Debug-Frame (~600 Zeichen) blockiert die Schleife während des Sendens nicht.

---

## Dateireferenz

### `config/`

| Datei | Zuständig für |
|---|---|
| `Config.h` | Alle Pins (ein Block pro Platine: `BOARD_ESP32_S3/C3/CLASSIC`, `BOARD_STM32H743`) und Einstellungen: iBUS und Signalverlust; Ruderausschläge; Klappen; Servo-Umkehr; Einbau von IMU und Kompass; ARM; Failsafe (RTH oder Gleiten); das Pitotrohr (`PITOT_*`); alle Zahlenwerte der Autopilot-Modi und -Funktionen; der Zyklus; WLAN; MAVLink; Debugging |
| `Channels.h` | Kanalnamen: `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `ARM`, `SWB`, `SWC`, `SWD`, `VRA`, `VRB` |
| `Controls.h` | Die Tabelle `BINDINGS`: was jeder Schalter und jeder Drehregler tut, eine Zeile pro Kanal, Prüfungen per `static_assert` |

### `hal/`

| Datei | Zuständig für |
|---|---|
| `IBoard.h` | Der Einstiegspunkt zur Hardware: `i2c()`, `displayI2c()` (ein zweiter Bus für das Display, kann `nullptr` sein), `spi()`, `rcUart()`, `gpsUart()`, `telemetryUart()` (MAVLink, kann `nullptr` sein), `servo(ServoChannel::*)` (7 Ausgänge mit AUX1/AUX2), `setBuzzer()` |
| `Rtos.h` | Die FreeRTOS-Tasks, auf dem ESP32 (Kern 0) und auf dem STM32 (Prioritäten) auf dieselbe Weise, der freie Heap |
| `II2CBus.h` | Der I2C-Bus: Grundoperationen in der Form von `Wire` + die Hilfsfunktionen `writeRegister()`, `readRegisters()` (prüft, dass genau `count` Bytes angekommen sind), `readRegister()`, `probe()` |
| `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | SPI, UART, ein PWM-Ausgang (`measurePulseUs()` — Diagnose des tatsächlichen Impulses) |
| `RegisterDevice.h` | `IRegisterDevice` — „ein Satz 8-Bit-Register“; `I2cRegisterDevice` (Adresse), `SpiRegisterDevice` (CS, Frequenz, Dummy-Bytes vor den Daten) |
| `esp32/Esp32Board.h` | Die Implementierung von `IBoard`: `Wire` (Sensoren), `Wire1` (das Display, wenn der Chip zwei I2C-Controller hat), `SPI`, zwei `HardwareSerial`, 5 LEDC-Kanäle |
| `esp32/Esp32I2CBus.h` | `II2CBus` über jedem beliebigen `TwoWire`, Timeout 5 ms |
| `esp32/Esp32ServoOutput.h` | PWM über LEDC: 50 Hz, 14 Bit; Pin −1 — der Ausgang ist nicht herausgeführt. Die Bibliothek ESP32Servo wird nicht verwendet — siehe [Einschränkungen](#bekannte-einschränkungen) |
| `esp32/Esp32SpiBus.h`, `esp32/Esp32UartPort.h` | Dünne Wrapper über `SPI` und `HardwareSerial` |
| `stm32/*` | STM32H743: `Stm32Board` (+ UART4 des Funkmodems), die Busse, PWM-Timer, `Stm32FlashStorage` (Einstellungen in einem Flash-Sektor, von einem Hintergrund-Task geschrieben), `compat/Preferences.h` |

### `storage/`

| Datei | Zuständig für |
|---|---|
| `KeyValueStore.h` | Ein Abbild „Namensraum/Schlüssel → Bytes“ mit CRC32 im RAM über beliebigem Speichermedium (`IFlashStorage`); ein identischer Wert wird nicht neu geschrieben |
| `KvPreferences.h` | Die `Preferences`-API des ESP32 über `KeyValueStore` |

### `rc/`

| Datei | Zuständig für |
|---|---|
| `RcChannelState.h` | Eine Momentaufnahme der 10 Kanäle |
| `RcInput.h` | `clamp()`, `centered(us, max, reverse)` |
| `IBusReceiver.h` | iBUS → Kanäle: 32-Byte-Frame, CRC, der Kanalwert sind die unteren 12 Bit (`& 0x0FFF`); `isSignalLost()` = keine Frames (oder noch gar keiner) ∥ der Failsafe-Wert des Gases; Frame-Zähler |

### `control/`

| Datei | Zuständig für |
|---|---|
| `ControlCommand.h` | Das Kommando an die Ruder in physikalischen Vorzeichen — die gemeinsame Sprache von Knüppeln, Autopilot und Mischer |
| `ControlMixer.h` | `fromSticks(rc)` → `ControlCommand`; `updateFlaps(Ziel, now)`; `mix(command)` → PWM mit Servo-Umkehr; Flaperons: die Querruder `flaps ± roll` (das Minus ist eine Bremsklappe) |
| `FlapsController.h` | Sanftes Aus- und Einfahren der Klappen, die Zeit wird als Parameter übergeben |
| `ThrottleManager.h` | Das Gas vom Knüppel; bei Signalverlust — `FAILSAFE_THROTTLE` |
| `ArmingManager.h` | ARM über den Schalter SwA (ein Wechsel OFF→ON bei Gas unten + die Sensorprüfungen des Modus), DISARM sofort |
| `FlightOutputState.h` | Die gewünschten PWM-Werte: `aileronLeft`, `aileronRight`, `elevator`, `rudder`, `throttle`, `aux1` (Nutzlast), `aux2` (Kamera) |
| `Beeper.h` | Der Summer: über die Funktion `BEEPER` oder „Modell verloren“ am Boden |
| `FlightOutputs.h` | Die Tabelle der Ausgänge (`outputInfo()`: Schlüssel, Name, Pin, Pflichtangabe, Statusfeld) und alles, was in einer Schleife darübergelegt ist: `begin()`, `write()`, `setFailsafe()`, Status, `printPulseSelfTest()` |
| `FlightController.h` | Die Reihenfolge der Operationen in jedem Zyklus, der Signalverlust (`applyLinkLoss()`), Getter für die Telemetrie |

### `autopilot/`

| Datei | Zuständig für |
|---|---|
| `AutopilotTypes.h` | `AutopilotMode` (12 Modi), `Feature`, `Knob`, `PilotInputs`, Namen |
| `ControlBinding.h` | `Binding`, die Fabriken `Bind::modes/mode/feature/knob`, die Prüfungen `BindingCheck` |
| `PilotSwitches.h` | Die Zuordnungstabelle → der Modus, die Funktionen und die Drehregler jedes Zyklus; die Belegung beim Einschalten |
| `Autopilot.h` | 12 Modi, Failsafe RTH/Gleiten, Geofence, Startpunkt, Kurvenkoordination, Autotrimmung; `update(armed, linkLost, Gas, Knüppel)` → `getCommand()`, `applyThrottle()` |
| `Navigation.h` | `Geo` (Entfernung, Peilung, Versatz), `Guidance` (Rollen auf einen Kurs, das Vektorfeld des Kreises) |
| `AltitudeSpeedController.h` | Nicken für die Höhe, Gas für die Fluggeschwindigkeit (TECS-lite) |
| `LaunchController.h`, `SoaringController.h` | Die Zustandsautomaten für den Handstart und den Segelflug |
| `AutoTrim.h` | Autotrimmung, gespeichert im NVS/Flash |
| `PidController.h` | PID: D auf der Rate des Sensors (Gyroskop, Variometer), Anti-Windup, Integrator ohne ARM eingefroren |
| `feedback/*` | **Vorarbeit, nicht angeschlossen:** adaptive Rückführung, Start und Landung — siehe [Rückführung](#rückführung-vorarbeit-nicht-angeschlossen) |

### `sensors/`

| Datei | Zuständig für |
|---|---|
| `SensorInterface.h` | Die Schnittstellen `Sensor`/`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` und die Datenstrukturen |
| `SensorSelection.h` | Welcher Chip einkompiliert wird (`#define SENSOR_*`, per Build-Flag überschreibbar) und an welchem Bus er hängt (`SELECTED_*_DEVICE(board)`) |
| `SensorMounting.h` | Drehung der Chip-Achsen in die Flugzeugachsen (0/90/180/270° im Uhrzeigersinn) — für den Kompass und für eine IMU ohne Einbaukalibrierung |
| `imu/ImuOrientation.h` | Einbau der IMU als Matrix „Chip-Achsen → Flugzeugachsen“: aus `IMU_ROTATION_CW_DEG` oder aus drei Lagen (waagerecht, Nase hoch, rechter Flügel nach unten) mit Plausibilitätsprüfung; im NVS gespeichert |
| `imu/ImuSensorBase.h` | Das Gemeinsame aller IMUs: Gyro-Kalibrierung + Prüfung vor dem Flug (Ruhe, 1g, „oben“ passt zum Einbau), Einbaukalibrierung (`calibrateOrientation()`), Skalierung, Drehung, Luftfahrt-Vorzeichen, Busfehler |
| `imu/AttitudeEstimator.h` | Komplementärfilter für Rollen/Nicken, Integral des Gierens |
| `imu/MPU6050_Sensor.h` | MPU6050/MPU6500 (Chip über WHO_AM_I erkannt): ±2000°/s, ±16g, DLPF ~41 Hz, 1 kHz. **Auf dem Prüfstand** |
| `imu/ICM42688_Sensor.h` | ICM-42688-P: ±2000°/s, ±16g, 1 kHz, UI-Filter 50 Hz. Nicht an der Hardware getestet |
| `imu/LSM6DSV_Sensor.h` | LSM6DSV/16X/32X: ±2000°/s, ±16g, 960 Hz, LPF1/LPF2; I2C 0x6A/0x6B oder SPI. Nicht an der Hardware getestet |
| `imu/ICM45686_Sensor.h` | ICM-45686: ±2000°/s, ±16g, 1,6 kHz, Tiefpassfilter über die indirekten IPREG-Register; I2C 0x68/0x69 oder SPI. Nicht an der Hardware getestet |
| `baro/BarometerBase.h` | Das Gemeinsame aller Barometer: Abfrage nur neuer Messwerte, Höhe, Vertikalgeschwindigkeit über einen Tiefpassfilter, Kalibrierung der Basis, Fehler |
| `baro/BMP388_Sensor.h` | BMP388 über I2C oder SPI (mit dem SPI-Dummy-Byte), Bosch-Kompensation, Auslesen über das Bereitschafts-Flag. **Auf dem Prüfstand (I2C)** |
| `baro/BME280_Sensor.h` | BME280/BMP280, Bosch-Kompensation §8.1. Nicht an der Hardware getestet |
| `baro/SPL06_Sensor.h` | SPL06-001: Koeffizienten und Formeln aus dem Datenblatt, 32 Hz ×16; I2C 0x76/0x77 oder SPI. Nicht an der Hardware getestet |
| `baro/BMP581_Sensor.h` | BMP581: die Abfolge der BMP5_SensorAPI, 16×/2×, IIR; I2C 0x46/0x47 oder SPI; dient sowohl als Haupt-Barometer als auch als Pitotrohr. Nicht an der Hardware getestet |
| `mag/MagnetometerBase.h` | Das Gemeinsame aller Kompasse: Abfrage mit 50 Hz, Hard-Iron-Kalibrierung im NVS, Achsendrehung, Kurs, Fehler |
| `mag/QMC5883P_Sensor.h` | QMC5883P, 0x2C. **Auf dem Prüfstand** |
| `mag/QMC5883L_Sensor.h` | QMC5883L, 0x0D |
| `mag/QMC6309_Sensor.h` | QMC6309, 0x7C: ±8 G, 200 Hz. Nicht an der Hardware getestet |
| `gps/UbloxM10_Gps.h` | u-blox M10: Konfiguration über CFG-VALSET (115200 Baud, 10 Hz, NAV-PVT, ohne NMEA), Auswertung von NAV-PVT. Auf dem Prüfstand nicht angeschlossen |
| `airspeed/AirspeedSensor.h` | Die Schnittstelle des Fluggeschwindigkeitssensors: Differenzdruck, IAS, TAS, Dichte |
| `airspeed/PitotDualBaroAirspeed.h` | Das selbstgebaute Pitotrohr: ein BMP581 im Rohr + ein Barometer im Rumpf; Nullpunkt am Boden, Tiefpassfilter, Dichte aus dem statischen Druck, Fehlererkennung |

### `telemetry/` und die Anwendung

| Datei | Zuständig für |
|---|---|
| `DebugLogger.h` | Log nach Kanälen (`LogSettings.h`): jeder hat seine eigene Zeile, seine eigene Entprelltoleranz und seinen Modus; schweigt, solange das Menü geöffnet ist |
| `DebugConsole.h` | Ein Textmenü im Port-Monitor (`h`) und Hotkeys (`l`/Leertaste/`s`/`i`/`o`/`m`/`p`/`b`); schreibt die Log-Einstellungen beim Verlassen des Menüs ins NVS und nur ohne ARM |
| `LogSettings.h` | Die Log-Kanäle (STAT, RC, OUT, ATT, AP, ALT, MAG, GPS, IMU, NAV, SYS) und ihre Modi: aus / bei Änderung / dauerhaft; im NVS gespeichert |
| `WebDebugServer.h` | Der Access Point, die Routen, das JSON `/api/status`, der Befehlsbriefkasten; eine eigene Task auf Kern 0 |
| `WebDashboardPage.h` | HTML/JS des Dashboards als ein einziges Literal; die Zeilen für Kanäle, Ausgänge und Sensoren baut der Browser aus dem JSON |
| `OledDisplay.h` | SSD1306 über U8g2 auf `II2CBus`, eine eigene Task (`Rtos`) |
| `MavlinkCodec.h`, `MavlinkTelemetry.h` | MAVLink 2 für QGroundControl / Mission Planner: Frames, Streams, PID-Parameter, Moduswechsel vom Boden aus |
| `LoopStats.h` | Frequenz, mittlere und schlechteste Zykluszeit pro Sekunde (OLED) und die schlechteste seit dem letzten Auslesen (`takePeakUs()`, Zeile SYS) |
| `src/main.cpp` | ESP32: Erzeugen der Objekte, `setup()`, `loop()` mit `vTaskDelayUntil` |
| `src/stm32/main.cpp` | STM32H743: dieselben Objekte, MAVLink, die Blackbox auf der SD-Karte, die Tasks `flight`/`storage`/`oled`/`bbox` |
| `src/stm32/sd_msp.cpp`, `src/stm32/bootloader.cpp` | STM32H743: Pins und Takte von SDMMC1 für `HAL_SD_Init`; die Konsolentaste `D` — Neustart in den USB-DFU-Bootloader |

---

## Vorzeichenkonvention: von der IMU bis zum Servo

Ein einziges Vorzeichensystem für die ganze Kette — deshalb bewegen Knüppel
und Autopilot die Ruder garantiert in dieselbe Richtung, und die Richtung
jedes Servos wird an genau einer Stelle festgelegt.

**1. Sensorachsen → Flugzeugachsen.** `ImuSensorBase` dreht die Chipachsen
mit der Matrix `ImuOrientation` (body = R · chip) in die Flugzeugachsen: X zur
Nase, Y nach links, Z nach oben. Die Matrix stammt:

- aus der **Einbaukalibrierung** (Befehl `o`, im NVS gespeichert) — die
  Platine kann beliebig liegen. Drei Lagen: „waagerecht“ liefert die Z-Achse
  (und den Horizont — der Nullpunktversatz des Beschleunigungssensors steckt
  darin), „Nase hoch“ liefert die X-Achse (den zu Z senkrechten Teil von
  „oben“), „rechter Flügel nach unten“ liefert die Y-Achse. Die Nase aus
  Schritt 2 und die Nase aus Schritt 3 (Y × Z) müssen auf ~25° genau
  übereinstimmen, sonst hat der Pilot in die falsche Richtung gekippt — die
  Kalibrierung wird abgelehnt; das Ergebnis ist der Mittelwert beider
  Schätzungen. Geprüft an 300 zufälligen Einbaulagen
  (`test/test_imu_orientation`, Fehler < 0,1°);
- andernfalls aus `Config::IMU_ROTATION_CW_DEG` (Platine mit dem Chip nach
  oben; der Wert gibt an, wohin die X-Achse des *Chips* zeigt, wenn die Nase
  auf „12 Uhr“ steht), und der Horizont ist die Lage beim Einschalten.

Bei jeder Gyro-Kalibrierung (Einschalten, `i`) erfolgt eine **Prüfung vor dem
Flug**: Gyro-Rauschen < 0,5 °/s (Ruhe; in Ruhe ~0,08), |a| ≈ 1g, „oben“
innerhalb von 45° des gespeicherten Wertes (die Platine wurde nicht
umgesetzt). Wird sie nicht bestanden — `ImuSensor::getPreflightProblem()` ≠
nullptr: `ArmingManager` armt keine stabilisierten Modi, und
`Autopilot::imuReady()` = false (Null-Korrekturen in allen Modi, auch beim
Gleiten bei Verbindungsverlust).

> Beim aktuellen GY-521 (ein Klon mit MPU6500) ist der Chip um 90° gedreht
> gegenüber den aufgedruckten Pfeilen eingelötet: der X-Pfeil im Bestückungsdruck
> = die Y-Achse des Chips. Deshalb gilt ohne Einbaukalibrierung
> `IMU_ROTATION_CW_DEG = 90`. Prüfung nach jedem Umsetzen: Nase hoch → P wächst
> ins Positive, rechter Flügel nach unten → R ins Positive.

**2. Winkel und Raten (`ImuData`) — Luftfahrt-Vorzeichen:**

| Größe | „+“ bedeutet |
|---|---|
| `roll`, `gyroX` | rechter Flügel nach unten |
| `pitch`, `gyroY` | Nase hoch |
| `yaw`, `gyroZ` | Nase nach rechts (von oben gesehen im Uhrzeigersinn) |

**3. Das Kommando (`ControlCommand`, µs Ausschlag, ±500 = voller Weg):**

| Feld | „+“ bedeutet | Vom Knüppel |
|---|---|---|
| `roll` | Rollen nach rechts (rechtes Querruder hoch, linkes runter) | CH1: 2000 = nach rechts |
| `pitch` | Nase hoch (Höhenruder hoch) | CH2 mit umgekehrtem Vorzeichen: 2000 = von sich weg = Nase runter |
| `yaw` | Nase nach rechts (Seitenruder und Bugrad nach rechts) | CH4: 2000 = nach rechts |
| `flaps` | Klappen nach unten (beide Querruder nach unten) | SwB (CH6): 0 oder `FLAPS_DEPLOYED_US`, sanft innerhalb von `FLAPS_TRANSITION_MS` |

Der PID berechnet `Fehler = Ziel − Ist`: Rollen nach rechts (roll > 0) →
negatives Rollkommando → das Flugzeug richtet sich auf. Die
Autopilot-Korrekturen werden **vor** dem Mischer zum Knüppelkommando addiert,
in denselben Vorzeichen.

**4. Kommando → PWM.** `ControlMixer::mix()` berechnet den Ausschlag der
Hinterkante jeder Fläche (Querruder: runter = „+“, links = `flaps + roll`,
rechts = `flaps − roll`; Höhenruder: hoch = „+“; Seitenruder: rechts = „+“)
und wandelt ihn in PWM `1500 ± Ausschlag` um, wobei das Vorzeichen bei Servos
mit `Config::*_REVERSED = true` umgekehrt wird. Die Standardwerte
reproduzieren das frühere Verhalten der Firmware für die Knüppel. Die Prüfung
am fertig aufgebauten Flugzeug steht in der Checkliste vor dem Flug im
[`PILOT_GUIDE.md`](PILOT_GUIDE.md). Die Umkehr muss in `Config.h` geändert
werden, **nicht am Sender** — sonst laufen Knüppel und Autopilot
auseinander.

---

## Karte der RC-Kanäle, ARM und Failsafe

Die Quelle ist `include/config/Channels.h`. Sender FS-i6 (10 Kanäle, Mode 2) +
Empfänger FS-iA6B, iBUS 115200.

| Kanal | Bedienelement am Sender | Name | Zweck |
|---|---|---|---|
| CH1 | rechter Knüppel ←→ | `AILERON` | Rollen |
| CH2 | rechter Knüppel ↑↓ | `ELEVATOR` | Nicken |
| CH3 | linker Knüppel ↑↓ | `THROTTLE` | Gas, voller Weg; < 950 = Failsafe des Empfängers |
| CH4 | linker Knüppel ←→ | `RUDDER` | Seitenruder + Lenkrad (ein Servo) |
| CH5 | SwA | `ARM` | ≥ 1750 = ARM (am FS-i6 ist das der Schalter nach unten, zu sich hin) |
| CH6 | SwB | `SWB` | standardmäßig die Klappen (≥ 1750 — ausgefahren) |
| CH7 | SwC (3 Stellungen) | `SWC` | standardmäßig der Modus: < 1250 MANUAL, 1250–1749 STABILIZE, ≥ 1750 AUTO_TAKEOFF |
| CH8 | SwD | `SWD` | standardmäßig RTH |
| CH9 | VrA | `VRA` | standardmäßig die Stabilisierungsstärke |
| CH10 | VrB | `VRB` | standardmäßig die Reisegeschwindigkeit |

CH6–CH10 werden mit einer einzigen Zeile in `include/config/Controls.h`
zugewiesen ([AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#eine-funktion-mit-einer-zeile-zuweisen)).

**ARM** (`ArmingManager`): der Schalter wechselt von OFF auf ON, das Gas liegt
unter `THROTTLE_LOW_US`, und die Sensorprüfungen für den aktuellen Modus sind
bestanden. Andernfalls wird abgelehnt, mit dem Grund auf Serial; ein neuer
Zyklus OFF→ON ist nötig. Wird die Platine mit dem Schalter in ON
eingeschaltet, armt sie nicht. OFF — sofort DISARM. Solange nicht gearmt ist,
wird das Gas zum ESC zwangsweise auf `PWM_MIN` gesetzt.

**Verbindungsverlust** (`IBusReceiver::isSignalLost()`):

1. Länger als `RX_TIMEOUT_US` (500 ms) keine Frames — Kabelbruch oder
   Stromausfall am Empfänger. Bis zum ersten Frame nach dem Einschalten gilt
   die Verbindung ebenfalls als verloren: die Standardwerte der Kanäle (alle
   1500) werden nicht für Senderbefehle gehalten.
2. Gas < `RX_FAILSAFE_THROTTLE_US` (950) — das im Sender eingestellte
   Failsafe. **Der FS-iA6B hört beim Verlust des Senders nicht auf, Frames zu
   senden**, sondern wiederholt die letzten Werte (am Prüfstand geprüft);
   ohne ein im Sender eingerichtetes Failsafe wird der Verbindungsverlust
   daher nicht erkannt. Die Einrichtung steht im `PILOT_GUIDE.md`.

Was bei Verbindungsverlust geschieht (`FlightController::applyLinkLoss()`):

- **das Flugzeug ist gearmt, GPS und ein Startpunkt sind vorhanden**
  (`FAILSAFE_RTH`) — **Rückkehr zum Startpunkt** mit Motor, über dem
  Startpunkt Kreise; auf dem OLED — `FSRTH`, im Log — `FAILSAFE_RTH`;
- **das Flugzeug ist gearmt, kein GPS** — **Gleiten**, Motor auf
  `FAILSAFE_THROTTLE`: `Autopilot` hält in jedem Modus, selbst in MANUAL, die
  Querlage `FAILSAFE_GLIDE_ROLL_DEG` (0 — geradeaus, 10–20° — ein Kreis über
  dem Piloten) und die Nicklage `FAILSAFE_GLIDE_PITCH_DEG` (−3°, damit ohne
  Motor keine Geschwindigkeit verloren geht), die Klappen sind eingefahren;
  auf dem OLED — `GLIDE`, im Log — der Modus `FAILSAFE_GLIDE`;
- **nicht gearmt** (am Boden) oder die IMU antwortet nicht — Ruder in
  Neutralstellung;
- Modus und Funktionen werden nicht mehr von den Schaltern umgeschaltet, die
  Sensoren werden weiter gelesen. ARM wird nicht zurückgenommen — nach
  Wiederherstellung der Verbindung gehorcht das Flugzeug wieder den Knüppeln
  und dem gewählten Modus (Automatikstart und Handstart nur von vorn).

---

## Sensordaten

Die Strukturen stehen in `include/sensors/SensorInterface.h`.

### `ImuData`

| Feld | Einheit | Bedeutung |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/s | Drehraten in den Flugzeugachsen, Luftfahrt-Vorzeichen (siehe oben) |
| `accelX`, `accelY`, `accelZ` | g | Beschleunigung in den Flugzeugachsen: X zur Nase, Y nach links, Z nach oben |
| `roll`, `pitch` | ° | Komplementärfilter (α = 0,98, τ ≈ 0,1 s); starten sofort mit dem Winkel aus dem Beschleunigungssensor |
| `yaw` | ° | Integral des Gyroskops, driftet langsam; der Anfangswert ist der Kompasskurs |
| `temperature` | °C | Chiptemperatur (Formel für den MPU6050 oder MPU6500) |
| `timestamp` | µs | `micros()` zum Zeitpunkt des Lesens |

Kalibrierung der IMU (bei jedem Start und mit dem Befehl `i`): 2 s ruhig,
Gyroskop → Nullpunktversatz, Beschleunigungssensor → **die aktuelle Lage
wird zum Horizont**.

### `BarometerData`

| Feld | Einheit | Bedeutung |
|---|---|---|
| `pressure` | Pa | Druck |
| `temperature` | °C | Sensortemperatur |
| `altitude` | m | Höhe **relativ zum Kalibrierpunkt** (beim Start); Formel `44330·(1 − (P/P0)^0.1903)` |
| `verticalSpeed` | m/s | Ableitung der Höhe über die echten Messwerte (50 Hz) durch einen Tiefpassfilter mit τ = 0,5 s |
| `timestamp` | µs | Zeitpunkt des letzten neuen Messwerts |

### `MagData`

| Feld | Einheit | Bedeutung |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | Das Feld nach der Hard-Iron-Kalibrierung, in den Flugzeugachsen (`MAG_ROTATION_CW_DEG`) |
| `headingDegrees` | ° (0..360) | `atan2(magY, magX)`, ohne Neigungskompensation; die Zählrichtung wurde am fertig aufgebauten Flugzeug noch nicht überprüft |
| `timestamp` | µs | Zeitpunkt des Lesens (50 Hz) |

### `GpsData`

| Feld | Einheit | Bedeutung |
|---|---|---|
| `latitude`, `longitude` | ° | Aus UBX-NAV-PVT |
| `altitude` | m | Über dem Meeresspiegel (hMSL) |
| `groundSpeed`, `heading` | m/s, ° | Geschwindigkeit und Kurs über Grund |
| `numSatellites`, `fixType` | — | 0 = kein Fix, 2 = 2D, 3 = 3D |
| `horizontalAccuracy`, `verticalAccuracy` | m | Genauigkeitsschätzungen des Moduls |

**Was `isAvailable()` bedeutet.** Bei I2C-Sensoren: der Sensor hat bei
`begin()` geantwortet **und** die letzten Lesevorgänge scheitern nicht
hintereinander (MPU — ~0,1 s; Barometer und Kompass — ~0,5 s ohne Antwort).
Scheitert ein Lesevorgang, werden die Daten nicht mit Müll überschrieben: die
vorherigen bleiben erhalten und der Fehlerzähler wächst (sichtbar mit dem
Befehl `s`). Beim GPS: mindestens ein gültiges NAV-PVT, und das letzte ist
nicht älter als `GPS_TIMEOUT_US`.

**Wenn der Sensor fehlt** (`nullptr` oder `isAvailable() == false`), gibt
`Autopilot` keine Korrekturen, und das Flugzeug wird wie in MANUAL geflogen.
`main.cpp` kalibriert nur die Sensoren, die geantwortet haben.

---

## Aufschlüsselung von FlightController::update()

Wird aus `loop()` alle 2 ms aufgerufen. Die Reihenfolge ist die Priorität:

1. **`receiver.update()`** — Auswertung der angesammelten iBUS-Bytes.
2. **Schalter** — `switches->update(rc)`, nur bei lebender Verbindung (in
   einem Failsafe-Frame spiegeln die Kanäle die Schalter nicht wider): der
   Modus (nur bei einem Wechsel), die Funktionen, die Drehregler.
3. **Gas des Piloten** — `throttle.update(rc, receiverFailsafe)`.
4. **Knüppel** — `mixer.fromSticks(rc)` × `Knob::RATES`; Klappen —
   `mixer.updateFlaps(target)` (Bremse, Schalter, Drehregler; ohne Verbindung
   — 0).
5. **Sensoren und Autopilot** — `autopilot->update(armed, linkLost, pilotThrottle, sticks)`
   **immer**, auch ohne Verbindung: die Winkelfilter dürfen nicht einfrieren.
   Solange nicht gearmt ist, läuft der PID (die Ruder reagieren auf Neigung —
   praktisch auf dem Tisch), aber der Integrator wird auf Null gehalten. Ohne
   Verbindung und gearmt — Failsafe RTH oder Gleiten.
6. **Summer** — `Beeper`.
7. **Verbindungsverlust** — `applyLinkLoss()`: gearmt — Ruder und Gas folgen
   dem Failsafe-Kommando des Autopiloten, sonst Neutral und Motor aus;
   `return`. Absolute Priorität vor allem, was darunter folgt.
8. **ARM** — `arming.update(rc, false)`.
9. **Kommando** — `autopilot->getCommand()`: in den stabilisierten Modi ist der
   Knüppel der gewünschte Winkel, und der Autopilot gibt die endgültigen
   Ruderkommandos aus.
10. **Mischer** — `mixer.mix(command)` → PWM der Querruder (Klappen + Rollen),
    des Höhenruders und des Seitenruders unter Berücksichtigung der Umkehr.
11. **Gas** — `autopilot->applyThrottle(pilotThrottle)`: das Gas des Piloten,
    das Gas des Autopiloten oder das Maximum aus beiden (Automatikstart).
    Danach, wenn nicht gearmt oder `MOTOR_KILL`, — zwangsweise `PWM_MIN`.
    Diese Prüfung steht ganz am Ende, damit kein Modus das Gas am ARM vorbei
    durchschleusen kann.
12. **AUX** — Nutzlast (`PAYLOAD_DROP`) und Kamera (`CAMERA_TILT`,
    `CAMERA_STAB`).
13. **`outputs.write(output)`** — PWM auf die 7 Ausgänge.

---

## HTTP-API des Web-Dashboards

Die Implementierung ist `include/telemetry/WebDebugServer.h`. Der Access
Point: SSID `OpenPlane-Debug`, Passwort `12345678`, Adresse
`http://192.168.4.1`.

### `GET /api/status`

```json
{
  "rc": [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1500],
  "armed": false,
  "failsafe": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "rudder":       { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true },
    "aux1":         { "us": 1000, "attached": true },
    "aux2":         { "us": 1500, "attached": true }
  },
  "flapsUs": 0,
  "imu":  { "attached": true, "available": true, "roll": 0.12, "pitch": -0.40, "yaw": 38.50 },
  "baro": { "attached": true, "available": true, "altitude": 0.05, "climb": 0.01 },
  "mag":  { "attached": true, "available": true, "heading": 41.9 },
  "gps":  { "attached": true, "available": true, "fix": 3, "numSV": 12, "lat": 55.750000, "lon": 37.610000, "alt": 150.0 },
  "airspeed": { "attached": true, "available": true, "ias": 14.2, "tas": 14.3, "dp": 123.4 },
  "autopilot": {
    "attached": true, "mode": 1, "modeName": "STABILIZE",
    "desiredRoll": 0.0, "desiredPitch": 0.0, "targetAlt": 0.0,
    "rollCorr": 0.0, "pitchCorr": 0.0, "throttleCorr": 0.0,
    "kpRoll": 5.000, "kiRoll": 0.500, "kdRoll": 0.500,
    "kpPitch": 5.000, "kiPitch": 0.500, "kdPitch": 0.500,
    "nav": { "gps": true, "home": true, "homeDist": 120, "homeBearing": 185,
             "course": 90, "targetCourse": 90, "speed": 14.3, "fence": false, "stall": false },
    "features": ["FLAPS"]
  }
}
```

- `attached` — das Objekt ist im Build vorhanden; `available` — der Sensor
  antwortet tatsächlich. Die Datenfelder werden **nur** bei `available: true`
  hinzugefügt.
- `outputs.*.attached` — der MCU hat einen LEDC-Kanal und einen Pin
  zugewiesen; ob ein physisches Servo angeschlossen ist, ist per Software
  nicht zu sehen (zur Prüfung des Impulses — die Konsole, Befehl `p`).
- `rollCorr`/`pitchCorr` — das endgültige Kommando des Autopiloten minus die
  Knüppel, in µs. `throttleCorr` — das Gas des Autopiloten, in % (0, solange
  das Gas beim Piloten liegt).
- `nav` — Navigation: der Startpunkt, die Entfernung und die Peilung dazu, der
  Kurs und der Zielkurs, die Geschwindigkeit für die Navigation (Pitotrohr /
  GPS), der Geofence, Strömungsabriss; `features` — die aktivierten
  Schalterfunktionen.

### `POST /api/setmode`

`{ "mode": 1 }` — die Nummer des `AutopilotMode`: `0` MANUAL, `1` STABILIZE, `2`
AUTO_TAKEOFF, `3` ALT_HOLD, `4` ACRO, `5` CRUISE, `6` LOITER, `7` RTH, `8`
LAUNCH, `9` AUTO_LAND, `10` SOARING, `11` RESCUE. Der Modus bleibt bestehen, bis
der Pilot den Modusschalter umlegt.

### `POST /api/setpid`

`{ "kpRoll": 5, "kiRoll": 0.5, "kdRoll": 0.5 }` — jedes der Felder `kpRoll`,
`kiRoll`, `kdRoll`, `kpPitch`, `kiPitch`, `kdPitch`; ausgelassene behalten ihre
bisherigen Werte.

Beide Befehle werden von der Flugschleife im nächsten Zyklus angewendet (siehe
[FreeRTOS-Tasks](#freertos-tasks-und-regelschleife)).

### `GET /`

Das HTML-Dashboard: Balken der 10 Kanäle, ARM/Verbindung, die Ausgänge, die
Sensoren, Modus-Schaltflächen, das PID-Formular. Es fragt `/api/status` alle
200 ms ab.

---

## Konsole und Diagnose

Der Port-Monitor — 115200, Anschluss „COM“. Die Implementierung sind
`DebugConsole` und `DebugLogger` ([Referenz](reference/telemetry.md)). Die
Tasten wirken sofort, Enter ist nicht nötig; die Kalibrierungen und `p`
blockieren die Schleife und sind deshalb nur ohne ARM verfügbar.

| Taste | Was sie tut |
|---|---|
| `h` / `?` | Hauptmenü |
| `l` | Das Menü „was ins Log ausgegeben wird“ (Kanäle, Modi, Periode) |
| Leertaste | Log pausieren / fortsetzen |
| `s` | `printStatus()` aller Sensoren: Daten, Fehlerzähler des Busses, Kalibrierungen, die Prüfung vor dem Flug |
| `i` | Gyro-Kalibrierung + Prüfung vor dem Flug (2 s ruhig) |
| `o` | Kalibrierung des IMU-Einbaus über drei Lagen, wird im NVS gespeichert |
| `m` | Kompasskalibrierung (15 s Drehen), wird im NVS gespeichert |
| `p` | Selbsttest der Ausgänge: der echte Impuls an jedem Pin gegenüber dem erwarteten |

Das Log ist in Kanäle gegliedert (`STAT`, `RC`, `OUT`, `ATT`, `AP`, `ALT`,
`MAG`, `GPS`, `IMU`, `SYS`), jeder mit dem Modus „aus / bei Änderung /
dauerhaft“; die Einstellungen werden im NVS gespeichert und beim Schließen
des Menüs geschrieben, nur ohne ARM. Standardmäßig sind `STAT` (bei Änderung)
und `SYS` (einmal alle 10 s) aktiviert:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
SYS  loop 500 Hz, avg 700 us, max 1400 us (der schlechteste Wert in 10 s) | iBUS ok=... crc_err=... | heap ... KB | uptime ... s
```

Die Formate aller Kanäle stehen in der [Referenz](reference/telemetry.md#debuglogger).

---

## Boardauswahl und Pinbelegung

| Befehl | `board` | Makro | Status |
|---|---|---|---|
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` + N16R8 (`qio_opi`, 16 MB) | `BOARD_ESP32_S3` | **Hauptplatine, die Standardauswahl.** Am Prüfstand mit allen Sensoren getestet |
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | Der alte Prototyp, ist unter Handsteuerung geflogen |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | Für den Prüfstand, die Pinbelegung wurde nicht an der Hardware getestet |
| `pio run -e stm32h743` | `weact_mini_h743vitx` | `BOARD_STM32H743` | STM32H743VIT6: die vollständige Firmware + MAVLink + Blackbox auf SD; auf einer nackten Platine getestet ([unten](#stm32h743)) |
| `pio run -e stm32h743-devebox` | `devebox_h743vitx` | `BOARD_STM32H743` | Dasselbe auf dem DevEBox H743: die Konsole ist USB-CDC, die Firmware wird per DFU geflasht |

| Zweck | ESP32-S3 (Prüfstand) | ESP32-C3 | ESP32 classic |
|---|---|---|---|
| Querruder links / rechts | GPIO4 / GPIO5 | GPIO5 / GPIO4 | GPIO13 / GPIO14 |
| Höhenruder / ESC | GPIO6 / GPIO7 | GPIO6 / GPIO7 | GPIO27 / GPIO26 |
| Seitenruder | GPIO18 | — (kein Pin) | GPIO25 |
| iBUS RX | GPIO17 | GPIO8 | GPIO16 |
| I2C der Sensoren SDA / SCL | GPIO41 / GPIO42 | GPIO1 / GPIO3 | GPIO21 / GPIO22 |
| I2C des OLED SDA / SCL | GPIO1 / GPIO2 | — | — |
| SPI SCK / MISO / MOSI | GPIO12 / 13 / 11 | GPIO0 / 10 / 20 | GPIO18 / 19 / 23 |
| SPI CS ICM42688 / BMP388 | GPIO14 / GPIO21 | GPIO21 / GPIO2 ⚠️ | GPIO32 / GPIO5 |
| GPS RX / TX | GPIO39 / GPIO40 (UART2) | GPIO9 ⚠️ / keiner (UART0) | GPIO4 / GPIO17 (UART2) |
| Serial | UART0 → Anschluss „COM“ | USB-CDC | UART0 |

- **ESP32-S3 N16R8:** GPIO33–37 sind vom Octal-PSRAM belegt, 26–32 vom Flash,
  19/20 vom USB, 43/44 vom Serial, 48 ist die RGB-LED; 0/3/45/46 sind
  Strapping-Pins.
- **ESP32-C3:** die Querruder an GPIO4/5 sind gegenüber dem S3 vertauscht. Für
  den vollen Satz reichen die Pins nicht: der CS des BMP388 und der RX des GPS
  liegen auf Strapping-Pins, das GPS hat kein TX (nur Empfang, kein UBX-CFG).
  Einzelheiten stehen in `Config.h`.

### STM32H743

Der STM32H743VIT6 (Cortex-M7 mit 480 MHz, 2 MB Flash, 1 MB RAM) führt die
**vollständige Firmware** aus: dieselben Sensoren, derselbe Autopilot,
dieselben Schalter, dieselbe Konsole und dasselbe Display wie auf dem
ESP32-S3, dazu MAVLink-Telemetrie und eine Blackbox auf der SD-Karte. Er lässt
sich bauen, besteht cppcheck und alle nativen Tests des gemeinsamen Codes. An
der Hardware wurde die **DevEBox-H743-Platine ohne Sensoren** getestet: Start,
Konsole über USB, SD-Karte, Blackbox —
[TESTING.md](TESTING.md#tests-auf-dem-stm32-board) — sowie iBUS, ARM und PWM an die
Servos und den Motor: Steuerung vom Sender im manuellen Modus (auf Video). Die
Sensoren am STM32 warten noch auf einen Prüfstand. Die Hauptflugplatine ist der
ESP32-S3.

- **HAL** — `include/hal/stm32/`: `Stm32Board` (dieselbe API wie `Esp32Board`,
  plus `telemetryUart()`), `Stm32I2CBus`, `Stm32SpiBus`, `Stm32UartPort`,
  `Stm32ServoOutput` (Hardware-PWM von `HardwareTimer`, ein Timer für mehrere
  Ausgänge). Ausführlich — [reference/hal.md](reference/hal.md#implementierung-für-den-stm32h743).
- **Einstellungen und Kalibrierungen** — nicht im NVS, sondern in einem
  `KeyValueStore` im letzten Sektor des Flashs (`include/storage/`,
  `hal/stm32/Stm32FlashStorage.h`). Der Projektcode schreibt weiterhin
  `#include <Preferences.h>`: im Env `stm32h743` liegt das Verzeichnis
  `include/hal/stm32/compat/` in `-I`, und dort befindet sich ein
  `Preferences` mit derselben API. Das Abbild trägt einen CRC32: ein
  beschädigtes (Strom fiel während des Löschens aus) wird als leer gelesen. Das
  Schreiben in den Flash erfolgt in einer Hintergrund-Task: das Löschen eines
  128-KB-Sektors dauert Sekunden, aber der Sektor liegt in Bank 2, während der
  Code aus Bank 1 läuft, und die Flug-Task verdrängt die Hintergrund-Task ohne
  anzuhalten.
- **Tasks** — FreeRTOS aus der Bibliothek STM32duino FreeRTOS, ein Kern,
  Verdrängung nach Priorität (`hal/Rtos.h`): `flight` (5) — die Flugschleife,
  MAVLink, Log, Konsole; `oled` (1) und `storage` (1) — im Hintergrund;
  `bbox` (2) — das Schreiben der Blackbox auf die SD-Karte.
- **Die Blackbox auf der SD-Karte** — SDMMC1, 4 Bit, 24 MHz
  (`hal/stm32/Stm32SdCard.h`, die Pins in `src/stm32/sd_msp.cpp`). Die Karte
  bleibt ein gewöhnliches FAT32: auf ihr liegt eine vorab angelegte Datei
  `BLACKBOX.BIN`, die Firmware schreibt rohe Blöcke hinein und rührt das
  Dateisystem selbst nicht an (`storage/Fat32File.h` ist nur lesend).
  Vorbereitung der Karte und Auslesen — [BLACKBOX.md](BLACKBOX.md#sd-karte-stm32h743).
- **Telemetrie** — MAVLink 2 auf UART4 (`telemetry/MavlinkTelemetry.h`) statt
  des WLAN-Dashboards: QGroundControl / Mission Planner, Wechsel von Modus und
  PID vom Boden aus. Ausführlich —
  [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md#bodenstation-wlan-dashboard-und-mavlink).
- **Pinbelegung** — der Block `BOARD_STM32H743` in `Config.h`, die Pins wurden
  aus den freien des WeAct MiniSTM32H743VITx gewählt und mit den Tabellen von
  STM32duino abgeglichen:

| Zweck | STM32H743 | Peripherie |
|---|---|---|
| Querruder links / rechts | PA0 / PA1 | TIM2_CH1 / CH2 |
| Höhenruder / ESC | PA2 / PA3 | TIM2_CH3 / CH4 |
| Seitenruder | PD14 | TIM4_CH3 |
| AUX1 (Nutzlast) / AUX2 (Kamera) | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 |
| iBUS RX (TX — Reserve) | PE7 (PE8) | UART7 |
| I2C der Sensoren SDA / SCL | PB11 / PB10 | I2C2 |
| I2C des OLED SDA / SCL | PB9 / PB8 | I2C1 |
| SPI SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 |
| SPI CS IMU / Barometer | PB12 / PD10 | GPIO |
| GPS RX / TX | PD9 / PD8 | USART3 |
| Funkmodem MAVLink RX / TX | PD0 / PD1 | UART4 |
| Summer | PE15 | GPIO |
| Serial | PA10 / PA9 | LPUART1 |

- **DevEBox H743 (MCUDEV)** — das Env `stm32h743-devebox`: derselbe Code, eine
  eigene Kernvariante, die Konsole über USB-C als virtueller COM-Port (CDC) —
  ein USB-UART ist nicht nötig. Das erste Flashen der Firmware geschieht per
  USB über den eingebauten Bootloader (DFU):
  1. Windows: einmalig den WinUSB-Treiber für „STM32 BOOTLOADER“ installieren
     ([Zadig](https://zadig.akeo.ie): DFU in FS Mode → WinUSB → Install Driver).
  2. Den Pin **BT0** (BOOT0) mit einem Draht mit **3V3** verbinden, **RST**
     drücken und loslassen: die Platine ist im DFU-Modus (am DevEBox gibt es
     keinen BOOT0-Taster).
  3. `pio run -e stm32h743-devebox -t upload` (`upload_protocol = dfu`).
  4. Der BT0-Draht kann entfernt werden — die Firmware startet von selbst.

  Danach wird der Draht nicht mehr gebraucht: die Taste **`D`** in der Konsole
  (aus jedem Menü, nicht bei ARM) startet die Platine in den Bootloader neu:
  eine Markierung im RAM → Reset → Sprung in den Systemspeicher, noch bevor die
  Takte konfiguriert sind (`src/stm32/bootloader.cpp`). Ein Sprung direkt aus
  der laufenden Firmware hängt auf dem H7 — an der Platine geprüft, deshalb
  zwei Schritte. Nötig ist eine geöffnete Konsole (USB-CDC); antwortet die
  Platine nicht — RST bei gestecktem BT0-Draht.
- **Einstiegspunkt** — `src/stm32/main.cpp` (aus den ESP32-Builds über
  `build_src_filter` ausgeschlossen). Die Objekte sind dieselben wie in
  `src/main.cpp`; anstelle von `loop()` gibt es Tasks, und
  `vTaskStartScheduler()` steht am Ende von `setup()`.
- **Erstes Einschalten der Platine:** `pio run -e stm32h743 -t upload`
  (ST-Link), der Monitor an LPUART1 über einen USB-UART; `b` — ob die Sensoren
  an den Bussen sichtbar sind, `s` — Sensorstatus, `p` — Impulse an den
  Ausgängen (Propeller abnehmen), danach der Sender und QGroundControl über das
  Funkmodem.

---

## Wie man einen neuen Sensor hinzufügt

### A) Ein weiterer Chip einer bestehenden Kategorie (IMU, Barometer, Kompass)

Das Gemeinsame ist bereits in den Basisklassen geschrieben — der Chiptreiber
wird klein:

1. Legen Sie `include/sensors/<category>/<Name>_Sensor.h` an und erben Sie von
   `ImuSensorBase` / `BarometerBase` / `MagnetometerBase`. Der Konstruktor
   nimmt einen `IRegisterDevice&` entgegen — der Treiber weiß nicht, ob es I2C
   oder SPI ist.
2. Implementieren Sie:
   - `begin()` — `device.begin()`, die Chip-ID prüfen, die Register
     schreiben, `setAvailable(true/false)` aufrufen;
   - IMU: `readSample()` (rohe Werte für accel/gyro/temp in den Chipachsen),
     `accelLsbPerG()`, `gyroLsbPerDps()`, `temperatureC()`;
   - Barometer: `isNewSampleReady()` (ein Bereitschafts-Flag oder einfach
     `true`) und `readSample()` (Druck in Pa, Temperatur in °C); die
     Abfrageperiode steht im Konstruktor der Basis;
   - Kompass: `readRaw()` (X/Y/Z in den Chipachsen) und
     `lsbPerMicroTesla()`; der Name des NVS-Namensraums für die Kalibrierung
     steht im Konstruktor der Basis.
3. Braucht der Chip über SPI ein Dummy-Byte vor den Daten oder eine besondere
   Frequenz — fügen Sie eine statische Fabrik `spiDevice(bus, cs)` hinzu, wie
   bei `BMP388_Sensor`.
4. Ein Zweig in `SensorSelection.h`: `#define SENSOR_<CATEGORY>_<NAME>`,
   `using Selected... = ...;` und `#define SELECTED_..._DEVICE(board) ...`
   (`I2cRegisterDevice(board.i2c(), address)` oder die SPI-Fabrik). `main.cpp`
   wird beim Sensorwechsel nicht angefasst.
5. Prüfen Sie den Build mit dem neuen Sensor, ohne die Datei zu ändern — mit
   einem Flag: `PLATFORMIO_BUILD_FLAGS="-DSENSOR_BARO=SENSOR_BARO_<NAME>" pio run`,
   dann alle drei Umgebungen, dann an der Hardware.

### B) Eine neue Kategorie

1. Die Datenstruktur und die Schnittstelle gehören in `SensorInterface.h`,
   nach dem Vorbild von `GpsSensor`/`GpsData`.
2. Hat die Kategorie gemeinsame Logik (Filter, Kalibrierung) — eine Basisklasse
   nach dem Vorbild von `BarometerBase`.
3. Ein nullbarer Zeiger im Konstruktor von `Autopilot` (ohne Sensor — keine
   Wirkung, statt eines Absturzes) und Felder in `GET /api/status` mit einem
   Paar `attached`/`available`.

### Ein neuer Bus oder eine neue Peripherie

Eine neue Schnittstelle in `include/hal/`, die Implementierung in
`include/hal/esp32/` und in `include/hal/stm32/`, der Zugriff über `IBoard`.

---

## Wie man einen neuen Autopilot-Modus hinzufügt

1. Ein Wert in `enum AutopilotMode` (`autopilot/AutopilotTypes.h`, vor
   `MODE_COUNT`), der Name und ein Kurzname (bis zu 5 Zeichen, für das OLED) in
   `AutopilotNames::mode()` / `modeShort()`.
2. Ein Handler `run<Mode>()` und ein Zweig in `Autopilot::runMode()`; die
   Anfangsziele (Kurs, Höhe, Kreismittelpunkt) kommen in `initializeMode()`.
   Der Modus setzt `desiredRoll`/`desiredPitch` und ruft `stabilizeOrManual()`
   auf (ohne IMU liegen die Ruder beim Piloten) oder `stabilizeOrNeutral()`
   (ohne IMU — Neutral). Ohne den nötigen Sensor — sicheres Verhalten, kein
   Absturz. Der Integrator sammelt nur bei `armed`.
3. Gas: `throttleMode` (`PILOT` / `AUTO` / `AT_LEAST`) und `autoThrottlePct`,
   oder `autoThrottle()` — das Reisegas vom Drehregler / vom Pitotrohr.
   `FlightController` ändert sich dabei nicht.
4. Am Sender — eine einzige Zeile in `config/Controls.h`
   (`Bind::mode(Channels::SWD, MODE_NEW)`). Dashboard und MAVLink übernehmen
   den Modus über seine Nummer; für MAVLink — der nächstliegende
   ArduPlane-Modus in `MavlinkModes::toCustomMode()` / `fromCustomMode()`.
5. Braucht der Modus Sensoren für ARM — `ArmingManager`.
6. Tests: die Reaktion auf jeden Sensor — `test/native/test_autopilot_modes`;
   der Flug im geschlossenen Regelkreis — ein Szenario in `test/native/test_sim`
   (das Flugzeugmodell `helpers/PlaneSim.h`, der Prüfaufbau
   `helpers/SimHarness.h`). Danach — der Tisch ohne Propeller: die Ruder müssen
   auf Neigung in die Richtung des Aufrichtens reagieren.
7. Ein Abschnitt im [AUTOPILOT_GUIDE.md](AUTOPILOT_GUIDE.md).

---

## Rückführung (Vorarbeit, nicht angeschlossen)

`include/autopilot/feedback/` ist der nächste Schritt für den Autopiloten.
**Weder `FlightController` noch `Autopilot` noch `main.cpp` binden diese
Dateien ein:** einen Prototyp für Flugtests gibt es noch nicht, und die
Firmware läuft ohne sie. Geprüft werden sie durch eine Simulation im
geschlossenen Regelkreis (`test/test_feedback/`) direkt auf der Platine.

### Wozu

`Autopilot` ist heute ein PID auf den Winkel: Fehler × Faktor = Ruder. Er
weiß nicht, was daraus am Flugzeug geworden ist, und die Faktoren stimmen nur
für eine einzige Geschwindigkeit: bei niedriger Geschwindigkeit ist das Ruder
schwächer und der PID korrigiert zu wenig, bei hoher Geschwindigkeit zu viel.
Die Rückführung schließt den Regelkreis über **die Reaktion des Flugzeugs**:

- das Ruder wurde ausgeschlagen, aber das Flugzeug dreht langsamer als nötig —
  mehr dazugeben, bis es ankommt;
- wie viel Ruder nötig ist, wird im Flug gemessen und mit der Geschwindigkeit
  neu berechnet;
- das Flugzeug dreht in die falsche Richtung — das Vorzeichen ist vertauscht,
  umdrehen und prüfen;
- der Winkel wurde ausgeglichen, aber die Geschwindigkeit sinkt — Gas und
  Nase nach unten, bis das Flugzeug nicht abkippt;
- Start und Landung — in Phasen, nach dem, was die Sensoren anzeigen.

### Module

| Datei | Was sie tut |
|---|---|
| `FlightSnapshot.h` | Alles, was die Rückführung in einem Zyklus über das Flugzeug weiß. Der einzige Eingang: die Module lesen die Sensoren und das RC nicht direkt, deshalb lassen sie sich auf einer Simulation und auf Logs laufen lassen |
| `FeedbackOutput.h` | Die Ausgabe eines Zyklus: Ruderausschläge je Achse, ob die Achse eingeschaltet ist, das Vorzeichen der Achse, das Gas (setzen / nicht darunter), der Grund |
| `FeedbackConfig.h` | Alle Konstanten (beim Anschließen wandern sie in `Config.h`) |
| `SpeedEstimator.h` | Die Geschwindigkeit (Pitotrohr > GPS) und die Längsbeschleunigung aus der IMU: `dV/dt = g·(ax − sin θ)` — „die Geschwindigkeit sinkt“ ist auch ohne Geschwindigkeitssensor zu sehen |
| `AirborneDetector.h` | In der Luft / am Boden: Lernen, Sammeln des Integrals und Suchen nach dem Strömungsabriss ergeben nur im Flug Sinn |
| `ControlEffectivenessEstimator.h` | Lernt für jede Achse das Modell `ε = b·u(t−delay) + a·ω + c` mit der rekursiven Methode der kleinsten Quadrate |
| `AdaptiveRateController.h` | Eine Kaskade Winkel → Winkelgeschwindigkeit → Winkelbeschleunigung → Ruder über das gelernte Modell |
| `StallGuard.h` | Schutz vor Geschwindigkeitsverlust und Strömungsabriss |
| `TakeoffSequencer.h`, `LandingSequencer.h`, `PhaseTargets.h` | Start (von einer Bahn oder aus der Hand) und Landung in Phasen, nach den Sensoren |
| `FeedbackSupervisor.h` | Alles zusammen: die Reihenfolge innerhalb eines Zyklus, die Prioritäten, `requestTakeoff()`/`requestLanding()`/`cancelPhase()`, `printStatus()`, der Anschlussplan |
| `FeedbackModules.h` | Ein einziges Include für alles |

### Wie es funktioniert

**Ruderwirksamkeit.** Das Achsenmodell: Winkelbeschleunigung
`ε = b·u + a·ω + c`. `b` ist, wie viele °/s² 1 µs Ruder bringt (das Vorzeichen
ist die Richtung der Reaktion), `a` ist die Dämpfung (die Luft bremst die
Drehung; ohne diesen Term würde die Schätzung von `b` bei stationärer Drehung
gegen Null gehen), `c` ist ein konstantes Moment (Schwerpunktlage, Trimmung,
Propeller). Die Kraft eines Ruders ∝ ρV², deshalb wird `b` bei einer
Referenzgeschwindigkeit gelernt und mit `(V/Vref)²` multipliziert: das
Flugzeug wird schneller — das Ruder „wird sofort stärker“, ohne neu zu
lernen. Die angezeigte Geschwindigkeit des Pitotrohrs enthält die Luftdichte
bereits, die Höhe ist also von selbst berücksichtigt; ohne
Geschwindigkeitssensor ist der Maßstab 1, und `b` wird direkt gelernt.

Die Daten werden in Intervallen von 20 ms genommen: die mittlere
Beschleunigung über ein Intervall ist die Differenz des Gyroskops an den
Enden / die Dauer, und ihr entsprechen das mittlere Ruder und die mittlere
Winkelgeschwindigkeit im selben Intervall (das Ruder — mit der Verzögerung
`RESPONSE_DELAY_MS`). Danach durchlaufen beide Seiten der Gleichung denselben
2-Hz-Tiefpassfilter: das Verhältnis ändert sich nicht, während die hohen
Frequenzen, bei denen das Modell der „reinen Verzögerung“ wegen der Trägheit
des Servos lügt, entfallen. Gelernt werden kann nur in der Luft und nur, wenn
das Ruder „angeregt“ wird (ein Ausschlag ≥ `MIN_EXCITATION_US` binnen ~0,3 s);
die Knüppel des Piloten regen ebenfalls an, deshalb lernt die Schätzung auch
in MANUAL.

**Der Regler.** Drei Stufen, Achse für Achse:

```
ω* = ANGLE_GAIN · (target − angle)              "Nase 10° unten — mit 40°/s anheben"
ε* = (ω* − ω + I) / RATE_TAU,  I += Ki·(ω* − ω)  "dreht langsamer als nötig — nachsteuern"
surface = (ε* − a·ω − c) / b                    über das gelernte Modell
```

Das Integral `I` wird in °/s gespeichert und nicht in µs Ruder — deshalb bleibt
es richtig, wenn sich die Schätzung von `b` ändert. Am Boden ist das Integral
eingefroren (außer dem Kurs im Startlauf / Ausrollen), und am Ruderanschlag
sammelt es sich nicht in Richtung des Anschlags an. Die koordinierte Kurve
wird berücksichtigt (wenn die Geschwindigkeit bekannt ist): in der
Schräglage braucht man ein Nicken von `g·sin φ·tg φ / V` und ein Gieren von
`g·sin φ / V`.

**Achsenvorzeichen — nur am Boden.** Im Flug werden die Achsen weder
abgeschaltet noch umgedreht: der Einbau der IMU wird durch die Kalibrierung
`o` und die Prüfung beim Einschalten bestimmt, die Ruderrichtungen durch die
Prüfung des Piloten vor dem Flug. Indirekte Anzeichen in der Luft (ein
Abkippen, ein Trudeln, Figuren, Böen) können täuschen, und eine
abgeschaltete oder umgedrehte Achse kostet in einem solchen Moment das
Flugzeug. Ist die Schätzung von `b` für eine Achse sicher negativ, so ist das
nur eine Warnung in `reason` („reagiert verkehrt herum auf das Ruder? am
Boden prüfen“); eine negative Schätzung gelangt nicht in den Regler — die
Achse arbeitet mit dem A-priori-Modell.

**Schutz vor Strömungsabriss.** Zwei Stufen. *LowEnergy* — die Geschwindigkeit
sinkt schnell bei angehobener Nase, oder sie liegt nahe am Strömungsabriss
(< 1.25·Vs), oder das Höhenruder hat an Wirksamkeit verloren: Gas ≥ 80 %,
Nicken ≤ 5°. *Stall* — die Geschwindigkeit liegt unter der Abrissgeschwindigkeit,
die Nase fällt gegen das Höhenruder, der Flügel kippt bei geringer Energie
gegen die Querruder weg: Vollgas, Nase nach unten, Schräglage ≤ 10°,
Querruder begrenzt (ein großer Querruderausschlag lässt die Flügelspitze
abreißen). Die Maßnahmen werden mit Hysterese aufgehoben (Geschwindigkeit
≥ 1.5·Vs). Bei Verbindungsverlust wird das Gas nicht angetastet, und dicht
über dem Boden (Abfangen, Ausrollen) ist der Schutz abgeschaltet — die Landung
selbst ist ein kontrollierter Strömungsabriss.

**Start.** `WaitThrottle` (der Motor steht) → der Pilot gab Gas ≥ 50 % →
`GroundRoll` (Vollgas, Flügel waagerecht, den Kurs halten Seitenruder und
Rad, das Höhenruder ist frei) → Abhebegeschwindigkeit oder ein Timeout ohne
Geschwindigkeitssensor → `Climb` (12°, Vollgas) → Höhe 30 m → `Complete`. Aus der
Hand (`TAKEOFF_HAND_LAUNCH`) statt des Startlaufs — `WaitLaunch`: der Motor
startet erst nach dem Wurf (Längsbeschleunigung ≥ 1g). Gas vor dem Abheben
weggenommen — Abbruch.

**Landung.** `Approach` (Gas 25 %, Sinken mit 1 m/s — das Nicken aus dem Fehler
der Vertikalgeschwindigkeit, die Schräglage vom Piloten ≤ 20°) → Höhe 2 m →
`Flare` (Gas 0, das Sinken wird nach derselben Regel auf 0.3 m/s gedämpft) →
Aufprall am Beschleunigungssensor oder „tief und dreht nicht“ → `Rollout`
(Kurs mit dem Rad) → `Complete`. Gas des Piloten ≥ 80 % — Durchstarten. Zum
Abfangen braucht man einen Entfernungsmesser: das Barometer irrt sich um einen
Meter.

**Prioritäten** (`FeedbackSupervisor`): nicht gearmt > Schutz vor
Strömungsabriss > Start/Landung > Ziele des Modus. Bei Verbindungsverlust
werden die Phasen abgebrochen, und die Stabilisierung führt die Gleitziele des
Failsafe aus.

### Simulation

`test/test_feedback/test_main.cpp` (am PC: `pio test -e native -f test_feedback`) — ein Flugzeugmodell (unabhängige Achsen,
Verzögerung und Trägheit des Servos, Ruderwirksamkeit ∝ V², Dämpfung ∝ V, konstante
Momente, Auftrieb über den Anstellwinkel in Abhängigkeit von der Geschwindigkeit, Strömungsabriss, Fahrwerk mit
Lenkrad) und 10 Szenarien:

| Szenario | Was geprüft wird |
|---|---|
| Herausdrehen aus 30° Schräglage / −15° Nicken mit konstantem Moment | Aufrichten und „weiter nachsteuern“: das Integral findet die Trimmung selbst |
| Schütteln um ±15° bei 14 und 20 m/s, ohne Geschwindigkeitssensor | Die Schätzung von `b` konvergiert zur Wahrheit und wird mit der Geschwindigkeit umgerechnet |
| Vertauschtes Querruder, der Pilot wackelt in MANUAL mit den Flügeln | Die Schätzung von `b` ist negativ → nur eine Warnung, die Achse wird nicht abgeschaltet |
| 30 s Turbulenz | Böen werden abgefangen, die Schräglage geht nicht über 10° hinaus |
| Nase 15° bei 20 % Gas (mit Geschwindigkeitssensor und ohne) | Die Geschwindigkeit fällt nicht bis zum Strömungsabriss |
| Start von einer Bahn mit dem Reaktionsmoment des Propellers | Phasen, Höhe, Kurs im Startlauf |
| Landung aus 15 m | Phasen, kein Gas nahe am Boden, sanfte Landung |
| Verbindungsverlust im Startlauf; nicht gearmt; MANUAL | Abbruch, das Gas wird nicht angetastet, die Ruder bleiben beim Piloten |

Das Modell ist grob — es prüft die Logik und die Vorzeichen, nicht die
Abstimmung auf ein bestimmtes Flugzeug.

```bash
pio test -e native -f test_feedback      # am PC, in Sekunden
pio test -e esp32-s3 -f test_feedback    # flasht die Testfirmware und führt sie aus
pio run -t upload                        # die normale Firmware zurückspielen
```

### Anschlussplan

1. `FlightController::update()` füllt nach dem Lesen der Sensoren und der
   Berechnung der Kommandos einen `FlightSnapshot` und ruft
   `FeedbackSupervisor::update()` auf. Zuerst im **Schattenmodus**: die
   Ausgabe geht nur ins Log (`printStatus()`) und ins Dashboard, nicht an die
   Ruder. Im Flug unter Handsteuerung muss die Schätzung von `b` jeder Achse
   positiv sein und mit der Geschwindigkeit wachsen.
2. Am Boden, das Flugzeug in den Händen, STABILIZE: kippen — die Ruder
   halten dagegen.
3. Eine Achse nach der anderen: `deflectionUs` statt
   `Autopilot::getRollCorrection()` (zuerst nur das Rollen), dann das Nicken.
4. Gas: `throttleOverridePercent`/`throttleFloorPercent` — nach
   `Autopilot::applyThrottle()`, vor dem Failsafe (das Failsafe steht über
   allem).
5. Start/Landung — auf einen freien Schalter; den Modus `AUTO_TAKEOFF` aus
   `Autopilot` entfernen.
6. Die Konstanten von `FeedbackConfig` — nach `Config.h`; der
   Fluggeschwindigkeitssensor — eine Implementierung von `AirspeedSensor` und
   eine Kategorie in `SensorSelection.h`.

---

## Wie man ein neues Board hinzufügt

1. `[env:<name>]` in `platformio.ini` mit einem eindeutigen
   `-D BOARD_ESP32_<NAME>`.
2. Ein Block `#elif defined(BOARD_ESP32_<NAME>)` in `Config.h` mit allen Pins,
   einschließlich `PIN_I2C2_SDA/SCL` (−1, wenn kein OLED vorhanden ist).
   Rechnen Sie das GPIO-Budget vorher durch: Flash/PSRAM/USB/Strapping.
3. Die Servo-Ausgänge brauchen 5 LEDC-Kanäle — jeder ESP32 hat sie. Gibt es
   für das Seitenruder keinen Pin — `PIN_RUDDER = -1`, und der Ausgang wird
   einfach abgeschaltet.
4. Ändern Sie `default_envs` nicht, bevor das Board an der Hardware getestet
   wurde; vermerken Sie im Commit ausdrücklich, wenn die Pinbelegung nicht
   getestet ist.

---

## Befehle zum Bauen, Aufspielen und Überwachen

```bash
pio run                        # das Standard-Board bauen (esp32-s3)
pio run -t upload              # aufspielen
pio device monitor             # Monitor, 115200
pio run -e esp32-s3 -e esp32-c3 -e esp32-dev -e stm32h743   # prüfen, dass alle Boards bauen
```

- **ESP32-S3:** das Aufspielen und das Serial laufen über den Anschluss „COM“
  (CH343). Hängt sich die Brücke auf (Windows antwortet „das Gerät funktioniert
  nicht“ — das kann an Störungen durch den ESC liegen), hilft es, das Kabel neu
  zu stecken; aufspielen lässt sich auch über den Anschluss „USB“ (das
  eingebaute USB-JTAG): `pio run -t upload --upload-port <USB COM port>`.
- Solange der Port-Monitor geöffnet ist, gelingt das Aufspielen auf denselben
  Port nicht.
- `lib_deps`: `olikraus/U8g2` (OLED) ist die einzige externe Bibliothek.
- `test/` — ausführlich im [`TESTING.md`](TESTING.md):
  - `pio test -e native -e native-stm32` — 387 Tests am PC (Hardware-Attrappen
    in `test/native/support/`), Abdeckung — `gcovr`;
  - `pio test -e esp32-s3` — `test_feedback/` (die Simulation der Rückführung
    im geschlossenen Regelkreis) und `test_imu_orientation/` auf der Platine
    selbst; jeder spielt eine Testfirmware auf, danach die normale mit
    `pio run -t upload` aufspielen.
- Statische Analyse: `pio check -e esp32-s3` (cppcheck), `pio check -e stm32h743` (cppcheck über
  `hal/stm32/` und `src/stm32/`) und `tools/clang-tidy.sh`
  (das Profil `.clang-tidy`).

---

## Bekannte Einschränkungen

- **Der Autopilot wurde nicht im Flug getestet.** Auf dem Tisch wurden die
  Vorzeichen live geprüft (Neigung → Korrektur in Richtung des Aufrichtens),
  die PID-Faktoren sind Startwerte.
- **STABILIZE ist ein Aufrichten über den Knüppeln**, kein „Winkelmodus“
  (FBWA), bei dem der Knüppel den Roll-/Nickwinkel vorgibt. Pilot und
  Autopilot addieren sich.
- **Das Gleiten bei Verbindungsverlust wurde nicht im Flug getestet.** Die
  Winkel `FAILSAFE_GLIDE_*` sind Startwerte; das Nicken von −3° wird auf ein
  bestimmtes Flugzeug abgestimmt (die Nase darf weder bis zum Strömungsabriss
  steigen noch abtauchen).
- **Der Horizont.** Mit der Einbaukalibrierung (`o`) — stammt er aus ihr (NVS);
  der Nullpunktversatz des Beschleunigungssensors driftet mit der Temperatur
  (~1–2° pro 20 °C), wenn der Horizont „weggewandert“ ist — `o` wiederholen.
  Ohne sie — die Lage beim Einschalten (waagerecht einschalten).
- **Der Einbau des Kompasses** wird weiterhin durch `MAG_ROTATION_CW_DEG`
  festgelegt (die Kalibrierung über Lagen betrifft ihn nicht).
- **Kompass:** der Kurs ist ohne Neigungskompensation, die Zählrichtung wurde
  am fertig aufgebauten Flugzeug nicht überprüft, und die Kalibrierung muss
  bereits im Flugzeug erfolgen. Kein Modus nutzt den Kurs bisher.
- **Das GPS** wird nicht für die Navigation verwendet; am ESP32-C3 nur
  Empfang.
- **Die Rückführung (`autopilot/feedback/`) ist nicht angeschlossen** und wurde
  nur in der Simulation mit einem groben Flugzeugmodell geprüft. Alle Zahlen
  in `FeedbackConfig.h`, die mit „прикидка“ („grobe Schätzung“) gekennzeichnet sind, müssen an einem echten
  Flugzeug verfeinert werden; einen Fluggeschwindigkeitssensor gibt es noch
  nicht (ohne ihn wird die Ruderwirksamkeit langsamer gelernt, und ein
  Strömungsabriss ist nur an der Verzögerung zu erkennen).
- **An der Hardware nicht getestet:** `ICM42688_Sensor` (über `ImuSensorBase`
  auf die gemeinsame Konvention gebracht), `BME280_Sensor` (die
  Bosch-Kompensation wurde neu implementiert), BMP388 über SPI,
  `QMC5883L_Sensor`, die GPS-Einrichtung über CFG-VALSET. Beim Anschließen —
  das Startlog, `s` in der Konsole, die Vorzeichen durch Neigen.
- **I2C auf dem Steckbrett fängt Störungen ein** vom ESC/Motor (vereinzelte
  Fehler sind über `s` zu sehen). Die Treiber überstehen sie, aber im
  Flugzeug sollten die I2C-Leitungen kurz sein und fern von den Leistungsleitungen
  liegen.
- **ESP32Servo wird nicht verwendet.** Version 3.2.1 verteilt auf dem ESP32-S3
  die Servos auf die MCPWM und verwechselt in `attachPin()` die Nummer der
  MCPWM-Einheit mit der Timernummer: GPIO6/7 gaben das Signal von GPIO4/5
  aus (der ESC wurde vom rechten Knüppel gesteuert). Die Ausgänge wurden auf
  LEDC umgeschrieben; die Bibliothek nur nach einer Prüfung mit `p`
  zurückholen.
- **Der ESC ist 50-Hz-PWM**, einen Modus zur Kalibrierung des Gasbereichs gibt
  es in der Firmware noch nicht.
- **Web-Dashboard:** das Passwort des Access Points ist schwach, und Befehle
  werden auch im Flug angenommen. Es ist ein Werkzeug für Prüfstand und Feld,
  nicht für den Flug.
- **Mechanik des Prototyps:** der erste Prototyp ist geflogen, dabei fielen
  eine schwache Befestigung des Motors und eine unzureichende Steifigkeit des
  Flügels auf.
- **Die Lizenz ist die OpenPlane License** ([LICENSE](LICENSE.md)): MIT mit
  verpflichtender Nennung des Autors, einem Verbot der militärischen Nutzung
  und einem Verbot vorsätzlicher Schädigung von Menschen und Sachen ohne deren
  schriftliche Einwilligung. Fügen Sie den Dateien keine anderen Lizenzköpfe
  hinzu und entfernen Sie den Namen des Autors nicht.

---

## Wie man Änderungen vornimmt

- **Kleine Commits:** ein logischer Schritt — ein Commit.
- **Tests und Analyse vor einem Commit:** `pio test -e native -e native-stm32`,
  `pio check -e esp32-s3`, `pio check -e stm32h743`, `tools/clang-tidy.sh` —
  alles grün ([`TESTING.md`](TESTING.md)).
- **Bauen Sie alle Boards** nach Änderungen am gemeinsamen Code — der S3 ist
  die Hauptplatine, aber C3, das 38-Pin-Board und `stm32h743` dürfen nicht
  kaputtgehen; vor einem Release — `tools/build_matrix.sh` (alle Boards × alle
  Sensoren).
- **Prüfen Sie an der Hardware, was sich prüfen lässt:** Vorzeichen — durch
  Neigen, Ausgänge — mit dem Befehl `p`, die Verbindung — durch Ausschalten des
  Senders.
- **Erfinden Sie keine APIs.** Gleichen Sie mit den Quellen des Frameworks in
  `~/.platformio/packages/framework-arduinoespressif32/` (Arduino core 2.0.x)
  ab — das Internet beschreibt oft die Version 3.x mit einer anderen API (zum
  Beispiel LEDC).
- **Schönen Sie den Status nicht.** An der Hardware nicht getestet — schreiben
  Sie es auch so.
- **Eine Schicht darf nicht mehr wissen, als ihr zusteht.** Braucht eine untere
  Klasse plötzlich eine obere, muss die Logik in `FlightController` nach oben
  wandern.
- **Wenn Sie einen Datenvertrag ändern** (`FlightOutputState`,
  `ControlCommand`, `ImuData`, das JSON von `/api/status`) — aktualisieren Sie
  alle Verbraucher im selben Commit.
