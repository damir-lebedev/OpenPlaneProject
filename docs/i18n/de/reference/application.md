# APPLICATION — `src/main.cpp` und `src/stm32/main.cpp`

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/application.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

[← Referenz](README.md)

Die beiden Einstiegspunkte sind die **composition root** ihrer Platinen: die
einzige Übersetzungseinheit der Firmware und die einzige Stelle, an der die
Objekte erzeugt und über Referenzen verknüpft werden. Eine Flugsteuerlogik
enthalten sie nicht, und der Satz der Objekte ist derselbe; unterschiedlich
sind die Platine, die Telemetrie (WLAN oder MAVLink) und die Art, wie die
Flugschleife getaktet wird.

## Globale Objekte (gemeinsame)

Die Reihenfolge der Deklaration = die Reihenfolge der Konstruktion.

| Objekt | Typ | Verknüpfungen |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | der Bus aus `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | mit Pitotrohr — das ist der statische Druck |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | nur wenn `SENSOR_MAG != NONE`, sonst `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | nur wenn `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | nur wenn `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | alle Sensoren (nullbar) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, die Tabelle `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | alles oben Genannte |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | der Controller, der Autopilot, die Statistik |
| `debugConsole` | `DebugConsole` | der Controller, die Ausgänge, der Autopilot, das Log, `&board` (Busabfrage `b`) |
| `oledDisplay` | `OledDisplay` | der Controller, der Autopilot, die Statistik |
| ESP32: `webDebugServer` | `WebDebugServer` | der Controller, der Autopilot |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, der Controller, der Autopilot, die Statistik |

## `src/main.cpp` — ESP32 (S3, C3, 38-Pin)

| Funktion | Beschreibung |
|---|---|
| `static void printBanner()` | der Begrüßungsbildschirm im `Serial` |
| `static void setupSensors()` | `begin()` jedes Sensors; Kalibrierung der Sensoren, die geantwortet haben: IMU `calibrate()` (2 s ruhig + die Prüfung vor dem Flug), Barometer `calibrateAltitude()`, Kompass — der erste Messwert nach 25 ms legt den IMU-Kurs fest (`setYaw`); GPS `begin()`; Pitotrohr `begin()` (der Nullpunkt — in der ersten Sekunde der Schleife); `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **vor** `begin(115200)`; der Begrüßungsbildschirm; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; der Webserver; die Schalterbelegung; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; die Periode ist `vTaskDelayUntil(LOOP_PERIOD_MS)`; eine Verspätung > 100 ms — die Zählung beginnt neu (ohne „Aufholen“) |

## `src/stm32/main.cpp` — STM32H743

Der Einstiegspunkt des Env `stm32h743` (in den ESP32-Builds ist das
Verzeichnis `src/stm32/` über `build_src_filter` ausgeschlossen). An der
Hardware wurde die DevEBox-H743-Platine ohne Sensoren getestet (Start, Konsole
über USB, SD-Karte, Blackbox, iBUS und Handsteuerung der Servos und des
Motors); als Ganzes läuft sie am PC mit den Tests `test/native_stm32` (das
Env `native-stm32`). Daneben: `sd_msp.cpp` — die Pins und Takte von SDMMC1,
`bootloader.cpp` — die Konsolentaste `D` (Neustart in DFU).

| Funktion | Beschreibung |
|---|---|
| `setup()` | `Serial.begin(115200)`; der Begrüßungsbildschirm; `board.begin()`; die Ausgänge in die sichere Stellung; `Stm32FlashStorage::store().mount()` — das Abbild der Einstellungen (leer / N Bytes / beschädigt — Standardwerte); `setupSensors()` (wie am ESP32); `flightController.begin()`; `mavlink.begin()`; OLED; die Schalterbelegung; `debugLogger.begin()`; die Tasks `flight` und `storage`; `vTaskStartScheduler()` (kehrt nicht zurück) |
| `static void flightTask(void*)` | Priorität `Rtos::PRIORITY_FLIGHT`, Stack 16 KB: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; `vTaskDelayUntil(LOOP_PERIOD_MS)`, eine Verspätung > 100 ms — die Zählung beginnt neu |
| `static void storageTask(void*)` | im Hintergrund: `Stm32FlashStorage::instance().service()` einmal alle 100 ms — Löschen und Schreiben des Einstellungssektors, wird von der Flug-Task verdrängt |
| `loop()` | leer: nach `vTaskStartScheduler()` laufen nur noch die Tasks |

Die Konsole (`Serial`, LPUART1 PA9/PA10, 115200) ist dieselbe `DebugConsole`
wie am ESP32: `h` Menü, `s` Sensoren, `b` Busabfrage, `p` Ausgänge,
Kalibrierungen.

## Invarianten

- Die Ausgänge gehen in die sichere Stellung, **bevor** die Sensoren
  initialisiert werden (die IMU-Kalibrierung hält die Schleife ~2 s an).
- ESP32: der TX-Puffer des `Serial` wird vor `begin()` gesetzt. STM32: die
  UART-Puffer sind `SERIAL_RX/TX_BUFFER_SIZE` in `platformio.ini`.
- Kein Objekt besitzt ein anderes: alle Referenzen sind nicht besitzend, die
  Lebensdauer ist das ganze Programm.
- Um zu ändern, was ein Schalter tut — `config/Controls.h`, nicht `main.cpp`.
