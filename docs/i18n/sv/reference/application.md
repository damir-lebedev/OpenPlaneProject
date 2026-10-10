# APPLIKATION – `src/main.cpp` och `src/stm32/main.cpp`

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/application.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

De två ingångspunkterna är **composition root** för sina kort: firmwarens enda
översättningsenhet och det enda stället där objekten
skapas och länkas med referenser. Där finns ingen flyglogik och uppsättningen
objekt är densamma; det som skiljer är kortet, telemetrin (Wi-Fi eller
MAVLink) och hur flygslingan drivs.

## Globala objekt (gemensamma)

Deklarationsordningen = konstruktionsordningen.

| Objekt | Typ | Länkar |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | – |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | bussen från `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | med pitotrör – det är det statiska trycket |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | bara om `SENSOR_MAG != NONE`, annars `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | bara om `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | bara om `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | alla sensorer (nullbara) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, tabellen `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | allt ovanför |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | regulatorn, autopiloten, statistiken |
| `debugConsole` | `DebugConsole` | regulatorn, utgångarna, autopiloten, loggen, `&board` (bussavsökning `b`) |
| `oledDisplay` | `OledDisplay` | regulatorn, autopiloten, statistiken |
| ESP32: `webDebugServer` | `WebDebugServer` | regulatorn, autopiloten |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, regulatorn, autopiloten, statistiken |

## `src/main.cpp` – ESP32 (S3, C3, 38-pin)

| Funktion | Beskrivning |
|---|---|
| `static void printBanner()` | bannern till `Serial` |
| `static void setupSensors()` | `begin()` för varje sensor; kalibrering av dem som svarade: IMU `calibrate()` (2 s orörligt + kontrollen före flygning), barometer `calibrateAltitude()`, kompass – den första mätningen efter 25 ms sätter IMU:ns kurs (`setYaw`); GPS `begin()`; pitot `begin()` (nollpunkt – under slingans första sekund); `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **före** `begin(115200)`; bannern; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; webbservern; brytarnas fördelning; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; perioden är `vTaskDelayUntil(LOOP_PERIOD_MS)`; en försening > 100 ms – räkningen börjar om (ingen ”ikapptagning”) |

## `src/stm32/main.cpp` – STM32H743

Ingångspunkten för env `stm32h743` (i ESP32-byggena utesluts katalogen `src/stm32/`
via `build_src_filter`). På hårdvara har DevEBox H743-kortet
utan sensorer provats (uppstart, konsolen via USB, SD-kortet,
den svarta lådan, iBUS och manuell styrning av servona och motorn); det hela
körs på en dator av testerna `test/native_stm32` (env `native-stm32`).
Bredvid: `sd_msp.cpp` – SDMMC1-stiften och klockorna, `bootloader.cpp` –
konsoltangenten `D` (omstart i DFU).

| Funktion | Beskrivning |
|---|---|
| `setup()` | `Serial.begin(115200)`; bannern; `board.begin()`; utgångarna till ett säkert läge; `Stm32FlashStorage::store().mount()` – inställningsavbilden (tom / N byte / korrupt – standardvärden); `setupSensors()` (som på ESP32); `flightController.begin()`; `mavlink.begin()`; OLED; brytarnas fördelning; `debugLogger.begin()`; uppgifterna `flight` och `storage`; `vTaskStartScheduler()` (returnerar inte) |
| `static void flightTask(void*)` | prioritet `Rtos::PRIORITY_FLIGHT`, stack 16 KB: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; `vTaskDelayUntil(LOOP_PERIOD_MS)`, en försening > 100 ms – räkningen börjar om |
| `static void storageTask(void*)` | bakgrund: `Stm32FlashStorage::instance().service()` var 100:e ms – radering och skrivning av inställningssektorn, avbryts av flyguppgiften |
| `loop()` | tom: efter `vTaskStartScheduler()` körs bara uppgifterna |

Konsolen (`Serial`, LPUART1 PA9/PA10, 115200) är samma `DebugConsole` som
på ESP32: `h` meny, `s` sensorer, `b` bussavsökning, `p` utgångar, kalibreringar.

## Invarianter

- Utgångarna går till ett säkert läge **innan** sensorerna initieras
  (IMU-kalibreringen håller slingan i ~2 s).
- ESP32: `Serial`:s TX-buffert ställs in före `begin()`. STM32: UART-
  buffertarna är `SERIAL_RX/TX_BUFFER_SIZE` i `platformio.ini`.
- Inget objekt äger ett annat: alla referenser är icke-ägande, livslängden är
  hela programmet.
- För att ändra vad en brytare gör – `config/Controls.h`, inte `main.cpp`.
