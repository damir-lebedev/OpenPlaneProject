# APPLICATION — `src/main.cpp` and `src/stm32/main.cpp`

> 🌐 This page is a translation of the [Russian original](../../../reference/application.md). If the translation and the original differ, the original is authoritative. The firmware prints its console messages in Russian, so they are quoted as is.

[← Reference](README.md)

The two entry points are the **composition root** for their boards: the only
translation unit of the firmware and the only place where the objects are
created and linked by references. There is no flight logic in them and the set
of objects is the same; what differs is the board, the telemetry (Wi-Fi or
MAVLink) and how the flight loop is driven.

## Global objects (common)

The order of declaration = the order of construction.

| Object | Type | Links |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | the bus from `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | with a pitot tube — this is the static pressure |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | only if `SENSOR_MAG != NONE`, otherwise `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | only if `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | only if `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | all the sensors (nullable) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, the `Controls::BINDINGS` table |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | everything above |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | the controller, the autopilot, the statistics |
| `debugConsole` | `DebugConsole` | the controller, the outputs, the autopilot, the log, `&board` (bus scan `b`) |
| `oledDisplay` | `OledDisplay` | the controller, the autopilot, the statistics |
| ESP32: `webDebugServer` | `WebDebugServer` | the controller, the autopilot |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, the controller, the autopilot, the statistics |

## `src/main.cpp` — ESP32 (S3, C3, 38-pin)

| Function | Description |
|---|---|
| `static void printBanner()` | the banner to `Serial` |
| `static void setupSensors()` | `begin()` of every sensor; calibration of the ones that responded: IMU `calibrate()` (2 s motionless + the preflight check), barometer `calibrateAltitude()`, compass — the first sample after 25 ms sets the IMU heading (`setYaw`); GPS `begin()`; pitot `begin()` (zero — in the first second of the loop); `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **before** `begin(115200)`; the banner; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; the web server; the switch layout; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; the period is `vTaskDelayUntil(LOOP_PERIOD_MS)`; a lateness > 100 ms — the count restarts (no "catching up") |

## `src/stm32/main.cpp` — STM32H743

The entry point of the `stm32h743` env (in the ESP32 builds the `src/stm32/`
directory is excluded through `build_src_filter`). On hardware the DevEBox H743
board without sensors has been tested (boot, the console over USB, the SD card,
the black box, iBUS and manual control of the servos and the motor); the whole
thing is run on a PC by the `test/native_stm32` tests (the `native-stm32` env).
Next to it: `sd_msp.cpp` — the SDMMC1 pins and clocks, `bootloader.cpp` — the
console `D` key (reboot into DFU).

| Function | Description |
|---|---|
| `setup()` | `Serial.begin(115200)`; the banner; `board.begin()`; the outputs to a safe position; `Stm32FlashStorage::store().mount()` — the settings image (empty / N bytes / corrupted — defaults); `setupSensors()` (as on the ESP32); `flightController.begin()`; `mavlink.begin()`; OLED; the switch layout; `debugLogger.begin()`; the `flight` and `storage` tasks; `vTaskStartScheduler()` (does not return) |
| `static void flightTask(void*)` | priority `Rtos::PRIORITY_FLIGHT`, stack 16 KB: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; `vTaskDelayUntil(LOOP_PERIOD_MS)`, a lateness > 100 ms — the count restarts |
| `static void storageTask(void*)` | background: `Stm32FlashStorage::instance().service()` once every 100 ms — erasing and writing the settings sector, preempted by the flight task |
| `loop()` | empty: after `vTaskStartScheduler()` only the tasks run |

The console (`Serial`, LPUART1 PA9/PA10, 115200) is the same `DebugConsole` as
on the ESP32: `h` menu, `s` sensors, `b` bus scan, `p` outputs, calibrations.

## Invariants

- The outputs go to a safe position **before** the sensors are initialized
  (the IMU calibration holds the loop for ~2 s).
- ESP32: the `Serial` TX buffer is set before `begin()`. STM32: the UART
  buffers are `SERIAL_RX/TX_BUFFER_SIZE` in `platformio.ini`.
- No object owns another: all the references are non-owning, the lifetime is
  the whole program.
- To change what a switch does — `config/Controls.h`, not `main.cpp`.
