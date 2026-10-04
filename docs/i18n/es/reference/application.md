# APPLICATION — `src/main.cpp` y `src/stm32/main.cpp`

> 🌐 Esta página es una traducción del [original en ruso](../../../reference/application.md). Si la traducción y el original difieren, prevalece el original. El firmware muestra los mensajes de la consola en ruso, por lo que se citan tal cual. La traducción la ha hecho una IA y no la han revisado hablantes nativos. Si encuentras errores, escribe a [Damir Lebedev](https://github.com/damir-lebedev) o abre una [incidencia](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referencia](README.md)

Los dos puntos de entrada son la **composition root** de sus placas: la única
unidad de traducción del firmware y el único lugar donde se crean los objetos
y se enlazan mediante referencias. No contienen lógica de vuelo y el conjunto
de objetos es el mismo; difieren la placa, la telemetría (Wi-Fi o MAVLink) y
la forma de ejecutar el ciclo de vuelo.

## Objetos globales (comunes)

El orden de declaración = el orden de construcción.

| Objeto | Tipo | Enlaces |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | el bus de `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | con el tubo de Pitot, es la presión estática |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | solo si `SENSOR_MAG != NONE`, si no, `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | solo si `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | solo si `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | todos los sensores (pueden ser nulos) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, la tabla `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | todo lo anterior |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | el controlador, el piloto automático, las estadísticas |
| `debugConsole` | `DebugConsole` | el controlador, las salidas, el piloto automático, el registro, `&board` (sondeo de los buses `b`) |
| `oledDisplay` | `OledDisplay` | el controlador, el piloto automático, las estadísticas |
| ESP32: `webDebugServer` | `WebDebugServer` | el controlador, el piloto automático |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, el controlador, el piloto automático, las estadísticas |

## `src/main.cpp` — ESP32 (S3, C3, 38 pines)

| Función | Descripción |
|---|---|
| `static void printBanner()` | la pantalla de presentación en `Serial` |
| `static void setupSensors()` | `begin()` de cada sensor; calibración de los que respondieron: IMU `calibrate()` (2 s inmóvil + la comprobación prevuelo), barómetro `calibrateAltitude()`, brújula: la primera muestra a los 25 ms fija el rumbo de la IMU (`setYaw`); GPS `begin()`; Pitot `begin()` (el cero, en el primer segundo del ciclo); `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **antes** de `begin(115200)`; la presentación; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; el servidor web; la disposición de los interruptores; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; el periodo es `vTaskDelayUntil(LOOP_PERIOD_MS)`; un retraso > 100 ms: se vuelve a contar desde cero (sin «ponerse al día») |

## `src/stm32/main.cpp` — STM32H743

El punto de entrada del env `stm32h743` (en las compilaciones para ESP32 el
directorio `src/stm32/` se excluye mediante `build_src_filter`). En hardware se
ha probado la placa DevEBox H743 sin sensores (arranque, consola por USB,
tarjeta SD, caja negra, iBUS y control manual de los servos y del motor); en
conjunto se ejecuta en el PC con las pruebas `test/native_stm32` (el env
`native-stm32`). Junto a él: `sd_msp.cpp`, los pines y relojes de SDMMC1, y
`bootloader.cpp`, la tecla `D` de la consola (reinicio en DFU).

| Función | Descripción |
|---|---|
| `setup()` | `Serial.begin(115200)`; la presentación; `board.begin()`; las salidas a posición segura; `Stm32FlashStorage::store().mount()`: la imagen de ajustes (vacía / N bytes / dañada: valores por defecto); `setupSensors()` (como en la ESP32); `flightController.begin()`; `mavlink.begin()`; OLED; la disposición de los interruptores; `debugLogger.begin()`; las tareas `flight` y `storage`; `vTaskStartScheduler()` (no retorna) |
| `static void flightTask(void*)` | prioridad `Rtos::PRIORITY_FLIGHT`, pila de 16 KB: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; `vTaskDelayUntil(LOOP_PERIOD_MS)`, un retraso > 100 ms: se vuelve a contar desde cero |
| `static void storageTask(void*)` | en segundo plano: `Stm32FlashStorage::instance().service()` una vez cada 100 ms: borrado y escritura del sector de ajustes, la tarea de vuelo la expropia |
| `loop()` | vacío: tras `vTaskStartScheduler()` solo funcionan las tareas |

La consola (`Serial`, LPUART1 PA9/PA10, 115200) es la misma `DebugConsole` que
en la ESP32: `h` menú, `s` sensores, `b` sondeo de buses, `p` salidas,
calibraciones.

## Invariantes

- Las salidas pasan a la posición segura **antes** de inicializar los sensores
  (la calibración de la IMU mantiene el ciclo ~2 s).
- ESP32: el búfer TX de `Serial` se fija antes de `begin()`. STM32: los búferes
  de las UART son `SERIAL_RX/TX_BUFFER_SIZE` en `platformio.ini`.
- Ningún objeto es propietario de otro: todas las referencias son no
  propietarias y su vida útil es la del programa entero.
- Para cambiar lo que hace un interruptor: `config/Controls.h`, no `main.cpp`.
