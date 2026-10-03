# APPLICATION — `src/main.cpp` и `src/stm32/main.cpp`

[← Справочник](README.md)

Две точки входа — **composition root** для своих плат: единственная единица
трансляции прошивки, единственное место, где создаются объекты и связываются
ссылками. Логики полёта в них нет, набор объектов одинаковый; отличаются плата,
телеметрия (Wi-Fi или MAVLink) и то, как крутится полётный цикл.

## Глобальные объекты (общие)

Порядок объявления = порядок конструирования.

| Объект | Тип | Связи |
|---|---|---|
| `board` | `Esp32Board` / `Stm32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | шина из `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | с трубкой Пито — это статика |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | только если `SENSOR_MAG != NONE`, иначе `nullptr` |
| `gpsSensor`, `gpsReceiver` | `SelectedGps`, `GpsSensor* const` | только если `SENSOR_GPS != NONE` |
| `pitotDevice`, `pitotBaro`, `pitotSensor`, `airspeedSensor` | `SELECTED_PITOT_DEVICE(board)`, `SelectedPitotBaro` (`"PITOT-BMP581"`), `PitotDualBaroAirspeed(pitotBaro, baroSensor)` | только если `SENSOR_AIRSPEED != NONE` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | все датчики (nullable) |
| `pilotSwitches` | `PilotSwitches` | `&autopilot`, таблица `Controls::BINDINGS` |
| `armingManager` | `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | всё выше |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | контроллер, автопилот, статистика |
| `debugConsole` | `DebugConsole` | контроллер, выходы, автопилот, лог, `&board` (опрос шин `b`) |
| `oledDisplay` | `OledDisplay` | контроллер, автопилот, статистика |
| ESP32: `webDebugServer` | `WebDebugServer` | контроллер, автопилот |
| STM32: `mavlink` | `MavlinkTelemetry` | `*board.telemetryUart()`, контроллер, автопилот, статистика |

## `src/main.cpp` — ESP32 (S3, C3, 38-pin)

| Функция | Описание |
|---|---|
| `static void printBanner()` | заставка в `Serial` |
| `static void setupSensors()` | `begin()` каждого датчика; калибровка ответивших: IMU `calibrate()` (2 с неподвижно + предполётная проверка), барометр `calibrateAltitude()`, компас — первый отсчёт через 25 мс задаёт курс IMU (`setYaw`); GPS `begin()`; трубка `begin()` (ноль — первую секунду цикла); `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **до** `begin(115200)`; баннер; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; веб-сервер; раскладка тумблеров; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; период `vTaskDelayUntil(LOOP_PERIOD_MS)`; опоздание > 100 мс — отсчёт заново (без «догоняния») |

## `src/stm32/main.cpp` — STM32H743

Точка входа env `stm32h743` (в сборках ESP32 каталог `src/stm32/` исключён
через `build_src_filter`). На железе проверена плата DevEBox H743 без датчиков (загрузка,
консоль по USB, SD-карта, чёрный ящик, iBUS и ручное управление сервами и мотором); целиком гоняется на ПК тестами
`test/native_stm32` (env `native-stm32`). Рядом: `sd_msp.cpp` — выводы и
тактирование SDMMC1, `bootloader.cpp` — клавиша `D` консоли (перезагрузка в DFU).

| Функция | Описание |
|---|---|
| `setup()` | `Serial.begin(115200)`; баннер; `board.begin()`; выходы в безопасное положение; `Stm32FlashStorage::store().mount()` — образ настроек (пусто / N байт / повреждён — по умолчанию); `setupSensors()` (как на ESP32); `flightController.begin()`; `mavlink.begin()`; OLED; раскладка тумблеров; `debugLogger.begin()`; задачи `flight` и `storage`; `vTaskStartScheduler()` (не возвращается) |
| `static void flightTask(void*)` | приоритет `Rtos::PRIORITY_FLIGHT`, стек 16 КБ: `flightController.update()` → `mavlink.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; `vTaskDelayUntil(LOOP_PERIOD_MS)`, опоздание > 100 мс — отсчёт заново |
| `static void storageTask(void*)` | фоновая: `Stm32FlashStorage::instance().service()` раз в 100 мс — стирание и запись сектора настроек, вытесняется полётной задачей |
| `loop()` | пустая: после `vTaskStartScheduler()` работают только задачи |

Консоль (`Serial`, LPUART1 PA9/PA10, 115200) — та же `DebugConsole`, что на
ESP32: `h` меню, `s` датчики, `b` опрос шин, `p` выходы, калибровки.

## Инварианты

- Выходы уходят в безопасное положение **до** инициализации датчиков
  (калибровка IMU держит цикл ~2 с).
- ESP32: буфер TX `Serial` задаётся до `begin()`. STM32: буферы UART —
  `SERIAL_RX/TX_BUFFER_SIZE` в `platformio.ini`.
- Ни один объект не владеет другим: все ссылки невладеющие, время жизни —
  вся программа.
- Изменить, что делает тумблер, — `config/Controls.h`, не `main.cpp`.
