# APPLICATION — `src/main.cpp`

[← Справочник](README.md)

`src/main.cpp` — **composition root**: единственная единица трансляции
прошивки, единственное место, где создаются объекты и связываются ссылками.
Логики полёта в нём нет.

## Глобальные объекты

Порядок объявления = порядок конструирования.

| Объект | Тип | Связи |
|---|---|---|
| `board` | `Esp32Board` | — |
| `imuDevice`, `imuSensor` | `SELECTED_IMU_DEVICE(board)`, `SelectedImu` | шина из `SensorSelection.h` |
| `baroDevice`, `baroSensor` | `SELECTED_BARO_DEVICE(board)`, `SelectedBaro` | |
| `magDevice`, `magSensor`, `magnetometer` | … `SelectedMag`, `MagnetometerSensor* const` | только если `SENSOR_MAG != NONE`, иначе `magnetometer = nullptr` |
| `gpsSensor`, `gps` | `SelectedGps`, `GpsSensor* const` | только если `SENSOR_GPS != NONE`, иначе `gps = nullptr` |
| `ibusReceiver` | `IBusReceiver` | `board.rcUart()` |
| `controlMixer`, `throttleManager` | `ControlMixer`, `ThrottleManager` | |
| `flightOutputs` | `FlightOutputs` | `board` |
| `autopilot` | `Autopilot` | IMU, баро, `magnetometer`, `gps` |
| `modeSelector`, `armingManager` | `AutopilotModeSelector`, `ArmingManager` | `&autopilot` |
| `flightController` | `FlightController` | всё выше |
| `loopStats` | `LoopStats` | |
| `debugLogger` | `DebugLogger` | контроллер, автопилот, статистика |
| `debugConsole` | `DebugConsole` | контроллер, выходы, автопилот, лог |
| `webDebugServer` | `WebDebugServer` | контроллер, автопилот |
| `oledDisplay` | `OledDisplay` | контроллер, автопилот, статистика |

## Функции

| Функция | Описание |
|---|---|
| `static void printBanner()` | Заставка в `Serial` |
| `static void setupSensors()` | `begin()` каждого датчика; калибровка ответивших: IMU `calibrate()` (2 с неподвижно + предполётная проверка), барометр `calibrateAltitude()`, компас — первый отсчёт через 25 мс задаёт начальный курс IMU (`setYaw`); GPS `begin()`; `autopilot.begin()` |
| `void setup()` | `Serial.setTxBufferSize(4096)` **до** `begin(115200)`; баннер; `board.begin()`; `flightOutputs.begin()` + `setFailsafe()`; `setupSensors()`; `flightController.begin()`; OLED; веб-сервер; подсказки; `debugLogger.begin()` |
| `void loop()` | `applyPendingCommands()` → `flightController.update()` → `debugLogger.update()` → `debugConsole.update()` → `loopStats.record()`; фиксированный период `vTaskDelayUntil(LOOP_PERIOD_MS)`; если такт опоздал больше чем на 100 мс — отсчёт начинается заново (без «догоняния») |

## Инварианты

- Выходы уходят в безопасное положение **до** инициализации датчиков
  (калибровка IMU держит цикл ~2 с).
- Буфер TX `Serial` задаётся до `begin()` — иначе он не применяется.
- Ни один объект не владеет другим: все ссылки невладеющие, время жизни всех
  объектов — вся программа.
