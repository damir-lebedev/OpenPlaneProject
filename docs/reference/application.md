# APPLICATION — `src/main.cpp` и `src/stm32/main.cpp`

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

---

<a id="src-stm32-main-cpp"></a>

## `src/stm32/main.cpp` — bring-up для STM32H743 (заготовка)

Точка входа env `stm32h743` (в сборках ESP32 каталог `src/stm32/` исключён
через `build_src_filter`). На железе не проверялась.

Полная прошивка на STM32 пока не собирается из-за трёх ESP32-зависимостей выше
HAL: `Preferences` (NVS) — калибровки датчиков и настройки лога; Wi-Fi-дашборд
`WebDebugServer`; задачи FreeRTOS на втором ядре (веб, OLED). Поэтому здесь
собрано то, что от MCU уже не зависит.

| Объект | Тип | Связи |
|---|---|---|
| `board` | `Stm32Board` | — |
| `ibusReceiver`, `controlMixer`, `throttleManager`, `flightOutputs` | как в `src/main.cpp` | |
| `armingManager` | `ArmingManager` | без автопилота (`nullptr`) |
| `flightController` | `FlightController` | без автопилота и селектора режимов |
| `imuSpi`, `baroSpi` | `SpiRegisterDevice` | те же параметры, что у `ICM42688_Sensor::spiDevice` / `BMP388_Sensor::spiDevice` — только для проверки ID |

| Функция | Описание |
|---|---|
| `setup()` | `Serial.begin(115200)`; баннер; `board.begin()`; выходы в безопасное положение; опрос шин; `flightController.begin()` |
| `loop()` | `flightController.update()` → консоль; фиксированный период `LOOP_PERIOD_MS` по `millis()` (без FreeRTOS); опоздание > 100 мс — отсчёт заново |
| `printBuses()` | Скан I2C датчиков и экрана (адреса 0x08..0x77), ID чипов на SPI: ICM42688 `WHO_AM_I` (0x75 → 0x47), BMP388 `CHIP_ID` (0x00 → 0x50) |
| `printStatus()` | Связь, кадры iBUS (хорошие/битые), ARM, текущие импульсы выходов |

Консоль (`Serial`, LPUART1 PA9/PA10, 115200): `s` — состояние, `p` —
самопроверка выходов (`FlightOutputs::printPulseSelfTest()`), `b` — опрос шин.

Что нужно для полной прошивки на STM32: замена `Preferences` (EEPROM-эмуляция
во флеше), телеметрия вместо Wi-Fi (радиомодем на `PIN_TELEM_RX/TX`), отрисовка
OLED и телеметрия из основного цикла или отдельных задач FreeRTOS/таймеров.

