# DEVELOPER_GUIDE.md — Руководство разработчика OpenPlaneProject

Этот документ — техническая карта прошивки OpenPlaneProject: где в каком файле
обрабатывается какой датчик, какие данные откуда куда текут, как устроена
архитектура классов и HTTP API веб-дашборда. Рассчитан на разработчика,
который уже пишет C++ и хочет быстро сориентироваться в конкретно этом
репозитории (ветка `oop-refactor`), а не в общих основах языка или PlatformIO.

Если вы ищете обзор проекта, полётные характеристики прототипа или
дорожную карту развития — см. `../README.md` и `ROADMAP.md`. Здесь — только
код: файлы, классы, методы, контракты между слоями.

> Проект в активной разработке. Часть кода (автопилот, датчики) написана,
> компилируется и подключена в прошивку, но ещё не проверена в полёте —
> это явно отмечено там, где это касается конкретного модуля. Не
> дорабатывайте документ "по ощущению" — если сомневаетесь, что именно
> делает код, перечитайте исходник, а не гадайте.

---

## Содержание

1. [Архитектура слоёв](#архитектура-слоёв)
2. [Справочник файлов](#справочник-файлов-include)
3. [Карта RC-каналов](#карта-rc-каналов)
4. [Данные датчиков](#данные-датчиков)
5. [Разбор FlightController::update()](#разбор-flightcontrollerupdate)
6. [HTTP API веб-дашборда](#http-api-веб-дашборда)
7. [Выбор платы и распиновка](#выбор-платы-и-распиновка)
8. [Как добавить новый датчик](#как-добавить-новый-датчик)
9. [Как добавить новый режим автопилота](#как-добавить-новый-режим-автопилота)
10. [Как добавить новую плату](#как-добавить-новую-плату)
11. [Команды сборки/заливки/монитора](#команды-сборкизаливкимонитора)
12. [Известные ограничения](#известные-ограничения)
13. [Как вносить изменения](#как-вносить-изменения)

---

## Архитектура слоёв

Прошивка построена как объектная архитектура с чёткой однонаправленной
зависимостью: нижний слой ничего не знает о верхнем. Классы почти целиком
живут в заголовочных файлах (`include/*.h`), `src/main.cpp` — единственная
точка сборки (composition root), которая создаёт все объекты и связывает их
в `setup()`/`loop()`.

```
┌───────────────────────────────────────────────────────────────────────┐
│ APPLICATION (src/main.cpp)                                            │
│   composition root: создаёт все объекты, setup()/loop()               │
└──────────────────────────────┬────────────────────────────────────────┘
                                │ владеет и вызывает
                                ▼
┌───────────────────────────────────────────────────────────────────────┐
│ COORDINATION                                                          │
│   FlightController.h  — единственный оркестратор update()             │
│   FeatureManager.h    — назначение RC-слотов на функции автопилота    │
│   DebugLogger.h       — печать состояния в Serial                     │
│   WebDebugServer.h    — веб-дашборд (HTTP JSON API)                   │
└───────┬───────────────────────┬───────────────────────┬───────────────┘
        │ читает/пишет          │ читает                │ читает
        ▼                       ▼                       ▼
┌───────────────────┐  ┌─────────────────────┐  ┌──────────────────────┐
│ FLIGHT LOGIC       │  │ FLIGHT LOGIC (авто-  │  │ FLIGHT LOGIC (общее) │
│ (ручное управление)│  │ пилот)               │  │                      │
│ ControlMixer.h     │  │ Autopilot.h          │  │ ArmingManager.h      │
│ ThrottleManager.h  │  │ (PID_Controller +    │  │ FlightOutputState.h  │
│                    │  │  4 режима)           │  │ (POD-контракт)       │
└─────────┬──────────┘  └──────────┬───────────┘  └──────────┬───────────┘
          │ читает RcChannelState  │ читает Sensor*           │
          ▼                        ▼                          │
┌───────────────────────────┐  ┌──────────────────────────────┴────────┐
│ HARDWARE ABSTRACTION (RC)  │  │ SENSORS                                │
│ RcChannelState.h  (снимок) │  │ SensorInterface.h (Sensor/ImuSensor/   │
│ RcInput.h (clamp/centered) │  │   BarometerSensor/MagnetometerSensor/  │
│ IBusReceiver.h (IUartPort→ │  │   GpsSensor — абстрактные)             │
│   снимок iBUS,             │  │ MPU6050_Sensor / ICM42688_Sensor       │
│   isSignalLost())          │  │ BME280_Sensor / BMP388_Sensor          │
│                             │  │ QMC5883P_Sensor, UbloxM10_Gps          │
│                             │  │ SensorSelection.h — какой чип выбран   │
└─────────────────────────────┘  └─────────────────────────────────────┘
          │                                       ▲
          ▼                                       │ (читают через II2CBus/
┌───────────────────────────────────────────────────────────────────────┐ ISpiBus/IUartPort)
│ HARDWARE ABSTRACTION (выход)                                          │
│   FlightOutputs.h — знает про FlightOutputState, пишет через           │
│                      IServoOutput (не про Servo/ESP32Servo напрямую)   │
│   Channels.h — имена логических индексов RC-каналов                   │
└──────────────────────────────┬────────────────────────────────────────┘
                                │ всё выше зависит только от интерфейсов
                                ▼
┌───────────────────────────────────────────────────────────────────────┐
│ HAL — "МОЗГ" (include/hal/)                                           │
│   IBoard / II2CBus / ISpiBus / IUartPort / IServoOutput — абстрактные  │
│   hal/esp32/Esp32Board.h (+ Esp32I2CBus/Esp32SpiBus/Esp32UartPort/     │
│     Esp32ServoOutput) — единственная сегодня реализация, обёртки      │
│     над Wire/SPI/HardwareSerial/ESP32Servo                            │
│   Config.h — выбор платы (BOARD_ESP32_*) + пины + тюнинговые константы │
│                                                                         │
│   Чтобы перейти на другой MCU (например STM32) — пишется              │
│   hal/stm32/Stm32Board.h с тем же публичным API IBoard; всё выше       │
│   (сенсоры, FlightOutputs, IBusReceiver, автопилот) не меняется.       │
└───────────────────────────────────────────────────────────────────────┘
```

Правило зависимостей, которое держит архитектуру чистой:

- **HAL** (`hal/IBoard.h` и другие интерфейсы + `hal/esp32/*`) — единственный
  слой, которому разрешено знать про конкретный MCU (`Wire`, `SPIClass`,
  `HardwareSerial`, `ESP32Servo`). Всё остальное обращается только к
  интерфейсам (`IBoard`/`II2CBus`/`ISpiBus`/`IUartPort`/`IServoOutput`).
- **Hardware abstraction** (`IBusReceiver`, `FlightOutputs`, `RcChannelState`,
  `RcInput`, `Config`, `Channels`) не знает ничего ни о самолёте, ни о
  failsafe, ни о PID — только парсинг байтов iBUS и запись PWM-значений через
  HAL-интерфейсы, без прямого обращения к MCU.
- **Sensors** (`SensorInterface`, конкретные `*_Sensor.h`/`UbloxM10_Gps.h`) не
  знают ничего об автопилоте — только читают шину через `II2CBus`/`ISpiBus`/
  `IUartPort` и отдают структуры `ImuData`/`BarometerData`/`MagData`/`GpsData`.
  Какой конкретно чип каждой категории скомпилирован — решает
  `sensors/SensorSelection.h`, а не эти файлы.
- **Flight logic** (`ControlMixer`, `ThrottleManager`, `ArmingManager`,
  `Autopilot`) — чистые функции/классы над данными: не знают про UART, Servo,
  Wi-Fi, время системы (кроме `micros()`/`millis()`, которые им передают явно
  или которые они читают напрямую там, где это самодостаточно, например ПИД).
- **Coordination** (`FlightController`, `FeatureManager`, `DebugLogger`,
  `WebDebugServer`) — единственный слой, которому разрешено знать сразу про
  несколько нижних слоёв одновременно и решать порядок операций.
- **Application** (`main.cpp`) — единственное место, где создаётся `Esp32Board`
  и конкретные экземпляры датчиков (через алиасы `SelectedImu`/`SelectedBaro`/
  `SelectedMag`/`SelectedGps` из `SensorSelection.h`, не по именам классов
  напрямую) и происходит связывание (dependency injection руками, без
  DI-фреймворка).

---

## Справочник файлов (include/)

Порядок — как в `include/include.h` (реальный порядок `#include` в этом
файле; `FlightController.h` и `DebugLogger.h` подключаются в самом конце,
после `WebDebugServer.h`, а не сразу после `ArmingManager.h`).

| Файл | Зона ответственности | Ключевые типы / методы | Знает | НЕ знает (принципиально) |
|---|---|---|---|---|
| `hal/IBoard.h`, `II2CBus.h`, `ISpiBus.h`, `IUartPort.h`, `IServoOutput.h` | Абстрактные интерфейсы "мозга" (MCU) — единственная точка входа в железо для всего остального кода | `IBoard`: `begin()`, `i2c()`, `spi()`, `rcUart()`, `gpsUart()`, `servo(channel)`; `II2CBus`/`ISpiBus`/`IUartPort`/`IServoOutput` по форме зеркалят Wire/SPI/HardwareSerial/Servo | Контракт, за которым может стоять любой MCU | Ничего о конкретной реализации — это чистые интерфейсы |
| `hal/esp32/Esp32Board.h` (+ `Esp32I2CBus.h`, `Esp32SpiBus.h`, `Esp32UartPort.h`, `Esp32ServoOutput.h`) | Единственная сегодня реализация `IBoard` — обёртки над `Wire`/`SPIClass`/`HardwareSerial`/`ESP32Servo`, пины берутся из `Config.h` | `Esp32Board::begin()` (выделяет PWM-таймеры, инициализирует шины i2c/spi один раз) | Все пины и какой `HardwareSerial`/`SPIClass`/`Servo` за каким логическим каналом стоит | Ничего о логике полёта — чистый адаптер API |
| `Config.h` | Выбор платы (`BOARD_ESP32_C3` / `BOARD_ESP32_S3` / `BOARD_ESP32_CLASSIC`) через `#elif` по билд-флагу; все тюнинговые константы и пины (включая SPI и GPS UART) проекта | Пины (`PIN_AILERON_LEFT`, `PIN_SPI_SCK/MISO/MOSI`, `PIN_SPI_CS_ICM42688/BMP388`, `PIN_GPS_RX/TX` и т. п.), `IBUS_BAUDRATE`, `RX_TIMEOUT_US`, `PWM_MIN/CENTER/MAX`, `AILERON_MAX_US`/`ELEVATOR_MAX_US`, `THROTTLE_LOW_US`, `ARM_LOW_TIME_MS`, `FAILSAFE_AILERON/ELEVATOR/THROTTLE`, `THROTTLE_LIMIT_PERCENT`, `THROTTLE_BOOST_TIME_MS`, `DEBUG_INTERVAL_MS` | Какая плата собирается (по макросу из `platformio.ini`) | Ничего о поведении в рантайме — это просто константы |
| `Channels.h` | Имена логических индексов RC-каналов; список слотов автопилота | Константы `AILERON`, `ELEVATOR`, `THROTTLE`, `RUDDER`, `FLAPS`, `AUX_1..AUX_5` (`BOOST` = отдельно), `FEATURE_SLOTS = {CH6, CH7, CH9, CH10}` | Какой индекс массива соответствует какой физической ручке | Что делать с этими значениями |
| `RcChannelState.h` | POD-снимок всех 10 каналов приёмника | `RcChannelState`: `uint16_t` на канал, `get(idx)`/`set(idx, us)`/`reset()` | Безопасные значения по умолчанию (`throttle = PWM_MIN`, остальные = `PWM_CENTER`) | Откуда взялись значения (UART/тест/что угодно) |
| `IBusReceiver.h` | Парсинг протокола iBUS из `IUartPort` в `RcChannelState` | `IBusReceiver(IUartPort&)`, `begin()`, `update()`, `isSignalLost()` (по `RX_TIMEOUT_US`), `getState()` — возвращает `const RcChannelState&` | Формат кадра iBUS, тайминг потери сигнала | Ничего о самолёте: не знает про failsafe-поведение, микшер, ARM, ни про конкретный MCU (пины/формат кадра UART фиксированы в `Esp32UartPort`, не здесь) — только байты → каналы |
| `RcInput.h` | Статические хелперы преобразования PWM-значений | `clamp(us)` — ограничение в `[PWM_MIN, PWM_MAX]`; `centered(us, maximumDeflection, reverse=false)` — линейно отображает `[PWM_MIN, PWM_MAX]` в `[-maximumDeflection, +maximumDeflection]`, с опцией инверсии знака | Диапазоны и центр из `Config.h`; предельное отклонение передаётся вызывающим кодом (не берётся из `Config` само по себе) | Смысл канала (что именно центрируется) |
| `FlightOutputState.h` | POD-контракт "желаемое положение поверхностей" между микшером/автопилотом и физическим выходом | `FlightOutputState`: `aileronLeft`, `aileronRight`, `elevator`, `throttle` (все `uint16_t`, µs) | Ничего, кроме своих 4 полей | Как эти значения были посчитаны и как будут физически выведены |
| `ControlMixer.h` | Обычная (не статическая) функция-метод RC → `FlightOutputState` для ручного режима | `FlightOutputState calculate(const RcChannelState&) const` — вызывается на экземпляре `ControlMixer`; элероны синхронно (зеркально друг другу, через `RcInput::centered(..., Config::AILERON_MAX_US)`) + offset от закрылков (CH5: 0 / 50µs / 100µs), лифт — отображение CH2 через `RcInput::centered(..., Config::ELEVATOR_MAX_US)` | Логику смешивания каналов в выходы | UART, `Servo`, failsafe, время — чистая функция без побочных эффектов |
| `ThrottleManager.h` | CH3 (throttle) + CH8 (boost) → итоговый µs для ESC | `uint16_t update(rc, receiverFailsafe)` — само возвращает готовый итоговый throttle (отдельного геттера нет); применяет обычное ограничение `THROTTLE_LIMIT_PERCENT` (40%) или форсаж на `THROTTLE_BOOST_TIME_MS` (5с) при активации CH8; `isBoostActive()`/`isBoostReady()` — вспомогательные геттеры состояния | Логику ограничения и форсажа газа, состояние "разблокирован/активен" по CH8 | Ничего про `Servo` — отдаёт только число в µs |
| `ArmingManager.h` | Отслеживание состояния ARM | `update(uint16_t throttle, bool receiverFailsafe)`, `isArmed()`; ARM = газ удержан на минимуме (`< THROTTLE_LOW_US`) не менее `ARM_LOW_TIME_MS` (1.5с). Принимает уже извлечённое значение throttle, а не весь `RcChannelState` | Историю положения throttle во времени | **Честно:** `armed` сейчас НЕ блокирует throttle — это только индикатор состояния для совместимости со старым поведением прошивки, не предохранитель |
| `FlightOutputs.h` | Единственный класс, знающий порядок серво-каналов (2 элерона, лифт, ESC) — само железо спрятано за `IBoard`/`IServoOutput` | `FlightOutputs(IBoard&)`, `begin()` (подключает 4 канала через `board.servo(0..3)`; печатает OK/FAIL в Serial по каждому отдельно), `write(FlightOutputState)`, `setFailsafe()` | Порядок и назначение каналов (`ServoChannel::AILERON_LEFT` и т. д. из `hal/IBoard.h`) | Какой MCU/API реально выдаёт PWM — это знает только `Esp32ServoOutput`; физическое состояние сервопривода тоже не определить программно (нет обратной связи по току) |
| `sensors/SensorInterface.h` | Абстрактные интерфейсы датчиков (Dependency Inversion) | `Sensor` (база: `begin()`, `isAvailable()`, `update()`, `getSensorType()`, `printStatus()`); `ImuSensor` (добавляет `getImuData()`→`const ImuData&`, `calibrate()`, `setYaw()`); `BarometerSensor` (добавляет `getBarometerData()`→`const BarometerData&`, `calibrateAltitude()`, `setSeaLevelPressure()`); `MagnetometerSensor` (добавляет `getMagData()`→`const MagData&`, `calibrate()` — offset-only); `GpsSensor` (добавляет `getGpsData()`→`const GpsData&`, `hasFix()`); структуры `ImuData`, `BarometerData`, `MagData`, `GpsData` | Контракт данных по категориям | Ничего о конкретном чипе — реализации в `sensors/*_Sensor.h`/`UbloxM10_Gps.h`, актуальный выбор — в `SensorSelection.h` |
| `sensors/MPU6050_Sensor.h` | Реализация `ImuSensor` для гироскопа+акселерометра MPU6050 (GY-521), I2C | `MPU6050_Sensor(II2CBus&, address=0x68)`, `begin()`, `calibrate()`, `update()`, `getImuData()`, `isAvailable()`; регистры `0x6B/0x1B/0x1C/0x19/0x1A/0x3B` через `II2CBus` (не `Wire` напрямую) | Собственную реализацию чтения регистров MPU6050 (НЕ обёртка над сторонней библиотекой); комплементарный фильтр для roll/pitch (70% гироскоп + 30% акселерометр); yaw — чистый интеграл гироскопа | Что yaw будет медленно уплывать — это ожидаемое поведение, явно отмеченное в коде, а не баг |
| `sensors/ICM42688_Sensor.h` | Реализация `ImuSensor` для ICM-42688-P (плата "601N1"), SPI | `ICM42688_Sensor(ISpiBus&, csPin)`, тот же публичный контракт, что у MPU6050; `WHO_AM_I`=0x75 (ожидает 0x47), банк регистров через `REG_BANK_SEL`=0x76, burst-чтение с `TEMP_DATA1`=0x1D (порядок temp→accel→gyro, не как у MPU6050) | FS_SEL выбран так же, как ±250°/с и ±2g у MPU6050 — те же коэффициенты масштаба (131/16384), комплементарный фильтр скопирован один в один | Ничего специфичного — драйвер написан по структуре MPU6050_Sensor.h намеренно |
| `sensors/BME280_Sensor.h` | Реализация `BarometerSensor` для BME280, I2C | `BME280_Sensor(II2CBus&, address=0x76)`, `begin()`, `calibrateAltitude()`, `update()`, `getBarometerData()`, `isAvailable()` | Собственную реализацию (НЕ `Adafruit_BME280`); приближённую формулу перевода давления в высоту (не полную 26-коэффициентную компенсацию из даташита BME280) | Точную абсолютную высоту — она приближение; относительная скорость подъёма (climb rate) стабильна для ПИД ALT_HOLD, но абсолютная высота — нет. Точная альтернатива — `BMP388_Sensor.h` |
| `sensors/BMP388_Sensor.h` | Реализация `BarometerSensor` для BMP388 (Bosch), SPI, с настоящей компенсацией по датащиту | `BMP388_Sensor(ISpiBus&, csPin)`, тот же контракт, что у BME280; читает 21 байт NVM-калибровки с регистра 0x31, компенсирует давление/температуру по floating-point формуле из датащита §9.3 (temp считается первой — pressure от неё зависит) | Высота считается той же барометрической формулой, что в BME280_Sensor.h — она не специфична для чипа | Ничего не приближает — коэффициенты и формула честно по датащиту Bosch |
| `sensors/QMC5883P_Sensor.h` | Реализация `MagnetometerSensor` для QMC5883P (плата "GY-273"), I2C | `QMC5883P_Sensor(II2CBus&, address)`, `begin()`, `update()`, `getMagData()`, `calibrate()` (offset-only: вращать 15 сек, min/max по осям) | Offset-калибровку (hard-iron), 2D-курс без тilt-компенсации | **⚠️ Регистры/адрес/масштаб — заготовка по схеме QMC5883L, НЕ сверены с датащитом конкретно QMC5883P** — см. `// TODO(verify)` в начале файла, обязательно проверить перед полётом |
| `sensors/UbloxM10_Gps.h` | Реализация `GpsSensor` для u-blox M10 (QUESCAN, UBX-M10050-KB), UART, протокол UBX binary | `UbloxM10_Gps(IUartPort&)`, `begin()` (шлёт UBX-CFG-RATE/CFG-MSG, без ожидания ACK), `update()` (побайтовый парсер UBX-кадров), `getGpsData()`, `hasFix()` | Разбирает только NAV-PVT (класс 0x01, id 0x07, 92 байта) — этого достаточно для координат/высоты/скорости/курса/фикса/спутников/точности разом | `isAvailable()` здесь значит "хотя бы один валидный кадр разобран", а не "ACK по шине" — у GPS/UART нет протокольного ACK, в отличие от I2C |
| `sensors/SensorSelection.h` | **Единственное место, где выбирается, какой конкретно чип каждой категории скомпилирован** | `#define SENSOR_IMU/BARO/MAG/GPS` + `using SelectedImu/Baro/Mag/Gps = ...` | Список всех поддержанных чипов (комментарий в начале файла) и какой конструктор у какого чипа (комментарий у каждой `#if`-ветки) | Ничего о том, как эти классы используются — это делает `main.cpp` |
| `Autopilot.h` | ПИД-регулятор общего назначения + логика 4 режимов полёта | `PID_Controller` (защита от windup, от скачков `dt`); `Autopilot`: `begin()`, `update()`, `setMode()`, `getMode()`, `getRollCorrection()`/`getPitchCorrection()`/`getThrottleCorrection()` (в µs/условных единицах); режимы `MANUAL`/`STABILIZE`/`AUTO_TAKEOFF`/`ALT_HOLD` | Принимает `ImuSensor*`/`BarometerSensor*`/`MagnetometerSensor*`/`GpsSensor*` только через конструктор (все 4 nullable, сеттеров для подмены после создания нет); STABILIZE — ПИД крена/тангажа к 0; AUTO_TAKEOFF — сценарий по времени (1с разгон, 2с тангаж 15°, далее тангаж 10°, газ по фазам 30/60/100%); ALT_HOLD — ПИД газа к высоте на момент включения режима | Если указатель на датчик `nullptr` — просто не даёт коррекций, не падает. Mag/GPS сейчас используются только для телеметрии (`getMagnetometerSensor()`/`getGpsSensor()`) плюс разовой установки начального yaw по компасу в `main.cpp::setup()` — **никакой навигационной логики по GPS нет**, это осознанно вне объёма текущей реализации. **Важно (честно):** `getThrottleCorrection()` считается на каждом режиме, но `FlightController` её **не** прибавляет к итоговому throttle — сейчас реально применяются только `getRollCorrection()`/`getPitchCorrection()` (см. раздел про `FlightController::update()` ниже) |
| `FeatureManager.h` | Назначение 4 функций автопилота на 4 свободных вспомогательных RC-канала | `begin()`, `update(rc)`; читает `Channels::FEATURE_SLOTS = {CH6, CH7, CH9, CH10}`; фильтрует дребезг (изменение `<50µs` игнорируется); вызывает `autopilot->setMode()` | Текущее назначение слот→функция (по умолчанию: CH6=AUTO_TAKEOFF, CH7=ALT_HOLD, CH9=STABILIZE, CH10=MANUAL), может быть изменено на лету через `POST /api/assignfeature` | Внутреннюю логику режимов автопилота — только переключает `setMode()` |
| `WebDebugServer.h` | Веб-дашборд по Wi-Fi (точка доступа) | `begin(port)`, `update()`; SSID `OpenPlane-Debug`, пароль `12345678`, IP `192.168.4.1`; эндпоинты `GET /`, `GET /api/status`, `POST /api/setmode`, `POST /api/setpid`, `POST /api/assignfeature` (полная форма — см. раздел HTTP API) | Публичные геттеры `FlightController`/`Autopilot`/`FeatureManager`/датчиков, агрегирует их в JSON | Ничего не решает и не изменяет напрямую — режим/PID меняет через публичные сеттеры `Autopilot`/`FeatureManager` |
| `FlightController.h` | Единственный оркестратор всей логики полёта, `update()` вызывается из `loop()` | `begin()`, `update()`, геттеры для `DebugLogger`/`WebDebugServer` (текущие каналы, armed, failsafe, boost, выходы) | Порядок вызова всех нижних слоёв и приоритет failsafe (см. раздел ниже) | Детали реализации каждого нижнего слоя — только вызывает их публичные методы |
| `DebugLogger.h` | Печать состояния в Serial, не чаще `Config::DEBUG_INTERVAL_MS` (100мс) и только когда что-то реально изменилось | `update()` собирает кадр (10 каналов + RX/ARM/BOOST + выходы (µs), плюс `Autopilot`/`FeatureManager::printStatus(Print&)`, если указатели переданы) во внутренний буфер и сравнивает с предыдущим — идентичный кадр в Serial не уходит (см. `Config::DEBUG_ONLY_ON_CHANGE`, допуск на дребезг RC/PWM — `DEBUG_CHANGE_DEADBAND_US`) | Публичные геттеры `FlightController`, `Autopilot`, `FeatureManager` | Ничего не решает, только читает и печатает |
| `src/main.cpp` | Composition root — единственная точка сборки | `setup()`: `Serial` → `board.begin()` (шины/PWM-таймеры) → `flightOutputs.begin()` + `setFailsafe()` → `датчики.begin()` (+ `calibrate()`/`calibrateAltitude()` только если `isAvailable()`) → (если есть mag) `magSensor.calibrate()` + разовая установка `imuSensor.setYaw()` по компасу → (если есть GPS) `gpsSensor.begin()` → `autopilot.begin()` → `featureManager.begin()` → `flightController.begin()` → `webDebugServer.begin(0)`. `loop()` (~500Гц, `delay(2)`): `webDebugServer.update()` → `flightController.update()` → `debugLogger.update()` | Создаёт `Esp32Board` и РЕАЛЬНЫЕ экземпляры `SelectedImu`/`SelectedBaro`/`SelectedMag`/`SelectedGps` (алиасы из `SensorSelection.h`, не по именам классов напрямую), `Autopilot`, `FeatureManager` (не заглушки) и связывает их | Логику ни одного из слоёв — только создаёт объекты и вызывает их `begin()`/`update()` в нужном порядке. Не знает, какой конкретно чип скомпилирован за `SelectedImu` и т. п. |

---

## Карта RC-каналов

Источник: `include/Channels.h`. Приёмник — iBUS, 10 каналов, диапазон
1000–2000 µs (`Config::PWM_MIN`/`PWM_MAX`).

| Канал | Имя | Назначение | Диапазон / логика |
|---|---|---|---|
| CH1 | `AILERON` | Крен (элероны) | 1000–2000 µs, центр 1500 |
| CH2 | `ELEVATOR` | Тангаж (лифт) | 1000–2000 µs, центр 1500 |
| CH3 | `THROTTLE` | Газ | 1000 = выкл, 2000 = макс |
| CH4 | `RUDDER` | Рысканье | **сейчас не используется** |
| CH5 | `FLAPS` | Закрылки, 3 положения | `<1250` → убраны (offset 0); `1250–1749` → половина (offset +50µs); `>=1750` → выпущены (offset +100µs) |
| CH6 | `AUX_1` | Свободен, слот автопилота №1 | по умолчанию `AUTO_TAKEOFF` |
| CH7 | `AUX_2` | Свободен, слот автопилота №2 | по умолчанию `ALT_HOLD` |
| CH8 | `BOOST` | Форсаж газа | **используется `ThrottleManager`, НЕ свободен для автопилота** |
| CH9 | `AUX_4` | Свободен, слот автопилота №3 | по умолчанию `STABILIZE` |
| CH10 | `AUX_5` | Свободен, слот автопилота №4 | по умолчанию `MANUAL` |

**Слоты автопилота** (`Channels::FEATURE_SLOTS = {CH6, CH7, CH9, CH10}`)
намеренно обходят CH8, чтобы не конфликтовать с boost. Назначение
слот → функция задаётся по умолчанию в порядке выше, но может быть изменено
на лету через `POST /api/assignfeature`.

Переключатель слота: `<1500µs` = выключено, `>=1500µs` = включено. Если
активно несколько слотов одновременно, побеждает слот с **меньшим индексом**
(CH6 приоритетнее CH7, CH7 приоритетнее CH9, и т. д.).

**Логика BOOST (CH8)**, реализована в `ThrottleManager`:

| Значение CH8 | Состояние |
|---|---|
| `< 1250µs` | Форсаж разблокирован (готов к активации) |
| `>= 1750µs` | Активирует форсаж: throttle = максимум на `THROTTLE_BOOST_TIME_MS` (5 секунд), затем выключается сам |
| после активации | нужно вернуть CH8 в `<1250µs`, чтобы разблокировать форсаж повторно |

---

## Данные датчиков

### `ImuData` (`include/sensors/SensorInterface.h`)

| Поле | Единица | Смысл |
|---|---|---|
| `gyroX`, `gyroY`, `gyroZ` | °/с | Угловая скорость по осям, сырые данные с гироскопа |
| `accelX`, `accelY`, `accelZ` | g | Линейное ускорение по осям |
| `roll`, `pitch`, `yaw` | ° | Углы ориентации после комплементарного фильтра (roll/pitch) и интегрирования (yaw) |
| `temperature` | °C | Температура кристалла датчика |
| `timestamp` | µс | `micros()` с момента старта платы, момент считывания |

### `BarometerData` (`include/sensors/SensorInterface.h`)

| Поле | Единица | Смысл |
|---|---|---|
| `pressure` | Па | Атмосферное давление |
| `temperature` | °C | Температура датчика |
| `altitude` | м | Высота, **относительно точки калибровки** (не абсолютная над уровнем моря) |
| `verticalSpeed` | м/с | Скорость подъёма/снижения (climb rate) |
| `timestamp` | µс | `micros()` с момента старта платы |

### `GpsData` (`include/sensors/SensorInterface.h`)

| Поле | Единица | Смысл |
|---|---|---|
| `latitude`, `longitude` | ° | Координаты (`double`, из UBX-NAV-PVT, 1e-7° разрешение) |
| `altitude` | м | Высота над уровнем моря (hMSL из NAV-PVT) |
| `groundSpeed` | м/с | Путевая скорость |
| `heading` | ° (0..360) | Курс по земле (course over ground) |
| `numSatellites` | — | Число спутников в решении |
| `fixType` | — | 0=нет фикса, 2=2D, 3=3D (как в UBX-NAV-PVT.fixType) |
| `horizontalAccuracy`, `verticalAccuracy` | м | Оценки точности от самого модуля (hAcc/vAcc) |
| `timestamp` | µс | `micros()` на момент разбора кадра |

### `MagData` (`include/sensors/SensorInterface.h`)

| Поле | Единица | Смысл |
|---|---|---|
| `magX`, `magY`, `magZ` | µT | Поле по осям, после вычета offset-калибровки |
| `headingDegrees` | ° (0..360) | Курс из `atan2(magY, magX)`, **без** tilt-компенсации по roll/pitch |
| `timestamp` | µс | `micros()` на момент считывания |

**Кто пишет:** `update()` каждого конкретного драйвера
(`MPU6050_Sensor`/`ICM42688_Sensor` → `ImuData`,
`BME280_Sensor`/`BMP388_Sensor` → `BarometerData`, `QMC5883P_Sensor` →
`MagData`, `UbloxM10_Gps` → `GpsData`). Какой именно класс скомпилирован в
каждой категории — решает `sensors/SensorSelection.h`. Драйверы читают шину
через `II2CBus`/`ISpiBus`/`IUartPort` (см. `include/hal/`), не через
`Wire`/`SPI`/`HardwareSerial` напрямую.

**Кто читает:** `Autopilot::update()` получает `ImuSensor*`/`BarometerSensor*`/
`MagnetometerSensor*`/`GpsSensor*` в конструкторе (все 4 nullable) и читает
`getImuData()`/`getBarometerData()` для расчёта коррекций STABILIZE и
ALT_HOLD; `MagData`/`GpsData` сейчас используются только для телеметрии и
разовой установки начального yaw по компасу (см. `main.cpp::setup()`) — без
навигационной логики. `WebDebugServer` читает все четыре
`getImuData()`/`getBarometerData()`/`getMagData()`/`getGpsData()` +
`isAvailable()` для отдачи в `GET /api/status`.

**`isAvailable()` — что это значит.** Для I2C/SPI-датчиков (IMU, барометр,
магнитометр) это не "объект создан" и не "код скомпилирован с поддержкой
датчика" — это **реальный ответ по шине** (ACK на запрос к регистру). Если
датчик физически не распаян или не отвечает, `isAvailable()` вернёт `false`
при каждом вызове. **Для GPS это другое** — у UART/UBX нет протокольного
ACK на уровне шины, поэтому `UbloxM10_Gps::isAvailable()` означает "хотя бы
один валидный кадр NAV-PVT успешно разобран после `begin()`", а не "модуль
физически отвечает".

**Поведение при отсутствии датчика.** Ничего не падает и не виснет.
`Autopilot` принимает указатели на `ImuSensor`/`BarometerSensor`, которые
могут быть `nullptr`, либо не-`nullptr`, но с `isAvailable() == false` — в
обоих случаях `Autopilot` просто **не даёт коррекций** (`getRollCorrection()`
и другие геттеры возвращают нейтральное значение), а `FlightController`
продолжает работать в чистом ручном режиме через `ControlMixer` и
`ThrottleManager`. Это подтверждено живым тестом на плате: автопилот
скомпилирован и подключён в прошивку, но датчики физически ещё не распаяны
на текущем прототипе — они отвечают `NO_RESPONSE`, и самолёт при этом
продолжает нормально летать на ручном управлении.

`main.cpp` учитывает это в `setup()`: калибровка (`calibrate()`/
`calibrateAltitude()`) вызывается **только если `isAvailable()`**, потому
что калибровка требует неподвижности платформы и не имеет смысла на
несуществующем датчике.

---

## Разбор FlightController::update()

`FlightController::update()` — единственное место, где решается порядок
операций за один тик. Вызывается из `loop()` в `main.cpp` на каждой
итерации (~500 Гц, `delay(2)` между итерациями). Порядок фиксированный и
важен именно в этой последовательности:

1. **`receiver.update()`** — читает UART, парсит очередной кадр iBUS (если
   он есть) в свежий `RcChannelState`. Это первый шаг, потому что все
   остальные решения принимаются на основе актуальных данных приёмника.

2. **`featureManager`/автопилот `update()`** (только если сигнал есть) —
   `FeatureManager` читает состояние вспомогательных каналов и при
   необходимости переключает режим `Autopilot` через `setMode()`; сам
   `Autopilot::update()` пересчитывает коррекции (`rollCorrection`,
   `pitchCorrection`, `throttleCorrection`) на основе текущих данных
   датчиков и текущего режима.

3. **Проверка failsafe** — если `receiver.isSignalLost()` (нет валидного
   кадра дольше `RX_TIMEOUT_US` = 500 мс): немедленно `armed = false`,
   throttle выставляется в failsafe-значение (1000µs, выкл), вызывается
   `outputs.setFailsafe()`, и функция **сразу завершается (`return`)**, не
   доходя до микшера, ARM и автопилота. Это **абсолютный приоритет**:
   независимо от того, что происходило на предыдущих двух шагах, потеря
   сигнала с приёмника обязана немедленно перевести самолёт в безопасное
   состояние (элероны/лифт → нейтраль 1500µs, газ → 1000µs), а не
   продолжать выполнять устаревшие или мусорные команды. Именно поэтому
   проверка стоит сразу после чтения приёмника и до всей остальной логики,
   а не в конце функции.

4. **`arming.update(throttle, false)`** — обновляет состояние ARM (газ на
   минимуме дольше `ARM_LOW_TIME_MS` → armed = true). Выполняется только
   если сигнал есть (шаг 3 не прервал выполнение).

5. **`mixer.calculate()`** — `ControlMixer` считает базовый
   `FlightOutputState` из текущего `RcChannelState`: элероны зеркально
   синхронно + offset закрылков, лифт — отображение CH2. Это чистый
   ручной расчёт, не зависящий от armed/автопилота.

6. **Если armed** — `FlightController` прибавляет к выходу микшера из шага
   5 коррекции **крена и тангажа**:
   `aileronLeft += pitchCorr - rollCorr`, `aileronRight += pitchCorr +
   rollCorr`, `elevator += pitchCorr` (то есть `pitchCorrection` влияет
   сразу на оба элерона и на лифт, а `rollCorrection` — дифференциально
   на элероны), с последующим `constrain()` в `[PWM_MIN, PWM_MAX]`.
   **Честно:** `getThrottleCorrection()` в этом месте кода **не
   используется** — коррекция газа, которую считают режимы AUTO_TAKEOFF и
   ALT_HOLD, сейчас нигде не суммируется с итоговым throttle перед ESC.
   Она доступна только для чтения/отображения (например, в
   `GET /api/status` как `autopilot.throttleCorr`). Условие "только если
   armed" для roll/pitch-коррекций — намеренное ограничение: автопилот не
   должен вмешиваться в управление, пока самолёт не прошёл процедуру ARM.

7. **`throttle.update(rc, false)`** — `ThrottleManager` берёт положение
   газа из текущего `RcChannelState` (шаг 6 на throttle не влияет, см.
   выше) и применяет ограничение 40% хода или форсаж по CH8; итоговое
   значение записывается в `output.throttle`.

8. **`outputs.write()`** — `FlightOutputs` физически выставляет PWM на 4
   канала (2 элерона, лифт, ESC) через `Servo`/`ESP32Servo`.

После `update()` `FlightController` предоставляет публичные геттеры
(текущие каналы, armed, failsafe, boost, итоговые выходы), которые читают
`DebugLogger` и `WebDebugServer` — сам `FlightController` не знает об их
существовании.

---

## HTTP API веб-дашборда

Реализация: `include/WebDebugServer.h`. Точка доступа Wi-Fi:

| Параметр | Значение |
|---|---|
| SSID | `OpenPlane-Debug` |
| Пароль | `12345678` |
| IP точки доступа | `192.168.4.1` |

### `GET /api/status`

Один агрегированный эндпоинт, отдаёт JSON со всем состоянием борта.
Поля `attached`/`available` присутствуют **всегда** — так дашборд честно
показывает разницу между "этого нет в схеме" и "есть в схеме, но не
отвечает", вместо того чтобы молча пропускать данные. Пример ниже
соответствует состоянию сразу после старта (сигналов RC ещё не было,
датчики не отвечают, ни один слот автопилота не активен):

```json
{
  "rc": [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500],
  "armed": false,
  "failsafe": false,
  "boost": false,
  "outputs": {
    "aileronLeft":  { "us": 1500, "attached": true },
    "aileronRight": { "us": 1500, "attached": true },
    "elevator":     { "us": 1500, "attached": true },
    "esc":          { "us": 1000, "attached": true }
  },
  "imu": {
    "attached": true,
    "available": false
  },
  "baro": {
    "attached": true,
    "available": false
  },
  "autopilot": {
    "attached": true,
    "mode": 0,
    "modeName": "MANUAL",
    "desiredRoll": 0.0,
    "desiredPitch": 0.0,
    "targetAlt": 0.0,
    "rollCorr": 0.0,
    "pitchCorr": 0.0,
    "throttleCorr": 0.0,
    "kpRoll": 0.05, "kiRoll": 0.01, "kdRoll": 0.02,
    "kpPitch": 0.05, "kiPitch": 0.01, "kdPitch": 0.02
  },
  "features": {
    "ch": [6, 7, 9, 10],
    "feature": [1, 2, 3, 4],
    "active": [false, false, false, false],
    "active_mode": 0
  }
}
```

Важная деталь формата: когда `imu.available` (или `baro.available`) равно
`false`, поля `roll`/`pitch`/`yaw` (соответственно `altitude`/`climb`) в
JSON **отсутствуют вовсе**, а не приходят нулями — сервер добавляет их в
ответ только внутри `if (available)`. Клиентский JS-код в `GET /` это
учитывает и не пытается их читать, пока `available` не станет `true`.

| Раздел | Поле | Тип | Смысл |
|---|---|---|---|
| — | `rc` | `number[10]` | Массив всех 10 RC-каналов в µs, по порядку CH1..CH10 |
| — | `armed` | `bool` | Текущее состояние ARM |
| — | `failsafe` | `bool` | Активен ли failsafe (потеря сигнала) |
| — | `boost` | `bool` | Активен ли форсаж газа |
| `outputs.*` | `us` | `number` | Текущее выставленное значение PWM в µs |
| `outputs.*` | `attached` | `bool` | Подключён ли программно данный выход (`Servo::attach()` успешен) |
| `imu` | `attached` | `bool` | Объект `MPU6050_Sensor` создан и подключён в прошивку (передан в `Autopilot`) |
| `imu` | `available` | `bool` | Датчик реально отвечает по I2C (ACK) |
| `imu` | `roll`/`pitch`/`yaw` | `number` (°) | Присутствуют в JSON, только если `available: true`; при `false` этих ключей в объекте нет вовсе |
| `baro` | `attached`/`available` | `bool` | Аналогично `imu` |
| `baro` | `altitude` | `number` (м) | Высота относительно калибровки; присутствует только при `available: true` |
| `baro` | `climb` | `number` (м/с) | Вертикальная скорость; присутствует только при `available: true` |
| `autopilot` | `attached` | `bool` | Объект `Autopilot` создан |
| `autopilot` | `mode` | `number` (0–3) | Числовой код текущего режима (`AutopilotMode`) |
| `autopilot` | `modeName` | `string` | `"MANUAL"`/`"STABILIZE"`/`"AUTO_TAKEOFF"`/`"ALT_HOLD"` |
| `autopilot` | `desiredRoll`/`desiredPitch` | `number` (°) | Целевые углы текущего режима |
| `autopilot` | `targetAlt` | `number` (м) | Целевая высота для ALT_HOLD |
| `autopilot` | `rollCorr`/`pitchCorr` | `number` (µs) | Текущие коррекции, реально прибавляемые `FlightController`-ом к элеронам/лифту при armed |
| `autopilot` | `throttleCorr` | `number` | Коррекция газа, которую считают AUTO_TAKEOFF/ALT_HOLD — **отображается для диагностики, но сейчас не суммируется** с итоговым throttle в `FlightController::update()` (см. раздел про `update()` выше) |
| `autopilot` | `kpRoll`..`kdPitch` | `number` | Текущие коэффициенты ПИД по крену и тангажу |
| `features` | `ch` | `number[4]` | Номера каналов слотов (по умолчанию `[6, 7, 9, 10]`) |
| `features` | `feature` | `number[4]` | Числовой код функции автопилота на каждом слоте: `0=DISABLED, 1=AUTO_TAKEOFF, 2=ALT_HOLD, 3=STABILIZE, 4=MANUAL` (значения `FeatureType`, не строки) |
| `features` | `active` | `bool[4]` | Активен ли (>=1500µs) каждый слот прямо сейчас |
| `features` | `active_mode` | `number` (0–3) | Код режима (`AutopilotMode`, те же коды, что и в `POST /api/setmode`), который в итоге победил с учётом приоритета по меньшему индексу слота |

### `POST /api/setmode`

Устанавливает режим автопилота напрямую (в обход RC-слотов).

```json
{ "mode": 1 }
```

Значения `mode`: `0 = MANUAL`, `1 = STABILIZE`, `2 = AUTO_TAKEOFF`,
`3 = ALT_HOLD`.

### `POST /api/setpid`

Задаёт коэффициенты ПИД по крену и тангажу. Любое поле можно не указывать —
оно останется текущим (частичное обновление).

```json
{ "kpRoll": 1.2, "kiRoll": 0.05, "kdRoll": 0.3 }
```

Полный набор допустимых полей: `kpRoll`, `kiRoll`, `kdRoll`, `kpPitch`,
`kiPitch`, `kdPitch`.

### `POST /api/assignfeature`

Переназначает, какая функция автопилота висит на каком RC-слоте.

```json
{ "slot": 0, "feature": 2 }
```

`slot`: индекс в `Channels::FEATURE_SLOTS` (`0` = CH6, `1` = CH7, `2` = CH9,
`3` = CH10). `feature`: код функции (`FeatureType` из `FeatureManager.h`):
`0 = DISABLED`, `1 = AUTO_TAKEOFF`, `2 = ALT_HOLD`, `3 = STABILIZE`,
`4 = MANUAL`.

### `GET /`

Отдаёт HTML-дашборд: живые бары всех 10 RC-каналов, статус каждого выхода
и датчика (`attached`/`available`), кнопки переключения режимов
автопилота, форма для правки ПИД, интерфейс назначения RC-слотов на
функции автопилота. Дашборд опрашивает `GET /api/status` каждые 200мс и
дергает POST-эндпоинты по действиям пользователя — вся логика на
JS-стороне, сервер только отдаёт статичный HTML и обслуживает JSON API.

---

## Выбор платы и распиновка

Переключение платы — одна опция сборки в `platformio.ini`, макрос платы
транслируется в `include/Config.h` через `#elif`.

| Команда | `board` (platformio.ini) | Макрос (Config.h) | Статус |
|---|---|---|---|
| `pio run -e esp32-c3` | `esp32-c3-devkitm-1` | `BOARD_ESP32_C3` | **Единственная плата, реально прошитая и облётанная на стенде.** Используется по умолчанию (`default_envs = esp32-c3`) |
| `pio run -e esp32-s3` | `esp32-s3-devkitc-1` | `BOARD_ESP32_S3` | Планируемый основной лётный контроллер. Пины подобраны по документации чипа, **не проверены на реальном железе** |
| `pio run -e esp32-dev` | `esp32dev` | `BOARD_ESP32_CLASSIC` | Обычная классическая ESP32 38-pin, для стенда/отладки. **Тоже не облётана на железе** |

### Распиновка по платам

| Назначение | ESP32-C3 (облётана) | ESP32-S3 (не облётана) | ESP32 classic (не облётана) |
|---|---|---|---|
| Элерон левый | GPIO5 | GPIO4 | GPIO13 |
| Элерон правый | GPIO4 | GPIO5 | GPIO14 |
| Лифт | GPIO6 | GPIO6 | GPIO27 |
| ESC (газ) | GPIO7 | GPIO7 | GPIO26 |
| iBUS RX | GPIO8 | GPIO17 | GPIO16 |
| I2C SDA | GPIO1 | GPIO8 | GPIO21 |
| I2C SCL | GPIO3 | GPIO9 | GPIO22 |
| SPI SCK / MISO / MOSI | GPIO0 / GPIO10 / GPIO20 | GPIO12 / GPIO13 / GPIO11 | GPIO18 / GPIO19 / GPIO23 |
| SPI CS (ICM42688 / BMP388) | GPIO21 / GPIO2 ⚠️ | GPIO10 / GPIO21 | GPIO32 / GPIO33 |
| GPS UART RX / TX | GPIO9 ⚠️ / **нет пина (-1)** | GPIO15 / GPIO16 | GPIO4 / GPIO17 |

Заметьте: у ESP32-C3 элероны на GPIO4/GPIO5 переставлены местами
относительно ESP32-S3 (лево/право поменяны) — это так в актуальном
`Config.h`, при переносе прошивки между этими платами физически проверяйте
направление элеронов на столе перед первым включением газа.

**⚠️ ESP32-C3: пинов физически не хватает на полный комплект.** После
7 уже занятых пинов (ailerons/elevator/esc/ibus/i2c) свободно ровно 6 GPIO
(`0, 2, 9, 10, 20, 21`), а SPI (5 пинов) + GPS UART (2 пина) требуют 7.
Поэтому `PIN_SPI_CS_BMP388` и `PIN_GPS_RX` посажены на strapping-пины
(GPIO2/GPIO9 — тот же класс риска, что уже принят для `PIN_IBUS=8`), а
`PIN_GPS_TX` не определён вовсе (`-1`) — GPS на C3 работает только на
приём, без отправки UBX-CFG (см. `Config.h` и заголовок `UbloxM10_Gps.h`).
Если нужен GPS с полной настройкой — используйте `esp32-s3`, либо
освободите пин на C3, выбрав I2C-барометр (BME280) вместо SPI (BMP388) в
`SensorSelection.h`. Это неподтверждённая раскладка — сверяйте с реальной
распиновкой вашей конкретной платы SuperMini перед пайкой.

Чтобы поменять пины для конкретной платы — правьте блок `#elif
defined(BOARD_ESP32_...)` в `include/Config.h`, добавлять новый `[env:...]`
в `platformio.ini` для этого не нужно.

---

## Как добавить новый датчик

Два разных сценария — почти всегда нужен только первый:

### A) Ещё один чип **той же категории** (например, второй вариант барометра)

Это самый частый случай — интерфейс категории уже существует
(`ImuSensor`/`BarometerSensor`/`MagnetometerSensor`/`GpsSensor` в
`include/sensors/SensorInterface.h`), нужен только новый класс:

1. Создайте `include/sensors/<Имя>_Sensor.h`, реализующий обязательные
   методы базового `Sensor` (`begin()`, `isAvailable()`, `update()`,
   `getSensorType()`, `printStatus()`) плюс методы своего интерфейса
   категории. Конструктор берёт ссылку на нужную шину — `II2CBus&`
   (I2C), `ISpiBus&` + свой CS-пин (SPI), либо `IUartPort&` (UART) — из
   `include/hal/`, **не** `Wire`/`SPI`/`HardwareSerial` напрямую (см. слой
   HAL в начале документа). `isAvailable()` обязан отражать **реальный
   ответ железа** (ACK по шине для I2C/SPI; для UART/GPS — свой критерий,
   см. `UbloxM10_Gps.h`), а не факт создания объекта.
2. Пишите доступ к регистрам напрямую через переданную шину, как это
   сделано в `MPU6050_Sensor.h`/`ICM42688_Sensor.h`/`BMP388_Sensor.h`,
   если только не подключаете стороннюю библиотеку осознанно (тогда
   добавьте `lib_deps` в `platformio.ini` и явно упомяните это — проект до
   сих пор избегал сторонних библиотек для датчиков намеренно).
3. Добавьте новую ветку в `include/sensors/SensorSelection.h`: новый
   `#define SENSOR_<КАТЕГОРИЯ>_<ИМЯ>`, новую `#elif` ветку с `#include` и
   `using Selected... = <Имя>_Sensor`, с комментарием, каким должен быть
   вызов конструктора в `main.cpp` (I2C-адрес vs SPI CS vs UART-ссылка).
   Допишите чип в список в шапке файла.
4. **`main.cpp` в этом сценарии почти не трогается** — только если меняете
   активный `#define SENSOR_<КАТЕГОРИЯ>` на новый чип, тогда сверьте вызов
   конструктора с комментарием у ветки (аргументы отличаются для I2C/SPI/
   UART датчиков).
5. Соберите (`pio run -e esp32-c3 -e esp32-s3 -e esp32-dev`) — в том числе
   с новым `#define`, выставленным активным, чтобы поймать несовпадение
   конструктора сразу, а не когда кто-то реально переключит датчик. Если у
   вас физически распаян чип — проверьте на реальной плате перед тем как
   считать задачу готовой.

### B) Совсем новая категория датчика (такой ещё не было — GPS/mag уже есть)

1. Добавьте новую POD-структуру данных и абстрактный интерфейс в
   `include/sensors/SensorInterface.h` по образцу `GpsSensor`/`GpsData`
   или `MagnetometerSensor`/`MagData`.
2. Дальше — как в сценарии A (новый класс, новая ветка в
   `SensorSelection.h`), плюс:
3. Если датчик должен участвовать в автопилоте — добавьте nullable
   указатель в конструктор `Autopilot` по аналогии с `MagnetometerSensor*`/
   `GpsSensor*` (сеттеров для замены датчика после создания нет; указатель
   может быть `nullptr` — `Autopilot` обязан не давать коррекций/эффектов
   в этом случае, а не падать). Не добавляйте новую логику режимов полёта
   заодно, если задача — просто "довести провода" (так сделано для GPS:
   данные доступны для телеметрии, навигационной логики нет).
4. Добавьте новые поля в `GET /api/status` в `WebDebugServer.h` по
   аналогии с `imu`/`baro`/`mag`/`gps` — обязательно с парой
   `attached`/`available`, и с остальными полями только внутри
   `if (available)`, как это сделано сейчас.
5. Соберите и протестируйте так же, как в сценарии A.

### Если нужна ещё и новая шина/периферия (например, CAN)

Это отдельная, более редкая задача — добавьте новый абстрактный интерфейс
в `include/hal/` (по образцу `II2CBus`/`ISpiBus`/`IUartPort`) и его
реализацию в `include/hal/esp32/`, затем доступ к ней через `IBoard`.
Датчики выше по стеку так и останутся написанными против интерфейса, не
против конкретного MCU.

---

## Как добавить новый режим автопилота

1. Добавьте новое значение в перечисление режимов в `Autopilot.h` (после
   существующих `MANUAL`/`STABILIZE`/`AUTO_TAKEOFF`/`ALT_HOLD`).
2. Реализуйте логику режима внутри `Autopilot::update()` — по аналогии с
   существующими: STABILIZE использует `PID_Controller` по крену/тангажу,
   AUTO_TAKEOFF — сценарий по времени через `millis()`, ALT_HOLD — ПИД газа
   по высоте от `BarometerSensor`. Новый режим обязан корректно вести себя,
   если нужный ему датчик недоступен (`isAvailable() == false` или
   указатель `nullptr`) — деградировать до отсутствия коррекции, а не
   падать и не использовать неинициализированные данные.
3. Обновите `getRollCorrection()`/`getPitchCorrection()`/
   `getThrottleCorrection()`, если новый режим использует новые оси
   коррекции — либо переиспользуйте существующие геттеры, если новых осей
   не нужно. Если режим должен управлять газом через `FlightController`, а
   не только отображаться в API — учтите, что сейчас `getThrottleCorrection()`
   никем не суммируется с итоговым throttle (см. "Известные ограничения"),
   и эту связь тоже придётся добавить в `FlightController::update()`.
4. Если режим должен быть доступен через RC-слоты — убедитесь, что
   `FeatureManager.h` знает про новый код режима (список
   feature-кодов, которые можно назначить на `Channels::FEATURE_SLOTS`).
5. Добавьте `modeName` для нового режима в `WebDebugServer.h`
   (`GET /api/status` → `autopilot.modeName`), а также кнопку/пункт в
   HTML-дашборде (`GET /`), чтобы режим можно было включить и увидеть
   вручную без RC-передатчика.
6. Соберите (`pio run -e esp32-c3`) и протестируйте на стенде сначала без
   мотора/пропеллера — новый режим меняет коррекции, которые прибавляются
   к выходу микшера только при `armed`.

---

## Как добавить новую плату

1. Добавьте новый блок `[env:<имя>]` в `platformio.ini` по образцу
   существующих трёх (`platform = espressif32`, нужный `board =`, свой
   `build_flags` с новым уникальным макросом `-D BOARD_ESP32_<ИМЯ>`,
   унаследовав общие флаги через `${env.build_flags}`).
2. Добавьте соответствующий `#elif defined(BOARD_ESP32_<ИМЯ>)` блок в
   `include/Config.h` с пинами для всех обязательных сигналов: элерон
   левый/правый, лифт, ESC, iBUS RX, I2C SDA/SCL, SPI SCK/MISO/MOSI +
   2 CS (ICM42688/BMP388), GPS UART RX/TX — сверяйтесь с распиновкой
   конкретного чипа/девборды, не копируйте пины другой платы бездумно.
   Посчитайте бюджет GPIO заранее (сколько пинов реально свободно после
   flash/USB/strapping) — на ESP32-C3 SuperMini их физически не хватает
   на полный комплект (см. предупреждение в разделе распиновки выше),
   проверьте, что у вашей платы такой проблемы нет или заложите тот же
   компромисс (GPS без TX, CS на strapping-пине).
3. Не трогайте `default_envs`, если новая плата ещё не проверена на
   реальном железе — она должна быть доступна через явный `-e <имя>`, но
   не становиться платой по умолчанию, пока не облётана (см. как это
   сделано для `esp32-s3`/`esp32-dev` сейчас).
4. Соберите: `pio run -e <имя>`. Если сборка проходит, но на руках нет
   физической платы — явно пометите в коммите/PR, что распиновка не
   проверена на железе (так же как это честно отмечено для ESP32-S3 и
   classic ESP32 в текущем репозитории).
5. После первой реальной прошивки и облёта — обновите пометку "не
   облётана" на "облётана" в документации и `README.md`.

---

## Команды сборки/заливки/монитора

| Плата | Сборка | Заливка | Монитор |
|---|---|---|---|
| ESP32-C3 (по умолчанию, облётана) | `pio run -e esp32-c3` | `pio run -e esp32-c3 -t upload` | `pio device monitor -b 115200` |
| ESP32-S3 | `pio run -e esp32-s3` | `pio run -e esp32-s3 -t upload` | `pio device monitor -b 115200` |
| ESP32 classic 38-pin | `pio run -e esp32-dev` | `pio run -e esp32-dev -t upload` | `pio device monitor -b 115200` |

Скорость монитора — 115200 (совпадает с `Config::IBUS_BAUDRATE` только по
совпадению значения, это отдельная, независимая настройка отладочного
порта `Serial`, не UART приёмника iBUS).

Где что лежит:

- `platformio.ini` — окружения сборки, `lib_deps` (`madhephaestus/ESP32Servo`
  — единственная внешняя зависимость проекта), общие build flags.
- `include/*.h` — вся логика (см. таблицу файлов выше).
- `src/main.cpp` — единственный `.cpp`-файл с точкой входа, composition
  root.
- `test/` — зарезервировано под PlatformIO Unit Testing (`pio test`); на
  данный момент в папке лежит только стандартный boilerplate-`README` от
  PlatformIO, реальных тестов пока нет.

---

## Известные ограничения

Перечислено честно, без приукрашивания — это важно понимать перед тем как
дорабатывать соответствующие модули:

- **IMU/барометр физически не распаяны на текущем прототипе.** Код в
  `MPU6050_Sensor.h`/`BME280_Sensor.h` подключён и работает (проверено
  живым тестом: `isAvailable()` честно возвращает `NO_RESPONSE`), но
  автопилот пока не испытан в реальном полёте. Плата продолжает нормально
  летать чистым ручным управлением, если датчиков нет — это заложенное
  поведение, а не временный костыль.
- **`Autopilot::getThrottleCorrection()` сейчас ни на что не влияет.**
  AUTO_TAKEOFF и ALT_HOLD считают коррекцию газа и отдают её через
  `getThrottleCorrection()` (видна в `GET /api/status` как
  `autopilot.throttleCorr`), но `FlightController::update()` прибавляет к
  выходу микшера только `getRollCorrection()`/`getPitchCorrection()` (к
  элеронам и лифту) — throttleCorrection нигде не суммируется с итоговым
  throttle перед `ThrottleManager`/ESC. То есть на практике управление
  газом через AUTO_TAKEOFF/ALT_HOLD сейчас не долетает до ESC, только
  крен/тангаж.
- **Высота с BME280 — приближение.** Формула перевода давления в высоту не
  использует полную 26-коэффициентную компенсацию из даташита конкретного
  чипа, поэтому абсолютная высота не точна. Относительная скорость подъёма
  (climb rate) достаточно стабильна для работы ПИД ALT_HOLD, но не
  полагайтесь на абсолютное значение `altitude` как на точную величину.
- **`FlightOutputs::begin()` не проверяет физическое подключение серво.**
  `attached: true` означает только, что GPIO/таймер выделен программно
  (`Servo::attach()` не вернул ошибку) — обратной связи по току нет, узнать
  программно, стоит ли реальный сервопривод на этом пине, невозможно.
- **`ArmingManager` не блокирует газ.** `armed` — это индикатор состояния
  для совместимости со старым поведением прошивки, а не предохранитель.
  Единственный реальный предохранитель по газу — failsafe при потере
  сигнала приёмника (шаг 3 в `FlightController::update()`).
- **ESP32-S3 и классическая ESP32 38-pin — распиновка не облётана на
  реальном железе.** Пины подобраны по документации чипов, но ни разу не
  проверялись на физической плате в полёте или даже на стенде.
- **QMC5883P_Sensor.h — регистры/адрес/масштаб не сверены с датащитом.**
  Значения в файле — заготовка по типовой схеме более распространённого
  QMC5883L (тот же чип-семейство, но другая версия), явно помечены
  `// TODO(verify)`. Не полагайтесь на них без проверки по датащиту
  конкретно QMC5883P и без реальной калибровки на физическом датчике.
- **ESP32-C3 SuperMini: GPS без отправки конфигурации.** Из-за нехватки
  GPIO (см. раздел про распиновку) `PIN_GPS_TX` на C3 не определён — GPS-
  модуль работает только на приём, `UbloxM10_Gps` не может отправить
  UBX-CFG-RATE/CFG-MSG, и модуль отдаёт то, что настроено на нём по
  заводским настройкам (обычно NMEA на 1Гц, не UBX/25Гц). Полноценный GPS
  сейчас реалистичен только на ESP32-S3.
- **⚠️ `Esp32Board` жёстко создаёт GPS-порт на `HardwareSerial(2)` —
  на ESP32-C3 такого UART физически нет (у C3 их всего два, 0 и 1).**
  Это отдельная и более серьёзная проблема, чем отсутствие `PIN_GPS_TX`
  выше: если включить `SENSOR_GPS_UBLOX_M10` и собрать под `esp32-c3`,
  `gpsUart().begin()` внутри `UbloxM10_Gps::begin()` обратится к
  несуществующему `UART_NUM_2`. Сейчас это не проявляется только потому,
  что GPS выключен по умолчанию (`SensorSelection.h`) — перед включением
  GPS на C3 нужно перевести `Esp32Board` на реально существующий номер
  UART для этой платы.
- **`QMC5883P_Sensor::calibrate()` блокирует выполнение на 15 секунд** в
  цикле `while`/`delay()`, а `main.cpp::setup()` вызывает её безусловно,
  когда магнитометр включён. На ядрах arduino-esp32 3.x/IDF5.x у
  `setup()`/`loop()` есть Task Watchdog с таймаутом порядка 5 секунд —
  такая калибровка без `yield()`/сброса вотчдога рискует словить
  panic/перезагрузку прямо во время калибровки. Сейчас не проявляется,
  потому что магнитометр выключен по умолчанию.
- **`UbloxM10_Gps::hasValidFrame` выставляется в `true` при первом
  разобранном NAV-PVT кадре и никогда не сбрасывается обратно.** Если
  модуль перестанет присылать данные (потеря питания, обрыв провода),
  `isAvailable()`/`hasFix()` продолжат честно врать, что GPS на связи, а
  `getGpsData()` — отдавать последние (устаревшие) координаты.
- **Yaw по магнитометру устанавливается только один раз при старте.**
  `main.cpp::setup()` зовёт `imuSensor.setYaw(magSensor.getMagData().headingDegrees)`
  один раз после калибровки, если магнитометр доступен — дальше yaw ведёт
  только гироскоп (как и раньше) и будет медленно уплывать в течение
  полёта. Непрерывной sensor-fusion между IMU и магнитометром нет.
- **GPS не участвует в навигации.** `UbloxM10_Gps`/`GpsData` подключены к
  `Autopilot` только для телеметрии (`GET /api/status`, `printStatus()`) и
  разового yaw-init — нет режима полёта, который использует координаты/
  скорость GPS для управления (RTL, waypoint и т. п. — см. `ROADMAP.md`).
- **Лицензия проекта пока не выбрана.** Не указывайте конкретную лицензию
  в коде, коммитах или документации, пока это не решено явно (см.
  `README.md`, "License will be added later").
- **История финансирования/юридическая форма проекта неизвестна** — не
  придумывайте детали, которых нет в публичных материалах репозитория.
- **Первый прототип планера уже летал, но выявлены механические проблемы**:
  недостаточная прочность крепления мотора, недостаточная прочность крыла
  (нужно усиление карбоном), нужна донастройка сервоприводов. Это не
  относится к прошивке напрямую, но важно понимать контекст: часть багов,
  которые выглядят "программными" на стенде, физически проявляются иначе в
  реальном полёте из-за механики, а не кода.

---

## Как вносить изменения

Рабочая конвенция этого проекта, сложившаяся на практике (не формальность):

- **Маленькие коммиты.** Один логический шаг — один коммит. Легче
  откатить, легче ревьюить, легче найти регрессию бисектом.
- **Собирайте после каждого изменения.** `pio run -e esp32-c3` (это
  единственная реально облётанная плата — проверяйте сборку на ней в
  первую очередь) после любой правки в `include/*.h` или `src/main.cpp`,
  прежде чем переходить к следующему шагу. Ошибка компиляции, найденная
  сразу после маленького изменения, находится за секунды; та же ошибка,
  найденная после десяти изменений, может стоить часов.
- **Не выдумывайте API библиотек.** Прежде чем полагаться на конкретный
  метод/сигнатуру `Servo`/`ESP32Servo`/`Wire`/`HardwareSerial`, **проверьте
  установленные исходники фреймворка** (`framework-arduinoespressif32` в
  папке пакетов PlatformIO, обычно
  `~/.platformio/packages/framework-arduinoespressif32/`) — версия API на
  ESP32 (особенно C3 с его архитектурой RISC-V и другим набором периферии)
  местами отличается от классических Arduino-плат, и документация в
  интернете нередко описывает не ту версию библиотеки, что реально
  установлена в проекте.
- **Не приукрашивайте статус модулей в комментариях/документации.** Если
  что-то не проверено на железе (как ESP32-S3, ESP32 classic, автопилот в
  полёте) или недоделано (как `throttleCorrection`, который пока никуда не
  подключён) — так и пишите прямым текстом, по образцу уже существующих
  пометок в этом документе и в `Config.h`/`platformio.ini`.
- **Слой не должен знать больше, чем ему положено.** Если правка требует,
  чтобы, например, `IBusReceiver` начал знать про `ArmingManager` — это
  сигнал, что логику нужно поднять на уровень `FlightController`, а не
  протаскивать зависимость вниз по архитектуре.
- **Меняя контракт данных** (`FlightOutputState`, `ImuData`,
  `BarometerData`, JSON-формат `GET /api/status`) — обновляйте всех
  потребителей в том же коммите или явно указывайте в сообщении коммита,
  какие потребители ещё предстоит обновить.

---
