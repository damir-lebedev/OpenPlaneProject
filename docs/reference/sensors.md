# SENSORS — датчики

[← Справочник](README.md)

Датчики построены в три уровня:

1. **Интерфейсы категорий** (`SensorInterface.h`) — то, что видят `Autopilot`,
   `ArmingManager` и телеметрия.
2. **Базовые классы категорий** (`ImuSensorBase`, `BarometerBase`,
   `MagnetometerBase`) — всё общее: калибровки, фильтры, поворот осей, знаки,
   счёт ошибок, хранение в NVS. Паттерн Template Method.
3. **Драйверы чипов** — только регистры и формулы из даташита. Получают
   `IRegisterDevice&` (шину не различают) или `IUartPort&`.

Какой чип скомпилирован — решает `SensorSelection.h`.

---

## Интерфейсы и структуры данных

**Файл:** `sensors/SensorInterface.h`

### Структуры

| Структура | Поля |
|---|---|
| `ImuData` | `gyroX/Y/Z` °/с (крен + правое крыло вниз, тангаж + нос вверх, рысканье + нос вправо); `accelX/Y/Z` g (оси самолёта X к носу, Y влево, Z вверх); `roll` (−180..180), `pitch` (−90..90), `yaw` (−180..180, интеграл гироскопа) °; `temperature` °C; `timestamp` мкс |
| `BarometerData` | `pressure` Па; `temperature` °C; `altitude` м **относительно точки калибровки**; `verticalSpeed` м/с; `timestamp` мкс |
| `MagData` | `magX/Y/Z` мкТл после hard-iron калибровки в осях самолёта; `headingDegrees` 0..360 (без компенсации наклона); `timestamp` |
| `GpsData` | `latitude`, `longitude` (double, °); `altitude` м MSL; `groundSpeed` м/с; `heading` 0..360; `numSatellites`; `fixType` (0 нет, 2 — 2D, 3 — 3D); `horizontalAccuracy`, `verticalAccuracy` м; `timestamp` |

### `Sensor` (интерфейс)

| Метод | Описание |
|---|---|
| `bool begin()` | Опознать и настроить чип; `true` — датчик работает |
| `bool isAvailable() const` | Подключён и отвечает **сейчас** |
| `void update()` | Вызывать каждый такт; сам решает, пора ли читать |
| `const char* getSensorType() const` | Имя для лога |
| `void printStatus() const` | Строка диагностики (консоль `s`) |

### `ImuSensor : Sensor`

| Метод | Описание |
|---|---|
| `const ImuData& getImuData() const` | Последние данные |
| `void calibrate()` | Калибровка гироскопа (неподвижно) + предполётная проверка |
| `void setYaw(float)` | Задать курс (например, по компасу при старте) |
| `virtual void calibrateOrientation()` | Калибровка установки платы (по умолчанию не поддерживается) |
| `virtual const char* getPreflightProblem() const` | Проблема предполётной проверки или `nullptr` (по умолчанию `nullptr`) |

### `BarometerSensor : Sensor`

`getBarometerData()`, `calibrateAltitude()` (текущая высота = 0),
`setSeaLevelPressure(Па)`.

### `MagnetometerSensor : Sensor`

`getMagData()`, `calibrate()` (hard-iron: вращать 15 с).

### `GpsSensor : Sensor`

`getGpsData()`, `hasFix()` (есть хотя бы 2D-фикс).

---

## `AirspeedSensor`

**Файл:** `sensors/airspeed/AirspeedSensor.h` · **Вид:** интерфейс · **Реализация:** `PitotDualBaroAirspeed`

`AirspeedData { differentialPressurePa, indicatedMs, trueMs, airDensity, timestamp }`:
перепад после нуля и фильтра, приборная скорость (ρ0 = 1.225), истинная (ρ
по статике и температуре), плотность. Методы: `getAirspeedData()`,
`calibrateZero()` (начать обнуление заново), `isZeroing()`.

## `PitotDualBaroAirspeed`

**Файл:** `sensors/airspeed/PitotDualBaroAirspeed.h` · **Наследует:** `AirspeedSensor` · **Статус:** проверена в замкнутой симуляции с шумом, не облётана

Самодельная трубка Пито на **двух абсолютных барометрах**: `total` —
BMP581 внутри трубки (полное давление), `stat` — основной барометр фюзеляжа
(статика). Руководство по сборке — [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md#трубка-пито-своими-руками).

| Метод | Описание |
|---|---|
| `PitotDualBaroAirspeed(BarometerSensor& total, BarometerSensor& stat)` | |
| `begin()` | `begin()` барометра трубки (статика уже запущена), начать обнуление |
| `update()` | новый отсчёт трубки → перепад − ноль, ФНЧ `PITOT_FILTER_TAU_S`; первые `PITOT_ZERO_SAMPLES` отсчётов — усреднение нуля; скорость `√(2·Δp/ρ)·PITOT_SCALE` |
| `isAvailable()` | оба барометра живы, ноль набран, нет неисправности, отсчёт свежее `PITOT_STALE_US` |
| `hasFault()` | перепад ниже −`PITOT_NEGATIVE_FAULT_PA` дольше `PITOT_NEGATIVE_FAULT_MS` (шланги, вода) |
| `getZeroOffset()`, `printStatus()` | диагностика |
| `static speedFrom(Δp, ρ)`, `static densityOf(p, T)` | формулы |

---

## namespace `SensorMounting`

**Файл:** `sensors/SensorMounting.h`

`inline void rotateToBody(uint16_t rotationCwDeg, float chipX, float chipY, float& bodyX, float& bodyY)` —
поворот осей чипа вокруг вертикали (чип вверх) в оси самолёта (X к носу,
Y влево). `rotationCwDeg` — куда смотрит ось X чипа, по часовой сверху:

| Значение | bodyX | bodyY |
|---|---|---|
| 0 (и любое неизвестное) | chipX | chipY |
| 90 | chipY | −chipX |
| 180 | −chipX | −chipY |
| 270 | −chipY | chipX |

Используется компасом (`MAG_ROTATION_CW_DEG`) и IMU без калибровки установки.

---

## `SensorSelection.h`

**Файл:** `sensors/SensorSelection.h` · **Вид:** препроцессорная конфигурация

Единственное место смены физического датчика: готовый набор одной строкой
(`SENSOR_KIT`) или каждый датчик отдельно. Любой выбор можно переопределить
флагом сборки (`-D SENSOR_KIT=SENSOR_KIT_LSM6DSV_PITOT`,
`-D SENSOR_IMU=SENSOR_IMU_ICM45686_SPI`).

| Набор `SENSOR_KIT` | IMU | Барометр | Компас | Воздушная скорость | GPS |
|---|---|---|---|---|---|
| `SENSOR_KIT_BENCH_GY521` (1, по умолчанию) | MPU6500 | BMP581 I2C (0x46/0x47) | QMC5883P | — | — |
| `SENSOR_KIT_LSM6DSV_PITOT` (2) | LSM6DSV | SPL06 (фюзеляж) | QMC6309 | BMP581 в трубке | M10 |
| `SENSOR_KIT_ICM45686_PITOT` (3) | ICM-45686 | SPL06 (фюзеляж) | QMC6309 | BMP581 в трубке | M10 |
| `SENSOR_KIT_CUSTOM` (0) | задать все пять макросов ниже | | | | |

| Макрос выбора | Варианты |
|---|---|
| `SENSOR_IMU` | `MPU6050` (1), `ICM42688` (2, SPI), `LSM6DSV` (3), `LSM6DSV_SPI` (4), `ICM45686` (5), `ICM45686_SPI` (6) |
| `SENSOR_BARO` | `BME280` (1), `BMP388` (2, SPI), `BMP388_I2C` (3), `SPL06` (4), `SPL06_SPI` (5), `BMP581` (6), `BMP581_SPI` (7) |
| `SENSOR_MAG` | `NONE` (0), `QMC5883P` (1), `QMC5883L` (2), `QMC6309` (3) |
| `SENSOR_AIRSPEED` | `NONE` (0), `PITOT_BMP581` (1) — BMP581 I2C 0x47 в трубке |
| `SENSOR_GPS` | `NONE` (0), `UBLOX_M10` (1) |

Все сочетания собираются на всех платах — `tools/build_matrix.sh`.

Результат — псевдонимы типов и фабрики устройств:

| Имя | MPU6050 / ICM42688 и т.д. |
|---|---|
| `SelectedImu`, `SELECTED_IMU_DEVICE(board)` | `MPU6050_Sensor` + `I2cRegisterDevice(i2c, 0x68)` / `ICM42688_Sensor` + `spiDevice(spi, PIN_SPI_CS_IMU)` |
| `SelectedBaro`, `SELECTED_BARO_DEVICE(board)` | `BME280_Sensor` + I2C 0x76 / `BMP388_Sensor` + SPI (CS `PIN_SPI_CS_BARO`) / `BMP388_Sensor` + I2C 0x76 |
| `SelectedMag`, `SELECTED_MAG_DEVICE(board)` | `QMC5883P_Sensor` + I2C 0x2C / `QMC5883L_Sensor` + I2C 0x0D / `QMC6309_Sensor` + I2C 0x7C (для NONE не определены) |
| новые IMU | `LSM6DSV_Sensor` + I2C 0x6A (запасной 0x6B) или SPI; `ICM45686_Sensor` + I2C 0x68 (0x69) или SPI |
| новые барометры | `SPL06_Sensor` + I2C 0x76 (0x77) или SPI; `BMP581_Sensor` + I2C 0x46 (0x47) или SPI |
| `SelectedPitotBaro`, `SELECTED_PITOT_DEVICE(board)` | `BMP581_Sensor` + I2C 0x47 (для NONE не определены) |
| `SelectedGps` | `UbloxM10_Gps` (для NONE не определён) |

`main.cpp` оборачивает создание компаса, GPS и трубки в `#if SENSOR_* != SENSOR_*_NONE`.

---

## `ImuOrientation`

**Файл:** `sensors/imu/ImuOrientation.h` · **Зависит от:** `SensorMounting`, `Preferences` (NVS)

Матрица поворота `R` из осей чипа в оси самолёта: `body = R · chip`; строки
`R` — оси самолёта в осях чипа.

| Метод | Описание |
|---|---|
| `ImuOrientation()` | Единичная (эквивалент `fromYawSteps(0)`) |
| `static ImuOrientation fromYawSteps(uint16_t cwDeg)` | Поворот вокруг вертикали шагами 90°, плата чипом вверх |
| `static const char* fromPoses(level[3], noseUp[3], rightWingDown[3], ImuOrientation& out)` | Калибровка по трём позам (показания акселерометра «верх» в осях чипа). `nullptr` — успех, иначе причина отказа |
| `void apply(const float chip[3], float body[3]) const` | Применить поворот |
| `float tiltFromLevelDeg(const float chipUp[3]) const` | Угол между измеренным «верхом» и осью Z самолёта (180°, если вектор нулевой) |
| `bool load(const char* ns)` | Загрузить из NVS; отклоняет не-ортонормированную или левую тройку |
| `void save(const char* ns) const` | Сохранить в NVS |
| `void describe(Print&) const` | «нос = +Y чипа, верх = +Z чипа» (с углом, если ось не совпадает с осью чипа ±14°) |

Алгоритм `fromPoses`: Z = норм(level); X₁ = часть noseUp ⟂ Z; Y = часть
rightWingDown ⟂ Z, X₂ = Y × Z; X = норм(X₁ + X₂), Y = Z × X. Отказы:

| Условие | Сообщение |
|---|---|
| нулевой вектор | «нет показаний акселерометра» |
| наклон шага 2 или 3 вне 20..80° | «в шаге N нужен наклон 30-60°» |
| cos(X₁, X₂) < −0.5 | «шаги 2 и 3 противоречат друг другу…» (нос опустили или не то крыло) |
| cos(X₁, X₂) < 0.9 (≈25°) | «…наклоняли не те оси…» |

---

## `AttitudeEstimator`

**Файл:** `sensors/imu/AttitudeEstimator.h`

Комплементарный фильтр крена/тангажа и интеграл рысканья, не зависит от чипа.

| Метод | Описание |
|---|---|
| `void reset()` | Следующий `update()` начнёт сразу с угла по акселерометру |
| `void setYaw(float deg)` | Задать курс (приводится к ±180) |
| `void update(ax, ay, az, rollRate, pitchRate, yawRate, nowUs)` | Ускорение в g в осях самолёта, скорости °/с |
| `getRoll()`, `getPitch()`, `getYaw()` | ° |

`roll_acc = atan2(ay, az)`, `pitch_acc = atan2(ax, √(ay² + az²))`;
`angle = 0.98·(angle + rate·dt) + 0.02·angle_acc` (τ ≈ 0.1 с при 2 мс).
Первый вызов после `reset()` — сразу углы акселерометра; `dt ≤ 0` или > 0.1 с —
шаг пропускается (пауза, зависание шины).

---

## `ImuSensorBase`

**Файл:** `sensors/imu/ImuSensorBase.h` · **Наследует:** `ImuSensor` · **Вид:** абстрактный

`RawImuSample` — один отсчёт в единицах АЦП в осях чипа:
`accelX/Y/Z`, `gyroX/Y/Z`, `temperature` (`int16_t`).

Конвейер `update()` → `process()`:

```
сырые отсчёты − смещения (АЦП) → масштаб (g, °/с) → ImuOrientation (оси самолёта)
→ авиационные знаки (gyroY, gyroZ со сменой знака) → AttitudeEstimator
```

| Метод | Описание |
|---|---|
| `bool isAvailable() const` | `begin()` успешен и < 50 ошибок чтения подряд |
| `void update()` | Одно чтение; ошибка — данные не меняются, счётчики растут |
| `void calibrate()` | 200 отсчётов × 10 мс: смещение гироскопа, шум, «верх»; без калибровки установки — горизонт = положение сейчас. Затем предполётная проверка |
| `void calibrateOrientation()` | Три позы (`capturePose`: неподвижно ~1 с, поза отличается от прошлых ≥ 20°, таймаут 30 с), `ImuOrientation::fromPoses`, сохранение в NVS. Снимает проблемы установки предполётной проверки |
| `const char* getPreflightProblem() const` | Текст проблемы или `nullptr` |
| `void setYaw(float)`, `getSensorType()`, `printStatus()` | |

Защищённое API для драйверов:

| Метод | Описание |
|---|---|
| `ImuSensorBase(const char* name, const char* nvsNamespace)` | Своё пространство NVS у каждого драйвера |
| `virtual bool readSample(RawImuSample&) = 0` | Один отсчёт; `false` — чип не ответил |
| `virtual float accelLsbPerG() const = 0`, `gyroLsbPerDps() const = 0` | Масштабы |
| `virtual float temperatureC(int16_t raw) const = 0` | Формула температуры |
| `void setAvailable(bool)` | Результат `begin()`; при `true` загружает установку из NVS или `Config::IMU_ROTATION_CW_DEG` |
| `void setName(const char*)` | Уточнить имя после опознания |

Предполётная проверка (`runPreflightCheck`), по порядку:

| Проблема | Условие |
|---|---|
| `NotResponding` | меньше половины отсчётов калибровки прочитано |
| `Moved` | шум гироскопа > 0.5 °/с |
| `NotOneG` | \|a\| отличается от 1g больше чем на 0.2g |
| `MountingMismatch` | (установка откалибрована) «верх» дальше 45° от сохранённого |
| `NotChipUp` | (не откалибрована) плата не лежит чипом вверх (`z < 0.5g`) |

---

## `MPU6050_Sensor`

**Файл:** `sensors/imu/MPU6050_Sensor.h` · **Наследует:** `ImuSensorBase` · **NVS:** `imu_mpu6050` · **Статус:** на стенде (MPU6500)

MPU6050 / MPU6500 / MPU9250 / MPU9255 и клоны (платы GY-521), I2C 0x68/0x69 или
SPI. Чип определяется по `WHO_AM_I` (0x68 — MPU6050, иначе семейство 6500).

- `begin()`: WHO_AM_I (нет ответа → недоступен), имя по ID, сброс, выход из
  sleep (PLL), ±2000 °/с, ±16 g, DLPF ~41 Гц, 1 кГц; у 6500 — отдельный ФНЧ
  акселерометра `ACCEL_CONFIG2`.
- `readSample()`: 14 байт с `0x3B`, big-endian: accel XYZ, temp, gyro XYZ.
- Масштабы: 2048 LSB/g, 16.4 LSB/(°/с).
- Температура: MPU6050 `raw/340 + 36.53`, MPU6500 `raw/333.87 + 21`.

---

## `ICM42688_Sensor`

**Файл:** `sensors/imu/ICM42688_Sensor.h` · **Наследует:** `ImuSensorBase` · **NVS:** `imu_icm42688` · **Статус:** не проверен на железе

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 МГц, без
  фиктивного байта.
- `begin()`: банк 0, программный сброс, `WHO_AM_I == 0x47`, Low Noise,
  ±2000 °/с / ±16 g, 1 кГц, UI-фильтр 50 Гц.
- `readSample()`: 14 байт с `0x1D`, big-endian: temp, accel XYZ, gyro XYZ.
- Температура: `raw/132.48 + 25`.

---

## `LSM6DSV_Sensor`

**Файл:** `sensors/imu/LSM6DSV_Sensor.h` · **Наследует:** `ImuSensorBase` · **NVS:** `imu_lsm6dsv` · **Статус:** не проверен на железе

LSM6DSV / LSM6DSV16X / LSM6DSV32X (ST). Регистры сверены с ST `lsm6dsv-pid` и ArduPilot.

- `begin()`: `WHO_AM_I` (0x0F) = 0x70; `SW_RESET` (CTRL3 бит 0) и ожидание;
  32X — по биту варианта в CTRL8 (у него свой код ±16 g); BDU + автоинкремент,
  ±2000 °/с с LPF1, ±16 g с LPF2, 960 Гц high-performance.
- `readSample()`: 14 байт с 0x20, little-endian: temp, gyro XYZ, accel XYZ.
- Масштабы: 1000/0.488 LSB/g, 1000/70 LSB/(°/с); температура `raw/256 + 25`.
- `static spiDevice(bus, cs)` — SPI режим 0, без фиктивного байта.

## `ICM45686_Sensor`

**Файл:** `sensors/imu/ICM45686_Sensor.h` · **Наследует:** `ImuSensorBase` · **NVS:** `imu_icm45686` · **Статус:** не проверен на железе

ICM-45686 (TDK). Регистры сверены с драйвером TDK, Zephyr и ArduPilot.

- `begin()`: сброс `REG_MISC2` (0x7F), `WHO_AM_I` (0x72) = 0xE9; ±2000 °/с и ±16 g
  на 1.6 кГц (`GYRO/ACCEL_CONFIG0` = 0x15), Low Noise (`PWR_MGMT0` = 0x0F); ФНЧ
  ODR/32 — чтение-модификация-запись **косвенных** регистров IPREG
  (0xA4AC, 0xA583) через окно 0x7C..0x7E; 45 мс на запуск гироскопа.
- `readSample()`: 14 байт с 0x00, little-endian: accel XYZ, gyro XYZ, temp.
- Масштабы: 2048 LSB/g, 16.4 LSB/(°/с); температура `raw/132.48 + 25`.

---

## `BarometerBase`

**Файл:** `sensors/baro/BarometerBase.h` · **Наследует:** `BarometerSensor` · **Вид:** абстрактный

| Метод | Описание |
|---|---|
| `bool isAvailable() const` | `begin()` успешен и < 100 ошибок подряд |
| `void update()` | Не чаще `pollPeriodUs`: `isNewSampleReady()` → `readSample()` → высота и вертикальная скорость |
| `void calibrateAltitude()` | 20 отсчётов × 50 мс: средняя абсолютная высота = база; сброс высоты и скорости |
| `void setSeaLevelPressure(Па)` | P₀ (по умолчанию 101325) |
| `getBarometerData()`, `getSensorType()`, `printStatus()` | |

Защищённое API: конструктор `(name, pollPeriodUs)`,
`virtual bool isNewSampleReady(bool& ready) = 0`,
`virtual bool readSample(float& pressurePa, float& temperatureC) = 0`,
`setAvailable(bool)`.

Формулы: `h = 44330 · (1 − (P/P₀)^0.1903) − база`; вертикальная скорость —
производная высоты по **реальным новым** отсчётам через ФНЧ τ = 0.5 с
(`dt` вне `(0, 0.5 с)` — скорость не обновляется). Чтение только новых отсчётов
исключает «ступенчатый» шум (0 м/с вперемешку со скачками Δh/2 мс).

---

## `BMP388_Sensor`

**Файл:** `sensors/baro/BMP388_Sensor.h` · **Наследует:** `BarometerBase` · **Статус:** на стенде (I2C)

- `static SpiRegisterDevice spiDevice(ISpiBus&, uint8_t cs)` — SPI 8 МГц,
  **1 фиктивный байт** (даташит §5.3.2).
- `begin()`: chip ID `0x50`, программный сброс, 21 байт NVM-коэффициентов с
  `0x31` (масштабы §9.1), OSR ×8/×1, ODR 50 Гц, IIR 3, normal mode.
- `isNewSampleReady()`: флаг `drdy_press` (бит 0x20) регистра STATUS; опрос
  каждые 5 мс.
- `readSample()`: 6 байт с `0x04`; компенсация Bosch §9.3 (double):
  температура первой (`tLin`), затем давление.

---

## `BME280_Sensor`

**Файл:** `sensors/baro/BME280_Sensor.h` · **Наследует:** `BarometerBase` · **Статус:** не проверен на железе

BME280 (ID 0x60) и BMP280 (ID 0x58), I2C 0x76/0x77 или SPI без фиктивного
байта. Влажность не читается.

- `begin()`: chip ID, сброс, 24 байта калибровки с `0x88`, `CTRL_HUM` (только
  BME280, пишется **до** `CTRL_MEAS`), `CONFIG` = IIR 4 + 0.5 мс, `CTRL_MEAS`
  = T×2, P×8, normal.
- Флага готовности нет — `ready = true`, опрос раз в 25 мс.
- Компенсация — формулы Bosch §8.1 (double); защита от деления на ноль.

---

## `SPL06_Sensor`

**Файл:** `sensors/baro/SPL06_Sensor.h` · **Наследует:** `BarometerBase` · **Статус:** не проверен на железе

SPL06-001 (Goertek). Формулы — датащит §4.9.

- `begin()`: `ID` (0x0D) = 0x10 (0x11 — SPA06, другой набор коэффициентов —
  отвергается); сброс, ожидание `COEF_RDY | SENSOR_RDY`; 18 байт коэффициентов
  (знаковые 12/20/16-битные поля); источник температуры — по `COEF_SRCE`;
  давление 16× (32 Гц), температура 1×, непрерывный режим.
- `isNewSampleReady()` — бит `PRS_RDY`; `readSample()` — 24-битные
  big-endian отсчёты, `kP = 253952`, `kT = 524288`.

## `BMP581_Sensor`

**Файл:** `sensors/baro/BMP581_Sensor.h` · **Наследует:** `BarometerBase` · **Статус:** основной барометр набора по умолчанию, не проверен на железе

BMP581 (Bosch). Последовательность — официальный BMP5_SensorAPI.

- `begin()`: фиктивное чтение (для SPI), `CHIP_ID` (0x01) = 0x50/0x51;
  программный сброс, `INT_STATUS` POR и `STATUS` NVM готова без ошибок;
  standby → OSR (давление 16×, температура 2×), IIR, DRDY; проверка
  выполнимости ODR (`OSR_EFF`); непрерывный режим.
- Отсчёт — по DRDY или раз в 40 мс, если флаг потерян; температура
  `int24/65536`, давление `uint24/64`.
- Два экземпляра в сборке с трубкой: основной барометр и `PITOT-BMP581`.

---

## `MagnetometerBase`

**Файл:** `sensors/mag/MagnetometerBase.h` · **Наследует:** `MagnetometerSensor` · **Вид:** абстрактный

| Метод | Описание |
|---|---|
| `bool isAvailable() const` | `begin()` успешен и < 25 ошибок подряд |
| `void update()` | 50 Гц: `readRaw()` → вычесть смещения → масштаб → поворот `MAG_ROTATION_CW_DEG` → курс `atan2(Y, X)` в 0..360 |
| `void calibrate()` | 15 с вращения: смещение = (min + max)/2 по осям (hard-iron), сохранение в NVS. Ни одного удачного чтения — калибровка отклоняется, прежняя в NVS не трогается |
| `getMagData()`, `getSensorType()`, `printStatus()` | |

Защищённое API: конструктор `(name, nvsNamespace)`,
`virtual bool readRaw(int16_t raw[3]) = 0`, `virtual float lsbPerMicroTesla() const = 0`,
`setAvailable(bool)` (при `true` загружает калибровку из NVS).

Курс без компенсации наклона: верен, пока самолёт почти в горизонте. Нос на
север → поле вдоль +X → 0°; нос на восток → 90°.

---

## `QMC5883P_Sensor`

**Файл:** `sensors/mag/QMC5883P_Sensor.h` · **Наследует:** `MagnetometerBase` · **NVS:** `qmc5883p` · **Статус:** на стенде

`DEFAULT_ADDRESS = 0x2C`. `begin()`: chip ID `0x80` (регистр 0x00), программный
сброс, знаки осей `0x29 = 0x06`, `CONTROL2 = 0x08` (SET/RESET, ±8 Гс),
`CONTROL1 = 0xCD` (normal, 200 Гц, OSR 8/8). Данные — 6 байт с `0x01`,
little-endian. 37.5 LSB/мкТл.

---

## `QMC5883L_Sensor`

**Файл:** `sensors/mag/QMC5883L_Sensor.h` · **Наследует:** `MagnetometerBase` · **NVS:** `qmc5883l` · **Статус:** не проверен на железе

`DEFAULT_ADDRESS = 0x0D`. `begin()`: `probe()` (у чипа нет надёжного ID),
`SET/RESET = 0x01`, `CONTROL1 = 0x1D` (continuous, 200 Гц, ±8 Гс, OSR 512).
Данные — 6 байт с `0x00`, little-endian. 30 LSB/мкТл. Регистры **не совместимы**
с QMC5883P.

---

## `QMC6309_Sensor`

**Файл:** `sensors/mag/QMC6309_Sensor.h` · **Наследует:** `MagnetometerBase` · **NVS:** `qmc6309` · **Статус:** не проверен на железе

QMC6309 (QST), I2C 0x7C — адрес за пределами обычного диапазона 0x08..0x77
(опрос шин консоли `b` проходит до 0x7F).

- `begin()`: `CHIP_ID` (0x00) = 0x90; сброс (CTRL2 0x80 → 0x00), ожидание
  `NVM_RDY | NVM_LOAD_DONE`; ±8 Гс, 200 Гц, set/reset; LPF 16, OSR 8, normal.
- `readRaw()`: X/Y/Z little-endian с 0x01; 40.96 LSB/мкТл.

---

## `UbloxM10_Gps`

**Файл:** `sensors/gps/UbloxM10_Gps.h` · **Наследует:** `GpsSensor` · **Зависит от:** `IUartPort`, `Config` · **Статус:** на стенде не подключён

u-blox M10 по UART, протокол UBX. Разбирается только **NAV-PVT** (класс 0x01,
id 0x07, 92 байта).

| Метод | Описание |
|---|---|
| `explicit UbloxM10_Gps(IUartPort&)` | |
| `bool begin()` | 9600 бод → CFG-VALSET `UART1_BAUDRATE = 115200` → 115200 бод → CFG-VALSET: 10 Гц, NAV-PVT на UART1, UBX вкл, NMEA выкл. Без TX-пина (`PIN_GPS_TX < 0`) — только слушает на 9600. Всегда `true` (без ACK) |
| `bool isAvailable() const` | Был валидный NAV-PVT и последний не старше `GPS_TIMEOUT_US` |
| `void update()` | Скормить парсеру всё из UART |
| `const GpsData& getGpsData() const`, `bool hasFix() const` | `hasFix` = доступен и `fixType ≥ 2` |
| `getSensorType()`, `printStatus()` | `printStatus()` показывает `available` по `isAvailable()` (с учётом таймаута) |

Парсер — побайтовый автомат `SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 →
PAYLOAD → CK_A → CK_B`, контрольная сумма Флетчера-8 по class+id+len+payload.
Длина > 512 — сбой синхронизации, поиск заново. Разбор полей NAV-PVT: `fixType`
(20), `numSV` (23), `lon`/`lat` (24/28, ×1e−7), `hMSL` (36, мм), `hAcc`/`vAcc`
(40/44, мм), `gSpeed` (60, мм/с), `headMot` (64, ×1e−5 °, приводится к 0..360).

Вложенный `ValsetBuilder` — payload UBX-CFG-VALSET (version 0, layer RAM, пары
ключ U4 LE — значение 1/2/4 байта LE, буфер 64 байта).
