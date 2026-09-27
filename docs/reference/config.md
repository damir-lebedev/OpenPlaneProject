# CONFIG — `Config`, `Channels`, `Controls`

[← Справочник](README.md)

Слой конфигурации — только `constexpr`-константы, без кода. Логика классов не
должна содержать «магических» пинов, таймаутов и порогов: всё, что может
понадобиться поменять под конкретный самолёт или плату, живёт здесь.

---

## namespace `Config`

**Файл:** `include/config/Config.h` · **Зависит от:** `<stdint.h>` ·
**Используется:** почти всеми слоями.

### Пины (зависят от платы)

Блок пинов выбирается макросом, который задаёт `[env:*]` в `platformio.ini`
(`-D BOARD_ESP32_S3` / `BOARD_ESP32_C3` / `BOARD_ESP32_CLASSIC` /
`BOARD_STM32H743`). Без макроса — `#error`. Блок STM32 описан
[ниже](#stm32h743).

| Константа | Тип | Назначение | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Элероны | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Руль высоты | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Регулятор мотора | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Руль направления + колесо; `-1` — выход отключён | 18 | −1 | 25 |
| `PIN_IBUS` | `uint8_t` | RX приёмника iBUS (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | Шина датчиков (`Wire`) | 8 / 9 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | Шина OLED (`Wire1`); `-1` — нет | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | Общая SPI-шина | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | CS IMU по SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | CS барометра по SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | UART GPS; TX `-1` — только приём | 15 / 16 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | Номер аппаратного UART для GPS | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | серво-выходы: сброс груза, камера; `-1` — нет | 41, 42 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | пищалка через транзистор; `-1` — нет | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_RX/TX` | `int8_t` | **Только S3:** резерв под плату полётника ([FC_BOARD.md](../FC_BOARD.md)) | 47, 3, 10, 39/40 | — | — |

SPI-шина датчиков называется `PIN_SENSOR_SPI_*`, а не `PIN_SPI_*`: в ядре
STM32duino (и других ядрах Arduino) `PIN_SPI_SCK/MISO/MOSI` — макросы варианта,
они подменили бы константы `Config`.

<a id="stm32h743"></a>

#### STM32H743VIT6 (`BOARD_STM32H743`)

Платы пока нет: распиновка **не проверена на железе** (прошивка гоняется на ПК, env `native-stm32`). Пины выбраны из свободных
на WeAct MiniSTM32H743VITx (плата PlatformIO env `stm32h743`) и сверены с
таблицами `PeripheralPins` варианта STM32duino. Значения — макросы варианта
(`PA0`…), поэтому в начале `Config.h` под `#if defined(BOARD_STM32H743)`
подключается `<Arduino.h>`. Тип всех пинов — `int16_t` (у аналоговых пинов
номер `0xC0 + N`). Номеров UART нет — периферию ядро выбирает по пинам.

| Константа | Пин | Периферия |
|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_ESC` | PA0 / PA1 / PA2 / PA3 | TIM2_CH1..CH4 |
| `PIN_RUDDER` | PD14 | TIM4_CH3 |
| `PIN_IBUS` / `PIN_IBUS_TX` | PE7 / PE8 | UART7 (TX — резерв под iBUS-SENS) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 — датчики |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | PB9 / PB8 | I2C1 — экран (на WeAct — разъём камеры) |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | PB13 / PB14 / PB15 | SPI2 |
| `PIN_SPI_CS_IMU` / `PIN_SPI_CS_BARO` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — груз / камера |
| `PIN_BUZZER` | PE15 | GPIO — пищалка |
| `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PC0, PC1 | ADC1_INP10, ADC1_INP11 — резерв |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 — радиомодем MAVLink (эти же пины — FDCAN1) |

Консоль `Serial` — LPUART1 (PA9 TX / PA10 RX), дефолт варианта.

### iBUS и потеря связи

| Константа | Значение | Смысл |
|---|---|---|
| `IBUS_CHANNELS` | 10 | Сколько каналов из кадра используется |
| `IBUS_FRAME_LENGTH` | 32 | Длина кадра, байт |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | Заголовок кадра |
| `IBUS_BAUDRATE` | 115200 | Скорость UART |
| `RX_TIMEOUT_US` | 500 000 | Нет корректного кадра дольше — связь потеряна |
| `RX_FAILSAFE_THROTTLE_US` | 950 | Газ ниже — приёмник сообщает failsafe пульта |

### GPS

| Константа | Значение | Смысл |
|---|---|---|
| `GPS_TIMEOUT_US` | 2 000 000 | NAV-PVT старше — `UbloxM10_Gps::isAvailable() == false` |

### Диапазон PWM и ходы рулей

| Константа | Значение | Смысл |
|---|---|---|
| `PWM_MIN` / `PWM_CENTER` / `PWM_MAX` | 1000 / 1500 / 2000 | Стандартный импульс RC, мкс |
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 | Отклонение от центра при полном ходе стика, мкс |

### Закрылки (флапероны)

| Константа | Значение | Смысл |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 выше — закрылки выпущены (не 1500: до первого кадра каналы = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Отклонение каждого элерона вниз, мкс (~20° качалки MG90S) |
| `FLAPS_TRANSITION_MS` | 1000 | Время полного выпуска/уборки |

### Направление серво

`AILERON_LEFT_REVERSED`, `AILERON_RIGHT_REVERSED`, `ELEVATOR_REVERSED` (`true`),
`RUDDER_REVERSED` — единственное место, где задаётся реверс. `ControlMixer`
считает в физических знаках и меняет знак только здесь, поэтому стики и
автопилот не могут разойтись. Реверс на пульте делать **нельзя**.

### Установка датчиков

| Константа | Значение | Смысл |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | Поворот осей чипа IMU вокруг вертикали (0/90/180/270), куда смотрит ось X чипа. Используется, только пока нет калибровки установки `o` в NVS |
| `MAG_ROTATION_CW_DEG` | 0 | То же для компаса (калибровки установки у компаса нет) |

### ARM

| Константа | Значение | Смысл |
|---|---|---|
| `ARM_SWITCH_ON_US` | 1750 | CH5 выше — тумблер ARM включён |
| `THROTTLE_LOW_US` | 1050 | Газ ниже — «газ внизу», армиться можно |

### Failsafe

| Константа | Значение | Смысл |
|---|---|---|
| `FAILSAFE_AILERON` / `ELEVATOR` / `RUDDER` | 1500 | Нейтраль рулей |
| `FAILSAFE_THROTTLE` | 1000 | Мотор выключен |
| `FAILSAFE_GLIDE_ROLL_DEG` | 0.0 | Крен планирования при потере связи в воздухе |
| `FAILSAFE_GLIDE_PITCH_DEG` | −3.0 | Тангаж планирования (чуть ниже горизонта) |
| `FAILSAFE_RTH` | `true` | При GPS и доме потеря связи в воздухе — домой с мотором, а не планирование |

### Тумблеры, трубка Пито, автопилот

Числа всех режимов и функций — в `Config.h` рядом с подробными комментариями;
что они значат для пилота — [AUTOPILOT_GUIDE.md](../AUTOPILOT_GUIDE.md).

| Группа | Константы |
|---|---|
| Тумблеры | `SWITCH_ON_US` = 1750 (канал выше — тумблер включён; не 1500, чтобы до первого кадра ничего не включилось) |
| Трубка Пито | `PITOT_ZERO_SAMPLES` 50, `PITOT_FILTER_TAU_S` 0.1, `PITOT_SCALE` 1.0, `PITOT_NEGATIVE_FAULT_PA/MS` 30/2000, `PITOT_STALE_US` 200 000 |
| Стабилизация | `MAX_BANK_DEG` 45 (крутилка 15…60), `STAB_MAX_PITCH_DEG` 25, `STAB_INTEGRATOR_ZONE_DEG` 10, `STAB_GAIN_MIN/MAX` 0.25/2 |
| ACRO | `ACRO_MAX_RATE_DPS` 180, `ACRO_RATE_GAIN_US_PER_DPS` 1.5 |
| Навигация | `NAV_COURSE_GAIN`, `NAV_BANK_LIMIT_DEG` 40, `NAV_GPS_COURSE_MIN_SPEED_MS` 3, `NAV_ASSUMED_SPEED_MS` 15 |
| Высота | `NAV_ALT_GAIN`, `NAV_MAX_CLIMB/SINK_MS` 3/3, `NAV_CLIMB_KP/KI_DEG`, `NAV_MAX_CLIMB/DIVE_PITCH_DEG` 15/−12 |
| Газ и скорость | `CRUISE_THROTTLE_PCT` 55 (30…85), `CRUISE_AIRSPEED_MS` 14 (10…22), `AIRSPEED_THROTTLE_KP/KI`, `THROTTLE_PER_CLIMB_PCT`, `AUTO_THROTTLE_MIN/MAX_PCT` |
| Сваливание | `STALL_SPEED_MS` 8, `STALL_MARGIN_MS` 2, `STALL_BANK_LIMIT_DEG` 20 |
| Круги и дом | `LOITER_RADIUS_M` 50 (25…150), `LOITER_CONVERGENCE`, `RTH_ALTITUDE_M` 40, `HOME_MIN_SATELLITES` 6, `HOME_MAX_HACC_M` 5 |
| Геозабор | `GEOFENCE_ALWAYS_ON` false, `FENCE_RADIUS_M` 500, `FENCE_ALTITUDE_M` 120 |
| Запуск с руки | `LAUNCH_ACCEL_G` 1.5, `LAUNCH_ACCEL_TIME_MS` 40, `LAUNCH_MOTOR_DELAY_MS` 300, `LAUNCH_THROTTLE_PCT`, `LAUNCH_CLIMB_PITCH_DEG` 15, `LAUNCH_CLIMB_MS` 6000, `LAUNCH_ALTITUDE_M` 30 |
| Посадка | `LAND_GLIDE_PITCH_DEG` −4, `LAND_FLARE_ALTITUDE_M` 3, `LAND_FLARE_PITCH_DEG` 4 |
| Парение | `SOAR_*`: планирование −3°, термик > 0.5 м/с за 1.5 с, круг 25°, выход < −0.2 м/с за 8 с, мотор ниже 30 м до 100 м, домой дальше 400 м |
| RESCUE | `RESCUE_PITCH_DEG` 8, `RESCUE_THROTTLE_PCT` 70 |
| Автотриммер | `AUTOTRIM_RATE` 0.2, `AUTOTRIM_MAX_US` 120, `AUTOTRIM_MAX_ROLL_DEG` 15, `AUTOTRIM_MAX_RATE_DPS` 30, запись на земле: `AUTOTRIM_SAVE_MAX_ALT_M` 3, `_CLIMB_MS` 0.5, `_SPEED_MS` 3 |
| Координация | `TURN_COORD_RUDDER_MIX` 0.3, `TURN_COORD_PITCH_US` 150, `TURN_COORD_IN_NAV_MODES` true |
| Функции | `AIRBRAKE_US` 250, `PAYLOAD_CLOSED/OPEN_US` 1000/2000, `CAMERA_TILT_MIN/MAX_DEG` −90/30, `CAMERA_US_PER_DEG`, `RATES_MIN/MAX_PCT` 30/100, `LOST_MODEL_BEEP_DELAY_MS` 10 000 |

### Цикл, Wi-Fi, отладка

| Константа | Значение | Смысл |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | Период полётного цикла (500 Гц); также номинальный `dt` для `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | Точка доступа дашборда (пароль слабый — инструмент стенда) |
| `WEB_SERVER_PORT` | 80 | Порт HTTP |
| `TELEM_BAUDRATE` | 57600 | Скорость радиомодема MAVLink (дефолт SiK) |
| `MAVLINK_SYSTEM_ID` / `MAVLINK_COMPONENT_ID` | 1 / 1 | Адрес борта в MAVLink |
| `DEBUG_INTERVAL_MS` | 100 | Как часто `DebugLogger` проверяет каналы лога |
| `DEBUG_CHANGE_DEADBAND_US` | 3 | Допуск на дребезг RC/PWM в режиме «при изменении» |

---

## namespace `Channels`

**Файл:** `include/config/Channels.h` · **Зависит от:** `<stdint.h>`

Единственное место, где физический номер канала связывается с назначением.
Значения — **индексы** (0-based) в `RcChannelState`.

| Константа | Индекс | Канал | Орган FS-i6 | Назначение |
|---|---|---|---|---|
| `AILERON` | 0 | CH1 | правый стик ←→ | Крен |
| `ELEVATOR` | 1 | CH2 | правый стик ↑↓ | Тангаж (2000 = от себя = нос вниз) |
| `THROTTLE` | 2 | CH3 | левый стик ↑↓ | Газ |
| `RUDDER` | 3 | CH4 | левый стик ←→ | Руль направления + колесо |
| `ARM` | 4 | CH5 | SwA | Тумблер ARM (переназначить нельзя) |
| `SWB` | 5 | CH6 | SwB | по таблице `Controls.h` (по умолчанию — закрылки) |
| `SWC` | 6 | CH7 | SwC (3 поз.) | по умолчанию — режим MANUAL / STABILIZE / AUTO_TAKEOFF |
| `SWD` | 7 | CH8 | SwD | по умолчанию — RTH |
| `VRA` | 8 | CH9 | VrA | по умолчанию — `STAB_GAIN` |
| `VRB` | 9 | CH10 | VrB | по умолчанию — `CRUISE_SPEED` |
| `COUNT` | 10 | | | число каналов |

---

## namespace `Controls`

**Файл:** `include/config/Controls.h` · **Зависит от:** `ControlBinding.h`, `Channels`

`constexpr Binding BINDINGS[]` — что делает каждый тумблер и крутилка,
**одна строка на канал** (`Bind::modes/mode/feature/knob`, см.
[autopilot.md](autopilot.md#binding-bind-bindingcheck)). Рядом — закомментированные
готовые идеи. Три `static_assert` ловят ошибки таблицы при сборке: стик или
ARM в таблице, канал вне диапазона, повтор канала, больше одного тумблера
выбора режима.
