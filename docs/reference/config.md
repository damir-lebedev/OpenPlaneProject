# CONFIG — `Config` и `Channels`

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
| `PIN_SPI_CS_ICM42688` | `uint8_t` | CS IMU по SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BMP388` | `uint8_t` | CS барометра по SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | UART GPS; TX `-1` — только приём | 15 / 16 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | Номер аппаратного UART для GPS | 2 | 0 | 2 |
| `PIN_AUX1..3`, `PIN_BUZZER`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_RX/TX` | `int8_t` | **Только S3:** резерв под плату полётника ([FC_BOARD.md](../FC_BOARD.md)), прошивкой пока не используются | 41, 42, 47, 38, 3, 10, 39/40 | — | — |

SPI-шина датчиков называется `PIN_SENSOR_SPI_*`, а не `PIN_SPI_*`: в ядре
STM32duino (и других ядрах Arduino) `PIN_SPI_SCK/MISO/MOSI` — макросы варианта,
они подменили бы константы `Config`.

<a id="stm32h743"></a>

#### STM32H743VIT6 — заготовка (`BOARD_STM32H743`)

Платы пока нет: распиновка **не проверена на железе**. Пины выбраны из свободных
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
| `PIN_SPI_CS_ICM42688` / `PIN_SPI_CS_BMP388` | PB12 / PD10 | GPIO |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD9 / PD8 | USART3 |
| `PIN_AUX1` / `PIN_AUX2` | PD15 / PE9 | TIM4_CH4 / TIM1_CH1 — резерв |
| `PIN_BUZZER`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC` | PE15, PC0, PC1 | GPIO, ADC1_INP10, ADC1_INP11 — резерв |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PD0 / PD1 | UART4 (эти же пины — FDCAN1) — резерв |

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

### Цикл, Wi-Fi, отладка

| Константа | Значение | Смысл |
|---|---|---|
| `LOOP_PERIOD_MS` | 2 | Период полётного цикла (500 Гц); также номинальный `dt` для `PidController` |
| `WIFI_AP_SSID` / `WIFI_AP_PASSWORD` | `"OpenPlane-Debug"` / `"12345678"` | Точка доступа дашборда (пароль слабый — инструмент стенда) |
| `WEB_SERVER_PORT` | 80 | Порт HTTP |
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
| `ARM` | 4 | CH5 | SwA | Тумблер ARM |
| `FLAPS` | 5 | CH6 | SwB | Закрылки |
| `AUX_2` | 6 | CH7 | SwC (3 поз.) | Режим автопилота |
| `AUX_3` | 7 | CH8 | SwD | Свободен |
| `AUX_4` | 8 | CH9 | VrA | Свободен |
| `AUX_5` | 9 | CH10 | VrB | Свободен |
