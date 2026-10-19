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
`BOARD_STM32H743` / `BOARD_MATEK_H743`). Без макроса — `#error`. Блок STM32 описан
[ниже](#stm32h743vit6-board_stm32h743).

| Константа | Тип | Назначение | S3 | C3 | classic |
|---|---|---|---|---|---|
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` | `uint8_t` | Элероны | 4 / 5 | 5 / 4 | 13 / 14 |
| `PIN_ELEVATOR` | `uint8_t` | Руль высоты | 6 | 6 | 27 |
| `PIN_ESC` | `uint8_t` | Регулятор мотора | 7 | 7 | 26 |
| `PIN_RUDDER` | `int8_t` | Руль направления + колесо; `-1` — выход отключён | 18 | −1 | 25 |
| `PIN_RC_RX` | `uint8_t` | RX приёмника: iBUS, SBUS или CRSF (UART1) | 17 | 8 | 16 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | `uint8_t` | Шина датчиков (`Wire`) | 41 / 42 | 1 / 3 | 21 / 22 |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | `int8_t` | Шина OLED (`Wire1`); `-1` — нет | 1 / 2 | −1 | −1 |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI` | `uint8_t` | Общая SPI-шина | 12 / 13 / 11 | 0 / 10 / 20 | 18 / 19 / 23 |
| `PIN_SPI_CS_IMU` | `uint8_t` | CS IMU по SPI | 14 | 21 | 32 |
| `PIN_SPI_CS_BARO` | `uint8_t` | CS барометра по SPI | 21 | 2 | 5 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | `int8_t` | UART GPS; TX `-1` — только приём | 39 / 40 | 9 / −1 | 4 / 17 |
| `UART_NUM_GPS` | `uint8_t` | Номер аппаратного UART для GPS | 2 | 0 | 2 |
| `PIN_AUX1`, `PIN_AUX2` | `int8_t` | серво-выходы: сброс груза, закрылки; `-1` — нет | 15, 16 | −1, −1 | 33, 15 |
| `PIN_BUZZER` | `int8_t` | пищалка через транзистор; `-1` — нет | 38 | −1 | 2 |
| `PIN_AUX3`, `PIN_LIGHT`, `PIN_VBAT_ADC`, `PIN_CURRENT_ADC`, `PIN_TELEM_TX/RX` | `int8_t` | **Только S3:** резерв под плату полётника ([FC_BOARD.md](../FC_BOARD.md)) | 47, 21, 8, 3, 9/10 | — | — |

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
| `PIN_RC_RX` / `PIN_RC_TX` | PE7 / PE8 | UART7 (TX — резерв под телеметрию в пульт) |
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

#### Matek H743-WING V3 (`BOARD_MATEK_H743`)

Готовый полётник: env `matek-h743` задаёт и `BOARD_STM32H743` (весь код STM32 общий), и свой блок пинов — по описанию платы в ArduPilot (hwdef MatekH743), на железе не проверен. Пины распаянного на плате помечены `@fixed`.

| Константа | Пин | Площадка, периферия |
|---|---|---|
| `PIN_ESC` | PB0 | S1, TIM1_CH2N |
| `PIN_AILERON_LEFT` / `PIN_AILERON_RIGHT` / `PIN_ELEVATOR` / `PIN_RUDDER` | PA0 / PA1 / PA2 / PA3 | S3–S6, TIM2_CH1..CH4 |
| `PIN_AUX1` / `PIN_AUX2` | PD12 / PD13 | S7 / S8, TIM4_CH1 / CH2 |
| `PIN_RC_RX` / `PIN_RC_TX` | PC7 / PC6 | R6 / T6, USART6 |
| `PIN_GPS_RX` / `PIN_GPS_TX` | PD6 / PD5 | R2 / T2, USART2 |
| `PIN_TELEM_RX` / `PIN_TELEM_TX` | PE7 / PE8 | R7 / T7, UART7 |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | PB11 / PB10 | I2C2 на плате: барометр DPS310 |
| `PIN_I2C_EXT_SDA` / `PIN_I2C_EXT_SCL` | PB7 / PB6 | SDA / SCL, I2C1 — внешний разъём: компас, трубка Пито |
| `PIN_I2C2_SDA` / `PIN_I2C2_SCL` | −1 | шины экрана нет |
| `PIN_SENSOR_SPI_SCK` / `MISO` / `MOSI`, `PIN_SPI_CS_IMU` | PA5 / PA6 / PD7, PC15 | SPI1 на плате: ICM-42688-P |
| `PIN_BUZZER` | PA15 | BZ+ / BZ− |
| `PIN_VBAT_ADC` / `PIN_CURRENT_ADC` | PC0 / PC1 | VBAT (делитель ×11) / Curr (40 А на вольт) |

`PIN_I2C_EXT_SDA` / `PIN_I2C_EXT_SCL` есть и в общем блоке STM32: там они −1 — компас и трубка Пито на шине датчиков. Делители `BLACKBOX_VBAT_DIVIDER` / `BLACKBOX_CURRENT_DIVIDER` и `IMU_MOUNT_CALIBRATION_REQUIRED` тоже у каждой платы свои.

### Приёмник и потеря связи

| Константа | Значение | Смысл |
|---|---|---|
| `RX_PROTOCOL` | `IBUS` | Протокол приёмника: `IBUS` (FlySky), `SBUS` (FrSky, Futaba, RadioMaster…), `CRSF` (ExpressLRS, TBS Crossfire) |
| `RC_CHANNELS` | 10 | Сколько каналов из кадра используется |
| `IBUS_FRAME_LENGTH` | 32 | Длина кадра, байт |
| `IBUS_HEADER_0` / `IBUS_HEADER_1` | `0x20` / `0x40` | Заголовок кадра |
| `IBUS_BAUDRATE` | 115200 | Скорость UART |
| `SBUS_BAUDRATE` | 100000 | Скорость SBUS (8E2, сигнал инвертирован) |
| `CRSF_BAUDRATE` | 420000 | Скорость CRSF (как у ExpressLRS по умолчанию) |
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
| `AILERON_MAX_US`, `ELEVATOR_MAX_US`, `RUDDER_MAX_US` | 500 / 500 / 300 | Отклонение от центра при полном ходе стика, мкс. Руль направления — меньше: на той же серве колесо шасси |
| `THROTTLE_LIMIT_PCT` | 100 | Потолок газа на ESC, %, одинаково для стика и автопилота (`FlightController::capThrottle`). Для стендовых тестов на слабой сборке 3S1P ставили 50; тесты считают ожидаемый выход от этого значения |
| `THROTTLE_KNOB_MIN_PCT` | 10 | Крутилка `Knob::THROTTLE_LIMIT`: предел газа 10 … 55 … 100 % поверх `THROTTLE_LIMIT_PCT`, разрешённое растягивается на весь ход стика. Не привязана — предела нет |

### ESC и реверс мотора

| Константа | Значение | Смысл |
|---|---|---|
| `ESC_BIDIRECTIONAL` | `true` | Двунаправленный ESC (BLHeli 80A в режиме Bidirectional/3D): газ переводится в импульс вокруг нейтрали (`EscSignal`), «мотор стоит» — 1500, реверс доступен. `false` — обычный ESC: 1000 = стоп, реверса нет |
| `ESC_NEUTRAL_US` | 1500 | Нейтраль двунаправленного ESC: мотор стоит; выше — вперёд, ниже — назад |
| `REVERSE_THROTTLE_LIMIT_PCT` | 50 | Предел газа назад, % (и не больше `THROTTLE_LIMIT_PCT`): винт назад тянет плохо, а ток берёт полный |
| `REVERSE_SETTLE_MS` | 3000 | После смены направления ARM запрещён столько времени — мотор, снятый с ARM на газу, успевает остановиться |

### Закрылки (флапероны)

| Константа | Значение | Смысл |
|---|---|---|
| `FLAPS_SWITCH_ON_US` | 1750 | CH6 выше — закрылки выпущены (не 1500: до первого кадра каналы = 1500) |
| `FLAPS_DEPLOYED_US` | 220 | Отклонение каждого элерона вниз, мкс (~20° качалки MG90S) |
| `FLAPS_TRANSITION_MS` | 1000 | Время полного выпуска/уборки |

### Реверс

Всё в одном блоке `Config.h`, одна строка — одна вещь, остальное не трогается.

| Флаг | По умолчанию | Что разворачивает |
|---|---|---|
| `AILERON_LEFT_REVERSED` / `AILERON_RIGHT_REVERSED` | `false` / `true` | каждый элерон отдельно (серво в крыле зеркальные) |
| `ELEVATOR_REVERSED` | `true` | руль высоты |
| `RUDDER_REVERSED` | `false` | руль направления и рулевое колесо (одна серва) |
| `AUX1_REVERSED` | `false` | серво сброса груза: открыто ↔ закрыто |
| `AUX2_REVERSED` | `false` | серво камеры |
| `MOTOR_REVERSED` | `false` | направление вращения мотора — вместо перепайки двух проводов; только при `ESC_BIDIRECTIONAL` (иначе сборка не пройдёт) |
| `FLAPS_REVERSED` | `false` | закрылки и воздушный тормоз на элеронах; крен не трогается |
| `CHANNEL_REVERSED[10]` | все `false` | каналы пульта CH1…CH10, как Reverse в меню пульта: стик, тумблер или крутилка наоборот. CH3 (газ) — нельзя: потеря связи определяется по сырому газу < 950 мкс (`static_assert`) |

`ControlMixer` считает в физических знаках, а знак меняется только здесь,
поэтому стики и автопилот не могут разойтись. Руль крутится не туда и от
стика, и от автопилота — флаг выхода; не туда только от стика — флаг канала.
`Config::directedUs(us, reversed)` — зеркало импульса относительно 1500.

### Установка датчиков

| Константа | Значение | Смысл |
|---|---|---|
| `IMU_ROTATION_CW_DEG` | 90 | Поворот осей чипа IMU вокруг вертикали (0/90/180/270), куда смотрит ось X чипа. Используется, только пока нет калибровки установки `o` в NVS |
| `IMU_MOUNT_CALIBRATION_REQUIRED` | `false` (Matek H743 — `true`) | ARM со стабилизацией — только после калибровки установки `o`: IMU распаян на плате полётника, а плату ставят как угодно. Своё у каждой платы, в её блоке пинов |
| `MAG_ROTATION_CW_DEG` | 0 | То же для компаса (калибровки установки у компаса нет) |

### Шасси, калибровка на земле, сигналы

| Константа | Значение | Смысл |
|---|---|---|
| `LANDING_GEAR` | `LandingGear::TAIL_WHEEL` | Какое колесо рулевое: `TAIL_WHEEL` — хвостовое (на стоянке нос задран, на разбеге сначала хвост вверх), `NOSE_WHEEL` — носовое |
| `CAL_SWITCH` | `Channels::SWB` | Тумблер жеста калибровки самолёта на земле (не SwD — он переключает реверс мотора) |
| `CAL_SWITCH_FLIPS` / `CAL_SWITCH_WINDOW_MS` | 3 / 4000 | Без ARM, газ внизу: столько раз вниз-вверх за это время — калибровка (повтор — отмена) |
| `CAL_STILL_MS`, `CAL_STILL_DPS`, `CAL_STILL_G` | 2000, 3.0, 0.15 | Шаг засчитывается: столько мс подряд вращение меньше и перегрузка 1g ± столько |
| `CAL_PARK_WAIT_MS` | 5000 | Шаг 2 ждёт, пока самолёт переставят, — или столько (носовое колесо: та же поза) |
| `CAL_STEP_TIMEOUT_MS` | 60 000 | Не замер за это время — калибровка отменяется (ошибка) |
| `CAL_TAIL_PARK_MIN/MAX_DEG` | 3 / 25 | Хвостовое колесо: стояночный тангаж в этих пределах, иначе ошибка («хвост не опустили») |
| `CAL_NOSE_PARK_MAX_DEG`, `CAL_PARK_MAX_ROLL_DEG` | 6, 10 | Носовое колесо: стоянка почти ровно; крен на стоянке — не больше |
| `SIGNAL_MOTOR` / `SIGNAL_SURFACES` | `true` / `true` | Сигнал мотором (пауза импульсов ESC — регулятор играет мелодию готовности) и кивком рулей |
| `SIGNAL_NOD_US`, `SIGNAL_NOD_MS` | 200, 150 | Кивок рулей высоты и направления: столько мкс вверх, потом вниз, по столько мс |
| `SIGNAL_ESC_PAUSE_MS`, `SIGNAL_PERIOD_MS` | 1000, 3000 | Импульсы на ESC пропадают на столько; сигналы идут с таким периодом (мелодия успевает доиграть) |

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
| Автовзлёт | `TAKEOFF_TRIGGER_US` 1500; проверка: `TAKEOFF_CHECK_MS` 1000, `_CHECK_TIMEOUT_MS` 5000, `TAKEOFF_PARK_PITCH/ROLL_TOL_DEG` 4/5, `TAKEOFF_UNCALIBRATED_TILT_DEG` 10; газ `TAKEOFF_THROTTLE_PCT` 100 за `TAKEOFF_THROTTLE_RAMP_MS` 2000; курс `TAKEOFF_HEADING_KP` 10, `TAKEOFF_YAW_RATE_KD` 2; `TAKEOFF_ROTATE_SPEED_MS` 11; отрыв `TAKEOFF_LIFTOFF_MS` 3000, `_LIFTOFF_ALT_M` 1, `_LIFTOFF_CONFIRM_MS` 500; набор `TAKEOFF_CLIMB_PITCH_DEG` 8 до `TAKEOFF_SAFE_ALTITUDE_M` 30 (без барометра — `TAKEOFF_CLIMB_MS` 15 000); прерывание: `TAKEOFF_ABORT_ROLL_DEG` 15, `_ABORT_HEADING_DEG` 30, `TAKEOFF_NOSE_DOWN_MS` 300, `TAKEOFF_GROUND_TIMEOUT_MS` 15 000; «только гироскоп»: `TAKEOFF_GYRO_ONLY_CLIMB/MAX/ABORT_MS` 5000/30 000/3000 |
| Хвостовое колесо | `TD_TAIL_UP_ELEVATOR_US` 250 (руль от себя не больше), `TD_ROLL_PITCH_DEG` 1, `TD_PITCH_KP` 40 (хвост вверх), `TD_ROLL_PITCH_KP` 15 (разбег и отрыв), `TD_PITCH_KD` 3, `TD_TAIL_UP_DONE_DEG` 2, `TD_TAIL_UP_TIMEOUT_MS` 6000, `TD_ROLL_MS` 2500, `TD_ROTATE_RATE_DPS` 2, `TD_TAIL_CLEARANCE_DEG` 3, `TD_MIN_PITCH_DEG` −3 |
| Носовое колесо | `TRI_ROLL_ELEVATOR_US` 0, `TRI_ROLL_MS` 3000, `TRI_ROTATE_RATE_DPS` 5, `TRI_MAX_GROUND_PITCH_DEG` 10 |
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

### Чёрный ящик

Подробно — [BLACKBOX.md](../BLACKBOX.md).

| Константа | Значение | Смысл |
|---|---|---|
| `BLACKBOX_RING_BYTES` / `_NO_PSRAM_BYTES` | 4 МБ / 32 КБ | Очередь записей в PSRAM (без PSRAM — во внутренней памяти) |
| `BLACKBOX_RING_STM32_BYTES` | 384 КБ | STM32H743: очередь в ОЗУ — 10 с предзаписи и запас на задержки карты |
| `BLACKBOX_SD_FILE` / `BLACKBOX_SD_MAX_BYTES` | `BLACKBOX.BIN` / 256 МБ | STM32H743: файл на SD-карте и потолок его используемой части (время сверки при включении растёт вместе с областью) |
| `BLACKBOX_PREROLL_MS` / `_POSTROLL_MS` | 10 000 / 10 000 | Запись до старта (ARM + газ) и после DISARM |
| `BLACKBOX_LANDED_STOP_MS` | 30 000 | Заармлено, мотор стоит, самолёт неподвижен столько — стоп |
| `BLACKBOX_LANDED_GYRO_DPS` / `_ACCEL_G` / `_CLIMB_MS` / `_SPEED_MS` | 5 / 0.1 / 0.5 / 2 | Что считается «неподвижен» |
| `BLACKBOX_RESET_HOLD_MS` | 60 000 | После сбойной перезагрузки — запись не короче |
| `BLACKBOX_MIN_FREE_BYTES` | 10 МБ | Стёртого места наготове; старые полёты стираются целиком на земле |
| `BLACKBOX_ERASE_PAUSE_MS` | 100 | Пауза между стираниями |
| `BLACKBOX_IMU_DIVIDER` | 1 | IMU каждый N-й такт (1 — 500 Гц) |
| `BLACKBOX_VBAT_DIVIDER` / `_CURRENT_DIVIDER` | 6.6 / 1.667 (Matek H743: 11 / 1) | Делители батареи (56k/10k) и датчика тока (10k/15k) на плате полётника |

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
