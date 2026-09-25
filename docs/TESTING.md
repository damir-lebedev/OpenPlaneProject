# TESTING.md — тесты, покрытие и статический анализ

Прошивка проверяется на двух уровнях:

| Где | Команда | Что |
|---|---|---|
| **ПК (native)** | `pio test -e native` | Все наборы: заголовки прошивки собираются на ПК без изменений, железо заменено управляемыми фейками. Считается покрытие |
| **Плата** | `pio test -e esp32-s3` | `test_feedback` и `test_imu_orientation` на реальном ESP32-S3 (прошивает тестовую прошивку; потом верните обычную: `pio run -t upload`) |

Архитектурный контекст — [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-тестируемость).

---

## Быстрый старт

```bash
pip install platformio gcovr        # один раз
pio test -e native                  # все нативные тесты (~40 с)
gcovr                               # покрытие по файлам (настройки — gcovr.cfg)
gcovr --html-details -o coverage/index.html   # HTML-отчёт (coverage/ в .gitignore)

pio test -e native -f native/test_rc          # один набор
pio test -e native -f test_feedback           # симуляция обратной связи на ПК
```

Перед подсчётом покрытия после изменений в тестах полезно начать с чистой
сборки: `rm -rf .pio/build/native`, иначе в отчёт попадут счётчики прошлых
запусков.

---

## Как устроена нативная сборка

`[env:native]` в `platformio.ini`: `platform = native`, Unity, `-std=gnu++17`,
`-D BOARD_ESP32_S3` (распиновка основной платы), `-I test/native/support`,
`-Wall -Wextra -Wshadow`, покрытие `--coverage` и
`-fkeep-inline-functions -fkeep-static-functions` — без них gcov не видит
ни разу не вызванные функции заголовков и завышает покрытие.

### Фейки железа — `test/native/support/`

Заголовки с теми же именами и сигнатурами, что у Arduino core ESP32 2.0.x,
ESP-IDF и библиотек, но поверх симулированного мира в `namespace fake`:

| Файл | Заменяет | Что умеет симуляция |
|---|---|---|
| `Arduino.h`, `Print.h`, `WString.h`, `Stream.h` | ядро Arduino | Макросы (`constrain`, `sq`, `DEG_TO_RAD`…), `map()`, `String`, форматирование `print()` как у оригинала. `ARDUINO` намеренно **не** определён |
| `esp32-hal-fake.h` | время, GPIO, LEDC, FreeRTOS, `ESP` | Часы идут только по `fake::advance*()`/`delay()`; `millis()/micros()` — `uint32_t`, как на ESP32 (переполнение ведёт себя как на плате). LEDC-каналы, `pulseIn` по реальной скважности (виден, только если включён входной буфер пина). Задачи регистрируются; `fake::runTask(task, n)` выполняет n проходов её бесконечного цикла. Критические секции считаются |
| `HardwareSerial.h` | UART | Порты регистрируются по номеру (`fake::uart(1)`); `pushRx()`, `txBytes()`. `Serial` = UART0 |
| `Wire.h` | I2C | Устройства по адресу; `fake::RegisterMapDevice` — регистры с автоинкрементом, журнал записей, сбои (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), хуки `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Устройства по пину CS; `fake::SpiRegisterMapDevice` — протокол Bosch/InvenSense, `dummyBytes` перед данными |
| `Preferences.h` | NVS | Хранилище в памяти, поведение `begin(readOnly)`/`get*`/`getBytes` как у оригинала; `failBegin` |
| `WiFi.h`, `WebServer.h` | Wi-Fi AP, HTTP | Результат `softAP()` задаёт тест; `WebServer::request(метод, uri, тело)` вызывает зарегистрированный обработчик; `fake::webServers()` — все экземпляры |
| `U8g2lib.h` | U8g2 | Вместо пикселей — список нарисованных строк и прямоугольников; `begin()/sendBuffer()` гоняют байты через пользовательский byte-callback; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

`test/native/helpers/TestSupport.h` — общее для наборов: `resetWorld()`
(вызывается из `setUp()`), дублёры `FakeUart`/`FakeServo`/`FakeBoard` и
датчиков (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), сборщик кадров
`ibusFrame()`, стенды `I2cRig`/`SpiRig` (драйвер поверх настоящих
`Esp32I2CBus`/`Esp32SpiBus` и `*RegisterDevice` с симулированным чипом).

Тесты на плате (`test_feedback`, `test_imu_orientation`) переносимые: при
`ARDUINO` — `setup()/loop()`, иначе `main()`. Наборы `test/native/*` на плате
не собираются (`test_ignore = native/*` в `[esp32_common]`).

---

## Наборы тестов

| Набор | Тестов | Что проверяет |
|---|---|---|
| `native/test_hal` | 17 | Помощники `II2CBus` (NACK, короткое чтение — буфер не трогается), `I2cRegisterDevice`, `SpiRegisterDevice` (бит чтения, фиктивный байт BMP388), `Esp32I2CBus` (таймаут 5 мс), `Esp32SpiBus` (режимы 0–3), `Esp32UartPort` (8N1, пины), `Esp32ServoOutput` (50 Гц/14 бит, ограничение импульса, отказ LEDC, измерение через входной буфер), `Esp32Board` (шины, UART, порядок каналов) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, разбор iBUS: кадры по частям, CRC, 12-битные значения, failsafe пульта, таймаут 500 мс (и через переполнение `micros()`), мусор, пересинхронизация |
| `native/test_control` | 21 | Закрылки (скорость, первый вызов, паузы), микшер (знаки, реверс, флапероны), газ, автомат ARM и проверки датчиков режима, таблица выходов и самопроверка импульсов |
| `native/test_autopilot` | 22 | ПИД (D по скорости датчика, интеграл, anti-windup, `dt`), все режимы, автовзлёт по времени, ALT_HOLD, планирование при потере связи, селектор CH7 |
| `native/test_flight_controller` | 12 | Полный такт `FlightController` на настоящих классах: приоритеты потеря связи > ARM > стики/автопилот > газ |
| `native/test_imu` | 21 | MPU6050/6500/9250 и ICM-42688: опознание, регистры, масштабы, поворот осей и авиационные знаки, ошибки шины, калибровка гироскопа и предполётная проверка (движение, не 1g, плата перевёрнута, установка сменилась), калибровка установки по трём позам на симулированном времени, NVS, фильтр ориентации |
| `native/test_baro_mag_gps` | 23 | `BarometerBase` (опрос, высота, вертикальная скорость, ошибки), BMP388 по I2C и SPI, BME280/BMP280 по эталону Bosch (25.08 °C, 100653.27 Па), компасы (курс, hard-iron калибровка в NVS), u-blox M10 (байты CFG-VALSET, NAV-PVT, битые кадры, таймаут), `SensorSelection` |
| `native/test_telemetry` | 27 | `LoopStats`, `LogSettings` (NVS, версия), `DebugLogger` (все каналы, допуски, периоды, пауза), `DebugConsole` (меню, горячие клавиши, запрет при ARM, сохранение только без ARM), `WebDebugServer` (маршруты, JSON, почтовый ящик, пробелы и экспонента в JSON), `OledDisplay` (байты по I2C, кадр, инверсия при потере связи) |
| `native/test_feedback_units` | 14 | Модули обратной связи по отдельности: источники скорости, в воздухе/на земле, RLS-оценка (пропуски, триммер), регулятор (anti-windup), все признаки сваливания, отмены взлёта/посадки, `printStatus` |
| `native/test_app` | 9 | `src/main.cpp` целиком на виртуальном стенде: `setup()` с симулированными MPU6500/BMP388/QMC5883P/OLED, фиксированный период `loop()`, пульт → сервы, ARM и газ, режимы, потеря связи в воздухе, консоль, дашборд, экран |
| `test_feedback` | 10 | Замкнутая симуляция самолёта с контуром обратной связи (на ПК и на плате) |
| `test_imu_orientation` | 5 | Калибровка установки IMU на 300 случайных установках (на ПК и на плате) |
| **Всего** | **197** | |

---

## Покрытие

Считается `gcovr` по `include/` и `src/` (всё, что входит в прошивку).
Состояние на момент добавления тестов:

| Слой | Строки | Ветвления |
|---|---|---|
| `autopilot` | 211/211 (100%) | 95/99 (96.0%) |
| `autopilot/feedback` | 683/702 (97.3%) | 501/570 (87.9%) |
| `control` | 210/212 (99.1%) | 120/128 (93.8%) |
| `hal` | 75/75 (100%) | 16/16 (100%) |
| `hal/esp32` | 96/96 (100%) | 18/20 (90.0%) |
| `rc` | 92/92 (100%) | 41/42 (97.6%) |
| `sensors` (все) | 1039/1040 (99.9%) | 486/541 (89.8%) |
| `telemetry` | 745/748 (99.6%) | 546/570 (95.8%) |
| `src` (`main.cpp`) | 52/52 (100%) | 8/12 (66.7%) |
| **Итого** | **3203/3228 (99.2%)** | **1831/1998 (91.6%)**; функции 510/511 (99.8%) |

Что осталось непокрытым и почему:

- **Бросок с руки** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`) —
  при `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false` недостижим; появится в
  тестах, когда константа станет настраиваемой (переезд в `Config.h`).
- **Зависимое от платы:** выход без пина (`PIN_RUDDER = -1` бывает только на
  C3), GPS без TX-пина (C3) — нативная сборка использует распиновку S3.
- **Защитные ветки**, до которых нельзя дойти через публичный API:
  `default`/`Count` в `switch` по перечислениям, `return "?"`.
- Файлы без исполняемых строк (`Config.h`, `Channels.h`, `FeedbackConfig.h`,
  структуры `ControlCommand`/`FlightOutputState`/`FlightSnapshot`/
  `FeedbackOutput`/`PhaseTargets`, макросы `SensorSelection.h`, HTML
  дашборда) в отчёт не попадают — они компилируются в тесты, но gcov в них
  нечего считать.

---

## Статический анализ

| Инструмент | Команда | Профиль |
|---|---|---|
| GCC | `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3` (и `-c3`, `-dev`); `pio run -e stm32h743` | Нативная сборка тестов — всегда с `-Wall -Wextra -Wshadow`; `stm32h743` — с `-Wall -Wextra` (`build_src_flags`; `-Wshadow` шумит на заголовках самого STM32duino) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `check_*` в `[esp32_common]`: `include/` и `src/` (кроме `stm32/`), warning/style/performance/portability, встроенные подавления `// cppcheck-suppress` только для ложных срабатываний (колбэк U8g2, `setup/loop`). У `stm32h743` — те же флаги по `include/hal/stm32/` и `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` и др.; отключённые проверки с объяснением — в самом файле |

clang-tidy запускается с фейками из `test/native/support`: заголовки ESP-IDF
clang под хост-архитектуру разобрать не может (при попытке `pio check` с
`clangtidy` анализ обрывается на ошибках разбора и честно ничего не
проверяет). `misc-include-cleaner` следит, чтобы каждый заголовок подключал
то, чем пользуется: «зонтичные» заголовки (`FeedbackModules.h`, API
`IBoard.h`/`RegisterDevice.h`, макросы `SensorSelection.h`) помечены
`// IWYU pragma: export`. Код под STM32 (`include/hal/stm32/`, `src/stm32/`)
фейками не покрыт, поэтому скрипт его пропускает — его проверяют сборка и
cppcheck env `stm32h743`. Нативными тестами он тоже не покрыт (фейков
STM32duino нет) и в `gcovr` не попадает: это заготовка без железа.

На момент аудита все три инструмента дают **0 замечаний** по коду проекта на
всех трёх платах ESP32 и на заготовке `stm32h743`, включая альтернативные наборы датчиков
(`-DSENSOR_IMU=SENSOR_IMU_ICM42688 -DSENSOR_BARO=SENSOR_BARO_BME280
-DSENSOR_MAG=SENSOR_MAG_QMC5883L -DSENSOR_GPS=SENSOR_GPS_UBLOX_M10` и
BMP388 по SPI без компаса).

---

## Как писать новые тесты

1. Модуль с логикой без железа — прямой юнит-тест: время передаётся
   параметром или двигается `fake::advanceMs()`.
2. Драйвер чипа — через `I2cRig`/`SpiRig`: регистры симулированного чипа,
   проверка записанных значений (`chip.lastWrite(reg)`) и разбора данных.
   Для формул — эталон из даташита или независимый расчёт, а не копия кода.
3. Классы с бесконечными задачами FreeRTOS — `fake::findTask("имя")` +
   `fake::runTask(task, n)`.
4. Новый набор — папка `test/native/test_<имя>/test_main.cpp` с `main()`;
   `setUp()` зовёт `resetWorld()`, если набору не нужно состояние между
   тестами.
5. Нашли баг — сначала тест, который его ловит, потом исправление.
