# TESTING.md — тесты, покрытие и статический анализ

Прошивка проверяется на двух уровнях:

| Где | Команда | Что |
|---|---|---|
| **ПК (native)** | `pio test -e native` | Заголовки прошивки собираются на ПК без изменений, железо заменено управляемыми фейками: модули, драйверы, замкнутые симуляции полёта, прошивка ESP32 целиком (S3 и 38-pin) с каждым набором датчиков. Считается покрытие |
| **ПК (native-stm32)** | `pio test -e native-stm32` | Прошивка STM32H743 целиком (`src/stm32/main.cpp`) поверх слоя фейков STM32duino: задачи FreeRTOS, флеш, MAVLink, датчики на I2C и SPI |
| **Матрица сборок** | `tools/build_matrix.sh` | 4 платы × 6 наборов датчиков с `-Wall -Wextra (-Wshadow)`; любое предупреждение в коде проекта — ошибка |
| **Плата** | `pio test -e esp32-s3` | `test_feedback` и `test_imu_orientation` на реальном ESP32-S3 (прошивает тестовую прошивку; потом верните обычную: `pio run -t upload`) |
| **Плата STM32** | `pio test -e stm32h743-devebox -f test_blackbox_sd` | Чёрный ящик на **настоящей SD-карте** DevEBox H743, плюс `test_feedback` и `test_imu_orientation` на Cortex-M7 — [ниже](#тесты-на-плате-stm32) |

Архитектурный контекст — [`ARCHITECTURE.md §12`](ARCHITECTURE.md#12-тестируемость).

---

## Быстрый старт

```bash
pip install platformio gcovr        # один раз
# Windows: нужен g++ в PATH, например WinLibs (winlibs.com, zip UCRT):
# распаковать и добавить mingw64\bin в PATH — установка не нужна
pio test -e native -e native-stm32  # все нативные тесты (~1.5 мин)
gcovr                               # покрытие по файлам (настройки — gcovr.cfg)
tools/build_matrix.sh               # все платы × все датчики (~25 мин)
gcovr --html-details -o coverage/index.html   # HTML-отчёт (coverage/ в .gitignore)

pio test -e native -f native/test_rc          # один набор
pio test -e native -f test_feedback           # симуляция обратной связи на ПК

# Траектории замкнутых симуляций в CSV (для графиков):
OPENPLANE_SIM_DIR=/tmp/sim pio test -e native -f native/test_sim
# Поток MAVLink — на проверку эталонным декодером (pip install pymavlink):
OPENPLANE_MAVLINK_DUMP=/tmp/tlm.bin pio test -e native -f native/test_mavlink
python3 tools/check_mavlink.py /tmp/tlm.bin
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
| `esp32-hal-fake.h` | время, GPIO, АЦП, LEDC, FreeRTOS, PSRAM, `ESP` | Часы идут только по `fake::advance*()`/`delay()`; `millis()/micros()` — `uint32_t`, как на ESP32 (переполнение ведёт себя как на плате). LEDC-каналы, `pulseIn` по реальной скважности (виден, только если включён входной буфер пина), `analogReadMilliVolts` — напряжение из `fake::gpio().analogMv`. Задачи регистрируются (хэндл ненулевой); `fake::runTask(task, n)` выполняет n проходов её бесконечного цикла, `ulTaskNotifyTake` считается проходом, `xTaskNotifyGive` — счётчиком. Мьютексы FreeRTOS — флаг «занят». `psramFound()`/`ps_malloc()`. Критические секции считаются |
| `HardwareSerial.h` | UART | Порты регистрируются по номеру (`fake::uart(1)`); `pushRx()`, `txBytes()`, смена скорости на ходу (`updateBaudRate`, история — `baudChanges()`). `Serial` = UART0 |
| `esp_partition.h` | разделы флеша ESP-IDF | Раздел — вектор байт с поведением NOR: стирание только секторами 4 КБ, стёртое = 0xFF, запись только опускает биты (попытка поднять бит считается — `bitRaises`); `beforeWrite` — «пропало питание»; счётчики чтений, записей, стираний |
| `esp_system.h` | причина перезагрузки | `esp_reset_reason()` из `fake::chip().resetReason` |
| `Wire.h` | I2C | Устройства по адресу; `fake::RegisterMapDevice` — регистры с автоинкрементом, журнал записей, сбои (`present`, `failWrites`, `failReads`, `failReadIf`, `shortRead`), хуки `beforeRead`/`onRegisterWrite` |
| `SPI.h` | SPI | Устройства по пину CS; `fake::SpiRegisterMapDevice` — протокол Bosch/InvenSense, `dummyBytes` перед данными |
| `Preferences.h` | NVS | Хранилище в памяти, поведение `begin(readOnly)`/`get*`/`getBytes` как у оригинала; `failBegin` |
| `WiFi.h`, `WebServer.h` | Wi-Fi AP, HTTP | Результат `softAP()` задаёт тест; `WebServer::request(метод, uri, тело)` вызывает зарегистрированный обработчик; `fake::webServers()` — все экземпляры |
| `U8g2lib.h` | U8g2 | Вместо пикселей — список нарисованных строк и прямоугольников; `begin()/sendBuffer()` гоняют байты через пользовательский byte-callback; `fake::displays()` |
| `soc/*.h` | ESP-IDF | `SOC_I2C_NUM = 2`, `GPIO_PIN_MUX_REG`, `PIN_INPUT_ENABLE` |

### Слой STM32duino — `test/native/support_stm32/` (env `native-stm32`)

Лежит в `-I` раньше `support/` и дополняет те же фейки тем, что есть только у
STM32duino. `<Preferences.h>` в этой среде — **настоящий**
`include/hal/stm32/compat/Preferences.h` поверх `KeyValueStore`.

| Файл | Заменяет | Что умеет |
|---|---|---|
| `Arduino.h` | ядро STM32duino | пины `PA0..PE15` (порт·16 + номер), `pin_size_t`, `PinMap_TIM` для пинов сервовыходов, `HardwareTimer` (импульс виден `fake::timerPulseUs(pin)` и `pulseIn()`), `Uart`, `noInterrupts()` |
| `STM32FreeRTOS.h` | STM32duino FreeRTOS | `xTaskCreate` в общий реестр задач (стек в словах), `vTaskStartScheduler()` возвращается — задачи тест крутит сам (`fake::runTask`), `xPortGetFreeHeapSize` |
| `EEPROM.h` | EEPROM-эмуляция | «флеш» 8 КБ (стёрт = 0xFF) и буфер, `fake::eeprom()` — счётчики и порча образа |
| `SPI.h` | | `SPIMode` |

В общих фейках для STM32 добавлено: `TwoWire(sda, scl)`, `setSDA/SCL` и
`fake::wireWithSda(pin)` (найти вторую шину платы), `HardwareSerial(rx, tx)` и
`fake::uartByRx(pin)`, `SPIClass::setSCLK/MISO/MOSI`.

### Эмуляторы чипов и модель самолёта — `test/native/helpers/`

| Файл | Что это |
|---|---|
| `ChipEmulators.h` | LSM6DSV, ICM-45686 (с косвенными регистрами IPREG), QMC6309, SPL06-001, BMP581, кадры NAV-PVT u-blox — регистровые карты на I2C или SPI, данные из «мира» `World` (углы и скорости, высота, воздушная скорость, курс, координаты), в осях чипа с учётом `IMU_ROTATION_CW_DEG` |
| `PlaneSim.h` | модель самолёта ~1.2 кг: точка с массой + вращение по крену/тангажу, CL(α) со сваливанием, сопротивление, тяга, ветер, термики, земля |
| `SimHarness.h` | замкнутый контур: пульт → кадр iBUS → `IBusReceiver` → `PilotSwitches` → `Autopilot` → `FlightController` → ШИМ → отклонения рулей → `PlaneSim` → датчики (в том числе трубка Пито на двух шумных барометрах). CSV-траектория при `OPENPLANE_SIM_DIR` |

`test/native/helpers/TestSupport.h` — общее для наборов: `resetWorld()`
(вызывается из `setUp()`), дублёры `FakeUart`/`FakeServo`/`FakeBoard` и
датчиков (`FakeImu`, `FakeBaro`, `FakeMag`, `FakeGps`), сборщик кадров
`ibusFrame()`, стенды `I2cRig`/`SpiRig` (драйвер поверх настоящих
`Esp32I2CBus`/`Esp32SpiBus` и `*RegisterDevice` с симулированным чипом).

Тесты на плате (`test_feedback`, `test_imu_orientation`) переносимые: при
`ARDUINO` — `setup()/loop()`, иначе `main()`. Наборы `test/native/*` на плате
не собираются (`test_ignore` в `[esp32_common]` и `[env:stm32h743]`: шаблоны —
каждый на своей строке, через пробел PlatformIO читает их как один).
На STM32: `pio test -e stm32h743`.

---

## Наборы тестов

| Набор | Тестов | Что проверяет |
|---|---|---|
| `native/test_hal` | 17 | Помощники `II2CBus` (NACK, короткое чтение — буфер не трогается), `I2cRegisterDevice`, `SpiRegisterDevice` (бит чтения, фиктивный байт BMP388), `Esp32I2CBus` (таймаут 5 мс), `Esp32SpiBus` (режимы 0–3), `Esp32UartPort` (8N1, пины), `Esp32ServoOutput` (50 Гц/14 бит, ограничение импульса, отказ LEDC, измерение через входной буфер), `Esp32Board` (шины, UART, порядок каналов, AUX, пищалка) |
| `native/test_rc` | 16 | `RcChannelState`, `RcInput`, разбор iBUS: кадры по частям, CRC, 12-битные значения, failsafe пульта, таймаут 500 мс (и через переполнение `micros()`), мусор, пересинхронизация |
| `native/test_control` | 21 | Закрылки (скорость, первый вызов, паузы), микшер (знаки, реверс, флапероны), газ, автомат ARM и проверки датчиков режима, таблица выходов и самопроверка импульсов |
| `native/test_autopilot` | 22 | ПИД (D по скорости датчика, интеграл, anti-windup, `dt`), STABILIZE как режим углов, автовзлёт по времени, ALT_HOLD рулём высоты, планирование при потере связи |
| `native/test_autopilot_modes` | 31 | Все 12 режимов и реакция каждого на отсутствие датчика, таблица привязок и `static_assert`, функции и крутилки, навигация (курс, круг, дом, геозабор), failsafe RTH/планирование, запуск с руки, парение, автотриммер (запись только на земле) |
| `native/test_flight_controller` | 12 | Полный такт `FlightController` на настоящих классах: приоритеты потеря связи > ARM > стики/автопилот > газ; AUX, `MOTOR_KILL`, пищалка |
| `native/test_imu` | 21 | MPU6050/6500/9250 и ICM-42688: опознание, регистры, масштабы, поворот осей и авиационные знаки, ошибки шины, калибровка гироскопа и предполётная проверка, калибровка установки по трём позам, NVS, фильтр ориентации |
| `native/test_baro_mag_gps` | 23 | `BarometerBase`, BMP388 по I2C и SPI, BME280/BMP280 по эталону Bosch, компасы (курс, hard-iron калибровка в NVS), u-blox M10 (CFG-VALSET, NAV-PVT, битые кадры, таймаут), `SensorSelection` |
| `native/test_sensors_new` | 23 | LSM6DSV (16X/32X, запасной адрес, SPI), ICM-45686 (косвенные регистры), QMC6309, SPL06-001 (формулы датащита), BMP581 (DRDY и запасной путь), трубка Пито (ноль, фильтр, плотность, перепутанные шланги, устаревшие данные, «полёт» с шумом двух барометров) |
| `native/test_storage` | 16 | `KeyValueStore` (перезагрузка, износ — одинаковое не пишется, переполнение без потери данных, CRC, пропажа питания при стирании, мусор, версия формата), `KvPreferences` (поведение как у NVS ESP32) |
| `native/test_mavlink` | 20 | Кодек против эталонных кадров pymavlink (v1, v2, подписанный), CRC, пересинхронизация; телеметрия: частоты потоков, HEARTBEAT/ATTITUDE/POSITION/HUD/GPS/SYS_STATUS, параметры ПИД (список, чтение, запись, отказ плохих значений), смена режима с земли, ARM с земли — отказ, миссии — 0, переполненный буфер UART не блокирует цикл |
| `native/test_blackbox` | 19 | Чёрный ящик: формат и CRC, кольцо секторов на NOR-фейке (свежий раздел без стирания, мусор стирается всегда, старые полёты — целиком и только ради места, последний не трогается, переход через конец кольца, голова после перезагрузки, обрыв питания, недописанная запись по CRC), запись полёта на настоящих `FlightController`/`Autopilot`: старт по ARM и газу с предзаписью, стоп после DISARM и «стоит на земле», потеря связи не останавливает, запись после сбойной перезагрузки, вручную, события, батарея, флеш кончился в воздухе, полёт длиннее раздела, выгрузка кадрами с CRC и сменой скорости, меню консоли `k`, нет раздела — выключен |
| `native/test_blackbox_scan` | 3 | Выборочная сверка кольца при включении против полной: 300 случайных историй колец × 5 шагов проб (голова, номера и список полётов совпадают, а когда картина не сходится — уступает полной) и стоимость на области SD 64 МБ (≈530 чтений вместо 32 тысяч) |
| `native/test_telemetry` | 28 | `LoopStats`, `LogSettings` (NVS, версия), `DebugLogger` (все каналы, NAV), `DebugConsole` (меню, горячие клавиши, опрос шин `b`, запрет при ARM, сохранение только без ARM), `WebDebugServer` (маршруты, JSON, почтовый ящик), `OledDisplay` (байты по I2C, кадр, инверсия при потере связи) |
| `native/test_sim` | 15 | Замкнутые полёты всей прошивки с моделью самолёта: выход из крена, CRUISE в боковой ветер, LOITER, RTH, failsafe RTH/планирование, геозабор, автовзлёт с полосы, запуск с руки, автопосадка, термик, RESCUE из спирали, удержание скорости и защита от сваливания, настоящая трубка Пито в контуре, автотриммер «кривого» самолёта, отказы датчиков в полёте (IMU, барометр, трубка Пито, GPS) |
| `native/test_feedback_units` | 14 | Модули обратной связи по отдельности: источники скорости, в воздухе/на земле, RLS-оценка, регулятор, признаки сваливания, отмены взлёта/посадки |
| `native/test_app` | 10 | `src/main.cpp` на ESP32-S3 со стендовым набором MPU6500/BMP388/QMC5883P/OLED: период `loop()`, пульт → сервы, ARM, режимы, потеря связи, консоль, дашборд, экран, чёрный ящик (задача на ядре 0, запись по газу, полёт после DISARM, `bb list`) |
| `native/test_app_lsm6dsv_pitot` | 9 | `src/main.cpp` на ESP32-S3 с лётным набором: LSM6DSV + QMC6309 + SPL06 + BMP581 в трубке + GPS — опознание всех чипов, ноль трубки и скорость, высота, дом по GPS, STABILIZE по углам с чипа, RTH на дом, опрос шин, дашборд |
| `native/test_app_icm45686_esp32dev` | 5 | `src/main.cpp` на **ESP32 38-pin** (`BOARD_ESP32_CLASSIC`) с набором ICM-45686 + QMC6309 + SPL06 + BMP581: распиновка платы, фильтры IPREG, запуск с руки, стабилизация и скорость, опрос одной шины |
| `native_stm32/test_app_stm32_lsm6dsv_pitot` | 9 | `src/stm32/main.cpp` на **STM32H743** с лётным набором: задачи и приоритеты, период 2 мс, трубка, таймеры ШИМ и `pulseIn`, MAVLink в полёте, смена режима из GCS, запись настроек фоновой задачей во «флеш», экран на I2C1, консоль |
| `native_stm32/test_app_stm32_icm45686_spi` | 4 | STM32H743 с ICM-45686 и BMP581 **по SPI** + QMC6309: испорченный флеш при включении, ALT_HOLD из GCS держит высоту, потеря связи → RTH, видно в MAVLink; перезапись битого образа |
| `native_stm32/test_blackbox_sd` | 29 | Чёрный ящик на SD-карте: FAT32 (MBR и без, каталог на двух кластерах, шумовые записи, чужой/разбросанный/пустой том), `SdFileRegion` (неполные блоки, кэш, стирание, границы, сбои), настоящий драйвер `Stm32SdCard` поверх фейкового `HAL_SD` (4 бита, запасные скорости, повтор, занятая карта, невыровненные буферы), кольцо на карте (перезагрузка, пропажа питания, стоимость сверки), метка «кольцо пусто», запись полёта на `FlightController`, сбойная перезагрузка по `RCC->RSR`, АЦП батареи, ошибки карты в полёте, медленная карта, выгрузка по консоли, клавиша `D` |
| `native_stm32/test_app_stm32_blackbox_sd` | 3 | `src/stm32/main.cpp` с картой: загрузка находит карту и файл, задача `bbox` пишет полёт, период цикла не растягивается, `bb list` |
| `native_stm32/test_app_stm32_no_sd` | 2 | `src/stm32/main.cpp` без карты: ящик выключен и объясняет почему, самолёт летает, меню `k` не ломается |
| `test_feedback` | 10 | Замкнутая симуляция самолёта с контуром обратной связи (на ПК и на плате) |
| `test_imu_orientation` | 5 | Калибровка установки IMU на 300 случайных установках (на ПК и на плате) |
| **Всего** | **387** | 340 в `native` + 47 в `native-stm32` (плюс 9 только на плате — `test_blackbox_sd`) |

### Тесты на плате STM32

`test/test_blackbox_sd` — не нативный: драйвер SDMMC, карта и время настоящие.
Тесты идут в задаче FreeRTOS, а рядом работает задача-имитатор полётного цикла
с высшим приоритетом (период 2 мс): она вытесняет тесты посреди обращений к карте,
как в прошивке. Без неё не поймать ошибку, которую на плате и нашли: при
вытеснении FIFO SDMMC переполнялся (`HAL_SD_ERROR_RX_OVERRUN`), в голом
цикле этого нет.

| Тест | Что проверяет |
|---|---|
| `reset_cause_is_a_normal_one` | причина перезагрузки (`RCC->RSR`) — не сторожевой таймер и не просадка |
| `card_is_detected_on_four_bit_bus` | карта опознана на 4 битах и 24 МГц |
| `file_is_found_and_contiguous` | `BLACKBOX.BIN` найден на FAT32 и лежит подряд |
| `multi_block_writes_work_at_every_length` | запись 1, 2, 4, 8 блоков одним обращением |
| `pages_write_with_bounded_latency_and_read_back_intact` | страницы по 256 Б: худшая запись < 250 мс (предел SD), устойчиво > 40 КБ/с, чтение и стирание |
| `header_scan_cost_on_the_whole_area` | цена чтения заголовка сектора и полной сверки |
| `storage_erase_all_write_flights_and_find_them_after_reopen` | стирание всего, два полёта по 20 000 записей, «перезагрузка»: сверка выборочная и < 2 с, записи читаются по порядку с верным CRC; пустое кольцо — по метке за < 100 мс |
| `blackbox_keeps_up_for_20_seconds_in_real_time` | настоящий `BlackBox` на 500 Гц IMU в реальном времени: ни одной потерянной записи, полёт читается после «перезагрузки» |
| `the_flight_task_was_not_disturbed` | запись на карту не сбила период задачи-имитатора (отклонение < 3 мс) |

Запуск (карта с файлом — `python tools/blackbox.py sd-prepare E:`; **тест стирает все
полёты в файле**):

```bash
pio test -e stm32h743-devebox -f test_blackbox_sd
pio test -e stm32h743-devebox -f test_feedback -f test_imu_orientation
```

Плата в режиме DFU (на DevEBox — провод BT0→3V3 и RST, драйвер WinUSB через
Zadig, подробно — [DEVELOPER_GUIDE](DEVELOPER_GUIDE.md#stm32h743)). Консоль
STM32 — USB CDC: после заливки порт появляется не сразу, и `pio test` иногда
не успевает его открыть («could not open port») — тогда запустите
`pio test ... --without-testing` и читайте вывод любой терминалкой с включённым
DTR (тесты ждут, пока порт откроют, до 60 с). После тестов плата ждёт клавишу
**`D`** — она перезагружает в DFU без провода.

Результаты на DevEBox H743 + карта 16 ГБ (2026-10-02): `test_blackbox_sd` — 9/9,
`test_feedback` — 10/10, `test_imu_orientation` — 5/5; цифры скорости карты —
в [BLACKBOX.md](BLACKBOX.md#что-измерено-на-плате).

### Стендовые прошивки — `test/bench/`

Не наборы тестов, а отдельные мини-проекты PlatformIO, которые заливаются
на плату вместо лётной прошивки (`pio test` их не видит: папки не
начинаются с `test_`). Пины и пределы берут из общего `Config.h`.

| Проект | Что делает |
|---|---|
| `bench/elevator_sweep` | Программно качает стик руля высоты (CH2) через `ControlMixer` и `FlightOutputs`, как живой стик: вверх 100% хода, вниз 60%, плавно, с паузами; 20 с работы — 20 с в нейтрали. В крайних положениях меряет импульс на выходах. Газ на минимуме |

Залить: `pio run -d test/bench/elevator_sweep -t upload`. Вернуть лётную
прошивку: `pio run -e esp32-s3 -t upload`.

---

## Покрытие

Считается `gcovr` по `include/` и `src/` (всё, что входит в прошивку), по
обеим нативным средам вместе:
`gcovr -r . --filter include/ --filter src/ .pio/build/native .pio/build/native-stm32`.

| Слой | Строки | Ветвления |
|---|---|---|
| `autopilot` | 920/943 (97.6%) | 645/731 (88.2%) |
| `autopilot/feedback` | 683/702 (97.3%) | 501/570 (87.9%) |
| `control` | 252/256 (98.4%) | 171/189 (90.5%) |
| `hal` | 98/102 (96.1%) | 26/26 (100%) |
| `hal/esp32` | 101/102 (99.0%) | 21/22 (95.5%) |
| `hal/stm32` | 149/158 (94.3%) | 35/52 (67.3%) |
| `rc` | 92/92 (100%) | 41/42 (97.6%) |
| `sensors` (все) | 1444/1446 (99.9%) | 716/835 (85.7%) |
| `storage` | 220/220 (100%) | 158/178 (88.8%) |
| `telemetry` | 1413/1440 (98.1%) | 1123/1269 (88.5%) |
| `src` (`main.cpp`, `stm32/main.cpp`) | 118/123 (95.9%) | 20/29 (69.0%) |
| **Итого** | **5490/5584 (98.3%)** | **3457/3943 (87.7%)**; функции 877/902 (97.2%) |

Что осталось непокрытым и почему:

- **Бросок с руки** (`TakeoffSequencer`: `WaitLaunch`, `launchDetected()`) —
  при `FeedbackConfig::TAKEOFF_HAND_LAUNCH = false` недостижим; появится в
  тестах, когда константа станет настраиваемой (переезд в `Config.h`).
- **Зависимое от платы:** выход без пина (`PIN_RUDDER = -1` бывает только на
  C3), GPS без TX-пина (C3) — нативные тесты гоняют распиновку S3, 38-pin и
  STM32, но не C3 (C3 проверяется матрицей сборок).
- **STM32:** ветки ошибок ядра (нет таймера на пине, пул таймеров исчерпан),
  сообщение «FreeRTOS не запустился» — на ПК `vTaskStartScheduler()` всегда
  возвращается.
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
| GCC | `tools/build_matrix.sh` (или `PLATFORMIO_BUILD_SRC_FLAGS="-Wall -Wextra -Wshadow" pio run -e esp32-s3`) | Все платы × все наборы датчиков. Нативная сборка тестов — всегда с `-Wall -Wextra -Wshadow`; `stm32h743` — с `-Wall -Wextra` (`build_src_flags`; `-Wshadow` шумит на заголовках самого STM32duino) |
| cppcheck | `pio check -e esp32-s3`; `pio check -e stm32h743` | `check_*` в `[esp32_common]`: `include/` и `src/` (кроме `stm32/`), warning/style/performance/portability, встроенные подавления `// cppcheck-suppress` только для ложных срабатываний (колбэк U8g2, `setup/loop`). У `stm32h743` — те же флаги по `include/hal/stm32/` и `src/stm32/` |
| clang-tidy | `tools/clang-tidy.sh` | `.clang-tidy`: bugprone, clang-analyzer, performance, `misc-include-cleaner` и др.; отключённые проверки с объяснением — в самом файле |

clang-tidy запускается с фейками из `test/native/support`: заголовки ESP-IDF
clang под хост-архитектуру разобрать не может (при попытке `pio check` с
`clangtidy` анализ обрывается на ошибках разбора и честно ничего не
проверяет). `misc-include-cleaner` следит, чтобы каждый заголовок подключал
то, чем пользуется: «зонтичные» заголовки (`FeedbackModules.h`, API
`IBoard.h`/`RegisterDevice.h`, макросы `SensorSelection.h`) помечены
`// IWYU pragma: export`. Код под STM32 (`include/hal/stm32/`, `src/stm32/`)
скрипт пропускает — его проверяют сборка, cppcheck env `stm32h743` и тесты
env `native-stm32`.

Матрица сборок на момент последнего прохода — **24/24 без предупреждений**:

| Плата | bench-gy521 | lsm6dsv-pitot | icm45686-pitot | lsm6dsv-spi + spl06-spi | icm45686-spi + bmp581-spi | icm45686 + bmp581 i2c |
|---|---|---|---|---|---|---|
| esp32-s3 | OK | OK | OK | OK | OK | OK |
| esp32-dev (38 pin) | OK | OK | OK | OK | OK | OK |
| esp32-c3 | OK | OK | OK | OK | OK | OK |
| stm32h743 | OK | OK | OK | OK | OK | OK |

cppcheck (`esp32-s3`, `stm32h743`) — 0 замечаний по коду проекта.

---

## Как писать новые тесты

1. Модуль с логикой без железа — прямой юнит-тест: время передаётся
   параметром или двигается `fake::advanceMs()`.
2. Драйвер чипа — через `I2cRig`/`SpiRig`: регистры симулированного чипа,
   проверка записанных значений (`chip.lastWrite(reg)`) и разбора данных.
   Для формул — эталон из даташита или независимый расчёт, а не копия кода.
3. Классы с бесконечными задачами FreeRTOS — `fake::findTask("имя")` +
   `fake::runTask(task, n)`; так же крутится полётная задача STM32.
4. Прошивка целиком с другим набором датчиков или другой платой — отдельный
   набор, который до `#include "../../../src/main.cpp"` задаёт
   `SENSOR_KIT` (или `#undef BOARD_ESP32_S3` + `#define BOARD_ESP32_CLASSIC`);
   чипы — `helpers/ChipEmulators.h`. Для STM32 — `test/native_stm32/`.
5. Новый режим автопилота — сценарий замкнутого полёта в `test_sim`.
6. Новый набор — папка `test/native/test_<имя>/test_main.cpp` с `main()`;
   `setUp()` зовёт `resetWorld()`, если набору не нужно состояние между
   тестами.
7. Нашли баг — сначала тест, который его ловит, потом исправление.
