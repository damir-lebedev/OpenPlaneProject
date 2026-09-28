# TELEMETRY — лог, консоль, веб-дашборд, OLED

[← Справочник](README.md)

Телеметрия полностью отделена от полётной логики: она только читает
константные геттеры `FlightController`, `Autopilot`, датчиков и `LoopStats`.
Единственный путь «обратно» — команды дашборда через почтовый ящик
`WebDebugServer`, применяемые полётным циклом.

---

## `LoopStats`

**Файл:** `telemetry/LoopStats.h` · **Вид:** struct

Частота и длительность полётного цикла.

| Член | Описание |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | Публикуются раз в секунду; читаются из других задач (32-битные — без «рваного» чтения) |
| `void record(uint32_t durationUs)` | Вызывать каждый такт из `loop()` |
| `uint32_t takePeakUs()` | Худший такт с прошлого вызова (для строки SYS раз в 10 с); вызывать из той же задачи, что `record()` |

`maxUs` — худшее только за последнюю секунду; редкий затык виден через
`takePeakUs()`.

---

## `LogSettings`

**Файл:** `telemetry/LogSettings.h` · **Зависит от:** `Preferences` (NVS, пространство `debuglog`)

### `LogChannel` (enum class)

| Канал | Префикс | Что выводит | По умолчанию |
|---|---|---|---|
| `Status` | `STAT` | связь, ARM, режим, закрылки, датчики | при изменении |
| `Rc` | `RC` | каналы пульта | выкл |
| `Outputs` | `OUT` | выходы на рули и ESC | выкл |
| `Attitude` | `ATT` | крен, тангаж, курс | выкл |
| `Autopilot` | `AP` | цели и коррекции | выкл |
| `Altitude` | `ALT` | высота, вертикальная скорость | выкл |
| `Heading` | `MAG` | курс по компасу | выкл |
| `Gps` | `GPS` | спутники, координаты | выкл |
| `Imu` | `IMU` | гироскоп и акселерометр | выкл |
| `Nav` | `NAV` | дом, курс, скорость, трубка Пито, включённые функции | выкл |
| `System` | `SYS` | частота цикла, память (раз в 10 с), только выкл/вкл | вкл |
| `Count` | — | число каналов | — |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`.

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`.

| Метод | Описание |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | Строка таблицы каналов |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 мс (по кругу) |
| `LogSettings()`, `void setDefaults()` | Режимы по умолчанию, период 1 с |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | Режим канала |
| `void setMode(uint8_t, LogMode)` | Для `periodicOnly` `OnChange` превращается в `Periodic` |
| `void cycleMode(uint8_t)` | выкл → при изменении → постоянно → выкл (SYS: выкл ↔ вкл) |
| `void setAll(LogMode)` | Всем каналам; SYS не трогается командой «всё при изменении» |
| `uint16_t periodMs() const`, `void cyclePeriod()` | Период режима «постоянно» |
| `static const char* modeName(LogMode, bool periodicOnly)` | «выкл» / «при изменении» / «постоянно» (или «вкл») |
| `void load()` | Из NVS; при несовпадении `VERSION` или длины — остаются значения по умолчанию; неизвестный код режима → по умолчанию канала |
| `void save() const` | В NVS (режимы, период, версия) |

`VERSION` меняется вместе со списком каналов — старые настройки сбрасываются
(`VERSION = 2`: добавлен канал NAV). Клавиши каналов в меню: `1`..`9`, NAV —
`n`, SYS — `s`.

---

## `DebugLogger`

**Файл:** `telemetry/DebugLogger.h` · **Зависит от:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

Вывод состояния в монитор порта по каналам: у каждого своя строка, свой
режим и свои допуски.

| Метод | Описание |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | Раз в `DEBUG_INTERVAL_MS` обходит каналы (молчит на паузе и пока открыто меню) |
| `LogSettings& getSettings()`, `void saveSettings() const` | Для меню консоли |
| `void suspend(bool)` | Меню открыто — молчать; при снятии — `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | Пауза пробелом; при снятии — `refresh()` |
| `void refresh()` | Следующий тик напечатает все включённые каналы |

Логика канала (`updateChannel`):

- `Off` — не печатать;
- `Periodic` — раз в `periodMs()` (SYS — раз в 10 с), значения «как есть»;
- `OnChange` — строка собирается с **допусками** (вложенная `Shown` держит
  старое значение, пока новое не уйдёт дальше допуска: RC/PWM 3 мкс, углы
  0.5°, курс 1°, коррекции 2, высота 0.3 м, ускорение 0.03 g, координаты
  1e−5°) и печатается, только если отличается от прошлой напечатанной.

Вложенные типы: `LineBuffer : Print` (строка до 200 байт для сравнения перед
печатью), `Shown` (значение с гистерезисом).

Форматы строк:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 м  Vz +0.10 м/с  цель 0.0 м
MAG  курс 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 м/с hacc 1.2 м
IMU  gyro +0.1 -0.2 +0.0 °/с  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (худший за 10 с) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` различает `LOST(нет кадров)` и `LOST(failsafe пульта)`; `IMU=` —
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## `DebugConsole`

**Файл:** `telemetry/DebugConsole.h` · **Зависит от:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (опрос шин)

Текстовое меню в мониторе порта. Автомат экранов `Screen::{None, Main, Log}`.

| Метод | Описание |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | с платой — команда `b` и пункт меню 7 |
| `static const char* guessI2cDevice(uint8_t address)` | чип по адресу: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | Однострочная подсказка |
| `void update()` | Обработать все байты из `Serial`; если настройки лога изменены, меню закрыто и **не armed** — сохранить в NVS |

Горячие клавиши (вне меню): `h`/`?` — главное меню; `l` — меню лога; пробел —
пауза лога; `s` — статус датчиков; `i` — калибровка гироскопа; `o` —
калибровка установки IMU; `m` — калибровка компаса; `p` — самопроверка выходов;
`b` — опрос шин I2C (0x08..0x7F — до 0x7F, потому что QMC6309 сидит на 0x7C) с
именами чипов; прочее — подсказка. `\r`/`\n` игнорируются.

Меню лога: `1`..`9` — цикл режима каналов 0..8, `n` — NAV, `s` — SYS, `p` — период, `a` —
всё «при изменении», `x` — всё выкл, `d` — по умолчанию, `0`/`q` — назад, `l`/`h` —
закрыть.

Блокирующие действия (`i`, `o`, `m`, `p`) **запрещены при ARM**. Пока меню
открыто, лог приостановлен (`DebugLogger::suspend`). Ширина пунктов меню
считается в символах UTF-8, а не в байтах (кириллица — 2 байта).

---

## `WebDashboardPage`

**Файл:** `telemetry/WebDashboardPage.h` · **Вид:** namespace

`static const char HTML[] PROGMEM` — вся страница (HTML + CSS + JS) одним
литералом. Всё динамическое строит браузер по JSON `/api/status` (опрос
200 мс): строки каналов, выходов и датчиков создаются по ключам JSON, новый
выход появится без правки страницы. Поле ПИД, которое пользователь начал
править, опрос больше не перезаписывает.

---

## `WebDebugServer`

**Файл:** `telemetry/WebDebugServer.h` · **Зависит от:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| Метод | Описание |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Wi-Fi AP (`persistent(false)` — без записи во флеш), маршруты, задача `web` на ядре 0. `false`, если точка доступа не поднялась |
| `void applyPendingCommands()` | Вызывать из полётного цикла: забрать команды под спинлоком и применить к автопилоту |

Маршруты:

| Маршрут | Ответ |
|---|---|
| `GET /` | Страница дашборда |
| `GET /api/status` | JSON состояния (`buildStatusJson()`), формат — [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`; нет тела → 400 `no data`; нет автопилота → 503; неверный режим → 400 `invalid mode` |
| `POST /api/setpid` | Любые из `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch`; пропущенные остаются текущими |
| прочее | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — почтовый ящик под
`portMUX`. `extractJsonNumber(body, key, fallback)` — минимальный разбор
плоского JSON без ArduinoJson: `"key"`, пробелы, `:`, пробелы, число в
любой записи JSON (знак, дробь, экспонента `1e-7`); нет ключа или числа —
`fallback`.

В JSON поля `attached`/`available` есть **всегда**; данные датчика — только
при `available: true`.

---

## `OledDisplay`

**Файл:** `telemetry/OledDisplay.h` · **Зависит от:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

SSD1306 128×64 (I2C 0x3C) на второй шине I2C; своя задача `oled`
(`Rtos::startTask`: ядро 0 на ESP32, низкий приоритет на STM32), раз в 200 мс.

| Метод | Описание |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` или экран не отвечает на 0x3C → `false`; иначе настройка U8g2 и запуск задачи |

U8g2 передаёт байты через `byteCallback` поверх `II2CBus` (экран не знает про
`Wire1`). C-колбэк не получает контекст, поэтому шина хранится в статической
переменной `busSlot()` — экран на борту один.

Экран:

```
RX ok ARM STAB FL       связь (потеря — инверсная строка) / ARM / режим / закрылки
R  +1.2 P  -0.4         крен / тангаж, °           (IMU --)
Alt +0.3 Vz +0.1 A14    высота / вертикальная скорость / воздушная скорость, если есть трубка (BARO --)
H123 T1000 Y1500        курс / газ / руль направления (H---)
L1500 R1500 E1500       элероны / руль высоты
Loop 500Hz max1100us    частота и худший такт за секунду
```

Короткие имена режимов — `AutopilotNames::modeShort()` (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`);
при потере связи в воздухе — `GLIDE` или `FSRTH`.

---

## `Mavlink` (кодек)

**Файл:** `telemetry/MavlinkCodec.h` · **Вид:** namespace · **Зависит от:** ничего (переносимый)

MAVLink 2 без сгенерированной библиотеки: упаковка полей в порядке MAVLink
(сверено с pymavlink), CRC-16/MCRF4XX + `CRC_EXTRA`, обрезка хвостовых нулей.

| Сущность | Описание |
|---|---|
| `Msg::*` | идентификаторы: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | `CRC_EXTRA` сообщения, −1 — неизвестно |
| `crcAccumulate`, `crcCalculate` | X.25 (как `crc_accumulate()` mavlink) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — поля по порядку |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — кадр v2 с последовательным `seq` |
| `Message` | принятое сообщение: `msgid`, `sysid`, `compid`, payload (дополнен нулями), чтение полей по смещению |
| `Parser` | `bool feed(byte)` → `message()`; v1 и v2, подпись v2 пропускается; `goodCount()`, `badCrcCount()`; сообщения с неизвестным `CRC_EXTRA` молча пропускаются |

## `MavlinkModes`

**Файл:** `telemetry/MavlinkTelemetry.h` · **Вид:** namespace

| Функция | Описание |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | номер режима ArduPlane: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2; failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | обратно, для команд с земли; AUTO, CIRCLE, GUIDED — `false` |
| `isAutonomous(mode)` | флаг `AUTO_ENABLED` в HEARTBEAT |

## `MavlinkTelemetry`

**Файл:** `telemetry/MavlinkTelemetry.h` · **Зависит от:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

Телеметрия по радиомодему для QGroundControl / Mission Planner (борт —
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Используется на STM32
(UART4), где нет Wi-Fi.

| Метод | Описание |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | открыть порт, сообщение «OpenPlane online» |
| `void update()` | из полётного цикла: разбор входящих (≤ 128 байт за такт), сообщения о событиях, не больше 2 кадров за такт |
| `void statusText(severity, text)` | в ленту GCS (очередь на 4 строки, до 50 символов) |
| `isGcsConnected()` | HEARTBEAT от GCS за последние 3 с |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | диагностика |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

Потоки (Гц): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2; HEARTBEAT, SYS_STATUS 1;
HOME_POSITION 0.2. Кадр уходит, только если `availableForWrite()` вмещает его
— иначе ждёт следующего такта (цикл никогда не блокируется).

Входящие: HEARTBEAT GCS; PARAM_REQUEST_LIST / READ / SET (ПИД — сразу в
автопилот, значения 0..100, не сохраняются); SET_MODE и COMMAND_LONG
`DO_SET_MODE` (176) — режим до следующего щелчка тумблера; `COMPONENT_ARM_DISARM`
(400) — **DENIED**; `REQUEST_MESSAGE` (512) — внеочередная отправка потока;
MISSION_REQUEST_LIST — MISSION_COUNT 0 с тем же `mission_type`.
Проверка потока внешним декодером — `tools/check_mavlink.py` (pymavlink).
