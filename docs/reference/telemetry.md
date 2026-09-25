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

`VERSION` меняется вместе со списком каналов — старые настройки сбрасываются.

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

**Файл:** `telemetry/DebugConsole.h` · **Зависит от:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`

Текстовое меню в мониторе порта. Автомат экранов `Screen::{None, Main, Log}`.

| Метод | Описание |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&)` | |
| `void printHint() const` | Однострочная подсказка |
| `void update()` | Обработать все байты из `Serial`; если настройки лога изменены, меню закрыто и **не armed** — сохранить в NVS |

Горячие клавиши (вне меню): `h`/`?` — главное меню; `l` — меню лога; пробел —
пауза лога; `s` — статус датчиков; `i` — калибровка гироскопа; `o` —
калибровка установки IMU; `m` — калибровка компаса; `p` — самопроверка выходов;
прочее — подсказка. `\r`/`\n` игнорируются.

Меню лога: `1`..`9` — цикл режима каналов 0..8, `s` — SYS, `p` — период, `a` —
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

SSD1306 128×64 (I2C 0x3C) на второй шине I2C; своя задача `oled` на ядре 0,
раз в 200 мс.

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
Alt +0.3 Vz +0.1        высота / вертикальная скорость (BARO --)
H123 T1000 Y1500        курс / газ / руль направления (H---)
L1500 R1500 E1500       элероны / руль высоты
Loop 500Hz max1100us    частота и худший такт за секунду
```

Короткие имена режимов: `MAN`, `STAB`, `TKOFF`, `ALT`, при потере связи в
воздухе — `GLIDE`.
