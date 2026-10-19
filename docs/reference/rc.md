# RC — приём команд пульта

[← Справочник](README.md)

Слой RC превращает байты UART в значения каналов и признак «связи нет». Он
ничего не знает про самолёт, ARM, failsafe-поведение и сервоприводы. Протокол
приёмника — iBUS, SBUS или CRSF — задаёт `Config::RX_PROTOCOL`; новый протокол —
ещё один разборщик кадров рядом с тремя имеющимися.

---

## `RcChannelState`

**Файл:** `rc/RcChannelState.h` · **Зависит от:** `Config`, `Channels`

Снимок 10 каналов приёмника (мкс), без логики управления.

| Метод | Описание |
|---|---|
| `RcChannelState()` | Вызывает `reset()` |
| `void reset()` | Безопасные значения: все каналы `PWM_CENTER`, газ — `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | Значение канала; индекс вне диапазона → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | Записать канал; индекс вне диапазона игнорируется |
| `const uint16_t* data() const` | Весь массив (для отладки) |

---

## `RcInput`

**Файл:** `rc/RcInput.h` · **Вид:** набор статических функций · **Зависит от:** `Config`

Общие преобразования RC-сигналов.

| Метод | Описание |
|---|---|
| `static uint16_t clamp(uint16_t value)` | Ограничение `PWM_MIN..PWM_MAX` |
| `static uint16_t reversed(uint16_t value)` | Канал наоборот: `3000 − clamp(value)` (1000 ↔ 2000) — для `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Линейно: 1000 → `−max`, 1500 → 0, 2000 → `+max` (вход сначала ограничивается); `reverse` меняет знак. Результат ограничен ±`max` |

Пример: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**Файл:** `rc/RcReceiver.h` · **Зависит от:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

Приёмник пульта: открывает UART в формате протокола, отдаёт байты разборщику и
делает общее для всех протоколов — первые `RC_CHANNELS` = 10 каналов, реверс
каналов, таймаут кадров, failsafe, счётчики.

| Протокол (`Config::RX_PROTOCOL`) | Приёмники | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper и большинство других | 100000, 8E2, инвертированный — инверсию делает сам UART |
| `CRSF` | ExpressLRS, TBS Crossfire и Tracer | 420000, 8N1 |

| Метод | Описание |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | Порт не открывается в конструкторе; в тестах протокол — любой |
| `void begin()` | Открыть UART в формате протокола (`IUartPort::begin(baud, format)`), отсчёт таймаута от «сейчас» |
| `void update()` | Вычитать всё, что накопилось в UART; вызывать каждый такт |
| `const RcChannelState& getState() const` | Последние принятые каналы |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Ещё не было ни одного кадра **или** последний старше `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | Приёмник сообщил о потере связи: флаг failsafe в кадре SBUS, качество связи 0 % у CRSF или газ < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` последнего корректного кадра |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Счётчики принятых кадров и отброшенных (CRC, конец кадра) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | Протокол; имя — `"iBUS"`, `"SBUS"`, `"CRSF"` |
| `int16_t getLinkQuality() const` | Качество связи по радио, % — только CRSF, иначе −1 |
| `bool isPortReady() const` | UART открылся в нужном формате; `false` — порт так не умеет (SBUS без инверсии), связи не будет |

Свободная функция `rxProtocolName(Config::RxProtocol)` — имя протокола; его же
печатает паспорт прошивки (`@id`, поле `rx`).

Инварианты:

- До первого корректного кадра `isSignalLost() == true` — значения по
  умолчанию (все 1500) не принимаются за команды пульта.
- Признак failsafe пересчитывается на **каждом** кадре каналов — связь
  восстанавливается первым же кадром с нормальным газом (у CRSF — ещё и
  статистикой с качеством связи больше 0).
- Каналы с `Config::CHANNEL_REVERSED` сохраняются развёрнутыми
  (`RcInput::reversed`); failsafe определяется по **сырому** газу до этого, а
  реверс CH3 запрещён `static_assert`.

---

## `IBusParser`

**Файл:** `rc/IBusParser.h` · **Зависит от:** `Config`

Побайтовый разбор FlySky iBUS: `bool feed(uint8_t)` — `true`, когда собран кадр с
верной CRC; каналы — `values()`. **Кадр** (32 байта): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(первые 30 байт)`; значение канала — **младшие 12 бит** (в
старших FS-iA6B передаёт служебные данные, например в failsafe `0x2384` → 900 мкс).
Ждёт `0x20`, следующий байт должен быть `0x40`, иначе поиск заново (второй `0x20`
сам может начинать кадр). Флага failsafe в iBUS нет: FS-iA6B сообщает о потере
связи газом ниже нормы.

---

## `SbusParser`

**Файл:** `rc/SbusParser.h` · **Зависит от:** `Packed11`

**Кадр** (25 байт): `0x0F | 16 каналов по 11 бит (22 байта) | флаги | конец`;
флаги: бит 2 — потерян один кадр по радио (это ещё не потеря связи), бит 3 —
failsafe; конец — `0x00` у SBUS, `0x04`/`0x14`/`0x24`/`0x34` у SBUS2. Байт `0x0F`
бывает и в данных, поэтому кадр принимается, только когда на месте и начало, и
конец; иначе разбор продолжается со следующего `0x0F` внутри принятого — поток
синхронизируется за кадр-два.

---

## `CrsfParser`

**Файл:** `rc/CrsfParser.h` · **Зависит от:** `Packed11`

**Кадр:** `0xC8 | длина | тип | данные | CRC8`; длина — тип + данные + CRC, кадр
не длиннее 64 байт; CRC8 с полиномом `0xD5` (DVB-S2) — по типу и данным
(`static uint8_t crc8(...)`). Нужны два типа: `0x16` — каналы (16 по 11 бит, как в
SBUS) и `0x14` — статистика связи (третий байт — качество связи, %). ExpressLRS по
умолчанию при потере связи просто перестаёт слать каналы — это таймаут кадров; в
режиме «последние значения» каналы идут, но качество связи 0 — это `failsafe()`.

---

## `Packed11`

**Файл:** `rc/Packed11.h` · **Вид:** пространство имён

16 каналов по 11 бит в 22 байтах, младшими битами вперёд — так пакуют и SBUS,
и CRSF. `toMicros(raw)`: 172…1811 → 988…2012 мкс, 992 → 1500 (`5/8 · значение + 880`,
как в Betaflight и INAV); `unpack(data, out)` — 22 байта → 16 значений в мкс.
