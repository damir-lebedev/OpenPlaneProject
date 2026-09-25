# RC — приём команд пульта

[← Справочник](README.md)

Слой RC превращает байты UART в значения каналов и признак «связи нет». Он
ничего не знает про самолёт, ARM, failsafe-поведение и сервоприводы — замена
протокола (S-Bus, PPM) затрагивает только этот слой.

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
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Линейно: 1000 → `−max`, 1500 → 0, 2000 → `+max` (вход сначала ограничивается); `reverse` меняет знак. Результат ограничен ±`max` |

Пример: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**Файл:** `rc/IBusReceiver.h` · **Зависит от:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

Побайтовый парсер протокола FlySky iBUS.

**Формат кадра** (32 байта): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(первые 30 байт)`. Берутся первые `IBUS_CHANNELS` = 10 каналов;
значение канала — **младшие 12 бит** (в старших FS-iA6B передаёт служебные
данные, например в failsafe `0x2384` → 900 мкс).

| Метод | Описание |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | Порт не открывается в конструкторе |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, отсчёт таймаута от «сейчас» |
| `void update()` | Вычитать всё, что накопилось в UART; вызывать каждый такт |
| `const RcChannelState& getState() const` | Последние принятые каналы |
| `bool isSignalLost() const` | `isFrameTimeout() || isFailsafeReported()` |
| `bool isFrameTimeout() const` | Ещё не было ни одного кадра **или** последний старше `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | В последнем кадре газ < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` последнего корректного кадра |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Счётчики корректных кадров и ошибок CRC |

Автомат разбора (`processByte`): ждёт `0x20`; следующий байт должен быть `0x40`,
иначе поиск начинается заново; затем набирает 32 байта и вызывает
`processFrame()`. Кадр с неверной CRC отбрасывается целиком (каналы не
меняются, `badFrames++`).

Инварианты:

- До первого корректного кадра `isSignalLost() == true` — значения по
  умолчанию (все 1500) не принимаются за команды пульта.
- Признак failsafe пересчитывается на **каждом** корректном кадре — связь
  восстанавливается первым же кадром с нормальным газом.
