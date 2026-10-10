# RC – mottagning av sändarens kommandon

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/rc.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

RC-lagret omvandlar UART-byte till kanalvärden och en flagga för ”ingen förbindelse”. Det
vet ingenting om flygplanet, ARM, failsafe-beteende eller servon – att byta
protokoll (S-Bus, PPM) påverkar bara det här lagret.

---

## `RcChannelState`

**Fil:** `rc/RcChannelState.h` · **Beror på:** `Config`, `Channels`

En ögonblicksbild av mottagarens 10 kanaler (µs), utan styrlogik.

| Metod | Beskrivning |
|---|---|
| `RcChannelState()` | Anropar `reset()` |
| `void reset()` | Säkra värden: alla kanaler `PWM_CENTER`, gas – `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | Kanalvärdet; ett index utanför intervallet → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | Skriv en kanal; ett index utanför intervallet ignoreras |
| `const uint16_t* data() const` | Hela arrayen (för felsökning) |

---

## `RcInput`

**Fil:** `rc/RcInput.h` · **Typ:** en uppsättning statiska funktioner · **Beror på:** `Config`

Vanliga omvandlingar av RC-signaler.

| Metod | Beskrivning |
|---|---|
| `static uint16_t clamp(uint16_t value)` | Begränsar till `PWM_MIN..PWM_MAX` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linjär: 1000 → `−max`, 1500 → 0, 2000 → `+max` (indata begränsas först); `reverse` vänder tecknet. Resultatet begränsas till ±`max` |

Exempel: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**Fil:** `rc/IBusReceiver.h` · **Beror på:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

En tolk för FlySkys iBUS-protokoll som arbetar byte för byte.

**Ramformat** (32 byte): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(de första 30 byten)`. De första `IBUS_CHANNELS` = 10 kanalerna
tas; kanalvärdet är de **låga 12 bitarna** (i de höga bitarna bär FS-iA6B
tjänstedata, till exempel i failsafe `0x2384` → 900 µs).

| Metod | Beskrivning |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | Porten öppnas inte i konstruktorn |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, tidsgränsen räknas från ”nu” |
| `void update()` | Läs ut allt som ackumulerats i UART:en; anropa den varje cykel |
| `const RcChannelState& getState() const` | De senast mottagna kanalerna |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Det har ännu inte kommit en enda ram **eller** den senaste är äldre än `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | I den senaste ramen är gasen < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` för den senaste korrekta ramen |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Räknare för korrekta ramar och CRC-fel |

Tolkningens tillståndsmaskin (`processByte`): väntar på `0x20`; nästa byte måste
vara `0x40`, annars börjar sökningen om; sedan samlar den in 32 byte och anropar
`processFrame()`. En ram med fel CRC kasseras helt (kanalerna
ändras inte, `badFrames++`).

Invarianter:

- Fram till den första korrekta ramen är `isSignalLost() == true` – standardvärdena
  (alla 1500) tas inte för sändarkommandon.
- Failsafe-flaggan räknas om vid **varje** korrekt ram – förbindelsen
  återställs av allra första ramen med normal gas.
