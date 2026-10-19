# RC – mottagning av sändarens kommandon

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/rc.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

RC-lagret omvandlar UART-byte till kanalvärden och en flagga för ”ingen förbindelse”. Det
vet ingenting om flygplanet, ARM, failsafe-beteende eller servon. Mottagarens
protokoll – iBUS, SBUS eller CRSF – bestäms av `Config::RX_PROTOCOL`; ett nytt
protokoll är bara ännu en ramtolk bredvid de tre som finns.

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
| `static uint16_t reversed(uint16_t value)` | Kanal åt andra hållet: `3000 − clamp(value)` (1000 ↔ 2000) – för `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linjär: 1000 → `−max`, 1500 → 0, 2000 → `+max` (indata begränsas först); `reverse` vänder tecknet. Resultatet begränsas till ±`max` |

Exempel: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**Fil:** `rc/RcReceiver.h` · **Beror på:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

Radiomottagaren: öppnar UART i protokollets format, lämnar byten till tolken och
gör det som är gemensamt för alla protokoll – de första `RC_CHANNELS` = 10
kanalerna, kanalreversering, ramtimeout, failsafe, räknare.

| Protokoll (`Config::RX_PROTOCOL`) | Mottagare | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper och de flesta andra | 100000, 8E2, inverterad – UART:en inverterar själv |
| `CRSF` | ExpressLRS, TBS Crossfire och Tracer | 420000, 8N1 |

| Metod | Beskrivning |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | Porten öppnas inte i konstruktorn; i tester går vilket protokoll som helst |
| `void begin()` | Öppna UART i protokollets format (`IUartPort::begin(baud, format)`), räkna timeouten från ”nu” |
| `void update()` | Läs allt som samlats i UART; anropa varje cykel |
| `const RcChannelState& getState() const` | De senast mottagna kanalerna |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Ingen ram har kommit än **eller** den senaste är äldre än `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | Mottagaren rapporterade förlorad förbindelse: failsafe-flaggan i en SBUS-ram, länkkvalitet 0 % i CRSF eller gas < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` för den senaste giltiga ramen |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Räknare för mottagna och kasserade ramar (CRC, ramslut) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | Protokollet; namnet är `"iBUS"`, `"SBUS"` eller `"CRSF"` |
| `int16_t getLinkQuality() const` | Radiolänkens kvalitet, % – bara CRSF, annars −1 |
| `bool isPortReady() const` | UART öppnades i rätt format; `false` – porten klarar det inte (SBUS utan invertering), det blir ingen förbindelse |

Den fria funktionen `rxProtocolName(Config::RxProtocol)` ger protokollets namn;
samma namn skriver firmwarens pass (`@id`, fältet `rx`).

Invarianter:

- Fram till den första giltiga ramen gäller `isSignalLost() == true` – standardvärdena
  (alla 1500) tas inte som kommandon från sändaren.
- Failsafe-flaggan räknas om vid **varje** kanalram – förbindelsen kommer tillbaka
  med den första ramen med normal gas (i CRSF även med en statistikram med
  länkkvalitet över 0).
- Kanaler med `Config::CHANNEL_REVERSED` sparas reverserade (`RcInput::reversed`);
  failsafe avgörs innan dess från den **råa** gasen, och reversering av CH3 förbjuds
  av `static_assert`.

---

## `IBusParser`

**Fil:** `rc/IBusParser.h` · **Beror på:** `Config`

Byte för byte-tolk för FlySky iBUS: `bool feed(uint8_t)` returnerar `true` när en ram
med korrekt CRC är komplett; kanalerna finns i `values()`. **Ram** (32 byte): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(första 30 byte)`; kanalvärdet är de **låga 12 bitarna** (i de
höga skickar FS-iA6B tjänstedata, t.ex. i failsafe `0x2384` → 900 µs). Den väntar
på `0x20`; nästa byte måste vara `0x40`, annars börjar sökningen om (ett andra
`0x20` kan själv börja en ram). iBUS har ingen failsafe-flagga: FS-iA6B rapporterar
förlorad förbindelse med gas under det normala.

---

## `SbusParser`

**Fil:** `rc/SbusParser.h` · **Beror på:** `Packed11`

**Ram** (25 byte): `0x0F | 16 kanaler à 11 bitar (22 byte) | flaggor | slut`;
flaggor: bit 2 – en ram förlorad över radion (ännu inte förlorad förbindelse),
bit 3 – failsafe; slut – `0x00` för SBUS, `0x04`/`0x14`/`0x24`/`0x34` för SBUS2.
Byten `0x0F` förekommer även i data, så en ram godtas bara när både början och
slutet är på plats; annars fortsätter tolkningen från nästa `0x0F` i det mottagna
– flödet synkroniseras inom en eller två ramar.

---

## `CrsfParser`

**Fil:** `rc/CrsfParser.h` · **Beror på:** `Packed11`

**Ram:** `0xC8 | längd | typ | data | CRC8`; längd = typ + data + CRC, en ram är
högst 64 byte; CRC8 med polynomet `0xD5` (DVB-S2) över typ och data
(`static uint8_t crc8(...)`). Två typer behövs: `0x16` – kanaler (16 à 11 bitar, som
i SBUS) och `0x14` – länkstatistik (tredje byten är länkkvaliteten, %). Vid
förlorad förbindelse slutar ExpressLRS som standard helt enkelt skicka kanaler –
det är ramtimeouten; i läget ”senaste position” fortsätter kanalerna komma men
länkkvaliteten är 0 – det är `failsafe()`.

---

## `Packed11`

**Fil:** `rc/Packed11.h` · **Typ:** namnrymd

16 kanaler à 11 bitar i 22 byte, minst signifikanta bitar först – så packar både SBUS
och CRSF dem. `toMicros(raw)`: 172…1811 → 988…2012 µs, 992 → 1500 (`5/8 · värde + 880`,
som i Betaflight och INAV); `unpack(data, out)` gör 22 byte till 16 värden i µs.
