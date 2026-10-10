# TELEMETRI – logg, konsol, webbpanel, OLED, svart låda

> 🌐 Den här sidan är en översättning av [det ryska originalet](../../../reference/telemetry.md). Om översättningen och originalet skiljer sig åt gäller originalet. Firmwaren skriver ut sina konsolmeddelanden på ryska, så de citeras oförändrade. Översättningen är gjord av AI och har inte granskats av personer med svenska som modersmål. Rapportera fel till [Damir Lebedev](https://github.com/damir-lebedev) eller i [ärendehanteraren](https://github.com/damir-lebedev/OpenPlaneProject/issues).

[← Referens](README.md)

Telemetrin är helt åtskild från flyglogiken: den läser bara
const-getters i `FlightController`, `Autopilot`, sensorerna och `LoopStats`.
Den enda vägen ”tillbaka” är panelens kommandon, som skickas genom `WebDebugServer`:s
brevlåda och tillämpas av flygslingan.

---

## `LoopStats`

**Fil:** `telemetry/LoopStats.h` · **Typ:** struktur

Flygslingans frekvens och varaktighet.

| Medlem | Beskrivning |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | Publiceras en gång per sekund; läses från andra uppgifter (32-bitarsvärden – inga ”sönderslitna” läsningar) |
| `void record(uint32_t durationUs)` | Anropa varje cykel från `loop()` |
| `uint32_t takePeakUs()` | Den sämsta cykeln sedan föregående anrop (för SYS-raden var 10:e s); anropa den från samma uppgift som `record()` |

`maxUs` är den sämsta bara under den senaste sekunden; ett sällsynt hack syns via
`takePeakUs()`.

---

## `LogSettings`

**Fil:** `telemetry/LogSettings.h` · **Beror på:** `Preferences` (NVS, namnrymden `debuglog`)

### `LogChannel` (enum class)

| Kanal | Prefix | Vad den skriver ut | Standard |
|---|---|---|---|
| `Status` | `STAT` | förbindelse, ARM, läge, klaffar, sensorer | vid ändring |
| `Rc` | `RC` | sändarkanaler | av |
| `Outputs` | `OUT` | utgångar till roderytorna och ESC:n | av |
| `Attitude` | `ATT` | roll, tippning, kurs | av |
| `Autopilot` | `AP` | mål och korrigeringar | av |
| `Altitude` | `ALT` | höjd, vertikal hastighet | av |
| `Heading` | `MAG` | kompasskurs | av |
| `Gps` | `GPS` | satelliter, koordinater | av |
| `Imu` | `IMU` | gyroskop och accelerometer | av |
| `Nav` | `NAV` | hempunkt, kurs, fart, pitotrör, påslagna funktioner | av |
| `System` | `SYS` | slingans frekvens, minne (var 10:e s), bara av/på | på |
| `Count` | – | antalet kanaler | – |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`.

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`.

| Metod | Beskrivning |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | En rad i kanaltabellen |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (cykliskt) |
| `LogSettings()`, `void setDefaults()` | Standardlägena, en period på 1 s |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | En kanals läge |
| `void setMode(uint8_t, LogMode)` | För `periodicOnly`-kanaler blir `OnChange` till `Periodic` |
| `void cycleMode(uint8_t)` | av → vid ändring → kontinuerligt → av (SYS: av ↔ på) |
| `void setAll(LogMode)` | För alla kanaler; SYS rörs inte av kommandot ”alla vid ändring” |
| `uint16_t periodMs() const`, `void cyclePeriod()` | Perioden för läget ”kontinuerligt” |
| `static const char* modeName(LogMode, bool periodicOnly)` | ”av” / ”vid ändring” / ”kontinuerligt” (eller ”på”) |
| `void load()` | Från NVS; om `VERSION` eller längden inte stämmer behålls standardvärdena; en okänd lägeskod → kanalens standard |
| `void save() const` | Till NVS (lägena, perioden, versionen) |

`VERSION` ändras tillsammans med kanallistan – de gamla inställningarna nollställs
(`VERSION = 2`: kanalen NAV lades till). Kanaltangenterna i menyn: `1`..`9`, NAV –
`n`, SYS – `s`.

---

## `DebugLogger`

**Fil:** `telemetry/DebugLogger.h` · **Beror på:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

Utskrift av tillståndet till seriemonitorn per kanal: var och en har sin egen rad, sitt eget
läge och sina egna toleranser.

| Metod | Beskrivning |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | En gång per `DEBUG_INTERVAL_MS` går den igenom kanalerna (tyst under paus och medan menyn är öppen) |
| `LogSettings& getSettings()`, `void saveSettings() const` | För konsolmenyn |
| `void suspend(bool)` | Menyn är öppen – var tyst; vid frisläppning – `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | Paus med mellanslag; vid frisläppning – `refresh()` |
| `void refresh()` | Nästa cykel skriver ut alla påslagna kanaler |

Kanallogiken (`updateChannel`):

- `Off` – skriv inte ut;
- `Periodic` – en gång per `periodMs()` (SYS – en gång per 10 s), värdena ”som de är”;
- `OnChange` – raden sätts ihop med **toleranser** (den inbäddade `Shown` behåller
  det gamla värdet tills det nya går utanför toleransen: RC/PWM 3 µs, vinklar
  0,5°, kurs 1°, korrigeringar 2, höjd 0,3 m, acceleration 0,03 g, koordinater
  1e−5°) och skrivs ut bara om den skiljer sig från den senast utskrivna.

De inbäddade typerna: `LineBuffer : Print` (en rad på upp till 200 byte för jämförelse före
utskrift), `Shown` (ett värde med hysteres).

Radformaten:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  target 0.0 m
MAG  heading 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (worst in 10 s) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` skiljer på `LOST(no frames)` och `LOST(transmitter failsafe)`; `IMU=` är
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## `DebugConsole`

**Fil:** `telemetry/DebugConsole.h` · **Beror på:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (bussavsökning)

En textmeny i seriemonitorn. En tillståndsmaskin av skärmar `Screen::{None, Main, Log}`.

| Metod | Beskrivning |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | med ett kort – kommandot `b` och menypunkt 7 |
| `static const char* guessI2cDevice(uint8_t address)` | kretsen efter adress: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | En ledtråd på en rad |
| `void update()` | Bearbeta alla byte från `Serial`; om logginställningarna har ändrats, menyn är stängd och planet **inte är armerat** – spara i NVS |

Snabbtangenter (utanför menyn): `h`/`?` – huvudmenyn; `l` – loggmenyn; mellanslag –
pausa loggen; `s` – sensorstatus; `i` – gyroskopkalibrering; `o` –
kalibrering av IMU-monteringen; `m` – kompasskalibrering; `p` – självtest av utgångar;
`b` – avsökning av I2C-bussarna (0x08..0x7F – upp till 0x7F, eftersom QMC6309 sitter på 0x7C) med
kretsnamn; allt annat – en ledtråd. `\r`/`\n` ignoreras.

Loggmenyn: `1`..`9` – växla läge för kanalerna 0..8, `n` – NAV, `s` – SYS, `p` – period, `a` –
allt ”vid ändring”, `x` – allt av, `d` – standardvärden, `0`/`q` – tillbaka, `l`/`h` –
stäng.

De blockerande åtgärderna (`i`, `o`, `m`, `p`) är **förbjudna under ARM**. Medan menyn
är öppen är loggen pausad (`DebugLogger::suspend`). Menypunkternas bredd
räknas i UTF-8-tecken snarare än byte (kyrilliska tecken tar 2 byte).

---

## `WebDashboardPage`

**Fil:** `telemetry/WebDashboardPage.h` · **Typ:** namnrymd

`static const char HTML[] PROGMEM` – hela sidan (HTML + CSS + JS) som en enda
literal. Allt dynamiskt byggs av webbläsaren från JSON i `/api/status` (som frågas
var 200:e ms): raderna för kanaler, utgångar och sensorer skapas från JSON-nycklarna, så en ny
utgång dyker upp utan att sidan redigeras. Ett PID-fält som användaren har börjat
redigera skrivs inte längre över av förfrågningarna.

---

## `WebDebugServer`

**Fil:** `telemetry/WebDebugServer.h` · **Beror på:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| Metod | Beskrivning |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | En Wi-Fi-AP (`persistent(false)` – ingenting skrivs till flash), rutterna, uppgiften `web` på kärna 0. `false` om åtkomstpunkten inte startade |
| `void applyPendingCommands()` | Anropa den från flygslingan: ta kommandona under ett spinlock och tillämpa dem på autopiloten |

Rutterna:

| Rutt | Svar |
|---|---|
| `GET /` | Panelens sida |
| `GET /api/status` | Tillståndets JSON (`buildStatusJson()`), formatet finns i [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`; ingen kropp → 400 `no data`; ingen autopilot → 503; ett felaktigt läge → 400 `invalid mode` |
| `POST /api/setpid` | Valfria av `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch`; de utelämnade förblir som de är |
| allt annat | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` – en brevlåda under ett
`portMUX`. `extractJsonNumber(body, key, fallback)` – en minimal tolk av
platt JSON utan ArduinoJson: `"key"`, mellanslag, `:`, mellanslag, ett tal i
valfri JSON-notation (ett tecken, ett bråk, en exponent `1e-7`); om nyckeln eller talet saknas –
`fallback`.

I JSON finns fälten `attached`/`available` **alltid**; sensordata bara
när `available: true`.

---

## `OledDisplay`

**Fil:** `telemetry/OledDisplay.h` · **Beror på:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

En SSD1306 128×64 (I2C 0x3C) på den andra I2C-bussen; en egen uppgift `oled`
(`Rtos::startTask`: kärna 0 på ESP32, en låg prioritet på STM32), var 200:e ms.

| Metod | Beskrivning |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` eller skärmen svarar inte på 0x3C → `false`; annars ställer den in U8g2 och startar uppgiften |

U8g2 skickar byte genom en `byteCallback` ovanpå `II2CBus` (skärmen vet ingenting om
`Wire1`). En C-callback får ingen kontext, så bussen hålls i en statisk
variabel `busSlot()` – det finns bara en skärm ombord.

Skärmen:

```
RX ok ARM STAB FL       förbindelse (förlust – inverterad rad) / ARM / läge / klaffar
R  +1.2 P  -0.4         roll / tippning, °           (IMU --)
Alt +0.3 Vz +0.1 A14    höjd / vertikal hastighet / lufthastighet, om pitotrör finns (BARO --)
H123 T1000 Y1500        kurs / gas / sidroder (H---)
L1500 R1500 E1500       skevroder / höjdroder
Loop 500Hz max1100us    frekvens och den sämsta cykeln under sekunden
```

De korta lägesnamnen är `AutopilotNames::modeShort()` (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`);
vid förlorad förbindelse i luften – `GLIDE` eller `FSRTH`.

---

## `BlackBox`

**Fil:** `telemetry/BlackBox.h` · **Beror på:** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

Inspelning av flygningen till flash (ESP32-S3) eller ett SD-kort (STM32H743). Vad som ska hämtas, när och hur – se [BLACKBOX.md](../BLACKBOX.md).

| Metod | Beskrivning |
|---|---|
| `bool begin(bool startTask = true)` | Läser mediet (`BlackBoxStorage::begin()`), allokerar kön (PSRAM på ESP32, `malloc` på STM32), kontrollerar det raderade utrymmet (upp till 0,3 s), startar uppgiften `bbox` (`Rtos::startTask`). Om det inte finns plats för inspelning (ingen partition, inget kort eller ingen fil) – `false`, den svarta lådan är avstängd |
| `void update(uint32_t workUs)` | Från `loop()` efter varje cykel: händelser, start/stopp, ögonblicksbilder i kön, väcker skrivaruppgiften |
| `void writerStep()` | Ett steg i skrivaruppgiften: en eller två sidor till flash, eller en radering på marken |
| `requestManualStart()` / `requestManualStop()` | Inspelning för hand (konsol `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (skriver ut kön innan END registreras) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | För konsolen |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [baud]` – för `tools/blackbox.py` (över USB CDC påverkar hastigheten ingenting) |

De plattformsspecifika delarna: återställningsorsaken – `readResetCause()`; batterispänningen och strömmen –
ADC:n (`analogReadMilliVolts` på S3, en 12-bitars `analogRead` på STM32); mediefelen
(`BlackBoxStorage::writeErrors`/`eraseErrors`) hamnar i loggen en gång per sekund
som en händelse ”medium: skrivfel …” och stör inte flygningen.

## `BlackBoxStorage`

**Fil:** `telemetry/BlackBoxStorage.h` · **Beror på:** `IFlashRegion`

En ring av sektorer på 4 KB: huvudet och listan över flygningar kommer från sektorhuvudena vid `begin()` (det första varvet läser huvudet i varje sektor och kommer ihåg de äkta, det andra bara dem: ett tomt område läses en gång); `openFlight()`/`append()`/`flush()`/`closeFlight()` – skrivning sida för sida (en CRC-8 på varje post); `eraseStep(target, protect, allowErase)` – ett steg av kontroll/radering framför huvudet: skräp – alltid, flygningar – i sin helhet och bara så länge mindre än `target` är ledigt; `protect` rörs aldrig.

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing` är en bytekö av poster mellan uppgifter/kärnor under `Rtos::CriticalSection`; när den svämmar över kastar den de äldsta. `BlackBoxFormat` – sektorhuvudet, posttyperna och strukturerna, schemasträngarna (storleken kontrolleras med `static_assert`), CRC-8 och CRC-32.

---

## `Mavlink` (kodek)

**Fil:** `telemetry/MavlinkCodec.h` · **Typ:** namnrymd · **Beror på:** ingenting (portabel)

MAVLink 2 utan det genererade biblioteket: packning av fält i MAVLink-ordning
(kontrollerad mot pymavlink), CRC-16/MCRF4XX + `CRC_EXTRA`, trimning av avslutande nollor.

| Entitet | Beskrivning |
|---|---|
| `Msg::*` | identifierare: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | ett meddelandes `CRC_EXTRA`, −1 – okänt |
| `crcAccumulate`, `crcCalculate` | X.25 (som mavlinks `crc_accumulate()`) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` – fälten i ordning |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` – en v2-ram med löpande `seq` |
| `Message` | ett mottaget meddelande: `msgid`, `sysid`, `compid`, nyttolasten (utfylld med nollor), läsning av fält efter förskjutning |
| `Parser` | `bool feed(byte)` → `message()`; v1 och v2, v2-signaturen hoppas över; `goodCount()`, `badCrcCount()`; meddelanden med okänd `CRC_EXTRA` hoppas tyst över |

## `MavlinkModes`

**Fil:** `telemetry/MavlinkTelemetry.h` · **Typ:** namnrymd

| Funktion | Beskrivning |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | ArduPlane-lägets nummer: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2; failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | det omvända, för kommandon från marken; AUTO, CIRCLE, GUIDED – `false` |
| `isAutonomous(mode)` | flaggan `AUTO_ENABLED` i HEARTBEAT |

## `MavlinkTelemetry`

**Fil:** `telemetry/MavlinkTelemetry.h` · **Beror på:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

Telemetri via ett radiomodem för QGroundControl / Mission Planner (farkosten är
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Används på STM32
(UART4), som inte har Wi-Fi.

| Metod | Beskrivning |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | öppna porten, meddelandet ”OpenPlane online” |
| `void update()` | från flygslingan: tolka inkommande data (≤ 128 byte per cykel), händelsemeddelanden, högst 2 ramar per cykel |
| `void statusText(severity, text)` | till GCS:ens flöde (en kö med 4 rader, upp till 50 tecken) |
| `isGcsConnected()` | en HEARTBEAT från GCS:en inom de senaste 3 s |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | diagnostik |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

Strömmarna (Hz): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2; HEARTBEAT, SYS_STATUS 1;
HOME_POSITION 0,2. En ram skickas bara om `availableForWrite()` har plats för den
– annars väntar den till nästa cykel (slingan blockeras aldrig).

Inkommande: GCS:ens HEARTBEAT; PARAM_REQUEST_LIST / READ / SET (PID – direkt in i
autopiloten, värden 0..100, sparas inte); SET_MODE och COMMAND_LONG
`DO_SET_MODE` (176) – läget fram till nästa omslag av brytaren; `COMPONENT_ARM_DISARM`
(400) – **DENIED**; `REQUEST_MESSAGE` (512) – en extra sändning av en ström;
MISSION_REQUEST_LIST – MISSION_COUNT 0 med samma `mission_type`.
Kontroll av strömmen med en extern avkodare – `tools/check_mavlink.py` (pymavlink).
