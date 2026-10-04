# RC — Empfang der Senderbefehle

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/rc.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert.

[← Referenz](README.md)

Die RC-Schicht wandelt UART-Bytes in Kanalwerte und in ein Kennzeichen „keine
Verbindung“ um. Sie weiß nichts über das Flugzeug, ARM, das Failsafe-Verhalten
und die Servos — der Austausch des Protokolls (S-Bus, PPM) betrifft nur diese
Schicht.

---

## `RcChannelState`

**Datei:** `rc/RcChannelState.h` · **Abhängig von:** `Config`, `Channels`

Eine Momentaufnahme der 10 Kanäle des Empfängers (µs), ohne Steuerlogik.

| Methode | Beschreibung |
|---|---|
| `RcChannelState()` | Ruft `reset()` auf |
| `void reset()` | Sichere Werte: alle Kanäle `PWM_CENTER`, das Gas — `PWM_MIN` |
| `uint16_t get(uint8_t index) const` | Der Kanalwert; ein Index außerhalb des Bereichs → `PWM_CENTER` |
| `void set(uint8_t index, uint16_t value)` | Einen Kanal schreiben; ein Index außerhalb des Bereichs wird ignoriert |
| `const uint16_t* data() const` | Das ganze Array (zum Debuggen) |

---

## `RcInput`

**Datei:** `rc/RcInput.h` · **Art:** eine Menge statischer Funktionen · **Abhängig von:** `Config`

Gemeinsame Umrechnungen der RC-Signale.

| Methode | Beschreibung |
|---|---|
| `static uint16_t clamp(uint16_t value)` | Begrenzung auf `PWM_MIN..PWM_MAX` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linear: 1000 → `−max`, 1500 → 0, 2000 → `+max` (der Eingang wird zuerst begrenzt); `reverse` kehrt das Vorzeichen um. Das Ergebnis ist auf ±`max` begrenzt |

Beispiel: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `IBusReceiver`

**Datei:** `rc/IBusReceiver.h` · **Abhängig von:** `IUartPort`, `RcChannelState`, `Config`, `Channels`

Ein Byte-für-Byte-Parser des FlySky-iBUS-Protokolls.

**Frame-Format** (32 Bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(first 30 bytes)`. Genommen werden die ersten
`IBUS_CHANNELS` = 10 Kanäle; der Kanalwert sind die **unteren 12 Bit** (in den
oberen überträgt der FS-iA6B Servicedaten, zum Beispiel im Failsafe
`0x2384` → 900 µs).

| Methode | Beschreibung |
|---|---|
| `explicit IBusReceiver(IUartPort& serial)` | Der Port wird im Konstruktor nicht geöffnet |
| `void begin()` | `serial.begin(IBUS_BAUDRATE)`, die Zeitüberwachung zählt ab „jetzt“ |
| `void update()` | Alles auslesen, was sich im UART angesammelt hat; in jedem Zyklus aufrufen |
| `const RcChannelState& getState() const` | Die zuletzt empfangenen Kanäle |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Es gab noch keinen einzigen Frame **oder** der letzte ist älter als `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | Im letzten Frame das Gas < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` des letzten korrekten Frames |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Zähler korrekter Frames und von CRC-Fehlern |

Der Parse-Automat (`processByte`): wartet auf `0x20`; das nächste Byte muss
`0x40` sein, sonst beginnt die Suche von vorn; dann sammelt er 32 Bytes und
ruft `processFrame()` auf. Ein Frame mit falschem CRC wird vollständig
verworfen (die Kanäle ändern sich nicht, `badFrames++`).

Invarianten:

- Bis zum ersten korrekten Frame ist `isSignalLost() == true` — die
  Standardwerte (alle 1500) werden nicht für Senderbefehle gehalten.
- Das Failsafe-Kennzeichen wird bei **jedem** korrekten Frame neu berechnet —
  die Verbindung ist schon mit dem ersten Frame mit normalem Gas wieder da.
