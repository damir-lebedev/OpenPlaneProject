# RC — Empfang der Senderbefehle

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/rc.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

[← Referenz](README.md)

Die RC-Schicht wandelt UART-Bytes in Kanalwerte und in ein Kennzeichen „keine
Verbindung“ um. Sie weiß nichts über das Flugzeug, ARM, das Failsafe-Verhalten
und die Servos. Das Empfängerprotokoll — iBUS, SBUS oder CRSF — legt
`Config::RX_PROTOCOL` fest; ein neues Protokoll ist nur ein weiterer
Frame-Parser neben den drei vorhandenen.

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
| `static uint16_t reversed(uint16_t value)` | Kanal umgekehrt: `3000 − clamp(value)` (1000 ↔ 2000) – für `Config::CHANNEL_REVERSED` |
| `static int16_t centered(uint16_t input, int16_t maxDeflection, bool reverse = false)` | Linear: 1000 → `−max`, 1500 → 0, 2000 → `+max` (der Eingang wird zuerst begrenzt); `reverse` kehrt das Vorzeichen um. Das Ergebnis ist auf ±`max` begrenzt |

Beispiel: `centered(1750, 500) == 250`, `centered(1750, 500, true) == -250`.

---

## `RcReceiver`

**Datei:** `rc/RcReceiver.h` · **Abhängig von:** `IUartPort`, `IBusParser`, `SbusParser`, `CrsfParser`, `RcChannelState`, `RcInput`, `Config`, `Channels`

Der Fernsteuerempfänger: öffnet den UART im Format des Protokolls, gibt die Bytes
an den Parser weiter und erledigt, was allen Protokollen gemeinsam ist — die
ersten `RC_CHANNELS` = 10 Kanäle, Kanalumkehr, Frame-Timeout, Failsafe, Zähler.

| Protokoll (`Config::RX_PROTOCOL`) | Empfänger | UART |
|---|---|---|
| `IBUS` | FlySky FS-iA6B, FS-iA10B | 115200, 8N1 |
| `SBUS` | FrSky, Futaba, RadioMaster, Jumper und die meisten anderen | 100000, 8E2, invertiert — der UART invertiert selbst |
| `CRSF` | ExpressLRS, TBS Crossfire und Tracer | 420000, 8N1 |

| Methode | Beschreibung |
|---|---|
| `explicit RcReceiver(IUartPort& serial, Config::RxProtocol protocol = Config::RX_PROTOCOL)` | Der Port wird im Konstruktor nicht geöffnet; in Tests ist jedes Protokoll möglich |
| `void begin()` | UART im Format des Protokolls öffnen (`IUartPort::begin(baud, format)`), Timeout ab „jetzt“ zählen |
| `void update()` | Alles lesen, was sich im UART angesammelt hat; in jedem Takt aufrufen |
| `const RcChannelState& getState() const` | Die zuletzt empfangenen Kanäle |
| `bool isSignalLost() const` | `isFrameTimeout() \|\| isFailsafeReported()` |
| `bool isFrameTimeout() const` | Noch kein einziger Frame **oder** der letzte ist älter als `RX_TIMEOUT_US` |
| `bool isFailsafeReported() const` | Der Empfänger meldet Verbindungsverlust: Failsafe-Flag im SBUS-Frame, Verbindungsqualität 0 % bei CRSF oder Gas < `RX_FAILSAFE_THROTTLE_US` |
| `uint32_t getLastFrameTime() const` | `micros()` des letzten gültigen Frames |
| `uint32_t getGoodFrameCount() const` / `getBadFrameCount() const` | Zähler der angenommenen und verworfenen Frames (CRC, Frame-Ende) |
| `Config::RxProtocol getProtocol() const` / `const char* getProtocolName() const` | Das Protokoll; der Name ist `"iBUS"`, `"SBUS"` oder `"CRSF"` |
| `int16_t getLinkQuality() const` | Qualität der Funkverbindung, % — nur CRSF, sonst −1 |
| `bool isPortReady() const` | Der UART wurde im nötigen Format geöffnet; `false` — der Port kann das nicht (SBUS ohne Invertierung), es wird keine Verbindung geben |

Die freie Funktion `rxProtocolName(Config::RxProtocol)` liefert den Namen des
Protokolls; denselben druckt der Firmware-Pass (`@id`, Feld `rx`).

Invarianten:

- Bis zum ersten gültigen Frame gilt `isSignalLost() == true` — die Standardwerte
  (alle 1500) werden nicht als Befehle der Fernsteuerung genommen.
- Das Failsafe-Kennzeichen wird bei **jedem** Kanal-Frame neu berechnet — die
  Verbindung kommt mit dem ersten Frame mit normalem Gas zurück (bei CRSF auch
  mit einer Statistik, deren Verbindungsqualität über 0 liegt).
- Kanäle mit `Config::CHANNEL_REVERSED` werden umgekehrt gespeichert
  (`RcInput::reversed`); Failsafe wird vorher am **rohen** Gas erkannt, und die
  Umkehr von CH3 verbietet ein `static_assert`.

---

## `IBusParser`

**Datei:** `rc/IBusParser.h` · **Abhängig von:** `Config`

Byteweiser Parser für FlySky iBUS: `bool feed(uint8_t)` liefert `true`, sobald ein
Frame mit gültigem CRC vollständig ist; die Kanäle stehen in `values()`. **Frame** (32 Bytes): `0x20 0x40 | CH1 lo hi … CH14 lo hi | CRC lo hi`,
`CRC = 0xFFFF − Σ(erste 30 Bytes)`; der Kanalwert sind die **unteren 12 Bit** (in
den oberen sendet der FS-iA6B Dienstdaten, z. B. im Failsafe `0x2384` → 900 µs).
Er wartet auf `0x20`; das nächste Byte muss `0x40` sein, sonst beginnt die Suche
neu (ein zweites `0x20` kann selbst einen Frame beginnen). iBUS hat kein
Failsafe-Flag: Der FS-iA6B meldet Verbindungsverlust mit Gas unter dem
Normalbereich.

---

## `SbusParser`

**Datei:** `rc/SbusParser.h` · **Abhängig von:** `Packed11`

**Frame** (25 Bytes): `0x0F | 16 Kanäle zu 11 Bit (22 Bytes) | Flags | Ende`;
Flags: Bit 2 — ein Frame über Funk verloren (noch kein Verbindungsverlust),
Bit 3 — Failsafe; Ende — `0x00` bei SBUS, `0x04`/`0x14`/`0x24`/`0x34` bei SBUS2.
Das Byte `0x0F` kommt auch in den Daten vor, deshalb wird ein Frame nur
angenommen, wenn Anfang und Ende an ihrem Platz sind; sonst geht das Parsen ab
dem nächsten `0x0F` im Empfangenen weiter — der Datenstrom synchronisiert sich
innerhalb von ein, zwei Frames.

---

## `CrsfParser`

**Datei:** `rc/CrsfParser.h` · **Abhängig von:** `Packed11`

**Frame:** `0xC8 | Länge | Typ | Daten | CRC8`; Länge = Typ + Daten + CRC, ein
Frame ist höchstens 64 Bytes lang; CRC8 mit dem Polynom `0xD5` (DVB-S2) über Typ
und Daten (`static uint8_t crc8(...)`). Zwei Typen werden gebraucht: `0x16` —
Kanäle (16 zu 11 Bit, wie bei SBUS) und `0x14` — Verbindungsstatistik (das dritte
Byte ist die Verbindungsqualität, %). Bei Verbindungsverlust hört ExpressLRS
standardmäßig einfach auf, Kanäle zu senden — das ist der Frame-Timeout; im Modus
„letzte Position“ kommen die Kanäle weiter, aber die Verbindungsqualität ist 0 —
das ist `failsafe()`.

---

## `Packed11`

**Datei:** `rc/Packed11.h` · **Art:** Namensraum

16 Kanäle zu 11 Bit in 22 Bytes, niederwertigste Bits zuerst — so packen sie SBUS
und CRSF. `toMicros(raw)`: 172…1811 → 988…2012 µs, 992 → 1500 (`5/8 · Wert + 880`,
wie in Betaflight und INAV); `unpack(data, out)` macht aus 22 Bytes 16 Werte in µs.
