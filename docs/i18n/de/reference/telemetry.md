# TELEMETRY — Protokoll, Konsole, Web-Dashboard, OLED, Blackbox

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/telemetry.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

[← Referenz](README.md)

Die Telemetrie ist von der Flugsteuerung vollständig getrennt: Sie liest nur die
konstanten Getter von `FlightController`, `Autopilot`, den Sensoren und `LoopStats`.
Der einzige Weg „zurück“ sind die Befehle des Dashboards, die über den Briefkasten von
`WebDebugServer` laufen und von der Flugschleife angewendet werden.

---

## `LoopStats`

**Datei:** `telemetry/LoopStats.h` · **Art:** struct

Die Frequenz und die Dauer der Flugschleife.

| Member | Beschreibung |
|---|---|
| `volatile uint32_t hz, avgUs, maxUs` | Werden einmal pro Sekunde veröffentlicht; von anderen Tasks gelesen (32-Bit-Werte — keine „zerrissenen“ Lesevorgänge) |
| `void record(uint32_t durationUs)` | In jedem Takt aus `loop()` aufrufen |
| `uint32_t takePeakUs()` | Der schlechteste Takt seit dem vorigen Aufruf (für die SYS-Zeile alle 10 s); aus demselben Task aufrufen wie `record()` |

`maxUs` ist nur der schlechteste Wert der letzten Sekunde; ein seltener Aussetzer ist über
`takePeakUs()` sichtbar.

---

## `LogSettings`

**Datei:** `telemetry/LogSettings.h` · **Abhängig von:** `Preferences` (NVS, der Namensraum `debuglog`)

### `LogChannel` (enum class)

| Kanal | Präfix | Was er ausgibt | Standard |
|---|---|---|---|
| `Status` | `STAT` | Verbindung, ARM, Modus, Klappen, Sensoren | bei Änderung |
| `Rc` | `RC` | Kanäle des Senders | aus |
| `Outputs` | `OUT` | Ausgänge zu den Rudern und zum ESC | aus |
| `Attitude` | `ATT` | Rollen, Nicken, Kurs | aus |
| `Autopilot` | `AP` | Ziele und Korrekturen | aus |
| `Altitude` | `ALT` | Höhe, Vertikalgeschwindigkeit | aus |
| `Heading` | `MAG` | Kurs nach dem Kompass | aus |
| `Gps` | `GPS` | Satelliten, Koordinaten | aus |
| `Imu` | `IMU` | Gyroskop und Beschleunigungssensor | aus |
| `Nav` | `NAV` | Heimatpunkt, Kurs, Geschwindigkeit, Pitotrohr, aktivierte Funktionen | aus |
| `System` | `SYS` | Schleifenfrequenz, Speicher (alle 10 s), nur aus/ein | ein |
| `Count` | — | die Zahl der Kanäle | — |

`LogMode` (enum class): `Off`, `OnChange`, `Periodic`.

`LogChannelInfo`: `tag`, `title`, `periodicOnly`, `defaultMode`.

| Methode | Beschreibung |
|---|---|
| `static constexpr uint8_t COUNT`, `PERIOD_OPTIONS = 4` | |
| `static const LogChannelInfo& info(uint8_t)` | Eine Zeile der Kanaltabelle |
| `static uint16_t periodOption(uint8_t)` | 200 / 500 / 1000 / 2000 ms (reihum) |
| `LogSettings()`, `void setDefaults()` | Die Standardmodi, eine Periode von 1 s |
| `LogMode mode(uint8_t)`, `mode(LogChannel)` | Der Modus eines Kanals |
| `void setMode(uint8_t, LogMode)` | Bei `periodicOnly`-Kanälen wird `OnChange` zu `Periodic` |
| `void cycleMode(uint8_t)` | aus → bei Änderung → dauerhaft → aus (SYS: aus ↔ ein) |
| `void setAll(LogMode)` | Für alle Kanäle; SYS wird vom Befehl „alles bei Änderung“ nicht angefasst |
| `uint16_t periodMs() const`, `void cyclePeriod()` | Die Periode des Modus „dauerhaft“ |
| `static const char* modeName(LogMode, bool periodicOnly)` | „aus“ / „bei Änderung“ / „dauerhaft“ (oder „ein“) |
| `void load()` | Aus dem NVS; stimmen `VERSION` oder die Länge nicht, bleiben die Standardwerte; ein unbekannter Moduscode → der Standard des Kanals |
| `void save() const` | In das NVS (die Modi, die Periode, die Version) |

`VERSION` ändert sich zusammen mit der Kanalliste — die alten Einstellungen werden zurückgesetzt
(`VERSION = 2`: Der Kanal NAV wurde hinzugefügt). Die Kanaltasten im Menü: `1`..`9`, NAV —
`n`, SYS — `s`.

---

## `DebugLogger`

**Datei:** `telemetry/DebugLogger.h` · **Abhängig von:** `FlightController`, `Autopilot*`, `LoopStats*`, `LogSettings`, `Config`

Ausgabe des Zustands im seriellen Monitor nach Kanälen: Jeder hat seine eigene Zeile, seinen eigenen
Modus und seine eigenen Toleranzen.

| Methode | Beschreibung |
|---|---|
| `DebugLogger(FlightController&, Autopilot* = nullptr, LoopStats* = nullptr)` | |
| `void begin()` | `settings.load()` |
| `void update()` | Einmal pro `DEBUG_INTERVAL_MS` geht es die Kanäle durch (schweigt in der Pause und solange das Menü offen ist) |
| `LogSettings& getSettings()`, `void saveSettings() const` | Für das Konsolenmenü |
| `void suspend(bool)` | Das Menü ist offen — schweigen; beim Aufheben — `refresh()` |
| `void setPaused(bool)`, `bool isPaused() const` | Pause mit der Leertaste; beim Aufheben — `refresh()` |
| `void refresh()` | Der nächste Takt gibt alle aktivierten Kanäle aus |

Die Logik eines Kanals (`updateChannel`):

- `Off` — nichts ausgeben;
- `Periodic` — einmal pro `periodMs()` (SYS — einmal alle 10 s), Werte „wie sie sind“;
- `OnChange` — die Zeile wird mit **Toleranzen** zusammengesetzt (das verschachtelte `Shown` hält
  den alten Wert, bis sich der neue weiter als die Toleranz davon entfernt: RC/PWM 3 µs, Winkel
  0,5°, Kurs 1°, Korrekturen 2, Höhe 0,3 m, Beschleunigung 0,03 g, Koordinaten
  1e−5°) und wird nur ausgegeben, wenn sie sich von der zuletzt ausgegebenen unterscheidet.

Die verschachtelten Typen: `LineBuffer : Print` (eine Zeile von bis zu 200 Bytes zum Vergleichen vor
der Ausgabe), `Shown` (ein Wert mit Hysterese).

Die Zeilenformate:

```
STAT RX=OK ARM=NO MODE=STABILIZE FLAPS=UP IMU=OK BARO=OK
RC   1:1500 2:1500 3:1000 …
OUT  AIL-L 1500 AIL-R 1500 ELE 1500 RUD 1500 ESC 1000
ATT  R +1.2 P -0.4 Y 123.0
AP   STABILIZE want R +0.0 P +0.0 corr R -6 P +2 THR +0
ALT  0.3 m  Vz +0.10 m/s  Ziel 0.0 m
MAG  Kurs 123°
GPS  fix=3 sats=12 lat … lon … v 0.0 m/s hacc 1.2 m
IMU  gyro +0.1 -0.2 +0.0 °/s  acc +0.01 -0.02 +1.00 g
SYS  loop 500 Hz, avg 700 us, max 1400 us (der schlechteste in 10 s) | iBUS ok=… crc_err=… | heap … KB | uptime … s
```

`RX=` unterscheidet `LOST(keine Frames)` von `LOST(Failsafe des Senders)`; `IMU=` ist
`NONE` / `NO_RESPONSE` / `CHECK_FAILED` / `OK`.

---

## `DebugConsole`

**Datei:** `telemetry/DebugConsole.h` · **Abhängig von:** `FlightController`, `FlightOutputs`, `Autopilot`, `DebugLogger`, `LogSettings`, `IBoard*` (Bus-Scan)

Ein Textmenü im seriellen Monitor. Ein Automat der Bildschirme `Screen::{None, Main, Log}`.

| Methode | Beschreibung |
|---|---|
| `DebugConsole(FlightController&, FlightOutputs&, Autopilot&, DebugLogger&, IBoard* boardForScan = nullptr)` | mit Board — der Befehl `b` und Menüpunkt 7 |
| `static const char* guessI2cDevice(uint8_t address)` | der Chip nach der Adresse: 0x6A LSM6DSV, 0x68 MPU/ICM, 0x76 BME280/BMP388/SPL06, 0x46/0x47 BMP581, 0x7C QMC6309, 0x2C QMC5883P, 0x0D QMC5883L, 0x3C OLED |
| `void printHint() const` | Ein einzeiliger Hinweis |
| `void update()` | Alle Bytes aus `Serial` verarbeiten; wurden die Protokolleinstellungen geändert, ist das Menü geschlossen und das Flugzeug **nicht armed** — im NVS speichern |

Hotkeys (außerhalb des Menüs): `h`/`?` — das Hauptmenü; `l` — das Protokollmenü; Leertaste —
das Protokoll pausieren; `s` — Status der Sensoren; `i` — Kalibrierung des Gyroskops; `o` —
Kalibrierung des IMU-Einbaus; `m` — Kalibrierung des Kompasses; `p` — Selbsttest der Ausgänge;
`b` — Scan der I2C-Busse (0x08..0x7F — bis 0x7F, weil der QMC6309 auf 0x7C sitzt) mit
den Namen der Chips; alles andere — ein Hinweis. `\r`/`\n` werden ignoriert.

Das Protokollmenü: `1`..`9` — schalten den Modus der Kanäle 0..8 reihum um, `n` — NAV, `s` — SYS, `p` — Periode, `a` —
alles „bei Änderung“, `x` — alles aus, `d` — Standard, `0`/`q` — zurück, `l`/`h` —
schließen.

Die blockierenden Aktionen (`i`, `o`, `m`, `p`) sind **bei ARM verboten**. Solange das Menü
offen ist, ist das Protokoll angehalten (`DebugLogger::suspend`). Die Breite der Menüpunkte
wird in UTF-8-Zeichen gezählt, nicht in Bytes (Kyrillisch belegt 2 Bytes).

---

## `WebDashboardPage`

**Datei:** `telemetry/WebDashboardPage.h` · **Art:** namespace

`static const char HTML[] PROGMEM` — die ganze Seite (HTML + CSS + JS) in einem einzigen
Literal. Alles Dynamische baut der Browser aus dem JSON von `/api/status` (abgefragt
alle 200 ms): Die Zeilen der Kanäle, Ausgänge und Sensoren entstehen aus den JSON-Schlüsseln, ein neuer
Ausgang erscheint also ohne Änderung der Seite. Ein PID-Feld, das der Benutzer zu
bearbeiten begonnen hat, überschreibt die Abfrage nicht mehr.

---

## `WebDebugServer`

**Datei:** `telemetry/WebDebugServer.h` · **Abhängig von:** `WebServer`, `WiFi`, `FlightController`, `Autopilot*`, `WebDashboardPage`, `Config`

| Methode | Beschreibung |
|---|---|
| `explicit WebDebugServer(FlightController&, Autopilot* = nullptr)` | |
| `bool begin()` | Ein WLAN-AP (`persistent(false)` — nichts wird in den Flash geschrieben), die Routen, der Task `web` auf Kern 0. `false`, wenn der Access Point nicht hochkam |
| `void applyPendingCommands()` | Aus der Flugschleife aufrufen: nimmt die Befehle unter einem Spinlock und wendet sie auf den Autopiloten an |

Die Routen:

| Route | Antwort |
|---|---|
| `GET /` | Die Dashboard-Seite |
| `GET /api/status` | Das Zustands-JSON (`buildStatusJson()`), das Format steht im [DEVELOPER_GUIDE](../DEVELOPER_GUIDE.md#get-apistatus) |
| `POST /api/setmode` | `{"mode":0..3}` → 200 `{"status":"ok"}`; kein Body → 400 `no data`; kein Autopilot → 503; falscher Modus → 400 `invalid mode` |
| `POST /api/setpid` | Beliebige von `kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch`; die ausgelassenen bleiben, wie sie sind |
| alles andere | 404 |

`PendingCommands { hasMode, mode, hasPid, pid[6] }` — ein Briefkasten unter einem
`portMUX`. `extractJsonNumber(body, key, fallback)` — eine minimale Auswertung von
flachem JSON ohne ArduinoJson: `"key"`, Leerzeichen, `:`, Leerzeichen, eine Zahl in
beliebiger JSON-Schreibweise (Vorzeichen, Bruch, Exponent `1e-7`); fehlt der Schlüssel oder die Zahl —
`fallback`.

Im JSON sind die Felder `attached`/`available` **immer** vorhanden; die Sensordaten nur
bei `available: true`.

---

## `OledDisplay`

**Datei:** `telemetry/OledDisplay.h` · **Abhängig von:** U8g2, `II2CBus`, `FlightController`, `Autopilot*`, `LoopStats`

Ein SSD1306 128×64 (I2C 0x3C) am zweiten I2C-Bus; ein eigener Task `oled`
(`Rtos::startTask`: Kern 0 beim ESP32, niedrige Priorität beim STM32), alle 200 ms.

| Methode | Beschreibung |
|---|---|
| `OledDisplay(FlightController&, Autopilot*, const LoopStats&)` | |
| `bool begin(II2CBus* displayBus)` | `nullptr` oder das Display antwortet bei 0x3C nicht → `false`; sonst richtet es U8g2 ein und startet den Task |

U8g2 sendet die Bytes über einen `byteCallback` auf `II2CBus` (das Display weiß nichts von
`Wire1`). Ein C-Callback bekommt keinen Kontext, deshalb wird der Bus in einer statischen
Variablen `busSlot()` gehalten — an Bord gibt es nur ein Display.

Das Display:

```
RX ok ARM STAB FL       Verbindung (Verlust — invertierte Zeile) / ARM / Modus / Klappen
R  +1.2 P  -0.4         Rollen / Nicken, °           (IMU --)
Alt +0.3 Vz +0.1 A14    Höhe / Vertikalgeschwindigkeit / Fluggeschwindigkeit, falls ein Pitotrohr da ist (BARO --)
H123 T1000 Y1500        Kurs / Gas / Seitenruder (H---)
L1500 R1500 E1500       Querruder / Höhenruder
Loop 500Hz max1100us    Frequenz und der schlechteste Takt in der Sekunde
```

Die Kurznamen der Modi sind `AutopilotNames::modeShort()` (`MAN`, `STAB`,
`TKOFF`, `ALT`, `ACRO`, `CRZ`, `LOIT`, `RTH`, `LNCH`, `LAND`, `SOAR`, `RESQ`);
bei Verbindungsverlust in der Luft — `GLIDE` oder `FSRTH`.

---

## `BlackBox`

**Datei:** `telemetry/BlackBox.h` · **Abhängig von:** `FlightController`, `Autopilot`, `LoopStats`, `BlackBoxStorage`, `PilotSwitches*`

Aufzeichnung des Flugs im Flash (ESP32-S3) oder auf einer SD-Karte (STM32H743). Was, wann und wie ausgelesen wird — siehe [BLACKBOX.md](../BLACKBOX.md).

| Methode | Beschreibung |
|---|---|
| `bool begin(bool startTask = true)` | Liest den Träger (`BlackBoxStorage::begin()`), reserviert die Warteschlange (PSRAM beim ESP32, `malloc` beim STM32), gleicht den gelöschten Platz ab (bis zu 0,3 s), startet den Task `bbox` (`Rtos::startTask`). Gibt es keinen Platz zum Aufzeichnen (Partition, Karte, Datei) — `false`, die Blackbox ist aus |
| `void update(uint32_t workUs)` | Aus `loop()` nach jedem Takt: Ereignisse, Start/Stopp, Schnappschüsse in die Warteschlange, weckt den Schreib-Task |
| `void writerStep()` | Ein Schritt des Schreib-Tasks: ein bis zwei Seiten in den Flash oder ein Löschvorgang am Boden |
| `requestManualStart()` / `requestManualStop()` | Aufzeichnung von Hand (Konsole `k` → `r`) |
| `State getState()` / `bool isRecording()` | `Off`, `Idle`, `Recording`, `Stopping` (schreibt die Warteschlange zu Ende, bevor END aufgezeichnet wird) |
| `printStatus(Print&)` / `printFlights(Print&)` / `eraseAll()` | Für die Konsole |
| `void handleHostCommand(const char*)` | `bb list`, `bb get <n> [Baud]` — für `tools/blackbox.py` (über USB CDC hat die Geschwindigkeit keinen Einfluss) |

Die plattformabhängigen Teile: der Grund für den Neustart — `readResetCause()`; Batteriespannung und -strom —
der ADC (`analogReadMilliVolts` beim S3, ein 12-Bit-`analogRead` beim STM32); die Fehler des Trägers
(`BlackBoxStorage::writeErrors`/`eraseErrors`) landen einmal pro Sekunde im Protokoll
als Ereignis „Träger: Schreibfehler …“ und stören den Flug nicht.

## `BlackBoxStorage`

**Datei:** `telemetry/BlackBoxStorage.h` · **Abhängig von:** `IFlashRegion`

Ein Ring aus Sektoren zu 4 KB: Der Kopf und die Liste der Flüge stammen bei `begin()` aus den Sektorköpfen (der erste Durchlauf liest den Kopf jedes Sektors und merkt sich die echten, der zweite nur diese: Ein leerer Bereich wird einmal gelesen); `openFlight()`/`append()`/`flush()`/`closeFlight()` — seitenweises Schreiben (ein CRC-8 an jedem Eintrag); `eraseStep(target, protect, allowErase)` — ein Schritt des Abgleichs/Löschens vor dem Kopf: Müll — immer, Flüge — ganz und nur solange weniger als `target` frei ist; `protect` wird nie angefasst.

## `BlackBoxRing`, `BlackBoxFormat`

`BlackBoxRing` ist eine Byte-Warteschlange von Einträgen zwischen Tasks/Kernen unter `Rtos::CriticalSection`; läuft sie über, wirft sie die ältesten weg. `BlackBoxFormat` — der Sektorkopf, die Typen und Strukturen der Einträge, die Schemazeichenketten (die Größe prüft `static_assert`), CRC-8 und CRC-32.

---

## `Mavlink` (Codec)

**Datei:** `telemetry/MavlinkCodec.h` · **Art:** namespace · **Abhängig von:** nichts (portabel)

MAVLink 2 ohne die generierte Bibliothek: Packen der Felder in der MAVLink-Reihenfolge
(mit pymavlink abgeglichen), CRC-16/MCRF4XX + `CRC_EXTRA`, Abschneiden der Nullen am Ende.

| Entität | Beschreibung |
|---|---|
| `Msg::*` | Bezeichner: HEARTBEAT, SYS_STATUS, SET_MODE, PARAM_*, GPS_RAW_INT, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW, MISSION_REQUEST_LIST/COUNT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, REQUEST_DATA_STREAM, VFR_HUD, COMMAND_LONG/ACK, HOME_POSITION, STATUSTEXT |
| `int crcExtraOf(uint32_t id)` | das `CRC_EXTRA` einer Nachricht, −1 — unbekannt |
| `crcAccumulate`, `crcCalculate` | X.25 (wie `crc_accumulate()` von mavlink) |
| `Payload` | `u8/i8/u16/i16/u32/i32/u64/f32/chars(text, size)` — die Felder der Reihe nach |
| `Encoder(sysid, compid)` | `size_t encode(out, msgid, payload)` — ein v2-Frame mit fortlaufendem `seq` |
| `Message` | eine empfangene Nachricht: `msgid`, `sysid`, `compid`, die Nutzlast (mit Nullen aufgefüllt), Lesen der Felder über den Offset |
| `Parser` | `bool feed(byte)` → `message()`; v1 und v2, die Signatur von v2 wird übersprungen; `goodCount()`, `badCrcCount()`; Nachrichten mit unbekanntem `CRC_EXTRA` werden stillschweigend übersprungen |

## `MavlinkModes`

**Datei:** `telemetry/MavlinkTelemetry.h` · **Art:** namespace

| Funktion | Beschreibung |
|---|---|
| `toCustomMode(mode, failsafeReturning, failsafeGliding)` | die Modusnummer von ArduPlane: MANUAL 0, STABILIZE→FBWA 5, ALT_HOLD→FBWB 6, ACRO 4, CRUISE 7, LOITER 12, RTH→RTL 11, AUTO_TAKEOFF/LAUNCH→TAKEOFF 13, AUTO_LAND→AUTO 10, SOARING→THERMAL 24, RESCUE→STABILIZE 2; Failsafe → RTL 11 / CIRCLE 1 |
| `fromCustomMode(custom, mode&)` | umgekehrt, für Befehle vom Boden; AUTO, CIRCLE, GUIDED — `false` |
| `isAutonomous(mode)` | das Flag `AUTO_ENABLED` im HEARTBEAT |

## `MavlinkTelemetry`

**Datei:** `telemetry/MavlinkTelemetry.h` · **Abhängig von:** `IUartPort`, `FlightController`, `Autopilot*`, `LoopStats*`

Telemetrie über ein Funkmodem für QGroundControl / Mission Planner (das Fahrzeug ist
`MAV_TYPE_FIXED_WING`, `MAV_AUTOPILOT_ARDUPILOTMEGA`). Wird beim STM32 verwendet
(UART4), der kein WLAN hat.

| Methode | Beschreibung |
|---|---|
| `MavlinkTelemetry(IUartPort&, FlightController&, Autopilot*, const LoopStats* = nullptr)` | |
| `void begin(uint32_t baud = TELEM_BAUDRATE)` | den Port öffnen, die Nachricht „OpenPlane online“ |
| `void update()` | aus der Flugschleife: wertet das Eingehende aus (≤ 128 Bytes pro Takt), Ereignisnachrichten, nicht mehr als 2 Frames pro Takt |
| `void statusText(severity, text)` | in den GCS-Feed (eine Warteschlange von 4 Zeilen, bis zu 50 Zeichen) |
| `isGcsConnected()` | ein HEARTBEAT der GCS innerhalb der letzten 3 s |
| `getSentFrames()`, `getDeferredFrames()`, `getParser()` | Diagnose |
| `static const char* paramName(uint8_t)` | `RLL_KP`, `RLL_KI`, `RLL_KD`, `PTCH_KP`, `PTCH_KI`, `PTCH_KD` |

Die Datenströme (Hz): ATTITUDE 10; GLOBAL_POSITION_INT, VFR_HUD 5; GPS_RAW_INT,
RC_CHANNELS, SERVO_OUTPUT_RAW, NAV_CONTROLLER_OUTPUT 2; HEARTBEAT, SYS_STATUS 1;
HOME_POSITION 0,2. Ein Frame wird nur gesendet, wenn `availableForWrite()` Platz dafür hat
— sonst wartet er auf den nächsten Takt (die Schleife wird nie blockiert).

Eingehend: der HEARTBEAT der GCS; PARAM_REQUEST_LIST / READ / SET (der PID — direkt in den
Autopiloten, Werte 0..100, nicht gespeichert); SET_MODE und COMMAND_LONG
`DO_SET_MODE` (176) — der Modus bis zum nächsten Umlegen des Schalters; `COMPONENT_ARM_DISARM`
(400) — **DENIED**; `REQUEST_MESSAGE` (512) — ein außerplanmäßiges Senden eines Datenstroms;
MISSION_REQUEST_LIST — MISSION_COUNT 0 mit demselben `mission_type`.
Das Prüfen des Datenstroms mit einem externen Decoder — `tools/check_mavlink.py` (pymavlink).
