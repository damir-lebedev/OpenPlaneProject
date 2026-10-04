# CONTROL und COORDINATION — der Mischer, das Gas, ARM, die Ausgänge, der Orchestrator

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/control.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert.

[← Referenz](README.md)

Die CONTROL-Schicht ist Logik über Daten, ohne UART, PWM und WLAN.
`FlightController` (COORDINATION) ist die einzige Klasse, die alle unteren
Schichten in einem einzigen Zyklus zusammenführt.

---

## `ControlCommand`

**Datei:** `control/ControlCommand.h` · **Art:** struct

Ein Kommando an die Ruder in **physikalischen Vorzeichen**, µs Ausschlag (±500
= voller Weg). Die gemeinsame Sprache von Knüppeln, Autopilot und Mischer.

| Feld | „+“ bedeutet |
|---|---|
| `int16_t roll` | Rollen nach rechts (rechtes Querruder hoch, linkes runter) |
| `int16_t pitch` | Nase hoch (Höhenruder hoch) |
| `int16_t yaw` | Nase nach rechts (Seitenruder und Rad nach rechts) |
| `int16_t flaps` | Klappen nach unten (beide Querruder runter); „−“ — Bremsklappe (beide hoch) |

Alle Felder sind standardmäßig 0.

---

## `FlightOutputState`

**Datei:** `control/FlightOutputState.h` · **Art:** struct

Die gewünschten Ausgangsimpulse, µs PWM. Standardmäßig — Ruder in
Neutralstellung und Gas `PWM_MIN`.

| Feld | Standardwert |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `aux1` | `PAYLOAD_CLOSED_US` — der Nutzlastabwurf ist geschlossen |
| `aux2` | `PWM_CENTER` — die Kamera |

---

## `FlapsController`

**Datei:** `control/FlapsController.h` · **Abhängig von:** `Config`

Sanftes Aus- und Einfahren der Klappen: die Stellung bewegt sich auf das Ziel
zu (ein beliebiger Wert — Klappen vom Schalter, vom Drehregler, eine Bremsklappe
nach oben), nicht schneller als der volle Weg `FLAPS_DEPLOYED_US` in
`FLAPS_TRANSITION_MS`. Die Zeit wird als Parameter übergeben.

| Methode | Beschreibung |
|---|---|
| `int16_t update(float targetUs, uint32_t nowMs)` | Ein Schritt auf das Ziel zu; liefert die aktuelle Stellung, µs (+ nach unten, − nach oben) |
| `int16_t getPosition() const` | Die aktuelle Stellung |

Invarianten:

- **Der erste Aufruf** setzt die Stellung sofort auf das Ziel — die Klappen
  „fahren“ beim Einschalten nicht auf dem Tisch aus.
- Der Zeitschritt ist auf `MAX_STEP_MS = 20` begrenzt: nach einer langen Pause
  (Failsafe, Kalibrierung) springen die Klappen nicht in einem Zyklus auf das
  Ziel.

---

## `ControlMixer`

**Datei:** `control/ControlMixer.h` · **Abhängig von:** `RcInput`, `RcChannelState`, `FlapsController`, `ControlCommand`, `FlightOutputState`, `Config`, `Channels`

Die aerodynamische Logik in zwei Schritten. Besitzt den `FlapsController`.

| Methode | Beschreibung |
|---|---|
| `ControlCommand fromSticks(const RcChannelState& rc) const` | CH1 → `roll` (2000 = nach rechts); CH2 → `pitch` **mit umgekehrtem Vorzeichen** (2000 = von sich weg = Nase runter); CH4 → `yaw` |
| `int16_t updateFlaps(float targetUs, uint32_t nowMs)` | das Ziel der Klappen wählt `FlightController` (Bremsklappe → Klappenschalter → Drehregler), hier — der sanfte Weg |
| `FlightOutputState mix(const ControlCommand& c) const` | Kommando → PWM. Rollen/Nicken/Gieren werden durch den Weg (`*_MAX_US`) begrenzt; Querruder: links = `flaps + roll`, rechts = `flaps − roll` (nach unten = „+“); PWM = `1500 ± Ausschlag` mit dem Vorzeichen aus `Config::*_REVERSED`, begrenzt auf 1000..2000. `throttle` wird nicht gefüllt |
| `int16_t getFlaps() const` | Die aktuelle Klappenstellung, µs |

Flaperons: beim Ausfahren senken sich beide Querruder um `FLAPS_DEPLOYED_US`
(das neue „Neutral“), und das Rollen wirkt darüber. Bei vollem Rollen stößt das
sich senkende Querruder früher an das Ende seines Wegs als das sich hebende —
das wirkt als Querruderdifferenzierung.

---

## `ThrottleManager`

**Datei:** `control/ThrottleManager.h` · **Abhängig von:** `RcInput`, `RcChannelState`, `Config`, `Channels`

| Methode | Beschreibung |
|---|---|
| `uint16_t update(const RcChannelState& rc, bool receiverFailsafe) const` | Das Gas von CH3, auf 1000..2000 begrenzt; bei Verbindungsverlust — `FAILSAFE_THROTTLE` |

Weiß nichts über ARM und den Autopiloten — deren Korrekturen wendet
`FlightController` an.

---

## `ArmingManager`

**Datei:** `control/ArmingManager.h` · **Abhängig von:** `Autopilot` (nullbar), `RcChannelState`, `Config`, `Channels`

ARM über einen eigenen Schalter SwA (CH5). Der Automat steht in
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Methode | Beschreibung |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Ohne Autopilot wird nur das Gas geprüft |
| `void update(const RcChannelState& rc, bool receiverFailsafe)` | Bei Failsafe — nichts (der Schalter in einem Failsafe-Frame spiegelt den Piloten nicht wider). Schalter OFF → DISARM, `switchSeenOff = true`. Ein Wechsel OFF→ON → Prüfungen → ARM oder Ablehnung |
| `bool isArmed() const` | Gearmt |
| `const char* getLastRefusalReason() const` | Der Grund der letzten Ablehnung oder `nullptr`; wird beim Ausschalten des Schalters zurückgesetzt |

`checkFailureReason(rc)` — die ARM-Prüfungen:

| Bedingung | Ablehnungsgrund |
|---|---|
| Gas ≥ `THROTTLE_LOW_US` | „Gas nicht auf Minimum“ |
| Jeder Modus außer MANUAL, die IMU existiert, antwortet aber nicht | „die IMU antwortet nicht…“ |
| Jeder Modus außer MANUAL, die IMU hat ein Problem bei der Prüfung vor dem Flug | der Text von `ImuSensor::getPreflightProblem()` |
| Ein Modus mit Höhe (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), das Barometer existiert, antwortet aber nicht | „das Barometer antwortet nicht…“ |

Ein Sensor, der im Build nicht vorhanden ist (`nullptr`), blockiert ARM nicht;
in MANUAL armt das Flugzeug sogar ganz ohne Sensoren. Ein GPS-Fix gehört
absichtlich nicht zu den Prüfungen: ohne GPS verhalten sich die
Navigationsmodi sicher (ein Kreis auf der Stelle), und der Startpunkt wird
gespeichert, sobald das GPS Satelliten findet.

Invarianten: Einschalten der Platine mit dem Schalter in ON armt nicht; ein
Versuch pro Wechsel OFF→ON; Verbindungsverlust hebt ARM nicht auf.

---

## `FlightOutputs`

**Datei:** `control/FlightOutputs.h` · **Abhängig von:** `IBoard`, `FlightOutputState`, `Config`

Die einzige Klasse, die den Satz und die Reihenfolge der PWM-Ausgänge kennt.
Alle Ausgänge sind in einer Tabelle beschrieben; `begin()`, `write()`, der
Status und der Selbsttest laufen in einer Schleife darüber.

### `FlightOutputs::OutputInfo`

| Feld | Beschreibung |
|---|---|
| `const char* key` | Der Name im JSON/Log (`aileronLeft`, …, `esc`, `rudder`, `aux1`, `aux2`) |
| `const char* label` | Der Name für Menschen |
| `int16_t pin` | Die Pinnummer; `-1` — nicht herausgeführt. `int16_t`, weil am STM32 die Nummern analoger Pins `0xC0 + N` sind |
| `bool required` | Ohne ihn fliegt das Flugzeug nicht (das Seitenruder ist optional) |
| `uint16_t FlightOutputState::* field` | Ein Zeiger auf das Feld des Zustands |

| Methode | Beschreibung |
|---|---|
| `static const OutputInfo& outputInfo(uint8_t ch)` | Eine Tabellenzeile; die Reihenfolge = `ServoChannel` |
| `explicit FlightOutputs(IBoard& board)` | |
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` jedes Ausgangs, gibt den Status aus; `true`, wenn alle **erforderlichen** einen Kanal bekommen haben |
| `bool isAttached(uint8_t ch) const` | Der Ausgang ist angeschlossen (ein Index außerhalb des Bereichs → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | Der Wert des Ausgangs aus dem Zustand, nach der Tabelle |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | Der gemessene Impuls gegenüber dem erwarteten an jedem herausgeführten Pin; „OK“ bei einer Abweichung ≤ 15 µs |
| `void write(const FlightOutputState&)` | Alle Ausgänge schreiben und den Zustand merken |
| `void setFailsafe()` | Ruder in Neutralstellung (`FAILSAFE_*`), Gas `FAILSAFE_THROTTLE`; AUX wie zuvor (die Nutzlast wird bei Verbindungsverlust nicht abgeworfen) |
| `void setBuzzer(bool on)` | der Summer der Platine (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Der zuletzt geschriebene Zustand |

Einen Ausgang hinzufügen: eine Tabellenzeile + ein Feld in
`FlightOutputState` + ein Index in `ServoChannel` (+ ein Pin und ein
LEDC-Kanal in `Esp32Board`).

---

## `FlightController`

**Datei:** `control/FlightController.h` · **Schicht:** COORDINATION ·
**Abhängig von:** `IBusReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`

Der einzige Koordinator der Regelschleife: er parst selbst nicht den UART,
fasst das PWM nicht an und berechnet den Mischer nicht — er ruft nur die
anderen in der richtigen Reihenfolge auf. Ein ausführliches Diagramm steht in
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-der-regelzyklus-flightcontrollerupdate).

| Methode | Beschreibung |
|---|---|
| `FlightController(IBusReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Ohne Autopilot — reine Handsteuerung; ohne Schalter — nur die Knüppel |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Ein Zyklus (siehe unten) |
| `bool isReceiverFailsafe() const` | Die Verbindung ist verloren |
| `const IBusReceiver& getReceiver() const` | Für das Log (Frame-Zähler, Ursache des Verlusts) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Das zuletzt auf die Ausgänge Geschriebene |
| `const RcChannelState& getRcState() const` | Die Kanäle |
| `const FlightOutputs& getOutputs() const` | Die Tabelle der Ausgänge und `attached` |
| `int16_t getFlapsUs() const` | Die Klappenstellung |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Die Schalter und Drehregler dieses Zyklus |
| `bool isLostModelBeeping() const` | Der „Ich bin hier“-Summer läuft |

Die Reihenfolge von `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. bei lebender Verbindung — `switches->update(rc)` (der Modus, die Funktionen, die Drehregler);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. die Knüppel `mixer.fromSticks(rc)` (bei lebender Verbindung) × `Knob::RATES`;
   die Klappen `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → sanft, bei Verbindungsverlust — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **immer**;
6. der Summer: `Beeper::update(BEEPER, armed, failsafe, now)`;
7. Verbindung verloren → `applyLinkLoss()` und Verlassen des Zyklus;
8. `arming.update(rc, false)`;
9. `command = autopilot->getCommand()` (oder die Knüppel ohne Autopilot), die Klappen — ihre eigenen;
10. `output = mixer.mix(command)`; `output.throttle = autopilot->applyThrottle(pilotThrottle)`;
11. nicht gearmt oder `MOTOR_KILL` → `throttle = PWM_MIN` (als Letztes);
12. AUX1 — die Nutzlast (`PAYLOAD_DROP`), AUX2 — die Kamera (`Knob::CAMERA_TILT`, `CAMERA_STAB` zieht das Nicken ab);
13. `outputs.write(output)`.

`applyLinkLoss()`: ist der Autopilot im Failsafe (gearmt: RTH oder Gleiten) —
folgen Ruder und Gas dem Kommando des Autopiloten (die Klappen fahren sanft
ein, `MOTOR_KILL` schaltet den Motor weiterhin ab, AUX wie zuvor); sonst
`outputs.setFailsafe()`.

---

## `Beeper`

**Datei:** `control/Beeper.h` · **Abhängig von:** `Config`

| Methode | Beschreibung |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | der Zustand des Summers: 2 Hz, wenn `Feature::BEEPER` oder „Modell verloren“ (nicht gearmt, länger als `LOST_MODEL_BEEP_DELAY_MS` keine Verbindung) |
| `bool isLostModel() const` | der Modus „suchen Sie mich im Gras“ |
