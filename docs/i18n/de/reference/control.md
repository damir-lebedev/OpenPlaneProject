# CONTROL und COORDINATION — der Mischer, das Gas, ARM, die Ausgänge, der Orchestrator

> 🌐 Diese Seite ist die Übersetzung des [russischen Originals](../../../reference/control.md). Weichen Übersetzung und Original voneinander ab, gilt das Original. Die Firmware gibt ihre Konsolenmeldungen auf Russisch aus; sie werden daher unverändert zitiert. Die Übersetzung wurde von einer KI erstellt und nicht von Muttersprachlern geprüft. Fehler bitte an [Damir Lebedev](https://github.com/damir-lebedev) melden oder im [Issue-Tracker](https://github.com/damir-lebedev/OpenPlaneProject/issues) eintragen.

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

`throttle` ist der Gasbetrag (1000..2000 = 0..100 %) in beiden Richtungen; `FlightOutputs` setzt ihn zusammen mit `motorReverse` in den ESC-Impuls um (über `EscSignal`).

| Feld | Standardwert |
|---|---|
| `aileronLeft`, `aileronRight`, `elevator`, `rudder` | `PWM_CENTER` |
| `throttle` | `PWM_MIN` |
| `bool motorReverse` | `false` – der Motor läuft vorwärts |
| `bool escPulsesOff` | `false`; `true` – überhaupt keine Impulse zum ESC (Motorsignal, nur ohne ARM) |
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
| `FlightOutputState mix(const ControlCommand& c) const` | Kommando → PWM. Rollen/Nicken/Gieren werden durch den Weg (`*_MAX_US`) begrenzt; Querruder: links = `flaps + roll`, rechts = `flaps − roll` (nach unten = „+“); PWM = `1500 ± Ausschlag` mit dem Vorzeichen aus `Config::*_REVERSED`, das Vorzeichen der Klappen – `FLAPS_REVERSED`, begrenzt auf 1000..2000. `throttle` wird nicht gefüllt |
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

## `EscSignal`

**Datei:** `control/EscSignal.h` · **Art:** namespace · **Abhängig von:** `Config`

Gas (1000..2000 = 0..100 %) und Richtung → Impuls am ESC-Pin.

| Funktion | Beschreibung |
|---|---|
| `uint16_t pulseUs(uint16_t throttleUs, bool reverse)` | `ESC_BIDIRECTIONAL` (BLHeli 80A in 3D): 0 % → `ESC_NEUTRAL_US` (1500), vorwärts 1500..2000, rückwärts 1500..1000. Regler mit einer Drehrichtung: 1000..2000 unverändert, `reverse` wird ignoriert. Gas außerhalb von 1000..2000 wird begrenzt |
| `uint16_t stopUs()` | Der Impuls „Motor steht“ (1500 beim bidirektionalen, 1000 beim gewöhnlichen ESC) |

Nur Umrechnung, keine Schutzfunktionen: Wann die Richtung wechseln darf, entscheidet `MotorReverse`, die Gasgrenze rückwärts setzt `FlightController`.

---

## `MotorReverse`

**Datei:** `control/MotorReverse.h` · **Abhängig von:** `Config`

Die Motorrichtung nach dem Schalter `Feature::MOTOR_REVERSE` (standardmäßig SwD) – **nur ohne ARM**.

| Methode | Beschreibung |
|---|---|
| `void update(bool requested, bool armed, uint32_t nowMs)` | Ohne ARM – die Richtung folgt sofort dem Schalter (im Serial `MotorReverse: мотор НАЗАД` / `вперёд` – „Motor RÜCKWÄRTS“ / „vorwärts“). Mit ARM – der Schalter wirkt nicht, einmal wird „реверс НЕ переключён“ („Umkehr NICHT umgeschaltet“) ausgegeben, die neue Richtung wartet auf DISARM. Mit `ESC_BIDIRECTIONAL = false` wird die Umkehr nie eingeschaltet |
| `bool isReversed() const` | Der Motor läuft rückwärts |
| `bool isPending() const` | Der Schalter steht schon anders; die Richtung wartet auf DISARM |
| `const char* armBlockReason(uint32_t nowMs) const` | Grund, ARM abzulehnen: Seit dem letzten Richtungswechsel ist weniger als `REVERSE_SETTLE_MS` vergangen (der Motor dreht vielleicht noch); sonst `nullptr` |

---

## `ArmingManager`

**Datei:** `control/ArmingManager.h` · **Abhängig von:** `Autopilot` (nullbar), `RcChannelState`, `Config`, `Channels`

ARM über einen eigenen Schalter SwA (CH5). Der Automat steht in
[ARCHITECTURE.md §7](../ARCHITECTURE.md#arm-armingmanager).

| Methode | Beschreibung |
|---|---|
| `explicit ArmingManager(Autopilot* ap = nullptr)` | Ohne Autopilot wird nur das Gas geprüft |
| `void update(const RcChannelState& rc, bool receiverFailsafe, const char* blockReason = nullptr)` | Bei Failsafe — nichts (der Schalter in einem Failsafe-Frame spiegelt den Piloten nicht wider). Schalter OFF → DISARM, `switchSeenOff = true`. Ein Wechsel OFF→ON → Prüfungen → ARM oder Ablehnung `blockReason` ist ein externer Ablehnungsgrund (von `FlightController`: `MotorReverse::armBlockReason()`) |
| `bool isArmed() const` | Gearmt |
| `const char* getLastRefusalReason() const` | Der Grund der letzten Ablehnung oder `nullptr`; wird beim Ausschalten des Schalters zurückgesetzt |

`checkFailureReason(rc)` — die ARM-Prüfungen:

| Bedingung | Ablehnungsgrund |
|---|---|
| Gas ≥ `THROTTLE_LOW_US` | „Gas nicht auf Minimum“ |
| Jeder Modus außer MANUAL, die IMU existiert, antwortet aber nicht | „die IMU antwortet nicht…“ |
| Jeder Modus außer MANUAL, die IMU hat ein Problem bei der Prüfung vor dem Flug | der Text von `ImuSensor::getPreflightProblem()` |
| Ein Modus mit Höhe (`needsAltitude`: ALT_HOLD, CRUISE, LOITER, RTH, AUTO_LAND, SOARING), das Barometer existiert, antwortet aber nicht | „das Barometer antwortet nicht…“ |
| Alles andere bestanden, aber `blockReason` übergeben | `blockReason` (z. B. „Motorrichtung gerade gewechselt …“) |

Ein Sensor, der im Build nicht vorhanden ist (`nullptr`), blockiert ARM nicht;
in MANUAL armt das Flugzeug sogar ganz ohne Sensoren. Ein GPS-Fix gehört
absichtlich nicht zu den Prüfungen: ohne GPS verhalten sich die
Navigationsmodi sicher (ein Kreis auf der Stelle), und der Startpunkt wird
gespeichert, sobald das GPS Satelliten findet.

Invarianten: Einschalten der Platine mit dem Schalter in ON armt nicht; ein
Versuch pro Wechsel OFF→ON; Verbindungsverlust hebt ARM nicht auf.

---

## `FlightOutputs`

**Datei:** `control/FlightOutputs.h` · **Abhängig von:** `IBoard`, `FlightOutputState`, `EscSignal`, `Config`

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
| `bool begin()` | `attach(PWM_MIN, PWM_MAX)` für jeden Ausgang, Ausgabe des Status und des ESC-Typs; `true`, wenn alle **Pflicht**ausgänge einen Kanal bekommen haben |
| `bool isAttached(uint8_t ch) const` | Der Ausgang ist angeschlossen (ein Index außerhalb des Bereichs → `false`) |
| `static uint16_t valueOf(const FlightOutputState&, uint8_t ch)` | Impuls am Ausgangspin: das Feld laut Tabelle, beim ESC über `EscSignal::pulseUs(throttle, motorReverse)`, mit `escPulsesOff` – 0 |
| `void printStatus() const` | `Outputs: aileronLeft(GPIO4)=OK …` |
| `void printPulseSelfTest()` | Der gemessene Impuls gegenüber dem erwarteten an jedem herausgeführten Pin; „OK“ bei einer Abweichung ≤ 15 µs |
| `void write(const FlightOutputState&)` | Alle Ausgänge schreiben und den Zustand merken; ESC mit `escPulsesOff` – `IServoOutput::stopPulses()` |
| `void setFailsafe()` | Ruder neutral (`FAILSAFE_*`), Gas `FAILSAFE_THROTTLE` (1500 am Pin eines bidirektionalen ESC); AUX wie zuvor (die Last wird bei Verbindungsverlust nicht abgeworfen) |
| `void setBuzzer(bool on)` | der Summer der Platine (`IBoard::setBuzzer`) |
| `const FlightOutputState& getLastState() const` | Der zuletzt geschriebene Zustand |

Einen Ausgang hinzufügen: eine Tabellenzeile + ein Feld in
`FlightOutputState` + ein Index in `ServoChannel` (+ ein Pin und ein
LEDC-Kanal in `Esp32Board`).

---

## `FlightController`

**Datei:** `control/FlightController.h` · **Schicht:** COORDINATION ·
**Abhängig von:** `RcReceiver`, `ControlMixer`, `ThrottleManager`, `ArmingManager`, `FlightOutputs`, `Autopilot*`, `PilotSwitches*`, `Beeper`, `MotorReverse`, `GroundCalibration`, `SignalPlayer`

Der einzige Koordinator der Regelschleife: er parst selbst nicht den UART,
fasst das PWM nicht an und berechnet den Mischer nicht — er ruft nur die
anderen in der richtigen Reihenfolge auf. Ein ausführliches Diagramm steht in
[ARCHITECTURE.md §6](../ARCHITECTURE.md#6-der-regelzyklus-flightcontrollerupdate).

| Methode | Beschreibung |
|---|---|
| `FlightController(RcReceiver&, ControlMixer&, ThrottleManager&, ArmingManager&, FlightOutputs&, Autopilot* = nullptr, PilotSwitches* = nullptr)` | Ohne Autopilot — reine Handsteuerung; ohne Schalter — nur die Knüppel |
| `void begin()` | `outputs.setFailsafe()`, `receiver.begin()` |
| `void update()` | Ein Zyklus (siehe unten) |
| `bool isReceiverFailsafe() const` | Die Verbindung ist verloren |
| `const RcReceiver& getReceiver() const` | Für das Log (Frame-Zähler, Ursache des Verlusts) |
| `bool isArmed() const`, `const ArmingManager& getArming() const` | ARM |
| `const FlightOutputState& getOutputState() const` | Das zuletzt auf die Ausgänge Geschriebene |
| `const RcChannelState& getRcState() const` | Die Kanäle |
| `const FlightOutputs& getOutputs() const` | Die Tabelle der Ausgänge und `attached` |
| `int16_t getFlapsUs() const` | Die Klappenstellung |
| `const PilotSwitches* getSwitches() const`, `const PilotInputs& getInputs() const` | Die Schalter und Drehregler dieses Zyklus |
| `bool isLostModelBeeping() const` | Der „Ich bin hier“-Summer läuft |
| `const MotorReverse& getMotorReverse() const` | Motorrichtung und anstehende Umkehr |
| `void toggleGroundCalibration()` | Konsole `c`: Bodenkalibrierung des Flugzeugs – starten (nur ohne ARM), läuft sie – abbrechen |
| `void playSignal(uint8_t count)` | Konsole `e`: ein Signal (ohne ARM – auch per Motor) |
| `const GroundCalibration& getCalibration() const`, `const SignalPlayer& getSignals() const` | Für OLED, Log und Blackbox |

Die Reihenfolge von `update()`:

1. `receiver.update()`; `failsafe = receiver.isSignalLost()`;
2. bei lebender Verbindung — `switches->update(rc)` (der Modus, die Funktionen, die Drehregler);
3. `pilotThrottle = throttle.update(rc, failsafe)`;
4. die Knüppel `mixer.fromSticks(rc)` (bei lebender Verbindung) × `Knob::RATES`;
   die Klappen `mixer.updateFlaps(target)`: `AIRBRAKE` → −`AIRBRAKE_US`, `FLAPS` →
   `FLAPS_DEPLOYED_US`, `Knob::FLAPS` → sanft, bei Verbindungsverlust — 0;
5. `autopilot->update(armed, failsafe, pilotThrottle, sticks)` — **immer**;
   in der Umkehr wird statt `pilotThrottle` `PWM_MIN` übergeben (für den Autopiloten ist der Motor aus);
6. Kalibrierung: ein ARM-Umlegen währenddessen bricht sie ab; ohne ARM und mit Verbindung – die Geste `CAL_SWITCH`; ein Schritt `GroundCalibration::update()`; ihre Signale und das Signal der Autostart-Ablehnung gehen an `SignalPlayer`; der Summer: `Beeper::update(BEEPER, armed, failsafe, now)` oder das Signallicht;
7. Verbindung verloren → `applyLinkLoss()` und Verlassen des Takts;
8. `arming.update(rc, false, Grund)` – ARM ist gesperrt, solange eine Kalibrierung läuft oder ein Signal spielt (und in dem Takt, in dem ein ARM-Umlegen eine Kalibrierung abgebrochen hat), sonst `reverse.armBlockReason(now)`; dann `reverse.update(MOTOR_REVERSE, armed, now)` – bei DISARM wechselt die Richtung im selben Takt;
9. `command = autopilot->getCommand()` (oder die Knüppel ohne Autopilot), die Klappen – eigene; das Nicken des Signals geht auf Höhen- und Seitenruder;
10. `output = mixer.mix(command)`; `output.motorReverse = reverse.isReversed()`;
    vorwärts – `output.throttle = autopilot->applyThrottle(pilotThrottle)` mit der
    Grenze `THROTTLE_LIMIT_PCT` × Drehregler `Knob::THROTTLE_LIMIT`, rückwärts – das Gas des Piloten mit der Grenze
    `min(REVERSE_THROTTLE_LIMIT_PCT, Vorwärtsgrenze)`; der erlaubte Bereich wird über den ganzen Knüppelweg gestreckt;
11. nicht scharf oder `MOTOR_KILL` → `throttle = PWM_MIN` (zuletzt); ohne ARM setzt das Motorsignal `escPulsesOff`;
12. AUX1 — die Nutzlast (`PAYLOAD_DROP`), AUX2 — die Kamera (`Knob::CAMERA_TILT`, `CAMERA_STAB` zieht das Nicken ab); `AUX1_REVERSED`/`AUX2_REVERSED` spiegeln den Impuls.
13. `outputs.write(output)`.

`applyLinkLoss()`: Ist der Autopilot im Failsafe (scharf: RTH oder Gleiten) – Ruder und Gas nach dem Befehl des Autopiloten (die Klappen fahren sanft ein, `MOTOR_KILL` und Umkehr stoppen den Motor, AUX wie zuvor); sonst `outputs.setFailsafe()`, und solange ein Signal spielt – Neutral mit Nicken und ESC-Impulspause (der Sender wurde während der Kalibrierung weggelegt).

---

## `Beeper`

**Datei:** `control/Beeper.h` · **Abhängig von:** `Config`

| Methode | Beschreibung |
|---|---|
| `bool update(bool requested, bool armed, bool linkLost, uint32_t nowMs)` | der Zustand des Summers: 2 Hz, wenn `Feature::BEEPER` oder „Modell verloren“ (nicht gearmt, länger als `LOST_MODEL_BEEP_DELAY_MS` keine Verbindung) |
| `bool isLostModel() const` | der Modus „suchen Sie mich im Gras“ |

---

## `GroundCalibration`

**Datei:** `control/GroundCalibration.h` · **Hängt ab von:** `Config`, `RcChannelState`, `ImuSensor`, `ImuOrientation`

Bodenkalibrierung des Flugzeugs: Schritt 1 – Heck hoch, Rumpf waagerecht (der Horizont, `ImuOrientation::withLevel()`), Schritt 2 – auf den Rädern (die Standlage). Ein Schritt gilt von selbst nach `CAL_STILL_MS` Stillstand; Schritt 2 wartet, bis das Flugzeug umgestellt wird, oder `CAL_PARK_WAIT_MS`. Gespeichert wird nur als Ganzes (`ImuSensor::setGroundCalibration()`), nach der Prüfung der Standlage gegen `LANDING_GEAR`. Blockiert den Takt nicht.

`Stage`: `IDLE`, `LEVEL`, `PARK`, `DONE`, `FAILED`.

| Methode | Beschreibung |
|---|---|
| `bool watchSwitch(const RcChannelState&, uint32_t nowMs)` | die Geste: `CAL_SWITCH` `CAL_SWITCH_FLIPS`-mal runter und hoch innerhalb von `CAL_SWITCH_WINDOW_MS`, Gas unten; der erste Blick und Gas nicht unten beginnen die Zählung neu |
| `void forgetSwitch()` | ARM oder keine Verbindung – Zählung von vorn |
| `bool start(const ImuSensor*, uint32_t nowMs, const char* source)` | starten (Signal 1); keine IMU oder nicht unterstützt – Fehler (Signal 3) |
| `void cancel(const char* why)` | Abbruch ohne Signal |
| `void update(ImuSensor*, bool armed, bool signalBusy, uint32_t nowMs)` | ein Schritt: ARM bricht ab; während ein Signal spielt, wird nichts übernommen; Zeitüberschreitung `CAL_STEP_TIMEOUT_MS` – Fehler |
| `isActive()`, `getStage()`, `getStageSinceMs()`, `getFailure()`, `getParkPitch()`, `getParkRoll()` | Zustand und Ergebnis |
| `uint8_t takeSignal()` | wie viele Signale zu spielen sind: 1 – gestartet / Schritt 1, 2 – fertig, 3 – Fehler (wird einmal abgeholt) |
| `static const char* stageShort(Stage)` | `CAL1`, `CAL2`, `CALOK`, `CALER` – für das OLED |

---

## `SignalPlayer`

**Datei:** `control/SignalPlayer.h` · **Hängt ab von:** `Config`

Signale an den Piloten – eine Funktion der Zeit seit Beginn der Serie. Ein Signal: Höhen- und Seitenruder nicken (`SIGNAL_NOD_US` hoch, dann runter, je `SIGNAL_NOD_MS`), ohne ARM pausieren die ESC-Impulse `SIGNAL_ESC_PAUSE_MS` lang (danach spielt der Regler seine Bereitschaftsmelodie), das Summerlicht leuchtet während des Nickens; das nächste Signal folgt nach `SIGNAL_PERIOD_MS`.

| Methode | Beschreibung |
|---|---|
| `void play(uint8_t count, uint32_t nowMs, bool motor)` | eine Serie; `motor` – ob der ESC angefasst werden darf (und `SIGNAL_MOTOR`) |
| `void stop()` | Ruhe |
| `bool isBusy(uint32_t nowMs) const`, `uint8_t getCount(uint32_t nowMs) const` | eine Serie läuft (ARM ist solange gesperrt) |
| `bool escPulsesOff(uint32_t nowMs) const` | ESC-Impulse jetzt wegnehmen |
| `int16_t nodUs(uint32_t nowMs) const` | Rudernicken, µs (`SIGNAL_SURFACES`) |
| `bool light(uint32_t nowMs) const` | Summer / LED an `PIN_BUZZER` |
